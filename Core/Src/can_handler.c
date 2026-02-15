/**
  ****************************************************************************
  * @file    can_handler.c
  * @brief   CAN communication implementation for ESP32-STM32 link
  *
  *          The STM32 is the safety authority on the CAN bus:
  *            – RX filters accept only valid ESP32 message IDs
  *            – All received commands pass through Safety_Validate*()
  *            – Heartbeat includes system state and fault flags
  ****************************************************************************
  */

#include "can_handler.h"
#include "motor_control.h"
#include "safety_system.h"
#include "sensor_manager.h"
#include "service_mode.h"
#include <math.h>

/* NaN/Inf sanitization — mirrors motor_control.c helper.
 * Returns safe_default if val is NaN or Inf.             */
static inline float sanitize_float(float val, float safe_default)
{
    if (isnan(val) || isinf(val)) {
        return safe_default;
    }
    return val;
}

/* Safe-default for speed when NaN/Inf detected — ensures gear change is rejected */
#define SANITIZE_SPEED_DEFAULT  99.0f

/* Global variables */
extern FDCAN_HandleTypeDef hfdcan1;
CAN_Stats_t can_stats = {0};

/* Internal state */
static uint32_t last_tx_heartbeat = 0;
static uint8_t  heartbeat_counter = 0;

/* Bus-off recovery state (non-blocking, timestamp-based) */
static uint8_t  busoff_active       = 0;    /* 1 = bus-off detected, recovery in progress */
static uint32_t busoff_last_attempt = 0;    /* Timestamp of last recovery attempt         */
static uint8_t  busoff_retry_count  = 0;    /* Number of recovery attempts since bus-off  */

/* Internal helper to send a CAN frame */
static HAL_StatusTypeDef TransmitFrame(uint32_t msg_id, uint8_t *payload, uint32_t len) {
    FDCAN_TxHeaderTypeDef tx_hdr = {0};
    
    /* Map byte count to FDCAN DLC code */
    uint32_t dlc_code;
    switch (len) {
        case 0:  dlc_code = FDCAN_DLC_BYTES_0; break;
        case 1:  dlc_code = FDCAN_DLC_BYTES_1; break;
        case 2:  dlc_code = FDCAN_DLC_BYTES_2; break;
        case 3:  dlc_code = FDCAN_DLC_BYTES_3; break;
        case 4:  dlc_code = FDCAN_DLC_BYTES_4; break;
        case 5:  dlc_code = FDCAN_DLC_BYTES_5; break;
        case 6:  dlc_code = FDCAN_DLC_BYTES_6; break;
        case 7:  dlc_code = FDCAN_DLC_BYTES_7; break;
        case 8:  dlc_code = FDCAN_DLC_BYTES_8; break;
        default: dlc_code = FDCAN_DLC_BYTES_8; break; /* Clamp to max */
    }
    
    tx_hdr.Identifier = msg_id;
    tx_hdr.IdType = FDCAN_STANDARD_ID;
    tx_hdr.TxFrameType = FDCAN_DATA_FRAME;
    tx_hdr.DataLength = dlc_code;
    tx_hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_hdr.BitRateSwitch = FDCAN_BRS_OFF;
    tx_hdr.FDFormat = FDCAN_CLASSIC_CAN;
    tx_hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_hdr.MessageMarker = 0;
    
    HAL_StatusTypeDef result = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_hdr, payload);
    
    if (result == HAL_OK) {
        can_stats.tx_count++;
    } else {
        can_stats.tx_errors++;
    }
    
    return result;
}

/* ================================================================== */
/*  CAN RX Filter Configuration                                       */
/* ================================================================== */

/**
 * @brief  Configure FDCAN RX filters to only accept valid ESP32 IDs.
 *
 * This enforces CAN-bus authority: the STM32 only processes messages
 * from the known ESP32 command ID range and rejects everything else.
 */
static void CAN_ConfigureFilters(void)
{
    FDCAN_FilterTypeDef filter = {0};

    /* Filter 0: Accept ESP32 heartbeat (0x011) */
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_DUAL;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = CAN_ID_HEARTBEAT_ESP32;
    filter.FilterID2    = CAN_ID_HEARTBEAT_ESP32;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

    /* Filter 1: Accept ESP32 commands (0x100–0x102) */
    filter.FilterIndex  = 1;
    filter.FilterType   = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = CAN_ID_CMD_THROTTLE;
    filter.FilterID2    = CAN_ID_CMD_MODE;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

    /* Filter 2: Accept ESP32 service commands (0x110) */
    filter.FilterIndex  = 2;
    filter.FilterType   = FDCAN_FILTER_DUAL;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = CAN_ID_SERVICE_CMD;
    filter.FilterID2    = CAN_ID_SERVICE_CMD;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

    /* Filter 3: Accept ESP32 obstacle data (0x208–0x209) */
    filter.FilterIndex  = 3;
    filter.FilterType   = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = CAN_ID_OBSTACLE_DISTANCE;
    filter.FilterID2    = CAN_ID_OBSTACLE_SAFETY;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

    /* Reject all non-matching standard IDs */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                  FDCAN_REJECT,   /* non-matching std */
                                  FDCAN_REJECT,   /* non-matching ext */
                                  FDCAN_REJECT_REMOTE,
                                  FDCAN_REJECT_REMOTE);
}

/* ================================================================== */
/*  Public API                                                         */
/* ================================================================== */

void CAN_Init(void) {
    /* Reset statistics */
    can_stats.tx_count = 0;
    can_stats.rx_count = 0;
    can_stats.tx_errors = 0;
    can_stats.rx_errors = 0;
    can_stats.last_heartbeat_esp32 = HAL_GetTick();
    can_stats.busoff_count = 0;
    heartbeat_counter = 0;

    /* Reset bus-off recovery state */
    busoff_active       = 0;
    busoff_last_attempt = 0;
    busoff_retry_count  = 0;

    /* Skip hardware activation if FDCAN peripheral init failed.
     * System continues without CAN — Safety_CheckCANTimeout() will
     * detect the missing heartbeat and keep the system in STANDBY.  */
    extern bool fdcan_init_ok;
    if (!fdcan_init_ok) return;
    
    /* Configure RX acceptance filters */
    CAN_ConfigureFilters();

    /* Enable RX FIFO0 new message notifications */
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        return;  /* Non-fatal: CAN disabled, safety timeout will engage */
    }
    
    /* Start FDCAN peripheral */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        return;  /* Non-fatal: CAN disabled, safety timeout will engage */
    }
}

void CAN_SendHeartbeat(void) {
    uint32_t current_time = HAL_GetTick();
    
    /* Send every 100ms */
    if ((current_time - last_tx_heartbeat) >= 100) {
        /* Per CAN protocol doc (0x001):
         *   Byte 0: alive_counter  (uint8, cyclic 0-255, rollover is intentional)
         *   Byte 1: system_state   (uint8, 0=Boot..5=Error; 3=Degraded is new)
         *   Byte 2: fault_flags    (bitmask)
         *   Byte 3: error_code     (Safety_Error_t, specific fault ID for HMI) */
        uint8_t payload[4];
        payload[0] = heartbeat_counter++;
        payload[1] = (uint8_t)Safety_GetState();
        payload[2] = Safety_GetFaultFlags();
        payload[3] = (uint8_t)Safety_GetError();

        TransmitFrame(CAN_ID_HEARTBEAT_STM32, payload, 4);
        last_tx_heartbeat = current_time;
    }
}

void CAN_SendStatusSpeed(uint16_t fl, uint16_t fr, uint16_t rl, uint16_t rr) {
    uint8_t speed_data[8];
    
    /* Pack wheel speeds as little-endian 16-bit values */
    speed_data[0] = (uint8_t)(fl & 0xFF);
    speed_data[1] = (uint8_t)((fl >> 8) & 0xFF);
    speed_data[2] = (uint8_t)(fr & 0xFF);
    speed_data[3] = (uint8_t)((fr >> 8) & 0xFF);
    speed_data[4] = (uint8_t)(rl & 0xFF);
    speed_data[5] = (uint8_t)((rl >> 8) & 0xFF);
    speed_data[6] = (uint8_t)(rr & 0xFF);
    speed_data[7] = (uint8_t)((rr >> 8) & 0xFF);
    
    TransmitFrame(CAN_ID_STATUS_SPEED, speed_data, 8);
}

void CAN_SendStatusCurrent(uint16_t fl, uint16_t fr, uint16_t rl, uint16_t rr) {
    uint8_t current_data[8];
    
    /* Pack currents as little-endian 16-bit values */
    current_data[0] = (uint8_t)(fl & 0xFF);
    current_data[1] = (uint8_t)((fl >> 8) & 0xFF);
    current_data[2] = (uint8_t)(fr & 0xFF);
    current_data[3] = (uint8_t)((fr >> 8) & 0xFF);
    current_data[4] = (uint8_t)(rl & 0xFF);
    current_data[5] = (uint8_t)((rl >> 8) & 0xFF);
    current_data[6] = (uint8_t)(rr & 0xFF);
    current_data[7] = (uint8_t)((rr >> 8) & 0xFF);
    
    TransmitFrame(CAN_ID_STATUS_CURRENT, current_data, 8);
}

void CAN_SendStatusTemp(int8_t t1, int8_t t2, int8_t t3, int8_t t4, int8_t t5) {
    uint8_t temp_data[5];
    
    /* Pack 5 temperature values as signed bytes */
    temp_data[0] = (uint8_t)t1;
    temp_data[1] = (uint8_t)t2;
    temp_data[2] = (uint8_t)t3;
    temp_data[3] = (uint8_t)t4;
    temp_data[4] = (uint8_t)t5;
    
    TransmitFrame(CAN_ID_STATUS_TEMP, temp_data, 5);
}

void CAN_SendStatusSafety(bool abs, bool tcs, uint8_t error_code) {
    uint8_t safety_data[3];
    
    safety_data[0] = abs ? 1 : 0;
    safety_data[1] = tcs ? 1 : 0;
    safety_data[2] = error_code;
    
    TransmitFrame(CAN_ID_STATUS_SAFETY, safety_data, 3);
}

void CAN_SendStatusSteering(int16_t angle, bool calibrated) {
    uint8_t steering_data[3];
    
    /* Pack angle as signed 16-bit little-endian + calibration flag */
    steering_data[0] = (uint8_t)(angle & 0xFF);
    steering_data[1] = (uint8_t)((angle >> 8) & 0xFF);
    steering_data[2] = calibrated ? 1 : 0;
    
    TransmitFrame(CAN_ID_STATUS_STEERING, steering_data, 3);
}

/**
 * @brief  Send per-wheel traction scale to ESP32.
 *
 * Exposes the ABS/TCS per-wheel scale factor already computed by the
 * safety system (safety_status.wheel_scale[0..3]).  Each value is
 * converted from float 0.0–1.0 to uint8 0–100 (percent).
 *
 *   Byte 0: FL traction %  (0 = fully inhibited, 100 = full power)
 *   Byte 1: FR traction %
 *   Byte 2: RL traction %
 *   Byte 3: RR traction %
 *
 * CAN ID: 0x205   DLC: 4   Rate: 100 ms (10 Hz)
 */
void CAN_SendStatusTraction(void) {
    uint8_t data[4];

    data[0] = (uint8_t)(safety_status.wheel_scale[MOTOR_FL] * 100.0f);
    data[1] = (uint8_t)(safety_status.wheel_scale[MOTOR_FR] * 100.0f);
    data[2] = (uint8_t)(safety_status.wheel_scale[MOTOR_RL] * 100.0f);
    data[3] = (uint8_t)(safety_status.wheel_scale[MOTOR_RR] * 100.0f);

    TransmitFrame(CAN_ID_STATUS_TRACTION, data, 4);
}

/**
 * @brief  Send explicit temperature sensor mapping to ESP32.
 *
 * Uses the same DS18B20 readings already acquired by Temperature_ReadAll().
 * The payload assigns an unambiguous meaning to each byte:
 *
 *   Byte 0: Motor FL temperature (°C, int8_t)
 *   Byte 1: Motor FR temperature (°C, int8_t)
 *   Byte 2: Motor RL temperature (°C, int8_t)
 *   Byte 3: Motor RR temperature (°C, int8_t)
 *   Byte 4: Ambient temperature  (°C, int8_t)
 *
 * Sensor index mapping: 0=FL, 1=FR, 2=RL, 3=RR, 4=Ambient.
 * Values are not filtered or recalculated — raw DS18B20 readings.
 * If a sensor is disabled in Service Mode the value is still reported.
 *
 * CAN ID: 0x206   DLC: 5   Rate: 1000 ms (1 Hz)
 */
void CAN_SendStatusTempMap(void) {
    uint8_t data[5];

    data[0] = (uint8_t)(int8_t)Temperature_Get(0);  /* FL  */
    data[1] = (uint8_t)(int8_t)Temperature_Get(1);  /* FR  */
    data[2] = (uint8_t)(int8_t)Temperature_Get(2);  /* RL  */
    data[3] = (uint8_t)(int8_t)Temperature_Get(3);  /* RR  */
    data[4] = (uint8_t)(int8_t)Temperature_Get(4);  /* AMB */

    TransmitFrame(CAN_ID_STATUS_TEMP_MAP, data, 5);
}

/**
 * @brief  Send battery bus current and voltage to ESP32.
 *
 * Reads the INA226 on channel INA226_CHANNEL_BATTERY (100A shunt on
 * 24V bus) and transmits current and voltage so the ESP32 HMI can
 * display battery current in the upper-right corner of the screen.
 *
 *   Byte 0-1: Battery current (0.01 A units, uint16 little-endian)
 *   Byte 2-3: Battery voltage (0.01 V units, uint16 little-endian)
 *
 * CAN ID: 0x207   DLC: 4   Rate: 100 ms (10 Hz)
 */
void CAN_SendStatusBattery(void) {
    uint8_t data[4];

    float amps = Current_GetAmps(INA226_CHANNEL_BATTERY);
    float volts = Voltage_GetBus(INA226_CHANNEL_BATTERY);
    uint16_t amps_raw = (uint16_t)(amps * 100.0f);
    uint16_t volts_raw = (uint16_t)(volts * 100.0f);

    data[0] = (uint8_t)(amps_raw & 0xFF);
    data[1] = (uint8_t)((amps_raw >> 8) & 0xFF);
    data[2] = (uint8_t)(volts_raw & 0xFF);
    data[3] = (uint8_t)((volts_raw >> 8) & 0xFF);

    TransmitFrame(CAN_ID_STATUS_BATTERY, data, 4);
}

void CAN_SendError(uint8_t error_code, uint8_t subsystem) {
    uint8_t error_data[2];
    
    error_data[0] = error_code;
    error_data[1] = subsystem;
    
    TransmitFrame(CAN_ID_DIAG_ERROR, error_data, 2);
}

/**
 * @brief  Send raw encoder diagnostic data over CAN.
 *
 * Uses CAN_ID_DIAG_ERROR (0x300) with a dedicated subsystem tag
 * (0x10) to distinguish from regular error reports.  Diagnostic
 * only — not part of any control path.
 *
 *   Byte 0:    0x10 (encoder diagnostic tag)
 *   Byte 1:    reserved (0)
 *   Byte 2-5:  raw_count (int32_t, little-endian)
 *   Byte 6-7:  delta     (int16_t, little-endian)
 */
void CAN_SendDiagnosticEncoder(int32_t raw_count, int16_t delta) {
    uint8_t data[8];
    data[0] = 0x10;  /* Encoder diagnostic subsystem tag */
    data[1] = 0;
    data[2] = (uint8_t)( raw_count        & 0xFF);
    data[3] = (uint8_t)((raw_count >>  8) & 0xFF);
    data[4] = (uint8_t)((raw_count >> 16) & 0xFF);
    data[5] = (uint8_t)((raw_count >> 24) & 0xFF);
    data[6] = (uint8_t)( delta       & 0xFF);
    data[7] = (uint8_t)((delta >> 8) & 0xFF);

    TransmitFrame(CAN_ID_DIAG_ERROR, data, 8);
}

/**
 * @brief  Send command acknowledgment to ESP32.
 *
 * Transmits a 3-byte ACK frame after the STM32 has validated
 * and accepted or rejected an ESP32 command.
 *
 *   Byte 0: cmd_id_low — low byte of the original command CAN ID
 *                         (e.g. 0x02 for CMD_MODE 0x102)
 *   Byte 1: result     — CAN_AckResult_t (0=OK, 1=REJECTED, 2=INVALID,
 *                         3=BLOCKED_BY_SAFETY)
 *   Byte 2: system_state — current SystemState_t for context
 *
 * CAN ID: 0x103   DLC: 3   Rate: on-demand (after each command)
 */
void CAN_SendCommandAck(uint8_t cmd_id_low, CAN_AckResult_t result) {
    uint8_t ack_data[3];

    ack_data[0] = cmd_id_low;
    ack_data[1] = (uint8_t)result;
    ack_data[2] = (uint8_t)Safety_GetState();

    TransmitFrame(CAN_ID_CMD_ACK, ack_data, 3);
}

/**
 * @brief  Send service mode status to ESP32.
 *
 * Transmits three frames:
 *   0x301 — fault bitmask   (4 bytes, little-endian uint32)
 *   0x302 — enabled bitmask (4 bytes, little-endian uint32)
 *   0x303 — disabled bitmask(4 bytes, little-endian uint32)
 *
 * ESP32 uses these to populate the service/diagnostic menu.
 */
void CAN_SendServiceStatus(void) {
    uint8_t data[4];
    uint32_t val;

    /* Fault bitmask */
    val = ServiceMode_GetFaultMask();
    data[0] = (uint8_t)(val & 0xFF);
    data[1] = (uint8_t)((val >> 8) & 0xFF);
    data[2] = (uint8_t)((val >> 16) & 0xFF);
    data[3] = (uint8_t)((val >> 24) & 0xFF);
    TransmitFrame(CAN_ID_SERVICE_FAULTS, data, 4);

    /* Enabled bitmask */
    val = ServiceMode_GetEnabledMask();
    data[0] = (uint8_t)(val & 0xFF);
    data[1] = (uint8_t)((val >> 8) & 0xFF);
    data[2] = (uint8_t)((val >> 16) & 0xFF);
    data[3] = (uint8_t)((val >> 24) & 0xFF);
    TransmitFrame(CAN_ID_SERVICE_ENABLED, data, 4);

    /* Disabled bitmask */
    val = ServiceMode_GetDisabledMask();
    data[0] = (uint8_t)(val & 0xFF);
    data[1] = (uint8_t)((val >> 8) & 0xFF);
    data[2] = (uint8_t)((val >> 16) & 0xFF);
    data[3] = (uint8_t)((val >> 24) & 0xFF);
    TransmitFrame(CAN_ID_SERVICE_DISABLED, data, 4);
}

/* Helper to extract byte count from FDCAN DLC */
static uint8_t ExtractDLC(uint32_t dlc_code) {
    switch (dlc_code) {
        case FDCAN_DLC_BYTES_0:  return 0;
        case FDCAN_DLC_BYTES_1:  return 1;
        case FDCAN_DLC_BYTES_2:  return 2;
        case FDCAN_DLC_BYTES_3:  return 3;
        case FDCAN_DLC_BYTES_4:  return 4;
        case FDCAN_DLC_BYTES_5:  return 5;
        case FDCAN_DLC_BYTES_6:  return 6;
        case FDCAN_DLC_BYTES_7:  return 7;
        case FDCAN_DLC_BYTES_8:  return 8;
        default: return 0;
    }
}

void CAN_ProcessMessages(void) {
    FDCAN_RxHeaderTypeDef rx_hdr;
    uint8_t rx_payload[8];
    
    /* Process all pending messages in RX FIFO0 */
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_hdr, rx_payload) != HAL_OK) {
            can_stats.rx_errors++;
            continue;
        }
        
        can_stats.rx_count++;
        uint8_t msg_len = ExtractDLC(rx_hdr.DataLength);
        
        /* Parse received messages based on ID.
         *
         * All actuator commands are validated through the safety layer
         * before being applied.  The STM32 enforces physical reality:
         * it may clamp, rate-limit, or reject any ESP32 request.       */
        switch (rx_hdr.Identifier) {
            case CAN_ID_HEARTBEAT_ESP32:
                can_stats.last_heartbeat_esp32 = HAL_GetTick();
                Safety_UpdateCANRxTime();
                break;
                
            case CAN_ID_CMD_THROTTLE:
                if (msg_len >= 1) {
                    float requested_pct = (float)rx_payload[0];
                    float validated_pct = Safety_ValidateThrottle(requested_pct);
                    Traction_SetDemand(validated_pct);
                }
                break;
                
            case CAN_ID_CMD_STEERING:
                if (msg_len >= 2) {
                    int16_t angle_raw = (int16_t)(rx_payload[0] | (rx_payload[1] << 8));
                    float requested_deg = (float)angle_raw / 10.0f;
                    float validated_deg = Safety_ValidateSteering(requested_deg);
                    Steering_SetAngle(validated_deg);
                }
                break;
                
            case CAN_ID_CMD_MODE:
                if (msg_len < 1) {
                    CAN_SendCommandAck(0x02, ACK_INVALID);
                    break;
                }
                if (!Safety_IsCommandAllowed()) {
                    CAN_SendCommandAck(0x02, ACK_BLOCKED_BY_SAFETY);
                    break;
                }
                {
                    uint8_t mode_flags = rx_payload[0];
                    bool enable_4x4 = (mode_flags & 0x01) != 0;
                    bool tank_turn  = (mode_flags & 0x02) != 0;
                    bool mode_ok = false;
                    /* STM32 decides: mode change only allowed at low speed */
                    if (Safety_ValidateModeChange(enable_4x4, tank_turn)) {
                        Traction_SetMode4x4(enable_4x4);
                        Traction_SetAxisRotation(tank_turn);
                        mode_ok = true;
                    }

                    /* Byte 1 (optional): gear position (P/R/N/D).
                     * Backward compatible: if only 1 byte is sent, gear
                     * remains unchanged (defaults to FORWARD on init).
                     * Gear changes are only accepted at very low speed
                     * (same constraint as mode changes).                  */
                    bool gear_ok = true;
                    if (msg_len >= 2) {
                        uint8_t gear_raw = rx_payload[1];
                        if (gear_raw <= (uint8_t)GEAR_FORWARD_D2) {
                            GearPosition_t requested = (GearPosition_t)gear_raw;
                            /* Gear change only allowed near standstill */
                            float avg_spd = (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                                             Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) / 4.0f;
                            avg_spd = sanitize_float(avg_spd, SANITIZE_SPEED_DEFAULT);
                            if (avg_spd <= 1.0f) {
                                Traction_SetGear(requested);
                            } else {
                                gear_ok = false;
                            }
                        } else {
                            gear_ok = false;
                        }
                    }

                    if (mode_ok && gear_ok) {
                        CAN_SendCommandAck(0x02, ACK_OK);
                    } else {
                        CAN_SendCommandAck(0x02, ACK_REJECTED);
                    }
                }
                break;

            case CAN_ID_SERVICE_CMD:
                /* Service mode commands from ESP32:
                 *   Byte 0: command (0=disable, 1=enable, 0xFF=factory restore)
                 *   Byte 1: module_id (only for disable/enable)
                 *
                 * Critical modules cannot be disabled — ServiceMode_DisableModule
                 * will reject the request.  This is a safety constraint. */
                if (msg_len < 1) {
                    CAN_SendCommandAck(0x10, ACK_INVALID);
                    break;
                }
                {
                    uint8_t cmd = rx_payload[0];
                    if (cmd == 0xFF) {
                        /* Factory restore — re-enable all modules */
                        ServiceMode_FactoryRestore();
                        CAN_SendCommandAck(0x10, ACK_OK);
                    } else if (msg_len >= 2) {
                        uint8_t mod_id = rx_payload[1];
                        if (mod_id < MODULE_COUNT) {
                            if (cmd == 0) {
                                ServiceMode_DisableModule((ModuleID_t)mod_id);
                                CAN_SendCommandAck(0x10, ACK_OK);
                            } else if (cmd == 1) {
                                ServiceMode_EnableModule((ModuleID_t)mod_id);
                                CAN_SendCommandAck(0x10, ACK_OK);
                            } else {
                                CAN_SendCommandAck(0x10, ACK_INVALID);
                            }
                        } else {
                            CAN_SendCommandAck(0x10, ACK_INVALID);
                        }
                    } else {
                        CAN_SendCommandAck(0x10, ACK_INVALID);
                    }
                }
                break;

            case CAN_ID_OBSTACLE_DISTANCE:
                /* Obstacle distance from ESP32 (0x208):
                 *   Byte 0-1: minimum distance (mm, uint16 LE)
                 *   Byte 2:   zone level (0–5, uint8)
                 *   Byte 3:   sensor health (0=unhealthy, 1=healthy)
                 *   Byte 4:   rolling counter (uint8, 0–255)
                 *
                 * Processed by Obstacle_ProcessCAN() in safety_system.c.
                 * The STM32 applies a simplified backstop limiter
                 * independently of the ESP32's 5-zone logic.            */
                if (msg_len >= 5) {
                    Obstacle_ProcessCAN(rx_payload, msg_len);
                }
                break;

            case CAN_ID_OBSTACLE_SAFETY:
                /* Obstacle safety state from ESP32 (0x209):
                 *   Informational only — STM32 computes its own
                 *   obstacle_scale from the raw distance in 0x208.
                 *   This message is accepted but not parsed (reserved
                 *   for future ESP32 HMI → STM32 coordination).         */
                break;
                
            default:
                /* Unknown message ID – filtered out by hardware,
                 * should never reach here.                        */
                break;
        }
    }
}

/* ================================================================== */
/*  Bus-Off Detection and Recovery                                     */
/* ================================================================== */

/**
 * @brief  Query whether the CAN bus is currently in bus-off state.
 * @return true if bus-off is active, false otherwise.
 */
bool CAN_IsBusOff(void)
{
    return (busoff_active != 0);
}

/**
 * @brief  Check FDCAN for bus-off condition and attempt non-blocking recovery.
 *
 * Called from the 10 ms safety loop in main.c.  Uses the FDCAN protocol
 * status register to detect bus-off.  When bus-off is detected:
 *   1. Sets SAFETY_ERROR_CAN_BUSOFF
 *   2. Transitions to SYS_STATE_SAFE
 *   3. Attempts non-blocking recovery at CAN_BUSOFF_RETRY_INTERVAL_MS intervals
 *
 * Recovery sequence: Stop → DeInit → Init → ConfigFilters → ActivateNotification → Start
 *
 * If recovery succeeds, the bus-off flag is cleared.  The system will
 * recover from SAFE via the existing Safety_CheckCANTimeout() path once
 * heartbeat messages resume.
 *
 * No blocking delays.  Watchdog continues to be fed by the main loop.
 */
void CAN_CheckBusOff(void)
{
    FDCAN_ProtocolStatusTypeDef psr;

    /* If a recovery attempt is in progress, enforce retry interval */
    if (busoff_active) {
        uint32_t now = HAL_GetTick();
        if ((now - busoff_last_attempt) < CAN_BUSOFF_RETRY_INTERVAL_MS) {
            return;  /* Not yet time for next attempt */
        }

        /* Too many retries — stop attempting, system stays in SAFE */
        if (busoff_retry_count >= CAN_BUSOFF_MAX_RETRIES) {
            return;
        }

        /* Attempt recovery: Stop → DeInit → Init → Filters → Notify → Start */
        busoff_last_attempt = now;
        busoff_retry_count++;

        HAL_FDCAN_Stop(&hfdcan1);

        if (HAL_FDCAN_DeInit(&hfdcan1) != HAL_OK) {
            return;  /* DeInit failed — retry next interval */
        }

        if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
            return;  /* Init failed — retry next interval */
        }

        CAN_ConfigureFilters();

        if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
            return;  /* Notification setup failed — retry next interval */
        }

        if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
            return;  /* Start failed — retry next interval */
        }

        /* Recovery successful — clear bus-off state.
         * The safety system will recover from SAFE via
         * Safety_CheckCANTimeout() when heartbeats resume. */
        busoff_active     = 0;
        busoff_retry_count = 0;
        Safety_ClearError(SAFETY_ERROR_CAN_BUSOFF);
        return;
    }

    /* Normal operation: poll FDCAN protocol status for bus-off */
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &psr) != HAL_OK) {
        return;  /* Cannot read status — skip this cycle */
    }

    if (psr.BusOff) {
        /* Bus-off detected — raise fault and enter SAFE */
        busoff_active       = 1;
        busoff_last_attempt = HAL_GetTick();
        busoff_retry_count  = 0;
        can_stats.busoff_count++;
        Safety_SetError(SAFETY_ERROR_CAN_BUSOFF);
        Safety_SetState(SYS_STATE_SAFE);
    }
}

bool CAN_IsESP32Alive(void) {
    uint32_t time_since_heartbeat = HAL_GetTick() - can_stats.last_heartbeat_esp32;
    return (time_since_heartbeat < CAN_TIMEOUT_HEARTBEAT_MS);
}