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

/* Global variables */
extern FDCAN_HandleTypeDef hfdcan1;
CAN_Stats_t can_stats = {0};

/* Internal state */
static uint32_t last_tx_heartbeat = 0;
static uint8_t  heartbeat_counter = 0;

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
    heartbeat_counter = 0;
    
    /* Configure RX acceptance filters */
    CAN_ConfigureFilters();

    /* Enable RX FIFO0 new message notifications */
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }
    
    /* Start FDCAN peripheral */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
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
                if (msg_len >= 1) {
                    uint8_t mode_flags = rx_payload[0];
                    bool enable_4x4 = (mode_flags & 0x01) != 0;
                    bool tank_turn  = (mode_flags & 0x02) != 0;
                    /* STM32 decides: mode change only allowed at low speed */
                    if (Safety_ValidateModeChange(enable_4x4, tank_turn)) {
                        Traction_SetMode4x4(enable_4x4);
                        Traction_SetAxisRotation(tank_turn);
                    }

                    /* Byte 1 (optional): gear position (P/R/N/D).
                     * Backward compatible: if only 1 byte is sent, gear
                     * remains unchanged (defaults to FORWARD on init).
                     * Gear changes are only accepted at very low speed
                     * (same constraint as mode changes).                  */
                    if (msg_len >= 2) {
                        uint8_t gear_raw = rx_payload[1];
                        if (gear_raw <= (uint8_t)GEAR_FORWARD_D2) {
                            GearPosition_t requested = (GearPosition_t)gear_raw;
                            /* Gear change only allowed near standstill */
                            float avg_spd = (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                                             Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) / 4.0f;
                            if (avg_spd <= 1.0f) {
                                Traction_SetGear(requested);
                            }
                        }
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
                if (msg_len >= 1) {
                    uint8_t cmd = rx_payload[0];
                    if (cmd == 0xFF) {
                        /* Factory restore — re-enable all modules */
                        ServiceMode_FactoryRestore();
                    } else if (msg_len >= 2) {
                        uint8_t mod_id = rx_payload[1];
                        if (mod_id < MODULE_COUNT) {
                            if (cmd == 0) {
                                ServiceMode_DisableModule((ModuleID_t)mod_id);
                            } else if (cmd == 1) {
                                ServiceMode_EnableModule((ModuleID_t)mod_id);
                            }
                        }
                    }
                }
                break;
                
            default:
                /* Unknown message ID – filtered out by hardware,
                 * should never reach here.                        */
                break;
        }
    }
}

bool CAN_IsESP32Alive(void) {
    uint32_t time_since_heartbeat = HAL_GetTick() - can_stats.last_heartbeat_esp32;
    return (time_since_heartbeat < CAN_TIMEOUT_HEARTBEAT_MS);
}