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

void CAN_SendError(uint8_t error_code, uint8_t subsystem) {
    uint8_t error_data[2];
    
    error_data[0] = error_code;
    error_data[1] = subsystem;
    
    TransmitFrame(CAN_ID_DIAG_ERROR, error_data, 2);
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