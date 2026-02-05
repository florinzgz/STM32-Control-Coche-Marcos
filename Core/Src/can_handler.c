/**
  ****************************************************************************
  * @file    can_handler.c
  * @brief   CAN communication implementation for ESP32-STM32 link
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

/* Internal helper to send a CAN frame */
static HAL_StatusTypeDef TransmitFrame(uint32_t msg_id, uint8_t *payload, uint32_t len) {
    FDCAN_TxHeaderTypeDef tx_hdr = {0};
    
    tx_hdr.Identifier = msg_id;
    tx_hdr.IdType = FDCAN_STANDARD_ID;
    tx_hdr.TxFrameType = FDCAN_DATA_FRAME;
    tx_hdr.DataLength = (len << 16); /* Convert to FDCAN length code */
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

void CAN_Init(void) {
    /* Reset statistics */
    can_stats.tx_count = 0;
    can_stats.rx_count = 0;
    can_stats.tx_errors = 0;
    can_stats.rx_errors = 0;
    can_stats.last_heartbeat_esp32 = HAL_GetTick();
    
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
        uint8_t heartbeat_payload[1] = {0x01};
        TransmitFrame(CAN_ID_HEARTBEAT_STM32, heartbeat_payload, 1);
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
        
        /* Parse received messages based on ID */
        switch (rx_hdr.Identifier) {
            case CAN_ID_HEARTBEAT_ESP32:
                can_stats.last_heartbeat_esp32 = HAL_GetTick();
                break;
                
            case CAN_ID_CMD_THROTTLE:
                if ((rx_hdr.DataLength >> 16) >= 1) {
                    uint8_t throttle_percent = rx_payload[0];
                    Traction_SetThrottle(throttle_percent);
                }
                break;
                
            case CAN_ID_CMD_STEERING:
                if ((rx_hdr.DataLength >> 16) >= 2) {
                    int16_t target_angle = (int16_t)(rx_payload[0] | (rx_payload[1] << 8));
                    Steering_SetTarget(target_angle);
                }
                break;
                
            case CAN_ID_CMD_MODE:
                if ((rx_hdr.DataLength >> 16) >= 1) {
                    uint8_t drive_mode = rx_payload[0];
                    Traction_SetMode(drive_mode);
                }
                break;
                
            default:
                /* Unknown message ID - ignore */
                break;
        }
    }
}

bool CAN_IsESP32Alive(void) {
    uint32_t time_since_heartbeat = HAL_GetTick() - can_stats.last_heartbeat_esp32;
    return (time_since_heartbeat < CAN_TIMEOUT_HEARTBEAT_MS);
}