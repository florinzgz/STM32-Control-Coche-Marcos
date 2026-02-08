/**
  ****************************************************************************
  * @file    can_handler.h
  * @brief   CAN bus handler for ESP32-S3 HMI communication
  *          Protocol: 500 kbps, CAN 2.0A (11-bit IDs)
  ****************************************************************************
  */

#ifndef __CAN_HANDLER_H
#define __CAN_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* CAN Message IDs (ESP32 ↔ STM32) */
#define CAN_ID_HEARTBEAT_STM32    0x001  // STM32 → ESP32 (100ms)
#define CAN_ID_HEARTBEAT_ESP32    0x011  // ESP32 → STM32 (100ms)
#define CAN_ID_CMD_THROTTLE       0x100  // ESP32 → STM32 (50ms)
#define CAN_ID_CMD_STEERING       0x101  // ESP32 → STM32 (50ms)
#define CAN_ID_CMD_MODE           0x102  // ESP32 → STM32 (on-demand)
#define CAN_ID_STATUS_SPEED       0x200  // STM32 → ESP32 (100ms)
#define CAN_ID_STATUS_CURRENT     0x201  // STM32 → ESP32 (100ms)
#define CAN_ID_STATUS_TEMP        0x202  // STM32 → ESP32 (1000ms)
#define CAN_ID_STATUS_SAFETY      0x203  // STM32 → ESP32 (100ms)
#define CAN_ID_STATUS_STEERING    0x204  // STM32 → ESP32 (100ms)
#define CAN_ID_DIAG_ERROR         0x300  // Both directions (on-demand)
#define CAN_ID_SERVICE_FAULTS     0x301  // STM32 → ESP32 (1000ms) fault bitmask
#define CAN_ID_SERVICE_ENABLED    0x302  // STM32 → ESP32 (1000ms) enabled bitmask
#define CAN_ID_SERVICE_DISABLED   0x303  // STM32 → ESP32 (1000ms) disabled bitmask
#define CAN_ID_SERVICE_CMD        0x110  // ESP32 → STM32 (on-demand) module control

/* Timeouts */
#define CAN_TIMEOUT_HEARTBEAT_MS  250    // Heartbeat timeout

/* CAN Statistics */
typedef struct {
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t tx_errors;
    uint32_t rx_errors;
    uint32_t last_heartbeat_esp32;
} CAN_Stats_t;

/* Function prototypes */
void CAN_Init(void);
void CAN_SendHeartbeat(void);
void CAN_SendStatusSpeed(uint16_t fl, uint16_t fr, uint16_t rl, uint16_t rr);
void CAN_SendStatusCurrent(uint16_t fl, uint16_t fr, uint16_t rl, uint16_t rr);
void CAN_SendStatusTemp(int8_t t1, int8_t t2, int8_t t3, int8_t t4, int8_t t5);
void CAN_SendStatusSafety(bool abs, bool tcs, uint8_t error_code);
void CAN_SendStatusSteering(int16_t angle, bool calibrated);
void CAN_SendError(uint8_t error_code, uint8_t subsystem);
void CAN_SendServiceStatus(void);
void CAN_ProcessMessages(void);
bool CAN_IsESP32Alive(void);

extern CAN_Stats_t can_stats;
extern FDCAN_HandleTypeDef hfdcan1;

#ifdef __cplusplus
}
#endif

#endif /* __CAN_HANDLER_H */
