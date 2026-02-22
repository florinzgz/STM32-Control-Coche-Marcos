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
#define CAN_ID_CMD_LED            0x120  // ESP32 → STM32 (on-demand) LED relay control
#define CAN_ID_STATUS_SPEED       0x200  // STM32 → ESP32 (100ms)
#define CAN_ID_STATUS_CURRENT     0x201  // STM32 → ESP32 (100ms)
#define CAN_ID_STATUS_TEMP        0x202  // STM32 → ESP32 (1000ms)
#define CAN_ID_STATUS_SAFETY      0x203  // STM32 → ESP32 (100ms)
#define CAN_ID_STATUS_STEERING    0x204  // STM32 → ESP32 (100ms)
#define CAN_ID_STATUS_TRACTION    0x205  // STM32 → ESP32 (100ms) per-wheel traction scale
#define CAN_ID_STATUS_TEMP_MAP    0x206  // STM32 → ESP32 (1000ms) explicit temp sensor map
#define CAN_ID_STATUS_BATTERY     0x207  // STM32 → ESP32 (100ms) battery 24V bus current + voltage
#define CAN_ID_OBSTACLE_DISTANCE  0x208  // ESP32 → STM32 (66ms) obstacle distance + zone + health
#define CAN_ID_OBSTACLE_SAFETY    0x209  // ESP32 → STM32 (100ms) obstacle safety state
#define CAN_ID_STATUS_LIGHTS      0x20A  // STM32 → ESP32 (1000ms) LED relay + light state
#define CAN_ID_DIAG_ERROR         0x300  // Both directions (on-demand)
#define CAN_ID_SERVICE_FAULTS     0x301  // STM32 → ESP32 (1000ms) fault bitmask
#define CAN_ID_SERVICE_ENABLED    0x302  // STM32 → ESP32 (1000ms) enabled bitmask
#define CAN_ID_SERVICE_DISABLED   0x303  // STM32 → ESP32 (1000ms) disabled bitmask
#define CAN_ID_SERVICE_CMD        0x110  // ESP32 → STM32 (on-demand) module control
#define CAN_ID_CMD_ACK            0x103  // STM32 → ESP32 (on-demand) command acknowledgment

/* Command ACK result codes (uint8_t) */
typedef enum {
    ACK_OK                  = 0,   /* Command accepted and applied           */
    ACK_REJECTED            = 1,   /* Command rejected (speed too high, etc) */
    ACK_INVALID             = 2,   /* Command payload invalid / malformed    */
    ACK_BLOCKED_BY_SAFETY   = 3    /* Command blocked by safety system state */
} CAN_AckResult_t;

/* Timeouts */
#define CAN_TIMEOUT_HEARTBEAT_MS  250    // Heartbeat timeout
#define CAN_TIMEOUT_OBSTACLE_MS   500    // Obstacle data timeout (fail-safe)

/* CAN bus-off recovery configuration */
#define CAN_BUSOFF_RETRY_INTERVAL_MS  500   /* Non-blocking retry interval      */
#define CAN_BUSOFF_MAX_RETRIES        10    /* Max recovery attempts before ERROR */

/* CAN Statistics */
typedef struct {
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t tx_errors;
    uint32_t rx_errors;
    uint32_t last_heartbeat_esp32;
    uint32_t busoff_count;                  /* Total bus-off events detected     */
} CAN_Stats_t;

/* Function prototypes */
void CAN_Init(void);
void CAN_SendHeartbeat(void);
void CAN_SendStatusSpeed(uint16_t fl, uint16_t fr, uint16_t rl, uint16_t rr);
void CAN_SendStatusCurrent(uint16_t fl, uint16_t fr, uint16_t rl, uint16_t rr);
void CAN_SendStatusTemp(int8_t t1, int8_t t2, int8_t t3, int8_t t4, int8_t t5);
void CAN_SendStatusSafety(bool abs, bool tcs, uint8_t error_code);
void CAN_SendStatusSteering(int16_t angle, bool calibrated);
void CAN_SendStatusTraction(void);
void CAN_SendStatusTempMap(void);
void CAN_SendStatusBattery(void);
void CAN_SendStatusLights(void);
void CAN_SendError(uint8_t error_code, uint8_t subsystem);
void CAN_SendDiagnosticEncoder(int32_t raw_count, int16_t delta);
void CAN_SendCommandAck(uint8_t cmd_id_low, CAN_AckResult_t result);
void CAN_SendServiceStatus(void);
void CAN_ProcessMessages(void);
bool CAN_IsESP32Alive(void);
void CAN_CheckBusOff(void);
bool CAN_IsBusOff(void);

/* LED relay state (PB10) — toggled via CAN 0x120 from ESP32 */
void LED_Relay_Set(bool on);
bool LED_Relay_Get(void);

extern CAN_Stats_t can_stats;
extern FDCAN_HandleTypeDef hfdcan1;

#ifdef __cplusplus
}
#endif

#endif /* __CAN_HANDLER_H */
