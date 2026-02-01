/**
  ******************************************************************************
  * @file    can_handler.h
  * @brief   CAN communication handler with ESP32 over FDCAN
  ******************************************************************************
  */

#ifndef __CAN_HANDLER_H
#define __CAN_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/* CAN message IDs */
typedef enum {
    /* Heartbeat messages */
    CAN_ID_HEARTBEAT_STM32 = 0x001,
    CAN_ID_HEARTBEAT_ESP32 = 0x011,
    
    /* Command messages (ESP32 → STM32) */
    CAN_ID_CMD_THROTTLE = 0x100,
    CAN_ID_CMD_STEERING = 0x101,
    CAN_ID_CMD_MODE = 0x102,
    
    /* Status messages (STM32 → ESP32) */
    CAN_ID_STATUS_SPEED = 0x200,
    CAN_ID_STATUS_CURRENT = 0x201,
    CAN_ID_STATUS_TEMP = 0x202,
    CAN_ID_STATUS_SAFETY = 0x203,
    CAN_ID_STATUS_STEERING = 0x204,
    
    /* Diagnostic messages (Both) */
    CAN_ID_DIAG_ERROR = 0x300
} CAN_MessageID_t;

/* Command structure for throttle */
typedef struct {
    uint8_t throttle_percent;  /* 0-100% */
} CAN_CMD_Throttle_t;

/* Command structure for steering */
typedef struct {
    int8_t steering_percent;   /* -100 to +100% */
} CAN_CMD_Steering_t;

/* Command structure for mode */
typedef struct {
    uint8_t gear;              /* 0=N, 1=F, 2=R */
    uint8_t drive_mode;        /* 0=4x2, 1=4x4 */
} CAN_CMD_Mode_t;

/* Status structure for speed */
typedef struct {
    uint16_t speed_fl;         /* RPM front left */
    uint16_t speed_fr;         /* RPM front right */
    uint16_t speed_rl;         /* RPM rear left */
    uint16_t speed_rr;         /* RPM rear right */
} CAN_STATUS_Speed_t;

/* Status structure for current */
typedef struct {
    uint16_t current_fl;       /* mA front left */
    uint16_t current_fr;       /* mA front right */
    uint16_t current_rl;       /* mA rear left */
    uint16_t current_rr;       /* mA rear right */
} CAN_STATUS_Current_t;

/* Status structure for temperature */
typedef struct {
    int16_t temp_motor_fl;     /* 0.1°C front left */
    int16_t temp_motor_fr;     /* 0.1°C front right */
    int16_t temp_motor_rl;     /* 0.1°C rear left */
    int16_t temp_motor_rr;     /* 0.1°C rear right */
} CAN_STATUS_Temp_t;

/* Status structure for safety */
typedef struct {
    uint8_t abs_active;        /* ABS status flags (4 bits, one per wheel) */
    uint8_t tcs_active;        /* TCS status flags (4 bits, one per wheel) */
    uint8_t safety_state;      /* Overall safety state */
    uint8_t error_flags;       /* Error flags */
} CAN_STATUS_Safety_t;

/* Status structure for steering */
typedef struct {
    int16_t steering_angle;    /* 0.1 degrees */
    uint16_t encoder_position; /* Raw encoder value */
} CAN_STATUS_Steering_t;

/* Diagnostic error structure */
typedef struct {
    uint16_t error_code;
    uint8_t subsystem;         /* Which subsystem reported error */
    uint8_t severity;          /* 0=info, 1=warning, 2=critical */
    uint32_t timestamp;        /* Milliseconds since boot */
} CAN_DIAG_Error_t;

/* Heartbeat structure */
typedef struct {
    uint32_t uptime_ms;        /* System uptime in milliseconds */
} CAN_Heartbeat_t;

/* Exported constants --------------------------------------------------------*/
#define CAN_BITRATE 500000      /* 500 kbps */
#define CAN_MAX_DLC 8           /* Maximum data length */

#define HEARTBEAT_INTERVAL_MS 100   /* Send heartbeat every 100ms */
#define HEARTBEAT_TIMEOUT_MS 250    /* Timeout if no heartbeat received */

/* Exported variables --------------------------------------------------------*/
extern volatile bool can_esp32_alive;
extern volatile uint32_t can_last_heartbeat_ms;

/* Exported function prototypes ----------------------------------------------*/

/* Initialization */
void CAN_Init(void);
void CAN_ConfigureFilters(void);

/* Transmission */
HAL_StatusTypeDef CAN_SendHeartbeat(void);
HAL_StatusTypeDef CAN_SendSpeedStatus(CAN_STATUS_Speed_t *speed);
HAL_StatusTypeDef CAN_SendCurrentStatus(CAN_STATUS_Current_t *current);
HAL_StatusTypeDef CAN_SendTempStatus(CAN_STATUS_Temp_t *temp);
HAL_StatusTypeDef CAN_SendSafetyStatus(CAN_STATUS_Safety_t *safety);
HAL_StatusTypeDef CAN_SendSteeringStatus(CAN_STATUS_Steering_t *steering);
HAL_StatusTypeDef CAN_SendError(CAN_DIAG_Error_t *error);

/* Reception */
void CAN_ProcessReceivedMessage(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);
void CAN_HandleThrottleCommand(uint8_t *data, uint8_t dlc);
void CAN_HandleSteeringCommand(uint8_t *data, uint8_t dlc);
void CAN_HandleModeCommand(uint8_t *data, uint8_t dlc);
void CAN_HandleHeartbeat(uint8_t *data, uint8_t dlc);

/* Periodic tasks */
void CAN_PeriodicTransmit(void);
void CAN_CheckHeartbeat(void);

/* Utility functions */
bool CAN_IsESP32Alive(void);
void CAN_LogError(uint16_t error_code, uint8_t subsystem, uint8_t severity);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_HANDLER_H */
