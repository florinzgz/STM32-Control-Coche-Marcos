/**
  ******************************************************************************
  * @file    can_handler.h
  * @brief   CAN bus communication handler
  * @author  florinzgz
  * @date    2026-02-01
  ******************************************************************************
  * @attention
  *
  * Protocolo CAN @ 500 kbps con ESP32-S3
  * Ver docs/CAN_PROTOCOL.md para detalles
  *
  ******************************************************************************
  */

#ifndef __CAN_HANDLER_H
#define __CAN_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief CAN message IDs (11-bit standard)
 */
typedef enum {
    /* Heartbeat messages */
    CAN_ID_HEARTBEAT_STM32  = 0x001,
    CAN_ID_HEARTBEAT_ESP32  = 0x011,
    
    /* Control commands (ESP32 → STM32) */
    CAN_ID_CMD_THROTTLE     = 0x100,
    CAN_ID_CMD_STEERING     = 0x101,
    CAN_ID_CMD_MODE         = 0x102,
    
    /* Status messages (STM32 → ESP32) */
    CAN_ID_STATUS_SPEED     = 0x200,
    CAN_ID_STATUS_CURRENT   = 0x201,
    CAN_ID_STATUS_TEMP      = 0x202,
    CAN_ID_STATUS_SAFETY    = 0x203,
    CAN_ID_STATUS_STEERING  = 0x204,
    
    /* Diagnostic messages */
    CAN_ID_DIAG_ERROR       = 0x300
} CAN_MessageID_t;

/**
 * @brief Error codes for diagnostic messages
 */
typedef enum {
    ERR_NONE = 0x00,
    ERR_TIMEOUT_CAN = 0x01,
    ERR_TEMP_CRITICAL = 0x02,
    ERR_CURRENT_OVERLOAD = 0x03,
    ERR_ENCODER_FAULT = 0x04,
    ERR_WHEEL_SENSOR = 0x05,
    ERR_WATCHDOG_RESET = 0x10,
    ERR_I2C_TIMEOUT = 0x20,
    ERR_ONEWIRE_TIMEOUT = 0x21,
    ERR_BATTERY_CRITICAL = 0x30,
    ERR_UNKNOWN = 0xFF
} ErrorCode_t;

/**
 * @brief CAN RX message structure
 */
typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} CAN_RxMessage_t;

/* Exported constants --------------------------------------------------------*/

#define CAN_TX_MAILBOX_FULL_TIMEOUT_MS  10

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Initialize CAN bus @ 500 kbps
 * @retval HAL status
 */
HAL_StatusTypeDef CAN_Init(void);

/**
 * @brief Configure CAN filters
 * @retval HAL status
 */
HAL_StatusTypeDef CAN_ConfigureFilters(void);

/**
 * @brief Start CAN communication
 * @retval HAL status
 */
HAL_StatusTypeDef CAN_Start(void);

/**
 * @brief Transmit CAN message
 * @param msg_id: CAN message ID (11-bit)
 * @param data: Pointer to data buffer
 * @param dlc: Data length (0-8 bytes)
 * @retval HAL status
 */
HAL_StatusTypeDef CAN_Transmit(uint32_t msg_id, uint8_t *data, uint8_t dlc);

/**
 * @brief Process received CAN messages
 * @retval None
 */
void CAN_ProcessMessages(void);

/**
 * @brief Send STM32 heartbeat message
 * @retval HAL status
 */
HAL_StatusTypeDef CAN_SendHeartbeat(void);

/**
 * @brief Send speed status (4 wheels)
 * @param speed_FL: Front left speed (mm/s)
 * @param speed_FR: Front right speed (mm/s)
 * @param speed_RL: Rear left speed (mm/s)
 * @param speed_RR: Rear right speed (mm/s)
 * @retval HAL status
 */
HAL_StatusTypeDef CAN_SendSpeedStatus(uint16_t speed_FL, uint16_t speed_FR, 
                                       uint16_t speed_RL, uint16_t speed_RR);

/**
 * @brief Send current status (6 sensors)
 * @param current_FL: Motor FL current (0.1A units)
 * @param current_FR: Motor FR current (0.1A units)
 * @param current_RL: Motor RL current (0.1A units)
 * @param current_RR: Motor RR current (0.1A units)
 * @param current_STEER: Steering motor current (0.1A units)
 * @param current_BATT: Battery current (0.1A units)
 * @retval HAL status
 */
HAL_StatusTypeDef CAN_SendCurrentStatus(uint8_t current_FL, uint8_t current_FR,
                                         uint8_t current_RL, uint8_t current_RR,
                                         uint8_t current_STEER, uint8_t current_BATT);

/**
 * @brief Send temperature status (5 sensors)
 * @param temp_FL: Motor FL temperature (°C)
 * @param temp_FR: Motor FR temperature (°C)
 * @param temp_RL: Motor RL temperature (°C)
 * @param temp_RR: Motor RR temperature (°C)
 * @param temp_AMB: Ambient temperature (°C)
 * @retval HAL status
 */
HAL_StatusTypeDef CAN_SendTempStatus(int8_t temp_FL, int8_t temp_FR,
                                      int8_t temp_RL, int8_t temp_RR,
                                      int8_t temp_AMB);

/**
 * @brief Send safety status (ABS/TCS)
 * @param abs_flags: ABS active flags (bit 0=FL, 1=FR, 2=RL, 3=RR)
 * @param tcs_flags: TCS active flags (bit 0=FL, 1=FR, 2=RL, 3=RR)
 * @param slip_max: Maximum slip percentage detected
 * @retval HAL status
 */
HAL_StatusTypeDef CAN_SendSafetyStatus(uint8_t abs_flags, uint8_t tcs_flags,
                                        uint8_t slip_max);

/**
 * @brief Send steering status
 * @param encoder_position: Encoder position (-720 to +720 counts)
 * @param steering_angle: Steering angle percentage (-100 to +100)
 * @retval HAL status
 */
HAL_StatusTypeDef CAN_SendSteeringStatus(int16_t encoder_position, int8_t steering_angle);

/**
 * @brief Send error diagnostic message
 * @param error_code: Error code
 * @param subsystem: Subsystem ID (0=Global, 1=Motor, 2=Sensor, 3=CAN)
 * @param error_data: Additional error data (16-bit)
 * @retval HAL status
 */
HAL_StatusTypeDef CAN_SendError(ErrorCode_t error_code, uint8_t subsystem, uint16_t error_data);

/**
 * @brief Check CAN heartbeat timeout
 * @retval 1 if timeout detected, 0 otherwise
 */
uint8_t CAN_CheckHeartbeatTimeout(void);

/**
 * @brief Calculate CRC8 checksum
 * @param data: Pointer to data buffer
 * @param length: Data length in bytes
 * @retval CRC8 checksum
 */
uint8_t CAN_CRC8(uint8_t *data, uint8_t length);

/* Exported variables --------------------------------------------------------*/

/* Last received heartbeat timestamp */
extern volatile uint32_t last_can_heartbeat_ms;

/* Last received commands */
extern volatile uint8_t rx_throttle_pct;
extern volatile int8_t rx_steering_angle;
extern volatile uint8_t rx_drive_mode;

/* Heartbeat counter */
extern volatile uint8_t tx_heartbeat_counter;

#ifdef __cplusplus
}
#endif

#endif /* __CAN_HANDLER_H */
