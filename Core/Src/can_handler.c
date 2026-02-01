/**
  ******************************************************************************
  * @file    can_handler.c
  * @brief   CAN bus communication handler implementation
  * @author  florinzgz
  * @date    2026-02-01
  ******************************************************************************
  * @attention
  *
  * Protocolo CAN @ 500 kbps con ESP32-S3
  * - ID 0x001: STM32 heartbeat (TX)
  * - ID 0x011: ESP32 heartbeat (RX)
  * - ID 0x100-0x102: Control commands (RX)
  * - ID 0x200-0x204: Status messages (TX)
  * - ID 0x300: Error messages (TX)
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can_handler.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define CAN_RX_FIFO0    0

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* CAN message buffer */
static FDCAN_TxHeaderTypeDef tx_header;
static FDCAN_RxHeaderTypeDef rx_header;
static uint8_t rx_data[8];

/* Heartbeat tracking */
volatile uint32_t last_can_heartbeat_ms = 0;
volatile uint8_t tx_heartbeat_counter = 0;

/* Received commands */
volatile uint8_t rx_throttle_pct = 0;
volatile int8_t rx_steering_angle = 0;
volatile uint8_t rx_drive_mode = 0;

/* Private function prototypes -----------------------------------------------*/
static void CAN_HandleRxMessage(CAN_RxMessage_t *msg);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief Initialize CAN bus @ 500 kbps
  * @retval HAL status
  */
HAL_StatusTypeDef CAN_Init(void)
{
  /* FDCAN1 is already initialized in main.c */
  
  /* Configure filters */
  if (CAN_ConfigureFilters() != HAL_OK) {
    return HAL_ERROR;
  }
  
  /* Configure global filter to reject all non-matching frames */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
    return HAL_ERROR;
  }
  
  /* Configure RX FIFO 0 watermark interrupt */
  if (HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_RX_FIFO0, 1) != HAL_OK) {
    return HAL_ERROR;
  }
  
  /* Activate RX FIFO 0 watermark notification */
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_WATERMARK, 0) != HAL_OK) {
    return HAL_ERROR;
  }
  
  return HAL_OK;
}

/**
  * @brief Configure CAN filters
  * @retval HAL status
  */
HAL_StatusTypeDef CAN_ConfigureFilters(void)
{
  FDCAN_FilterTypeDef filter_config;
  
  /* Configure standard ID filter to accept messages from ESP32 */
  filter_config.IdType = FDCAN_STANDARD_ID;
  filter_config.FilterIndex = 0;
  filter_config.FilterType = FDCAN_FILTER_MASK;
  filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter_config.FilterID1 = 0x000;      /* Accept all IDs */
  filter_config.FilterID2 = 0x000;      /* Mask: all bits don't care */
  
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter_config) != HAL_OK) {
    return HAL_ERROR;
  }
  
  return HAL_OK;
}

/**
  * @brief Start CAN communication
  * @retval HAL status
  */
HAL_StatusTypeDef CAN_Start(void)
{
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    return HAL_ERROR;
  }
  
  last_can_heartbeat_ms = HAL_GetTick();
  
  return HAL_OK;
}

/**
  * @brief Transmit CAN message
  * @param msg_id: CAN message ID (11-bit)
  * @param data: Pointer to data buffer
  * @param dlc: Data length (0-8 bytes)
  * @retval HAL status
  */
HAL_StatusTypeDef CAN_Transmit(uint32_t msg_id, uint8_t *data, uint8_t dlc)
{
  uint32_t dlc_code;
  
  /* Configure TX header */
  tx_header.Identifier = msg_id;
  tx_header.IdType = FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0;
  
  /* Convert DLC to FDCAN format */
  switch (dlc) {
    case 0: dlc_code = FDCAN_DLC_BYTES_0; break;
    case 1: dlc_code = FDCAN_DLC_BYTES_1; break;
    case 2: dlc_code = FDCAN_DLC_BYTES_2; break;
    case 3: dlc_code = FDCAN_DLC_BYTES_3; break;
    case 4: dlc_code = FDCAN_DLC_BYTES_4; break;
    case 5: dlc_code = FDCAN_DLC_BYTES_5; break;
    case 6: dlc_code = FDCAN_DLC_BYTES_6; break;
    case 7: dlc_code = FDCAN_DLC_BYTES_7; break;
    default: dlc_code = FDCAN_DLC_BYTES_8; break;
  }
  tx_header.DataLength = dlc_code;
  
  /* Add message to TX FIFO */
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, data) != HAL_OK) {
    return HAL_ERROR;
  }
  
  return HAL_OK;
}

/**
  * @brief Process received CAN messages
  * @retval None
  */
void CAN_ProcessMessages(void)
{
  CAN_RxMessage_t msg;
  
  /* Check if messages are available in RX FIFO 0 */
  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
    /* Get message from RX FIFO 0 */
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
      /* Copy to message structure */
      msg.id = rx_header.Identifier;
      msg.dlc = rx_header.DataLength >> 16; // Extract DLC
      memcpy(msg.data, rx_data, 8);
      
      /* Handle message */
      CAN_HandleRxMessage(&msg);
    }
  }
}

/**
  * @brief Send STM32 heartbeat message
  * @retval HAL status
  */
HAL_StatusTypeDef CAN_SendHeartbeat(void)
{
  uint8_t data[2];
  
  data[0] = tx_heartbeat_counter++;
  data[1] = (uint8_t)system_state;
  
  return CAN_Transmit(CAN_ID_HEARTBEAT_STM32, data, 2);
}

/**
  * @brief Send speed status (4 wheels)
  * @param speed_FL: Front left speed (mm/s)
  * @param speed_FR: Front right speed (mm/s)
  * @param speed_RL: Rear left speed (mm/s)
  * @param speed_RR: Rear right speed (mm/s)
  * @retval HAL status
  */
HAL_StatusTypeDef CAN_SendSpeedStatus(uint16_t speed_FL, uint16_t speed_FR, 
                                       uint16_t speed_RL, uint16_t speed_RR)
{
  uint8_t data[8];
  
  data[0] = (uint8_t)(speed_FL >> 8);
  data[1] = (uint8_t)(speed_FL & 0xFF);
  data[2] = (uint8_t)(speed_FR >> 8);
  data[3] = (uint8_t)(speed_FR & 0xFF);
  data[4] = (uint8_t)(speed_RL >> 8);
  data[5] = (uint8_t)(speed_RL & 0xFF);
  data[6] = (uint8_t)(speed_RR >> 8);
  data[7] = (uint8_t)(speed_RR & 0xFF);
  
  return CAN_Transmit(CAN_ID_STATUS_SPEED, data, 8);
}

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
                                         uint8_t current_STEER, uint8_t current_BATT)
{
  uint8_t data[7];
  
  data[0] = current_FL;
  data[1] = current_FR;
  data[2] = current_RL;
  data[3] = current_RR;
  data[4] = current_STEER;
  data[5] = current_BATT;
  data[6] = CAN_CRC8(data, 6);
  
  return CAN_Transmit(CAN_ID_STATUS_CURRENT, data, 7);
}

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
                                      int8_t temp_AMB)
{
  uint8_t data[6];
  
  data[0] = (uint8_t)temp_FL;
  data[1] = (uint8_t)temp_FR;
  data[2] = (uint8_t)temp_RL;
  data[3] = (uint8_t)temp_RR;
  data[4] = (uint8_t)temp_AMB;
  data[5] = CAN_CRC8(data, 5);
  
  return CAN_Transmit(CAN_ID_STATUS_TEMP, data, 6);
}

/**
  * @brief Send safety status (ABS/TCS)
  * @param abs_flags: ABS active flags (bit 0=FL, 1=FR, 2=RL, 3=RR)
  * @param tcs_flags: TCS active flags (bit 0=FL, 1=FR, 2=RL, 3=RR)
  * @param slip_max: Maximum slip percentage detected
  * @retval HAL status
  */
HAL_StatusTypeDef CAN_SendSafetyStatus(uint8_t abs_flags, uint8_t tcs_flags,
                                        uint8_t slip_max)
{
  uint8_t data[4];
  
  data[0] = abs_flags;
  data[1] = tcs_flags;
  data[2] = slip_max;
  data[3] = CAN_CRC8(data, 3);
  
  return CAN_Transmit(CAN_ID_STATUS_SAFETY, data, 4);
}

/**
  * @brief Send steering status
  * @param encoder_position: Encoder position (-720 to +720 counts)
  * @param steering_angle: Steering angle percentage (-100 to +100)
  * @retval HAL status
  */
HAL_StatusTypeDef CAN_SendSteeringStatus(int16_t encoder_position, int8_t steering_angle)
{
  uint8_t data[4];
  
  data[0] = (uint8_t)(encoder_position >> 8);
  data[1] = (uint8_t)(encoder_position & 0xFF);
  data[2] = (uint8_t)steering_angle;
  data[3] = CAN_CRC8(data, 3);
  
  return CAN_Transmit(CAN_ID_STATUS_STEERING, data, 4);
}

/**
  * @brief Send error diagnostic message
  * @param error_code: Error code
  * @param subsystem: Subsystem ID (0=Global, 1=Motor, 2=Sensor, 3=CAN)
  * @param error_data: Additional error data (16-bit)
  * @retval HAL status
  */
HAL_StatusTypeDef CAN_SendError(ErrorCode_t error_code, uint8_t subsystem, uint16_t error_data)
{
  uint8_t data[5];
  
  data[0] = (uint8_t)error_code;
  data[1] = subsystem;
  data[2] = (uint8_t)(error_data >> 8);
  data[3] = (uint8_t)(error_data & 0xFF);
  data[4] = CAN_CRC8(data, 4);
  
  return CAN_Transmit(CAN_ID_DIAG_ERROR, data, 5);
}

/**
  * @brief Check CAN heartbeat timeout
  * @retval 1 if timeout detected, 0 otherwise
  */
uint8_t CAN_CheckHeartbeatTimeout(void)
{
  uint32_t current_time = HAL_GetTick();
  
  if ((current_time - last_can_heartbeat_ms) > CAN_HEARTBEAT_TIMEOUT_MS) {
    return 1;
  }
  
  return 0;
}

/**
  * @brief Calculate CRC8 checksum
  * @param data: Pointer to data buffer
  * @param length: Data length in bytes
  * @retval CRC8 checksum
  */
uint8_t CAN_CRC8(uint8_t *data, uint8_t length)
{
  uint8_t crc = 0xFF;
  uint8_t i, j;
  
  for (i = 0; i < length; i++) {
    crc ^= data[i];
    for (j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc <<= 1;
      }
    }
  }
  
  return crc;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief Handle received CAN message
  * @param msg: Pointer to received message
  * @retval None
  */
static void CAN_HandleRxMessage(CAN_RxMessage_t *msg)
{
  uint8_t crc_calc, crc_recv;
  
  switch (msg->id) {
    case CAN_ID_HEARTBEAT_ESP32:
      /* ESP32 heartbeat received */
      last_can_heartbeat_ms = HAL_GetTick();
      break;
      
    case CAN_ID_CMD_THROTTLE:
      /* Throttle command: [throttle_pct, CRC] */
      if (msg->dlc >= 2) {
        crc_calc = CAN_CRC8(msg->data, 1);
        crc_recv = msg->data[1];
        if (crc_calc == crc_recv) {
          rx_throttle_pct = msg->data[0];
        }
      }
      break;
      
    case CAN_ID_CMD_STEERING:
      /* Steering command: [steering_angle (int8), CRC] */
      if (msg->dlc >= 2) {
        crc_calc = CAN_CRC8(msg->data, 1);
        crc_recv = msg->data[1];
        if (crc_calc == crc_recv) {
          rx_steering_angle = (int8_t)msg->data[0];
        }
      }
      break;
      
    case CAN_ID_CMD_MODE:
      /* Mode command: [drive_mode, CRC] */
      if (msg->dlc >= 2) {
        crc_calc = CAN_CRC8(msg->data, 1);
        crc_recv = msg->data[1];
        if (crc_calc == crc_recv) {
          rx_drive_mode = msg->data[0];
          
          /* Handle mode changes */
          if (rx_drive_mode == 1) {
            system_state = SYSTEM_STATE_ACTIVE;
          } else if (rx_drive_mode == 0) {
            system_state = SYSTEM_STATE_SAFE;
          }
        }
      }
      break;
      
    default:
      /* Unknown message ID */
      break;
  }
}

/**
  * @brief FDCAN1 RX FIFO 0 callback
  * @param hfdcan: FDCAN handle
  * @param RxFifo0ITs: RX FIFO 0 interrupt flags
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_WATERMARK) {
    /* Messages will be processed in main loop via CAN_ProcessMessages() */
  }
}
