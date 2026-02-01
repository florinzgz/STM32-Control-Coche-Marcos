/**
  ******************************************************************************
  * @file    can_handler.c
  * @brief   CAN communication implementation for ESP32 interface
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can_handler.h"
#include "motor_control.h"
#include "sensor_manager.h"
#include "safety_system.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
volatile bool can_esp32_alive = false;
volatile uint32_t can_last_heartbeat_ms = 0;

static FDCAN_TxHeaderTypeDef tx_header;
static FDCAN_RxHeaderTypeDef rx_header;
static uint8_t tx_data[CAN_MAX_DLC];
static uint8_t rx_data[CAN_MAX_DLC];

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef CAN_Transmit(uint32_t message_id, uint8_t *data, uint8_t dlc);

/**
  * @brief  Initialize CAN communication
  * @retval None
  */
void CAN_Init(void)
{
  /* Configure filters */
  CAN_ConfigureFilters();

  /* Start FDCAN */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Activate RX interrupt */
  if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                      FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  can_esp32_alive = false;
  can_last_heartbeat_ms = 0;
}

/**
  * @brief  Configure CAN filters
  * @retval None
  */
void CAN_ConfigureFilters(void)
{
  FDCAN_FilterTypeDef filter_config;

  /* Configure filter to accept all standard IDs */
  filter_config.IdType = FDCAN_STANDARD_ID;
  filter_config.FilterIndex = 0;
  filter_config.FilterType = FDCAN_FILTER_RANGE;
  filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter_config.FilterID1 = 0x000;
  filter_config.FilterID2 = 0x7FF;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter_config) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure global filter to reject remote frames */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                    FDCAN_ACCEPT_IN_RX_FIFO0,
                                    FDCAN_REJECT,
                                    FDCAN_FILTER_REMOTE,
                                    FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Send heartbeat message
  * @retval HAL status
  */
HAL_StatusTypeDef CAN_SendHeartbeat(void)
{
  CAN_Heartbeat_t heartbeat;
  heartbeat.uptime_ms = HAL_GetTick();

  memcpy(tx_data, &heartbeat, sizeof(CAN_Heartbeat_t));
  return CAN_Transmit(CAN_ID_HEARTBEAT_STM32, tx_data, sizeof(CAN_Heartbeat_t));
}

/**
  * @brief  Send speed status message
  * @param  speed: Pointer to speed structure
  * @retval HAL status
  */
HAL_StatusTypeDef CAN_SendSpeedStatus(CAN_STATUS_Speed_t *speed)
{
  memcpy(tx_data, speed, sizeof(CAN_STATUS_Speed_t));
  return CAN_Transmit(CAN_ID_STATUS_SPEED, tx_data, sizeof(CAN_STATUS_Speed_t));
}

/**
  * @brief  Send current status message
  * @param  current: Pointer to current structure
  * @retval HAL status
  */
HAL_StatusTypeDef CAN_SendCurrentStatus(CAN_STATUS_Current_t *current)
{
  memcpy(tx_data, current, sizeof(CAN_STATUS_Current_t));
  return CAN_Transmit(CAN_ID_STATUS_CURRENT, tx_data, sizeof(CAN_STATUS_Current_t));
}

/**
  * @brief  Send temperature status message
  * @param  temp: Pointer to temperature structure
  * @retval HAL status
  */
HAL_StatusTypeDef CAN_SendTempStatus(CAN_STATUS_Temp_t *temp)
{
  memcpy(tx_data, temp, sizeof(CAN_STATUS_Temp_t));
  return CAN_Transmit(CAN_ID_STATUS_TEMP, tx_data, sizeof(CAN_STATUS_Temp_t));
}

/**
  * @brief  Send safety status message
  * @param  safety: Pointer to safety structure
  * @retval HAL status
  */
HAL_StatusTypeDef CAN_SendSafetyStatus(CAN_STATUS_Safety_t *safety)
{
  memcpy(tx_data, safety, sizeof(CAN_STATUS_Safety_t));
  return CAN_Transmit(CAN_ID_STATUS_SAFETY, tx_data, sizeof(CAN_STATUS_Safety_t));
}

/**
  * @brief  Send steering status message
  * @param  steering: Pointer to steering structure
  * @retval HAL status
  */
HAL_StatusTypeDef CAN_SendSteeringStatus(CAN_STATUS_Steering_t *steering)
{
  memcpy(tx_data, steering, sizeof(CAN_STATUS_Steering_t));
  return CAN_Transmit(CAN_ID_STATUS_STEERING, tx_data, sizeof(CAN_STATUS_Steering_t));
}

/**
  * @brief  Send error diagnostic message
  * @param  error: Pointer to error structure
  * @retval HAL status
  */
HAL_StatusTypeDef CAN_SendError(CAN_DIAG_Error_t *error)
{
  memcpy(tx_data, error, sizeof(CAN_DIAG_Error_t));
  return CAN_Transmit(CAN_ID_DIAG_ERROR, tx_data, sizeof(CAN_DIAG_Error_t));
}

/**
  * @brief  Process received CAN message
  * @param  rx_header: Pointer to RX header
  * @param  rx_data: Pointer to RX data
  * @retval None
  */
void CAN_ProcessReceivedMessage(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
  uint32_t msg_id = rx_header->Identifier;
  uint8_t dlc = rx_header->DataLength >> 16;

  switch (msg_id)
  {
    case CAN_ID_HEARTBEAT_ESP32:
      CAN_HandleHeartbeat(rx_data, dlc);
      break;

    case CAN_ID_CMD_THROTTLE:
      CAN_HandleThrottleCommand(rx_data, dlc);
      break;

    case CAN_ID_CMD_STEERING:
      CAN_HandleSteeringCommand(rx_data, dlc);
      break;

    case CAN_ID_CMD_MODE:
      CAN_HandleModeCommand(rx_data, dlc);
      break;

    default:
      /* Unknown message ID */
      break;
  }
}

/**
  * @brief  Handle throttle command
  * @param  data: Pointer to data
  * @param  dlc: Data length
  * @retval None
  */
void CAN_HandleThrottleCommand(uint8_t *data, uint8_t dlc)
{
  if (dlc < sizeof(CAN_CMD_Throttle_t))
    return;

  CAN_CMD_Throttle_t *cmd = (CAN_CMD_Throttle_t *)data;

  /* Validate throttle range */
  if (cmd->throttle_percent <= 100)
  {
    Traction_SetThrottle(cmd->throttle_percent);
  }
}

/**
  * @brief  Handle steering command
  * @param  data: Pointer to data
  * @param  dlc: Data length
  * @retval None
  */
void CAN_HandleSteeringCommand(uint8_t *data, uint8_t dlc)
{
  if (dlc < sizeof(CAN_CMD_Steering_t))
    return;

  CAN_CMD_Steering_t *cmd = (CAN_CMD_Steering_t *)data;

  /* Validate steering range */
  if (cmd->steering_percent >= -100 && cmd->steering_percent <= 100)
  {
    float angle = (cmd->steering_percent / 100.0f) * MAX_STEERING_ANGLE;
    Steering_SetAngle(angle);
  }
}

/**
  * @brief  Handle mode command
  * @param  data: Pointer to data
  * @param  dlc: Data length
  * @retval None
  */
void CAN_HandleModeCommand(uint8_t *data, uint8_t dlc)
{
  if (dlc < sizeof(CAN_CMD_Mode_t))
    return;

  CAN_CMD_Mode_t *cmd = (CAN_CMD_Mode_t *)data;

  /* Validate and set gear */
  if (cmd->gear <= 2)
  {
    Traction_SetGear((Gear_t)cmd->gear);
  }

  /* Validate and set drive mode */
  if (cmd->drive_mode <= 1)
  {
    drive_mode = (DriveMode_t)cmd->drive_mode;
  }
}

/**
  * @brief  Handle heartbeat message
  * @param  data: Pointer to data
  * @param  dlc: Data length
  * @retval None
  */
void CAN_HandleHeartbeat(uint8_t *data, uint8_t dlc)
{
  if (dlc < sizeof(CAN_Heartbeat_t))
    return;

  can_esp32_alive = true;
  can_last_heartbeat_ms = HAL_GetTick();
}

/**
  * @brief  Periodic CAN transmission
  * @retval None
  */
void CAN_PeriodicTransmit(void)
{
  static uint32_t last_heartbeat_ms = 0;
  static uint32_t last_speed_ms = 0;
  static uint32_t last_current_ms = 0;
  static uint32_t last_temp_ms = 0;
  static uint32_t last_safety_ms = 0;
  static uint32_t last_steering_ms = 0;

  uint32_t now_ms = HAL_GetTick();

  /* Send heartbeat every 100ms */
  if (now_ms - last_heartbeat_ms >= HEARTBEAT_INTERVAL_MS)
  {
    last_heartbeat_ms = now_ms;
    CAN_SendHeartbeat();
  }

  /* Send speed status every 100ms */
  if (now_ms - last_speed_ms >= 100)
  {
    last_speed_ms = now_ms;
    CAN_STATUS_Speed_t speed;
    speed.speed_fl = (uint16_t)motors[MOTOR_FL].current_rpm;
    speed.speed_fr = (uint16_t)motors[MOTOR_FR].current_rpm;
    speed.speed_rl = (uint16_t)motors[MOTOR_RL].current_rpm;
    speed.speed_rr = (uint16_t)motors[MOTOR_RR].current_rpm;
    CAN_SendSpeedStatus(&speed);
  }

  /* Send current status every 100ms */
  if (now_ms - last_current_ms >= 100)
  {
    last_current_ms = now_ms;
    CAN_STATUS_Current_t current;
    current.current_fl = (uint16_t)(motors[MOTOR_FL].current_amps * 1000);
    current.current_fr = (uint16_t)(motors[MOTOR_FR].current_amps * 1000);
    current.current_rl = (uint16_t)(motors[MOTOR_RL].current_amps * 1000);
    current.current_rr = (uint16_t)(motors[MOTOR_RR].current_amps * 1000);
    CAN_SendCurrentStatus(&current);
  }

  /* Send temperature status every 1000ms */
  if (now_ms - last_temp_ms >= 1000)
  {
    last_temp_ms = now_ms;
    CAN_STATUS_Temp_t temp;
    temp.temp_motor_fl = (int16_t)(motors[MOTOR_FL].temperature_c * 10);
    temp.temp_motor_fr = (int16_t)(motors[MOTOR_FR].temperature_c * 10);
    temp.temp_motor_rl = (int16_t)(motors[MOTOR_RL].temperature_c * 10);
    temp.temp_motor_rr = (int16_t)(motors[MOTOR_RR].temperature_c * 10);
    CAN_SendTempStatus(&temp);
  }

  /* Send safety status every 100ms */
  if (now_ms - last_safety_ms >= 100)
  {
    last_safety_ms = now_ms;
    CAN_STATUS_Safety_t safety_status;
    safety_status.abs_active = 0;
    safety_status.tcs_active = 0;
    for (int i = 0; i < 4; i++)
    {
      if (safety_system.abs.active[i])
        safety_status.abs_active |= (1 << i);
      if (safety_system.tcs.active[i])
        safety_status.tcs_active |= (1 << i);
    }
    safety_status.safety_state = (uint8_t)safety_system.level;
    safety_status.error_flags = (uint8_t)(safety_system.flags.overcurrent_fl |
                                          (safety_system.flags.overtemp_fl << 1) |
                                          (safety_system.flags.can_timeout << 2));
    CAN_SendSafetyStatus(&safety_status);
  }

  /* Send steering status every 100ms */
  if (now_ms - last_steering_ms >= 100)
  {
    last_steering_ms = now_ms;
    CAN_STATUS_Steering_t steering;
    steering.steering_angle = (int16_t)(ackermann.steering_angle * 10);
    steering.encoder_position = (uint16_t)ackermann.encoder_position;
    CAN_SendSteeringStatus(&steering);
  }
}

/**
  * @brief  Check heartbeat timeout
  * @retval None
  */
void CAN_CheckHeartbeat(void)
{
  uint32_t now_ms = HAL_GetTick();

  if (can_esp32_alive)
  {
    if (now_ms - can_last_heartbeat_ms > HEARTBEAT_TIMEOUT_MS)
    {
      can_esp32_alive = false;
      safety_system.flags.can_timeout = true;
    }
  }
}

/**
  * @brief  Check if ESP32 is alive
  * @retval true if alive, false otherwise
  */
bool CAN_IsESP32Alive(void)
{
  return can_esp32_alive;
}

/**
  * @brief  Log error to CAN bus
  * @param  error_code: Error code
  * @param  subsystem: Subsystem identifier
  * @param  severity: Error severity
  * @retval None
  */
void CAN_LogError(uint16_t error_code, uint8_t subsystem, uint8_t severity)
{
  CAN_DIAG_Error_t error;
  error.error_code = error_code;
  error.subsystem = subsystem;
  error.severity = severity;
  error.timestamp = HAL_GetTick();

  CAN_SendError(&error);
}

/**
  * @brief  Low-level CAN transmit function
  * @param  message_id: CAN message ID
  * @param  data: Pointer to data
  * @param  dlc: Data length
  * @retval HAL status
  */
static HAL_StatusTypeDef CAN_Transmit(uint32_t message_id, uint8_t *data, uint8_t dlc)
{
  /* Configure TX header */
  tx_header.Identifier = message_id;
  tx_header.IdType = FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = dlc << 16;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0;

  /* Transmit message */
  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, data);
}

/**
  * @brief  FDCAN RX FIFO 0 callback
  * @param  hfdcan: FDCAN handle
  * @param  RxFifo0ITs: Interrupt flags
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
  {
    /* Retrieve message from RX FIFO 0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
      /* Process received message */
      CAN_ProcessReceivedMessage(&rx_header, rx_data);
    }
  }
}
