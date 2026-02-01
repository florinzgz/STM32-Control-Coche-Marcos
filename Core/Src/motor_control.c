/**
  ******************************************************************************
  * @file    motor_control.c
  * @brief   Motor control implementation with PWM, PID and Ackermann
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor_control.h"
#include "sensor_manager.h"
#include <math.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
MotorState_t motors[MOTOR_COUNT];
Ackermann_t ackermann;
DriveMode_t drive_mode = DRIVE_MODE_4X2;

static Gear_t current_gear = GEAR_NEUTRAL;
static uint8_t throttle_demand = 0;

/* Private function prototypes -----------------------------------------------*/
static void Motor_UpdatePWM(MotorID_t motor_id);
static void Motor_SetGPIO(MotorID_t motor_id);

/**
  * @brief  Initialize motor control system
  * @retval None
  */
void MotorControl_Init(void)
{
  /* Initialize all motors */
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    motors[i].pwm_duty = 0;
    motors[i].direction = MOTOR_DIR_FORWARD;
    motors[i].enabled = false;
    motors[i].current_rpm = 0.0f;
    motors[i].target_rpm = 0.0f;
    motors[i].current_amps = 0.0f;
    motors[i].temperature_c = 25.0f;

    /* Initialize PID controllers */
    if (i == MOTOR_STEER)
    {
      PID_Init(&motors[i].pid, PID_KP_STEER, PID_KI_STEER, PID_KD_STEER, 0.01f);
    }
    else
    {
      PID_Init(&motors[i].pid, PID_KP_SPEED, PID_KI_SPEED, PID_KD_SPEED, 0.01f);
    }
  }

  /* Initialize Ackermann geometry */
  Ackermann_Init();

  /* Ensure all motors are stopped */
  Motor_EmergencyStop();
}

/**
  * @brief  Set motor speed (PWM duty cycle)
  * @param  motor_id: Motor identifier
  * @param  duty_cycle: PWM duty cycle (0-100)
  * @retval None
  */
void Motor_SetSpeed(MotorID_t motor_id, uint16_t duty_cycle)
{
  if (motor_id >= MOTOR_COUNT)
    return;

  /* Clamp duty cycle */
  if (duty_cycle > MAX_PWM_DUTY)
    duty_cycle = MAX_PWM_DUTY;

  motors[motor_id].pwm_duty = duty_cycle;
  Motor_UpdatePWM(motor_id);
}

/**
  * @brief  Set motor direction
  * @param  motor_id: Motor identifier
  * @param  direction: Motor direction
  * @retval None
  */
void Motor_SetDirection(MotorID_t motor_id, MotorDirection_t direction)
{
  if (motor_id >= MOTOR_COUNT)
    return;

  motors[motor_id].direction = direction;
  Motor_SetGPIO(motor_id);
}

/**
  * @brief  Enable or disable motor
  * @param  motor_id: Motor identifier
  * @param  enable: true to enable, false to disable
  * @retval None
  */
void Motor_Enable(MotorID_t motor_id, bool enable)
{
  if (motor_id >= MOTOR_COUNT)
    return;

  motors[motor_id].enabled = enable;
  Motor_SetGPIO(motor_id);

  if (!enable)
  {
    Motor_SetSpeed(motor_id, 0);
  }
}

/**
  * @brief  Emergency stop all motors
  * @retval None
  */
void Motor_EmergencyStop(void)
{
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    Motor_SetSpeed(i, 0);
    Motor_Enable(i, false);
  }
}

/**
  * @brief  Set traction throttle (0-100%)
  * @param  throttle_percent: Throttle position
  * @retval None
  */
void Traction_SetThrottle(uint8_t throttle_percent)
{
  if (throttle_percent > 100)
    throttle_percent = 100;

  throttle_demand = throttle_percent;
}

/**
  * @brief  Set gear selection
  * @param  gear: Gear to select
  * @retval None
  */
void Traction_SetGear(Gear_t gear)
{
  current_gear = gear;

  /* Update motor directions based on gear */
  MotorDirection_t dir = MOTOR_DIR_FORWARD;
  if (gear == GEAR_REVERSE)
  {
    dir = MOTOR_DIR_REVERSE;
  }

  for (int i = MOTOR_FL; i <= MOTOR_RR; i++)
  {
    Motor_SetDirection(i, dir);
  }
}

/**
  * @brief  Apply Ackermann steering geometry to wheel speeds
  * @param  steering_angle: Steering angle in degrees
  * @retval None
  */
void Traction_ApplyAckermann(float steering_angle)
{
  if (current_gear == GEAR_NEUTRAL || throttle_demand == 0)
    return;

  float base_speed = throttle_demand;
  float fl_speed, fr_speed, rl_speed, rr_speed;

  /* Calculate individual wheel speeds using Ackermann geometry */
  Ackermann_CalculateWheelSpeeds(steering_angle, base_speed,
                                  &fl_speed, &fr_speed,
                                  &rl_speed, &rr_speed);

  /* Set target speeds */
  motors[MOTOR_FL].target_rpm = fl_speed;
  motors[MOTOR_FR].target_rpm = fr_speed;
  motors[MOTOR_RL].target_rpm = rl_speed;
  motors[MOTOR_RR].target_rpm = rr_speed;
}

/**
  * @brief  Update traction motor speeds
  * @retval None
  */
void Traction_UpdateSpeeds(void)
{
  /* Apply Ackermann geometry based on current steering angle */
  Traction_ApplyAckermann(ackermann.steering_angle);

  /* Update each motor */
  for (int i = MOTOR_FL; i <= MOTOR_RR; i++)
  {
    /* Enable motor if throttle is applied and not in neutral */
    bool should_enable = (throttle_demand > 0 && current_gear != GEAR_NEUTRAL);

    /* For 4x2 mode, only enable rear motors */
    if (drive_mode == DRIVE_MODE_4X2 && (i == MOTOR_FL || i == MOTOR_FR))
    {
      should_enable = false;
    }

    Motor_Enable(i, should_enable);

    if (should_enable)
    {
      /* Use PID to control speed */
      float pid_output = PID_Update(&motors[i].pid, motors[i].current_rpm);
      uint16_t duty = (uint16_t)(fabsf(pid_output));

      Motor_SetSpeed(i, duty);
    }
    else
    {
      Motor_SetSpeed(i, 0);
      PID_Reset(&motors[i].pid);
    }
  }
}

/**
  * @brief  Set steering angle
  * @param  angle_degrees: Target steering angle in degrees
  * @retval None
  */
void Steering_SetAngle(float angle_degrees)
{
  /* Clamp angle to maximum */
  if (angle_degrees > MAX_STEERING_ANGLE)
    angle_degrees = MAX_STEERING_ANGLE;
  if (angle_degrees < -MAX_STEERING_ANGLE)
    angle_degrees = -MAX_STEERING_ANGLE;

  ackermann.target_angle = angle_degrees;
}

/**
  * @brief  Update steering position using PID control
  * @retval None
  */
void Steering_UpdatePosition(void)
{
  /* Get current encoder position */
  int16_t encoder_pos = Steering_GetEncoderPosition();
  ackermann.encoder_position = encoder_pos;

  /* Convert encoder position to angle */
  float encoder_angle = (float)(encoder_pos - ackermann.encoder_zero) *
                        (360.0f / ENCODER_CPR);
  ackermann.steering_angle = encoder_angle;

  /* Update PID setpoint */
  motors[MOTOR_STEER].pid.setpoint = ackermann.target_angle;

  /* Calculate PID output */
  float pid_output = PID_Update(&motors[MOTOR_STEER].pid, ackermann.steering_angle);

  /* Determine direction and speed */
  if (fabsf(pid_output) > MIN_PWM_DUTY)
  {
    MotorDirection_t dir = (pid_output > 0) ? MOTOR_DIR_FORWARD : MOTOR_DIR_REVERSE;
    Motor_SetDirection(MOTOR_STEER, dir);
    Motor_SetSpeed(MOTOR_STEER, (uint16_t)fabsf(pid_output));
    Motor_Enable(MOTOR_STEER, true);
  }
  else
  {
    Motor_SetSpeed(MOTOR_STEER, 0);
    Motor_Enable(MOTOR_STEER, false);
  }
}

/**
  * @brief  Get encoder position
  * @retval Encoder count
  */
int16_t Steering_GetEncoderPosition(void)
{
  return (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
}

/**
  * @brief  Reset encoder to zero position
  * @retval None
  */
void Steering_ResetEncoder(void)
{
  ackermann.encoder_zero = Steering_GetEncoderPosition();
}

/**
  * @brief  Initialize PID controller
  * @param  pid: Pointer to PID structure
  * @param  kp: Proportional gain
  * @param  ki: Integral gain
  * @param  kd: Derivative gain
  * @param  dt: Sample time
  * @retval None
  */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float dt)
{
  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
  pid->dt = dt;
  pid->setpoint = 0.0f;
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->output_min = 0.0f;
  pid->output_max = 100.0f;
}

/**
  * @brief  Update PID controller
  * @param  pid: Pointer to PID structure
  * @param  measured_value: Current measured value
  * @retval PID output
  */
float PID_Update(PID_Controller_t *pid, float measured_value)
{
  /* Calculate error */
  float error = pid->setpoint - measured_value;

  /* Proportional term */
  float p_term = pid->Kp * error;

  /* Integral term */
  pid->integral += error * pid->dt;
  float i_term = pid->Ki * pid->integral;

  /* Derivative term */
  float derivative = 0.0f;
  if (pid->dt > 0.0f)
  {
    derivative = (error - pid->prev_error) / pid->dt;
  }
  float d_term = pid->Kd * derivative;

  /* Calculate output */
  float output = p_term + i_term + d_term;

  /* Clamp output */
  if (output > pid->output_max)
    output = pid->output_max;
  if (output < pid->output_min)
    output = pid->output_min;

  /* Save error for next iteration */
  pid->prev_error = error;

  return output;
}

/**
  * @brief  Reset PID controller
  * @param  pid: Pointer to PID structure
  * @retval None
  */
void PID_Reset(PID_Controller_t *pid)
{
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
}

/**
  * @brief  Initialize Ackermann geometry
  * @retval None
  */
void Ackermann_Init(void)
{
  ackermann.wheelbase = WHEELBASE_MM;
  ackermann.track_width = TRACK_WIDTH_MM;
  ackermann.encoder_position = 0;
  ackermann.encoder_zero = 0;
  ackermann.steering_angle = 0.0f;
  ackermann.target_angle = 0.0f;
  ackermann.max_angle = MAX_STEERING_ANGLE;
}

/**
  * @brief  Calculate individual wheel speeds using Ackermann geometry
  * @param  steering_angle: Steering angle in degrees
  * @param  base_speed: Base speed (0-100%)
  * @param  fl_speed: Front left speed output
  * @param  fr_speed: Front right speed output
  * @param  rl_speed: Rear left speed output
  * @param  rr_speed: Rear right speed output
  * @retval None
  */
void Ackermann_CalculateWheelSpeeds(float steering_angle, float base_speed,
                                     float *fl_speed, float *fr_speed,
                                     float *rl_speed, float *rr_speed)
{
  /* If going straight, all wheels same speed */
  if (fabsf(steering_angle) < 0.1f)
  {
    *fl_speed = base_speed;
    *fr_speed = base_speed;
    *rl_speed = base_speed;
    *rr_speed = base_speed;
    return;
  }

  /* Convert angle to radians */
  float angle_rad = steering_angle * (M_PI / 180.0f);

  /* Calculate turn radius */
  float turn_radius = ackermann.wheelbase / tanf(fabsf(angle_rad));

  /* Calculate speed ratios for each wheel */
  float inner_radius = turn_radius - (ackermann.track_width / 2.0f);
  float outer_radius = turn_radius + (ackermann.track_width / 2.0f);

  float speed_ratio_inner = inner_radius / turn_radius;
  float speed_ratio_outer = outer_radius / turn_radius;

  /* Assign speeds based on turn direction */
  if (steering_angle > 0)  /* Turning right */
  {
    *fl_speed = base_speed * speed_ratio_outer;
    *fr_speed = base_speed * speed_ratio_inner;
    *rl_speed = base_speed * speed_ratio_outer;
    *rr_speed = base_speed * speed_ratio_inner;
  }
  else  /* Turning left */
  {
    *fl_speed = base_speed * speed_ratio_inner;
    *fr_speed = base_speed * speed_ratio_outer;
    *rl_speed = base_speed * speed_ratio_inner;
    *rr_speed = base_speed * speed_ratio_outer;
  }
}

/**
  * @brief  Set PWM duty cycle for a specific timer channel
  * @param  htim: Timer handle
  * @param  channel: Timer channel
  * @param  duty: Duty cycle (0-100)
  * @retval None
  */
void PWM_SetDutyCycle(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t duty)
{
  if (duty > 100)
    duty = 100;

  uint32_t pulse = (duty * PWM_PERIOD) / 100;
  __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}

/**
  * @brief  Update PWM output for a motor
  * @param  motor_id: Motor identifier
  * @retval None
  */
static void Motor_UpdatePWM(MotorID_t motor_id)
{
  switch (motor_id)
  {
    case MOTOR_FL:
      PWM_SetDutyCycle(&htim1, TIM_CHANNEL_1, motors[motor_id].pwm_duty);
      break;
    case MOTOR_FR:
      PWM_SetDutyCycle(&htim1, TIM_CHANNEL_2, motors[motor_id].pwm_duty);
      break;
    case MOTOR_RL:
      PWM_SetDutyCycle(&htim1, TIM_CHANNEL_3, motors[motor_id].pwm_duty);
      break;
    case MOTOR_RR:
      PWM_SetDutyCycle(&htim1, TIM_CHANNEL_4, motors[motor_id].pwm_duty);
      break;
    case MOTOR_STEER:
      PWM_SetDutyCycle(&htim8, TIM_CHANNEL_1, motors[motor_id].pwm_duty);
      break;
    default:
      break;
  }
}

/**
  * @brief  Set GPIO outputs for motor control
  * @param  motor_id: Motor identifier
  * @retval None
  */
static void Motor_SetGPIO(MotorID_t motor_id)
{
  GPIO_PinState dir_state = (motors[motor_id].direction == MOTOR_DIR_FORWARD) ?
                             GPIO_PIN_RESET : GPIO_PIN_SET;
  GPIO_PinState en_state = motors[motor_id].enabled ? GPIO_PIN_SET : GPIO_PIN_RESET;

  switch (motor_id)
  {
    case MOTOR_FL:
      HAL_GPIO_WritePin(DIR_FL_GPIO_Port, DIR_FL_Pin, dir_state);
      HAL_GPIO_WritePin(EN_FL_GPIO_Port, EN_FL_Pin, en_state);
      break;
    case MOTOR_FR:
      HAL_GPIO_WritePin(DIR_FR_GPIO_Port, DIR_FR_Pin, dir_state);
      HAL_GPIO_WritePin(EN_FR_GPIO_Port, EN_FR_Pin, en_state);
      break;
    case MOTOR_RL:
      HAL_GPIO_WritePin(DIR_RL_GPIO_Port, DIR_RL_Pin, dir_state);
      HAL_GPIO_WritePin(EN_RL_GPIO_Port, EN_RL_Pin, en_state);
      break;
    case MOTOR_RR:
      HAL_GPIO_WritePin(DIR_RR_GPIO_Port, DIR_RR_Pin, dir_state);
      HAL_GPIO_WritePin(EN_RR_GPIO_Port, EN_RR_Pin, en_state);
      break;
    case MOTOR_STEER:
      HAL_GPIO_WritePin(DIR_STEER_GPIO_Port, DIR_STEER_Pin, dir_state);
      HAL_GPIO_WritePin(EN_STEER_GPIO_Port, EN_STEER_Pin, en_state);
      break;
    default:
      break;
  }
}
