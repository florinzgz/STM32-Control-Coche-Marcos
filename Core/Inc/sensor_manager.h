#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ---- Initialization ---- */
void Sensor_Init(void);

/* ---- Wheel speed (EXTI interrupt-driven) ---- */
void Wheel_FL_IRQHandler(void);
void Wheel_FR_IRQHandler(void);
void Wheel_RL_IRQHandler(void);
void Wheel_RR_IRQHandler(void);

float Wheel_GetSpeed_FL(void);
float Wheel_GetSpeed_FR(void);
float Wheel_GetSpeed_RL(void);
float Wheel_GetSpeed_RR(void);
float Wheel_GetRPM_FL(void);

/* ---- Steering center inductive sensor (EXTI) ---- */
void SteeringCenter_IRQHandler(void);
bool SteeringCenter_Detected(void);
void SteeringCenter_ClearFlag(void);

/* ---- Pedal (internal ADC dual-sample + software plausibility) ---- */
void  Pedal_Update(void);
float Pedal_GetValue(void);       /* ADC raw value                         */
float Pedal_GetPercent(void);     /* EMA-filtered 0–100% (used for control)*/
bool  Pedal_IsPlausible(void);    /* Software plausibility checks pass     */
bool  Pedal_IsContradictory(void); /* Dual samples active but disagree     */
float Pedal_GetRawPercent(void);    /* Unfiltered instantaneous 0–100%   */

/* ---- DS18B20 Temperature (OneWire) ---- */
void Temperature_StartConversion(void);
void Temperature_ReadAll(void);
void Temperature_PeriodicRescan(void);
float Temperature_Get(uint8_t index);
uint8_t Temperature_GetCount(void);

/* ---- INA226 Current (I2C via TCA9548A) ---- */
void Current_ReadAll(void);
float Current_GetAmps(uint8_t index);
float Voltage_GetBus(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_MANAGER_H */
