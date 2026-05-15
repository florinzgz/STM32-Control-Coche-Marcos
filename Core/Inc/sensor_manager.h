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

/* Call once per control cycle (e.g. 10 ms tier) to compute all wheel
 * speeds from accumulated EXTI pulse counts.  Getters below then
 * return the cached values — no redundant computation, no side-effects
 * on shared ISR counters from multiple call sites.                     */
void  Wheel_UpdateSpeeds(void);

float Wheel_GetSpeed_FL(void);   /* Returns km/h (cached from last update) */
float Wheel_GetSpeed_FR(void);
float Wheel_GetSpeed_RL(void);
float Wheel_GetSpeed_RR(void);
float Wheel_GetRPM_FL(void);

/* Returns true if wheel idx (0-3) has not received a valid pulse
 * within WHEEL_STALE_TIMEOUT_MS — indicates sensor disconnect or
 * vehicle stopped.                                                     */
bool  Wheel_IsStale(uint8_t idx);

/* ---- Steering center inductive sensor (EXTI) ---- */
void SteeringCenter_IRQHandler(void);
bool SteeringCenter_Detected(void);
void SteeringCenter_ClearFlag(void);

/* ---- Debounce EMI diagnostic counters (report-only) ----
 * Count edge pulses rejected by the µs-level DWT pre-filter (Step 0,
 * SENSOR_DEBOUNCE_US = 200 µs).  Counters saturate at 0xFFFFFFFF.
 * Diagnostic only — must NOT gate any control/safety path.             */
uint32_t Sensor_GetFilteredCount(uint8_t idx);   /* idx 0..3 (FL,FR,RL,RR), out-of-range returns 0 */
uint32_t Sensor_GetSteerFilteredCount(void);

/* ---- Pedal (internal ADC dual-sample + software plausibility) ---- */
void  Pedal_Update(void);
float Pedal_GetValue(void);       /* ADC raw value                         */
float Pedal_GetPercent(void);     /* EMA-filtered 0–100% (used for control)*/
bool  Pedal_IsPlausible(void);    /* Software plausibility checks pass     */
bool  Pedal_IsContradictory(void); /* Dual samples active but disagree     */
float Pedal_GetRawPercent(void);    /* Unfiltered instantaneous 0–100%   */

/* ---- Runtime endpoint calibration (pedal_cal_store) ----
 * Pedal_ApplyCalibration() replaces the raw→% endpoints used by
 * Pedal_RawToPercent().  The caller MUST range-validate the inputs
 * (see PedalCal_Validate()).  This function does not touch EMA,
 * plausibility, fault thresholds, or any other safety gate.
 * Pedal_GetRawADC() returns the most recent raw ADC reading
 * (12-bit) — used by the calibration UI to capture endpoints.       */
void     Pedal_ApplyCalibration(uint16_t adc_min, uint16_t adc_max);
uint16_t Pedal_GetRawADC(void);

/* ---- Fresh-conversion sampler for calibration stability check ----
 * Triggers a brand-new ADC conversion and returns the raw 12-bit count.
 * Unlike Pedal_GetRawADC() (which returns the value cached by the last
 * Pedal_Update() cycle), this helper forces hardware acquisition every
 * time it is called.  This is required by the pedal calibration
 * stability check (can_handler.c::pedalcal_sample_stable) which runs
 * inside CAN_ProcessMessages() — a context where the main 100 Hz loop
 * is paused, so the cached value would not refresh between samples.
 *
 * Safety: this function does NOT mutate any pedal pipeline state.
 * It does NOT touch pedal_raw_adc, pedal_raw_adc2, pedal_pct, EMA,
 * plausibility flags, dual-sample logic, fault thresholds, or rate
 * limiting.  It is a pure read with side-effects limited to the ADC
 * peripheral (Start/Poll/GetValue/Stop) — same sequence used inside
 * Pedal_ReadDualSample(), so HAL state machine remains consistent.   */
uint16_t Pedal_SampleRawNow(void);

/* ---- DS18B20 Temperature (OneWire) ---- */
void Temperature_StartConversion(void);
void Temperature_ReadAll(void);
void Temperature_PeriodicRescan(void);
float Temperature_Get(uint8_t index);
uint8_t Temperature_GetCount(void);

/* ---- DS18B20 diagnostic accessors (no CAN impact) ----
 * Report-only: consumed by UI / future diagnostic frame.  Do NOT gate
 * any existing safety path on these — they complement, not replace,
 * the Safety_SetError() calls inside OW_ReadTemperature().            */
bool     Temperature_IsTopologyValid(void);      /* all sensors present */
bool     Temperature_HasTopologyChanged(void);   /* ROM set changed     */
void     Temperature_ClearTopologyChanged(void); /* ack the latch       */
bool     Temperature_IsStale(uint8_t idx);       /* frozen/dead sensor  */
uint16_t Temperature_GetDiagnosticFlags(void);   /* packed bitmask      */
bool     Temperature_IsValid(uint8_t idx);       /* Roadmap 1.5: CRC/range OK */

/* ---- INA226 Current (I2C via TCA9548A) ---- */
void Current_ReadAll(void);
float Current_GetAmps(uint8_t index);
float Current_GetAmpsRaw(uint8_t index);  /* Unfiltered instantaneous reading */
float Voltage_GetBus(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_MANAGER_H */
