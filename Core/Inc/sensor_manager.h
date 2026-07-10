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

/* ---- Per-wheel diagnostics (report-only) ----
 * Expose raw sensing state so the operator can distinguish a genuine
 * sensor fault from expected behaviour when a wheel is turned by hand
 * (e.g. a sensor parked directly over a bolt reads a constant level and
 * emits no edges).  Diagnostic only — must NOT gate any control path.  */
uint32_t Wheel_GetPulseCount(uint8_t idx);      /* accepted EXTI pulses, 0 if idx invalid   */
uint32_t Wheel_GetLastEdgeAgeMs(uint8_t idx);   /* ms since last accepted edge (UINT32_MAX invalid) */
uint8_t  Wheel_GetGpioLevel(uint8_t idx);       /* current pin level 0/1, 0xFF if idx invalid */

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
 * Pedal_GetRawADC() / Pedal_GetRawADC2() expose the most recent two
 * raw ADC samples captured by Pedal_Update(); Pedal_GetRawADCDiff()
 * returns their absolute difference for diagnostics.                 */
void     Pedal_ApplyCalibration(uint16_t adc_min, uint16_t adc_max);
uint16_t Pedal_GetRawADC(void);
uint16_t Pedal_GetRawADC2(void);
uint16_t Pedal_GetRawADCDiff(void);

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

/* ---- I2C topology diagnostic accessors ----------------------------------
 * Let the HMI / diagnostic CAN frame (0x309) distinguish a missing TCA9548A
 * multiplexer (0x70) from a missing/dead INA226 (0x40) on a given channel.
 * Sensor_GetInaBusOkMask() additionally exposes whether the last bus-voltage
 * register read succeeded for each channel, so Safety_CheckBatteryVoltage()
 * can reject invalid samples without changing the existing CAN contract.     */
bool    Sensor_GetMuxPresent(void);           /* TCA9548A acked last cycle      */
uint8_t Sensor_GetInaOkMask(void);            /* bit i = INA226 ch i acked      */
uint8_t Sensor_GetInaBusOkMask(void);         /* bit i = bus-voltage read OK    */
uint8_t Sensor_GetInaExpectedMask(void);      /* bit i = ch i expected powered  */
uint8_t Sensor_GetI2cFailCount(void);         /* failed transactions this cycle */
uint8_t Sensor_GetI2cRecoveryAttempts(void);  /* bus-recovery attempts (sticky) */
bool    Sensor_GetI2cEverOk(void);            /* latched: any INA seen OK ever  */
/* Duration of the most recent Current_ReadAll() call (HAL_GetTick delta, ms).
 * Saturates at 255.  Visible on 0x309 byte 6 for field diagnostics.         */
uint8_t Sensor_GetI2cLastReadMs(void);

/* ---- I2C service-mode scan (Level 3 diagnostic, on-demand) ----
 * Active probe used by SERVICE_CMD 0xF6 → CAN 0x30B.  Reads SDA/SCL idle
 * levels, probes the TCA9548A (0x70) and each INA226 (0x40) behind the mux
 * channels, and attempts a bus recovery if SDA is held low.  Report-only —
 * never gates any control or safety path.                                  */
typedef struct {
    bool    scl_idle_high;       /* SCL line read high while idle           */
    bool    sda_idle_high;       /* SDA line read high while idle           */
    bool    recovery_attempted;  /* bus recovery was run (SDA stuck low)    */
    bool    recovery_success;    /* SDA released high after recovery         */
    bool    mux_present;         /* TCA9548A 0x70 acked                      */
    uint8_t ina_present_mask;    /* bit0..5 = INA226 0x40 acked behind ch0..5 */
    uint8_t fail_count;          /* i2c_fail_count after the scan            */
    uint8_t recovery_attempts;   /* sticky bus-recovery attempt counter      */
} Sensor_I2cScanResult_t;

Sensor_I2cScanResult_t Sensor_RunI2CServiceScan(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_MANAGER_H */
