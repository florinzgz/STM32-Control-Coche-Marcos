#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "eps_params.h"
#include <stdint.h>
#include <stdbool.h>

// Motor indices
#define MOTOR_FL  0  // Front Left
#define MOTOR_FR  1  // Front Right
#define MOTOR_RL  2  // Rear Left
#define MOTOR_RR  3  // Rear Right

/* Gear position — received from ESP32 via CAN_ID_CMD_MODE byte 1.
 * Traced to CAN_PROTOCOL.md: 0=PARK, 1=REVERSE, 2=NEUTRAL, 3=FORWARD.
 * Park (P) was implicit in the base firmware; it is now handled
 * explicitly as an active hold brake on the STM32 side.               */
typedef enum {
    GEAR_PARK    = 0,
    GEAR_REVERSE = 1,
    GEAR_NEUTRAL = 2,
    GEAR_FORWARD = 3,
    GEAR_FORWARD_D2 = 4   /* Full power (100 %) forward mode */
} GearPosition_t;

// Wheel state structure (per wheel)
typedef struct {
    float demandPct;      // Desired power 0-100%
    float currentA;       // Measured current (Amperes)
    float tempC;          // Temperature (Celsius)
    float speedKmh;       // Wheel speed (km/h)
    uint16_t pwm;         // PWM duty cycle (0-4249 for TIM1 @ 20kHz center-aligned)
    bool reverse;         // Direction flag
    float effortPct;      // Motor effort percentage
} WheelState_t;

// Traction system state
typedef struct {
    WheelState_t wheels[4];  // FL, FR, RL, RR
    bool mode4x4;            // true = 4x4, false = 4x2 (front only)
    float demandPct;         // Global throttle demand 0-100%
    bool axisRotation;       // Tank turn mode
} TractionState_t;

// Ackermann calculation result
typedef struct {
    float innerDeg;    // Inner wheel angle
    float outerDeg;    // Outer wheel angle
} AckermannResult_t;

/* Motor Control Functions */
void Motor_Init(void);
void Traction_Init(void);
void Traction_SetDemand(float throttlePct);
void Traction_SetMode4x4(bool enable);
void Traction_SetAxisRotation(bool enable);
void Traction_SetGear(GearPosition_t gear);
GearPosition_t Traction_GetGear(void);

/* ---- Runtime-configurable gear power limits (R-2) ----
 * Per-gear traction power limits expressed as integer percentages
 * (0..100).  Defaults equal the historic compile-time values
 * (D2=100, D1=60, R=60) so behaviour is unchanged until overridden.
 * Ranges and defaults live in gear_limits_store.h (single source).  */
bool Traction_ValidateGearLimits(uint8_t d2_pct, uint8_t d1_pct, uint8_t r_pct);
bool Traction_SetGearLimits(uint8_t d2_pct, uint8_t d1_pct, uint8_t r_pct);
void Traction_GetGearLimits(uint8_t *d2_pct, uint8_t *d1_pct, uint8_t *r_pct);

/* ---- Runtime-configurable per-gear acceleration RESPONSE profile ----
 * Per-gear factors (integer percent) that soften the pedal demand signal
 * before the global ramp limiter in Traction_SetDemand().  Defaults
 * (D2=100, D1=70, R=40) come from gear_limits_store.h.  The factor is
 * applied to positive demand only and clamped to <=100 %, so it can only
 * soften the demand, never amplify it.  Independent of the power limits
 * above; both apply on the same demand pipeline at different stages.    */
bool Traction_ValidateGearResponse(uint8_t d2_pct, uint8_t d1_pct, uint8_t r_pct);
bool Traction_SetGearResponse(uint8_t d2_pct, uint8_t d1_pct, uint8_t r_pct);
void Traction_GetGearResponse(uint8_t *d2_pct, uint8_t *d1_pct, uint8_t *r_pct);
void Traction_Update(void);
void Traction_EmergencyStop(void);
const TractionState_t* Traction_GetState(void);

/* Steering Functions */
void Steering_Init(void);
void Steering_SetAngle(float angle_deg);
void Steering_ControlLoop(void);
float Steering_GetCurrentAngle(void);
bool Steering_IsCalibrated(void);
void Steering_SetCalibrated(void);
void Steering_Neutralize(void);
void Steering_GetWheelAngles(float *out_fl_deg, float *out_fr_deg);

/* Encoder Health */
void  Encoder_CheckHealth(void);
bool  Encoder_HasFault(void);

/* Low-level PWM control (TIM1/TIM8 direct) */
void Motor_SetPWM_FL(uint16_t pwm, bool reverse);
void Motor_SetPWM_FR(uint16_t pwm, bool reverse);
void Motor_SetPWM_RL(uint16_t pwm, bool reverse);
void Motor_SetPWM_RR(uint16_t pwm, bool reverse);
void Motor_SetPWM_Steering(uint16_t pwm, bool reverse);

/* ---- Motor operating mode (BTS7960 physical states) ----
 *
 *  MOTOR_MODE_COAST: EN=LOW, PWM=0  → Hi-Z outputs, motor free-spinning
 *  MOTOR_MODE_BRAKE: EN=HIGH, PWM=0 → motor terminals shorted, passive brake
 *  MOTOR_MODE_DRIVE: EN=HIGH, PWM>0 → motor driven at requested duty
 *
 *  Used internally by Traction_Update() to explicitly select the desired
 *  BTS7960 physical state.  Eliminates the M4 bug where duty==0 always
 *  produced coast (EN=LOW) regardless of the intended brake state.       */
typedef enum {
    MOTOR_MODE_COAST = 0,   /* Hi-Z — free rolling                        */
    MOTOR_MODE_DRIVE = 1,   /* Active drive — RPWM or LPWM at duty cycle  */
    MOTOR_MODE_BRAKE = 2    /* Passive brake — motor windings shorted      */
} motor_mode_t;

/* Signed-speed API — positive = forward, negative = reverse, 0 = coast.
 * Generates RPWM/LPWM directly without external logic.
 * Guarantees RPWM and LPWM are never simultaneously non-zero.
 * NOTE: speed=0 always produces coast (EN=LOW).  For explicit brake
 * (EN=HIGH, PWM=0), use Motor_SetMode() with MOTOR_MODE_BRAKE via
 * the Traction_Update() pathway.                                       */
void Motor_SetSignedPWM_FL(int16_t speed);
void Motor_SetSignedPWM_FR(int16_t speed);
void Motor_SetSignedPWM_RL(int16_t speed);
void Motor_SetSignedPWM_RR(int16_t speed);
void Motor_SetSignedPWM_Steering(int16_t speed);

/* ---- Runtime active-brake override (field-test API) ----
 * Allows enabling active braking at runtime without recompilation.
 * pwm_ticks: LPWM duty during MOTOR_MODE_BRAKE (0 = passive, max 4249).
 *            Typical field-test values: 212 (~5%) to 425 (~10%).
 * Only effective when BRAKE_ACTIVE_FALLBACK is 0 (compile-time passive).
 * Thread-safe: single atomic uint16_t write.                           */
void Motor_SetBrakeActiveOverride(uint16_t pwm_ticks);
uint16_t Motor_GetBrakeActiveOverride(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
