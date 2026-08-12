#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "eps_params.h"
#include "drive_tuning_store.h"
#include "dynbrake_store.h"
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
    bool mode4x4;            // true = 4x4, false = 4x2 rear-wheel drive
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

/** Hito 2 (PR #445) — wheel-equality self-test RAW actuation feed.  Called
 * once per cycle by can_handler.c's CAN_WheelEqualityTick() to report which
 * wheel(s) the self-test wants driven this cycle (bit i: 0=FL,1=FR,2=RL,
 * 3=RR), at what raw PWM (0-100 %, applied IDENTICALLY/unequalized to every
 * masked wheel) and direction.  When active=true, Traction_Update()
 * bypasses gear/pedal/ABS/TCS/Ackermann/ramp AND
 * TractionOutput_Resolve4x4()/TractionOutput_Resolve4x2Rear() — see the
 * bypass block at the top of Traction_Update() for the full rationale.
 * active=false immediately releases control back to the normal pipeline. */
void Traction_SetWheelEqActuation(uint8_t wheel_mask, uint8_t pwm_pct,
                                  bool forward, bool active);

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

/* ---- Runtime-configurable drive tuning (FASE 2) ----
 * Pedal ramp rates and motor dead-zone (creep) compensation become runtime
 * variables seeded with the historic compile-time values:
 *   AccelRamp=50 %/s, BrakeRamp=100 %/s, ReverseRamp=50 %/s (reverse only),
 *   CreepEnable=1, CreepPower=8 %, CreepDelay=0 ms.
 * Ranges/defaults live in drive_tuning_store.h (single source of truth).
 * The defaults reproduce the original firmware behaviour exactly; only the
 * fórmulas' constants are replaced by these variables, nothing else.     */
bool Traction_ValidateDriveTuning(const DriveTuning_t *t);
bool Traction_SetDriveTuning(const DriveTuning_t *t);
void Traction_GetDriveTuning(DriveTuning_t *out);

/* ---- Runtime-configurable dynamic braking (BLOQUE 1) ----
 * Engine-braking (opposing torque proportional to the fall rate of the
 * demand) becomes fully runtime-tunable, seeded with SOFTER-than-historic
 * defaults: Factor=0.20, MaxPct=25 %, MinSpeed=3.0 km/h, RampDown=80 %/s,
 * RampUp=60 %/s (NEW — limits the brake APPLICATION rate, closing the jerk
 * that the historic asymmetric hard-snap-down logic could produce), and
 * Enable=1.  Factor==0 or Enable==0 fully disables dynamic braking (the
 * vehicle coasts freely on release).  Ranges/defaults live in
 * dynbrake_store.h (single source of truth).                             */
bool Traction_ValidateDynBrakeTuning(const DynBrakeTuning_t *t);
bool Traction_SetDynBrakeTuning(const DynBrakeTuning_t *t);
void Traction_GetDynBrakeTuning(DynBrakeTuning_t *out);
void Traction_Update(void);
void Traction_EmergencyStop(void);

/* Calibration movement lock (audit P5.4): enforce demand 0 + traction
 * H-bridges COAST (EN LOW / PWM 0) and return true only if the enables read
 * LOW and the resolved PWM is 0.  Called periodically while a pedal-cal
 * session runs so the loss of the lock aborts the session immediately. */
bool Traction_CalibrationLock(void);

/* Read-only confirmation of the calibration movement lock (audit fix).
 * Unlike Traction_CalibrationLock(), this NEVER modifies any output.  It only
 * READS the effective traction demand, the resolved final PWM duty, the four
 * traction enable lines and the traction relay state, and returns true when
 * the "cannot move" condition currently holds.  Safe to call from telemetry,
 * diagnostic and QUERY paths that must not disturb the traction outputs. */
bool Traction_IsCalibrationLockConfirmed(void);

/* ---- MOTION_INHIBIT_REASON instrumentation (0x315) ----
 * Returns the latest MOTION_INHIBIT_REASON bitfield (MOTION_INHIBIT_*),
 * recomputed every Traction_Update() cycle.  Instrumentation only: it
 * observes why the traction chain is (or is not) producing torque and does
 * NOT alter any control or safety behaviour.  See motion_inhibit.h.       */
uint16_t Traction_GetMotionInhibit(void);
float    Traction_GetEffectiveDemandPct(void);
uint8_t  Traction_GetFinalPwmPct(void);
uint8_t  Traction_GetWheelFinalPwmPct(uint8_t wheel);
float    Traction_GetBrakeReleasePct(void);
const TractionState_t* Traction_GetState(void);

/* Single source of truth for the physical driven-wheel layout.
 * 4x2: RL/RR driven, FL/FR coast.  4x4 or tank turn: all four driven. */
bool Traction_IsWheelDriven(uint8_t wheel);

/* ---- Runtime-configurable Ackermann geometry (service diag C2) ----
 * wheelbase_m/track_m default to WHEELBASE_M/TRACK_WIDTH_M (vehicle_physics.h)
 * and are overridden only by geometry_store.c.  maxInnerDeg keeps the
 * existing MAX_STEER_DEG clamp. */
void Ackermann_SetGeometry(float wheelbase_m, float track_m, float maxInnerDeg);

/** Last per-wheel Ackermann differential multiplier computed by
 * Traction_Update() (1.000 = no correction).  Service-diag C6 reads this to
 * verify the differential is exactly 1.000 on all four wheels when the
 * steering wheel is centred, without recomputing it independently. */
float Traction_GetAckermannDiff(uint8_t wheel);

/** Steering-centre deadband (degrees) below which the Ackermann
 * differential (ackermann_diff.h: ACKERMANN_DEADBAND_DEG) applies no
 * correction.  Exposed read-only so callers (Hito 2 wheel-equality
 * self-test) can build their own "steering centred" precondition without
 * duplicating the literal. */
float Traction_GetAckermannDeadbandDeg(void);

/* Steering Functions */
void Steering_Init(void);
void Steering_SetAngle(float angle_deg);
void Steering_ControlLoop(void);
float Steering_GetCurrentAngle(void);
bool Steering_IsCalibrated(void);
void Steering_SetCalibrated(void);
void Steering_Neutralize(void);
void Steering_GetWheelAngles(float *out_fl_deg, float *out_fr_deg);
/** @brief Return last motor effort applied by the EPS loop (0..100 %). */
float Steering_GetMotorEffortPct(void);
/** @brief Return raw TIM2 encoder count (before gear-ratio division). */
int32_t Steering_GetEncoderRaw(void);

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
