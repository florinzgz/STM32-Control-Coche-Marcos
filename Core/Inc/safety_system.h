/**
  ******************************************************************************
  * @file    safety_system.h
  * @brief   Safety systems including ABS, TCS, and protection mechanisms
  ******************************************************************************
  */

#ifndef __SAFETY_SYSTEM_H
#define __SAFETY_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/* Safety system state */
typedef enum {
    SAFETY_OK = 0,
    SAFETY_WARNING = 1,
    SAFETY_CRITICAL = 2,
    SAFETY_SHUTDOWN = 3
} SafetyLevel_t;

/* Protection flags */
typedef struct {
    bool overcurrent_fl : 1;
    bool overcurrent_fr : 1;
    bool overcurrent_rl : 1;
    bool overcurrent_rr : 1;
    bool overcurrent_steer : 1;
    bool overtemp_fl : 1;
    bool overtemp_fr : 1;
    bool overtemp_rl : 1;
    bool overtemp_rr : 1;
    bool can_timeout : 1;
    bool encoder_fault : 1;
    bool pedal_fault : 1;
    bool reserved : 4;
} ProtectionFlags_t;

/* ABS (Anti-lock Braking System) state */
typedef struct {
    bool enabled;
    bool active[4];                  /* Active flag per wheel (FL, FR, RL, RR) */
    float slip_threshold;            /* Wheel slip threshold (%) */
    float speed_threshold_kmh;       /* Minimum speed to activate ABS */
} ABS_State_t;

/* TCS (Traction Control System) state */
typedef struct {
    bool enabled;
    bool active[4];                  /* Active flag per wheel (FL, FR, RL, RR) */
    float slip_threshold;            /* Wheel slip threshold (%) */
    float max_torque_reduction;      /* Maximum torque reduction (%) */
} TCS_State_t;

/* Temperature protection */
typedef struct {
    float warning_temp_c;            /* Warning temperature threshold */
    float critical_temp_c;           /* Critical temperature threshold */
    float max_temp_c;                /* Maximum measured temperature */
    bool throttle_limited;           /* Throttle limit active flag */
    uint8_t throttle_limit_percent;  /* Current throttle limit (0-100%) */
} TempProtection_t;

/* Current protection */
typedef struct {
    float warning_current_a;         /* Warning current threshold */
    float critical_current_a;        /* Critical current threshold */
    float max_current_a;             /* Maximum measured current */
    uint32_t overcurrent_count;      /* Overcurrent event counter */
} CurrentProtection_t;

/* Watchdog state */
typedef struct {
    bool enabled;
    uint32_t timeout_ms;             /* Watchdog timeout in milliseconds */
    uint32_t last_refresh_ms;        /* Last refresh timestamp */
} Watchdog_t;

/* Overall safety state */
typedef struct {
    SafetyLevel_t level;
    ProtectionFlags_t flags;
    ABS_State_t abs;
    TCS_State_t tcs;
    TempProtection_t temp_protection;
    CurrentProtection_t current_protection;
    Watchdog_t watchdog;
    bool emergency_stop_active;
} SafetySystem_t;

/* Exported constants --------------------------------------------------------*/

/* ABS configuration */
#define ABS_SLIP_THRESHOLD 15.0f     /* 15% slip threshold */
#define ABS_MIN_SPEED_KMH 5.0f       /* Minimum speed for ABS activation */

/* TCS configuration */
#define TCS_SLIP_THRESHOLD 20.0f     /* 20% slip threshold */
#define TCS_MAX_TORQUE_REDUCTION 50.0f /* Maximum 50% torque reduction */

/* Temperature protection thresholds */
#define TEMP_WARNING_C 70.0f         /* Warning at 70°C */
#define TEMP_CRITICAL_C 80.0f        /* Critical at 80°C */
#define TEMP_SHUTDOWN_C 90.0f        /* Emergency shutdown at 90°C */

/* Current protection thresholds */
#define CURRENT_WARNING_A 15.0f      /* Warning at 15A */
#define CURRENT_CRITICAL_A 20.0f     /* Critical at 20A */
#define CURRENT_SHUTDOWN_A 25.0f     /* Emergency shutdown at 25A */

/* Watchdog configuration */
#define IWDG_TIMEOUT_MS 1000         /* Independent watchdog timeout */
#define IWDG_REFRESH_INTERVAL_MS 100 /* Refresh every 100ms */

/* CAN heartbeat timeout */
#define CAN_HEARTBEAT_TIMEOUT_MS 250 /* Timeout if no heartbeat */

/* Exported variables --------------------------------------------------------*/
extern SafetySystem_t safety_system;

/* Exported function prototypes ----------------------------------------------*/

/* Initialization */
void Safety_Init(void);
void Safety_EnableWatchdog(void);

/* Safety monitoring */
void Safety_Update(void);
SafetyLevel_t Safety_GetLevel(void);
bool Safety_IsSystemOK(void);

/* ABS functions */
void ABS_Init(void);
void ABS_Enable(bool enable);
void ABS_Update(void);
bool ABS_CheckWheelLock(uint8_t wheel_index);
void ABS_ControlWheel(uint8_t wheel_index, bool lock_detected);

/* TCS functions */
void TCS_Init(void);
void TCS_Enable(bool enable);
void TCS_Update(void);
bool TCS_CheckWheelSlip(uint8_t wheel_index);
float TCS_CalculateTorqueReduction(uint8_t wheel_index, float slip_percent);

/* Temperature protection */
void TempProtection_Init(void);
void TempProtection_Update(void);
uint8_t TempProtection_GetThrottleLimit(void);
bool TempProtection_IsOverheated(void);

/* Current protection */
void CurrentProtection_Init(void);
void CurrentProtection_Update(void);
bool CurrentProtection_IsOvercurrent(void);
void CurrentProtection_HandleOvercurrent(void);

/* Emergency procedures */
void Safety_EmergencyStop(void);
void Safety_EnterSafeMode(void);
void Safety_ClearErrors(void);

/* Watchdog functions */
void Watchdog_Init(void);
void Watchdog_Refresh(void);
bool Watchdog_CheckTimeout(void);

/* Diagnostic functions */
uint16_t Safety_GetErrorFlags(void);
void Safety_GetDiagnosticInfo(char *buffer, uint16_t buffer_size);

/* Protection checks */
bool Safety_CheckAllProtections(void);
void Safety_UpdateProtectionFlags(void);

#ifdef __cplusplus
}
#endif

#endif /* __SAFETY_SYSTEM_H */
