/* Safety System Implementation */

#include "safety_system.h"

SafetyStatus_t safety_status;
Safety_Error_t safety_error = SAFETY_ERROR_NONE;

void Safety_Init(void) {
    safety_status.abs_active = false;
    safety_status.tcs_active = false;
    safety_status.abs_wheel_mask = 0;
    safety_status.tcs_wheel_mask = 0;
    safety_status.abs_activation_count = 0;
    safety_status.tcs_activation_count = 0;
}

void ABS_Update(void) { }
bool ABS_IsActive(void) { return safety_status.abs_active; }
void ABS_Reset(void) { safety_status.abs_active = false; }
void TCS_Update(void) { }
bool TCS_IsActive(void) { return safety_status.tcs_active; }
void TCS_Reset(void) { safety_status.tcs_active = false; }
void Safety_CheckCurrent(void) { }
void Safety_CheckTemperature(void) { }
void Safety_CheckCANTimeout(void) { }
void Safety_CheckSensors(void) { }
void Safety_EmergencyStop(void) { }
void Safety_FailSafe(void) { }
void Safety_PowerDown(void) { }
void Safety_SetError(Safety_Error_t error) { safety_error = error; }
void Safety_ClearError(Safety_Error_t error) { if (safety_error == error) safety_error = SAFETY_ERROR_NONE; }
Safety_Error_t Safety_GetError(void) { return safety_error; }
bool Safety_IsError(void) { return safety_error != SAFETY_ERROR_NONE; }