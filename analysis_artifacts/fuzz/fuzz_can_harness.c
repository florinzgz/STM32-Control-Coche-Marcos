/*
 * CAN Message Fuzzing Harness for STM32-Control-Coche-Marcos
 *
 * This harness feeds arbitrary CAN payloads into the CAN message parser
 * to test bounds checking, DLC validation, and payload parsing robustness.
 *
 * Build:  gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE -DFUZZ_HARNESS \
 *              -Ianalysis_artifacts/stubs -ICore/Inc \
 *              -fsanitize=address,undefined -g -O1 \
 *              analysis_artifacts/fuzz/fuzz_can_harness.c \
 *              Core/Src/can_handler.c -o /tmp/fuzz_can -lm
 *
 * Run with AFL++:
 *   mkdir -p /tmp/fuzz_can_corpus && echo -ne '\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00' > /tmp/fuzz_can_corpus/seed1
 *   afl-fuzz -i /tmp/fuzz_can_corpus -o /tmp/fuzz_can_out -- /tmp/fuzz_can
 *
 * Run with libFuzzer (if available):
 *   clang -std=c11 -DHOST_TEST -D_GNU_SOURCE -DFUZZ_HARNESS -DLIBFUZZER \
 *         -fsanitize=fuzzer,address,undefined -g -O1 \
 *         -Ianalysis_artifacts/stubs -ICore/Inc \
 *         analysis_artifacts/fuzz/fuzz_can_harness.c Core/Src/can_handler.c -o /tmp/fuzz_can_lf -lm
 *   /tmp/fuzz_can_lf -max_total_time=60 /tmp/fuzz_can_corpus
 */

/* Redirect HAL functions BEFORE including the header so the static inline
 * stubs are skipped in favour of our fuzz-controlled implementations.     */
#define HAL_FDCAN_GetRxFifoFillLevel  HAL_FDCAN_GetRxFifoFillLevel_STUB
#define HAL_FDCAN_GetRxMessage        HAL_FDCAN_GetRxMessage_STUB

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32g4xx_hal.h"

#undef HAL_FDCAN_GetRxFifoFillLevel
#undef HAL_FDCAN_GetRxMessage

/* Fuzz state */
static uint8_t  fuzz_payload[64];
static uint32_t fuzz_payload_len = 0;
static uint32_t fuzz_id = 0;
static uint32_t fuzz_dlc_code = 0;
static int      fuzz_msg_available = 0;

/* Override: pretend one message in FIFO */
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo) {
    if (fuzz_msg_available) {
        fuzz_msg_available = 0;
        return 1;
    }
    return 0;
}

/* Override: return the fuzzed message */
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *hfdcan,
    uint32_t RxLocation, FDCAN_RxHeaderTypeDef *pRxHeader, uint8_t *pRxData) {
    pRxHeader->Identifier = fuzz_id;
    pRxHeader->IdType = FDCAN_STANDARD_ID;
    pRxHeader->DataLength = fuzz_dlc_code;
    memcpy(pRxData, fuzz_payload, 8);
    return HAL_OK;
}

/* External CAN processing function */
extern void CAN_ProcessMessages(void);

/* ------------------------------------------------------------------ */
/* Stub out all external symbols that can_handler.c references.       */
/* Include the real headers so types match exactly.                   */
/* ------------------------------------------------------------------ */
#include "safety_system.h"
#include "motor_control.h"
#include "sensor_manager.h"
#include "service_mode.h"
#include "error_log.h"
#include "math_safety.h"

/* --- Global variables referenced by can_handler.c --- */
bool fdcan_init_ok = false;
SafetyStatus_t safety_status = {0};
Safety_Error_t safety_error = SAFETY_ERROR_NONE;

/* --- Safety stubs (matching exact signatures from headers) --- */
SystemState_t Safety_GetState(void) { return SYS_STATE_ACTIVE; }
Safety_Error_t Safety_GetError(void) { return SAFETY_ERROR_NONE; }
uint8_t Safety_GetFaultFlags(void) { return 0; }
void Safety_SetDegradedLevel(DegradedLevel_t l, DegradedReason_t r) { (void)l; (void)r; }
void Safety_UpdateCANRxTime(void) {}
void Safety_SetEmergencyStop(void) {}
void Safety_ClearError(Safety_Error_t e) { (void)e; }
void Safety_SetError(Safety_Error_t e) { (void)e; }
float Safety_ValidateThrottle(float p) { return p; }
float Safety_ValidateSteering(float d) { return d; }
bool Safety_ValidateModeChange(bool e4, bool tt) { (void)e4; (void)tt; return true; }
DegradedLevel_t Safety_GetDegradedLevel(void) { return DEGRADED_LEVEL_NONE; }
float Safety_GetPowerLimitFactor(void) { return 1.0f; }

/* --- Traction stubs --- */
static TractionState_t dummy_traction = {0};
const TractionState_t* Traction_GetState(void) { return &dummy_traction; }
bool Startup_IsInhibited(void) { return false; }
void Traction_SetDemand(float d) { (void)d; }
void Traction_SetMode4x4(bool e) { (void)e; }
void Traction_SetAxisRotation(bool e) { (void)e; }
void Traction_SetGear(GearPosition_t g) { (void)g; }
void Traction_EmergencyStop(void) {}
void Steering_SetAngle(float a) { (void)a; }

/* --- Sensor stubs --- */
float Temperature_Get(uint8_t i) { (void)i; return 25.0f; }
uint8_t Temperature_GetCount(void) { return 0; }
float Current_GetAmps(uint8_t i) { (void)i; return 0.0f; }
float Voltage_GetBus(uint8_t i) { (void)i; return 24.0f; }

/* --- Math safety --- */
float sanitize_float(float v, float d) { return v; }
uint16_t float_to_u16_clamped(float v) { return (uint16_t)v; }

/* --- Service mode --- */
uint32_t ServiceMode_GetFaultMask(void) { return 0; }
uint32_t ServiceMode_GetEnabledMask(void) { return 0; }
uint32_t ServiceMode_GetDisabledMask(void) { return 0; }
void ServiceMode_HandleCAN(const uint8_t *d, uint8_t l) { (void)d; (void)l; }

/* --- Error log --- */
uint16_t ErrorLog_GetCount(void) { return 0; }
uint32_t ErrorLog_GetTotalEvents(void) { return 0; }

/* --- Obstacle stubs --- */
void Obstacle_ProcessCAN(const uint8_t *d, uint8_t l) { (void)d; (void)l; }
void Obstacle_ProcessSafetyCAN(const uint8_t *d, uint8_t l) { (void)d; (void)l; }

/* Map DLC code to byte count (matching FDCAN DLC encoding) */
static uint32_t dlc_to_code(uint8_t dlc) {
    /* FDCAN_DLC_BYTES_0 through FDCAN_DLC_BYTES_8 are 0x00000 through 0x80000 */
    if (dlc > 8) dlc = 8;
    return (uint32_t)dlc << 16;
}

#ifdef LIBFUZZER
int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
    if (size < 3) return 0;  /* Minimum: 2 bytes ID + 1 byte DLC + payload */

    fuzz_id = ((uint32_t)data[0] << 8) | data[1];
    /* Use standard CAN IDs (0x200-0x20F range matches project IDs) */
    fuzz_id = 0x200 + (fuzz_id % 16);

    uint8_t dlc = data[2] % 9;  /* DLC 0-8 */
    fuzz_dlc_code = dlc_to_code(dlc);
    fuzz_payload_len = dlc;

    memset(fuzz_payload, 0, sizeof(fuzz_payload));
    size_t copy_len = (size - 3 < 8) ? size - 3 : 8;
    memcpy(fuzz_payload, data + 3, copy_len);

    fuzz_msg_available = 1;
    CAN_ProcessMessages();
    return 0;
}
#else
/* stdin-based harness for AFL++ */
int main(void) {
    uint8_t buf[64];
    size_t n = fread(buf, 1, sizeof(buf), stdin);
    if (n < 3) return 0;

    fuzz_id = ((uint32_t)buf[0] << 8) | buf[1];
    fuzz_id = 0x200 + (fuzz_id % 16);

    uint8_t dlc = buf[2] % 9;
    fuzz_dlc_code = dlc_to_code(dlc);
    fuzz_payload_len = dlc;

    memset(fuzz_payload, 0, sizeof(fuzz_payload));
    size_t copy_len = (n - 3 < 8) ? n - 3 : 8;
    memcpy(fuzz_payload, buf + 3, copy_len);

    fuzz_msg_available = 1;
    CAN_ProcessMessages();

    return 0;
}
#endif
