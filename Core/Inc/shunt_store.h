/**
  ****************************************************************************
  * @file    shunt_store.h
  * @brief   Persistent per-channel INA226 shunt resistance storage — C3
  *
  * Stores the runtime-tunable shunt resistance (mOhm) for each of the six
  * INA226 channels (FL/FR/RL/RR motors, battery, steering) in flash so they
  * can be adjusted from the ESP32-HMI SERVICE_AUTOTEST menu without
  * re-flashing firmware.  Modelled 1:1 on battery_limits_store.c.
  *
  * Channel index mapping (matches project_config.h):
  *   0=FL, 1=FR, 2=RL, 3=RR (motors), 4=battery, 5=steering.
  *
  * Defaults: 0.75 mOhm (battery, index 4), 1.5 mOhm (all others) — matches
  * INA226_SHUNT_MOHM_BATTERY / INA226_SHUNT_MOHM_MOTOR in project_config.h.
  * Range: 0.10..5.00 mOhm per the problem statement.
  *
  * "Recalcula la calibracion del INA226 al guardar": this codebase computes
  * current entirely in software (amps = shunt_uV / shunt_mOhm), there is no
  * INA226 hardware CAL register write anywhere in sensor_manager.c.
  * sensor_manager.c reads the per-channel divisor straight from
  * ShuntStore_GetEffectiveMohm() on every Current_ReadAll()/
  * Sensor_UpdateChannel5Diag() cycle -- consistent with every other
  * Bloque C store, this reflects the RAM-staged value immediately on
  * Stage(), the same cycle it is called; Save() only adds persistence
  * across reboot, it is not what makes a staged edit "count".
  *
  * Safety invariants:
  *   - Stage()/Save() themselves are NOT callable during an active
  *     service-diag test (actuators must be stopped first) per Block C
  *     rules — changing the current divisor while a wheel is spinning
  *     would corrupt the wheel-test measurement it is being used to
  *     validate. This precondition is enforced by the caller
  *     (service_diag_session.c, Block A), not by this store.
  *   - On flash blank / CRC-invalid / out-of-range the module silently
  *     falls back to the compile-time defaults.  Boot is NEVER blocked.
  *
  * Flash layout:
  *   Page 114 (0x08072000, 4 KB) — dedicated to shunt calibration.
  *   Single slot with magic "SHN1" + CRC32 integrity check.
  ****************************************************************************
  */

#ifndef SHUNT_STORE_H
#define SHUNT_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#define SHUNT_STORE_NUM_CHANNELS   6U

/* ---- Hard validation range (matches the problem statement) ---- */
#define SHUNT_STORE_MOHM_MIN       0.10f
#define SHUNT_STORE_MOHM_MAX       5.00f

typedef struct {
    float mohm[SHUNT_STORE_NUM_CHANNELS];
} ShuntCal_t;

/* HAL-free single source used by production and host tests. */
static inline bool ShuntCal_ValidateValues(const ShuntCal_t *s)
{
    if (!s) return false;
    for (uint32_t i = 0; i < SHUNT_STORE_NUM_CHANNELS; i++) {
        /* Reject NaN/Inf explicitly: a NaN compared with < or > is always
         * false, so the range check below would silently let it through. */
        if (isnan(s->mohm[i]) || isinf(s->mohm[i])) return false;
        if (s->mohm[i] < SHUNT_STORE_MOHM_MIN ||
            s->mohm[i] > SHUNT_STORE_MOHM_MAX) return false;
    }
    return true;
}

void ShuntStore_Init(void);
bool ShuntStore_IsValid(void);
void ShuntStore_GetDefaults(ShuntCal_t *out);
bool ShuntStore_Validate(const ShuntCal_t *s);

/** Effective (RAM-staged) values, in mOhm, consumed by Current_ReadAll()
 *  and the CH5 diagnostic path in sensor_manager.c. */
float ShuntStore_GetEffectiveMohm(uint8_t channel);

/** Stage a candidate set in RAM only.  Applied to Current_ReadAll() the
 *  same cycle, via ShuntStore_GetEffectiveMohm() -- consistent with the
 *  sibling stores.  ShuntStore_Save() additionally persists it to flash so
 *  it survives a reboot; a caller (service_diag_session.c, Block A) is
 *  responsible for only invoking Stage()/Save() with actuators stopped,
 *  per the C3 Block-C rule. */
bool ShuntStore_Stage(const ShuntCal_t *s);
void ShuntStore_GetStaged(ShuntCal_t *out);
void ShuntStore_Revert(void);
void ShuntStore_ResetToDefaults(void);

/**
 * @brief Persist the staged shunt calibration to flash and immediately
 *        "recalculate the INA226 calibration" (push the new divisor into
 *        sensor_manager.c's runtime array).
 */
bool ShuntStore_Save(void);

#ifdef __cplusplus
}
#endif

#endif /* SHUNT_STORE_H */
