/**
  ****************************************************************************
  * @file    safety_system_patched.c
  * @brief   PR #429 safety-policy corrections layered over safety_system.c.
  *
  * This translation unit deliberately includes the production implementation
  * and replaces only the two audited entry points below.  Keeping the patch
  * narrow avoids duplicating the large safety module while preserving all
  * unrelated ABS/TCS/battery/thermal/obstacle behaviour byte-for-byte.
  ****************************************************************************
  */

/* Rename the legacy public entry points while the production source is
 * included.  Every other function and all module-static state remain in this
 * same translation unit and are therefore available to the corrected entry
 * points defined after the include. */
#define Safety_CheckRelayHealth  Safety_CheckRelayHealth_Legacy
#define Relay_SequencerUpdate    Relay_SequencerUpdate_Legacy
#include "safety_system.c"
#undef Relay_SequencerUpdate
#undef Safety_CheckRelayHealth

/**
 * @brief Update evidence-grade diagnostics without creating false DTC 16.
 *
 * The installed hardware has no post-relay voltage/contact feedback.  The
 * production diagnostic therefore deliberately downgrades every apparent
 * open-contact result to RELAY_OPEN_SUSPECTED.  The old latch ignored that
 * verdict and asserted RELAY OPEN whenever wheels were visibly rotating while
 * CH0..CH3 were below an adaptive 1.5..4 A threshold — a physically
 * contradictory conclusion and the exact fault observed with the vehicle
 * raised from the floor.
 *
 * Suspected relay/current-sense conditions remain fully visible through
 * Safety_GetRelayHealthDiag() and CAN 0x317, but cannot alter motion policy
 * until independent hardware evidence exists.
 */
void Safety_CheckRelayHealth(void)
{
    Safety_UpdateRelayHealthDiag();

    /* Do not let obsolete debounce state survive between diagnostic cycles.
     * A genuine hardware danger is still covered by overcurrent, battery,
     * watchdog, I2C and driver protections; only the unsupported DTC16 latch
     * is removed. */
    relay_chk_active        = 0U;
    relay_chk_debounce      = 0U;
    relay_chk_recovery_tick = 0U;
    relay_chk_fault_set_tick = 0U;
}

/**
 * @brief Deterministic relay sequence for every drive-capable state.
 *
 * ACTIVE, DEGRADED and LIMP_HOME all accept traction demand.  The former
 * ACTIVE-only sequencer could leave STANDBY->DEGRADED/LIMP_HOME with
 * power_ready=false forever.  Traction is now sequenced in every drive-capable
 * state, while the 12 V steering bridge is energised only after physical
 * centering has validated the steering reference.
 */
void Relay_SequencerUpdate(void)
{
    const bool drive_capable =
        (system_state == SYS_STATE_ACTIVE)   ||
        (system_state == SYS_STATE_DEGRADED) ||
        (system_state == SYS_STATE_LIMP_HOME);

    if (!drive_capable) {
        /* Preserve the direct PC12 ownership used by steering homing while the
         * normal sequence is IDLE in BOOT/STANDBY.  Only cancel a sequence that
         * was started by the traction state machine. */
        if (relay_seq_state == RELAY_SEQ_TRACTION_ON ||
            relay_seq_state == RELAY_SEQ_COMPLETE) {
            Relay_PowerDown();
        }
        return;
    }

    /* A direct STANDBY->DEGRADED/LIMP_HOME transition must start the normal
     * non-blocking power-up sequence just like STANDBY->ACTIVE. */
    if (relay_seq_state == RELAY_SEQ_IDLE) {
        Relay_PowerUp();
    }

    const uint32_t now = HAL_GetTick();
    if (relay_seq_state == RELAY_SEQ_TRACTION_ON &&
        (now - relay_seq_timestamp) >= RELAY_TRACTION_SETTLE_MS) {
        HAL_GPIO_WritePin(GPIOC, PIN_RELAY_STEER_PWR,
            Steering_IsCalibrated() ? GPIO_PIN_SET : GPIO_PIN_RESET);
        relay_seq_state = RELAY_SEQ_COMPLETE;
    }

    if (relay_seq_state == RELAY_SEQ_COMPLETE) {
        /* Keep command outputs coherent if calibration is cleared/recovered
         * after the traction sequence has already completed. */
        HAL_GPIO_WritePin(GPIOC, PIN_RELAY_TRAC, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, PIN_RELAY_STEER_PWR,
            Steering_IsCalibrated() ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}
