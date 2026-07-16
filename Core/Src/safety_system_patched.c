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
#define Safety_CheckRelayHealth    Safety_CheckRelayHealth_Legacy
#define Relay_SequencerUpdate      Relay_SequencerUpdate_Legacy
#define Safety_RelayOverrideUpdate Safety_RelayOverrideUpdate_Legacy
#include "safety_system.c"
#undef Safety_RelayOverrideUpdate
#undef Relay_SequencerUpdate
#undef Safety_CheckRelayHealth

/**
 * @brief Single authority that decides whether the steering-motor isolation
 *        relay (PC12, PIN_RELAY_STEER_PWR) may be energised (contact closed).
 *
 * PHYSICAL TOPOLOGY (owner-confirmed):
 *
 *     12 V BATTERY → INA226 CH5 + SHUNT → STEER MOTOR RELAY (PC12) → BTS7960/MOTOR
 *
 * PC12 drives the relay in the final branch that CONNECTS the steering motor.
 * De-energising PC12 opens that contact and electrically isolates the motor;
 * it does NOT remove the INA226/shunt supply (those sit before the relay).
 *
 * This is the ONE authorisation gate demanded by the EPS isolation policy.
 * When it returns false the relay command MUST be OFF, and — because a
 * MECHANICAL_ONLY / ELECTRICAL_HAZARD state is latched for the whole power
 * cycle — no later sequencer or diagnostic path may re-close PC12.  It NEVER
 * touches the traction relay (PC11) or the traction power-ready sequencer, so
 * traction is fully preserved.
 */
static bool Steering_MotorRelayAllowed(void)
{
    return Steering_IsCalibrated() &&
           Steering_EpsIsAvailable() &&
           !Steering_IsMechanicalOnly() &&
           (Steering_GetEpsState() != EPS_STATE_ELECTRICAL_HAZARD);
}

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
            Steering_MotorRelayAllowed() ? GPIO_PIN_SET : GPIO_PIN_RESET);
        relay_seq_state = RELAY_SEQ_COMPLETE;
    }

    if (relay_seq_state == RELAY_SEQ_COMPLETE) {
        /* Keep command outputs coherent every cycle.  Traction (PC11) stays
         * ON so Safety_IsPowerReady() remains valid; the steering-motor relay
         * (PC12) follows the single Steering_MotorRelayAllowed() authority so
         * a latched EPS isolation (MECHANICAL_ONLY / ELECTRICAL_HAZARD) can
         * NEVER re-close PC12 for the rest of the power cycle — the previous
         * Steering_IsCalibrated()-only check re-energised it after
         * Steering_DisableAssistFault(), the exact defect corrected here. */
        HAL_GPIO_WritePin(GPIOC, PIN_RELAY_TRAC, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, PIN_RELAY_STEER_PWR,
            Steering_MotorRelayAllowed() ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

/**
 * @brief Engineering relay-override tick that can never re-close PC12 once the
 *        EPS assist has been isolated.
 *
 * The legacy override lets a maintainer toggle the relay GPIOs from the
 * engineering menu while in STANDBY.  After a latched EPS isolation the
 * steering motor must stay electrically disconnected for the whole power
 * cycle, so any override attempt to close PC12 is force-cleared here.  The
 * traction relay (PC11) handling is left entirely to the legacy path.
 */
void Safety_RelayOverrideUpdate(void)
{
    Safety_RelayOverrideUpdate_Legacy();

    if (Steering_IsMechanicalOnly()) {
        /* Atomic single-pin release of the steering-motor relay (PC12).
         * BSRR upper half clears the bit in one bus cycle; PC11 untouched. */
        GPIOC->BSRR = (uint32_t)PIN_RELAY_STEER_PWR << 16U;
    }
}
