#ifndef TANK_TURN_POLICY_H
#define TANK_TURN_POLICY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Physical wheel order shared by motor_control: FL, FR, RL, RR. */
#define TANK_TURN_WHEEL_FL 0U
#define TANK_TURN_WHEEL_FR 1U
#define TANK_TURN_WHEEL_RL 2U
#define TANK_TURN_WHEEL_RR 3U

/* Hysteresis for the momentary 360/tank-turn session.
 * The mode may be selected with the pedal released.  It becomes armed only
 * after a genuine press (>=3 %) and is cancelled when the pedal returns to
 * rest (<=1 %), so ADC noise around zero cannot chatter the mode. */
#define TANK_TURN_PEDAL_ENGAGE_PCT  3.0f
#define TANK_TURN_PEDAL_RELEASE_PCT 1.0f

/* Bench-verified hardware polarity correction used ONLY in tank turn.
 * The productive base algorithm already produces the correct vehicle-level
 * pattern (left side one way, right side the other).  On the installed car the
 * rear BTS7960/motor mounting presents the opposite electrical polarity to the
 * front pair during this mode, which produced a diagonal pattern.  Keep the
 * front command and invert both rear commands before touching the real PWM.
 *
 * logical_direction is normalised to -1/0/+1. */
static inline int8_t TankTurn_ResolveHardwareDirection(uint8_t wheel,
                                                        int8_t logical_direction)
{
    int8_t direction = (logical_direction > 0) ? 1
                       : (logical_direction < 0) ? -1 : 0;
    if (wheel == TANK_TURN_WHEEL_RL || wheel == TANK_TURN_WHEEL_RR) {
        direction = (int8_t)-direction;
    }
    return direction;
}

typedef struct {
    bool pedal_seen;
    bool release_latched;
} TankTurnReleaseState_t;

static inline void TankTurnRelease_Reset(TankTurnReleaseState_t *state)
{
    if (state == 0) return;
    state->pedal_seen = false;
    state->release_latched = false;
}

/* Returns true once a started tank-turn session must be cancelled.
 * A request selected while the pedal is at zero remains available; release is
 * recognised only after the pedal has first crossed the engage threshold. */
static inline bool TankTurnRelease_Update(TankTurnReleaseState_t *state,
                                          bool mode_enabled,
                                          float pedal_pct)
{
    if (state == 0) return true;

    if (!mode_enabled) {
        TankTurnRelease_Reset(state);
        return false;
    }

    if (state->release_latched) return true;

    if (pedal_pct < 0.0f) pedal_pct = 0.0f;
    if (pedal_pct > 100.0f) pedal_pct = 100.0f;

    if (pedal_pct >= TANK_TURN_PEDAL_ENGAGE_PCT) {
        state->pedal_seen = true;
    }

    if (state->pedal_seen && pedal_pct <= TANK_TURN_PEDAL_RELEASE_PCT) {
        state->release_latched = true;
        return true;
    }

    return false;
}

/* Compile-time invariants exercised by every STM32 ARM and host build. */
_Static_assert(TANK_TURN_WHEEL_FL == 0U && TANK_TURN_WHEEL_FR == 1U &&
               TANK_TURN_WHEEL_RL == 2U && TANK_TURN_WHEEL_RR == 3U,
               "Tank-turn wheel ordering must remain FL,FR,RL,RR");
_Static_assert(TANK_TURN_PEDAL_RELEASE_PCT < TANK_TURN_PEDAL_ENGAGE_PCT,
               "Tank-turn pedal release threshold needs hysteresis");

#ifdef __cplusplus
}
#endif

#endif /* TANK_TURN_POLICY_H */
