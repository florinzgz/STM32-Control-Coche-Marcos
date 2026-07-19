/**
 ****************************************************************************
 * @file    mode_authority.h
 * @brief   Pure, host-testable arbitration of the DESIRED drive mode from its
 *          competing sources (physical 4x4 selector + local tank-turn request
 *          vs. the FlySky remote), including the momentary 360-mode policy.
 *
 *          DESIRED remains separate from the STM32-confirmed APPLIED state:
 *
 *            source arbitration  ->  desiredModeFlags  ->  ModeSync
 *              -> CMD_MODE -> ACK / heartbeat -> appliedModeFlags.
 *
 *          360/tank turn is deliberately NON-PERSISTENT and momentary:
 *            - an old NVS tank bit is cleared on every real Arduino boot;
 *            - selecting 360 with the pedal released is allowed;
 *            - after the pedal has genuinely been pressed (>=3 %), returning
 *              it to rest (<=1 %) clears the local toggle by reference;
 *            - the release latch prevents a held remote request from chattering
 *              back on until that source first requests tank=0.
 ****************************************************************************
 */
#ifndef MODE_AUTHORITY_H
#define MODE_AUTHORITY_H

#include <cstdint>

namespace mode_authority {

/// Bit 0 = 4x4, bit 1 = tank turn (mirrors can::MODE_FLAG_* / CMD_MODE payload).
inline constexpr uint8_t FLAG_4X4       = 0x01;
inline constexpr uint8_t FLAG_TANK_TURN = 0x02;
inline constexpr uint8_t FLAG_MASK      = FLAG_4X4 | FLAG_TANK_TURN;

inline constexpr uint8_t TANK_PEDAL_ENGAGE_PCT  = 3;
inline constexpr uint8_t TANK_PEDAL_RELEASE_PCT = 1;
static_assert(TANK_PEDAL_RELEASE_PCT < TANK_PEDAL_ENGAGE_PCT,
              "Tank-turn pedal thresholds require hysteresis");

struct TankReleaseGuard {
    bool requestActive  = false;
    bool pedalSeen      = false;
    bool releaseLockout = false;

    void reset() {
        requestActive  = false;
        pedalSeen      = false;
        releaseLockout = false;
    }

    bool allow(bool requested, uint8_t pedalPct, bool pedalValid) {
        if (!requested) {
            reset();
            return false;
        }

        if (releaseLockout) return false;

        if (!requestActive) {
            requestActive = true;
            pedalSeen = false;
        }

        if (pedalValid && pedalPct >= TANK_PEDAL_ENGAGE_PCT) {
            pedalSeen = true;
        }

        /* Once the driver has used the pedal, a real release OR an invalid /
         * contradictory pedal sample ends 360 mode.  The STM32 independently
         * enforces the same release rule, so loss of CAN cannot leave it active. */
        if (pedalSeen && (!pedalValid || pedalPct <= TANK_PEDAL_RELEASE_PCT)) {
            releaseLockout = true;
            return false;
        }

        return true;
    }
};

/* Inline C++17 state: can_rx observes the authoritative STM32 pedal frame on
 * the same Core-1 loop that calls arbitrate(), so no lock is required. */
inline uint8_t s_pedalPct = 0;
inline bool    s_pedalValid = false;
inline TankReleaseGuard s_localTankGuard;
inline TankReleaseGuard s_remoteTankGuard;
inline bool s_localBootSanitized = false;

inline void observePedal(uint8_t percent, bool plausible = true,
                         bool contradictory = false) {
    s_pedalPct = (percent <= 100U) ? percent : 100U;
    s_pedalValid = plausible && !contradictory;
}

/* Public for the host test and called automatically once in Arduino builds. */
inline void sanitizeLocalTankAtBoot(bool& localTankTurn) {
    localTankTurn = false;
    s_localTankGuard.reset();
    s_localBootSanitized = true;
}

inline void resetTankTurnPolicyForTest() {
    s_pedalPct = 0;
    s_pedalValid = false;
    s_localTankGuard.reset();
    s_remoteTankGuard.reset();
    s_localBootSanitized = false;
}

inline uint8_t arbitratePure(bool remoteActive, uint8_t remoteModeFlags,
                             bool localSelector4x4, bool localTankTurn) {
    if (remoteActive) {
        return static_cast<uint8_t>(remoteModeFlags & FLAG_MASK);
    }
    uint8_t flags = 0;
    if (localSelector4x4) flags |= FLAG_4X4;
    if (localTankTurn)    flags |= FLAG_TANK_TURN;
    return flags;
}

/**
 * Runtime arbitration overload.  localTankTurn is passed by reference so the
 * pedal-release edge clears the actual HMI toggle, not merely the outgoing CAN
 * bit.  The next touch therefore activates 360 normally with a single press.
 */
inline uint8_t arbitrate(bool remoteActive, uint8_t remoteModeFlags,
                         bool localSelector4x4, bool& localTankTurn) {
#ifdef ARDUINO
    /* Never restore a hazardous motion mode after a power cycle.  main.cpp may
     * load an old NVS bit for backward compatibility; the first arbitration
     * sanitises it before desiredModeFlags is ever sent to the STM32. */
    if (!s_localBootSanitized) {
        sanitizeLocalTankAtBoot(localTankTurn);
    }
#endif

    if (remoteActive) {
        const uint8_t masked = static_cast<uint8_t>(remoteModeFlags & FLAG_MASK);
        const bool requestedTank = (masked & FLAG_TANK_TURN) != 0U;
        const bool allowTank = s_remoteTankGuard.allow(requestedTank,
                                                        s_pedalPct,
                                                        s_pedalValid);
        uint8_t flags = static_cast<uint8_t>(masked & FLAG_4X4);
        if (allowTank) flags |= FLAG_TANK_TURN;
        return flags;
    }

    /* A remote release latch must not leak into a later independent session. */
    s_remoteTankGuard.reset();

    const bool allowTank = s_localTankGuard.allow(localTankTurn,
                                                   s_pedalPct,
                                                   s_pedalValid);
    if (localTankTurn && !allowTank && s_localTankGuard.releaseLockout) {
        localTankTurn = false;
    }

    uint8_t flags = 0;
    if (localSelector4x4) flags |= FLAG_4X4;
    if (allowTank)        flags |= FLAG_TANK_TURN;
    return flags;
}

/* Compatibility overloads keep existing pure tests/callers that pass a literal
 * or const bool completely stateless.  Production main.cpp passes a mutable
 * lvalue and therefore selects the runtime overload above. */
inline uint8_t arbitrate(bool remoteActive, uint8_t remoteModeFlags,
                         bool localSelector4x4, bool&& localTankTurn) {
    return arbitratePure(remoteActive, remoteModeFlags,
                         localSelector4x4, localTankTurn);
}

inline uint8_t arbitrate(bool remoteActive, uint8_t remoteModeFlags,
                         bool localSelector4x4, const bool& localTankTurn) {
    return arbitratePure(remoteActive, remoteModeFlags,
                         localSelector4x4, localTankTurn);
}

}  // namespace mode_authority

#endif  // MODE_AUTHORITY_H
