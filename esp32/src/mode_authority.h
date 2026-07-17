/**
 ****************************************************************************
 * @file    mode_authority.h
 * @brief   Pure, host-testable arbitration of the DESIRED drive mode from its
 *          competing sources (physical 4x4 selector + local tank-turn toggle
 *          vs. the FlySky remote).
 *
 *          §1/§2 of the mode-sync closure: main.cpp used to overload a single
 *          `currentModeFlags` as request, applied-state, selector base, tank
 *          base, remote base AND the value shown in the HMI.  The intent
 *          (DESIRED) is now separated from the STM32-confirmed value (APPLIED),
 *          and there is a SINGLE path that feeds the desired mode into ModeSync:
 *
 *            source arbitration  →  desiredModeFlags  →  g_modeSync.setDesired()
 *              →  ModeSync::update()  →  sendModeCommand()  →  ACK / heartbeat
 *              →  appliedModeFlags.
 *
 *          This header owns only the "source arbitration" step so the authority
 *          rules are unit-tested without any Arduino / CAN dependency:
 *            - REMOTE active  : the remote's requested mode is the desire.
 *            - LOCAL (default): the physical selector's 4x4 bit plus the local
 *                               tank-turn toggle are the desire.
 *          Switching authority simply re-evaluates this function, so the desired
 *          mode is recomputed and ModeSync resynchronises automatically.
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

/**
 * Arbitrate the driver's DESIRED drive mode from the active authority.
 *
 * @param remoteActive        True while the FlySky remote holds motion
 *                            authority (LOCAL sources are ignored).
 * @param remoteModeFlags     Mode requested by the remote (bit0=4x4, bit1=tank).
 * @param localSelector4x4    True when the physical selector reads 4x4.
 * @param localTankTurn       State of the local (HMI) tank-turn toggle.
 * @return  Desired mode flags, restricted to the two defined bits.
 */
inline uint8_t arbitrate(bool remoteActive, uint8_t remoteModeFlags,
                         bool localSelector4x4, bool localTankTurn) {
    if (remoteActive) {
        return (uint8_t)(remoteModeFlags & FLAG_MASK);
    }
    uint8_t flags = 0;
    if (localSelector4x4) flags |= FLAG_4X4;
    if (localTankTurn)    flags |= FLAG_TANK_TURN;
    return flags;
}

}  // namespace mode_authority

#endif  // MODE_AUTHORITY_H
