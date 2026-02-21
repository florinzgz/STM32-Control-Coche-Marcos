// =============================================================================
// ESP32-S3 — CAN Command TX Module
//
// Implements the HMI Control Contract:
//
//   §1 Continuous control: CMD_THROTTLE (0x100) and CMD_STEERING (0x101) are
//       transmitted every 20 ms unconditionally while driving is enabled,
//       even when the operator is not touching the screen (sends zeros).
//
//   §2 Throttle ramp-down: when touch is released, throttle decreases
//       monotonically to 0 over ≤150 ms instead of an instant cut.
//
//   §3 ACTIVE gate: control is only enabled after receiving ACTIVE state
//       from STM32 for at least 2 consecutive heartbeat cycles.
//       Until then, only heartbeat is sent — no throttle, no steering.
//
//   §4 Frequencies: Heartbeat 100 ms (unchanged), Throttle 20 ms,
//       Steering 20 ms, Mode/Gear on-demand only.
//
// No CAN protocol changes — uses existing frozen IDs and payloads.
// No STM32 changes required.
//
// Reference: docs/CAN_CONTRACT_FINAL.md rev 1.3
//            docs/HMI_ACTIVE_DRIVING_PLAN.md
// =============================================================================

#ifndef CAN_TX_H
#define CAN_TX_H

#include <cstdint>
#include "can_ids.h"

namespace can_tx {

/// Initialize module.  Call once from setup() after CAN bus init.
void init();

/// Periodic update — call every loop() iteration.
/// Handles 20 ms continuous TX, throttle ramp-down, and ACTIVE gate.
/// @param systemState  current state from STM32 heartbeat (byte 1 of 0x001)
void update(can::SystemState systemState);

/// Set desired throttle percentage (0–100).
/// The value takes effect on the next 20 ms TX cycle.
/// When touch is released, call setThrottle(0) — the module applies
/// the §2 ramp-down automatically.
void setThrottle(uint8_t pct);

/// Set desired steering angle in raw 0.1° units (int16, LE on wire).
/// Positive = right, negative = left.
void setSteering(int16_t angleRaw);

/// Set drive mode and gear.  Transmitted once (on-demand), not periodic.
/// @param flags  MODE_FLAG_4X4 | MODE_FLAG_TANK_TURN
/// @param gear   0=P, 1=R, 2=N, 3=D1, 4=D2
void setModeAndGear(uint8_t flags, uint8_t gear);

/// Returns true when the module is actively sending control commands
/// (ACTIVE gate passed, §3).
bool isControlEnabled();

/// Returns the current throttle value being transmitted (after ramp).
uint8_t currentThrottle();

/// Returns the current steering value being transmitted.
int16_t currentSteering();

} // namespace can_tx

#endif // CAN_TX_H
