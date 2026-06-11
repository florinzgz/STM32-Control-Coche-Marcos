# Steering Centering — PB5 + Encoder-Z Dual Reference

> **Status:** ACTIVE (rev 1.10 of the CAN contract)
> **Scope:** STM32 firmware + ESP32 HMI diagnostic
> **Golden rule:** PB5 is the primary reference. Z never centers on its own.

---

## 1. Purpose

The steering center is now established from **two** references:

| Reference | Sensor | Pin | Role |
|-----------|--------|-----|------|
| **Primary** | LJ12A3 inductive proximity | PB5 | Physical center **and safety**. Detects metal ≈ ±1.5° around center. |
| **Secondary** | Encoder Z (index) channel of the E6B2-CWZ6C | PB4 | **Precision / verification only.** One pulse per encoder revolution. |

The encoder is mechanically coupled **1:1** with the steering wheel
(volante). The wheel travels ≈ ±355° (total ≈ 710°). The maximum
mechanical deviation between wheel and encoder is ≈ 0.5°.

### Why two references

- PB5 is robust and safe but coarse (its detection band is ≈ ±1.5°).
- The Z pulse is far more precise (one sharp edge per revolution) but it
  is **ambiguous on its own** — see §3.

By combining them, PB5 says *"we are in the center zone"* and Z says
*"and the precise center is exactly here"*. **Z is only ever trusted when
PB5 has already confirmed the center zone.**

---

## 2. Operating rules (non-negotiable)

| Condition | Result |
|-----------|--------|
| PB5 active **and** Z within tolerance | Center validated — high precision (status **OK**) |
| PB5 active, no Z seen | Center valid, normal operation (status **Z NOT SEEN**) |
| Z active **without** PB5 | **NOT a center.** Ignored, logged as diagnostic only |
| PB5 + Z separated beyond tolerance | Possible mechanical misalignment (status **OUT OF WINDOW** / **MECHANICAL OFFSET**) |

- PB5 is **never** removed or replaced.
- Z can **never** recenter the system unless PB5 first confirms the center zone.
- A Z fault **never** forces SAFE and can **never** block ACTIVE — it is
  surfaced as a warning/diagnostic only.

---

## 3. How many Z pulses exist, and which one is the center?

Encoder 1:1 with the wheel; useful travel ≈ 710° = 710/360 ≈ **1.97
revolutions**. Therefore there are approximately **two** Z pulses inside the
full range of travel. Neither pulse is intrinsically "the center" — the Z
channel cannot, by itself, tell you which revolution you are on.

**Resolution strategy:** the *correct* center Z is defined as **the Z pulse
captured while PB5 confirms the center zone**. Concretely:

1. PB5 detects center → TIM2 is zeroed (existing behavior, unchanged).
2. The firmware looks at the **last Z capture** (TIM2 count latched in the
   PB4/Z ISR) and computes the offset `z_last_pos − center_count`.
3. The offset is **normalised modulo one revolution** (4800 counts) and folded
   into the range (−2400, +2400]. This collapses Z pulses from adjacent
   revolutions onto the same fundamental offset, so a Z from "the other turn"
   maps identically — and is only ever accepted because PB5 confirmed the zone.
4. If `|offset| ≤ window` the Z is accepted as the precise center reference.

This is why **a Z pulse seen without PB5 is meaningless**: there is no way to
distinguish the center-revolution Z from the neighbouring-revolution Z without
the PB5 zone confirmation.

### Should Z be mechanically aligned to the exact center?

| Option | Pros | Cons |
|--------|------|------|
| **A — PB5 only** | Simple, proven, safe. | Center precision limited to PB5 band (≈ ±1.5°). |
| **B — PB5 + Z with stored offset** *(implemented)* | Keeps PB5 as safety authority; adds Z precision; tolerates any fixed Z↔center offset via calibration; no mechanical work. | Requires the offset to be calibrated/stored once; offset must stay within the window. |
| **C — PB5 + Z mechanically aligned to center** | Smallest offset, conceptually clean. | Hard to achieve physically (≈ 0.5° play), and provides no practical benefit over B since B already absorbs the offset in firmware. |

**Decision: Option B.** It delivers the precision benefit with zero hardware
changes and keeps PB5 as the sole safety authority.

---

## 4. Tolerances

Conversion: **4800 counts = 360°**, so **1° ≈ 13.333 counts** (encoder/volante
frame — *not* divided by the 6.48 gear ratio).

| Constant | Value | ≈ degrees | Meaning |
|----------|-------|-----------|---------|
| `STEERING_Z_STRICT_COUNTS` | 10 | ≈ 0.75° | Strict high-precision window |
| `STEERING_Z_WINDOW_COUNTS` | 25 | ≈ 1.9° | Recommended PB5+Z acceptance window |
| `STEERING_Z_FAULT_COUNTS`  | 40 | ≈ 3.0° | Beyond this ⇒ mechanical-offset fault |

Reference points from the hardware: 0.5° ≈ 7 counts, 1.5° ≈ 20 counts. The PB5
detection band (≈ ±1.5°) therefore corresponds to ≈ ±20 counts, so the ±25
window comfortably contains a valid Z while still flagging a real misalignment.

Defined in `Core/Inc/project_config.h`.

---

## 5. Calibration flow

When the system completes centering (`Centering_Complete`):

1. PB5 detects center.
2. The current TIM2 count and the last Z capture are read **before** TIM2 is
   zeroed.
3. `SteeringZ_OnCenterConfirmed()` computes the normalised Z↔center offset.
4. TIM2 is zeroed (unchanged).
5. The offset + validity + tolerance are persisted via
   `SteeringCal_SaveWithZ()` (flash format v2).

Outcomes:

- **PB5 + Z valid** → calibration marked valid (`z_center_valid = 1`).
- **PB5 without Z** → operation allowed, Z marked *not validated*.
- **Z without PB5** → not used as center, logged as a diagnostic event.
- **PB5 + Z out of tolerance** → possible mechanical misalignment recorded.

Manual re-calibration is available from the HMI (`STEER_Z_OP_CALIBRATE`) but is
**only accepted while PB5 currently reads center** and only in BOOT/STANDBY.

---

## 6. Boot validation

At boot PB5 remains mandatory; Z is auxiliary.

| Boot condition | Behavior |
|----------------|----------|
| PB5 OK + Z OK | Center validated — dual reference |
| PB5 OK + Z absent | Center valid + diagnostic notice |
| PB5 OK + Z out of tolerance | Misalignment warning |
| Z without PB5 | Ignored as center |

The system **never** enters SAFE automatically because of a Z fault. The
justification: Z is a precision *aid*, not a safety input; PB5 alone is
sufficient for safe operation, so degrading to "PB5-only" on a Z anomaly is
strictly safer than tripping SAFE and is consistent with the rule that no
change may block ACTIVE.

---

## 7. Refinement (Phase 6) — decision

Using the stored offset to fine-trim the zero **only when PB5 confirms center
and Z is valid** was evaluated. To stay unambiguously safe, the shipped
behavior keeps the existing PB5-driven zero authoritative and uses Z for
**diagnostic + validation** only. The stored offset is available for a future,
separately-reviewed refinement; it is not applied to the live zero today.
Conditions that any such refinement must always satisfy: never use Z without
PB5, never use a Z from another revolution, never recenter from an isolated Z.

---

## 8. Telemetry & HMI

- CAN `DIAG_STEERING_Z` (**0x30E**, DLC 8, on-demand burst after a QUERY) — see
  `docs/CAN_CONTRACT_FINAL.md` §4.19 for the exact payload.
- Service action `SERVICE_ACTION_STEERING_Z` (**0xF8**) with QUERY / CALIBRATE /
  CLEAR sub-opcodes.
- HMI: a **STEERING Z CENTER** section on the Engineering → Encoder Calibration
  screen shows PB5 state, Z pulse count, Z last position, Z offset, Z calibrated,
  Z slip and the combined status (OK / Z NOT SEEN / Z OUT OF WINDOW / MECHANICAL
  OFFSET / NOT CALIBRATED), with QUERY / CALIBRATE Z OFFSET / CLEAR Z CALIBRATION
  actions. CLEAR requires a double confirmation. The ESP32 is **never** the
  authority — every action is validated on the STM32.

---

## 9. Flash format & migration

See `docs/STEERING_PERSISTENT_CALIBRATION.md`. Summary: the steering-cal page
gains a v2 record (`"STC2"`) adding `format_version`,
`z_center_offset_counts`, `z_center_tolerance`, `z_center_valid`. v1 (`"STC1"`)
records are read transparently (Z marked invalid) and rewritten as v2 on the
next save. No other flash page is touched and existing center calibration is
preserved.
