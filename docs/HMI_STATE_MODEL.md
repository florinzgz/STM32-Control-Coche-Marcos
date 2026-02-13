# HMI State Model

**Revision:** 1.0
**Status:** Normative — Derived from `v1.0-stm32-safety-baseline`
**Date:** 2026-02-07
**Scope:** HMI behavior on the ESP32-S3, governed exclusively by STM32 system states and CAN signals defined in `docs/CAN_CONTRACT_FINAL.md`

This document defines the deterministic behavior of the HMI (Human–Machine Interface) running on the ESP32-S3. Every screen, user action, overlay, and transition described here is dictated by the STM32 heartbeat (CAN ID 0x001) and associated status messages. The HMI has no independent authority over vehicle functions.

---

## 1. STM32 System States → HMI Screens

The STM32 reports its `system_state` in byte 1 of the heartbeat message (0x001). The HMI must display exactly one primary screen per state.

| system_state | Value | HMI Primary Screen | Purpose |
|--------------|-------|---------------------|---------|
| BOOT | 0 | **Boot / Splash** | Static brand splash. No interactive elements. |
| STANDBY | 1 | **Standby / Ready** | System ready indicator. Waiting for ACTIVE transition. |
| ACTIVE | 2 | **Drive Dashboard** | Full operational display with live telemetry. |
| DEGRADED | 3 | **Drive Dashboard (degraded)** | Degraded / limp mode. Drive dashboard with reduced-power indicator. Commands accepted with limits (40% power, 50% speed). |
| SAFE | 4 | **Safe-Mode Alert** | Prominent safety warning. Limited telemetry visible. |
| ERROR | 5 | **Error / Shutdown** | Unrecoverable-fault screen. No controls available. |

### Transition Rules

- The HMI must change screens within **one heartbeat cycle (≤ 100 ms)** of receiving a new `system_state` value.
- If `system_state` is outside the range 0–5, the HMI must treat it as equivalent to ERROR (5) and display the Error / Shutdown screen.
- Screen transitions are **one-way followers** of the STM32 state. The HMI never requests or influences a state change.

---

## 2. Information Visible per State

### 2.1 BOOT (system_state = 0)

| Element | Visible | Source |
|---------|---------|--------|
| Brand / splash graphic | Yes | Static asset |
| Firmware version | Optional | Local ESP32 build info |
| CAN link status | Yes | Frame arrival of 0x001 |
| Telemetry gauges | **No** | — |
| User controls | **No** | — |

### 2.2 STANDBY (system_state = 1)

| Element | Visible | Source |
|---------|---------|--------|
| "System Ready" indicator | Yes | State value |
| CAN link status | Yes | Frame arrival of 0x001 |
| fault_flags summary | Yes | Byte 2 of 0x001 |
| Temperature readings | Yes (read-only) | 0x202 |
| Speed / current gauges | **No** | Not meaningful while stopped |
| Drive controls | **No** | — |

### 2.3 ACTIVE (system_state = 2)

| Element | Visible | Source |
|---------|---------|--------|
| Wheel speeds (4) | Yes | 0x200 bytes 0–7 (uint16 LE, ×0.1 km/h) |
| Motor currents (4) | Yes | 0x201 bytes 0–7 (uint16 LE, ×0.01 A) |
| Temperatures (5) | Yes | 0x202 bytes 0–4 (int8, °C) |
| ABS active flag | Yes | 0x203 byte 0 |
| TCS active flag | Yes | 0x203 byte 1 |
| Safety error code | Yes | 0x203 byte 2 |
| Steering angle (actual) | Yes | 0x204 bytes 0–1 (int16 LE, ×0.1°) |
| Steering calibrated flag | Yes | 0x204 byte 2 |
| fault_flags bitmask | Yes | Byte 2 of 0x001 |
| Throttle request widget | Yes | Sends 0x100 |
| Steering request widget | Yes | Sends 0x101 |
| Mode request widget | Yes | Sends 0x102 |

### 2.4 DEGRADED (system_state = 3)

| Element | Visible | Source |
|---------|---------|--------|
| All ACTIVE elements | Yes | Same as ACTIVE (§2.3) |
| **"DEGRADED MODE" indicator** | Yes — amber overlay | State value |
| Power limit indicator (40%) | Yes | Informational — STM32 enforces |

### 2.5 SAFE (system_state = 4)

| Element | Visible | Source |
|---------|---------|--------|
| **"SAFE MODE" banner** | Yes — prominent, full-width | State value |
| fault_flags detail | Yes | Byte 2 of 0x001 |
| Safety error code | Yes | 0x203 byte 2 |
| Wheel speeds | Yes (read-only) | 0x200 |
| Motor currents | Yes (read-only) | 0x201 |
| Temperatures | Yes (read-only) | 0x202 |
| Steering angle | Yes (read-only) | 0x204 |
| Throttle / steering / mode controls | **Disabled** | — |

### 2.6 ERROR (system_state = 5)

| Element | Visible | Source |
|---------|---------|--------|
| **"SYSTEM ERROR" banner** | Yes — red, full-screen overlay | State value |
| fault_flags detail | Yes | Byte 2 of 0x001 |
| Safety error code | Yes | 0x203 byte 2 |
| DIAG_ERROR details | Yes, if received | 0x300 bytes 0–1 |
| "Manual reset required" message | Yes | Static text |
| All telemetry gauges | **Frozen at last known value** | — |
| All controls | **Disabled** | — |

---

## 3. User Actions — Allowed vs. Blocked

| Action | BOOT | STANDBY | ACTIVE | DEGRADED | SAFE | ERROR |
|--------|------|---------|--------|----------|------|-------|
| View telemetry | ✗ | Partial | ✓ | ✓ | ✓ (read-only) | Frozen |
| Send throttle (0x100) | ✗ | ✗ | ✓ | ✓ (limited) | **✗** | **✗** |
| Send steering (0x101) | ✗ | ✗ | ✓ | ✓ (limited) | **✗** | **✗** |
| Send mode change (0x102) | ✗ | ✗ | ✓ | ✓ (limited) | **✗** | **✗** |
| Acknowledge fault overlay | ✗ | ✗ | ✗ | ✓ (dismiss banner, not fault) | ✓ (dismiss banner, not fault) | ✗ |
| Access engineering menu | ✗ | ✓ (see §6) | **✗** | **✗** | ✗ | ✓ (see §6) |
| Scroll / navigate screens | ✗ | ✓ | ✓ | ✓ | Limited | ✗ |

### Blocking Enforcement

- When a control is blocked, the HMI must **not transmit** the corresponding CAN command. Greying-out a widget is insufficient by itself; the CAN TX path must be gated on `system_state == 2` (ACTIVE) or `system_state == 3` (DEGRADED).
- The STM32 will independently reject commands received outside ACTIVE state, but the HMI must not rely on this as the sole guard.

---

## 4. Fault Overlays and Warnings

### 4.1 fault_flags Overlay (byte 2 of 0x001)

Each bit in `fault_flags` maps to a visual indicator that is overlaid on the current screen without replacing it.

| Bit | Mask | Flag | HMI Overlay |
|-----|------|------|-------------|
| 0 | 0x01 | FAULT_CAN_TIMEOUT | ⚠ "CAN LINK LOST" — red banner, top of screen |
| 1 | 0x02 | FAULT_TEMP_OVERLOAD | 🌡 "OVERTEMP" — amber icon on affected gauge |
| 2 | 0x04 | FAULT_CURRENT_OVERLOAD | ⚡ "OVERCURRENT" — amber icon on affected gauge |
| 3 | 0x08 | FAULT_ENCODER_ERROR | 🔧 "ENCODER FAULT" — steering gauge marked invalid |
| 4 | 0x10 | FAULT_WHEEL_SENSOR | 🔧 "WHEEL SENSOR FAULT" — speed gauge(s) marked invalid |
| 5 | 0x20 | FAULT_ABS_ACTIVE | ABS indicator illuminated (informational, not a fault) |
| 6 | 0x40 | FAULT_TCS_ACTIVE | TCS indicator illuminated (informational, not a fault) |
| 7 | 0x80 | (reserved) | No display. Ignore. |

#### Rules

- Overlays appear within one heartbeat cycle of the flag being set.
- Overlays disappear within one heartbeat cycle of the flag being cleared.
- Multiple overlays may be displayed simultaneously; they must not obscure each other.
- Bits 5 and 6 (ABS_ACTIVE, TCS_ACTIVE) are informational indicators, not fault warnings. They should use a distinct visual style (e.g., steady icon) separate from fault banners.

### 4.2 error_code Overlay (byte 2 of 0x203)

The `error_code` field in STATUS_SAFETY provides finer fault classification. If `error_code != 0`, the HMI must display a fault detail panel.

| Code | Name | HMI Text |
|------|------|----------|
| 0 | SAFETY_ERROR_NONE | (no overlay) |
| 1 | SAFETY_ERROR_OVERCURRENT | "Motor overcurrent detected" |
| 2 | SAFETY_ERROR_OVERTEMP | "Motor overtemperature detected" |
| 3 | SAFETY_ERROR_CAN_TIMEOUT | "CAN heartbeat lost" |
| 4 | SAFETY_ERROR_SENSOR_FAULT | "Sensor plausibility failure" |
| 5 | SAFETY_ERROR_MOTOR_STALL | "Motor stall (reserved)" |
| 6 | SAFETY_ERROR_EMERGENCY_STOP | "EMERGENCY STOP ACTIVE" |
| 7 | SAFETY_ERROR_WATCHDOG | "Watchdog reset occurred" |

#### Rules

- The fault detail panel must include the numeric code and the human-readable name.
- If `error_code` changes, the panel updates immediately; it does not queue.
- The EMERGENCY_STOP text (code 6) must be rendered in the largest available font with a red background.

### 4.3 DIAG_ERROR (0x300)

When a DIAG_ERROR frame is received, the HMI should log the following for later review:

| Field | Byte(s) | Usage |
|-------|---------|-------|
| error_code | 0 | Map to name (same table as §4.2) |
| subsystem | 1 | 0 = Global, 1 = Motor, 2 = Sensor, 3 = CAN |

The HMI may display a brief toast notification on receipt but must not block the primary screen.

---

## 5. Behavior on CAN Heartbeat Loss

The HMI monitors arrival of the STM32 heartbeat (CAN ID 0x001). If no frame with ID 0x001 is received for more than **250 ms**, the following sequence must execute:

### 5.1 Detection

- The HMI must maintain a local timer reset on every reception of 0x001.
- The timeout threshold is **250 ms**, matching the STM32's own ESP32 heartbeat timeout.

### 5.2 Immediate Actions (within one display frame of timeout)

1. Display a full-screen **"STM32 HEARTBEAT LOST"** overlay in red.
2. **Disable all CAN command transmission** (throttle, steering, mode). No 0x100, 0x101, or 0x102 frames may be sent.
3. Freeze all telemetry gauges at their last known values and mark them as **stale** (e.g., grey-out or dashed border).
4. Begin an audible and/or visual alarm (if hardware supports it).

### 5.3 While Heartbeat Is Missing

- The HMI must **continue sending its own heartbeat** (0x011) at the normal 100 ms interval. This allows the STM32 to detect the ESP32 is still alive if the CAN bus recovers in one direction.
- No user interaction may dismiss the overlay while the heartbeat is absent.

### 5.4 Recovery

- When 0x001 is received again, the HMI must:
  1. Clear the "HEARTBEAT LOST" overlay.
  2. Resume reading `system_state` from the newly received heartbeat.
  3. Transition to the screen dictated by the current `system_state` (§1).
  4. Re-enable CAN command transmission **only if** `system_state == ACTIVE (2)` or `system_state == DEGRADED (3)`.
  5. Unfreeze telemetry gauges and remove stale markers.

---

## 6. Hidden / Engineering Menus

### 6.1 Purpose

An engineering menu provides diagnostic data, CAN bus statistics, raw register values, and firmware information for development and maintenance only. It must never be accessible during normal vehicle operation.

### 6.2 Entry Conditions

| Condition | Required |
|-----------|----------|
| `system_state` must be STANDBY (1) **or** ERROR (5) | **Yes** |
| A specific multi-step gesture or key sequence must be performed | **Yes** |
| Vehicle wheel speed must be 0 on all four channels (0x200 all zeros) | **Yes** |
| No FAULT_CAN_TIMEOUT (bit 0 of fault_flags) active | **Yes** |

All four conditions must be satisfied simultaneously. If any condition ceases to be true while the menu is open, the menu must close immediately and the HMI must return to the screen dictated by `system_state`.

### 6.3 Explicitly Forbidden States

| system_state | Engineering Menu Access |
|--------------|------------------------|
| BOOT (0) | **Forbidden** — system not yet initialized |
| STANDBY (1) | Allowed (if all entry conditions met) |
| ACTIVE (2) | **Forbidden** — vehicle may be in motion; no engineering access permitted |
| DEGRADED (3) | **Forbidden** — vehicle is in degraded drive mode; no engineering access permitted |
| SAFE (4) | **Forbidden** — active fault must be resolved first |
| ERROR (5) | Allowed (if all entry conditions met) — for post-mortem diagnostics |

### 6.4 Engineering Menu Contents

The engineering menu may display the following read-only data:

- ESP32 firmware version, build date, uptime
- STM32 alive_counter trend (from 0x001 byte 0)
- Raw fault_flags history
- CAN bus error counters (TX error, RX error, bus-off count)
- Last N DIAG_ERROR frames received (0x300 log)
- Raw hex dump of most recent frame per CAN ID

### 6.5 Engineering Menu Restrictions

- The engineering menu must **never** transmit any CAN command (0x100, 0x101, 0x102).
- The engineering menu must **never** allow modification of safety parameters, CAN filter settings, or calibration values.
- The engineering menu must **never** allow the operator to override, mask, or dismiss active faults.
- If `system_state` transitions to ACTIVE (2) or DEGRADED (3) while the engineering menu is open, the menu must close within one heartbeat cycle.

---

## 7. Rules the HMI Must Never Violate

The following rules are absolute and apply regardless of software version, configuration, or operator action.

1. **The HMI must not send throttle (0x100), steering (0x101), or mode (0x102) commands when `system_state` is anything other than ACTIVE (2) or DEGRADED (3).**

2. **The HMI must not suppress, delay, or visually minimize fault overlays derived from `fault_flags` or `error_code`.**

3. **The HMI must not present controls that imply the operator can override a SAFE or ERROR state.** There is no "dismiss", "override", or "force active" button.

4. **The HMI must not assume a command was executed.** The CAN contract (§5 of CAN_CONTRACT_FINAL.md) states that every requested value must be verified through the corresponding status message. The HMI must display **actual** values from status messages, never the locally requested values, as the authoritative state.

5. **The HMI must not stop sending its heartbeat (0x011).** Heartbeat transmission at 100 ms must continue in every system state, including during fault conditions and heartbeat-loss scenarios.

6. **The HMI must treat any `system_state` value outside the defined range (0–5) as ERROR (5).**

7. **The HMI must disable all actuator commands within one display frame of detecting STM32 heartbeat loss (> 250 ms without 0x001).** Refer to §5.

8. **The HMI must not provide engineering menu access when `system_state` is ACTIVE (2), DEGRADED (3), or SAFE (4).** Refer to §6.3.

9. **The HMI must not modify, reinterpret, or extend the CAN message definitions.** All CAN IDs, payloads, and timing are governed by `docs/CAN_CONTRACT_FINAL.md` revision 1.0. Any change requires a new contract revision.

10. **The HMI must not store or cache `system_state` across power cycles.** On every boot, the HMI must start in the BOOT screen and wait for the first valid 0x001 frame before transitioning.

---

## References

| Document | Relevance |
|----------|-----------|
| `docs/CAN_CONTRACT_FINAL.md` rev 1.0 | Authoritative CAN message definitions, payloads, timing, and validation rules |
| `docs/SAFETY_SYSTEMS.md` | ABS/TCS algorithms, thermal/current protection, watchdog, fault management |
| `docs/CAN_PROTOCOL.md` | CAN bus parameters, timing, and filter configuration |
| `docs/ESP32_STM32_CAN_CONNECTION.md` | Physical layer and hardware wiring |
| Release `v1.0-stm32-safety-baseline` | Frozen firmware baseline referenced by this document |
