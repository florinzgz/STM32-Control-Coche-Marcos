# Persistent Steering Calibration

## Overview

The steering centering system uses an inductive proximity sensor
(LJ12A3) to detect the mechanical center of the steering rack at
startup.  A full left-right sweep at low PWM locates the center,
zeroes the encoder, and marks the steering as calibrated.

This feature **persists the calibrated center position to flash** so
that a centering sweep can be skipped on power-up when the steering
has not moved since the last shutdown.  This enables fast startup
while automatically re-calibrating if the wheels were moved while
power was off.

## Flash Storage

| Field                     | Type     | Description                        |
|---------------------------|----------|------------------------------------|
| `magic`                   | uint32   | `0x53544331` ("STC1")              |
| `encoder_count_at_center` | int32    | TIM2 count at calibrated center    |
| `validity_flag`           | uint8    | `0xA5` when valid                  |
| `reserved[3]`             | uint8[3] | Padding for alignment              |
| `checksum`                | uint32   | CRC32 of all preceding fields      |

**Location:** Flash page 126 (`0x0807E000`), 4 KB.
Separate from EPS parameters (page 127) to avoid mutual destruction
on erase.

### Write Conditions

Calibration is written to flash only when **all** conditions are met:

1. Centering has just finished successfully (`CENTERING_DONE`).
2. System is in `BOOT` or `STANDBY` (vehicle speed = 0 by design).
3. No safety error is active (`SAFETY_ERROR_NONE`).

## Boot Validation

At startup, before the main loop begins:

1. **Read flash** — verify magic word and CRC32.
2. **Read encoder** — get the current TIM2 counter value (hardware
   resets to 0 on power-on).
3. **Compare** — `|current_encoder − stored_center| ≤ 100 counts`
   (~7.5° of encoder shaft).
4. **Center sensor check** — read PB5 (GPIO pin level).  The
   inductive sensor pulls the line LOW when the screw is detected.
   Pin must read LOW (`GPIO_PIN_RESET`).

### Decision Table

| Flash valid? | Encoder within tolerance? | Center sensor active? | Result                      |
|:---:|:---:|:---:|-------------------------------------------|
| ✗ | — | — | Normal centering sweep                     |
| ✓ | ✗ | — | Normal centering sweep                     |
| ✓ | ✓ | ✗ | Normal centering sweep                     |
| ✓ | ✓ | ✓ | **Skip sweep → calibration restored**      |

## Safety Invariant

> **Flash data alone NEVER authorises ACTIVE.**

The physical center sensor must confirm plausibility at every boot.
This prevents the system from trusting a stale flash value if:

- The wheels were physically moved while power was off.
- The encoder or sensor hardware degraded.
- Flash data was corrupted beyond CRC detection capability.

The existing requirement that ACTIVE requires valid steering
calibration is unchanged.  LIMP_HOME continues to work without
calibration.

## Behaviour Summary

### Valid stored calibration

```
Power-on → BOOT → SteeringCal_ValidateAtBoot() passes
         → SteeringCentering_MarkRestoredFromFlash()
         → STANDBY → ACTIVE  (no sweep, fast startup)
```

### Invalid or missing calibration

```
Power-on → BOOT → SteeringCal_ValidateAtBoot() fails
         → Normal centering sweep (left → right → center found)
         → Calibration saved to flash
         → STANDBY → ACTIVE
```

### Centering failure

```
Power-on → BOOT → Validation fails → Centering sweep fails
         → CENTERING_FAULT → SAFETY_ERROR_CENTERING
         → SYS_STATE_DEGRADED (LIMP mode, drive-home capable)
```

## Thresholds

| Parameter                         | Value | Unit   | Rationale                                           |
|-----------------------------------|-------|--------|-----------------------------------------------------|
| `STEERING_CAL_TOLERANCE_COUNTS`   | 100   | counts | ~7.5° encoder shaft; covers drift, rejects movement |
| Flash page                        | 126   | —      | Separate from EPS params (page 127)                 |
| CRC polynomial                    | CRC32 | —      | Standard Ethernet CRC32 (0xEDB88320)                |

## Failure Cases

| Failure                                    | Effect                                   |
|--------------------------------------------|------------------------------------------|
| Flash write fails after centering          | Non-critical; next boot re-sweeps        |
| Flash corrupted (bad CRC / magic)          | Treated as no stored data; re-sweeps     |
| Encoder drift exceeds tolerance            | Re-sweeps (wheels likely moved)          |
| Center sensor disagrees                    | Re-sweeps (wheels likely moved)          |
| Center sensor hardware fault               | Re-sweeps; if centering also fails → DEGRADED |
| Flash page erased by external tool         | Re-sweeps                                |
| Multiple power cycles without movement     | Flash stays valid; always fast startup   |

## Files Modified

| File                              | Change                                      |
|-----------------------------------|---------------------------------------------|
| `Core/Inc/steering_cal_store.h`   | New — public API header                     |
| `Core/Src/steering_cal_store.c`   | New — flash read/write/validate             |
| `Core/Inc/steering_centering.h`   | Added `SteeringCentering_MarkRestoredFromFlash()` |
| `Core/Src/steering_centering.c`   | Save calibration after centering; restore API |
| `Core/Src/main.c`                 | Boot-time validation and restore logic      |
| `Makefile`                        | Added `steering_cal_store.c` to sources     |

## What Is NOT Changed

- Driving behaviour, torque math, PID, or safety limits.
- CAN protocol or ESP32 code.
- ACTIVE state still requires valid steering calibration.
- LIMP_HOME still works without calibration.
- Safety error codes and state machine transitions.
