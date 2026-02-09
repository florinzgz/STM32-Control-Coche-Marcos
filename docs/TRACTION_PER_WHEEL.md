# Per-Wheel Traction Control

## Summary

Per-wheel slip now limits **only the slipping wheel(s)**, aligned with
FULL-FIRMWARE (`abs_system.cpp`, `tcs_system.cpp`, `traction.cpp`) behavior.

## Old Behavior (global intervention)

| Event | Action |
|-------|--------|
| ABS active on **any** wheel | `Traction_SetDemand(0)` — **all** wheels cut to zero |
| TCS active on **any** wheel | `Traction_SetDemand(pedal / 2)` — **all** wheels halved |
| `Safety_ValidateThrottle` | Global override: ABS → 0 %, TCS → 50 % of request |

A single slipping wheel penalized the entire vehicle.

## New Behavior (per-wheel modulation)

| Event | Action |
|-------|--------|
| ABS active on wheel *i* | `wheel_scale[i] = 0.0` — only that wheel cut |
| TCS active on wheel *i* | `wheel_scale[i] = 1.0 − reduction` — only that wheel reduced |
| Non-slipping wheels | `wheel_scale[j] = 1.0` — full power maintained |
| **All 4** wheels slip | Global fallback via `Traction_SetDemand()` (last resort) |

`Traction_Update()` multiplies each wheel's base PWM by its `wheel_scale`
before writing to the timer compare register.

## ABS vs TCS

| | ABS | TCS |
|---|---|---|
| **Detects** | Wheel locking (speed << average) | Wheel spinning (speed >> average) |
| **Slip direction** | `(avg − wheel) / avg` | `(wheel − avg) / avg` |
| **Threshold** | 15 % (`abs_system.cpp`) | 15 % (`tcs_system.cpp`) |
| **Min speed** | 10 km/h | 3 km/h |
| **Intervention** | Full cut (`scale = 0.0`) on locked wheel | Progressive: 40 % initial, +5 %/cycle, max 80 % |
| **Recovery** | Immediate when slip clears | Gradual at 25 %/s (`recoveryRatePerSec`) |
| **Base firmware fn** | `ABSSystem::modulateBrake(wheel, demand)` | `TCSSystem::modulatePower(wheel, demand)` |

When both ABS and TCS apply to the same wheel, the **minimum** scale wins.

## Safety Fallback Logic

1. **Per-wheel intervention** — normal case; only slipping wheel(s) limited.
2. **All 4 wheels slip** — global `Traction_SetDemand()` reduces overall demand
   (ABS: demand = 0; TCS: demand × 0.2).
3. **Emergency stop / CAN timeout / watchdog / HARD fault** — existing
   `Safety_FailSafe()` / `Safety_EmergencyStop()` / `SYS_STATE_ERROR`
   transitions are **unchanged** and take absolute priority.
4. **DEGRADED mode** — `Safety_GetPowerLimitFactor()` still applies its 40 %
   cap on top of per-wheel scaling.

## Files Changed

| File | Change |
|------|--------|
| `Core/Inc/safety_system.h` | Added `wheel_scale[4]` to `SafetyStatus_t` |
| `Core/Src/safety_system.c` | Per-wheel ABS/TCS with progressive TCS reduction and recovery |
| `Core/Src/motor_control.c` | `Traction_Update()` applies `wheel_scale[]` per motor |

No changes to: CAN protocol, encoder logic, steering logic, service mode API,
`vehicle_physics.h` constants.
