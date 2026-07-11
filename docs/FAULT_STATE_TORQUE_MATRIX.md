# Fault → State → Torque → Speed → Movement → Recovery Matrix

**Scope:** Item 7 of the instrumentation-first PR. A single, firmware-traced reference table mapping
each fault class onto the resulting system state, the torque/power/speed limits that state imposes,
whether movement is permitted, and how (or whether) the system recovers.

**This document changes no behaviour.** Every value is traced to `Core/Inc/safety_system.h` and
`Core/Src/safety_system.c` at the revision of this PR. Nothing is invented; where a limit is a compile
-time constant the macro name is given so it can be re-verified.

---

## 1. System states

Source: `safety_system.h:100-110` (`SystemState_t`).

| State | Enum | Meaning | Movement? |
|-------|------|---------|-----------|
| BOOT | 0 | Power-on, peripherals initialising | No |
| STANDBY | 1 | Ready, waiting for ESP32 heartbeat | No |
| ACTIVE | 2 | Normal operation — CAN commands accepted | **Yes** |
| DEGRADED | 3 | Limp/degraded — commands still accepted, limited | **Yes (limited)** |
| SAFE | 4 | Hardware danger — actuators inhibited | No |
| ERROR | 5 | Unrecoverable fault — power-down required | No |
| LIMP_HOME | 6 | CAN-loss degraded — minimal local-pedal drive | **Yes (walking pace)** |

Movement authority is confirmed by `safety_system.c:688-690` (traction allowed only in
ACTIVE / DEGRADED / LIMP_HOME).

---

## 2. Per-state limit factors

Sources: `Safety_GetPowerLimitFactor` (`safety_system.c:709`), `Safety_GetTractionCapFactor`
(`:749`), `Safety_GetSteeringLimitFactor` (`:729`), and the `DEGRADED_*`/`LIMP_HOME_*` macros
(`safety_system.h:245-272`).

| State / level | Power factor | Traction cap | Steering assist | Speed cap | Notes |
|---------------|-------------|--------------|-----------------|-----------|-------|
| ACTIVE | 1.00 | 1.00 | 1.00 | none | Full performance |
| DEGRADED L1 | 0.70 | 0.80 | 0.85 | — | `DEGRADED_L1_*_PCT` — minor sensor fault |
| DEGRADED L2 | 0.50 | 0.60 | 0.70 | — | `DEGRADED_L2_*_PCT` — thermal warning |
| DEGRADED L3 | 0.40 | 0.50 | 0.60 | — | `DEGRADED_L3_*_PCT` — persistent anomaly; 0.40 = `DEGRADED_POWER_LIMIT_PCT` |
| LIMP_HOME | 0.20 | 0.20 | 1.00 | **5 km/h** | `LIMP_HOME_TORQUE_LIMIT_FACTOR`, `LIMP_HOME_SPEED_LIMIT_KMH`; steering kept full; ramp 10 %/s (`LIMP_HOME_RAMP_RATE_PCT_PER_S`); no torque vectoring |
| SAFE | 0.00 | 0.00 | 0.00 | 0 | Commands rejected upstream; actuators inhibited |
| ERROR | 0.00 | 0.00 | 0.00 | 0 | Actuators + relays powered down; power cycle required |
| BOOT / STANDBY | 0.00 | 0.00 | 0.00 | 0 | Not yet operational |

The internal `DEGRADED_L1/L2/L3` levels all report as the single `SYS_STATE_DEGRADED` over CAN — no
contract change (`safety_system.h:168-176`).

---

## 3. Fault → state → recovery matrix

Each row is a fault class as handled in `safety_system.c`. "Recovery" describes the exact firmware
condition that clears the fault; unless stated, recovery returns to ACTIVE only after the underlying
condition clears **and** a stability window elapses.

| Fault (error code) | Trigger | Resulting state | Torque/limit effect | Movement | Recovery condition |
|--------------------|---------|-----------------|---------------------|----------|--------------------|
| **Overcurrent** (1) | `amps > MAX_CURRENT_A` (or batt `MAX_CURRENT_BATT_A`), or NaN/Inf | 1 event → DEGRADED L1; ≥2 consecutive → DEGRADED L3; ≥3 (`CONSECUTIVE_ERROR_THRESHOLD`) → **SAFE** | L1/L3 power+traction clamp; SAFE = 0 | Limited until SAFE | Counter decays after 1 s clean (`:1769`); error cleared → DEGRADED→ACTIVE (`:1774`) |
| **Overtemp warning** (2) | `temp > 80 °C` (`TEMP_WARNING_C`) | DEGRADED **L2** | Power 0.50 / traction 0.60 | Limited | All temps < 75 °C (5 °C hysteresis) → clear (`:1830`) |
| **Overtemp critical** (2) | `temp > 90 °C` (`TEMP_CRITICAL_C`) or NaN/Inf | **SAFE** | 0 | No | Temp back under hysteresis, then DEGRADED→ACTIVE path |
| **CAN timeout** (3) | ESP32 heartbeat lost | **LIMP_HOME** (from ACTIVE/DEGRADED, `:1868`) | 20 % torque, 5 km/h, full steering | **Yes, walking pace** | Heartbeats restored + stable `RECOVERY_HOLD_MS` (500 ms) → ACTIVE (`:1937`) |
| **CAN bus-off** (13) | FDCAN bus-off detected | **LIMP_HOME** | as CAN timeout | Yes, walking pace | Peripheral RUNNING **and** sustained heartbeats over `CAN_BUSOFF_RECOVERY_WINDOW_MS` (1000 ms) — see `busoff_recovery.c` (Item 3) |
| **Battery UV warning** (9) | `V < 20.0 V` (`BATTERY_UV_WARNING_V`) | DEGRADED (`:2665`) | derate | Limited | Voltage recovers; DEGRADED→ACTIVE |
| **Battery UV critical** (10) | `V < 18.0 V` (`BATTERY_UV_CRITICAL_V`) | **SAFE** (`:2512,2551`) | 0 | No | `V ≥ 18.5 V` (critical + hyst) stable for `BATTERY_UV_RECOVERY_STABLE_MS` (2000 ms) → clear (`:2578`) |
| **Battery OV warning** (14) | `V > 30.0 V` | DEGRADED | derate | Limited | Voltage back in range |
| **Battery OV critical** (15) | `V > 35.0 V` | **SAFE** | 0 | No | Voltage back in range + stability window |
| **I²C failure** (11) | TCA9548A mux fails to ACK | **SAFE** | 0 | No | Bus recovers; latch cleared per recovery window (`:2482`) — see Error Code 11 in `SENSOR_INTERFACE.md` |
| **Obstacle** (12) | ToF/LiDAR distance below threshold | Forward demand scaled → 0 (obstacle_scale); emergency/CAN-timeout path may set error | Forward torque blocked; reverse allowed | Forward blocked | Distance ≥ `OBSTACLE_RECOVERY_MM` (750 mm) |
| **Sensor/wheel fault** (4) | Wheel/encoder plausibility fault (powertrain-gated, `WHEEL_FAULT_DEBOUNCE_MS`) | DEGRADED | per-level clamp | Limited | Fault debounce clears; DEGRADED→ACTIVE |
| **Relay open** (16) | Insufficient motor current vs. relay-enabled | DEGRADED/SAFE | per handling | Limited/No | Sustain healthy `RELAY_CHK_RECOVERY_MS` (1500 ms) |
| **Emergency stop** (6) | `Safety_EmergencyStop()` | **ERROR** (`:3066`) | Relays powered down, MOE off | No | **None in software — power cycle required** |
| **Watchdog** (7) | IWDG timeout | MCU reset → BOOT | — | No | Full reboot; reset cause logged |

---

## 4. Recovery windows summary

| Window macro | Value | Applies to |
|--------------|-------|-----------|
| `RECOVERY_HOLD_MS` | 500 ms | LIMP_HOME → ACTIVE, generic clean-operation debounce |
| `CAN_BUSOFF_RECOVERY_WINDOW_MS` | 1000 ms | Bus-off recovery (Item 3) |
| `BATTERY_UV_RECOVERY_STABLE_MS` | 2000 ms | Battery UV-critical → clear |
| `RELAY_CHK_RECOVERY_MS` | 1500 ms | Relay-open hysteresis |
| Overcurrent decay | 1000 ms clean | Consecutive-error counter reset |
| Temp hysteresis | 5 °C below `TEMP_WARNING_C` | Overtemp clear |

**Key safety principle (traced, unchanged):** a *degradable* fault (single overcurrent, thermal
warning, UV warning, CAN loss) keeps the vehicle **moving under a clamp** so it can be driven to
safety, whereas a *dangerous* fault (critical overtemp, critical battery, I²C failure, ≥3 consecutive
overcurrent, emergency stop) drives torque/PWM to **zero** via SAFE/ERROR. The MOTION_INHIBIT_REASON
diagnostic (0x315, Item 1) makes the exact clamp/zeroing point observable end-to-end.

---

## 5. Cross-references

- `Core/Inc/safety_system.h`, `Core/Src/safety_system.c` — the authoritative source for every value here.
- `docs/CAN_CONTRACT_FINAL.md` — 0x315 DIAG_MOTION_INHIBIT (Item 1), 0x300 status state field.
- `docs/STM32_BOOT_TIME_VS_IWDG.md` — watchdog window (Item 6).
- `docs/SENSOR_INTERFACE.md` — I²C / Error Code 11.
