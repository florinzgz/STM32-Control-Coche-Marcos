# CAN Bus Contract — FINAL

**Revision:** 1.17
**Status:** ACTIVE — Added MOTION_INHIBIT_REASON instrumentation diagnostic 0x315 (additive)
**Date:** 2026-07-11
**Scope:** CAN communication between STM32G474RE (safety authority) and ESP32-S3 (HMI)

Any change to this contract requires a new numbered revision and a corresponding firmware release.

**Change log:**
- **1.0** (2025-06-15): Initial frozen contract. Baseline safety system.
- **1.1** (2026-02-13): Added obstacle CAN IDs (0x208, 0x209). Added CAN RX filter bank 3. Added `SAFETY_ERROR_OBSTACLE` (code 12). Updated RX filter table.
- **1.2** (2026-02-13): Integration audit corrections. Heartbeat byte 3 documented as `error_code` (matches code). Added STATUS_BATTERY (0x207) payload definition §4.13. Fixed speed plausibility threshold (25 km/h, matches code). Renumbered §4.13–§4.16. Added fault_flags bit 7 (FAULT_CENTERING).
- **1.3** (2026-02-13): Added CMD_ACK (0x103) command acknowledgment message (Phase 13). STM32 sends ACK after safety validation for CMD_MODE (0x102) and SERVICE_CMD (0x110). Added §3.5, §4.17. Added ACK_TIMEOUT_MS (200 ms). Backward-compatible — no existing IDs or payloads changed.
- **1.4** (2026-05-01): Added DWT-debounce EMI diagnostic counters: `DIAG_DEBOUNCE` (0x306, DLC 8, 1000 ms) and `DIAG_DEBOUNCE_STEER` (0x307, DLC 4, 1000 ms). Purely additive, STM32 → ESP32, diagnostic-only — no control / safety path consumes these values. ESP32 firmware without DEBOUNCE_DIAG submenu silently ignores the new IDs (backward-compatible).
- **1.5** (2026-06-01): Added `DIAG_I2C` (0x309, DLC 5, 1000 ms) I2C topology diagnostic: TCA9548A (0x70) presence + per-channel INA226 (0x40) health mask + fail/recovery counters. Surfaced on the HMI Safe Mode screen so a missing mux can be told apart from a dead INA226. Purely additive, STM32 → ESP32, diagnostic-only — no control / safety path consumes it (backward-compatible).
- **1.6** (2026-06-01): Added CAN/I2C delivery-observability diagnostics (audit "0x309: NO DATA"): `DIAG_CAN_META` (0x30A, DLC 8, 1000 ms — 0x309 call/tick/tx-ok/tx-err/fifo-drop counters), `DIAG_I2C_SCAN` (0x30B, DLC 8, on-demand) and `DIAG_FDCAN` (0x30C, DLC 6, on-demand) emitted after `SERVICE_CMD` action `0xF6` (`SERVICE_ACTION_I2C_SERVICE`). Added a `HAL_FDCAN_GetTxFifoFreeLevel()` guard in `TransmitFrame()` that counts FIFO-full drops instead of silently failing. Purely additive, STM32 → ESP32, diagnostic-only — no control / safety path consumes these values (backward-compatible).
- **1.7** (2026-06-02): Phase-based I2C diagnostics. Extended `DIAG_I2C` (0x309) to DLC 6 with byte5 = `ina_expected_mask` (bit i = channel i's power branch is energised this phase). STM32 now only counts I2C timeouts toward the bus-fault/recovery/Error-Code-11 logic for INA226 channels whose branch should be powered, so the motor INAs (ch0..3, wired after the traction relay) and steering INA (ch5) no longer trigger a false Code 11 in SAFE/STANDBY. Battery INA (ch4) stays always-expected/obligatory. HMI Safe Mode shows un-powered branches as **WAIT PWR** (cyan) instead of FAIL (red). Purely additive, STM32 → ESP32 — DLC-5 consumers ignore byte5 (backward-compatible).
- **1.8** (2026-06-11): Added configurable per-gear traction power limits. New `SERVICE_CMD` action `0xF7` (`SERVICE_ACTION_GEAR_LIMITS`) with sub-opcodes `SET_D2/D1/R` (0x01–0x03, byte2 = percent), `SAVE` (0x04), `RESET_DEFAULTS` (0x05) and `QUERY` (0x06); replies on the existing `CMD_ACK` (0x103, byte0 = 0x10). New telemetry `DIAG_GEAR_LIMITS` (0x30D, DLC 8, on-demand burst after QUERY) reports active + pending D2/D1/R limits and flags. The STM32 owns range validation (D2 30–100, D1 20–100, R 10–60), a STANDBY-only safety gate on all mutating ops, and flash persistence (page 122, magic+CRC32, mirroring the pedal-cal store). Purely additive, bidirectional config/diagnostic — firmware without the GEAR LIMITS screen ignores 0x30D and never sends 0xF7 (backward-compatible). The shipped reverse default is 60 % (`GEAR_POWER_REVERSE_PCT = 0.60`); the request's "R = 30 %" is achievable via the editor but is **not** the firmware default.
- **1.9** (2026-06-11): Extended GEAR LIMITS with a per-gear **acceleration response profile** reusing the same infrastructure (no new CAN ID, same `0xF7`/`0x30D`/page 122). New sub-opcodes `SET_D2/D1/R_RESPONSE` (0x07–0x09, byte2 = percent). `0x30D` now carries two interleaved frame kinds discriminated by byte0 **bit4** (0=POWER, 1=RESPONSE); a QUERY emits both. The response factor is applied in `Traction_SetDemand()` after EMA#2 and before the global ramp limiter, to **positive demand only**, clamped ≤ 100 % so it can only soften (never amplify) demand — it does not touch ABS/TCS/dynamic-braking/SAFE/LIMP. Response ranges D2 50–100, D1 30–100, R 20–80; defaults D2 100 / D1 70 / R 40. Flash slot upgraded to v2 (`"GLM2"`, 16 bytes, same CRC offset); v1 (`"GLM1"`, power-only) slots are migrated safely on read (power kept, response defaults applied, rewritten as v2 only on next SAVE). Purely additive and backward-compatible.
- **1.10** (2026-06-11): Added the **PB5 + encoder-Z dual steering-center reference** diagnostic. PB5 (LJ12A3 inductive sensor) remains the **primary** physical/safety center reference; the encoder **Z** (index) pulse on PB4 is a **secondary** precision/verification reference and can **never** center on its own — a Z pulse seen without PB5 confirmation is **not** a center. New telemetry `DIAG_STEERING_Z` (0x30E, DLC 8, on-demand burst after QUERY) reports PB5 live state, Z status, Z pulse count, last Z position, Z↔center offset, last Z error and the active tolerance. New `SERVICE_CMD` action `0xF8` (`SERVICE_ACTION_STEERING_Z`) with sub-opcodes `QUERY` (0x01, emits the 0x30E burst), `CALIBRATE` (0x02, recompute + store the Z↔center offset — **only accepted while PB5 reads center**) and `CLEAR` (0x03, invalidate the stored Z calibration); replies on the existing `CMD_ACK` (0x103, byte0 = 0x10). The STM32 owns all validation, the BOOT/STANDBY-only gate on CALIBRATE/CLEAR, the "PB5 must confirm center" gate on CALIBRATE, and flash persistence (steering-cal page, format v2 with safe v1→v2 migration). A Z fault never forces SAFE — it is diagnostic/warning only. Purely additive, bidirectional config/diagnostic — firmware without the STEERING Z screen ignores 0x30E and never sends 0xF8 (backward-compatible).
- **1.11** (2026-06-11): Added runtime-tunable **drive tuning** and **battery limits** stores. New `SERVICE_CMD` actions `0xFA` (`SERVICE_ACTION_DRIVE_TUNING`) and `0xFB` (`SERVICE_ACTION_BATTERY_LIMITS`), each with `SET`/`SAVE`/`RESET`/`QUERY` sub-ops (STANDBY-gated except QUERY). New field-stream telemetry `DIAG_DRIVE_TUNING` (0x310, DLC 8) and `DIAG_BATTERY_LIMITS` (0x311, DLC 8), emitted as on-demand bursts after QUERY (one frame per field). Defaults reproduce the original firmware behaviour exactly; flash pages 121 (`DTN1`) and 120 (`BAT1`). Purely additive, bidirectional config/diagnostic — see §4.20/§4.21 (backward-compatible).
- **1.12** (2026-07-05): Heartbeat / boot-reset / delivery observability (PR #410). New telemetry `DIAG_BOOT_RESET` (0x312, DLC 8, 1000 ms): bytes0-3 STM32 `HAL_GetTick()` uptime u32 LE, byte4 RCC reset-cause bitmask (POWERON/SOFTWARE/IWDG/WWDG/BROWNOUT/PIN), byte5 CAN software TX-queue depth (now), byte6 TX-queue depth high-water, byte7 reserved — lets the HMI tell an IWDG/brownout reset apart from a lost heartbeat and prove the heartbeat is never sitting behind a TX backlog. `DIAG_I2C` (0x309) extended to **DLC 8** with byte6 = `i2c_last_read_ms` (duration of the last `Current_ReadAll()` in ms, saturated at 255 = "255+") and byte7 reserved. `DIAG_CAN_META` (0x30A) byte7 now packs `bit0 = fdcan_init_ok` and `bits 7:1 = hb_tx_err` (heartbeat TX failure count, saturated at 127). ESP32 `decodeBootReset()` accepts DLC ≥ 5 (uptime/reset) and DLC ≥ 7 (txQ), dropping DLC < 5; `decodeI2cDiag()` reads byte6 only when DLC ≥ 7. The STM32 enqueues the 0x001 heartbeat first each loop so diagnostics never delay it. Purely additive, STM32 → ESP32, diagnostic-only — no control / safety path consumes these values; short-DLC legacy frames stay backward-compatible.
- **1.13** (2026-07-05): Added per-wheel speed-sensor fault-reason diagnostic `DIAG_WHEEL_SENSOR` (0x313, DLC 8, 1000 ms). Bytes 0-3 carry the `WheelDiag_t` reason code (0-8) for FL/FR/RL/RR, byte4 a reserved STEER/CENTER reason (currently 0), byte5 a per-channel GPIO-level mask, byte6 the latched service-mode fault mask, byte7 a flags nibble (powertrain_engaged / manual_movement / wheel_fault_debouncing / wheel_fault_latched) + a 4-bit sequence counter. Lets the HMI name **which** wheel is silent and **why** (NO_PULSE / STUCK_HIGH / STUCK_LOW / MISMATCH / IMPOSSIBLE_RATE vs. the non-fault MANUAL_MOVEMENT / DISABLED_STATE / OK), so a `SENSOR_FAULT` (0x10) is no longer an unlabelled event and an expected hand-spin is visibly distinct from a real fault under traction. ESP32 `decodeWheelSensorDiag()` accepts DLC ≥ 7 (reasons + masks) and reads byte7 only when DLC 8; DLC < 7 is counted and dropped. The wheel-diagnostic reason codes MUST mirror `WheelDiag_t` (Core/Inc/safety_system.h) + the UNKNOWN wire code (8). Purely additive, STM32 → ESP32, diagnostic-only — no control / safety path consumes it; the wheel-fault safety logic itself is unchanged (backward-compatible).
- **1.15** (2026-07-07): Added per-wheel **VALID (accepted) pulse-count** diagnostic `DIAG_WHEEL_PULSES` (0x314, DLC 8, 1000 ms). Bytes 0-1/2-3/4-5/6-7 carry the FL/FR/RL/RR accepted-edge counts (uint16 LE, saturated 0xFFFF) from `Wheel_GetPulseCount()` — the edges that PASSED the DWT 200 µs pre-filter and drive the speed/distance estimate, as opposed to the REJECTED (bounce/EMI) counts already reported by 0x306. Sent from the same `CAN_SendDebounceDiag()` 1 Hz cadence. The ESP32 `DEBOUNCE / CAN DIAG` submenu now shows both columns side by side (`VALID PULSES | REJECTED PULSES`) so the operator can confirm each wheel counts equally (a full wheel turn ≈ 6 valid pulses). ESP32 `decodeWheelPulses()` accepts DLC ≥ 8. Purely additive, STM32 → ESP32, diagnostic-only — no control / safety path consumes it (backward-compatible).
- **1.16** (2026-07-11): **CAN bus-off recovery is now gated on sustained heartbeats, not merely on the FDCAN peripheral returning to RUNNING.** Previously `CAN_CheckBusOff()` cleared `SAFETY_ERROR_CAN_BUSOFF` and reset the retry counter the instant `HAL_FDCAN_Start()` succeeded after a re-init; with a persistent physical fault (short, stuck node, wiring) the controller would immediately go bus-off again, the counter would keep resetting to 0, and the system would loop forever without ever reaching `CAN_BUSOFF_MAX_RETRIES` or declaring a stable recovery. The recovery state machine now has three phases: HEALTHY → RECOVERING (re-init at `CAN_BUSOFF_RETRY_INTERVAL_MS`, bounded by `CAN_BUSOFF_MAX_RETRIES`) → **PROBATION** (peripheral RUNNING, but the fault stays latched and no further re-init is attempted until valid ESP32 heartbeats are present **continuously** for `CAN_BUSOFF_RECOVERY_WINDOW_MS` = 1000 ms; a heartbeat gap or a fresh bus-off resets the window, and a fresh bus-off during probation does **not** reset the retry counter). The confirmation-window logic lives in a pure, host-unit-tested module (`busoff_recovery.c/.h`, `test_busoff_recovery.c`). No CAN wire format changes — this only affects **when** the internal `SAFETY_ERROR_CAN_BUSOFF` fault clears (backward-compatible on the bus).
- **1.17** (2026-07-11): Added the **MOTION_INHIBIT_REASON** instrumentation diagnostic `DIAG_MOTION_INHIBIT` (0x315, DLC 8, 100 ms). Byte0-1 carry a 16-bit reason bitfield (`MOTION_INHIBIT_*`: STATE_SAFE/STATE_ERROR/POWER_NOT_READY/GEAR_PARK/GEAR_NEUTRAL/NO_DEMAND/DEMAND_ZEROED/OBSTACLE_BLOCK/PWM_ZERO/TORQUE_LIMITED), byte2 system state, byte3 gear, byte4 operator demand % (int8), byte5 effective demand % after scaling (int8), byte6 max final PWM duty % (0..100), byte7 flags (bit0 power_ready, bit1 obstacle_forward_blocked, bits4-7 degraded level). It answers "why is the vehicle not moving?" end-to-end (pedal → demand → torque limiter → PWM → relay/driver-enable), and specifically distinguishes "no demand" from "demand zeroed before PWM" and "demand survived but final PWM is zero" (the *DEGRADED-40 %-with-zero-PWM* signature). The classification lives in a pure, host-unit-tested module (`motion_inhibit.c/.h`, `test_motion_inhibit.c`) and is emitted by `CAN_SendMotionInhibit()`; ESP32 `decodeMotionInhibit()` accepts DLC ≥ 8 (`test_motion_inhibit_decode.cpp`). **Instrumentation only** — no control or safety policy changed; the traction/safety pipeline is untouched (backward-compatible, STM32 → ESP32).
- **1.18** (2026-07-11): **ESP32 TWAI (CAN) bus-off recovery now mirrors the STM32 stable-heartbeat policy** — the physical `BUS_OFF → RECOVERING → STOPPED → RUNNING → BUS_OFF` oscillation captured on the vehicle was an **ESP32 TWAI** fault, *not* the STM32 FDCAN side (rev 1.16 fixed the STM32 side only). Previously `main.cpp` reset `busOffRecoveryCount` the instant the controller reported `STOPPED→RUNNING` or was merely observed `RUNNING`, so a bus that oscillated back to BUS_OFF kept clearing the counter forever, and after a fixed 10 attempts it gave up permanently (dead node after a transient disconnection). The decision is now delegated to a pure, host-unit-tested FSM (`esp32/src/twai_recovery.h`, `test_twai_recovery.cpp`): RUNNING is treated as **PROBATION** and recovery is confirmed only after advancing STM32 heartbeats are sustained continuously for a stable window (`stm32IsAlive`); the lifetime BUS_OFF event count is preserved; retries are fast-and-bounded then fall back to a slow **unbounded** backoff so the node always recovers after a physical reconnection. No CAN wire format changes — this only affects **when** the ESP32 declares its own TWAI controller recovered (backward-compatible on the bus).
- **1.19** (2026-07-11): **MOTION_INHIBIT_REASON (0x315) observability extended** (additive, backward-compatible). Six new reason bits were added in the previously-unused high half of byte0-1: `STARTUP_INHIBIT` 0x400, `PEDAL_FAULT` 0x800 (pedal implausible/contradictory), `SAFETY_SCALE_ZERO` 0x1000 (per-wheel ABS/TCS/driver-enable/safety scale collapsed to 0), `BATTERY_CUTOFF` 0x2000 (battery UV/OV limit or cutoff), `THERMAL_OVERCURRENT` 0x4000, `SERVICE_DISABLED` 0x8000 (traction relay module disabled via service mode). byte7 bits2-3 now carry the relay-sequence phase (0 idle / 1 in-progress / 2 complete). **These reflect the COMMANDED GPIO/sequencer state only — the firmware has NO physical relay-contact feedback and does not claim it.** All new inputs are read straight from existing accessors (`Startup_IsInhibited`, `Pedal_IsPlausible`, `Safety_GetError`, `ServiceMode_IsEnabled`, `safety_status.wheel_scale[]`, `Relay_IsSequenceInProgress`); the classifier stays a pure host-tested module (`test_motion_inhibit.c`, `test_motion_inhibit_decode.cpp`). **Instrumentation only** — no control or safety policy changed; legacy decoders that only read the low reason bits and byte7 bits0-1/4-7 are unaffected (STM32 → ESP32).


---

## 1. Roles and Authority

| Role | Device | Responsibility |
|------|--------|----------------|
| Safety authority | STM32G474RE | Validates, clamps, rate-limits, or rejects every command. Controls actuators, relays, and power. Monitors sensors. Enforces fail-safe. |
| HMI (intent sender) | ESP32-S3 | Sends operator intent only. Has no direct control over actuators. Must not assume any command will be executed as sent. |

The STM32 decides what is allowed. The ESP32 requests; the STM32 disposes.

---

## 2. CAN Bus Parameters

| Parameter | Value | Source |
|-----------|-------|--------|
| Standard | CAN 2.0A (classic CAN) | `FDCAN_FRAME_CLASSIC`, `FDCAN_CLASSIC_CAN` in `can_handler.c` |
| Identifier type | Standard 11-bit | `FDCAN_STANDARD_ID` in `can_handler.c` |
| Bitrate | 500 kbps | `MX_FDCAN1_Init` in `main.c`: 170 MHz / (17 × (1 + 14 + 5)) = 500,000 bps |
| Prescaler | 17 | `NominalPrescaler = 17` |
| Time Seg 1 | 14 TQ | `NominalTimeSeg1 = 14` |
| Time Seg 2 | 5 TQ | `NominalTimeSeg2 = 5` |
| Sync Jump Width | 1 TQ | `NominalSyncJumpWidth = 1` |
| Total quanta per bit | 20 (1 + 14 + 5) | — |
| FDCAN clock source | APB1 = 170 MHz (no divider) | `FDCAN_CLOCK_DIV1` in `main.c` |
| Auto retransmission | Enabled | `AutoRetransmission = ENABLE` |
| FD bit-rate switch | Off | `FDCAN_BRS_OFF` in `can_handler.c` |
| Max payload | 8 bytes | Classic CAN frame limit |
| Topology | Point-to-point (ESP32-S3 ↔ STM32G474RE) | — |
| Pins | PA11 (FDCAN1_RX), PA12 (FDCAN1_TX) — AF9 | `project_config.h` |
| Error handling | Hardware auto-retransmission; protocol exception disabled (`ProtocolException = DISABLE`) | `main.c` |

### RX Filter Policy

The STM32 uses a single MASK-based accept-all hardware filter (mask = 0x000, all bits don't-care) that routes every standard CAN ID into RXFIFO0. Non-matching standard and extended IDs are also accepted into RXFIFO0 via `HAL_FDCAN_ConfigGlobalFilter()`. Remote frames are rejected at the hardware level.

| Filter | Type | ID / Mask | Effect | Destination |
|--------|------|-----------|--------|-------------|
| 0 | Mask | ID=0x000, Mask=0x000 | Accept ALL standard 11-bit IDs | RXFIFO0 |
| Global (std) | Accept | — | Non-matching standard IDs accepted | RXFIFO0 |
| Global (ext) | Accept | — | Non-matching extended IDs accepted | RXFIFO0 |
| Global (remote) | Reject | — | All remote frames rejected | Discarded |

> **Note:** Filtering is performed in software by `CAN_ProcessMessages()`, which uses a switch/case on the received ID. Only known message IDs (0x011, 0x100–0x102, 0x110, 0x120, 0x208–0x209) trigger application logic; all other IDs are silently discarded. This accept-all hardware strategy ensures the FDCAN message-RAM filter element is always valid and the peripheral can leave INIT mode cleanly after `HAL_FDCAN_Start()`.

Source: `CAN_ConfigureFilters()` in `Core/Src/can_handler.c`

---

## 3. Message List

### 3.1 ESP32 → STM32 (Commands / Heartbeat)

| CAN ID | Name | DLC | Rate | Description | Source file |
|--------|------|-----|------|-------------|-------------|
| 0x011 | HEARTBEAT_ESP32 | 1 | 100 ms | ESP32 alive signal. Byte 0: rolling counter. STM32 validates counter advancement — frozen counter triggers CAN timeout. | `can_handler.c` |
| 0x100 | CMD_THROTTLE | 1 | 50 ms | Throttle request (percent) | `can_handler.c` |
| 0x101 | CMD_STEERING | 2 | 50 ms | Steering angle request (raw units) | `can_handler.c` |
| 0x102 | CMD_MODE | 2 | On-demand | Drive mode + gear request (byte0=mode flags, byte1=gear) | `can_handler.c` |
| 0x110 | SERVICE_CMD | 2 | On-demand | Service mode module control command | `can_handler.c` |
| 0x120 | CMD_LED | 2 | On-demand | LED relay control (byte0=front, byte1=rear) | `can_handler.c` |
| 0x208 | OBSTACLE_DISTANCE | 5 | 66 ms | Obstacle distance + zone + sensor health + rolling counter | `can_handler.c` |
| 0x209 | OBSTACLE_SAFETY | 4 | 100 ms | Obstacle safety state (informational): zone, sensor status, stuck flag, reserved | `can_handler.c` |

### 3.2 STM32 → ESP32 (Status / Heartbeat)

| CAN ID | Name | DLC | Rate | Description | Source file |
|--------|------|-----|------|-------------|-------------|
| 0x001 | HEARTBEAT_STM32 | 6 | 100 ms | System alive, state, fault flags, error code, status flags, relay status | `can_handler.c` |
| 0x200 | STATUS_SPEED | 8 | 100 ms | Four wheel speeds | `can_handler.c`, `main.c` |
| 0x201 | STATUS_CURRENT | 8 | 100 ms | Four motor currents | `can_handler.c`, `main.c` |
| 0x202 | STATUS_TEMP | 5 | 1000 ms | Five temperature sensors | `can_handler.c`, `main.c` |
| 0x203 | STATUS_SAFETY | 5 | 100 ms | ABS/TCS active flags, error code, system state, rx_errors | `can_handler.c`, `main.c` |
| 0x204 | STATUS_STEERING | 3 | 100 ms | Actual steering angle and calibration flag | `can_handler.c`, `main.c` |
| 0x205 | STATUS_TRACTION | 4 | 100 ms | Per-wheel traction scale (ABS/TCS) | `can_handler.c`, `main.c` |
| 0x206 | STATUS_TEMP_MAP | 5 | 1000 ms | Explicit temperature sensor mapping (FL/FR/RL/RR/AMB) | `can_handler.c`, `main.c` |
| 0x207 | STATUS_BATTERY | 4 | 100 ms | Battery bus current and voltage (INA226, 24 V bus) | `can_handler.c`, `main.c` |
| 0x20A | STATUS_LIGHTS | 2 | 1000 ms | LED relay state (byte0=front, byte1=rear) | `can_handler.c`, `main.c` |
| 0x20B | STATUS_PEDAL | 4 | 100 ms | Hall pedal telemetry: byte0=position %, byte1=fault flags (bit0 plausible, bit1 dual-sample contradiction), byte2-3=raw ADC (u16 LE). Telemetry-only, drives HMI THROTTLE bar + pedal-fault diagnostic. ESP32 reads byte1-3 only at DLC≥4 (legacy DLC 1 still valid). | `can_handler.c`, `main.c` |

### 3.3 Bidirectional (Diagnostic)

| CAN ID | Name | DLC | Rate | Description | Source file |
|--------|------|-----|------|-------------|-------------|
| 0x300 | DIAG_ERROR | 2 | On-demand | Error code and subsystem identifier | `can_handler.c` |
| 0x306 | DIAG_DEBOUNCE | 8 | 1000 ms | DWT-debounce filtered counts: 4× wheel u16 LE (FL,FR,RL,RR) saturated to 0xFFFF | `can_handler.c`, `sensor_manager.c` |
| 0x307 | DIAG_DEBOUNCE_STEER | 4 | 1000 ms | DWT-debounce filtered count for steering center: u32 LE | `can_handler.c`, `sensor_manager.c` |
| 0x309 | DIAG_I2C | 8 | 1000 ms | I2C topology diag: byte0=mux_present, byte1=ina_ok_mask (bit0..5=FL,FR,RL,RR,BAT,STEER), byte2=fail_count, byte3=recovery_attempts, byte4=flags(bit0=ever_ok), byte5=ina_expected_mask (bit i = branch powered this phase), byte6=i2c_last_read_ms (last Current_ReadAll() duration in ms, saturated at 255 = "255+"), byte7=reserved | `can_handler.c`, `sensor_manager.c` |
| 0x30A | DIAG_CAN_META | 8 | 1000 ms | 0x309 delivery meta: byte0-1=call_count u16 LE, byte2-3=tick_count u16 LE, byte4=tx_ok, byte5=tx_err, byte6=fifo_full_drops, byte7=bit0 fdcan_init_ok + bits7:1 hb_tx_err (saturated at 127) | `can_handler.c`, `main.c` |
| 0x30B | DIAG_I2C_SCAN | 8 | on-demand | Active I2C scan (after SERVICE 0xF6): byte0=bus_flags(bit0 scl_high,bit1 sda_high,bit2 rec_attempted,bit3 rec_success), byte1=mux_present, byte2=ina_present_mask, byte3=fail_count, byte4=recovery_attempts, byte5=scan_phase(0=unknown,1=bus_busy,2=tca_missing,3=tca_ack) | `can_handler.c`, `sensor_manager.c` |
| 0x30C | DIAG_FDCAN | 6 | on-demand | FDCAN error dump (after SERVICE 0xF6): byte0=last_error_code(LEC), byte1=state_flags(bit0 epassive,bit1 busoff,bit2 warning), byte2=tec, byte3=rec, byte4=tx_nack_flag, byte5=tx_consec_fail | `can_handler.c` |
| 0x30D | DIAG_GEAR_LIMITS | 8 | on-demand | Gear power limits + accel response (burst after SERVICE 0xF7 QUERY). Two interleaved frame kinds share this ID, selected by byte0 bit4: **bit4=0 POWER**, **bit4=1 RESPONSE**. byte0=flags(bit0 stored-valid,bit1 pending-differs,bit2 safety-ok/STANDBY,bit3 pending-valid,bit4 frame-kind), byte1-3=active D2/D1/R %, byte4-6=pending D2/D1/R %, byte7=system_state. A QUERY emits both frames back-to-back; a decoder must preserve the "other half" when updating. | `can_handler.c`, `motor_control.c` |
| 0x30E | DIAG_STEERING_Z | 8 | on-demand | PB5 + encoder-Z dual steering-center diagnostic (burst after SERVICE 0xF8 QUERY). byte0=flags(bit0-2 status: 0=NOT CALIBRATED,1=OK,2=Z NOT SEEN,3=Z OUT OF WINDOW,4=MECH OFFSET; bit3 PB5 live at center; bit4 Z calibration valid; bit5 Z slip), byte1=Z pulse count (saturating 255), byte2-3=last Z position int16 LE (TIM2 counts), byte4-5=Z↔center offset int16 LE (counts), byte6=last Z error int8, byte7=active tolerance (counts). Diagnostic-only — no control/safety path consumes it. | `can_handler.c`, `steering_z.c`, `steering_cal_store.c` |
| 0x310 | DIAG_DRIVE_TUNING | 8 | on-demand | Drive-tuning (ramp/creep) field-stream (burst after SERVICE 0xFA QUERY). One frame per field; a QUERY emits 10 sweeps × all 6 fields at 10 Hz. byte0=flags(bit0 stored-valid,bit1 pending-differs,bit2 safety-ok/STANDBY,bit3 pending-valid), byte1=field id (1=AccelRamp,2=BrakeRamp,3=ReverseRamp,4=CreepEnable,5=CreepPower,6=CreepDelay), byte2-3=active value u16 LE, byte4-5=pending value u16 LE, byte6=system_state, byte7=field count (6). Units: ramps %/s, CreepEnable 0/1, CreepPower %, CreepDelay ms. | `can_handler.c`, `motor_control.c`, `drive_tuning_store.c` |
| 0x311 | DIAG_BATTERY_LIMITS | 8 | on-demand | Battery voltage-limit field-stream (burst after SERVICE 0xFB QUERY). One frame per field; a QUERY emits 10 sweeps × all 5 fields at 10 Hz. byte0=flags(bit0 stored-valid,bit1 pending-differs,bit2 safety-ok/STANDBY,bit3 pending-valid), byte1=field id (1=Warning,2=Limit/derate,3=Cutoff,4=Recovery,5=Filter), byte2-3=active value u16 LE, byte4-5=pending value u16 LE, byte6=system_state, byte7=field count (5). Units: thresholds centivolts (V×100), Filter ms. Diagnostic/config only — never alters the safety state machine. | `can_handler.c`, `safety_system.c`, `battery_limits_store.c` |
| 0x312 | DIAG_BOOT_RESET | 8 | 1000 ms | Boot/reset + TX-backlog diag: byte0-3=STM32 uptime_ms u32 LE (HAL_GetTick, wraps ~49.7 days), byte4=reset_cause bitmask (bit0=POWERON/POR, bit1=SOFTWARE/SW, bit2=IWDG, bit3=WWDG, bit4=BROWNOUT/BOR, bit5=PIN), byte5=tx_queue_depth (software TX ring occupancy now, 0..31), byte6=tx_queue_depth_max (high-water mark, 0..31), byte7=reserved. ESP32 accepts DLC≥5 (uptime/reset) and DLC≥7 (txQ); DLC<5 dropped. Diagnostic-only — no control/safety path consumes it. | `can_handler.c`, `main.c` |
| 0x313 | DIAG_WHEEL_SENSOR | 8 | 1000 ms | Per-wheel speed-sensor fault-reason diag: byte0-3=reason FL/FR/RL/RR (WheelDiag_t code 0-8; while a channel fault is latched this is the **latched culprit reason**, not the self-healed live reason), byte4=reason STEER/CENTER (DISABLED_STATE=7 when steering-encoder module off, else 0=OK), byte5=gpio_level_mask (bit0 FL..bit3 RR, bit4 STEER), byte6=active_fault_mask (bit0 FL..bit3 RR, bit4 STEER), byte7=flags/seq (bit0 powertrain_engaged, bit1 manual_movement, bit2 wheel_fault_debouncing, bit3 wheel_fault_latched, bits4-7 sequence). Reason codes: 0=OK 1=NO_PULSE 2=STUCK_HIGH 3=STUCK_LOW 4=MISMATCH 5=IMPOSSIBLE_RATE 6=MANUAL_MOVEMENT 7=DISABLED_STATE 8=UNKNOWN. ESP32 accepts DLC≥7 (byte7 read only at DLC 8); DLC<7 dropped. Diagnostic-only — no control/safety path consumes it; wheel-fault safety logic unchanged. | `can_handler.c`, `safety_system.c`, `main.c` |
| 0x314 | DIAG_WHEEL_PULSES | 8 | 1000 ms | Per-wheel **VALID (accepted)** pulse counts: 4× wheel u16 LE (FL,FR,RL,RR) saturated to 0xFFFF. Pulses that PASSED the DWT 200 µs pre-filter (real wheel rotation, ~6/turn) — complements the REJECTED counts in 0x306. Diagnostic-only. | `can_handler.c`, `sensor_manager.c` |
| 0x315 | DIAG_MOTION_INHIBIT | 8 | 100 ms | **MOTION_INHIBIT_REASON** — why the traction chain is (or is not) producing torque. byte0-1=reason bitfield u16 LE (`MOTION_INHIBIT_*`: STATE_SAFE 0x01, STATE_ERROR 0x02, POWER_NOT_READY 0x04, GEAR_PARK 0x08, GEAR_NEUTRAL 0x10, NO_DEMAND 0x20, DEMAND_ZEROED 0x40, OBSTACLE_BLOCK 0x80, PWM_ZERO 0x100, TORQUE_LIMITED 0x200, STARTUP_INHIBIT 0x400, PEDAL_FAULT 0x800, SAFETY_SCALE_ZERO 0x1000, BATTERY_CUTOFF 0x2000, THERMAL_OVERCURRENT 0x4000, SERVICE_DISABLED 0x8000), byte2=system state, byte3=gear, byte4=operator demand % (int8), byte5=effective demand % (int8), byte6=max final PWM duty % (0..100), byte7=flags (bit0 power_ready, bit1 obstacle_forward_blocked, bits2-3 relay-seq phase [0 idle,1 in-progress,2 complete] = commanded GPIO/sequencer state only — **no physical relay-contact feedback**, bits4-7 degraded level). ESP32 `decodeMotionInhibit()` accepts DLC≥8. Instrumentation-only — no control/safety path consumes it. | `can_handler.c`, `motor_control.c`, `motion_inhibit.c` |
| 0x316 | DIAG_STEERING_CENTERING | 8 | 1000 ms | **Steering homing telemetry** — WHY the automatic centering sweep did/did not progress (renders "DIRECCIÓN NO SE MUEVE" with the real cause instead of "Error 8"). byte0=diag reason (`STEER_DIAG_*` 0..15: OK, RESTORED_FROM_FLASH, WAITING_POWER, CENTER_SENSOR_ACTIVE, SWEEP_LEFT, SWEEP_RIGHT, NO_ENCODER_MOVEMENT, ENCODER_FAULT, RANGE_EXCEEDED, TOTAL_TIMEOUT, LOST_HOMING_STATE, RELAY_NOT_READY, MODULE_DISABLED, ABORTED_SAFE, ABORTED_ERROR, UNKNOWN), byte1=FSM state low nibble (CenteringState_t) \| motor owner high nibble (0 CENTERING/1 EPS), byte2=flags (bit0 PB5 raw, bit1 PB5 debounced, bit2 PB5 already-active-at-sweep-start, bit3 PC12 relay commanded, bit4 power ready, bit5 PC4 EN_STEER commanded, bit6 encoder fault, bit7 restored-from-flash), byte3=system state low nibble (`STEER_DIAG_SS_*`) \| bit4 module disabled \| bit5 fault latched \| bit6 pwm requested (>0), byte4-5=PWM real (max CCR PA6/PA7, u16 LE), byte6-7=encoder delta from sweep origin (int16 LE, clamped). Classification is the pure host-tested `SteeringCentering_ClassifyDiag()`; packed by `SteerCentering_PackFrame()` (shared `steering_centering_frame.h`), emitted by `CAN_SendSteeringCenteringDiag()`; ESP32 `decodeSteeringCenteringDiag()` via `steering_diag_view::decode()` accepts DLC≥8. **Instrumentation only** — drives no motor/relay/PWM/FSM. | `can_handler.c`, `steering_centering.c`, `steering_centering_diag.c` |
| 0x317 | DIAG_RELAY_HEALTH | 8 | 1000 ms | **Traction relay / current-sense health** — evidence-graded cause so the HMI shows CURRENT SENSE INVALID vs RELAY OPEN SUSPECTED (with the numbers behind the verdict) instead of a bare "RELAY OPEN". byte0=diag reason (`RELAY_DIAG_*` 0..10: OK, OPEN_CONFIRMED, OPEN_SUSPECTED, CURRENT_SENSE_INVALID, SHUNT_OPEN, SHUNT_BYPASSED, POLARITY_REVERSED, DATA_STALE, INA_MISSING, SCALE_INVALID, INCONCLUSIVE), byte1=flags (bit0 relay commanded, bit1 relay sequence complete, bit2 power ready, bit3 any wheel moving, bit4 current reading valid, bit5 current sample stale [age ≥ 300 ms], bit6 expected wheel INA missing, bit7 polarity reversed), byte2-3=sum \|CH0..3\| current in centi-amps (u16 LE, saturating), byte4=throttle % (0..100), byte5=final PWM/traction demand % (0..100), byte6-7=current sample age ms (u16 LE, saturating). Classification is the pure host-tested `Relay_ClassifyHealth()`; the snapshot is built by `Safety_UpdateRelayHealthDiag()`, packed by `RelayHealth_PackFrame()` (shared `relay_health_frame.h`), emitted by `CAN_SendRelayHealthDiag()`; ESP32 `decodeRelayHealthDiag()` via `relay_health_view::decode()` accepts DLC≥8. Because this HW has NO post-relay voltage/contact feedback, the firmware downgrades any classifier `OPEN_CONFIRMED` to `OPEN_SUSPECTED` (never claims confirmation without independent evidence). **Instrumentation only** — the legacy `Safety_CheckRelayHealth()` latch is unchanged; drives no relay/PWM. | `can_handler.c`, `safety_system.c`, `relay_health_diag.c` |
| 0x318 | DIAG_INA_CH5 | 8 | 1000 ms | **Steering INA226 (CH5) channel diagnostic** — separates a genuinely MISSING chip (no I2C ACK) from a transport gap ("n/d" = ABSENCE of this frame), a lost config, an open/bypassed shunt, or reversed-polarity wiring, and carries a **SIGNED** shunt so a negative steering current is never flattened to zero. byte0=fault reason (`INA_CH5_*` 0..9: OK, PRESENT_NO_SHUNT, POLARITY_REVERSED, STALE, MUX_SELECT_FAIL, MISSING, WRONG_ID, CONFIG_LOST, READ_FAIL, UNKNOWN), byte1=flags (bit0 mux select ok, bit1 i2c ack, bit2 identity ok [MFG 0x5449 & DIE 0x2260], bit3 config readback ok, bit4 shunt read ok, bit5 bus read ok, bit6 channel powered, bit7 stale [age ≥ 500 ms]), byte2-3=raw shunt register (int16 LE, signed two's complement; µV = raw×2.5, mA = µV/1.5 mΩ — both derived signed, never zeroed), byte4-5=bus voltage mV (u16 LE, saturating), byte6-7=sample age ms (u16 LE, 0xFFFF = never sampled). Probe + classification via the pure host-tested `Ina226_ClassifyChannel()`; the snapshot is built each cycle by `Sensor_UpdateChannel5Diag()` (full identity/config/shunt/bus sequence, report-only — rolls back its own I2C fail count), packed by `Ina226Ch5_PackFrame()` (shared `ina226_ch5_frame.h`), emitted by `CAN_SendIna226Ch5Diag()`; ESP32 `decodeIna226Ch5Diag()` via `ina226_ch5_view::decode()` accepts DLC≥8, HMI shows CH5 MISSING / WRONG ADDRESS / CONFIG FAIL / PRESENT NO SHUNT / POLARITY REVERSED / STALE / OK on the INA226 LIVE DIAG page. **Instrumentation only** — never gates control; the probe's failures never trip the bus-fault/Error-Code-11 recovery. | `can_handler.c`, `sensor_manager.c`, `ina226_channel_diag.c` |

### 3.4 Obstacle Data (ESP32 → STM32)

See messages 0x208 and 0x209 in §3.1.

### 3.5 Command Acknowledgment (STM32 → ESP32)

| CAN ID | Name | DLC | Rate | Description | Source file |
|--------|------|-----|------|-------------|-------------|
| 0x103 | CMD_ACK | 3 | On-demand | Command acknowledgment after safety validation | `can_handler.c` |

The STM32 sends an ACK frame after processing on-demand commands (CMD_MODE 0x102, SERVICE_CMD 0x110). High-frequency commands (CMD_THROTTLE 0x100, CMD_STEERING 0x101) are NOT acknowledged individually — the ESP32 monitors status messages (0x200–0x207) for implicit confirmation.

ACK is sent only after:
1. Safety state validation (`Safety_IsCommandAllowed()`)
2. Command-specific validation (speed check, parameter range check)
3. Command acceptance or rejection decision

---

## 4. Payload Definitions

### 4.1 HEARTBEAT_STM32 (0x001) — STM32 → ESP32

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | alive_counter | uint8 | Incrementing counter, 0–255, intentional rollover |
| 1 | system_state | uint8 | Current system state (see section 6) |
| 2 | fault_flags | uint8 | Bitmask of active faults (see section 6) |
| 3 | error_code | uint8 | Current safety error code (Safety_Error_t, 0–13; see section 7) |
| 4 | status_flags | uint8 | bit 0: startup inhibit, bit 1: 4×4, bit 2: tank turn, bits 3–5: DS18B20 count |
| 5 | relay_status | uint8 | Power relay command state (3-bit wire layout — backward-compatible): bit 0 = reserved (always 0), bit 1 = TRAC (PC11, 24 V), bit 2 = DIR (PC12, 12 V), bits 3–6 reserved (0), bit 7 = SEQ_COMPLETE |

**Hardware note:** PC10 on the STM32 is available/unused. Bit 0 of `relay_status` is kept in the wire format for backward compatibility.

**Backward compatibility:** Consumers that check bit 0 will observe it as permanently 0 — safe degradation.

Source: `CAN_SendHeartbeat()` in `can_handler.c`

### 4.2 HEARTBEAT_ESP32 (0x011) — ESP32 → STM32

The STM32 does not parse the payload of this message. Only the arrival of a frame with ID 0x011 is used to reset the heartbeat watchdog. The ESP32 may define its own payload structure, but the STM32 ignores it.

Source: `CAN_ProcessMessages()` case `CAN_ID_HEARTBEAT_ESP32` in `can_handler.c`

### 4.3 CMD_THROTTLE (0x100) — ESP32 → STM32

| Byte | Field | Type | Unit | Range | Description |
|------|-------|------|------|-------|-------------|
| 0 | throttle_pct | uint8 | % | 0–100 | Requested throttle percentage |

The STM32 clamps this value to 0–100, rejects it entirely if not in ACTIVE state, forces zero if ABS is active, and halves it if TCS is active.

Source: `CAN_ProcessMessages()` and `Safety_ValidateThrottle()` in `safety_system.c`

### 4.4 CMD_STEERING (0x101) — ESP32 → STM32

| Byte | Field | Type | Unit | Range | Description |
|------|-------|------|------|-------|-------------|
| 0 | angle_LSB | uint8 | — | — | Low byte of signed 16-bit value |
| 1 | angle_MSB | uint8 | — | — | High byte of signed 16-bit value |

Decoding: `int16_t angle_raw = rx_payload[0] | (rx_payload[1] << 8)`
Conversion: `requested_deg = (float)angle_raw / 10.0`

The STM32 clamps the result to ±45.0°, rate-limits to 200°/s maximum, and rejects the command if not in ACTIVE state.

Source: `CAN_ProcessMessages()` and `Safety_ValidateSteering()` in `safety_system.c`

### 4.5 CMD_MODE (0x102) — ESP32 → STM32

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | mode_flags | uint8 | Bit 0: enable 4×4 (1 = 4×4, 0 = 4×2). Bit 1: tank turn (1 = enabled). Bits 2–7: reserved. |
| 1 | gear | uint8 | Gear position: 0=PARK, 1=REVERSE, 2=NEUTRAL, 3=FORWARD(D1), 4=FORWARD_D2. Optional — if DLC < 2, gear remains unchanged. |

The STM32 rejects the mode change unless in ACTIVE or DEGRADED state and average wheel speed is below 1.0 km/h. Gear changes are only accepted at ≤ 1.0 km/h.

Source: `CAN_ProcessMessages()` and `Safety_ValidateModeChange()` in `safety_system.c`

### 4.6 STATUS_SPEED (0x200) — STM32 → ESP32

| Bytes | Field | Type | Unit | Description |
|-------|-------|------|------|-------------|
| 0–1 | speed_FL | uint16 LE | 0.1 km/h | Front-left wheel speed |
| 2–3 | speed_FR | uint16 LE | 0.1 km/h | Front-right wheel speed |
| 4–5 | speed_RL | uint16 LE | 0.1 km/h | Rear-left wheel speed |
| 6–7 | speed_RR | uint16 LE | 0.1 km/h | Rear-right wheel speed |

Encoding at source: `(uint16_t)(Wheel_GetSpeed_XX() * 10)` — value is km/h multiplied by 10.

Source: `CAN_SendStatusSpeed()` in `can_handler.c`, called from `main.c`

### 4.7 STATUS_CURRENT (0x201) — STM32 → ESP32

| Bytes | Field | Type | Unit | Description |
|-------|-------|------|------|-------------|
| 0–1 | current_FL | uint16 LE | 0.01 A | Front-left motor current |
| 2–3 | current_FR | uint16 LE | 0.01 A | Front-right motor current |
| 4–5 | current_RL | uint16 LE | 0.01 A | Rear-left motor current |
| 6–7 | current_RR | uint16 LE | 0.01 A | Rear-right motor current |

Encoding at source: `(uint16_t)(Current_GetAmps(n) * 100)` — value is amperes multiplied by 100.

Source: `CAN_SendStatusCurrent()` in `can_handler.c`, called from `main.c`

### 4.8 STATUS_TEMP (0x202) — STM32 → ESP32

| Byte | Field | Type | Unit | Description |
|------|-------|------|------|-------------|
| 0 | temp_0 | int8 | °C | Temperature sensor 0 |
| 1 | temp_1 | int8 | °C | Temperature sensor 1 |
| 2 | temp_2 | int8 | °C | Temperature sensor 2 |
| 3 | temp_3 | int8 | °C | Temperature sensor 3 |
| 4 | temp_4 | int8 | °C | Temperature sensor 4 |

Values are truncated to integer degrees Celsius. Five DS18B20 sensors.

Source: `CAN_SendStatusTemp()` in `can_handler.c`, called from `main.c`

### 4.9 STATUS_SAFETY (0x203) — STM32 → ESP32

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | abs_active | uint8 | 1 = ABS currently active, 0 = inactive |
| 1 | tcs_active | uint8 | 1 = TCS currently active, 0 = inactive |
| 2 | error_code | uint8 | Current safety error code (see section 6) |
| 3 | state | uint8 | Current system state (`system_state`) |
| 4 | rx_errors | uint8 | FDCAN RX error counter |

Source: `CAN_SendStatusSafety()` in `can_handler.c`, called from `main.c`

### 4.10 STATUS_STEERING (0x204) — STM32 → ESP32

| Byte | Field | Type | Unit | Description |
|------|-------|------|------|-------------|
| 0 | angle_LSB | uint8 | — | Low byte of signed 16-bit actual angle |
| 1 | angle_MSB | uint8 | — | High byte of signed 16-bit actual angle |
| 2 | calibrated | uint8 | — | 1 = encoder calibrated, 0 = not calibrated |

Decoding: `int16_t angle_raw = byte[0] | (byte[1] << 8)`
Conversion: `actual_deg = (float)angle_raw / 10.0`

Encoding at source: `(int16_t)(Steering_GetCurrentAngle() * 10)`

Source: `CAN_SendStatusSteering()` in `can_handler.c`, called from `main.c`

### 4.11 STATUS_TRACTION (0x205) — STM32 → ESP32

| Byte | Field | Type | Unit | Range | Description |
|------|-------|------|------|-------|-------------|
| 0 | traction_FL | uint8 | % | 0–100 | Front-left wheel traction scale |
| 1 | traction_FR | uint8 | % | 0–100 | Front-right wheel traction scale |
| 2 | traction_RL | uint8 | % | 0–100 | Rear-left wheel traction scale |
| 3 | traction_RR | uint8 | % | 0–100 | Rear-right wheel traction scale |

Encoding: `(uint8_t)(safety_status.wheel_scale[i] * 100.0f)`

- 100 = full power available (no ABS/TCS intervention on this wheel)
- 0 = wheel fully inhibited (ABS has cut this wheel)
- Intermediate values = TCS is progressively limiting this wheel

The values are the same `wheel_scale[4]` array used by `Traction_Update()` to modulate per-wheel PWM. No recalculation is performed; the ESP32 receives exactly what the STM32 applies.

Source: `CAN_SendStatusTraction()` in `can_handler.c`, called from `main.c`

### 4.12 STATUS_TEMP_MAP (0x206) — STM32 → ESP32

| Byte | Field | Type | Unit | Range | Description |
|------|-------|------|------|-------|-------------|
| 0 | temp_motor_FL | int8 | °C | −128 to +127 | Motor FL temperature (DS18B20 sensor index 0) |
| 1 | temp_motor_FR | int8 | °C | −128 to +127 | Motor FR temperature (DS18B20 sensor index 1) |
| 2 | temp_motor_RL | int8 | °C | −128 to +127 | Motor RL temperature (DS18B20 sensor index 2) |
| 3 | temp_motor_RR | int8 | °C | −128 to +127 | Motor RR temperature (DS18B20 sensor index 3) |
| 4 | temp_ambient | int8 | °C | −128 to +127 | Ambient temperature (DS18B20 sensor index 4) |

Sensor index mapping:

| Byte | Sensor index | Physical location |
|------|-------------|-------------------|
| 0 | `Temperature_Get(0)` | Motor front-left |
| 1 | `Temperature_Get(1)` | Motor front-right |
| 2 | `Temperature_Get(2)` | Motor rear-left |
| 3 | `Temperature_Get(3)` | Motor rear-right |
| 4 | `Temperature_Get(4)` | Ambient |

Values are the same DS18B20 readings used by `Safety_CheckTemperature()`. No filtering or recalculation is performed. If a sensor is disabled in Service Mode, the last read value is still reported (the safety system handles threshold checks independently).

The existing STATUS_TEMP (0x202) message is unchanged and continues to transmit at the same rate. This message provides an explicit byte-to-sensor mapping for unambiguous HMI display.

Source: `CAN_SendStatusTempMap()` in `can_handler.c`, called from `main.c`

### 4.13 STATUS_BATTERY (0x207) — STM32 → ESP32

| Bytes | Field | Type | Unit | Description |
|-------|-------|------|------|-------------|
| 0–1 | battery_current | uint16 LE | 0.01 A | Battery bus current (INA226, 100 A shunt) |
| 2–3 | battery_voltage | uint16 LE | 0.01 V | Battery bus voltage (24 V nominal) |

Encoding at source: `(uint16_t)(Current_GetAmps(INA226_CHANNEL_BATTERY) * 100)` and `(uint16_t)(Voltage_GetBus(INA226_CHANNEL_BATTERY) * 100)`.

Source: `CAN_SendStatusBattery()` in `can_handler.c`, called from `main.c`

### 4.14 DIAG_ERROR (0x300) — Both Directions

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | error_code | uint8 | Error type (see safety error codes) |
| 1 | subsystem | uint8 | 0 = Global, 1 = Motor, 2 = Sensor, 3 = CAN |

Source: `CAN_SendError()` in `can_handler.c`

### 4.14a DIAG_DEBOUNCE (0x306) — STM32 → ESP32 (rev 1.4, additive)

DWT-debounce EMI diagnostic counters for the four wheel-speed channels. Each counter records the number of edge pulses rejected by the µs-level DWT pre-filter (Step 0 of `Wheel_IRQDebounced`, `SENSOR_DEBOUNCE_US = 200 µs`). Internal counters are 32-bit saturated; this frame truncates to uint16 with saturation to `0xFFFF`.

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0–1 | filtered_FL | uint16 LE | Wheel 0 (FL) filtered-pulse count, saturated to 0xFFFF |
| 2–3 | filtered_FR | uint16 LE | Wheel 1 (FR) filtered-pulse count, saturated to 0xFFFF |
| 4–5 | filtered_RL | uint16 LE | Wheel 2 (RL) filtered-pulse count, saturated to 0xFFFF |
| 6–7 | filtered_RR | uint16 LE | Wheel 3 (RR) filtered-pulse count, saturated to 0xFFFF |

**Diagnostic only** — must NOT gate any control or safety path on these values. Used by the ESP32 hidden engineering menu (`DEBOUNCE DEBUG` submenu, PIN 8989) for empirical validation of the EMI mitigation chain (TVS → EL817 opto → DWT 200 µs filter).

Source: `CAN_SendDebounceDiag()` in `can_handler.c` calling `Sensor_GetFilteredCount()` from `sensor_manager.c`.

### 4.14b DIAG_DEBOUNCE_STEER (0x307) — STM32 → ESP32 (rev 1.4, additive)

DWT-debounce EMI diagnostic counter for the steering-center inductive sensor (PB5/EXTI5). Full 32-bit value (no truncation) because steering events are extremely rare.

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0–3 | filtered_steer | uint32 LE | Steering-center filtered-pulse count, saturated to `0xFFFFFFFF` |

**Diagnostic only** — same constraints as 0x306.

Source: `CAN_SendDebounceDiag()` in `can_handler.c` calling `Sensor_GetSteerFilteredCount()` from `sensor_manager.c`.

### 4.14c DIAG_WHEEL_PULSES (0x314) — STM32 → ESP32 (rev 1.15, additive)

Per-wheel **VALID (accepted)** pulse counts for the four wheel-speed channels. Unlike 0x306 (which counts pulses *rejected* by the DWT 200 µs pre-filter), this frame reports the pulses that *passed* the filter and are used for the speed/distance estimate (`Wheel_GetPulseCount()` → internal `wheel_pulse[]`). Internal counters are 32-bit; this frame truncates to uint16 with saturation to `0xFFFF`. Counters reset to 0 on every reboot (`Sensor_Init`).

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0–1 | valid_FL | uint16 LE | Wheel 0 (FL) accepted-pulse count, saturated to 0xFFFF |
| 2–3 | valid_FR | uint16 LE | Wheel 1 (FR) accepted-pulse count, saturated to 0xFFFF |
| 4–5 | valid_RL | uint16 LE | Wheel 2 (RL) accepted-pulse count, saturated to 0xFFFF |
| 6–7 | valid_RR | uint16 LE | Wheel 3 (RR) accepted-pulse count, saturated to 0xFFFF |

Reference counts (6 pulses per wheel revolution): 1 turn ≈ 6 valid pulses, 3 m straight ≈ 16 valid pulses/wheel, 6 m out-and-back ≈ 32 valid pulses/wheel. REJECTED counts (0x306) may differ per wheel, but VALID counts should be similar across wheels for the same movement.

**Diagnostic only** — must NOT gate any control or safety path. Shown next to the REJECTED column on the ESP32 `DEBOUNCE / CAN DIAG` submenu as `VALID PULSES | REJECTED PULSES`. ESP32 `decodeWheelPulses()` accepts DLC ≥ 8; shorter frames are ignored. Additive — firmware without the field ignores the new ID.

Source: `CAN_SendDebounceDiag()` in `can_handler.c` calling `Wheel_GetPulseCount()` from `sensor_manager.c`.

### 4.15 OBSTACLE_DISTANCE (0x208) — ESP32 → STM32

| Byte | Field | Type | Unit | Range | Description |
|------|-------|------|------|-------|-------------|
| 0 | distance_LSB | uint8 | — | — | Low byte of minimum obstacle distance |
| 1 | distance_MSB | uint8 | — | — | High byte of minimum obstacle distance |
| 2 | zone | uint8 | — | 0–5 | Obstacle proximity zone (0=none, 5=emergency) |
| 3 | sensor_health | uint8 | — | 0–1 | 0 = sensor unhealthy, 1 = sensor healthy |
| 4 | counter | uint8 | — | 0–255 | Rolling counter, must increment each frame |

Decoding: `uint16_t distance_mm = byte[0] | (byte[1] << 8)`

The STM32 uses distance_mm to compute an independent `obstacle_scale` factor (0.0–1.0) via a speed-dependent state machine with temporal hysteresis (200 ms confirmation before applying emergency scale). Base static thresholds:
- distance < 500 mm → scale = 0.0, forward blocked (emergency zone, after 200 ms confirmation)
- distance 500–1000 mm → scale = 0.3
- distance 1000–1500 mm → scale = 0.7
- distance 1500–2000 mm → scale = 0.85
- distance 2000–4000 mm → scale = 0.95
- distance ≥ 4000 mm → scale = 1.0

Note: scale = 0.0 inhibits forward drive but does **NOT** trigger a SAFE state transition. The system remains in ACTIVE with obstacle_forward_blocked = 1. `Safety_SetError(SAFETY_ERROR_OBSTACLE)` is set, but setting this error does NOT automatically call `Safety_SetState(SAFE)`.

The rolling counter is checked for stale-data detection. If the counter does not change for 3 consecutive frames, a sensor_fault is raised: `obstacle_scale = OBSTACLE_FAULT_SCALE` (0.3), state = `OBS_STATE_SENSOR_FAULT`. **No SAFE state transition.**

**Timeout behavior:** If no 0x208 message is received for > 500 ms (after first reception):
- If obstacle was actively tracked (ACTIVE or CONFIRMING state):
  - `obstacle_scale = OBSTACLE_FAULT_SCALE` (0.3) — never relaxes to 1.0
  - `obstacle_state = OBS_STATE_SENSOR_FAULT`
  - **No SAFE state transition.**
- If no obstacle was active:
  - `obstacle_scale = 1.0`
  - `obstacle_state = OBS_STATE_NO_SENSOR`
  - **No SAFE state transition.**
- `Safety_SetError(SAFETY_ERROR_OBSTACLE)` is NOT called on timeout alone.

**Validation rules:**
- DLC must be ≥ 5 (messages with DLC < 5 are silently dropped)
- Zone must be 0–5 (values > 5 are clamped to 0)
- Counter must increment between frames (frozen counter = sensor_fault → scale = 0.3, no SAFE)
- Sensor health = 0 → sensor_fault: scale = 0.3, state = OBS_STATE_SENSOR_FAULT. **No SAFE state transition.**

Source: `CAN_ProcessMessages()` and `Obstacle_ProcessCAN()` in `safety_system.c`

### 4.16 OBSTACLE_SAFETY (0x209) — ESP32 → STM32

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | speedReductionFactor | uint8 | ESP32-computed factor × 100 (0–100). Informational only. |
| 1 | emergencyBrakeApplied | uint8 | 1 = ESP32 has applied emergency brake logic |
| 2 | collisionImminent | uint8 | 1 = ESP32 detects collision risk |
| 3 | obstacleZone | uint8 | ESP32-computed zone (0–5) |
| 4 | childReactionDetected | uint8 | 1 = child reaction detected by ESP32 |
| 5–7 | reserved | uint8 | Always 0x00 |

This message is **informational only** — the STM32 does NOT use these values for safety decisions. The STM32 computes its own `obstacle_scale` independently from the raw distance data in 0x208. This message is accepted by the CAN filter but not parsed, reserved for future ESP32→STM32 coordination.

Source: `CAN_ProcessMessages()` in `can_handler.c`

### 4.17 CMD_ACK (0x103) — STM32 → ESP32

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | cmd_id_low | uint8 | Low byte of the acknowledged command CAN ID (e.g. 0x02 for CMD_MODE 0x102, 0x10 for SERVICE_CMD 0x110) |
| 1 | result | uint8 | ACK result code (see table below) |
| 2 | system_state | uint8 | Current system state at time of ACK (see §6) |

**ACK result codes (byte 1):**

| Code | Name | Description |
|------|------|-------------|
| 0 | ACK_OK | Command accepted and applied |
| 1 | ACK_REJECTED | Command rejected (e.g. speed too high for mode/gear change) |
| 2 | ACK_INVALID | Command payload invalid or malformed (bad DLC, out-of-range parameter) |
| 3 | ACK_BLOCKED_BY_SAFETY | Command blocked because system is not in ACTIVE or DEGRADED state |

**Acknowledged commands:**

| Command | cmd_id_low | When ACK is sent |
|---------|-----------|-----------------|
| CMD_MODE (0x102) | 0x02 | After mode/gear validation and acceptance or rejection |
| SERVICE_CMD (0x110) | 0x10 | After service command processing |
| SERVICE_CMD I2C scan (0x110 byte0=0xF6) | 0xF6 | Immediate "scan started" echo sent **before** the synchronous I2C probe, so the HMI can tell a lost request (no 0xF6 echo) from a lost reply (echo but no 0x30B/0x30C). The normal post-scan ACK (cmd_id_low = 0x10) is still sent afterwards. |

**Not acknowledged (high-frequency):**

| Command | Reason |
|---------|--------|
| CMD_THROTTLE (0x100) | Continuous at 50 ms — implicit confirmation via status messages |
| CMD_STEERING (0x101) | Continuous at 50 ms — implicit confirmation via STATUS_STEERING (0x204) |

**ESP32 behavior:**
- Wait for ACK before updating UI state for mode/gear/service changes
- Timeout after `ACK_TIMEOUT_MS` (200 ms) — bounded, no infinite retry
- Display error state if ACK not received or result ≠ OK
- No blocking delays — ACK check is polled in the main loop

Source: `CAN_SendCommandAck()` in `can_handler.c`, `CAN_AckResult_t` in `can_handler.h`

### 4.18 SERVICE_CMD GEAR_LIMITS (0x110 action 0xF7) — ESP32 → STM32 (rev 1.8, extended rev 1.9, additive)

Configures the per-gear traction **power limits** (applied in `Traction_Update()`)
**and** the per-gear acceleration **response profile** (applied in
`Traction_SetDemand()` after EMA#2, before the global ramp limiter).
The frame reuses `SERVICE_CMD` (0x110); byte 0 selects the action, byte 1 the
sub-opcode, byte 2 the percent value (for SET ops).

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | action | uint8 | `0xF7` = `SERVICE_ACTION_GEAR_LIMITS` |
| 1 | op | uint8 | Sub-opcode (see table) |
| 2 | value | uint8 | Percent (SET ops only; ignored otherwise) |

**Sub-opcodes (byte 1):**

| Op | Name | DLC | Effect |
|----|------|-----|--------|
| 0x01 | GEAR_LIMIT_OP_SET_D2 | 3 | Stage pending D2 power limit = byte2 % (RAM only, not applied) |
| 0x02 | GEAR_LIMIT_OP_SET_D1 | 3 | Stage pending D1 power limit = byte2 % |
| 0x03 | GEAR_LIMIT_OP_SET_R  | 3 | Stage pending R power limit = byte2 % |
| 0x04 | GEAR_LIMIT_OP_SAVE   | 2 | Validate full pending set (power + response), persist to flash, apply |
| 0x05 | GEAR_LIMIT_OP_RESET_DEFAULTS | 2 | Persist + apply factory defaults (power D2=100/D1=60/R=60, response D2=100/D1=70/R=40) |
| 0x06 | GEAR_LIMIT_OP_QUERY  | 2 | Emit a `DIAG_GEAR_LIMITS` (0x30D) burst — POWER **and** RESPONSE frames (read-only, no safety gate) |
| 0x07 | GEAR_LIMIT_OP_SET_D2_RESPONSE | 3 | Stage pending D2 accel response = byte2 % (RAM only) |
| 0x08 | GEAR_LIMIT_OP_SET_D1_RESPONSE | 3 | Stage pending D1 accel response = byte2 % |
| 0x09 | GEAR_LIMIT_OP_SET_R_RESPONSE  | 3 | Stage pending R accel response = byte2 % |

**Validation & safety (STM32-side):**
- Power ranges: D2 30–100 %, D1 20–100 %, R 10–60 %.
- Response ranges: D2 50–100 %, D1 30–100 %, R 20–80 %. Out-of-range → `ACK_INVALID`.
  The response factor only **softens** demand (every max ≤ 100 %, never amplifies).
- All mutating ops (SET/SAVE/RESET) require `SYS_STATE_STANDBY`; otherwise
  `ACK_BLOCKED_BY_SAFETY`. QUERY is always allowed.
- Persistence: flash page 122 (0x0807A000), CRC32-protected. The slot is
  16 bytes; v1 (`"GLM1"`, power only) is migrated on read to v2 (`"GLM2"`,
  power + response) — the persisted power limits are kept and the response
  defaults (100/70/40) are applied until the next SAVE rewrites the slot as v2.
  A blank/corrupt slot falls back to all defaults. Identical flash mechanism to
  the pedal-calibration store (`gear_limits_store.c`). Values survive reboot and
  are re-applied at boot.
- Every op replies with exactly one `CMD_ACK` (0x103, cmd_id_low = 0x10).

Source: `gearlim_handle_service_cmd()` in `can_handler.c`,
`Traction_{Validate,Set,Get}GearLimits()` and
`Traction_{Validate,Set,Get}GearResponse()` in `motor_control.c`,
`GearLimitsStore_*` in `gear_limits_store.c`.

---

### 4.19 SERVICE_CMD STEERING_Z (0x110 action 0xF8) — ESP32 → STM32 (rev 1.10, additive)

Manages the **secondary** encoder-Z steering-center reference. PB5 (LJ12A3) is
and remains the **primary** physical/safety center reference. The encoder Z
(index) pulse on PB4 is a precision/verification reference **only**; it can never
center the system on its own, and a Z pulse observed without PB5 confirmation is
**not** treated as a center. The frame reuses `SERVICE_CMD` (0x110); byte 0
selects the action, byte 1 the sub-opcode.

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | action | uint8 | `0xF8` = `SERVICE_ACTION_STEERING_Z` |
| 1 | op | uint8 | Sub-opcode (see table) |

**Sub-opcodes (byte 1):**

| Op | Name | DLC | Effect |
|----|------|-----|--------|
| 0x01 | STEER_Z_OP_QUERY     | 2 | Emit a `DIAG_STEERING_Z` (0x30E) burst (read-only, no gate) |
| 0x02 | STEER_Z_OP_CALIBRATE | 2 | Recompute the Z↔center offset relative to the current center and store it. **Only accepted while PB5 currently reads center** (never trusts Z alone) and only in BOOT/STANDBY |
| 0x03 | STEER_Z_OP_CLEAR     | 2 | Invalidate the stored Z calibration (BOOT/STANDBY only) |

**Validation & safety (STM32-side):**
- QUERY is always allowed and only emits diagnostics.
- CALIBRATE requires (a) `SYS_STATE_BOOT` or `SYS_STATE_STANDBY` **and**
  (b) PB5 currently asserted at center (`PIN_STEER_CENTER` LOW). If PB5 is not at
  center the command is rejected — Z is never accepted as a center on its own.
- CLEAR requires BOOT/STANDBY.
- Tolerances (encoder/volante frame, 4800 CPR = 360°, ~13.33 counts/°):
  recommended window ±25 counts (~1.9°), strict ±10 counts, fault/mechanical
  offset > 40 counts (~3.0°). See `STEERING_Z_WINDOW_COUNTS`,
  `STEERING_Z_STRICT_COUNTS`, `STEERING_Z_FAULT_COUNTS` in `project_config.h`.
- A Z fault (Z not seen / out of window / mechanical offset) **never** forces
  SAFE — it surfaces as a diagnostic/warning only, so it can never block ACTIVE.
- Persistence: the steering-cal flash page is upgraded to format v2
  (`z_center_offset_counts`, `z_center_tolerance`, `z_center_valid`,
  `format_version`). v1 slots are migrated on read (Z marked not-valid until a
  PB5+Z calibration rewrites the page as v2); a blank/corrupt slot falls back to
  defaults. No other flash page is touched.
- Every op replies with exactly one `CMD_ACK` (0x103, cmd_id_low = 0x10).

**Status (0x30E byte0 bits0-2):** `0` NOT CALIBRATED, `1` OK (PB5 confirmed +
Z within window), `2` Z NOT SEEN (PB5 ok, no Z), `3` Z OUT OF WINDOW, `4`
MECHANICAL OFFSET. `z_center_valid` (bit4) is set **only** when status == OK.

Source: `steerz_handle_service_cmd()` in `can_handler.c`, `steering_z.c`
(`SteeringZ_Classify`/`SteeringZ_NormaliseOffset`/`SteeringZ_OnCenterConfirmed`),
`SteeringCal_SaveWithZ()` + Z getters in `steering_cal_store.c`.

---

### 4.20 SERVICE_CMD DRIVE_TUNING (0x110 action 0xFA) — ESP32 → STM32 (rev 1.11, additive)

Lets the Engineering menu view and tune the traction "feel" parameters applied in
`motor_control.c`: the pedal ramp rates (accel / brake / reverse) and the motor
creep dead-zone (enable / power / delay). `SET_*` sub-opcodes carry a uint16
little-endian value in bytes 2-3 and stage a RAM-only **pending** set; nothing is
applied or persisted until `SAVE`. **With the compile-time defaults the traction
pipeline behaves byte-for-byte as the original firmware** (AccelRamp 50 %/s,
BrakeRamp 100 %/s, ReverseRamp 50 %/s, CreepEnable on, CreepPower 8 %, CreepDelay
0 ms). Persistence: flash **page 121** (`DTN1` magic, CRC32) — no other page is
touched.

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | action | uint8 | `0xFA` = `SERVICE_ACTION_DRIVE_TUNING` |
| 1 | op | uint8 | Sub-opcode (see table) |
| 2-3 | value | uint16 LE | New value for `SET_*` ops (ignored otherwise) |

**Sub-opcodes (byte 1):**

| Op | Name | DLC | Effect |
|----|------|-----|--------|
| 0x01 | DRIVE_TUNE_OP_SET_ACCEL_RAMP   | 4 | Stage pending accel ramp (%/s, 1–200) |
| 0x02 | DRIVE_TUNE_OP_SET_BRAKE_RAMP   | 4 | Stage pending brake ramp (%/s, 1–200) |
| 0x03 | DRIVE_TUNE_OP_SET_REVERSE_RAMP | 4 | Stage pending reverse ramp (%/s, 1–200; applied only in GEAR_REVERSE up-ramp) |
| 0x04 | DRIVE_TUNE_OP_SET_CREEP_ENABLE | 4 | Stage pending creep enable (0/1) |
| 0x05 | DRIVE_TUNE_OP_SET_CREEP_POWER  | 4 | Stage pending creep floor (%, 0–20) |
| 0x06 | DRIVE_TUNE_OP_SET_CREEP_DELAY  | 4 | Stage pending creep delay (ms, 0–5000) |
| 0x07 | DRIVE_TUNE_OP_SAVE             | 2 | Validate pending set + persist to flash + apply (STANDBY only) |
| 0x08 | DRIVE_TUNE_OP_RESET_DEFAULTS   | 2 | Restore + persist compile-time defaults (STANDBY only) |
| 0x09 | DRIVE_TUNE_OP_QUERY            | 2 | Emit a `DIAG_DRIVE_TUNING` (0x310) burst (read-only, no gate) |

**Validation & safety (STM32-side):**
- QUERY is always allowed and only emits diagnostics.
- All `SET_*`/`SAVE`/`RESET` require `SYS_STATE_STANDBY`; otherwise rejected with
  `ACK_BLOCKED_BY_SAFETY`.
- Every staged `SET` re-validates the **whole** pending set
  (`Traction_ValidateDriveTuning()`): all ramps strictly > 0, CreepPower ≤ firmware
  cap (20 %), CreepEnable ∈ {0,1}, CreepDelay ≤ 5000 ms. A rejected value keeps the
  previous pending value and replies `ACK_INVALID` (reject-keep-previous, FASE 4).
- Tuning can only **shape** an already-validated demand: ramp rates limit/smooth,
  the creep floor only raises the *minimum* drive PWM (never the maximum). Flash
  data alone never authorises ACTIVE.
- Every op replies with exactly one `CMD_ACK` (0x103, cmd_id_low = 0x10).

Source: `drvtune_handle_service_cmd()` + `CAN_DriveTuningBurstUpdate()` in
`can_handler.c`, `Traction_{Get,Set,Validate}DriveTuning()` in `motor_control.c`,
`drive_tuning_store.c`.

---

### 4.21 SERVICE_CMD BATTERY_LIMITS (0x110 action 0xFB) — ESP32 → STM32 (rev 1.11, additive)

Lets the Engineering menu view and tune the battery voltage thresholds read by
`safety_system.c`: low-voltage warning, derate (DEGRADED) limit, SAFE cutoff,
SAFE→STANDBY recovery and an optional voltage filter time constant. `SET_*`
sub-opcodes carry a uint16 little-endian value (centivolts = V×100, or ms for the
filter) in bytes 2-3 and stage a RAM-only **pending** set; nothing is applied or
persisted until `SAVE`. **With the compile-time defaults the under-voltage
protection behaves identically to the original firmware** (Warning/Limit 20.00 V,
Cutoff 18.00 V, Recovery 18.50 V, Filter 0 ms = no filtering). This command path
**never alters the safety state machine** — it only changes the numeric thresholds
the existing machine compares against. Persistence: flash **page 120** (`BAT1`
magic, CRC32) — no other page is touched.

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | action | uint8 | `0xFB` = `SERVICE_ACTION_BATTERY_LIMITS` |
| 1 | op | uint8 | Sub-opcode (see table) |
| 2-3 | value | uint16 LE | New value (cV or ms) for `SET_*` ops (ignored otherwise) |

**Sub-opcodes (byte 1):**

| Op | Name | DLC | Effect |
|----|------|-----|--------|
| 0x01 | BATT_LIM_OP_SET_WARNING   | 4 | Stage pending low-voltage warning (cV, 1500–3000) |
| 0x02 | BATT_LIM_OP_SET_LIMIT     | 4 | Stage pending derate/DEGRADED threshold (cV, 1500–3000) |
| 0x03 | BATT_LIM_OP_SET_CUTOFF    | 4 | Stage pending SAFE cutoff (cV, 1400–2400) |
| 0x04 | BATT_LIM_OP_SET_RECOVERY  | 4 | Stage pending SAFE→STANDBY recovery (cV, 1400–2900) |
| 0x05 | BATT_LIM_OP_SET_FILTER    | 4 | Stage pending voltage filter time constant (ms, 0–5000) |
| 0x06 | BATT_LIM_OP_SAVE          | 2 | Validate pending set + persist to flash + apply (STANDBY only) |
| 0x07 | BATT_LIM_OP_RESET_DEFAULTS| 2 | Restore + persist compile-time defaults (STANDBY only) |
| 0x08 | BATT_LIM_OP_QUERY         | 2 | Emit a `DIAG_BATTERY_LIMITS` (0x311) burst (read-only, no gate) |

**Validation & safety (STM32-side):**
- QUERY is always allowed and only emits diagnostics.
- All `SET_*`/`SAVE`/`RESET` require `SYS_STATE_STANDBY`; otherwise rejected with
  `ACK_BLOCKED_BY_SAFETY`.
- Every staged `SET` re-validates the **whole** pending set
  (`Safety_ValidateBatteryLimits()`): hard ranges plus coherence rules
  Warning > Cutoff, Limit > Cutoff, Recovery > Cutoff, Warning ≤ OV (30.00 V),
  Limit ≤ OV. A rejected value keeps the previous pending value and replies
  `ACK_INVALID` (reject-keep-previous, FASE 4).
- The voltage filter is an optional EMA whose time constant honours the call
  cadence; `Filter = 0` bypasses it so the compare is bit-identical to today. The
  invalid/zero/out-of-range fail-safe always runs on the **raw** reading; the
  filter only smooths the numeric threshold comparison.
- Every op replies with exactly one `CMD_ACK` (0x103, cmd_id_low = 0x10).

Source: `battlim_handle_service_cmd()` + `CAN_BatteryLimitsBurstUpdate()` in
`can_handler.c`, `Safety_{Get,Set,Validate}BatteryLimits()` in `safety_system.c`,
`battery_limits_store.c`.

---

## 5. Requested vs. Actual Signals

| Signal | Requested (ESP32 → STM32) | Actual (STM32 → ESP32) |
|--------|---------------------------|------------------------|
| Throttle | 0x100 byte 0: requested % (0–100) | No direct actual throttle message. The STM32 applies the validated value internally. |
| Steering angle | 0x101 bytes 0–1: requested angle (°/10) | 0x204 bytes 0–1: actual measured angle (°/10) |
| Drive mode | 0x102 byte 0: requested mode flags | 0x103 CMD_ACK: explicit acceptance or rejection with result code |
| Gear change | 0x102 byte 1: requested gear | 0x103 CMD_ACK: explicit acceptance or rejection with result code |
| Service command | 0x110: module enable/disable/restore | 0x103 CMD_ACK: explicit acceptance or rejection with result code |
| Wheel speed | Not applicable | 0x200: actual measured speeds |
| Motor current | Not applicable | 0x201: actual measured currents |
| Temperature | Not applicable | 0x202: actual measured temperatures |
| ABS/TCS state | Not applicable | 0x203: actual safety flags |

The ESP32 must never assume that a requested value has been applied. For on-demand commands (mode, gear, service), the ESP32 must wait for a CMD_ACK (0x103) before updating UI state. For continuous commands (throttle, steering), the ESP32 must read status messages for implicit confirmation.

---

## 6. Heartbeat Definition

### STM32 Heartbeat (0x001)

| Property | Value |
|----------|-------|
| Transmission rate | Every 100 ms (10 Hz) |
| Payload length | 4 bytes |
| Byte 0 | alive_counter — cyclic 0–255, intentional rollover |
| Byte 1 | system_state — see table below |
| Byte 2 | fault_flags — see bitmask table below |
| Byte 3 | error_code — current `Safety_Error_t` value (0–12; see §7) for HMI fault display |

**System states (byte 1):**

| Value | State | Meaning |
|-------|-------|---------|
| 0 | BOOT | Power-on, peripherals initializing. No commands accepted. |
| 1 | STANDBY | Ready, waiting for ESP32 heartbeat. No commands accepted. |
| 2 | ACTIVE | Normal operation. Commands are accepted and validated. |
| 3 | DEGRADED | Limp / degraded mode. Commands accepted with reduced power/speed limits (40% power, 50% speed). |
| 4 | SAFE | Fault detected. Actuators inhibited. Traction stopped, steering centered. |
| 5 | ERROR | Unrecoverable fault. Relays de-energized. Manual reset required. |

Source: `SystemState_t` in `safety_system.h`

**Fault flags bitmask (byte 2):**

| Bit | Mask | Name | Meaning |
|-----|------|------|---------|
| 0 | 0x01 | FAULT_CAN_TIMEOUT | ESP32 heartbeat not received within 250 ms |
| 1 | 0x02 | FAULT_TEMP_OVERLOAD | Motor temperature exceeds 90 °C |
| 2 | 0x04 | FAULT_CURRENT_OVERLOAD | Motor current exceeds 25 A |
| 3 | 0x08 | FAULT_ENCODER_ERROR | Sensor fault (includes encoder) |
| 4 | 0x10 | FAULT_WHEEL_SENSOR | Sensor fault (includes wheel speed) |
| 5 | 0x20 | FAULT_ABS_ACTIVE | ABS is currently intervening |
| 6 | 0x40 | FAULT_TCS_ACTIVE | TCS is currently intervening |
| 7 | 0x80 | FAULT_CENTERING | Steering centering failed |

Note: Bits 3 and 4 are both set when `SAFETY_ERROR_SENSOR_FAULT` is the active error. They are not independently assignable in the current implementation.

**Extended fault flags (internal, not transmitted in byte 2):**

Battery undervoltage fault flags are defined at bits 8–9 in `safety_system.h` for internal tracking. They are NOT transmitted in the CAN heartbeat byte 2 (which is limited to 8 bits). The battery undervoltage condition is reported to the ESP32 via the `error_code` field in STATUS_SAFETY (0x203 byte 2), using codes 9 (warning) and 10 (critical).

| Bit | Define | Meaning |
|-----|--------|---------|
| 8 | FAULT_BATT_UV_WARN | Battery bus voltage < 20.0 V (warning → DEGRADED) |
| 9 | FAULT_BATT_UV_CRIT | Battery bus voltage < 18.0 V (critical → SAFE) |

Source: `Safety_GetFaultFlags()` in `safety_system.c`, fault flag defines in `safety_system.h`

### ESP32 Heartbeat (0x011)

| Property | Value |
|----------|-------|
| Expected rate | Every 100 ms (10 Hz) |
| Timeout | 250 ms — if no message with ID 0x011 is received within this window, the STM32 triggers SAFE state |
| Payload parsing | None. The STM32 does not read the payload. Only frame arrival matters. |

### Timeout Behavior

1. The STM32 checks for ESP32 heartbeat timeout every 10 ms in the main loop via `Safety_CheckCANTimeout()`.
2. If `HAL_GetTick() - last_can_rx_time > 250 ms`:
   - `SAFETY_ERROR_CAN_TIMEOUT` is set.
   - System transitions to SAFE state.
   - `Safety_FailSafe()` is called: traction motors stopped, steering centered.
3. If heartbeat is restored while in SAFE state with CAN timeout error:
   - Error is cleared.
   - System transitions back to ACTIVE.
4. If heartbeat is received while in STANDBY with no faults:
   - System transitions to ACTIVE.

Source: `Safety_CheckCANTimeout()` in `safety_system.c`

---

## 7. Fault and State Signaling

### Safety Error Codes

| Code | Name | Trigger |
|------|------|---------|
| 0 | SAFETY_ERROR_NONE | Normal operation |
| 1 | SAFETY_ERROR_OVERCURRENT | Any motor current > 25 A |
| 2 | SAFETY_ERROR_OVERTEMP | Any motor temperature > 90 °C |
| 3 | SAFETY_ERROR_CAN_TIMEOUT | No ESP32 heartbeat for > 250 ms |
| 4 | SAFETY_ERROR_SENSOR_FAULT | Sensor plausibility check failed (temperature outside −40 to 125 °C, current negative or > 50 A, speed negative or > 25 km/h) |
| 5 | SAFETY_ERROR_MOTOR_STALL | Reserved (not implemented in current firmware) |
| 6 | SAFETY_ERROR_EMERGENCY_STOP | Emergency stop triggered |
| 7 | SAFETY_ERROR_WATCHDOG | Independent watchdog timeout (IWDG, ~4.1 s period) |
| 8 | SAFETY_ERROR_CENTERING | Steering centering failed during boot |
| 9 | SAFETY_ERROR_BATTERY_UV_WARNING | Battery bus voltage < 20.0 V → DEGRADED (40 % power limit). Recovery requires > 20.5 V (0.5 V hysteresis). |
| 10 | SAFETY_ERROR_BATTERY_UV_CRITICAL | Battery bus voltage < 18.0 V or sensor failure → SAFE (actuators inhibited). No auto-recovery; operator must recharge and reset. |
| 11 | SAFETY_ERROR_I2C_FAILURE | I2C bus locked / unrecoverable |
| 12 | SAFETY_ERROR_OBSTACLE | Obstacle emergency (distance < 500 mm confirmed for ≥ 200 ms). **Note:** obstacle CAN timeout and sensor/stale-data faults do NOT trigger this error; they set `OBS_STATE_SENSOR_FAULT` and apply `OBSTACLE_FAULT_SCALE` (0.3). Setting `SAFETY_ERROR_OBSTACLE` does NOT automatically enter SAFE state. |

Source: `Safety_Error_t` in `safety_system.h`, threshold defines in `safety_system.c`

### SAFE State Actions

When the system enters SAFE state, `Safety_FailSafe()` executes:

1. `Traction_EmergencyStop()` — all traction motor outputs are cut immediately.
2. `Steering_SetAngle(0.0f)` — steering is driven to center position.
3. Relays remain energized (dynamic motor braking is still available).

### ERROR State Actions

When the system enters ERROR state, `Safety_PowerDown()` executes:

1. `Traction_EmergencyStop()` — motors stopped.
2. `Relay_PowerDown()` — all relays de-energized (Direction → Traction → Main, in that order).
3. Manual hardware reset is required to exit ERROR state.

### What the ESP32 Must Do

- Monitor `system_state` (byte 1 of 0x001) at all times.
- Display fault information from `fault_flags` (byte 2 of 0x001) and `error_code` (byte 2 of 0x203) to the operator.
- Continue sending heartbeat at 100 ms intervals regardless of system state.
- Cease sending actuator commands when `system_state` is not ACTIVE (2). The STM32 will reject them, but sending unnecessary traffic is discouraged.

### What the ESP32 Must Not Do

- Must not attempt to override or work around a SAFE or ERROR state.
- Must not interpret the absence of a fault flag as permission to exceed physical limits.
- Must not rely on the STM32 being in ACTIVE state without verifying it in every heartbeat.

---

## 8. Invalid and Forbidden Behavior

### Messages That Are Ignored

- Any CAN ID not in the set {0x011, 0x100, 0x101, 0x102, 0x110, 0x208, 0x209} is hardware-filtered and never reaches the STM32 application.
- A `CMD_THROTTLE` (0x100) with DLC < 1 is silently dropped.
- A `CMD_STEERING` (0x101) with DLC < 2 is silently dropped.
- A `CMD_MODE` (0x102) with DLC < 1 is silently dropped. If DLC < 2, only byte 0 (mode flags) is processed; gear remains unchanged.
- All commands received while `system_state` is not ACTIVE or DEGRADED are rejected. Throttle returns 0; steering holds the current position; mode changes return false.

### Conditions That Force SAFE State

| Condition | Threshold | Source |
|-----------|-----------|--------|
| ESP32 heartbeat lost | > 250 ms | `Safety_CheckCANTimeout()` |
| Motor overcurrent | > 25 A on any channel | `Safety_CheckCurrent()` |
| Motor overtemperature | > 90 °C on any sensor | `Safety_CheckTemperature()` |
| Temperature out of range | < -40 °C or > 125 °C | `Safety_CheckSensors()` |
| Current out of range | < 0 A or > 50 A | `Safety_CheckSensors()` |
| Wheel speed out of range | < 0 or > 25 km/h | `Safety_CheckSensors()` |
| Battery critical undervoltage | < 18.0 V or sensor failure | `Safety_CheckBatteryVoltage()` |
| Obstacle emergency distance | < 500 mm confirmed for ≥ 200 ms (sets `SAFETY_ERROR_OBSTACLE` + forward block, but does NOT enter SAFE) | `Obstacle_Update()` |
| Obstacle CAN timeout | > 500 ms: if obstacle active → scale = 0.3, `OBS_STATE_SENSOR_FAULT` (no SAFE); if not active → scale = 1.0, `OBS_STATE_NO_SENSOR` (no SAFE) | `Obstacle_Update()` |
| Obstacle sensor unhealthy | sensor_health = 0 → scale = 0.3, `OBS_STATE_SENSOR_FAULT` (no SAFE) | `Obstacle_Update()` |
| Obstacle stale data | Rolling counter frozen ≥ 3 frames → scale = 0.3, `OBS_STATE_SENSOR_FAULT` (no SAFE) | `Obstacle_Update()` |

### Conditions That Force ERROR State (Unrecoverable)

| Condition | Effect |
|-----------|--------|
| Emergency stop triggered | `Safety_EmergencyStop()` — motors stopped, relays de-energized, state = ERROR |
| IWDG watchdog timeout | Hardware reset (~4.1 s period, fed in main loop) |
| `Error_Handler()` | Interrupts disabled, all GPIOC outputs driven low, infinite loop |

### What the ESP32 Must Not Assume

- A sent command does not imply it was executed. Always verify via status messages or ACK.
- The throttle value received by the STM32 may be clamped (0–100%), zeroed (ABS active), or halved (TCS active).
- The steering angle may be clamped (±45°) or rate-limited (200°/s max).
- A mode change may be rejected — the ESP32 must wait for CMD_ACK (0x103) before updating UI.
- The STM32 may transition to SAFE at any time without prior warning.
- ABS and TCS interventions happen without ESP32 consent and override the requested throttle.
- ACK timeout (200 ms) does not imply command failure — it may indicate CAN bus congestion. The ESP32 must not retry indefinitely.

---

## 9. Command Validation Summary

| Command | Validation | Limits | ACK | Source |
|---------|------------|--------|-----|--------|
| Throttle (0x100) | Clamped to 0–100%. Zeroed if ABS active. Halved if TCS active. Rejected if not ACTIVE. | 0.0–100.0 % | No (implicit via status) | `Safety_ValidateThrottle()` |
| Steering (0x101) | Clamped to ±45°. Rate-limited to 200°/s. Returns current angle if not ACTIVE. | ±45.0°, 200°/s max rate | No (implicit via 0x204) | `Safety_ValidateSteering()` |
| Mode (0x102) | Rejected if not ACTIVE or if average wheel speed > 1 km/h. | Speed < 1.0 km/h | Yes (0x103) | `Safety_ValidateModeChange()` |
| Service (0x110) | Module enable/disable/factory restore. Critical modules cannot be disabled. | Module ID < MODULE_COUNT | Yes (0x103) | `ServiceMode_*()` |

---

## 10. Versioning and Stability

This document describes the CAN protocol as implemented in the current firmware. Revision 1.3 adds command acknowledgment (CMD_ACK 0x103) while maintaining backward compatibility with previous revisions — existing messages are unchanged. The 3-bit `relay_status` layout is preserved: bit 0 is now reserved/0, but no frame size, ID, or bit positions changed.

| Property | Value |
|----------|-------|
| Contract revision | 1.3 |
| Previous revision | 1.2 |
| Contract status | ACTIVE |
| Change policy | Any modification to CAN IDs, payloads, timing, or behavior requires a new contract revision number and a corresponding firmware release tag. |
| Backward compatibility | Revision 1.4 is backward-compatible with 1.3, 1.2, 1.1, and 1.0. No existing CAN IDs, payload layouts, or timing changed. New `DIAG_DEBOUNCE` (0x306) and `DIAG_DEBOUNCE_STEER` (0x307) are diagnostic-only and additive — ESP32 firmware without DEBOUNCE_DIAG submenu silently ignores the new IDs (`default: break` in `can_rx.cpp`). The 2026-04-23 firmware hardening retains the 3-bit `relay_status` layout — rev 1.3+ consumers continue to decode byte 5 unchanged (the former MAIN bit is simply always 0 now). CMD_ACK (0x103) added in 1.3 is additive only — ESP32 firmware without ACK support will silently ignore the message. |

The source files that define this contract are:

| File | Content |
|------|---------|
| `Core/Inc/can_handler.h` | CAN ID definitions, timeout constant, function prototypes |
| `Core/Src/can_handler.c` | TX/RX implementation, RX filters, message processing |
| `Core/Inc/safety_system.h` | State machine, fault flags, error codes, validation prototypes |
| `Core/Src/safety_system.c` | Safety logic, thresholds, command validation, fail-safe actions |
| `Core/Src/main.c` | Main loop timing, status message encoding and transmission |
| `Core/Inc/main.h` | Hardware pin definitions, sensor counts |
| `esp32/include/can_ids.h` | ESP32-side CAN ID definitions, timing constants, state/fault enums |
| `esp32/src/can_rx.cpp` | ESP32-side CAN RX decoding |
