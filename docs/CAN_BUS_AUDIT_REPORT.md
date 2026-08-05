# CAN Bus Communication Audit Report

**Date:** 2026-02-28
**Firmware:** STM32G474RE (`Core/Src/can_handler.c`, `Core/Src/safety_system.c`) + ESP32-S3 (`esp32/src/main.cpp`, `esp32/src/can_rx.cpp`)
**Scope:** Complete firmware-based audit of CAN bus communication between ESP32-S3 and STM32G474RE

---

## 1. CAN Configuration

### 1.1 STM32G474RE (Safety Authority)

| Parameter | Value | Source |
|-----------|-------|--------|
| Peripheral | FDCAN1 | `main.c` line 520 |
| Mode | **FDCAN_MODE_NORMAL** | `main.c` line 522 |
| Frame Format | FDCAN_FRAME_CLASSIC (CAN 2.0A) | `main.c` line 521 |
| Clock Source | APB1 = 170 MHz (FDCAN_CLOCK_DIV1) | `main.c` line 523 |
| Nominal Prescaler | 17 | `main.c` line 524 |
| Time Seg 1 | 14 TQ | `main.c` line 526 |
| Time Seg 2 | 5 TQ | `main.c` line 527 |
| Sync Jump Width | 1 TQ | `main.c` line 525 |
| **Bitrate** | **500 kbps** (170 MHz / 17 / 20 = 500,000) | Calculated |
| Auto Retransmission | **ENABLE** | `main.c` line 528 |
| Protocol Exception | DISABLE | `main.c` line 530 |
| ID Type | FDCAN_STANDARD_ID (11-bit) | `can_handler.c` line 79 |
| FD Bit-Rate Switch | FDCAN_BRS_OFF | `can_handler.c` line 83 |
| TX pins | PA12 (FDCAN1_TX, AF9) | `project_config.h` |
| RX pins | PA11 (FDCAN1_RX, AF9) | `project_config.h` |

### 1.2 ESP32-S3 (HMI)

| Parameter | Value | Source |
|-----------|-------|--------|
| Driver | ESP32-TWAI-CAN (TWAI peripheral) | `main.cpp` line 14 |
| Bitrate | **500 kbps** | `main.cpp` line 295: `ESP32Can.convertSpeed(500)` |
| TX pin | GPIO4 | `main.cpp` line 35 |
| RX pin | GPIO5 | `main.cpp` line 36 |
| RX Queue | 5 frames | `main.cpp` line 292 |
| TX Queue | 5 frames | `main.cpp` line 293 |
| ID Type | Standard (11-bit) | `frame.extd = 0` throughout |
| Frame format | Classic CAN | `data_length_code ≤ 8` throughout |

### 1.3 Configuration Match: ✅ PASS

Both sides are configured for:
- **500 kbps** — bitrates match
- **Standard 11-bit IDs** — no extended ID usage
- **Classic CAN (not FD)** — no bit-rate switching
- **Normal mode** — not loopback or silent

---

## 2. Message ID Consistency

### 2.1 ID Definitions

| CAN ID | Name | Defined in `can_ids.h` | Defined in `can_handler.h` | Match |
|--------|------|------------------------|----------------------------|-------|
| 0x001 | HEARTBEAT_STM32 | ✅ | ✅ | ✅ |
| 0x011 | HEARTBEAT_ESP32 | ✅ | ✅ | ✅ |
| 0x100 | CMD_THROTTLE | ✅ | ✅ | ✅ |
| 0x101 | CMD_STEERING | ✅ | ✅ | ✅ |
| 0x102 | CMD_MODE | ✅ | ✅ | ✅ |
| 0x103 | CMD_ACK | ✅ | ✅ | ✅ |
| 0x110 | SERVICE_CMD | ✅ | ✅ | ✅ |
| 0x120 | CMD_LED | ✅ | ✅ | ✅ |
| 0x200 | STATUS_SPEED | ✅ | ✅ | ✅ |
| 0x201 | STATUS_CURRENT | ✅ | ✅ | ✅ |
| 0x202 | STATUS_TEMP | ✅ | ✅ | ✅ |
| 0x203 | STATUS_SAFETY | ✅ | ✅ | ✅ |
| 0x204 | STATUS_STEERING | ✅ | ✅ | ✅ |
| 0x205 | STATUS_TRACTION | ✅ | ✅ | ✅ |
| 0x206 | STATUS_TEMP_MAP | ✅ | ✅ | ✅ |
| 0x207 | STATUS_BATTERY | ✅ | ✅ | ✅ |
| 0x208 | OBSTACLE_DISTANCE | ✅ | ✅ | ✅ |
| 0x209 | OBSTACLE_SAFETY | ✅ | ✅ | ✅ |
| 0x20A | STATUS_LIGHTS | ✅ | ✅ | ✅ |
| 0x300 | DIAG_ERROR | ✅ | ✅ | ✅ |
| 0x301 | SERVICE_FAULTS | ✅ | ✅ | ✅ |
| 0x302 | SERVICE_ENABLED | ✅ | ✅ | ✅ |
| 0x303 | SERVICE_DISABLED | ✅ | ✅ | ✅ |

**Result: ✅ All 23 IDs match between ESP32 and STM32 headers.**

### 2.2 DLC Consistency

| CAN ID | Name | STM32 TX DLC | ESP32 RX min DLC | ESP32 TX DLC | STM32 RX min DLC | Match |
|--------|------|-------------|------------------|-------------|------------------|-------|
| 0x001 | HEARTBEAT_STM32 | 5 | 4 (accepts ≥4) | — | — | ✅ |
| 0x011 | HEARTBEAT_ESP32 | — | — | 1 | 1 (accepts ≥1) | ✅ |
| 0x100 | CMD_THROTTLE | — | — | 1 | 1 | ✅ |
| 0x101 | CMD_STEERING | — | — | 2 | 2 | ✅ |
| 0x102 | CMD_MODE | — | — | 2 | 1 (accepts ≥1) | ✅ |
| 0x103 | CMD_ACK | 3 | 3 | — | — | ✅ |
| 0x110 | SERVICE_CMD | — | — | 2 | 1 (accepts ≥1) | ✅ |
| 0x120 | CMD_LED | — | — | 2 | 1 (accepts ≥1) | ✅ |
| 0x200 | STATUS_SPEED | 8 | 8 | — | — | ✅ |
| 0x201 | STATUS_CURRENT | 8 | 8 | — | — | ✅ |
| 0x202 | STATUS_TEMP | 5 | 5 | — | — | ✅ |
| 0x203 | STATUS_SAFETY | 6 | 6 | — | — | ✅ |
| 0x204 | STATUS_STEERING | 3 | 3 | — | — | ✅ |
| 0x205 | STATUS_TRACTION | 4 | 4 | — | — | ✅ |
| 0x206 | STATUS_TEMP_MAP | 5 | 5 | — | — | ✅ |
| 0x207 | STATUS_BATTERY | 4 | 4 | — | — | ✅ |
| 0x208 | OBSTACLE_DISTANCE | — | — | 5 | 5 | ✅ |
| 0x209 | OBSTACLE_SAFETY | — | — | 4 | 3 (accepts ≥3) | ✅ |
| 0x20A | STATUS_LIGHTS | 2 | 1 (accepts ≥1) | — | — | ✅ |
| 0x300 | DIAG_ERROR | 2 or 8 | 2 | — | — | ✅ |
| 0x301–0x303 | SERVICE_* | 4 | 4 | — | — | ✅ |

**Result: ✅ All DLCs are compatible. Receivers always check `min DLC ≤ actual DLC`.**

### 2.3 Byte Order

**All multi-byte fields use little-endian encoding:**
- STM32 pack: `data[0] = val & 0xFF; data[1] = (val >> 8) & 0xFF;` (LE)
- ESP32 unpack: `readU16LE()` / `readS16LE()` / `readU32LE()` (LE)

**Result: ✅ Byte order is consistent (little-endian throughout).**

### 2.4 RX Filter Coverage

| ESP32→STM32 ID | Filter | Accepted | Verified in `CAN_ProcessMessages()` |
|----------------|--------|----------|--------------------------------------|
| 0x011 | Filter 0 (Dual) | ✅ | ✅ `case CAN_ID_HEARTBEAT_ESP32` |
| 0x100 | Filter 1 (Range 0x100–0x102) | ✅ | ✅ `case CAN_ID_CMD_THROTTLE` |
| 0x101 | Filter 1 | ✅ | ✅ `case CAN_ID_CMD_STEERING` |
| 0x102 | Filter 1 | ✅ | ✅ `case CAN_ID_CMD_MODE` |
| 0x110 | Filter 2 (Range 0x110–0x120) | ✅ | ✅ `case CAN_ID_SERVICE_CMD` |
| 0x120 | Filter 2 | ✅ | ✅ `case CAN_ID_CMD_LED` |
| 0x208 | Filter 3 (Range 0x208–0x209) | ✅ | ✅ `case CAN_ID_OBSTACLE_DISTANCE` |
| 0x209 | Filter 3 | ✅ | ✅ `case CAN_ID_OBSTACLE_SAFETY` |

**Result: ✅ All ESP32 messages are accepted by STM32 hardware filters.**

---

## 3. Robustness

### 3.1 CAN Bus-Off Recovery

**Implementation:** `CAN_CheckBusOff()` in `can_handler.c` (called every 10 ms from main loop)

| Feature | Status | Details |
|---------|--------|---------|
| Detection | ✅ | Polls `HAL_FDCAN_GetProtocolStatus()` for `psr.BusOff` |
| Recovery | ✅ | Non-blocking: Stop → DeInit → Init → Filters → Start |
| Retry interval | 500 ms | `CAN_BUSOFF_RETRY_INTERVAL_MS` |
| Max retries | 10 | `CAN_BUSOFF_MAX_RETRIES` |
| Safety response | ✅ | Sets `SAFETY_ERROR_CAN_BUSOFF`, transitions to **LIMP_HOME** (not SAFE) |
| Statistics | ✅ | `can_stats.busoff_count` tracked |
| No blocking | ✅ | Timestamp-based, watchdog continues feeding |

**Result: ✅ Bus-off handling is robust and non-blocking.**

### 3.2 Auto-Retransmission

| Side | Enabled | Source |
|------|---------|--------|
| STM32 | ✅ | `AutoRetransmission = ENABLE` in `main.c` |
| ESP32 | ✅ | TWAI driver default (hardware auto-retransmission) |

**Result: ✅ Both sides auto-retransmit failed frames.**

### 3.3 Heartbeat Timeout

| Parameter | STM32 Side | ESP32 Side |
|-----------|------------|------------|
| Timeout | 250 ms (`CAN_TIMEOUT_MS`) | Application-level (checks `heartbeat.timestampMs`) |
| Check interval | 10 ms (safety loop) | ~16 ms (loop iteration) |
| Response on timeout | → **LIMP_HOME** | Inhibits motion commands |
| Recovery | CAN heartbeat restored → **ACTIVE** | Resumes normal operation |

### 3.4 ESP32 Heartbeat Counter Freeze Detection

**Implementation:** `can_handler.c` lines 43–53, 566–606

The STM32 validates that the ESP32's rolling counter (byte 0 of 0x011) is actually advancing:
- If the counter value stays the same for **5 consecutive frames** (500 ms), the heartbeat is NOT counted as alive
- `Safety_UpdateCANRxTime()` is not called → CAN timeout fires → **LIMP_HOME**
- When counter resumes changing → freeze clears → normal liveness restored

**Purpose:** Detects a "zombie" ESP32 where the main task is frozen but a timer ISR keeps sending heartbeats.

**Result: ✅ Frozen-counter detection prevents zombie ESP32 from maintaining false liveness.**

### 3.5 STM32 Heartbeat Counter Freeze Detection (ESP32 side)

**Implementation:** `main.cpp` lines 134–143, 379–408

Mirror logic on the ESP32:
- Tracks STM32 heartbeat counter (byte 0 of 0x001)
- After 5 identical counters → marks STM32 as non-alive → inhibits motion commands
- ESP32 logs `[SAFETY] STM32 heartbeat counter frozen`

**Result: ✅ Bidirectional frozen-counter detection.**

### 3.6 Obstacle Data Timeout

| Parameter | Value | Source |
|-----------|-------|--------|
| Timeout | 500 ms | `CAN_TIMEOUT_OBSTACLE_MS = 500` |
| Check | Every 10 ms in `Obstacle_Update()` | `safety_system.c` |
| Response | `obstacle_scale → 1.0` (no restriction) + state → `OBS_STATE_NO_SENSOR` | `safety_system.c` |
| Safety net | LIMP_HOME speed cap (5 km/h) provides physical safety without obstacle data | `safety_system.h` |

**Result: ✅ Obstacle data loss defaults to safe behavior.**

### 3.7 What Happens If ESP32 Stops Sending Data

**Sequence of events:**

1. **T+0 ms:** ESP32 stops sending (crash, power loss, cable disconnect)
2. **T+250 ms:** STM32 detects CAN timeout → `Safety_SetError(SAFETY_ERROR_CAN_TIMEOUT)`
3. **T+250 ms:** State transitions:
   - ACTIVE → **LIMP_HOME**
   - DEGRADED → **LIMP_HOME**
   - STANDBY → **LIMP_HOME** (if boot validation passed)
4. **T+250 ms onward:** Vehicle operates in LIMP_HOME mode:
   - Local pedal input only (no CAN throttle)
   - 20% torque limit
   - 5 km/h speed cap
   - Steering fully operational
   - No torque vectoring (Ackermann disabled)
5. **T+500 ms:** Obstacle CAN timeout → `obstacle_scale = 1.0` (no restriction — LIMP_HOME speed cap provides safety)

**Result: ✅ The vehicle remains mobile and safe without CAN/ESP32.**

---

## 4. Functional Safety

### 4.1 LIMP_HOME on CAN Loss

**YES.** The STM32 enters LIMP_HOME (not SAFE) when CAN is lost:

```c
// safety_system.c, Safety_CheckCANTimeout()
if (system_state == SYS_STATE_ACTIVE || system_state == SYS_STATE_DEGRADED) {
    Safety_SetState(SYS_STATE_LIMP_HOME);
}
```

Design philosophy: *"Communication loss is NOT a hazard — the vehicle can still operate at reduced capability with local pedal input."*

LIMP_HOME limits:
- Torque: 20% (`LIMP_HOME_TORQUE_LIMIT_FACTOR = 0.20f`)
- Speed: 5 km/h (`LIMP_HOME_SPEED_LIMIT_KMH = 5.0f`)
- Ramp: 10%/s (`LIMP_HOME_RAMP_RATE_PCT_PER_S = 10.0f`)

### 4.2 Range Validation on Received Data

| Data | Validation | Source |
|------|-----------|--------|
| Throttle | `Safety_ValidateThrottle()` — clamps 0–100%, rate-limited | `safety_system.c` |
| Steering | `Safety_ValidateSteering()` — rate-limited, range-clamped | `safety_system.c` |
| Mode change | `Safety_ValidateModeChange()` — speed gate (< 1 km/h) | `safety_system.c` |
| Gear change | Speed gate (avg speed ≤ 1 km/h) + range check | `can_handler.c` line 662 |
| Obstacle distance | Plausibility check (max approach rate), stuck-sensor detection | `safety_system.c` |
| Obstacle zone | Clamped to 0–5 | `safety_system.c` line 1415 |
| Obstacle counter | Stale-data detection (rolling counter freeze) | `safety_system.c` line 1407 |

**Result: ✅ All received CAN data is validated before use.**

### 4.3 Can a Corrupt Frame Activate Motors Improperly?

**NO.** Multiple independent barriers prevent this:

1. **CAN CRC:** CAN 2.0A has hardware CRC-15 — bit errors are detected at the physical layer
2. **Hardware filters:** Only white-listed IDs reach the application
3. **DLC checks:** All handlers verify minimum DLC before parsing
4. **Safety validation:** `Safety_ValidateThrottle()` and `Safety_ValidateSteering()` independently clamp values
5. **State gate:** Commands rejected unless system is in ACTIVE or DEGRADED
6. **Startup inhibit:** Power-On Movement Prevention blocks all motor commands until pedal released
7. **Obstacle backstop:** Independent distance-based torque limiting

**Even in the worst case** (CAN CRC miss, which is < 10⁻¹⁴ probability):
- A throttle byte of 255 is clamped to 100% by `Safety_ValidateThrottle()`
- That 100% is further clamped by degraded-mode power limits if applicable
- The obstacle scale further reduces output if an obstacle is present
- ABS/TCS per-wheel limiters prevent individual wheel lockup or spin

**Result: ✅ No single corrupted frame can cause unintended motor activation.**

### 4.4 State Machine Safety

```
BOOT → STANDBY → ACTIVE ⇄ DEGRADED → SAFE → ERROR
                ↘ LIMP_HOME ↗
```

| Transition | Trigger | Rationale |
|-----------|---------|-----------|
| ACTIVE → LIMP_HOME | CAN timeout (250 ms) | Communication loss, not a hazard |
| ACTIVE → DEGRADED | Non-critical fault (temp warning, sensor glitch) | Reduced limits, still drivable |
| ACTIVE → SAFE | Critical fault (overcurrent, inverter, watchdog) | Real hardware danger |
| DEGRADED → SAFE | Persistent fault (≥3 consecutive errors) | Escalation |
| any → ERROR | Unrecoverable (emergency stop, watchdog timeout) | Power-down required |
| LIMP_HOME → ACTIVE | CAN restored + steering calibrated | Normal operation resumes |

**Result: ✅ State machine is conservative. CAN loss → LIMP_HOME (mobile). Hardware danger → SAFE (stopped).**

---

## 5. Electrical Considerations

### 5.1 CAN Transceiver: TJA1051T/3

| Parameter | Value | Source |
|-----------|-------|--------|
| Part number | TJA1051T/3 | `CAN_PROTOCOL.md`, `ESP32_STM32_CAN_CONNECTION.md` |
| Quantity | 2 (one per MCU) | `ESP32_STM32_CAN_CONNECTION.md` |
| VCC | **5 V** (4.5–5.5 V) | Datasheet NXP |
| VIO | **3.3 V** (connected to MCU supply) | Datasheet NXP — obligatorio para niveles lógicos 3.3 V |
| S pin (Silent mode) | Connected to GND (normal mode) | `ESP32_STM32_CAN_CONNECTION.md` |
| Decoupling | 100 nF ceramic close to VCC | `ESP32_STM32_CAN_CONNECTION.md` |

> ⚠️ **CRÍTICO:** Pin 5 (VIO) **DEBE** conectarse a 3.3 V en ambos nodos. Si se deja flotante, RXD sigue VCC (5 V) y **destruye el GPIO5 del ESP32-S3** (máx. 3.6 V).

**Result: ✅ TJA1051T/3 properly configured for normal (non-silent) mode. VIO = 3.3 V required.**

### 5.2 Bus Termination (120Ω)

| Position | Resistor | Source |
|----------|----------|--------|
| STM32 end | 120Ω between CANH and CANL | `ESP32_STM32_CAN_CONNECTION.md` line 62 |
| ESP32 end | 120Ω between CANH and CANL | `ESP32_STM32_CAN_CONNECTION.md` line 78 |
| Combined impedance | ~60Ω (expected) | ISO 11898 compliant |

**Verification method:** Measure between CANH and CANL with bus unpowered → should read ~60Ω.

**Result: ✅ Correct dual-termination per ISO 11898.**

### 5.3 Common Ground

Both transceivers require a common GND reference:
- STM32 TJA1051 GND and ESP32 TJA1051 GND must be connected
- Specified in `ESP32_STM32_CAN_CONNECTION.md` line 29/39

**Result: ✅ Common ground is documented and required.**

### 5.4 Topology

Point-to-point (only 2 nodes on the bus) — simplest topology, no addressing conflicts possible.

---

## 6. Conclusions

### Is the CAN communication robust and safe?

**YES.** The implementation includes:
- ✅ Matching bitrate (500 kbps) on both sides
- ✅ Consistent message IDs and DLCs
- ✅ Little-endian byte order throughout
- ✅ Hardware RX filters (white-list only)
- ✅ Bus-off detection and non-blocking recovery
- ✅ Auto-retransmission enabled
- ✅ Heartbeat timeout (250 ms) → LIMP_HOME
- ✅ Bidirectional frozen-counter detection
- ✅ Obstacle data timeout (500 ms) with safe defaults
- ✅ Multi-layer command validation (state gate + range check + rate limit)
- ✅ Power-On Movement Prevention latch
- ✅ Proper bus termination (2× 120Ω)

### Risk of desynchronization?

**LOW.** Potential scenarios and mitigations:

| Scenario | Mitigation |
|----------|-----------|
| ESP32 crash / restart | STM32 detects timeout in 250 ms → LIMP_HOME. Recovery on heartbeat restore. |
| STM32 crash / restart | ESP32 detects frozen counter in 500 ms → inhibits commands. Gear re-sync on startup_inhibit transition. |
| CAN bus error burst | Auto-retransmission + bus-off recovery (up to 10 retries at 500 ms intervals) |
| Stale obstacle data | Rolling counter freeze detection (3 consecutive identical → reject) |
| Zombie ESP32 (ISR alive, task dead) | Heartbeat counter freeze detection (5 identical → timeout) |

### False state conditions?

| Condition | Can it happen? | Details |
|-----------|----------------|---------|
| False INVALID (obstacle) | Possible | If sensor loses power or has wiring issue. Mitigated: scale → 1.0 (no restriction), LIMP_HOME speed cap provides safety. |
| False LIMP_HOME | Possible | If CAN bus has noise/EMI causing repeated frame loss. Mitigated: auto-retransmission + bus-off recovery. LIMP_HOME still allows mobility. |
| False motor blockage | Very unlikely | Requires obstacle_scale = 0 AND forward gear. Obstacle emergency requires confirmed obstacle for 200 ms temporal hysteresis. Speed-dependent thresholds prevent false emergency stops at speed. |
| False SAFE | Very unlikely | Only triggered by real hardware danger (overcurrent > 25A, temp > 80°C, watchdog). Never by CAN loss alone. |

---

## 7. Documentation Fixes Applied

This audit identified and corrected the following documentation discrepancies:

1. **CAN_CONTRACT_FINAL.md Filter 2:** Was "Dual (exact match) 0x110", corrected to "Range 0x110–0x120" (matches code that includes CMD_LED 0x120)
2. **CAN_CONTRACT_FINAL.md HEARTBEAT_STM32 DLC:** Was 4, corrected to **5** (code sends 5 bytes: counter, state, faults, error, status_flags)
3. **CAN_CONTRACT_FINAL.md HEARTBEAT_ESP32 DLC:** Was "—", corrected to **1** (code sends 1 byte: rolling counter)
4. **CAN_CONTRACT_FINAL.md OBSTACLE_SAFETY DLC:** Was 8, corrected to **4** (code sends 4 bytes: zone, status, stuck, reserved)
5. **CAN_CONTRACT_FINAL.md missing CMD_LED:** Added 0x120 CMD_LED to §3.1 message list
6. **CAN_CONTRACT_FINAL.md missing STATUS_LIGHTS:** Added 0x20A STATUS_LIGHTS to §3.2 message list
7. **can_ids.h OBSTACLE_SAFETY comment:** Was "DLC 8", corrected to "DLC 4"

---

## References

- `Core/Src/can_handler.c` — STM32 CAN TX/RX implementation
- `Core/Src/safety_system.c` — State machine, obstacle processing, CAN timeout
- `Core/Src/main.c` — FDCAN peripheral init, main loop timing
- `Core/Inc/can_handler.h` — STM32 CAN ID definitions
- `Core/Inc/safety_system.h` — State machine, fault flags, obstacle thresholds
- `esp32/include/can_ids.h` — ESP32 CAN ID definitions (frozen contract mirror)
- `esp32/src/main.cpp` — ESP32 CAN init, heartbeat TX, STM32 liveness monitoring
- `esp32/src/can_rx.cpp` — ESP32 CAN RX decoder
- `esp32/src/can/can_obstacle.cpp` — ESP32 obstacle CAN TX (0x208, 0x209)
- `esp32/src/sensors/obstacle_sensor.cpp` — TOFSense-M UART driver
- `docs/CAN_CONTRACT_FINAL.md` — Frozen CAN protocol contract (rev 1.3)
- `docs/ESP32_STM32_CAN_CONNECTION.md` — Physical CAN connection guide
- `docs/CAN_PROTOCOL.md` — CAN protocol parameters
