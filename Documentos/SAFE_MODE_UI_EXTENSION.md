# SAFE MODE UI Extension — Passive Visualization

## Statement

**No safety logic modified.** This extension is purely additive, passive UI
rendering. No state machine transitions, CAN transmissions, motor control,
relay control, or any functional behavior has been changed.

## What Was Added

The SAFE MODE screen (shown when `system_state = SAFE`) now displays
additional read-only operator information alongside the existing telemetry:

| Feature | Description |
|---------|-------------|
| **Gear Display** | Current gear position (P/R/N/D1/D2) shown as highlighted bar at bottom |
| **Steering Visual** | Direction indicator (`<< LEFT`, `RIGHT >>`, `\| CENTER`) added to numeric angle |
| **Obstacle Sensor Bar** | Color-coded proximity bar with distance and state (CLEAR/NEAR/DANGER) |
| **LED System Status** | Front/Rear relay state (ON/OFF) and turn signal state (LEFT/RIGHT/HAZARD/OFF) |
| **Relay Status** | T D indicators (TRACTION/DIRECTION) with color coding |
| **I2C Bus Diagnostic** | Live TCA9548A (mux) presence + per-channel INA226 health + fail/recovery counters (helps locate I2C wiring faults) |

## Data Sources

| Data | Source | File:Line |
|------|--------|-----------|
| Gear position | `shifter::getGearRaw()` (MCP23017 I2C) | `esp32/src/shifter_input.h:51` |
| Steering angle | `VehicleData.steering().angleRaw` | `esp32/src/vehicle_data.h:86` |
| Obstacle distance | `VehicleData.obstacle().distanceCm` | `esp32/src/vehicle_data.h:152` |
| Front/Rear LED relay | `VehicleData.lights().frontRelayOn/rearRelayOn` | `esp32/src/vehicle_data.h:162-163` |
| Turn signal | `led_ctrl::getTurnSignal()` (local ESP32 state) | `esp32/src/led_controller.h:148` |
| Relay status | `VehicleData.heartbeat().relayStatus` | `esp32/src/vehicle_data.h:45` |
| I2C bus diagnostic | `VehicleData.i2cDiag()` (CAN 0x309 DIAG_I2C, STM32 `sensor_manager.c`) | `esp32/src/vehicle_data.h` (`I2cDiagData`), `esp32/src/can_rx.cpp` (`decodeI2cDiag`) |

## Layout Description

```
┌─────────────────────────────────────────────────┐ Y=0
│              SAFE MODE (amber banner)            │
├─────────────────────────────────────────────────┤ Y=40
│ Actuators inhibited - Controls disabled          │ Y=46
│ FAULT FLAGS: [value]                             │ Y=62-86
│ ERROR CODE:  [value]                             │ Y=90-114
├─────────────────────────────────────────────────┤ Y=118
│ READ-ONLY TELEMETRY                              │
│ SPEED:    FL [v]  FR [v]  RL [v]  RR [v]        │ Y=126-152
│ CURRENT:  FL [v]  FR [v]  RL [v]  RR [v]        │ Y=154-180
│ TEMP:     FL [v]  FR [v]  RL [v]  RR [v] AMB [v]│ Y=182-208
│ STEERING: -12.5° << LEFT                         │ Y=214  (NEW: visual indicator)
│                                                  │
│ OBSTACLE: [████████░░░░░░░░░░░░] 1.50m CLEAR    │ Y=246  (NEW)
│ LIGHTS:   F:ON  R:OFF  T:LEFT                    │ Y=266  (NEW)
├─────────────────────────────────────────────────┤ Y=284
│ [P] [R] [N] [D1] [D2]           M T D           │ Y=288  (NEW: gear + relay)
└─────────────────────────────────────────────────┘ Y=320
```

## Fallback Behavior

| Condition | Behavior |
|-----------|----------|
| No CAN data at all | Gear shows from physical shifter (I2C), all CAN fields show defaults (0) |
| Partial CAN only | Each tile renders independently; missing fields show zero/default |
| No obstacle sensor | Shows "NO DATA" in gray, bar empty |
| Shifter I2C disconnected | `getGearRaw()` returns last known value or default (N) |
| Turn signal not set | Shows "T:OFF" in gray |
| Relay status byte = 0 | All M T D indicators show gray (OFF) |

## Obstacle Thresholds

Uses existing thresholds from `ui_common.h::proximityColor()`:
- **CLEAR** (green): > 300 cm (3.0 m)
- **NEAR** (amber): 80–300 cm
- **DANGER** (red): < 80 cm

## Steering Direction Thresholds

- `angle < -150` (0.1° units, i.e. -15.0°) → `<< LEFT` (cyan)
- `angle > +150` (0.1° units, i.e. +15.0°) → `RIGHT >>` (cyan)
- otherwise → `| CENTER` (white)

## Modified Files

| File | Change |
|------|--------|
| `esp32/src/screens/safe_screen.h` | Added new tile enums (STILE_OBSTACLE, STILE_LED_STAT, STILE_GEAR, STILE_RELAY), new member variables for gear/obstacle/lights/relay/turn signal |
| `esp32/src/screens/safe_screen.cpp` | Extended `update()` to read new data sources, extended `draw()` with new tile rendering for all 5 features |
| `esp32/src/ui/ui_config.h` | Added layout constants (STILE_STEER_VIS_Y, STILE_OBSTACLE_Y, STILE_LED_STATUS_Y, STILE_GEAR_BAR_Y, STILE_RELAY_X, STILE_OBS_BAR_*, PAD_SAFE_*) |
| `esp32/src/led_controller.h` | Added `getTurnSignal()` read-only getter |
| `esp32/src/led_controller.cpp` | Implemented `getTurnSignal()` returning current turn signal state |
| `Documentos/SAFE_MODE_UI_EXTENSION.md` | This documentation file |

## Proof: No Functional Paths Modified

1. **No state machine changes**: `SafeScreen` only calls `onEnter/onExit/update/draw` — the same lifecycle as before. No new state transitions added.
2. **No CAN transmission**: No `canSend()` or equivalent calls added. All data is read-only from existing `VehicleData` getters.
3. **No motor/relay control**: No PWM, GPIO, or relay control functions called. Only rendering functions (`tft.drawString`, `tft.fillRect`, etc.).
4. **No new timers**: No `millis()` calls added. `frameTimeMs` parameter is unused (same as before).
5. **No dynamic allocation**: All buffers are stack-allocated (`char buf[...]`). Tile engine is static.
6. **No blocking calls**: All rendering is immediate (single TFT SPI transaction per tile).
7. **LED controller getter**: `getTurnSignal()` simply returns the existing `currentTurnSignal` static variable — zero side effects.

---

## Gear Data Source Consistency Verification

**Date**: 2026-04-15
**Verdict**: ✅ CONSISTENT — No fix required.

### 1. All Gear Data Sources Identified

| Location | Usage | File:Line |
|----------|-------|-----------|
| `shifter::getGearRaw()` | Physical MCP23017 I2C read (cached in static `currentGear_`) | `esp32/src/shifter_input.cpp:188-191` |
| `shifter::getGear()` | Same data, typed as `shifter::Gear` enum | `esp32/src/shifter_input.cpp:183-186` |
| `shifter::update()` | Polls MCP23017 Port A, updates `currentGear_` | `esp32/src/shifter_input.cpp:120-181` |

**There is NO CAN-based gear reception path.** Verified:
- `esp32/src/can_rx.cpp` — zero references to "gear", "Gear", "CMD_GEAR", or "0x102"
- `esp32/src/vehicle_data.h` — **no gear field exists** in `VehicleData`
- CAN gear flow is **ESP32 → STM32 only** (ESP32 sends gear to STM32 via `CMD_MODE 0x102`)

### 2. Authoritative Source

**SINGLE source of truth: `shifter::currentGear_`** (static variable in `shifter_input.cpp`)
- Populated by `shifter::update()` via MCP23017 I2C Port A read
- Exposed via `shifter::getGearRaw()` (uint8_t) and `shifter::getGear()` (enum)
- There is NO CAN-received gear. The STM32 does NOT echo gear back to ESP32.

### 3. SafeScreen vs DriveScreen — Source Comparison

| Screen | Source | File:Line | Mapping |
|--------|--------|-----------|---------|
| **DriveScreen** | `shifter::getGearRaw()` | `esp32/src/screens/drive_screen.cpp:215` | 0→P, 1→R, 2→N, 3→D1, 4→D2, default→N |
| **SafeScreen** | `shifter::getGearRaw()` | `esp32/src/screens/safe_screen.cpp:98` | 0→P, 1→R, 2→N, 3→D1, 4→D2, default→N |
| **main.cpp** (CAN TX) | `shifter::getGearRaw()` | `esp32/src/main.cpp:807` | Raw uint8_t sent to STM32 |
| **main.cpp** (mode cmd) | `shifter::getGearRaw()` | `esp32/src/main.cpp:351` | Raw uint8_t in CMD_MODE byte 1 |
| **main.cpp** (gear resync) | `shifter::getGearRaw()` | `esp32/src/main.cpp:758` | Raw uint8_t sent on STM32 restart |
| **main.cpp** (LED reverse) | `shifter::getGearRaw()` | `esp32/src/main.cpp:1191` | Compared to GEAR_REVERSE |

**Result: ✅ SAME source** — Both screens call `shifter::getGearRaw()` with identical switch-case mapping.

### 4. Data Flow Diagram

```
Physical Shifter (5 switches, active-low)
     │
     ▼
MCP23017 I2C Port A (GPIO 8/9, addr 0x20)
     │
     ▼  shifter::update()  [Core 1, main loop(), every ~50ms]
     │
     ▼
shifter::currentGear_  (static, module-scope)
     │
     ├──► shifter::getGearRaw()  →  SafeScreen::update()    [Core 0, safe_screen.cpp:98]
     │                                 switch(raw) → ui::Gear curGear_
     │                                   → SafeScreen::draw() renders gear bar
     │
     ├──► shifter::getGearRaw()  →  DriveScreen::update()   [Core 0, drive_screen.cpp:215]
     │                                 switch(raw) → ui::Gear curGear_
     │                                   → DriveScreen::draw() → GearDisplay::draw()
     │
     ├──► shifter::getGearRaw()  →  main.cpp:807  [Core 1, gear CAN TX to STM32]
     ├──► shifter::getGearRaw()  →  main.cpp:351  [Core 1, mode command CAN TX]
     ├──► shifter::getGearRaw()  →  main.cpp:758  [Core 1, gear resync on STM32 restart]
     └──► shifter::getGearRaw()  →  main.cpp:1191 [Core 1, reverse detection for LEDs]
```

### 5. Edge Case Validation

| Condition | Gear Shown | Source |
|-----------|-----------|--------|
| **No CAN** | Normal from I2C shifter (gear is independent of CAN) | `shifter_input.cpp:188-191` |
| **No I2C** (MCP23017 disconnected) | `N` (NEUTRAL) — safe default | `shifter_input.cpp:141,146,170` → `currentGear_ = Gear::NEUTRAL` |
| **I2C error (5+ consecutive)** | `N` (NEUTRAL) — backoff mode | `shifter_input.cpp:159-171` → error threshold + `Gear::NEUTRAL` |
| **Not initialized** | `N` (NEUTRAL) | `shifter_input.cpp:189` → guard returns `NEUTRAL` |
| **Startup** | `N` (NEUTRAL) | `shifter_input.cpp:40` → `static Gear currentGear_ = Gear::NEUTRAL` |

All fallbacks return NEUTRAL — no invented values, no undefined behavior.

### 6. Threading Note

`shifter::getGearRaw()` reads a single `static Gear currentGear_` (1-byte enum). On ESP32 Xtensa,
single-byte reads are atomic. Both screens (Core 0) read the same static variable that `shifter::update()`
(Core 1) writes. No mutex needed for 1-byte atomic read.

### 7. Confirmation

**SafeScreen and DriveScreen use identical gear source: `shifter::getGearRaw()` with identical mapping logic.**

No dual data source exists. No CAN-received gear path exists. No fix required.

---

## I2C Bus Diagnostic Block (rev 2026-06-01)

**Statement:** Purely additive, passive read-only UI. No safety logic, state
machine, CAN command, motor/relay control or timing behavior changed. It only
*surfaces* information the STM32 already had internally so the operator can
locate I2C wiring/hardware faults that force **Error Code 11**
(`SAFETY_ERROR_I2C_FAILURE`).

### What is shown

A dedicated block in the top-right column of the SAFE MODE screen:

```
I2C BUS DIAG
MUX 0x70: OK | FAIL
FL FR RL RR BT ST      ← per-channel INA226 health (green/red/gray)
fail:N rec:M           ← failed transactions this cycle / sticky recovery attempts
```

### Data flow

```
STM32 sensor_manager.c (Current_ReadAll / Sensor_Init)
   mux_present, ina_ok_mask, i2c_fail_count, i2c_recovery_attempts, i2c_ever_ok
        │  Sensor_GetMuxPresent() / Sensor_GetInaOkMask() / ...
        ▼
STM32 can_handler.c  CAN_SendI2CDiag()  → CAN 0x309 (DLC 5, 1 Hz)
        ▼
ESP32 can_rx.cpp  decodeI2cDiag()  → VehicleData::I2cDiagData
        ▼
ESP32 safe_screen.cpp  STILE_I2C tile (dirty-region, redraws only on change)
```

### How the STM32 distinguishes mux vs INA

In `Current_ReadAll()`, for each of the 6 mux channels:
- `TCA9548A_SelectChannel(i)` writes to **0x70**. If it ACKs → the mux is
  present (`mux_present = true`).
- The following INA226 shunt-register read (to **0x40**) sets `ina_last_read_ok`;
  if it ACKs, channel bit `i` of `ina_ok_mask` is set.

So a missing mux makes every channel-select fail (`MUX 0x70: FAIL`, all
channels gray = unknown), while a present mux with a dead sensor shows
`MUX 0x70: OK` and that channel red.

### Color legend

| Element | Color | Meaning |
|---------|-------|---------|
| `MUX 0x70` | green `OK` / red `FAIL` | TCA9548A acked / did not ack on the STM32 side |
| Channel (FL…ST) | green | INA226 acked behind that mux channel |
| Channel (FL…ST) | red | mux present but INA226 did not answer → that sensor/wiring |
| Channel (FL…ST) | gray | mux absent → per-channel state unknown |
| `fail:/rec:` | amber | failures or recovery attempts present | 
| `fail:/rec:` | gray | clean (0/0) |
| Whole block | gray `NO DATA` | no 0x309 frame received yet (does **not** invent OK) |

Channel labels map to: `FL FR RL RR BT(=BAT) ST(=STEER)` = mux channels 0..5.

### Troubleshooting guide (how to read it)

| Symptom on screen | Most likely cause | Where to check |
|-------------------|-------------------|----------------|
| `MUX 0x70: FAIL` (all channels gray) | Mux not reachable on PB8(SCL)/PB9(SDA) | 3V3 supply to TCA9548A, **4.7k pull-ups** on SDA/SCL (firmware uses `GPIO_NOPULL`), `!RESET` tied HIGH, A0/A1/A2 = GND (addr 0x70), SDA/SCL continuity |
| `MUX OK` + **all** channels red | INA226 power or common wiring | INA226 VS/VCC supply, that all INAs sit behind the mux (not on the main bus), shared SDA/SCL on the mux side |
| `MUX OK` + **one** channel red | That single INA226 / its channel wiring | INA226 on that mux channel: power, SDA/SCL of that channel, address must be 0x40 (a channel with two INAs at 0x40 conflicts) |
| `fail:`/`rec:` climbing intermittently | Weak pull-up, noise, loose terminal | Pull-up value, cable shielding, screw-terminal tightness on the MJK02 module |
| `NO DATA` | No 0x309 received | CAN link STM32↔ESP32, that the STM32 reached `Sensor_Init()` / 1 Hz task |

> **Important:** this block does **not** fix a physical fault — it only helps
> locate it. If the mux never reports OK, the cause is almost certainly
> pull-ups / supply / `!RESET` / address strapping, consistent with the
> pull-up findings in the previous PR.
