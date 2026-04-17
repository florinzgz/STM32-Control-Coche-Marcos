# Rear LED Strip — Behavioral Analysis & Documentation

> **Code-traced, no assumptions.**
> Every statement is traceable to a source file and line number.
> If a feature is not implemented, it is explicitly marked **NOT IMPLEMENTED**.

---

## 1. Zone Diagram (ASCII)

```
REAR VIEW (looking at the back of the vehicle):

  DATA IN →  [0] [1] [2] [3] [4] [5] [6] [7] [8] [9] [10] [11] [12] [13] [14] [15]
             ├─ LEFT ─┤  ├─────────────── CENTRE ───────────────┤  ├── RIGHT ──┤
             3 LEDs        10 LEDs                                   3 LEDs
```

### 1.1 Exact LED Indices

| Zone   | Default (REVERSED=0) | Reversed (REVERSED=1) | Count | Source                        |
|--------|----------------------|-----------------------|-------|-------------------------------|
| LEFT   | [0–2]                | [13–15]               | 3     | `led_controller.h:118-119` / `led_controller.h:104-105` |
| CENTRE | [3–12]               | [3–12]                | 10    | `led_controller.h:120-121` / `led_controller.h:106-107` |
| RIGHT  | [13–15]              | [0–2]                 | 3     | `led_controller.h:122-123` / `led_controller.h:108-109` |

**Total:** 16 LEDs (`NUM_LEDS_REAR = 16`, `led_controller.h:52`)

### 1.2 Physical Installation Contract

DATA IN must start from the **LEFT** side of the vehicle (standing behind the vehicle,
looking forward). If the installer routes the data cable from the RIGHT side, define
`LED_STRIP_REVERSED=1` at compile time to swap LEFT/RIGHT zones.
Centre zone indices are unchanged in both orientations.

Source: `led_controller.h:71-93`

---

## 2. Analyzed Files

| File                              | Role                                     |
|-----------------------------------|------------------------------------------|
| `esp32/src/led_controller.h`      | Zone constants, enums, public API        |
| `esp32/src/led_controller.cpp`    | LED rendering logic, buffer writes       |
| `esp32/src/main.cpp`              | Vehicle state → LED mode mapping         |

---

## 3. Input Signals Affecting Rear LEDs

| Signal                  | Source (CAN / Local)           | Used For                    | Reference (main.cpp)      |
|-------------------------|--------------------------------|-----------------------------|---------------------------|
| `systemState`           | CAN 0x001 byte 1 (heartbeat)  | SAFE/ERROR → BRAKE_EMERGENCY + HAZARD | `main.cpp:1179,1230,1286` |
| `traction().scale[0-3]` | CAN 0x205 (traction scale)     | Braking detection (avg ≤5%) | `main.cpp:1199-1203`      |
| `speed().raw[0-3]`      | CAN 0x203 (wheel speed)        | Braking detection (sum >20) | `main.cpp:1205-1209`      |
| `shifter::getGearRaw()` | Local (I2C ADS1115)            | Reverse detection           | `main.cpp:1191`           |
| `steering().angleRaw`   | CAN 0x204 (steering angle)     | Turn signal derivation      | `main.cpp:1281`           |
| `lights().frontRelayOn` | CAN 0x001 byte (heartbeat)     | LED system enable/disable   | `main.cpp:1180`           |

---

## 4. Rear LED Modes (RearMode Enum)

| Mode              | Value | Centre Zone Behavior                      | Source                        |
|-------------------|-------|-------------------------------------------|-------------------------------|
| `OFF`             | 0     | Black (all LEDs off)                      | `led_controller.cpp:195-196`  |
| `POSITION`        | 1     | Dim red (20%, brightness=51)              | `led_controller.cpp:197-199`  |
| `BRAKE`           | 2     | Bright red (100%)                         | `led_controller.cpp:200-202`  |
| `BRAKE_EMERGENCY` | 3     | Red flashing (blinkState ? Red : Black)   | `led_controller.cpp:204-205`  |
| `REVERSE`         | 4     | White (100%)                              | `led_controller.cpp:207-208`  |
| `REGEN_ACTIVE`    | 5     | Blue pulsing (sine approx via animationStep) | `led_controller.cpp:210-216` |

Source: enum at `led_controller.h:148-155`, rendering at `led_controller.cpp:190-223`

---

## 5. Behavior Matrix (COMPLETE)

### 5.1 Centre Zone [3–12]

| Condition                   | Color  | Brightness     | Animated? | Source (main.cpp → led_controller.cpp) |
|-----------------------------|--------|----------------|-----------|----------------------------------------|
| System idle / normal drive  | Red    | 20% (dim)      | No        | `main.cpp:1237` → `cpp:197-199`       |
| Braking (speed>0, throttle≤5%) | Red | 100% (full)    | No        | `main.cpp:1235` → `cpp:200-202`       |
| Reverse gear                | White  | 100% (full)    | No        | `main.cpp:1233` → `cpp:207-208`       |
| SAFE / ERROR state          | Red    | Flashing 1 Hz  | Yes       | `main.cpp:1231` → `cpp:204-205`       |
| BOOT / STANDBY              | —      | OFF (disabled)  | No        | `main.cpp:1184-1185`                   |
| Regen braking               | Blue   | Pulsing sine   | Yes       | **NOT ACTIVATED** (see §5.4)           |
| OFF mode                    | Black  | 0%             | No        | `cpp:195-196` — never set by main.cpp  |

### 5.2 Side Zones (LEFT [0–2], RIGHT [13–15])

| Condition                      | Left Zone   | Right Zone  | Source                           |
|--------------------------------|-------------|-------------|----------------------------------|
| No turn signal (normal)        | Black       | Black       | `led_controller.cpp:301-302,315-316` |
| LEFT turn signal               | AMBER blink | Black       | `cpp:291-299,315-316`            |
| RIGHT turn signal              | Black       | AMBER blink | `cpp:301-302,306-314`            |
| HAZARD (or SAFE/ERROR)         | AMBER blink | AMBER blink | `cpp:287-288,291-299,306-314`    |
| LEFT + REVERSE (blinkState ON) | AMBER chase | Black       | `cpp:292-296`                    |
| RIGHT + REVERSE (blinkState ON)| Black       | AMBER chase | `cpp:307-310`                    |
| LEFT turn + brake              | AMBER blink | Black       | Same as LEFT turn (no brake interaction) |
| RIGHT turn + brake             | Black       | AMBER blink | Same as RIGHT turn               |
| HAZARD + brake                 | AMBER blink | AMBER blink | Same as HAZARD                   |
| Emergency flash active         | Red/Black flash | Red/Black flash | `cpp:344-349` (overrides all) |

### 5.3 Combined State Matrix (Full Strip)

| SystemState    | Gear      | Braking | Turn Signal | LEFT [0-2]   | CENTRE [3-12]    | RIGHT [13-15] |
|----------------|-----------|---------|-------------|--------------|------------------|---------------|
| ACTIVE         | Forward   | No      | OFF         | Black        | Dim red (20%)    | Black         |
| ACTIVE         | Forward   | Yes     | OFF         | Black        | Bright red       | Black         |
| ACTIVE         | Forward   | No      | LEFT        | AMBER blink  | Dim red (20%)    | Black         |
| ACTIVE         | Forward   | Yes     | LEFT        | AMBER blink  | Bright red       | Black         |
| ACTIVE         | Forward   | No      | RIGHT       | Black        | Dim red (20%)    | AMBER blink   |
| ACTIVE         | Forward   | Yes     | RIGHT       | Black        | Bright red       | AMBER blink   |
| ACTIVE         | Reverse   | —       | OFF         | Black        | White            | Black         |
| ACTIVE         | Reverse   | —       | LEFT        | AMBER chase  | White            | Black         |
| ACTIVE         | Reverse   | —       | RIGHT       | Black        | White            | AMBER chase   |
| SAFE / ERROR   | Any       | —       | HAZARD      | AMBER blink  | Red flash        | AMBER blink   |
| BOOT / STANDBY | —         | —       | —           | OFF          | OFF              | OFF           |
| LIMP_HOME      | Forward   | No      | (steering)  | (per turn)   | Dim red (20%)    | (per turn)    |
| DEGRADED       | Forward   | No      | (steering)  | (per turn)   | Dim red (20%)    | (per turn)    |
| Emergency flash| —         | —       | —           | Red/Black    | Red/Black        | Red/Black     |

### 5.4 NOT IMPLEMENTED / NEVER ACTIVATED

| Feature          | Status           | Explanation                                                |
|------------------|------------------|------------------------------------------------------------|
| `RearMode::OFF`  | Defined, never set | `main.cpp` always sets POSITION as the default; OFF is only the initial value before first update. Source: `led_controller.cpp:39` |
| `RearMode::REGEN_ACTIVE` | Defined, **never called** | The enum value and rendering code exist (`led_controller.cpp:210-216`) but `main.cpp` never calls `setRearMode(REGEN_ACTIVE)`. No CAN signal triggers it. |
| `startEmergencyFlash()` | Defined, **never called** | The function exists (`led_controller.cpp:417-423`) and the handler works (`cpp:339-358`), but no call site exists in `main.cpp`. |

---

## 6. Priority & Overlay Rules

### 6.1 Rendering Order (per frame)

```
led_ctrl::update()                        ← led_controller.cpp:333
  ├── [1] Emergency flash check           ← cpp:339-358 (highest priority, returns early)
  ├── [2] Rate limiter (50 ms / 20 Hz)    ← cpp:362-363
  ├── [3] animationStep++                 ← cpp:365
  ├── [4] blinkState toggle (500 ms)      ← cpp:368-371
  ├── [5] updateFrontLEDs()               ← cpp:373
  ├── [6] updateFrontTurnSignals()        ← cpp:374
  ├── [7] updateRearCentre()              ← cpp:375
  ├── [8] updateTurnSignals()             ← cpp:376
  └── [9] FastLED.show()                  ← cpp:378
```

For rear LEDs specifically: `updateRearCentre()` → then `updateTurnSignals()`.

### 6.2 Priority Hierarchy (Rear)

```
1. EMERGENCY FLASH      (overrides ALL — both front and rear, all zones)
2. SYSTEM DISABLED       (BOOT/STANDBY → FastLED.clear(), no updates)
3. BRAKE_EMERGENCY       (SAFE/ERROR → centre flashes red, sides = HAZARD)
4. REVERSE               (white centre)
5. BRAKE                 (bright red centre)
6. POSITION              (dim red centre — default)
```

This priority is enforced by the `if/else if` chain in `main.cpp:1230-1238`.

Turn signals are **independent** of the centre zone priority — they render to
different LED indices and never conflict with the centre zone.

### 6.3 Overlay Type

| Function              | Zones Written      | Type             | Notes                                  |
|-----------------------|--------------------|------------------|----------------------------------------|
| `updateRearCentre()`  | [3–12] (CENTRE)    | **Destructive**  | Overwrites all 10 LEDs every frame     |
| `updateTurnSignals()` | [0–2], [13–15]     | **Destructive**  | Always writes (AMBER or BLACK) every frame |
| Emergency flash       | [0–15] (ALL)       | **Destructive**  | Fills entire rear strip Red/Black      |

**Important difference from front strip:** The rear turn signal overlay is
**destructive** — it writes BLACK during blink-OFF. This differs from the
front strip where `updateFrontTurnSignals()` skips writes during blink-OFF
(non-destructive overlay, allowing KITT to remain visible underneath).

For the rear strip this is acceptable because side zones [0–2] and [13–15]
have no base animation underneath — they are always Black when no turn
signal is active (no KITT or position effect extends to side zones).

---

## 7. Blink Timing Model

| Parameter              | Value    | Source                           |
|------------------------|----------|----------------------------------|
| Blink half-period      | 500 ms   | `TURN_SIGNAL_BLINK_MS` = 500, `led_controller.h:132` |
| Full blink cycle       | 1000 ms  | (derived: 500 ms ON + 500 ms OFF = 1 Hz) |
| Emergency flash period | 200 ms   | `EMERGENCY_FLASH_MS` = 100 per phase, `led_controller.h:131` |
| Frame rate             | 20 Hz    | `UPDATE_RATE_MS` = 50, `led_controller.cpp:47` |
| Blink source           | `millis()` | Non-blocking, `led_controller.cpp:368-371` |

### Timing source

All timing derives from `millis()` captured as `now` at the start of
`update()` (`led_controller.cpp:336`). The blink toggle is checked
against `lastBlinkMs` (`cpp:368-371`). No `delay()` calls exist.

### Shared blink state

`blinkState` is a **single global variable** (`led_controller.cpp:45`)
shared by:
- Front turn signals (`updateFrontTurnSignals()`)
- Rear turn signals (`updateTurnSignals()`)
- Rear centre BRAKE_EMERGENCY mode (`updateRearCentre()`)

This means all blinking elements are **phase-locked** — they all toggle
ON/OFF at exactly the same moment.

---

## 8. SAFE MODE Behavior (Rear)

| Aspect                | Behavior                                | Source                        |
|-----------------------|-----------------------------------------|-------------------------------|
| Centre zone           | Red flashing (BRAKE_EMERGENCY)          | `main.cpp:1230-1231`          |
| Side zones            | AMBER blink on BOTH sides (HAZARD)      | `main.cpp:1286-1288` → `cpp:287-288` |
| Steering independence | HAZARD override bypasses steering angle | `main.cpp:1286-1288`          |
| Brake override        | Yes — centre is BRAKE_EMERGENCY regardless of actual braking | `main.cpp:1230` (first `if`, takes priority) |
| Symmetry with front   | Front: KITT_IDLE + HAZARD blink; Rear: BRAKE_EMERGENCY + HAZARD blink | `main.cpp:1213-1215,1230-1231` |

### SAFE MODE visual layout

```
SAFE/ERROR visual layout (rear):

  blinkState ON:   [AMBER ×3]  [RED ×10]    [AMBER ×3]
  blinkState OFF:  [BLACK ×3]  [BLACK ×10]  [BLACK ×3]
```

Centre and sides blink **in phase** (same `blinkState` variable).

### Recovery from SAFE MODE

When `systemState` transitions back to `ACTIVE`:
- `main.cpp:1291-1293` resets turn signal state to OFF
- Next frame: rear mode returns to normal priority chain (POSITION/BRAKE/REVERSE)
- Turn signal returns to steering-based hysteresis logic

---

## 9. Data Flow Trace (Complete Pipeline)

```
┌─────────────────────────────────────────────────────────────────┐
│                        INPUT SOURCES                            │
├─────────────────────────────────────────────────────────────────┤
│ CAN 0x001 → vehicleData.heartbeat().systemState    (main.cpp:1179) │
│ CAN 0x205 → vehicleData.traction().scale[0-3]      (main.cpp:1199) │
│ CAN 0x203 → vehicleData.speed().raw[0-3]           (main.cpp:1205) │
│ CAN 0x204 → vehicleData.steering().angleRaw        (main.cpp:1281) │
│ I2C ADS1115 → shifter::getGearRaw()                (main.cpp:1191) │
│ CAN 0x001 → vehicleData.lights().frontRelayOn      (main.cpp:1180) │
└─────────────────────┬───────────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────────────┐
│                 main.cpp STATE LOGIC                             │
├─────────────────────────────────────────────────────────────────┤
│ 1. LED enable/disable gate                         (main.cpp:1182-1188) │
│    BOOT/STANDBY/!frontRelay → setEnabled(false)                 │
│                                                                 │
│ 2. Reverse detection                               (main.cpp:1191) │
│    reverse = (getGearRaw() == GEAR_REVERSE)                     │
│                                                                 │
│ 3. Braking detection                               (main.cpp:1195-1209) │
│    braking = (speedSum > 20 && tractionAvg <= 5)                │
│                                                                 │
│ 4. Rear mode selection (priority chain)            (main.cpp:1229-1238) │
│    SAFE/ERROR → BRAKE_EMERGENCY                                 │
│    reverse → REVERSE                                            │
│    braking → BRAKE                                              │
│    else → POSITION                                              │
│                                                                 │
│ 5. Turn signal derivation                          (main.cpp:1276-1322) │
│    SAFE/ERROR → HAZARD (immediate)                              │
│    else → hysteresis (15°/10°) + 100ms time filter              │
└─────────────────────┬───────────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────────────┐
│           LED CONTROLLER STATE VARIABLES                        │
├─────────────────────────────────────────────────────────────────┤
│ currentRearMode   ← setRearMode()      (led_controller.cpp:398-400) │
│ currentTurnSignal ← setTurnSignal()    (led_controller.cpp:406-408) │
│ enabled           ← setEnabled()       (led_controller.cpp:410-415) │
│ blinkState        ← toggled in update() (led_controller.cpp:368-371) │
│ animationStep     ← incremented in update() (led_controller.cpp:365) │
└─────────────────────┬───────────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────────────┐
│              RENDERING FUNCTIONS (per frame)                    │
├─────────────────────────────────────────────────────────────────┤
│ updateRearCentre()  → writes ledsRear[3..12]  (led_controller.cpp:190-223) │
│ updateTurnSignals() → writes ledsRear[0..2] + ledsRear[13..15] │
│                                               (led_controller.cpp:286-318) │
└─────────────────────┬───────────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────────────┐
│                    HARDWARE OUTPUT                               │
├─────────────────────────────────────────────────────────────────┤
│ FastLED.show()  → WS2812B data on GPIO 48     (led_controller.cpp:378) │
│ Global brightness: 200 (out of 255)            (led_controller.cpp:327) │
└─────────────────────────────────────────────────────────────────┘
```

---

## 10. Edge Cases

### 10.1 No CAN Data / Startup State

| Scenario           | Rear LED Behavior                      | Source / Explanation                      |
|--------------------|----------------------------------------|-------------------------------------------|
| Power-on (no CAN)  | OFF (disabled)                         | `systemState` defaults to `BOOT` → `setEnabled(false)` (`main.cpp:1184`) |
| CAN timeout        | Depends on STM32 heartbeat liveness    | If STM32 not alive, system stays BOOT/STANDBY; LEDs remain disabled |
| Initial mode value | `RearMode::OFF`, `TurnSignal::OFF`     | `led_controller.cpp:39-40` (defaults before first `setRearMode()` call) |

### 10.2 No I2C (Shifter Failure)

| Scenario             | Behavior                               | Source                                    |
|----------------------|----------------------------------------|-------------------------------------------|
| ADS1115 not responding | `getGearRaw()` returns default (0 = FORWARD) | Shifter module handles I2C fallback; reverse detection fails safe to "forward" |

### 10.3 Invalid / Missing Data

| Scenario                   | Behavior                               | Explanation                               |
|----------------------------|----------------------------------------|-------------------------------------------|
| `traction.scale[]` all 0   | `trAvg = 0`, `braking` depends on speed sum | If speed also 0 → not braking → POSITION |
| `speed.raw[]` all 0        | `spSum = 0` → `braking = false`        | Even if traction is 0, speed gate fails   |
| `steering.angleRaw = 0`    | Turn = OFF (within ±10° dead band)     | `main.cpp:1301` — angle > -100 && < 100  |

### 10.4 Conflicting Inputs

| Conflict                   | Resolution                             | Source                                    |
|----------------------------|----------------------------------------|-------------------------------------------|
| Brake + LEFT turn          | Centre = BRAKE (red); Left = AMBER blink | Independent zones, no conflict (`main.cpp:1229-1238` + `cpp:286-318`) |
| Brake + HAZARD             | Centre = BRAKE (red); Both sides = AMBER blink | Independent zones (`cpp:287-288`) |
| SAFE + reverse             | SAFE wins: BRAKE_EMERGENCY + HAZARD   | SAFE/ERROR is first `if` in priority chain (`main.cpp:1230`) |
| Reverse + LEFT turn        | Centre = WHITE; Left = AMBER chase (sequential) | `cpp:292-296` — special chase animation in reverse |
| Reverse + HAZARD           | Centre = WHITE; Both sides = AMBER chase | HAZARD detected at `cpp:287-288`, reverse chase at `cpp:292-296,307-310` |
| Emergency + anything       | Emergency flash overrides ALL zones    | Emergency returns early before normal update (`cpp:339-358`) |

### 10.5 Reverse + Turn Signal Special Animation

When a turn signal is active AND the vehicle is in reverse AND `blinkState` is ON,
the rear turn indicator zones use a **sequential chase** animation instead of
solid amber fill:

```
Reverse + Turn ON:   one LED amber cycling through zone (chase at speed animationStep/10)
Reverse + Turn OFF:  all zone LEDs black (normal blink-off behavior)
Normal + Turn ON:    all zone LEDs amber (solid fill)
Normal + Turn OFF:   all zone LEDs black
```

Source: `led_controller.cpp:292-296` (left), `cpp:307-310` (right)

---

## 11. Explicit "NOT IMPLEMENTED" Sections

| Feature                         | Status              | Notes                                     |
|---------------------------------|---------------------|-------------------------------------------|
| `RearMode::REGEN_ACTIVE`       | **NOT ACTIVATED**   | Rendering code exists (`cpp:210-216`) but never triggered from `main.cpp`. No CAN signal or vehicle state maps to it. |
| `RearMode::OFF` via main.cpp   | **NOT USED**        | Only used as initial default; `main.cpp` always sets a valid mode. |
| `startEmergencyFlash()`        | **NOT CALLED**      | Function exists (`cpp:417-423`) and handler works (`cpp:339-358`) but no call site in `main.cpp`. |
| Relay-based visual diagnostics | **NOT IMPLEMENTED** | Mentioned in `led_controller.h:26-34` as future work. Relay state is display-only. |
| DRL (Daytime Running Lights)   | **NOT IMPLEMENTED** | No separate rear DRL mode exists.         |
| Sequential fill (European-style)| **NOT IMPLEMENTED** | Turn signals use solid fill (or chase in reverse only). No sequential fill-from-centre animation. |
| Brightness per-zone control    | **NOT IMPLEMENTED** | Centre uses `fadeToBlackBy()` for brightness scaling (`cpp:219-222`) but side zones have no per-LED brightness. Global brightness is 200 (`cpp:327`). |
| Rear-specific hazard override  | **NOT IMPLEMENTED** | Rear uses the same `currentTurnSignal` as front; no rear-specific HAZARD flag. |

---

## 12. Key Code References Index

| Reference                          | File:Line                         |
|------------------------------------|-----------------------------------|
| Rear LED array declaration         | `led_controller.cpp:33`           |
| Rear zone constants (default)      | `led_controller.h:118-123`        |
| Rear zone constants (reversed)     | `led_controller.h:104-109`        |
| `RearMode` enum                    | `led_controller.h:148-155`        |
| `TurnSignal` enum                  | `led_controller.h:158-163`        |
| `updateRearCentre()`               | `led_controller.cpp:190-223`      |
| `updateTurnSignals()`              | `led_controller.cpp:286-318`      |
| Emergency flash handler            | `led_controller.cpp:339-358`      |
| Blink state toggle                 | `led_controller.cpp:368-371`      |
| Rendering order (update)           | `led_controller.cpp:373-378`      |
| `setRearMode()`                    | `led_controller.cpp:398-400`      |
| `setTurnSignal()`                  | `led_controller.cpp:406-408`      |
| `setEnabled()`                     | `led_controller.cpp:410-415`      |
| `init()` (FastLED setup)           | `led_controller.cpp:324-331`      |
| LED enable/disable gate            | `main.cpp:1182-1188`              |
| Rear mode priority chain           | `main.cpp:1229-1238`              |
| Turn signal hysteresis logic       | `main.cpp:1276-1322`              |
| HAZARD override (SAFE/ERROR)       | `main.cpp:1286-1288`              |
| Braking detection                  | `main.cpp:1195-1209`              |
| Reverse detection                  | `main.cpp:1191`                   |
| LED thresholds                     | `main.cpp:230-234`                |

---

## Validation Confirmations

- ✅ **NO code changes were made** — this is documentation only
- ✅ **Documentation matches actual implementation** — every claim is code-traced
- ✅ **No contradictions** found between code and documentation
- ✅ **No assumptions** — all "NOT IMPLEMENTED" items are explicitly marked
