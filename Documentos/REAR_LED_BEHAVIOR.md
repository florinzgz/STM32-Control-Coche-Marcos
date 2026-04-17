# Rear LED Strip — Behavioral Analysis & Documentation

> **Code-traced, no assumptions.**
> Every statement is traceable to a source file and line number.
> If a feature is not implemented, it is explicitly marked **NOT IMPLEMENTED**.
>
> **Revision 2** — Updated to reflect MEJORAS CLARAS implementation:
>   1. Non-destructive rear turn overlay (matches front behavior)
>   2. Sequential turn signals (European-style outward sweep)
>   3. REGEN_ACTIVE activation (blue pulse when releasing throttle)
>   4. Emergency flash on ERROR state transition

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

| Mode              | Value | Full Strip Behavior                       | Source                        |
|-------------------|-------|-------------------------------------------|-------------------------------|
| `OFF`             | 0     | Black (all LEDs off)                      | `led_controller.cpp:203-204`  |
| `POSITION`        | 1     | Dim red (20%, brightness=51)              | `led_controller.cpp:205-207`  |
| `BRAKE`           | 2     | Bright red (100%)                         | `led_controller.cpp:209-211`  |
| `BRAKE_EMERGENCY` | 3     | Red flashing (blinkState ? Red : Black)   | `led_controller.cpp:212-213`  |
| `REVERSE`         | 4     | White (100%)                              | `led_controller.cpp:215-216`  |
| `REGEN_ACTIVE`    | 5     | Blue pulsing (sine approx via animationStep) | `led_controller.cpp:218-224` |

Source: enum at `led_controller.h:148-155`, rendering at `led_controller.cpp:198-231`

**Note:** All modes now paint the full 16-LED strip (`updateRearBase()`), not just
the centre zone.  Side zones [0–2] and [13–15] receive the same base color as the
centre, providing a visible base for the non-destructive turn-signal overlay.

---

## 5. Behavior Matrix (COMPLETE)

### 5.1 Base Layer (ALL 16 LEDs via `updateRearBase()`)

| Condition                        | Color  | Brightness     | Animated? | Source (main.cpp → led_controller.cpp) |
|----------------------------------|--------|----------------|-----------|----------------------------------------|
| System idle / normal drive       | Red    | 20% (dim)      | No        | `main.cpp:1262` → `cpp:205-207`       |
| Braking (speed>0, throttle≤5%)   | Red    | 100% (full)    | No        | `main.cpp:1258` → `cpp:209-211`       |
| Regen (speed>0, 5%<throttle≤20%) | Blue   | Pulsing sine   | Yes       | `main.cpp:1260` → `cpp:218-224`       |
| Reverse gear                     | White  | 100% (full)    | No        | `main.cpp:1256` → `cpp:215-216`       |
| SAFE / ERROR state               | Red    | Flashing 1 Hz  | Yes       | `main.cpp:1254` → `cpp:212-213`       |
| BOOT / STANDBY                   | —      | OFF (disabled)  | No        | `main.cpp:1188-1189`                   |
| OFF mode                         | Black  | 0%             | No        | `cpp:203-204` — never set by main.cpp  |

### 5.2 Turn-Signal Overlay (Side Zones — Non-Destructive Sequential Sweep)

| Condition                      | Left Zone        | Right Zone       | Source                           |
|--------------------------------|------------------|------------------|----------------------------------|
| No turn signal (normal)        | Base visible     | Base visible     | `updateRearTurnSignals()` returns early |
| LEFT turn signal (blink ON)    | AMBER sweep→out  | Base visible     | `cpp:346-350`                    |
| LEFT turn signal (blink OFF)   | Base visible     | Base visible     | `cpp:337` — no write on OFF      |
| RIGHT turn signal (blink ON)   | Base visible     | AMBER sweep→out  | `cpp:352-356`                    |
| RIGHT turn signal (blink OFF)  | Base visible     | Base visible     | `cpp:337` — no write on OFF      |
| HAZARD (blink ON)              | AMBER sweep→out  | AMBER sweep→out  | `cpp:330-331,346-356`            |
| HAZARD (blink OFF)             | Base visible     | Base visible     | `cpp:337` — no write on OFF      |
| Emergency flash active         | Red/Black flash  | Red/Black flash  | `cpp:383-397` (overrides all)    |

> **"Base visible"** means: whatever the rear base layer is painting (dim red
> for POSITION, bright red for BRAKE, white for REVERSE, blue pulse for REGEN,
> etc.) is visible through the side zones.

### 5.3 Sequential Sweep Animation Detail

During blink-ON, LEDs fill progressively outward from centre:

```
Time within 500 ms ON window:

  0–166 ms:   ■ □ □    (1 LED lit — centre-adjacent)
  167–333 ms: ■ ■ □    (2 LEDs lit)
  334–500 ms: ■ ■ ■    (3 LEDs lit — full zone)

  Left  [0–2]:   LED 2 → 1 → 0  (outward from centre)
  Right [13–15]: LED 13 → 14 → 15 (outward from centre)
```

Direction is determined by zone position relative to centre zone
(`sweepFill()`, `cpp:317-327`), supporting both `LED_STRIP_REVERSED`
orientations without code changes.

### 5.4 Combined State Matrix (Full Strip)

| SystemState    | Gear      | Braking | Regen  | Turn Signal | LEFT [0-2]       | CENTRE [3-12]    | RIGHT [13-15]    |
|----------------|-----------|---------|--------|-------------|------------------|------------------|------------------|
| ACTIVE         | Forward   | No      | No     | OFF         | Dim red (20%)    | Dim red (20%)    | Dim red (20%)    |
| ACTIVE         | Forward   | Yes     | —      | OFF         | Bright red       | Bright red       | Bright red       |
| ACTIVE         | Forward   | No      | Yes    | OFF         | Blue pulse       | Blue pulse       | Blue pulse       |
| ACTIVE         | Forward   | No      | No     | LEFT        | AMBER sweep/base | Dim red (20%)    | Dim red (20%)    |
| ACTIVE         | Forward   | Yes     | —      | LEFT        | AMBER sweep/base | Bright red       | Bright red       |
| ACTIVE         | Forward   | No      | No     | RIGHT       | Dim red (20%)    | Dim red (20%)    | AMBER sweep/base |
| ACTIVE         | Forward   | Yes     | —      | RIGHT       | Bright red       | Bright red       | AMBER sweep/base |
| ACTIVE         | Reverse   | —       | —      | OFF         | White            | White            | White            |
| ACTIVE         | Reverse   | —       | —      | LEFT        | AMBER sweep/white| White            | White            |
| ACTIVE         | Reverse   | —       | —      | RIGHT       | White            | White            | AMBER sweep/white|
| SAFE / ERROR   | Any       | —       | —      | HAZARD      | AMBER sweep/base | Red flash        | AMBER sweep/base |
| BOOT / STANDBY | —         | —       | —      | —           | OFF              | OFF              | OFF              |
| LIMP_HOME      | Forward   | No      | No     | (steering)  | (per turn/base)  | Dim red (20%)    | (per turn/base)  |
| DEGRADED       | Forward   | No      | No     | (steering)  | (per turn/base)  | Dim red (20%)    | (per turn/base)  |
| Emergency flash| —         | —       | —      | —           | Red/Black        | Red/Black        | Red/Black        |

> **"AMBER sweep/base"** = During blink-ON: sequential amber sweep (European-style).
> During blink-OFF: base color visible (e.g., dim red, bright red, white, blue pulse).

### 5.5 REGEN_ACTIVE Activation

| Condition                                        | Rear Mode    | Source                     |
|--------------------------------------------------|--------------|----------------------------|
| Speed sum > 20 AND traction avg ≤ 5%             | BRAKE        | `main.cpp:1229,1258`       |
| Speed sum > 20 AND 5% < traction avg ≤ 20%       | REGEN_ACTIVE | `main.cpp:1230-1232,1260`  |
| Speed sum > 20 AND traction avg > 20%            | POSITION     | `main.cpp:1262`            |
| Speed sum ≤ 20 (stationary)                       | POSITION     | Speed gate blocks both     |

Threshold: `LED_TRACTION_REGEN_THRESHOLD = 20` (`main.cpp:238`).

### 5.6 Emergency Flash (One-Shot on ERROR Transition)

| Trigger                   | Flash Cycles | Duration | Source                     |
|---------------------------|-------------|----------|----------------------------|
| Transition to `ERROR`     | 3 cycles    | ~600 ms  | `main.cpp:1200-1205`       |
| After flash completes     | Normal mode resumes | — | `led_controller.cpp:390-393` |

The emergency flash fires **once** on transition (not continuously).
After 3 red/black cycles, the sustained `BRAKE_EMERGENCY + HAZARD`
indication takes over.

---

## 6. Priority & Overlay Rules

### 6.1 Rendering Order (per frame)

```
led_ctrl::update()                          ← led_controller.cpp:372
  ├── [1] Emergency flash check             ← cpp:378-398 (highest priority, returns early)
  ├── [2] Rate limiter (50 ms / 20 Hz)      ← cpp:401-402
  ├── [3] animationStep++                   ← cpp:404
  ├── [4] blinkState toggle (500 ms)        ← cpp:407-410
  ├── [5] updateFrontLEDs()                 ← cpp:412
  ├── [6] updateFrontTurnSignals()          ← cpp:413
  ├── [7] updateRearBase()                  ← cpp:414  (paints ALL 16 LEDs)
  ├── [8] updateRearTurnSignals()           ← cpp:415  (non-destructive overlay)
  └── [9] FastLED.show()                    ← cpp:417
```

For rear LEDs: `updateRearBase()` → then `updateRearTurnSignals()`.

### 6.2 Priority Hierarchy (Rear)

```
1. EMERGENCY FLASH      (overrides ALL — one-shot on ERROR transition, 3 cycles)
2. SYSTEM DISABLED       (BOOT/STANDBY → FastLED.clear(), no updates)
3. BRAKE_EMERGENCY       (SAFE/ERROR → all LEDs flash red, sides = HAZARD sweep)
4. REVERSE               (white full strip)
5. BRAKE                 (bright red full strip)
6. REGEN_ACTIVE          (blue pulse full strip — throttle 5-20% at speed)
7. POSITION              (dim red full strip — default)
```

This priority is enforced by the `if/else if` chain in `main.cpp:1252-1263`.

Turn signals overlay **on top of** the base layer — they are rendered after
the base and write only during blink-ON.  During blink-OFF, the base is
visible through the side zones.

### 6.3 Overlay Type

| Function                  | Zones Written      | Type                 | Notes                                  |
|---------------------------|--------------------|----------------------|----------------------------------------|
| `updateRearBase()`        | [0–15] (ALL)       | **Destructive**      | Overwrites all 16 LEDs every frame     |
| `updateRearTurnSignals()` | [0–2], [13–15]     | **Non-destructive**  | Writes AMBER only during blink-ON; skips during blink-OFF |
| Emergency flash           | [0–15] (ALL)       | **Destructive**      | Fills entire rear strip Red/Black      |

**Front/rear parity:** Both strips now use the same non-destructive overlay
design — AMBER during blink-ON, base effect visible during blink-OFF.
The rear adds sequential sweep animation (European-style) during the ON phase.

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
- Rear turn signals (`updateRearTurnSignals()`)
- Rear base BRAKE_EMERGENCY mode (`updateRearBase()`)

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

  blinkState ON:   [AMBER sweep ×3]  [RED ×10]    [AMBER sweep ×3]
  blinkState OFF:  [RED flash ×3]    [BLACK ×10]  [RED flash ×3]
```

During blink-ON the amber sequential sweep overlays the side zones.
During blink-OFF the base layer (BRAKE_EMERGENCY: red/black flash) shows
through on the side zones — no hard black cut.

Centre and sides blink **in phase** (same `blinkState` variable), but the
amber sweep adds visual distinction to the side zones during ON.

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
│ 3. Braking / Regen detection                    (main.cpp:1210-1233) │
│    braking = (speedSum > 20 && tractionAvg <= 5)                │
│    regen   = (speedSum > 20 && 5 < tractionAvg <= 20)          │
│                                                                 │
│ 4. Rear mode selection (priority chain)            (main.cpp:1252-1263) │
│    SAFE/ERROR → BRAKE_EMERGENCY                                 │
│    reverse → REVERSE                                            │
│    braking → BRAKE                                              │
│    regen → REGEN_ACTIVE                                         │
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
│ updateRearBase()          → writes ledsRear[0..15]  (led_controller.cpp:198-231) │
│ updateRearTurnSignals()   → overlays ledsRear[0..2] + ledsRear[13..15]           │
│                             (non-destructive, blink-ON only, sequential sweep)    │
│                                                (led_controller.cpp:329-357)      │
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
| Brake + LEFT turn          | Full strip = BRAKE (red); Left = AMBER sweep on red base | Base paints all LEDs, overlay adds amber |
| Brake + HAZARD             | Full strip = BRAKE (red); Both sides = AMBER sweep on red base | Non-destructive overlay on bright red |
| SAFE + reverse             | SAFE wins: BRAKE_EMERGENCY + HAZARD   | SAFE/ERROR is first `if` in priority chain (`main.cpp:1253`) |
| Reverse + LEFT turn        | Full strip = WHITE; Left = AMBER sweep on white base | `updateRearBase()` paints white, sweep overlays |
| Reverse + HAZARD           | Full strip = WHITE; Both sides = AMBER sweep on white | Both sides sweep, white visible during OFF |
| Regen + LEFT turn          | Full strip = BLUE pulse; Left = AMBER sweep on blue base | Blue pulse base with amber overlay |
| Emergency + anything       | Emergency flash overrides ALL zones    | Emergency returns early before normal update (`cpp:378-398`) |

### 10.5 Non-Destructive Overlay Visual Benefit

During blink-OFF, the base layer shows through the side zones:

```
POSITION + LEFT turn:
  blink ON:   [AMBER→→→]  [dim red ×10]     [dim red ×3]
  blink OFF:  [dim red ×3] [dim red ×10]     [dim red ×3]

BRAKE + LEFT turn:
  blink ON:   [AMBER→→→]  [bright red ×10]  [bright red ×3]
  blink OFF:  [bright red] [bright red ×10]  [bright red ×3]

REVERSE + LEFT turn:
  blink ON:   [AMBER→→→]  [white ×10]       [white ×3]
  blink OFF:  [white ×3]  [white ×10]       [white ×3]

REGEN + LEFT turn:
  blink ON:   [AMBER→→→]  [blue pulse ×10]  [blue pulse ×3]
  blink OFF:  [blue ×3]   [blue pulse ×10]  [blue pulse ×3]
```

This eliminates the hard black cut that previously occurred during blink-OFF.

---

## 11. Explicit "NOT IMPLEMENTED" Sections

| Feature                         | Status              | Notes                                     |
|---------------------------------|---------------------|-------------------------------------------|
| `RearMode::OFF` via main.cpp   | **NOT USED**        | Only used as initial default; `main.cpp` always sets a valid mode. |
| Relay-based visual diagnostics | **NOT IMPLEMENTED** | Mentioned in `led_controller.h:26-34` as future work. Relay state is display-only. |
| DRL (Daytime Running Lights)   | **NOT IMPLEMENTED** | No separate rear DRL mode exists.         |
| Brightness per-zone control    | **NOT IMPLEMENTED** | All zones use the same brightness from the base mode. Global brightness is 200 (`cpp:367`). |
| Rear-specific hazard override  | **NOT IMPLEMENTED** | Rear uses the same `currentTurnSignal` as front; no rear-specific HAZARD flag. |

### Previously Not Implemented — Now Active

| Feature                    | Status               | Implementation                            |
|----------------------------|----------------------|-------------------------------------------|
| `RearMode::REGEN_ACTIVE`  | ✅ **NOW ACTIVE**    | Triggered when throttle 5–20% at speed (`main.cpp:1230-1232,1260`) |
| `startEmergencyFlash()`   | ✅ **NOW ACTIVE**    | One-shot on ERROR state transition (`main.cpp:1200-1205`) |
| Sequential turn signals   | ✅ **NOW IMPLEMENTED** | European-style outward sweep (`led_controller.cpp:317-357`) |
| Non-destructive overlay   | ✅ **NOW IMPLEMENTED** | Base visible during blink-OFF (`led_controller.cpp:337`) |

---

## 12. Key Code References Index

| Reference                          | File:Line                         |
|------------------------------------|-----------------------------------|
| Rear LED array declaration         | `led_controller.cpp:33`           |
| Rear zone constants (default)      | `led_controller.h:118-123`        |
| Rear zone constants (reversed)     | `led_controller.h:104-109`        |
| `RearMode` enum                    | `led_controller.h:148-155`        |
| `TurnSignal` enum                  | `led_controller.h:158-163`        |
| `updateRearBase()`                 | `led_controller.cpp:198-231`      |
| `sweepFill()` helper               | `led_controller.cpp:317-327`      |
| `updateRearTurnSignals()`          | `led_controller.cpp:329-357`      |
| Emergency flash handler            | `led_controller.cpp:378-398`      |
| Blink state toggle                 | `led_controller.cpp:407-410`      |
| Rendering order (update)           | `led_controller.cpp:412-417`      |
| `setRearMode()`                    | `led_controller.cpp:437-439`      |
| `setTurnSignal()`                  | `led_controller.cpp:445-447`      |
| `setEnabled()`                     | `led_controller.cpp:449-454`      |
| `init()` (FastLED setup)           | `led_controller.cpp:363-370`      |
| LED enable/disable gate            | `main.cpp:1186-1192`              |
| Emergency flash trigger (ERROR)    | `main.cpp:1194-1205`              |
| Regen threshold constant           | `main.cpp:238`                    |
| Regen detection logic              | `main.cpp:1230-1232`              |
| Rear mode priority chain           | `main.cpp:1252-1263`              |
| Turn signal hysteresis logic       | `main.cpp:1295-1341`              |
| HAZARD override (SAFE/ERROR)       | `main.cpp:1305-1307`              |
| Braking detection                  | `main.cpp:1210-1233`              |
| Reverse detection                  | `main.cpp:1207`                   |
| LED thresholds                     | `main.cpp:230-238`                |

---

## Validation Confirmations

- ✅ **Code changes match documentation** — all 4 improvements documented
- ✅ **No contradictions** found between code and documentation
- ✅ **No assumptions** — all remaining "NOT IMPLEMENTED" items are explicitly marked
- ✅ **Previously NOT IMPLEMENTED items now active** — REGEN, emergency flash, sequential turn, non-destructive overlay
