# Boot Sequence Analysis — ESP32-S3 HMI

**Revision:** 1.0  
**Status:** Code-accurate — derived exclusively from source files  
**Date:** 2026-02-20  
**Scope:** What the display actually shows when ESP32 boots before STM32  
**Source files:** `esp32/src/main.cpp`, `screen_manager.cpp`, `boot_screen.cpp`,
`standby_screen.cpp`, `drive_screen.cpp`, `can_rx.cpp`, `vehicle_data.h`,
`ui/frame_limiter.h`, `ui/battery_indicator.cpp`, `ui/car_renderer.cpp`,
`ui/gear_display.cpp`, `ui/mode_icons.cpp`, `ui/obstacle_sensor.cpp`,
`ui/pedal_bar.cpp`

> **Note:** This document describes what the code _actually executes_.
> It does not describe intended behavior or documentation claims.

---

## 1. Initialization Order

`vehicleData` and `screenManager` are file-scope static objects in `main.cpp`.
Their constructors run before `setup()`:

```
C++ static constructors (before setup)
 └─ ScreenManager()
     ├─ currentScreen_ = &bootScreen_
     ├─ currentState_ = can::SystemState::BOOT  (= 0)
     ├─ frameLimiter_ constructed: lastFrameMs_ = 0
     └─ bootScreen_.onEnter():
         ├─ needsRedraw_    = true
         ├─ canLinked_      = false
         └─ prevCanLinked_  = false
```

Then `setup()` runs:

```
setup()
 ├─ Serial.begin(115200)
 ├─ delay(500)                         ← 500 ms blocking wait
 ├─ psramInit() + delay(10)            ← 10 ms blocking wait
 ├─ tft.init()
 ├─ tft.setRotation(1)                 ← landscape 480×320
 ├─ tft.fillScreen(0x2104)             ← screen goes DARK GRAY, no text yet
 ├─ tft.setTextColor(0xFFFF, 0x2104)
 ├─ tft.setTextSize(1)
 └─ ESP32Can.begin(500 kbps)
```

The display becomes dark gray (0x2104) in `setup()`. No text is drawn yet.
Boot screen text is drawn on the first call to `loop()`.

---

## 2. First loop() Iteration — Boot Screen Draws

`frameLimiter_.shouldDraw()` checks `millis() - lastFrameMs_ >= 50`.
Since `lastFrameMs_ = 0` and `setup()` takes at least 510 ms, `millis()` is
already well above 50 when `loop()` first runs → `shouldDraw()` returns `true`
on the very first call.

`bootScreen_.draw()` executes with `needsRedraw_ = true`:

```
fillScreen(COL_BG)                        ← dark gray
drawString("COCHE",    240, 120)          ← white, size 3
drawString("MARCOS",   240, 160)          ← white, size 3
drawString("HMI v1.0", 240, 200)          ← gray,  size 1
prevCanLinked_ = !canLinked_ = true       ← forces status redraw this same frame
```

Then the CAN-link status partial redraw fires (because `canLinked_(false) !=
prevCanLinked_(true)`):

```
fillRect(0, 220, 480, 20, COL_BG)
drawString("CAN: WAITING...", 240, 230)   ← red, size 1
```

**What the user sees after first `loop()`: white "COCHE / MARCOS" on dark gray,
gray "HMI v1.0", red "CAN: WAITING..." at the bottom of the splash.**

---

## 3. Behavior While No CAN Frames Arrive (NO_CAN State)

Every `loop()` iteration:

| Call | Effect |
|------|--------|
| `can_rx::poll(vehicleData)` | `ESP32Can.readFrame(frame, 0)` returns false immediately — no frames decoded, `vehicleData` unchanged |
| `screenManager.update(vehicleData)` | `newState = data.heartbeat().systemState = can::SystemState::BOOT (0)` — equals `currentState_(BOOT)` — **no transition** |
| `bootScreen_.update(data)` | `canLinked_ = (timestampMs > 0 && ...)` = `(0 > 0 && ...)` = **false** — link stays undetected |
| `bootScreen_.draw()` | `canLinked_ == prevCanLinked_` → **no redraw**, display unchanged |
| Heartbeat TX | 0x011 is sent every 100 ms regardless of CAN link status |

The display stays exactly as shown in §2 indefinitely. There is **no timeout**,
**no watchdog**, and **no fallback** on the ESP32 side that would change the
screen if CAN never appears.

---

## 4. What Happens When CAN Appears Late (PARTIAL_CAN State)

### 4.1 First 0x001 HEARTBEAT_STM32 frame received

`decodeHeartbeat()` runs:
```cpp
hb.aliveCounter = f.data[0];
hb.systemState  = static_cast<can::SystemState>(f.data[1]);
hb.faultFlags   = f.data[2];
hb.errorCode    = f.data[3];
hb.timestampMs  = millis();   // ← nonzero for first time
data.setHeartbeat(hb);
```

On the next `loop()`:

**Boot-screen CAN link indicator:**
```cpp
canLinked_ = (timestampMs > 0) && (now - timestampMs < 500) = true
```
`canLinked_` changes false→true → partial redraw of status line:
```
fillRect(0, 220, 480, 20, COL_BG)
drawString("CAN: LINKED", 240, 230)       ← green, size 1
```

**Screen-manager state check:**
- If `f.data[1] == 0` (BOOT): `newState == currentState_` → **no transition**, stays on boot screen with "CAN: LINKED"
- If `f.data[1] == 1` (STANDBY): transition fires → StandbyScreen (§5)
- If `f.data[1] == 2` (ACTIVE): transition fires → DriveScreen (§6)
- If `f.data[1] == 3` (DEGRADED): transition fires → DriveScreen
- If `f.data[1] == 4` (SAFE): transition fires → SafeScreen
- If `f.data[1] == 5` (ERROR): transition fires → ErrorScreen
- If `f.data[1] == 6` (LIMP_HOME): transition fires → DriveScreen
- Any other value: transition fires → ErrorScreen (default branch)

### 4.2 Other CAN frames received before screen transition

Status frames received while still on the boot screen are decoded into
`vehicleData` but produce no visible change — the boot screen only reads
`data.heartbeat().timestampMs`. All other data accumulates silently.

---

## 5. StandbyScreen — Triggered by systemState = 1 (STANDBY)

`ScreenManager::update()` detects `newState(1) != currentState_(0)`:
```cpp
bootScreen_.onExit()            ← no-op
currentState_ = STANDBY
currentScreen_ = &standbyScreen_
standbyScreen_.onEnter()        ← needsRedraw_=true, prevFaultFlags_=0xFF, prevTemps_=0x7F
frameLimiter_.forceNextFrame()  ← lastFrameMs_=0 → immediate draw
```

`standbyScreen_.update(data)` reads:
- `faultFlags_  = data.heartbeat().faultFlags`   ← from 0x001 byte 2
- `temps_[0..4] = data.temp().temps[0..4]`       ← from 0x202

`standbyScreen_.draw()` with `needsRedraw_=true`:
```
fillScreen(COL_BG)
drawString("READY",           240, 60)    ← green, size 3
drawString("CAN: LINKED",     240, 100)   ← green, size 1 (hardcoded, not re-checked)
drawString("TEMPERATURES",    240, 160)   ← gray
drawString("FL:",  80, 185)               ← gray
drawString("FR:",  80, 207)
drawString("RL:",  80, 229)
drawString("RR:",  80, 251)
drawString("AMB:", 80, 273)
drawString("FAULT FLAGS",     240, 290)   ← gray
```

Then dynamic elements render immediately (forced by `prevTemps_=0x7F`, `prevFaultFlags_=0xFF`):

**Temperature values** (from 0x202 STATUS_TEMP):

| Condition | Display |
|-----------|---------|
| 0x202 not yet received (`temps_[i]=0`) | `"  0 C"` white at (140, 185+i*22) |
| 0x202 received | Actual `int8_t` value, formatted as `"%3d C"` |

**Fault flags** (from 0x001 byte 2):

| Condition | Display |
|-----------|---------|
| `faultFlags == 0` (default) | `"NO FAULTS"` green, centered at (240, 308) |
| `faultFlags != 0` | `"FLAGS: 0xXX"` amber, centered at (240, 308) |

**No user interaction is required. No controls are available on this screen.**

---

## 6. DriveScreen — Triggered by systemState = 2, 3, or 6

Triggered by ACTIVE(2), DEGRADED(3), or LIMP_HOME(6). Same `driveScreen_`
instance for all three.

`driveScreen_.onEnter()`:
```cpp
needsFullRedraw_ = true
prevGear_        = ui::Gear::P       ← not D1, so first draw highlights D1
prevMode_        = {}                ← {false, false}
```

`driveScreen_.update(data)` reads these sources:

| Field cached | Source | CAN ID |
|-------------|--------|--------|
| `curTraction_[4]` | `data.traction().scale[0..3]` | 0x205 STATUS_TRACTION |
| `curTemp_[4]` | `data.tempMap().temps[0..3]` | 0x206 STATUS_TEMP_MAP |
| `curSteeringRaw_` | `data.steering().angleRaw` | 0x204 STATUS_STEERING |
| `curSpeedAvgRaw_` | average of `data.speed().raw[0..3]` | 0x200 STATUS_SPEED |
| `curBattVoltRaw_` | `data.battery().voltageRaw` | 0x207 STATUS_BATTERY |
| `curPedalPct_` | average of `data.traction().scale[0..3]` | 0x205 STATUS_TRACTION |
| `curGear_` | **hardcoded `ui::Gear::N`** | not read from CAN |
| `curMode_.is4x4` | **hardcoded `false`** | not read from CAN |
| `curMode_.isTankTurn` | **hardcoded `false`** | not read from CAN |
| `curObstacleCm_` | `data.obstacle().distanceCm` | 0x208 OBSTACLE_DISTANCE |

> **Motor currents (0x201 STATUS_CURRENT) are decoded into `vehicleData.current_`
> by `can_rx::poll()` but `DriveScreen::update()` does not read `data.current()`.
> Current values are never displayed on any screen.**

### 6.1 Full Redraw — First Frame of Drive Screen

`draw()` with `needsFullRedraw_=true`:

1. `fillScreen(COL_BG)` — dark gray
2. `ModeIcons::drawStatic()` — all three icons drawn **inactive** (gray border, gray text)
3. `BatteryIndicator::drawStatic()` — white battery outline and nub
4. `ObstacleSensor::drawStatic()` — "SENSOR FRONTAL" label, gray bar outline
5. `CarRenderer::drawStatic()` — car body rectangle, axle lines, steering circle
6. `PedalBar::drawStatic()` — "PEDAL" label, bar outline
7. `GearDisplay::drawStatic()` — all 5 gears (P R N D1 D2) with gray borders
8. Static labels: `"km/h"` gray at (240, 258), `"360"` cyan above steering gauge

Then forced-mismatch values are set to ensure all dynamic elements redraw:
```cpp
prevSpeedAvgRaw_  = curSpeedAvgRaw_ + 1
prevBattVoltRaw_  = curBattVoltRaw_ + 1
prevPedalPct_     = curPedalPct_ + 1
prevGear_         = P  (curGear_=N, so P != N → fires)
prevSteeringRaw_  = curSteeringRaw_ + 10
prevObstacleCm_   = curObstacleCm_ + 1
prevTraction_[i]  = curTraction_[i] + 1  (for each wheel)
prevTemp_[i]      = curTemp_[i] + 1
prevMode_.is4x4     = true   (! of false)
prevMode_.isTankTurn = true
```

### 6.2 Default Values on First Drive Screen Frame (No CAN Data)

| Zone (Y px) | Element | Default value | Display |
|-------------|---------|---------------|---------|
| 0–40 | Mode icon 4x4 | false (inactive) | gray border, gray "4x4" |
| 0–40 | Mode icon 4x2 | **true (active)** | **cyan fill, "4x2"** ← `!is4x4 = !false = true` |
| 0–40 | Mode icon 360° | false (inactive) | gray border, gray "360" |
| 0–40 | Battery | `voltageRaw=0` → `voltageToPercent(0)=0` (≤1800) → 0% | **red fill, "0%"** |
| 40–85 | Obstacle distance | `distanceCm=0` | **"---"**, empty gray bar |
| 85–230 | Each wheel torque | `traction.scale[i]=0` → `torqueColor(0)=COL_GREEN` | **green fill, "0%"** |
| 85–230 | Each wheel temp | `tempMap.temps[i]=0` | **"0C"** |
| 85–230 | Steering needle | `angleRaw=0` → `angleDeg=0` → needle points **straight up** | cyan line from center to top |
| 230–270 | Speed | `speedAvgRaw=0` → `"0.0"` | **"0.0" km/h** white, large |
| 270–300 | Pedal bar | `pedalPct=0` → `fillW=0`, no fill | **empty bar, "  0%"** |
| 300–320 | Gear | `curGear_=N` (hardcoded default) | **N highlighted green** |

> **The "4x2" mode icon is active (cyan) on every drive screen render** because
> `curMode_.is4x4` is hardcoded to `false` in `update()`, making `4x2=!false=true`
> permanently. The 360° icon is permanently inactive. Neither icon will ever
> change state during the current session because `curMode_` is always `{false, false}`.

> **The gear selector always shows N highlighted** because `curGear_` is
> unconditionally set to `ui::Gear::N` in every `update()` call. The gear
> never changes regardless of what CAN data arrives — the STM32 does not
> report the active gear over CAN.

### 6.3 What Each CAN Frame Changes on Drive Screen

| CAN ID | Rate | Element updated |
|--------|------|-----------------|
| 0x200 STATUS_SPEED | 100 ms | Speed value in km/h (average of 4 wheels × 0.1) |
| 0x201 STATUS_CURRENT | 100 ms | **Nothing** — decoded but not read by any screen |
| 0x204 STATUS_STEERING | 100 ms | Steering needle angle (angleRaw × 0.1°, clamped ±45°) |
| 0x205 STATUS_TRACTION | 100 ms | Each wheel torque % + fill color; pedal bar % |
| 0x206 STATUS_TEMP_MAP | 1000 ms | Each wheel temperature text (first 4 of 5 sensors) |
| 0x207 STATUS_BATTERY | 100 ms | Battery icon fill + percentage (18.0 V=0%, 25.2 V=100%) |
| 0x208 OBSTACLE_DISTANCE | 66 ms | Distance text ("X.XX m" or "---") and proximity bar |
| 0x001 HEARTBEAT_STM32 | 100 ms | Screen transition only — no drive screen element updated |

---

## 7. State Timeline

```
╔══════════════╗
║     BOOT     ║
║              ║
║  setup():    ║
║  Dark gray   ║  (~510 ms)
║  screen only ║
╚══════╤═══════╝
       │ first loop() call
       ▼
╔══════════════╗
║    NO_CAN    ║  Duration: indefinite (no timeout on ESP32 side)
║              ║
║ White text:  ║
║  "COCHE"     ║
║  "MARCOS"    ║
║  "HMI v1.0"  ║
║  (gray)      ║
║              ║
║ Red text:    ║
║ "CAN:        ║
║  WAITING..." ║
╚══════╤═══════╝
       │ 0x001 received with any systemState value
       │ (timestampMs becomes nonzero)
       ▼
╔══════════════╗
║  PARTIAL_CAN ║  Duration: until systemState byte changes to STANDBY or ACTIVE
║              ║
║ Same layout  ║
║ as NO_CAN    ║
║ but:         ║
║              ║
║ Green text:  ║
║ "CAN:LINKED" ║
║              ║
║ (stays here  ║
║  if STM32    ║
║  sends       ║
║  state=BOOT) ║
╚══════╤═══════╝
       │ systemState = STANDBY(1) in 0x001
       ▼
╔══════════════╗
║  STANDBY /   ║  Duration: until STM32 sends ACTIVE(2)/DEGRADED(3)/LIMP_HOME(6)
║  FULL_DATA   ║
║              ║
║ Green:       ║
║  "READY"     ║
║  "CAN:LINKED"║
║              ║
║ Live from    ║
║  0x202:      ║
║  5 temps     ║
║              ║
║ From 0x001:  ║
║  "NO FAULTS" ║
║  or flags    ║
╚══════╤═══════╝
       │ systemState = ACTIVE(2) in 0x001
       ▼
╔══════════════╗
║ DRIVE_READY  ║  Persistent until STM32 changes state
║              ║
║ Top bar:     ║
║  4x2 ← cyan  ║  ← hardcoded active
║  0% batt     ║  ← until 0x207 arrives
║              ║
║ Obstacle:    ║
║  "---"       ║  ← until 0x208 arrives
║              ║
║ 4 wheels:    ║
║  green/0%/0C ║  ← until 0x205+0x206 arrive
║              ║
║ Steering:    ║
║  needle up   ║  ← until 0x204 arrives
║              ║
║ Speed: 0.0   ║  ← until 0x200 arrives
║              ║
║ Pedal: 0%    ║  ← until 0x205 arrives
║              ║
║ Gear: N ●   ║  ← always, never changes from CAN
╚══════════════╝
```

---

## 8. No User Interaction Required

No screen transition, data update, or rendering step requires any user action.
`tft.getTouch()` is called once per loop (when `RUNTIME_MONITOR=1`) exclusively
to detect a 3-second hold for the debug overlay. No touch event changes the
screen state machine.

---

## 9. Blocking Conditions

| Condition | Blocking? | Detail |
|-----------|-----------|--------|
| `delay(500)` in setup() | Yes | One-time, 500 ms at boot |
| `delay(10)` in setup() | Yes | One-time, 10 ms PSRAM init |
| `frameLimiter_.shouldDraw()` returns false | Non-blocking | Loop continues; draw skipped; returns true after 50 ms elapsed |
| CAN bus absence | Non-blocking | `ESP32Can.readFrame(frame, 0)` returns false immediately (0 ms timeout) |
| Screen state machine | Non-blocking | No screen transition occurs without receiving 0x001 with a changed systemState byte; no internal timeout fires |
| ACK timeout (200 ms) | Non-blocking | Only fires if `ackPending=true`; never set in the current code (no CMD_MODE or SERVICE_CMD is sent) |

There is no `while(1)` or blocking `wait()` call in `loop()`.

---

## 10. Known Gaps Between Decoded and Displayed Data

| CAN ID | Data decoded | Displayed | Note |
|--------|-------------|-----------|------|
| 0x201 STATUS_CURRENT | ✅ | ❌ | Motor currents (4 × uint16, 0.01 A) stored in `vehicleData.current_` but no screen's `update()` reads them |
| 0x202 STATUS_TEMP | ✅ | Standby only | Not shown on drive screen; drive screen uses 0x206 instead |
| 0x203 STATUS_SAFETY | ✅ | Safe/Error screens only | ABS/TCS flags not shown on drive screen |
| Gear (CMD_MODE byte 1) | N/A | Drive screen | STM32 does not echo gear back; `curGear_` is hardcoded to N (neutral) |
| Mode flags | N/A | Drive screen | STM32 does not echo mode back; `curMode_` is hardcoded to `{false, false}` |
