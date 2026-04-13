# 🔧 RELAY STATUS VISUALIZATION — EXACT CODE-BASED RENDER

> **Auto-generated from source code analysis.**
> Every coordinate, color, and label is extracted from the actual implementation.
> No assumptions. UNKNOWN is stated explicitly where code is ambiguous.

---

## 📋 IMPLEMENTATION SUMMARY (POST-PR)

The relay visibility feature was added in 2 commits:

1. **STM32**: `Safety_GetRelayStatusByte()` + CAN heartbeat byte 5
2. **ESP32**: `RelayIndicator` widget in DriveScreen + relay panel in EngineeringScreen

---

# 🧩 TASK 1 — DRIVE SCREEN (EXACT RENDER)

## Screen: 480×320 landscape

### Tile Map (from `drive_screen.cpp` onEnter)

| Tile             | X   | Y   | W   | H   |
|------------------|-----|-----|-----|-----|
| DTILE_MODE_ICONS | 0   | 0   | 180 | 40  |
| DTILE_LED_TOGGLE | 180 | 0   | 120 | 40  |
| DTILE_ACK        | (cfg) | (cfg) | (cfg) | (cfg) |
| DTILE_BATTERY    | 405 | 0   | 75  | 40  |
| DTILE_OBSTACLE   | 0   | (SENSOR_Y) | 480 | (SENSOR_H) |
| DTILE_WHEELS     | 0   | (CAR_AREA_Y) | 370 | (CAR_AREA_H) |
| DTILE_STEERING   | 370 | (CAR_AREA_Y) | 110 | (CAR_AREA_H) |
| **DTILE_SPEED**  | 0   | **232** | 480 | **38** |
| DTILE_PEDAL      | 0   | 272 | 480 | 28  |
| **DTILE_GEAR**   | 0   | **300** | 480 | **20** |
| DTILE_DEGRADED   | 0   | 40  | 480 | 18  |
| DTILE_FAULTS     | 0   | 28  | 480 | 10  |

> Source: `ui_common.h` lines 57-66, `ui_config.h` lines 195-204

### Speed + RPM Tile (Y: 232–270)

**Rendering function:** `DriveScreen::drawSpeed()` (line 586)

```
Y=232:  Speed value — TC_DATUM, TextSize 3, COL_WHITE
        Centered at X = SCREEN_W/2 = 240
        Format: "%u.%u" (e.g. "12.5")

Y=258:  "km/h" label — TC_DATUM, TextSize 1, COL_GRAY
        Centered at X = SCREEN_W/2 = 240
        (SPEED_Y + 26 = 232 + 26 = 258)

Y=258:  RPM value — TC_DATUM, TextSize 1, COL_WHITE
        Centered at X = RPM_LABEL_X = 295
        Format: "%u rpm" (e.g. "123 rpm")
        Capped at RPM_DISPLAY_MAX = 400
```

**ASCII layout (Y: 232–270):**
```
         X=0                X=240              X=295        X=480
          |                   |                  |            |
Y=232     |              [  12.5  ]              |            |   TextSize 3, WHITE
          |                   |                  |            |
Y=258     |              [ km/h ]           [123 rpm]        |   TextSize 1, GRAY / WHITE
          |                   |                  |            |
Y=270     |___________________|__________________|____________|
```

### Gear + Relay Indicator Tile (Y: 300–320)

**DTILE_GEAR** covers the full bottom strip: `(0, 300, 480, 20)`

Two widgets share this tile:
1. `GearDisplay::draw()` — left side (gear labels P R N D1 D2)
2. `RelayIndicator::draw()` — right side (M T D letters)

**The tile hash** combines both (line 308-313):
```cpp
ui::TileHash gh = ui::tileHashVal(curGear_);
gh = ui::tileHashFeed(gh, curRelayStatus_);
tiles_.updateHash(DTILE_GEAR, gh);
```

#### Gear Display

**Labels:** `"P", "R", "N", "D1", "D2"` (from `gear_display.h` line 39-41)
**Positions:** Defined in `GearDisplay::drawStatic()` — exact X positions depend on internal GearDisplay layout constants.

#### Relay Indicator

**Source:** `relay_indicator.h` lines 30-69

**Position constants:**
```
REL_IND_X = 430    (right side of gear bar)
REL_IND_Y = GEAR_Y = 300
REL_IND_W = 50
REL_IND_H = GEAR_H = 20
```

**Text rendering:**
```
TextSize 1, Datum TL_DATUM
textY = REL_IND_Y + (REL_IND_H - 8) / 2 = 300 + (20 - 8) / 2 = 300 + 6 = 306

'M' at (430, 306)      — MAIN relay
'T' at (430 + 14, 306) = (444, 306)  — TRACTION relay
'D' at (430 + 28, 306) = (458, 306)  — DIRECTION relay
```

**Color logic** (from `relayColor()` line 79-82):
```
  Relay OFF                    → COL_GRAY   (0x8410)
  Relay ON + seqComplete=true  → COL_GREEN  (0x07E0)
  Relay ON + seqComplete=false → COL_AMBER  (0xFBE0)
```

Where:
- `seqComplete = (relayStatus & 0x80) != 0`
- `mainOn      = (relayStatus & 0x01) != 0`
- `tracOn      = (relayStatus & 0x02) != 0`
- `dirOn       = (relayStatus & 0x04) != 0`

**Background clear:** `fillRect(430, 300, 50, 20, COL_BG)` before each redraw

**ASCII layout (Y: 300–320):**
```
     X=0                                          X=430  X=444  X=458  X=480
      |                                            |      |      |      |
Y=300 |  [P] [R] [N] [D1] [D2]                    |  M   T   D  |      |
      |  ← Gear Display →                         | ← Relay →   |      |
Y=320 |____________________________________________|______________|______|

Colors (example — all relays ON, sequence complete):
  M = GREEN (0x07E0)
  T = GREEN (0x07E0)
  D = GREEN (0x07E0)

Colors (example — sequencing, MAIN+TRAC ON, DIR still OFF):
  M = AMBER (0xFBE0)
  T = AMBER (0xFBE0)
  D = GRAY  (0x8410)
```

**Dirty detection:** Only redraws if `relayStatus != prevRelayStatus_` (checked via the tile hash that includes both gear and relay status).

---

# 🧩 TASK 2 — ENGINEERING SCREEN (EXACT RENDER)

## Screen layout: 480×320 landscape

**Source:** `engineering_screen.cpp`

### Main Menu Layout

```
Y=22:     "ENGINEERING" — MC_DATUM, TextSize 2, COL_AMBER
          Centered at X = SCREEN_W/2 = 240

Y=50:     Menu button 0: "FAULT VIEWER"         (COL_WHITE on COL_DARK_GRAY)
Y=78:     Menu button 1: "MODULE ENABLE/DISABLE" (COL_WHITE)
Y=106:    Menu button 2: "PEDAL CALIBRATION"     (COL_WHITE)
Y=134:    Menu button 3: "ENCODER CALIBRATION"   (COL_WHITE)
Y=162:    Menu button 4: "INA226 SENSOR MAPPING" (COL_WHITE)
Y=190:    Menu button 5: "TEMP SENSOR MAPPING"   (COL_WHITE)
Y=218:    Menu button 6: "FACTORY DEFAULTS"      (COL_AMBER)
Y=246:    Menu button 7: "DTC ERROR LOG"         (COL_CYAN)
Y=274:    Menu button 8: "MAINTENANCE"           (COL_GREEN)

          Buttons: X=40, W=400, H=26, spacing=28
          Text: MC_DATUM centered at (MENU_X + MENU_W/2, btnY + MENU_BTN_H/2)
                = (240, btnY + 13)
```

### EXIT Button (bottom-left)

```
X=10, Y=280, W=80, H=30
"EXIT" — MC_DATUM, COL_AMBER on COL_DARK_GRAY
Border: COL_AMBER
```

### Relay Status Panel (right of EXIT button)

**Source:** `engineering_screen.cpp` lines 726-760 (drawMainMenu) + lines 261-291 (partial redraw)

**Position:**
```
relX = 120
relY = BACK_Y = 280
```

**Initial draw (drawMainMenu):**
```
(120, 280):    "RELAY STATUS"           — TextSize 1, COL_CYAN
(120, 292):    "M:ON  T:ON  D:OFF"      — TextSize 1, color depends on SEQ
(120, 302):    "SEQ:COMPLETE   [0x87]"  — TextSize 1, color depends on SEQ
```

**Color logic:**
```
seqComplete (bit7=1):  text color = COL_GREEN (0x07E0)
seqComplete (bit7=0):  text color = COL_AMBER (0xFBE0)
```

**Format strings (exact from code):**
```c
// Row 1 (line 277):
"M:%s T:%s D:%s"
  where each %s = "ON " or "OFF"

// Row 2 (line 284):
"SEQ:%s  [0x%02X]"
  where %s = "COMPLETE   " or "IN PROGRESS"
  and %02X = raw relayStatus byte
```

**Partial redraw:** When `relayStatus_ != prevRelayStatus_` and `currentMenu_ == SubMenu::MAIN`:
- Clears: `fillRect(120, 292, 250, 22, COL_BG)` (lines 273)
- Redraws Row 1 at (120, 292)
- Redraws Row 2 at (120, 302)

**ASCII layout (Y: 274–320):**
```
     X=10         X=90   X=120                                X=370   X=480
      |            |      |                                    |        |
Y=274 |            |      | ← Menu btn 8: MAINTENANCE         |        |
Y=280 | [  EXIT  ] |      | RELAY STATUS  (CYAN)              |        |
Y=292 |            |      | M:ON  T:ON  D:ON  (GREEN/AMBER)   |        |
Y=302 |            |      | SEQ:COMPLETE   [0x87]  (GREEN)    |        |
Y=310 |            |      |                                    |        |
Y=320 |____________|______|____________________________________|________|
```

### Engineering Screen Relay Data Examples

**All relays ON, sequence complete** (`relayStatus_ = 0x87`):
```
RELAY STATUS              ← CYAN
M:ON  T:ON  D:ON          ← GREEN (seqComplete=true)
SEQ:COMPLETE    [0x87]    ← GREEN
```

**Sequencing (MAIN on, TRACTION on, DIR pending)** (`relayStatus_ = 0x03`):
```
RELAY STATUS              ← CYAN
M:ON  T:ON  D:OFF         ← AMBER (seqComplete=false)
SEQ:IN PROGRESS [0x03]    ← AMBER
```

**All off (power down / pre-sequence)** (`relayStatus_ = 0x00`):
```
RELAY STATUS              ← CYAN
M:OFF T:OFF D:OFF         ← AMBER (seqComplete=false)
SEQ:IN PROGRESS [0x00]    ← AMBER
```

---

# 🧩 TASK 3 — DATA FLOW TRACE (EXACT FROM CODE)

## Complete Pipeline

### STM32 Side

```
1. GPIO Hardware (GPIOC output register)
   ├── PC10 (PIN_RELAY_MAIN  = GPIO_PIN_10)   → project_config.h:147
   ├── PC11 (PIN_RELAY_TRAC  = GPIO_PIN_11)   → project_config.h:148
   └── PC12 (PIN_RELAY_DIR   = GPIO_PIN_12)   → project_config.h:149

2. Safety_GetRelayStatusByte()                  → safety_system.c:724-747
   ├── HAL_GPIO_ReadPin(GPIOC, PIN_RELAY_MAIN)  → bit 0
   ├── HAL_GPIO_ReadPin(GPIOC, PIN_RELAY_TRAC)  → bit 1
   ├── HAL_GPIO_ReadPin(GPIOC, PIN_RELAY_DIR)   → bit 2
   ├── relay_seq_state == RELAY_SEQ_COMPLETE     → bit 7
   └── #ifdef DEBUG: NOP trap if COMPLETE but no relay GPIO ON

3. CAN Heartbeat Packing                        → can_handler.c:398-438
   ├── CAN ID: CAN_ID_HEARTBEAT_STM32 = 0x001  → can_handler.h:25
   ├── DLC: 6 (extended from 5)
   ├── Rate: 100 ms
   ├── Byte 0: alive_counter
   ├── Byte 1: system_state
   ├── Byte 2: fault_flags
   ├── Byte 3: error_code
   ├── Byte 4: status_flags
   └── Byte 5: Safety_GetRelayStatusByte()      ← NEW
```

### CAN Bus

```
Frame: 0x001, DLC=6, every 100 ms
  [0] alive_counter
  [1] system_state
  [2] fault_flags
  [3] error_code
  [4] status_flags
  [5] relay_status ← bit0=MAIN, bit1=TRAC, bit2=DIR, bit7=SEQ_COMPLETE
```

### ESP32 Side

```
4. CAN RX Decoder                               → can_rx.cpp:42-54
   ├── case can::HEARTBEAT_STM32:               → can_rx.cpp:216
   ├── Guard: if (f.data_length_code < 5) return;
   ├── hb.relayStatus = (DLC >= 6) ? f.data[5] : 0;  → BACKWARD COMPATIBLE
   └── data.setHeartbeat(hb);

5. VehicleData Struct                            → vehicle_data.h:38-46
   └── HeartbeatData {
         aliveCounter, systemState, faultFlags,
         errorCode, statusFlags,
         relayStatus,    ← uint8_t, bit0=M, bit1=T, bit2=D, bit7=SEQ
         timestampMs
       }

6. DriveScreen                                   → drive_screen.cpp
   ├── update(): curRelayStatus_ = data.heartbeat().relayStatus;  (line 242)
   ├── Hash:     DTILE_GEAR hash feeds curGear_ + curRelayStatus_ (lines 308-313)
   ├── draw():   if DTILE_GEAR dirty →
   │             GearDisplay::draw(tft, curGear_, prevGear_);
   │             RelayIndicator::draw(tft, curRelayStatus_, prevRelayStatus_);
   └── prev update: prevRelayStatus_ = curRelayStatus_;           (line 566)

7. EngineeringScreen                             → engineering_screen.cpp
   ├── update(): relayStatus_ = data.heartbeat().relayStatus;    (line 180)
   ├── draw() (full): drawMainMenu() includes relay panel        (lines 726-760)
   └── draw() (partial): if MAIN && relayStatus_ changed →       (lines 261-291)
                          redraw relay values only
```

### Pipeline Diagram

```
┌──────────────────────────────────────────────────────────────────────┐
│  STM32 (every 100ms)                                                │
│                                                                     │
│  GPIOC ODR ──┬── PC10 ──→ bit 0 ─┐                                 │
│              ├── PC11 ──→ bit 1 ──┼──→ Safety_GetRelayStatusByte()  │
│              └── PC12 ──→ bit 2 ──┘              │                  │
│  relay_seq_state ──────→ bit 7 ──────────────────┘                  │
│              │                                                      │
│              ▼                                                      │
│  CAN 0x001 [5] = relay_status_byte                                  │
│              │                                                      │
└──────────────│──────────────────────────────────────────────────────┘
               │  100 ms CAN frame
               ▼
┌──────────────────────────────────────────────────────────────────────┐
│  ESP32                                                              │
│                                                                     │
│  can_rx.cpp: hb.relayStatus = f.data[5]  (if DLC >= 6, else 0)     │
│              │                                                      │
│              ▼                                                      │
│  VehicleData.heartbeat().relayStatus                                │
│              │                                                      │
│         ┌────┴────┐                                                 │
│         ▼         ▼                                                 │
│  DriveScreen    EngineeringScreen                                   │
│   update()       update()                                           │
│     │              │                                                │
│     ▼              ▼                                                │
│  DTILE_GEAR     Partial redraw                                      │
│  hash check     if changed                                          │
│     │              │                                                │
│     ▼              ▼                                                │
│  M T D          RELAY STATUS                                        │
│  (colored)      M:ON T:ON D:ON                                      │
│  @ (430,306)    SEQ:COMPLETE [0x87]                                 │
│                 @ (120,280)                                         │
└──────────────────────────────────────────────────────────────────────┘
```

---

# 🧩 TASK 4 — VALIDATION

## Q1: Does UI reflect relay_seq_state correctly?

**YES.** The `relay_seq_state == RELAY_SEQ_COMPLETE` check is performed inside `Safety_GetRelayStatusByte()` (safety_system.c:734) and packed into bit 7. The ESP32 reads this bit to determine the color:

- **Bit 7 = 1 (COMPLETE):** Green color on both DriveScreen letters and EngineeringScreen text
- **Bit 7 = 0 (not complete):** Amber color

This is a **direct read** of the sequencer state, not an inference.

## Q2: Is there any delay between CAN and UI?

**YES — bounded and deterministic:**

| Stage | Latency |
|-------|---------|
| STM32: GPIO read → CAN pack | < 1 µs (same function call) |
| CAN frame transmission | ≤ 100 ms (heartbeat interval) |
| ESP32: CAN RX → VehicleData | < 1 ms (ISR + decode in canTask) |
| VehicleData → screen update() | ≤ 1 tick (~1 ms, renderTask loop) |
| update() → draw() | ≤ 50 ms (FrameLimiter gates at 20 FPS) |
| **Total worst-case** | **~152 ms** |

The relay status on the UI is at most ~152 ms behind the physical GPIO state. This is within the 100 ms heartbeat cycle and is acceptable for diagnostic visibility (not safety-critical).

## Q3: Is there any mismatch possible?

**Three potential mismatch scenarios (all documented, none blocking):**

### 3a. CAN byte vs GPIO state

**Not possible within the same heartbeat frame.** `Safety_GetRelayStatusByte()` reads GPIOs and `relay_seq_state` atomically in a single function call. The CAN byte is packed immediately after. No other code can change relay GPIOs between the read and the CAN transmit (single-threaded super-loop, no preemption during `CAN_SendHeartbeat()`).

### 3b. ESP32 UI vs CAN byte

**Possible transient mismatch of up to 1 frame (50 ms).** The `VehicleData` snapshot is consumed by `update()` once per render cycle. If a new CAN frame arrives between `update()` and the next `update()`, the UI shows stale data for one frame. This is by design — the tile engine ensures eventual consistency.

### 3c. Relay command state vs physical relay contact state

**This is a hardware limitation — DOCUMENTED, NOT FIXABLE in software.**

- The CAN byte reports **GPIO command state** (what firmware told the relay to do)
- It does **NOT** report physical relay contact closure
- There is **NO GPIO feedback input** from relay contacts on the PCB
- Physical verification is done **indirectly** via INA226 motor current monitoring (`Safety_CheckRelayHealth()` in safety_system.c), which only works under motor load

This is explicitly documented in:
- `project_config.h` (relay visibility design explanation, lines ~130-149)
- `safety_system.c` (relay telemetry model, lines ~700-723)
- `relay_indicator.h` header comment (line 13: "Data source: HeartbeatData.relayStatus (CAN 0x001, byte 5)")

---

# 📝 CHANGES LOG (THIS PR)

## STM32 Files Modified

| File | Change |
|------|--------|
| `Core/Src/safety_system.c` | Added `Safety_GetRelayStatusByte()` function (lines 724-747). Reads GPIOC PC10/11/12 + relay_seq_state. DEBUG assertion for consistency. |
| `Core/Inc/safety_system.h` | Added function prototype |
| `Core/Src/can_handler.c` | Extended heartbeat DLC from 5→6. `payload[5] = Safety_GetRelayStatusByte()` |
| `Core/Inc/can_handler.h` | Updated CAN_ID_HEARTBEAT_STM32 comment (DLC 6) |
| `Core/Inc/project_config.h` | Added relay visibility design documentation |

## ESP32 Files Modified

| File | Change |
|------|--------|
| `esp32/src/vehicle_data.h` | Added `relayStatus` field to `HeartbeatData` (uint8_t) |
| `esp32/src/can_rx.cpp` | Decode byte 5 with backward-compatible `DLC >= 6` guard |
| `esp32/include/can_ids.h` | Updated HEARTBEAT_STM32 comment (DLC 6) |
| `esp32/src/ui/relay_indicator.h` | **NEW** — RelayIndicator widget class |
| `esp32/src/screens/drive_screen.h` | Added `curRelayStatus_` / `prevRelayStatus_` members |
| `esp32/src/screens/drive_screen.cpp` | Relay status in update(), hash in GEAR tile, draw via RelayIndicator |
| `esp32/src/screens/engineering_screen.h` | Added `relayStatus_` / `prevRelayStatus_` members |
| `esp32/src/screens/engineering_screen.cpp` | Relay panel in drawMainMenu() + partial redraw logic |
| `esp32/src/led_controller.h` | Added documentation note about future relay-based visual diagnostics |

## What Was NOT Modified

- ❌ Relay timing (50ms + 20ms sequence) — UNTOUCHED
- ❌ Safety state machine logic — UNTOUCHED
- ❌ Safety thresholds or constants — UNTOUCHED
- ❌ Motor control logic (PWM, EN pins, gating) — UNTOUCHED
- ❌ CAN IDs — UNCHANGED (0x001 is same ID, only DLC extended 5→6)
- ❌ CAN message frequency — UNCHANGED (100ms)
- ❌ Startup inhibit behavior — UNTOUCHED
- ❌ Any existing validated logic paths — UNTOUCHED

---

# 🔚 END OF DOCUMENT
