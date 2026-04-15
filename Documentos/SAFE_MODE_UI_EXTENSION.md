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
| **Relay Status** | M T D indicators (MAIN/TRACTION/DIRECTION) with color coding |

## Data Sources

| Data | Source | File:Line |
|------|--------|-----------|
| Gear position | `shifter::getGearRaw()` (MCP23017 I2C) | `esp32/src/shifter_input.h:51` |
| Steering angle | `VehicleData.steering().angleRaw` | `esp32/src/vehicle_data.h:86` |
| Obstacle distance | `VehicleData.obstacle().distanceCm` | `esp32/src/vehicle_data.h:152` |
| Front/Rear LED relay | `VehicleData.lights().frontRelayOn/rearRelayOn` | `esp32/src/vehicle_data.h:162-163` |
| Turn signal | `led_ctrl::getTurnSignal()` (local ESP32 state) | `esp32/src/led_controller.h:148` |
| Relay status | `VehicleData.heartbeat().relayStatus` | `esp32/src/vehicle_data.h:45` |

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
