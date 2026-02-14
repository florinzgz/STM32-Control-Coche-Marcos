# HMI Rendering Strategy — Phase 14

**Revision:** 1.0
**Status:** Implementation ready
**Date:** 2026-02-13
**Scope:** Safe ESP32 HMI rendering architecture for STM32-Control-Coche-Marcos

---

## 1. Analysis of Reference Implementation (FULL-FIRMWARE-Coche-Marcos)

### 1.1 Rendering Model

The original implementation used:
- **TFT_eSPI** library with **ST7796 driver** (320×480 TFT)
- A monolithic `hud.cpp` (68 KB) containing all rendering logic
- A `hud_compositor.cpp` (26 KB) for layer compositing
- A `hud_manager.cpp` (52 KB) for state management
- Multiple menu screens mixed into the HUD directory
- Heavy use of `TFT_eSprite` for off-screen rendering

### 1.2 Root Cause Hypothesis — Reboot Loop

Based on analysis of the reference repository structure and its extensive bootloop fix documentation (20+ bootloop fix documents), the likely root causes are:

1. **PSRAM exhaustion**: The reference board (N16R8) used 8MB PSRAM with Octal PSRAM configuration. Multiple `TFT_eSprite` instances allocated from PSRAM could cause fragmentation and allocation failures.

2. **Stack overflow**: The `hud.cpp` at 68 KB suggests deep call stacks with recursive menu rendering. Combined with the `hud_compositor.cpp` layer system, stack depth could exceed task limits.

3. **Watchdog starvation**: Full-screen redraws at 320×480 pixels with SPI at 40 MHz take ~12 ms minimum per frame. Complex rendering with sprites, gradients, and text could exceed the watchdog timeout (typically 5 seconds, but accumulated blocking can trigger it).

4. **Dynamic allocation in loop()**: Creating/destroying `TFT_eSprite` objects or `String` objects inside the render loop causes heap fragmentation. On ESP32, this leads to eventual allocation failure and crash.

5. **Flash/PSRAM pin conflicts**: The N16R8 board configuration shows explicit avoidance of GPIOs 10-12 (flash) and 33-37 (PSRAM). Misconfiguration here causes immediate boot failure.

### 1.3 Heap Risk Analysis

| Risk | Severity | Mitigation |
|------|----------|------------|
| Sprite allocation in loop() | **Critical** | Pre-allocate all sprites in setup() |
| String concatenation | **High** | Use fixed char[] buffers with snprintf() |
| Dynamic container growth | **Medium** | Use std::array with fixed sizes |
| Font loading | **Low** | Fonts are in flash, read-only |

### 1.4 Stack Usage Risks

| Risk | Severity | Mitigation |
|------|----------|------------|
| Deep menu recursion | **Critical** | Flat state machine, no recursion |
| Large local buffers | **High** | Max 64 bytes per format buffer |
| Nested draw calls | **Medium** | Maximum 3 levels of function nesting in draw path |

### 1.5 CPU Load Estimation

| Operation | Estimated Time |
|-----------|---------------|
| SPI transfer 320×480 full frame | ~12 ms at 40 MHz |
| Partial redraw (changed regions only) | ~1-3 ms |
| Text rendering (10 labels) | ~0.5 ms |
| Geometric primitives (car body, wheels) | ~0.3 ms |
| Gradient pedal bar | ~0.4 ms |
| **Total estimated per frame** | **~2-4 ms (partial redraw)** |

### 1.6 Current ESP32 Repo Analysis

| Component | Status |
|-----------|--------|
| screen_manager.cpp | Functional state machine, calls update()+draw() every loop |
| vehicle_data | Clean data store, no logic, 12 data structs |
| can_rx | Non-blocking CAN polling, 13 frame decoders |
| main.cpp loop | ~100 Hz effective (CAN poll + heartbeat), no frame limiting |
| CAN polling frequency | Every loop iteration (non-blocking readFrame with timeout=0) |

---

## 2. Safe Rendering Architecture

### 2.1 Design Principles

1. **Deterministic render time**: Every frame completes in <5 ms
2. **No heap allocation in loop()**: All buffers pre-allocated at init
3. **Partial redraw only**: Only changed values are redrawn
4. **Frame rate limited**: Max 20 FPS (50 ms interval) — sufficient for telemetry
5. **Separation of update() and draw()**: Data preparation is separate from pixel output
6. **No String class**: All formatting uses fixed char[] with snprintf()
7. **No recursion**: Flat function call hierarchy
8. **Bounded loops only**: Every loop has a compile-time maximum iteration count

### 2.2 Frame Limiter

```
Frame interval: 50 ms (20 FPS)
Method: millis() comparison in screen_manager
Skip draw() if interval not elapsed
Always call update() for data freshness
```

### 2.3 Static Background vs Partial Redraw Strategy

| Element | Strategy |
|---------|----------|
| Car body outline | Draw once on screen enter, never redraw |
| Wheel rectangles (4) | Draw once, redraw fill color on torque change |
| Speed value | Partial redraw: clear old text area, draw new |
| Steering angle indicator | Partial redraw: clear+redraw rotation indicator |
| Battery percentage | Partial redraw on value change |
| Pedal bar | Partial redraw: update fill width only |
| Gear indicator | Partial redraw: unhighlight old, highlight new |
| Mode icons | Draw once, toggle highlight on state change |
| Torque/temp labels per wheel | Partial redraw on value change |

### 2.4 Memory-Safe Sprite Strategy

**Decision: No sprites used.**

Rationale: For this application, direct TFT drawing with partial redraw is safer and uses zero additional RAM. Sprites require PSRAM or large heap allocations. The display elements are simple enough (rectangles, text, lines) that sprites provide no benefit.

All drawing uses:
- `tft.fillRect()` for backgrounds and bars
- `tft.drawRect()` for outlines
- `tft.drawLine()` for car body lines
- `tft.setCursor()` + `tft.print()` for text with fixed char[] buffers
- `tft.fillTriangle()` for directional indicators

### 2.5 Rendering Pipeline

```
loop() {
    can_rx::poll()          // Non-blocking CAN receive
    ackCheck()              // Non-blocking ACK check
    screenManager.update()  // Always: read vehicle data
    screenManager.draw()    // Only if 50ms elapsed: render changes
    heartbeat()             // 100ms interval
}
```

### 2.6 Data Flow for Draw

```
update(vehicleData):
    1. Copy relevant fields to local cache (fixed-size struct)
    2. Compare with previous frame cache
    3. Set dirty flags for changed fields
    4. Format numeric strings into pre-allocated char[] buffers

draw():
    1. Check frame limiter — skip if too early
    2. For each dirty flag:
       a. Clear the specific screen region (fillRect with background)
       b. Draw the new value
    3. Clear all dirty flags
    4. Record frame timestamp
```

### 2.7 Screen Layout (320×480 TFT, Portrait)

```
┌──────────────────────────────────┐  0
│  [4x4] [4x2] [360°]    [BAT%]  │  0-35
├──────────────────────────────────┤  35
│         SPEED: XX.X km/h        │  35-70
├──────────────────────────────────┤  70
│                                  │
│    ┌──┐              ┌──┐       │
│    │FL│              │FR│       │  Wheel boxes
│    └──┘              └──┘       │  with torque %
│       ┌──────────┐              │  and temp
│       │          │              │
│       │  CAR     │  ←steering   │  Car body
│       │  BODY    │              │
│       │          │              │
│       └──────────┘              │
│    ┌──┐              ┌──┐       │
│    │RL│              │RR│       │
│    └──┘              └──┘       │
│                                  │
├──────────────────────────────────┤  380
│  P  R  N  D1  D2                │  380-410
├──────────────────────────────────┤  410
│  ████████░░░░░░░░░  75%   ▶    │  410-475
│  PEDAL BAR (gradient)            │
└──────────────────────────────────┘  480
```

### 2.8 Color Palette (RGB565)

| Element | Color | RGB565 |
|---------|-------|--------|
| Background | Dark gray | 0x2104 |
| Car body | White | 0xFFFF |
| Wheel normal (0-50%) | Green | 0x07E0 |
| Wheel medium (51-80%) | Yellow | 0xFFE0 |
| Wheel high (81-100%) | Red | 0xF800 |
| Speed text | White | 0xFFFF |
| Battery text | Cyan | 0x07FF |
| Active gear | Green | 0x07E0 |
| Inactive gear | Gray | 0x8410 |
| Pedal bar green | Green | 0x07E0 |
| Pedal bar yellow | Yellow | 0xFFE0 |
| Pedal bar red | Red | 0xF800 |
| Mode icon active | Cyan | 0x07FF |
| Mode icon inactive | Gray | 0x8410 |

---

## 3. File Structure

```
esp32/src/
├── main.cpp              (modified: TFT init, frame limiter)
├── screen_manager.cpp/h  (modified: frame limiter integration)
├── ui/
│   ├── ui_common.h       (layout constants, colors, helpers)
│   ├── frame_limiter.h   (frame rate control)
│   ├── car_renderer.h    (vehicle drawing with wheels)
│   ├── car_renderer.cpp
│   ├── pedal_bar.h       (pedal bar widget)
│   ├── pedal_bar.cpp
│   ├── gear_display.h    (gear indicator)
│   ├── gear_display.cpp
│   ├── battery_indicator.h (battery percentage)
│   ├── battery_indicator.cpp
│   └── mode_icons.h      (4x4, 4x2, 360° icons)
│       mode_icons.cpp
├── screens/
│   ├── drive_screen.cpp  (full implementation)
│   ├── boot_screen.cpp   (splash screen)
│   ├── standby_screen.cpp (ready screen)
│   ├── safe_screen.cpp   (safety alert)
│   └── error_screen.cpp  (error display)
```

---

## References

| Document | Relevance |
|----------|-----------|
| docs/CAN_CONTRACT_FINAL.md rev 1.3 | CAN message definitions |
| docs/HMI_STATE_MODEL.md rev 1.1 | Screen behavior specification |
| docs/ESP32_FIRMWARE_DESIGN.md | Architecture decisions |
| FULL-FIRMWARE-Coche-Marcos (GitHub) | Reference for analysis (not copied) |
