# Runtime Monitor Verification Audit

**Date:** 2026-02-15
**Files audited:** `main.cpp`, `screen_manager.cpp`, `drive_screen.cpp`,
`runtime_monitor.h`, `runtime_monitor.cpp`, `debug_overlay.h`,
`debug_overlay.cpp`, `frame_limiter.h`

---

## 1) Frame Timing Validity

✔ CONFIRMED

Full call chain with line numbers:

```
loop()                              main.cpp:111
├─ RTMON_CAN_BEGIN()                main.cpp:115
│  └─ can_rx::poll()               main.cpp:116
├─ RTMON_CAN_END()                  main.cpp:117
├─ RTMON_UI_BEGIN()                 main.cpp:123
│  └─ screenManager.update()       main.cpp:124
│     ├─ currentScreen_->update()  screen_manager.cpp:33
│     └─ if shouldDraw():          screen_manager.cpp:36
│        ├─ RTMON_FRAME_BEGIN()    screen_manager.cpp:37
│        ├─ RTMON_RENDER_BEGIN()   screen_manager.cpp:38
│        ├─ currentScreen_->draw() screen_manager.cpp:39
│        ├─ RTMON_RENDER_END()     screen_manager.cpp:40
│        └─ RTMON_FRAME_END()      screen_manager.cpp:41
├─ RTMON_UI_END()                   main.cpp:125
├─ RTMON_OVERLAY_UPDATE()           main.cpp:130
├─ RTMON_OVERLAY_DRAW()             main.cpp:131
└─ RTMON_LOG() (every 5s)           main.cpp:157
```

Frame cycle is: `FRAME_BEGIN → RENDER_BEGIN → draw() → RENDER_END → FRAME_END`

All five phases in correct order. No gaps. ✔

---

## 2) Micros Accuracy

✔ CONFIRMED

Every timing measurement uses `micros()`. Complete inventory:

| Function | Start (micros) | End (micros) | File:Line |
|----------|---------------|-------------|-----------|
| `frameBegin()` | `frameStartUs_ = micros()` | — | `runtime_monitor.cpp:57` |
| `frameEnd()` | — | `micros() - frameStartUs_` | `runtime_monitor.cpp:61` |
| `canBegin()` | `canStartUs_ = micros()` | — | `runtime_monitor.cpp:91` |
| `canEnd()` | — | `micros() - canStartUs_` | `runtime_monitor.cpp:95` |
| `uiBegin()` | `uiStartUs_ = micros()` | — | `runtime_monitor.cpp:105` |
| `uiEnd()` | — | `micros() - uiStartUs_` | `runtime_monitor.cpp:109` |
| `renderBegin()` | `renderStartUs_ = micros()` | — | `runtime_monitor.cpp:119` |
| `renderEnd()` | — | `micros() - renderStartUs_` | `runtime_monitor.cpp:123` |

Start/end pairs:
- `frameStartUs_` → `frameEnd()` line 61: `micros() - frameStartUs_` ✔
- `canStartUs_` → `canEnd()` line 95: `micros() - canStartUs_` ✔
- `uiStartUs_` → `uiEnd()` line 109: `micros() - uiStartUs_` ✔
- `renderStartUs_` → `renderEnd()` line 123: `micros() - renderStartUs_` ✔

No early returns between begin/end pairs:
- `frameBegin()` at `screen_manager.cpp:37` → `frameEnd()` at `screen_manager.cpp:41` — straight-line code, no returns between them ✔
- `canBegin()` at `main.cpp:115` → `canEnd()` at `main.cpp:117` — one function call between, no returns ✔
- `uiBegin()` at `main.cpp:123` → `uiEnd()` at `main.cpp:125` — one function call between, no returns ✔
- `renderBegin()` at `screen_manager.cpp:38` → `renderEnd()` at `screen_manager.cpp:40` — one function call between, no returns ✔

Exception: `millis()` is used only for FPS window calculation (`runtime_monitor.cpp:75`), not for frame timing. This is correct — FPS is frames-per-wall-clock-second. ✔

---

## 3) Overlay Execution Proof

✔ CONFIRMED

Call order in `main.cpp`:

```
line 123: RTMON_UI_BEGIN()
line 124: screenManager.update(vehicleData)   ← normal frame render happens here
line 125: RTMON_UI_END()
line 130: RTMON_OVERLAY_UPDATE(touched)        ← AFTER render
line 131: RTMON_OVERLAY_DRAW(tft)              ← AFTER render
```

The overlay draw (`main.cpp:131`) executes AFTER `screenManager.update()` (`main.cpp:124`) which contains the `draw()` call (`screen_manager.cpp:39`).

The overlay draw is OUTSIDE the `RTMON_FRAME_BEGIN/END` bracket (`screen_manager.cpp:37-41`) and OUTSIDE the `RTMON_RENDER_BEGIN/END` bracket (`screen_manager.cpp:38-40`).

Therefore: overlay drawing is **not counted** in frame timing and **cannot affect** layout timing measurements. ✔

---

## 4) Zone Redraw Truth

### top_bar

✔ CONFIRMED (after bug fix)

- Dirty flag (battery): `curBattVoltRaw_ != prevBattVoltRaw_` → `drive_screen.cpp:198`
- RTMON: `RTMON_ZONE_REDRAW(rtmon::Zone::TOP_BAR)` → `drive_screen.cpp:199`
- Dirty flag (mode icons): `curMode_.is4x4 != prevMode_.is4x4 || curMode_.isTankTurn != prevMode_.isTankTurn` → `drive_screen.cpp:218`
- RTMON: `RTMON_ZONE_REDRAW(rtmon::Zone::TOP_BAR)` → `drive_screen.cpp:219`

### obstacle

✔ CONFIRMED

- Dirty flag: `curObstacleCm_ != prevObstacleCm_` → `drive_screen.cpp:170`
- RTMON: `RTMON_ZONE_REDRAW(rtmon::Zone::OBSTACLE)` → `drive_screen.cpp:171`

### car

✔ CONFIRMED

- Dirty flag: `curTraction_[i] != prevTraction_[i] || curTemp_[i] != prevTemp_[i]` → `drive_screen.cpp:179`
- Dirty flag: `curSteeringRaw_ != prevSteeringRaw_` → `drive_screen.cpp:184`
- RTMON: `RTMON_ZONE_REDRAW(rtmon::Zone::CAR)` → `drive_screen.cpp:185`

### speed

✔ CONFIRMED

- Dirty flag: `curSpeedAvgRaw_ != prevSpeedAvgRaw_` → `drive_screen.cpp:164`
- RTMON: `RTMON_ZONE_REDRAW(rtmon::Zone::SPEED)` → `drive_screen.cpp:165`
- Internal early return: `drawSpeed()` returns early at `drive_screen.cpp:234` if equal — consistent with dirty flag check ✔

### pedal

✔ CONFIRMED

- Dirty flag: `curPedalPct_ != prevPedalPct_` → `drive_screen.cpp:210`
- RTMON: `RTMON_ZONE_REDRAW(rtmon::Zone::PEDAL)` → `drive_screen.cpp:211`

### gear

✔ CONFIRMED

- Dirty flag: `curGear_ != prevGear_` → `drive_screen.cpp:204`
- RTMON: `RTMON_ZONE_REDRAW(rtmon::Zone::GEAR)` → `drive_screen.cpp:205`

### Full redraw

✔ CONFIRMED

- Trigger: `needsFullRedraw_ == true` → `drive_screen.cpp:117`
- RTMON: `RTMON_FULL_REDRAW()` → `drive_screen.cpp:119`

### Previous BUG (now fixed)

ModeIcons at `drive_screen.cpp:216` (original line) could redraw when `curMode_ != prevMode_` without `RTMON_ZONE_REDRAW(TOP_BAR)`.

**Fix applied:** Added dirty check + `RTMON_ZONE_REDRAW(rtmon::Zone::TOP_BAR)` before `ModeIcons::draw()` at `drive_screen.cpp:218-219`.

No remaining uncounted redraws. ✔

---

## 5) Blocking Detection Validity

✔ CONFIRMED

CAN blocking timer boundaries in `main.cpp`:

```cpp
line 115: RTMON_CAN_BEGIN();          // canStartUs_ = micros()
line 116: can_rx::poll(vehicleData);  // ← ONLY CAN read/parse
line 117: RTMON_CAN_END();            // elapsed = micros() - canStartUs_
```

Between `CAN_BEGIN` and `CAN_END` there is exactly ONE function call: `can_rx::poll()`.

`ackCheck()` at `main.cpp:120` is OUTSIDE the CAN timing bracket. ✔
Heartbeat TX at `main.cpp:134-145` is OUTSIDE the CAN timing bracket. ✔
Serial output at `main.cpp:148-151` is OUTSIDE the CAN timing bracket. ✔

The CAN timer measures ONLY `can_rx::poll()`. ✔

UI timer boundaries in `main.cpp`:

```cpp
line 123: RTMON_UI_BEGIN();                    // uiStartUs_ = micros()
line 124: screenManager.update(vehicleData);   // ← update + conditional draw
line 125: RTMON_UI_END();                      // elapsed = micros() - uiStartUs_
```

Note: UI timer includes both `update()` and conditional `draw()` — this is intentional since `screen_manager.cpp:21-42` bundles both in `update()`. ✔

Render timer boundaries in `screen_manager.cpp`:

```cpp
line 38: RTMON_RENDER_BEGIN();     // renderStartUs_ = micros()
line 39: currentScreen_->draw();  // ← ONLY draw
line 40: RTMON_RENDER_END();      // elapsed = micros() - renderStartUs_
```

Render timer measures ONLY `draw()`. ✔

---

## 6) Compile-Out Safety

✔ CONFIRMED

When `RUNTIME_MONITOR = 0`:

### No RAM reserved

All static members are inside `#if RUNTIME_MONITOR` block:
- `runtime_monitor.cpp:14`: `#if RUNTIME_MONITOR` → entire file compiles to empty
- `runtime_monitor.h:29`: `#if RUNTIME_MONITOR` → all class/struct/enum definitions skipped
- `debug_overlay.cpp:11`: `#if RUNTIME_MONITOR` → entire file compiles to empty
- `debug_overlay.h:18`: `#if RUNTIME_MONITOR` → class definition skipped

Static members that would occupy RAM:
- `ringBuf_[120]` (480 bytes) — `runtime_monitor.cpp:24` — inside `#if` ✔
- `zoneCounts_[6]` (12 bytes) — `runtime_monitor.cpp:45` — inside `#if` ✔
- All 20 other static variables — `runtime_monitor.cpp:25-50` — inside `#if` ✔
- `DebugOverlay` statics (14 bytes) — `debug_overlay.cpp:21-24` — inside `#if` ✔

`lastRtMonMs` in `main.cpp:35` — inside `#if RUNTIME_MONITOR` ✔

### No function calls remain

All macros expand to `((void)0)` when disabled:

```cpp
// runtime_monitor.h:194-205
#define RTMON_FRAME_BEGIN()      ((void)0)
#define RTMON_FRAME_END()        ((void)0)
#define RTMON_CAN_BEGIN()        ((void)0)
#define RTMON_CAN_END()          ((void)0)
#define RTMON_UI_BEGIN()         ((void)0)
#define RTMON_UI_END()           ((void)0)
#define RTMON_RENDER_BEGIN()     ((void)0)
#define RTMON_RENDER_END()       ((void)0)
#define RTMON_ZONE_REDRAW(z)     ((void)0)
#define RTMON_FULL_REDRAW()      ((void)0)
#define RTMON_LOG()              ((void)0)
#define RTMON_RESET()            ((void)0)
```

```cpp
// debug_overlay.h:64-66
#define RTMON_OVERLAY_UPDATE(touch)   (false)
#define RTMON_OVERLAY_DRAW(tft)       ((void)0)
#define RTMON_OVERLAY_VISIBLE()       (false)
```

### No timing overhead

Preprocessor expansion example for `main.cpp` loop() with `RUNTIME_MONITOR = 0`:

```cpp
void loop() {
    unsigned long now = millis();

    ((void)0);                          // RTMON_CAN_BEGIN() → noop
    can_rx::poll(vehicleData);
    ((void)0);                          // RTMON_CAN_END() → noop

    ackCheck(vehicleData);

    ((void)0);                          // RTMON_UI_BEGIN() → noop
    screenManager.update(vehicleData);
    ((void)0);                          // RTMON_UI_END() → noop

    // #if RUNTIME_MONITOR block skipped entirely (lines 127-132)
    // #if RUNTIME_MONITOR block skipped entirely (lines 153-159)

    // ... heartbeat + serial unchanged
}
```

All `((void)0)` expressions are optimized out by compiler (-O2). Zero code generated. ✔

---

## 7) Worst-Case Scenario Correctness

✔ CONFIRMED — FPS drops, no frame skip.

The frame limiter (`frame_limiter.h:29-36`) uses a non-blocking comparison:

```cpp
bool shouldDraw() {
    unsigned long now = millis();
    if (now - lastFrameMs_ >= intervalMs_) {   // line 31
        lastFrameMs_ = now;                     // line 32 — resets to NOW
        return true;
    }
    return false;
}
```

If `draw()` takes longer than `intervalMs_` (50 ms):

1. `shouldDraw()` returns true, `lastFrameMs_` set to time T.
2. `draw()` executes, taking e.g. 80 ms.
3. Next `loop()` iteration: `now - lastFrameMs_` = ~80 ms ≥ 50 ms → `shouldDraw()` returns true immediately.
4. `lastFrameMs_` is set to `now` (T+80), NOT `lastFrameMs_ + intervalMs_`.

This means:
- **No frame skip** — every frame that is due is drawn.
- **FPS drops** — instead of 20 FPS, effective FPS = 1000/80 ≈ 12 FPS.
- **No accumulation** — because `lastFrameMs_ = now` (not `+= interval`), missed time is not recovered. One slow frame does not cause a burst of catch-up frames.

The runtime monitor correctly captures this:
- `frameEnd()` records the actual elapsed time in the ring buffer (`runtime_monitor.cpp:69`)
- `maxFrameUs_` is updated (`runtime_monitor.cpp:65`)
- `renderBlocking_` is set if >4ms (`runtime_monitor.cpp:125`)
- FPS counter reflects actual rendered frames per wall-clock second (`runtime_monitor.cpp:78-83`)

---

## Summary

| Point | Status | Bug Found? |
|-------|--------|------------|
| 1. Frame timing call chain | ✔ CONFIRMED | — |
| 2. micros() accuracy | ✔ CONFIRMED | — |
| 3. Overlay after render | ✔ CONFIRMED | — |
| 4. Zone redraw truth | ✔ CONFIRMED (after fix) | ✖ BUG FIXED: ModeIcons uncounted redraw |
| 5. CAN blocking boundaries | ✔ CONFIRMED | — |
| 6. Compile-out safety | ✔ CONFIRMED | — |
| 7. Worst-case behavior | ✔ CONFIRMED (FPS drops) | — |

**Bug fixed:** `drive_screen.cpp` — ModeIcons redraw was not counted by `RTMON_ZONE_REDRAW(TOP_BAR)`. Added dirty check before `ModeIcons::draw()` call.
