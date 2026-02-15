# Display Stability Validation — Final Screen Audit

**Date:** 2026-02-15
**Scope:** Drive Screen rendering on 480×320 TFT (landscape)

---

## 1) Deterministic Rendering

**Exact call chain from `loop()` to zone rendering:**

```
loop()                                          main.cpp:111
├─ RTMON_CAN_BEGIN()                            main.cpp:115
├─ can_rx::poll(vehicleData)                    main.cpp:116
├─ RTMON_CAN_END()                              main.cpp:117
├─ ackCheck(vehicleData)                        main.cpp:120
├─ RTMON_UI_BEGIN()                             main.cpp:123
├─ screenManager.update(vehicleData)            main.cpp:124
│   ├─ currentScreen_->update(data)             screen_manager.cpp:33
│   └─ if (frameLimiter_.shouldDraw())          screen_manager.cpp:36
│       ├─ RTMON_FRAME_BEGIN()                  screen_manager.cpp:37
│       ├─ RTMON_RENDER_BEGIN()                 screen_manager.cpp:38
│       ├─ currentScreen_->draw()               screen_manager.cpp:39
│       │   ├─ [if needsFullRedraw_]            drive_screen.cpp:117
│       │   │   └─ fillScreen + drawStatic      drive_screen.cpp:122–130
│       │   ├─ drawSpeed()                      drive_screen.cpp:167
│       │   ├─ ObstacleSensor::draw()           drive_screen.cpp:173
│       │   ├─ CarRenderer::drawWheels()        drive_screen.cpp:188
│       │   ├─ CarRenderer::drawSteering()      drive_screen.cpp:195
│       │   ├─ BatteryIndicator::draw()         drive_screen.cpp:201
│       │   ├─ GearDisplay::draw()              drive_screen.cpp:207
│       │   ├─ PedalBar::draw()                 drive_screen.cpp:213
│       │   ├─ ModeIcons::draw()                drive_screen.cpp:219
│       │   └─ copy cur→prev                    drive_screen.cpp:221–230
│       ├─ RTMON_RENDER_END()                   screen_manager.cpp:40
│       └─ RTMON_FRAME_END()                    screen_manager.cpp:41
├─ RTMON_UI_END()                               main.cpp:125
├─ RTMON_OVERLAY_UPDATE(touched)                main.cpp:130
├─ RTMON_OVERLAY_DRAW(tft)                      main.cpp:131
├─ heartbeat TX                                 main.cpp:134–145
├─ serial heartbeat                             main.cpp:148–151
└─ RTMON_LOG()                                  main.cpp:155–158
```

**Order is deterministic and identical every iteration.**
`update()` always runs. `draw()` only runs when `frameLimiter_.shouldDraw()` returns true.
Zone draw order is fixed: speed → obstacle → car → steering → battery → gear → pedal → mode.
Overlay always runs AFTER normal rendering.

---

## 2) Flicker Prevention

### Does full screen redraw still exist?

YES — but **only** inside the `needsFullRedraw_` guard:

```cpp
// drive_screen.cpp:117-118
if (needsFullRedraw_) {
    needsFullRedraw_ = false;    // ← immediately cleared
    tft.fillScreen(ui::COL_BG); // ← only here
```

`needsFullRedraw_` is set to `true` only in `onEnter()` (`drive_screen.cpp:40`).
`onEnter()` is called only on screen transitions (`screen_manager.cpp:28`).
During normal operation, `needsFullRedraw_` remains `false`. No full screen clear occurs.

The `fillScreen` at `main.cpp:92` is in `setup()` — runs once at boot.

### How is partial redraw guaranteed?

Each zone draw function receives both `current` and `previous` values.
Each function internally compares them and returns immediately if equal:

| Zone | Guard | File:Line |
|------|-------|-----------|
| speed | `if (curSpeedAvgRaw_ == prevSpeedAvgRaw_) return;` | `drive_screen.cpp:237` |
| obstacle | `curObstacleCm_` vs `prevObstacleCm_` passed to `ObstacleSensor::draw()` | `drive_screen.cpp:173` |
| car | `curTraction_[i]` vs `prevTraction_[i]` + `curTemp_[i]` vs `prevTemp_[i]` passed to `drawWheels()` | `drive_screen.cpp:188–192` |
| steering | `curSteeringRaw_` vs `prevSteeringRaw_` passed to `drawSteering()` | `drive_screen.cpp:195` |
| battery | `curBattVoltRaw_` vs `prevBattVoltRaw_` passed to `BatteryIndicator::draw()` | `drive_screen.cpp:201` |
| gear | `curGear_` vs `prevGear_` passed to `GearDisplay::draw()` | `drive_screen.cpp:207` |
| pedal | `curPedalPct_` vs `prevPedalPct_` passed to `PedalBar::draw()` | `drive_screen.cpp:213` |
| mode | `curMode_` vs `prevMode_` passed to `ModeIcons::draw()` | `drive_screen.cpp:219` |

### What prevents two redraws of the same zone in one frame?

Each zone is drawn exactly once per frame in a fixed sequential order (`drive_screen.cpp:163–219`).
After all zones are drawn, current values are copied to previous (`drive_screen.cpp:221–230`).
The copy happens ONCE at the end — not per-zone — so no zone can trigger twice.

### What happens when multiple values change at the same time?

All changed zones are drawn in the same frame — each exactly once.
Each zone draw checks its own dirty condition independently.
Multiple dirty zones in one frame = multiple small redraws, but each zone only once.

**No flicker possible during normal operation.**

---

## 3) Dirty Zone Logic Verification

Exact `if(...)` conditions that trigger each zone redraw:

### top_bar

```cpp
// drive_screen.cpp:198
if (curBattVoltRaw_ != prevBattVoltRaw_)

// drive_screen.cpp:216
if (curMode_.is4x4 != prevMode_.is4x4 || curMode_.isTankTurn != prevMode_.isTankTurn)
```

### obstacle

```cpp
// drive_screen.cpp:170
if (curObstacleCm_ != prevObstacleCm_)
```

### car

```cpp
// drive_screen.cpp:178–184
for (uint8_t i = 0; i < 4; ++i) {
    if (curTraction_[i] != prevTraction_[i] || curTemp_[i] != prevTemp_[i]) {
        carDirty = true;
        break;
    }
}
if (carDirty || curSteeringRaw_ != prevSteeringRaw_)
```

### speed

```cpp
// drive_screen.cpp:164
if (curSpeedAvgRaw_ != prevSpeedAvgRaw_)

// Also internal guard in drawSpeed():
// drive_screen.cpp:237
if (curSpeedAvgRaw_ == prevSpeedAvgRaw_) return;
```

### pedal

```cpp
// drive_screen.cpp:210
if (curPedalPct_ != prevPedalPct_)
```

### gear

```cpp
// drive_screen.cpp:204
if (curGear_ != prevGear_)
```

---

## 4) Frame Time Safety

### Expected average FPS

`FrameLimiter` interval = 50 ms → target **20 FPS** (`frame_limiter.h:20`).

### Worst recorded frame

The runtime monitor tracks `maxFrameUs_` via the ring buffer (`runtime_monitor.cpp:65`):
```cpp
if (elapsed > maxFrameUs_) maxFrameUs_ = elapsed;
```
Available in `getStats().maxFrameUs` and printed in `[RT]` serial log.

If `maxFrameUs` exceeds 4000 µs (4 ms), `renderBlocking_` is set to `true`
(`runtime_monitor.cpp:125`) and logged as `[RT] WARN blocking: render=YES`.

### What happens if frame > 33 ms (30 FPS threshold)?

The frame limiter uses `lastFrameMs_ = now` (not `+= interval`):
```cpp
// frame_limiter.h:31-32
if (now - lastFrameMs_ >= intervalMs_) {
    lastFrameMs_ = now;
    return true;
}
```

If `draw()` takes 80 ms, the next `shouldDraw()` triggers immediately (80 > 50).
FPS drops to `1000/80 ≈ 12 FPS`. No frames are skipped — each draws once when ready.
No backlog accumulates because `lastFrameMs_ = now` resets the window.

### Does UI skip updates or queue them?

`currentScreen_->update(data)` runs every `loop()` iteration (`screen_manager.cpp:33`).
`currentScreen_->draw()` only runs when `shouldDraw()` returns true (`screen_manager.cpp:36`).

**Data updates are never skipped. Rendering is throttled but never queued.**
If a draw frame is slow, the next draw uses the latest data — no stale queue.

---

## 5) Overlay Safety

### Draw order position

Overlay draw is at `main.cpp:131`, which is AFTER `screenManager.update()` at `main.cpp:124`.
The normal render (`draw()`) happens inside `screenManager.update()` at `screen_manager.cpp:39`.

Call order:
```
main.cpp:124  screenManager.update()  → draw() happens here
main.cpp:125  RTMON_UI_END()
main.cpp:130  RTMON_OVERLAY_UPDATE()  ← after render
main.cpp:131  RTMON_OVERLAY_DRAW()    ← after render
```

### Does overlay ever mark a zone dirty?

No. `DebugOverlay::draw()` (`debug_overlay.cpp:53–130`) only calls `tft.fillRect()`,
`tft.drawRect()`, `tft.drawString()` — direct TFT operations.
It never writes to any `prev*` or `cur*` variable. It never sets `needsFullRedraw_`.
It never calls `RTMON_ZONE_REDRAW()`.

### Does overlay increase redraw count?

No. The overlay draw is OUTSIDE the `RTMON_FRAME_BEGIN/END` bracket
(`screen_manager.cpp:37–41`). It is not counted in frame timing.
The overlay's `getStats()` call (`debug_overlay.cpp:60`) is read-only.

**Overlay cannot affect normal rendering.**

---

## 6) Memory Stability

### No heap allocation during runtime

Search for `new`, `malloc`, `calloc`, `realloc`, `String` in `esp32/src/`:

```
grep results:
- "No String class" in comments only (drive_screen.cpp:16, runtime_monitor.h:12)
- "no new libraries" in comments only (debug_overlay.h:6)
- "new frame" in comment only (frame_limiter.h:27)
- "Draw new line" in comment only (car_renderer.cpp:109)
```

No actual `new`, `malloc`, `calloc`, `realloc`, or `String` usage found.
All buffers are fixed-size stack arrays:
- `char buf[ui::FMT_BUF_MED]` — `drive_screen.cpp:243`
- `char buf[128]` — `runtime_monitor.cpp:223`
- `char buf[48]` — `debug_overlay.cpp:78`
- `char warnBuf[96]` — `runtime_monitor.cpp:237`

All runtime monitor storage is static BSS:
- `ringBuf_[120]` — `runtime_monitor.cpp:24`
- `zoneCounts_[6]` — `runtime_monitor.cpp:45`

### No String usage in render path

Confirmed: only `snprintf()` with `char[]` is used throughout.

### No dynamic sprite creation

Search for `createSprite`, `pushSprite`, `deleteSprite`, `TFT_eSprite`:

```
grep results: No matches found.
```

**Zero dynamic memory usage in render path.**

---

## FINAL ANSWER

```
DISPLAY STABLE
```

**Justification:**
1. ✔ Deterministic render order — fixed sequence every frame
2. ✔ No flicker — `fillScreen` only on screen enter, partial redraw via dirty flags
3. ✔ All 6 zone dirty conditions verified with exact `if(...)` guards
4. ✔ Frame timing safe — FPS drops gracefully, no skip, no queue
5. ✔ Overlay isolated — draws after render, never marks zones dirty
6. ✔ Zero heap allocation — all stack/BSS, no String, no sprites
