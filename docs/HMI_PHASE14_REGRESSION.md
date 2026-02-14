# HMI Phase 14 — Regression Document

**Revision:** 1.0
**Date:** 2026-02-13
**Scope:** Safety and performance regression analysis for Phase 14 HMI rendering

---

## 1. Stack Estimate

| Function | Max Stack Depth | Local Buffer Size |
|----------|----------------|-------------------|
| DriveScreen::update() | 2 levels | 0 bytes (direct field copy) |
| DriveScreen::draw() | 2 levels | 64 bytes (format buffers) |
| CarRenderer::draw() | 1 level | 32 bytes (format buffer) |
| PedalBar::draw() | 1 level | 16 bytes (format buffer) |
| GearDisplay::draw() | 1 level | 0 bytes (fixed strings) |
| BatteryIndicator::draw() | 1 level | 16 bytes (format buffer) |
| ModeIcons::draw() | 1 level | 0 bytes (fixed strings) |
| ScreenManager::update() | 3 levels | 0 bytes |

**Total worst-case stack per frame:** ~256 bytes
**ESP32 default task stack:** 8192 bytes
**Safety margin:** >97%

---

## 2. Heap Estimate

| Allocation | Size | When | Freed |
|------------|------|------|-------|
| TFT_eSPI object | ~200 bytes | setup() | Never (static) |
| Screen objects (5) | ~320 bytes each | Static | Never |
| UI widget objects | ~100 bytes each | Static member | Never |
| Format buffers | 0 (stack) | Per-frame | Auto (stack) |

**Total static heap:** ~2 KB
**Dynamic heap in loop():** 0 bytes
**ESP32 available heap:** ~300 KB
**Safety margin:** >99%

---

## 3. Frame Timing Estimate

| Phase | Time |
|-------|------|
| CAN polling | <0.1 ms |
| update() data copy | <0.1 ms |
| Dirty flag comparison | <0.05 ms |
| snprintf() formatting | <0.1 ms |
| Partial redraw (typical) | 1-3 ms |
| Full redraw (screen enter) | 8-12 ms |
| **Total per loop (no draw)** | **<0.5 ms** |
| **Total per loop (with draw)** | **<4 ms typical** |

Frame interval: 50 ms (20 FPS)
Duty cycle: <8% CPU for rendering

---

## 4. Watchdog Safety Confirmation

- ✅ No blocking calls in loop()
- ✅ No delay() calls in rendering path
- ✅ No while loops waiting for conditions
- ✅ CAN polling is non-blocking (readFrame timeout = 0)
- ✅ SPI TFT writes are hardware-buffered, non-blocking at driver level
- ✅ Frame limiter uses millis() comparison, never blocks
- ✅ Maximum single-frame draw time: <12 ms (full redraw worst case)
- ✅ ESP32 watchdog timeout: 5000 ms — margin >400×

---

## 5. No Recursion Verification

- ✅ Screen::draw() calls widget draw() methods — 1 level only
- ✅ Widget draw() methods call TFT primitives — 1 level only
- ✅ No function calls itself directly or indirectly
- ✅ No callback chains that could create cycles
- ✅ State machine uses switch/case, not recursive descent
- ✅ Screen transitions use pointer swap, not recursive construction

---

## 6. No Unbounded Loops

- ✅ CAN polling: while(readFrame()) with internal queue limit (5 frames max)
- ✅ Wheel rendering: for loop with NUM_WHEELS=4 (compile-time constant)
- ✅ Gear display: for loop with 5 fixed gear labels
- ✅ Pedal gradient: single fillRect, no iteration
- ✅ No user-input-dependent loop counts
- ✅ No linked list traversal

---

## 7. No Blocking Delays

- ✅ No delay() in loop()
- ✅ No vTaskDelay() calls
- ✅ No while(condition) busy-wait patterns
- ✅ No SPI transaction waits (TFT_eSPI handles internally)
- ✅ setup() uses delay(500) for serial only — acceptable for boot

---

## 8. No Dynamic Allocation in loop()

- ✅ No `new` or `malloc` calls in loop() or any function called from loop()
- ✅ No `String` class usage — all formatting uses stack char[] with snprintf()
- ✅ No `std::vector` or `std::string` — only `std::array` with fixed sizes
- ✅ No `TFT_eSprite` creation/destruction — direct TFT drawing only
- ✅ No temporary object construction that allocates heap

---

## 9. CAN Contract Compliance

- ✅ No new CAN IDs added
- ✅ No CAN ID definitions modified
- ✅ Heartbeat timing unchanged (100 ms)
- ✅ CAN contract rev 1.3 maintained
- ✅ STM32 firmware not modified

---

## 10. STM32 Impact Assessment

- ✅ No changes to Core/ directory
- ✅ No changes to Makefile
- ✅ No changes to .ioc files
- ✅ No changes to linker scripts
- ✅ ESP32 changes isolated to esp32/ directory

---

## Summary

| Metric | Value | Threshold | Status |
|--------|-------|-----------|--------|
| Stack per frame | ~256 B | <2048 B | ✅ PASS |
| Heap in loop() | 0 B | 0 B | ✅ PASS |
| Frame time (typical) | ~4 ms | <5 ms | ✅ PASS |
| Frame time (worst) | ~12 ms | <33 ms (30 FPS) | ✅ PASS |
| Recursion depth | 0 | 0 | ✅ PASS |
| Blocking calls | 0 | 0 | ✅ PASS |
| Dynamic allocs in loop | 0 | 0 | ✅ PASS |
| CAN IDs changed | 0 | 0 | ✅ PASS |
| STM32 files changed | 0 | 0 | ✅ PASS |
