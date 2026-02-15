# Drive Screen Layout Technical Verification

**Date:** 2026-02-14
**Orientation:** Landscape (480×320)
**Source files verified:** `ui_common.h`, `drive_screen.cpp`, `car_renderer.cpp`,
`obstacle_sensor.cpp`, `gear_display.cpp`, `pedal_bar.cpp`, `battery_indicator.cpp`,
`mode_icons.cpp`, `steering_display.cpp`, `main.cpp`

---

## 1) Resolution Confirmation

| Question | Answer | Source |
|----------|--------|--------|
| `tft.width()` returns | **480** | `SCREEN_W = 480` (`ui_common.h:21`) |
| `tft.height()` returns | **320** | `SCREEN_H = 320` (`ui_common.h:22`) |
| `tft.setRotation(1)` used? | **YES** | `main.cpp:86` — `tft.setRotation(1);` |
| Long axis horizontal? | **YES** | 480 px horizontal, 320 px vertical |

---

## 2) Car Renderer — Body Geometry

| Property | Value | Source |
|----------|-------|--------|
| Car body width | **100 px** | `CAR_BODY_W = 100` (`ui_common.h:72`) |
| Car body X start | **190** | `CAR_BODY_X = 190` (`ui_common.h:70`) |
| Car body X end | **290** | 190 + 100 = 290 |
| Car body center X | **240** | 190 + 100/2 = 240 |
| Screen center X | **240** | 480 / 2 = 240 |
| Centered? | **YES** | 240 == 240 ✓ |

**Centering formula:** `CAR_BODY_X = (SCREEN_W - CAR_BODY_W) / 2 = (480 - 100) / 2 = 190` ✓

| Property | Value | Source |
|----------|-------|--------|
| Car body Y start | **105** | `CAR_BODY_Y = 105` (`ui_common.h:71`) |
| Car body height | **110 px** | `CAR_BODY_H = 110` (`ui_common.h:73`) |
| Car body Y end | **215** | 105 + 110 = 215 |
| Car area zone | **Y 85–230** | `CAR_AREA_Y = 85`, `CAR_AREA_H = 145` |
| Within zone? | **YES** | 105 ≥ 85, 215 ≤ 230 ✓ |

---

## 3) Wheels — Exact Coordinates

| Wheel | X | Y | X+W (right) | Y+H (bottom) | Center X | Center Y |
|-------|---|---|-------------|---------------|----------|----------|
| FL | 130 | 95 | 174 | 139 | 152 | 117 |
| FR | 306 | 95 | 350 | 139 | 328 | 117 |
| RL | 130 | 180 | 174 | 224 | 152 | 202 |
| RR | 306 | 180 | 350 | 224 | 328 | 202 |

Wheel dimensions: **44 × 44 px** (`WHEEL_W = 44`, `WHEEL_H = 44`, `ui_common.h:76-77`)

**Symmetry check:**
- FL center X (152) + FR center X (328) = 480. Average = **240** = screen center ✓
- RL center X (152) + RR center X (328) = 480. Average = **240** = screen center ✓
- FL center Y (117) == FR center Y (117) ✓
- RL center Y (202) == RR center Y (202) ✓
- Wheels are **symmetric** around X=240. ✓

**Boundary check (all within 0–479 X, 85–230 Y):**
- Left edge: 130 ≥ 0 ✓
- Right edge: 350 ≤ 479 ✓
- Top edge: 95 ≥ 85 ✓
- Bottom edge: 224 ≤ 230 ✓
- **No wheel exceeds the car area.** ✓

---

## 4) Steering

| Question | Answer | Source |
|----------|--------|--------|
| Text "Steering: -12.3°" exists? | **NO** | `steering_display.cpp` is a no-op. `draw()` and `drawStatic()` are empty. |
| Only circular gauge exists? | **YES** | Drawn by `CarRenderer::drawStatic()` and `drawSteering()` |
| Gauge center X | **410** | `STEER_CX = 410` (`ui_common.h:93`) |
| Gauge center Y | **160** | `STEER_CY = 160` (`ui_common.h:94`) |
| Gauge radius | **24 px** (outline at 25) | `STEER_RADIUS = 24` (`ui_common.h:95`) |
| Gauge left edge | **385** | 410 - 25 = 385 |
| Gauge right edge | **435** | 410 + 25 = 435 |
| Gauge top edge | **135** | 160 - 25 = 135 |
| Gauge bottom edge | **185** | 160 + 25 = 185 |
| Within car area (Y 85–230)? | **YES** | 135 ≥ 85, 185 ≤ 230 ✓ |
| Within screen (X ≤ 479)? | **YES** | 435 ≤ 479 ✓ |

---

## 5) Gear Zone — Exact Coordinates

Constants: `GEAR_START_X = 50`, `GEAR_SPACING = 80`, `GEAR_LABEL_W = 52`, `GEAR_LABEL_H = 18`, `GEAR_Y = 300`

| Gear | Index | Box X | Box X+W | Center X | Center Y | Label |
|------|-------|-------|---------|----------|----------|-------|
| P | 0 | 50 | 102 | 76 | 309 | "P" |
| R | 1 | 130 | 182 | 156 | 309 | "R" |
| N | 2 | 210 | 262 | 236 | 309 | "N" |
| D1 | 3 | 290 | 342 | 316 | 309 | "D1" |
| D2 | 4 | 370 | 422 | 396 | 309 | "D2" |

Formula: `x = GEAR_START_X + i * GEAR_SPACING` → `[50, 130, 210, 290, 370]`

| Property | Value |
|----------|-------|
| Spacing between boxes | **80 px** (28 px gap between boxes) |
| Highlight style | **Box highlight** — filled rect (green) + border (white) for active; border (gray) for inactive |
| Text size | **2** (16px tall GLCD) |
| Bottom edge (Y + H) | 300 + 18 = **318** |
| Within 300–320? | **YES** — 300 ≤ 300, 318 ≤ 320 ✓ |
| Right edge of D2 | **422** ≤ 479 ✓ |
| No duplicates? | **YES** — Labels: `"P", "R", "N", "D1", "D2"` (5 unique, `gear_display.h:39-41`) |

---

## 6) Obstacle Sensor Zone — Exact Coordinates

Zone: **Y 40–85** (`SENSOR_Y = 40`, `SENSOR_H = 45`)

| Element | X | Y | Width | Height | Bottom Y |
|---------|---|---|-------|--------|----------|
| "SENSOR FRONTAL" label | centered at 240 (TC_DATUM) | 42 | ~84px (14 chars × 6px) | 8 | 50 |
| Distance text (e.g. "1.25 m") | centered at 240 (TC_DATUM) | 54 | ~42px | 8 | 62 |
| Clear rect for text | 200 | 54 | 80 | 12 | 66 |
| Proximity bar outline | 140 | 68 | 200 | 12 | 80 |
| Bar fill area | 142 | 70 | 196 | 8 | 78 |

**Bar centering:** center = 140 + 200/2 = **240** = screen center ✓

| Question | Answer |
|----------|--------|
| All within Y 40–85? | **YES** — max bottom = 80 ≤ 85 ✓ |
| Health indicator exists? | **NO** — no health/status dot in the code |
| Visual zone color dot exists? | **NO** — bar fill changes color per `proximityColor()`, no separate dot |

Color logic in `proximityColor()` (`ui_common.h:158-163`):
- `distanceCm == 0` → gray (no reading)
- `distanceCm > 150` → green (> 1.5 m)
- `distanceCm > 50` → yellow (0.5–1.5 m)
- `distanceCm ≤ 50` → red (< 0.5 m)

---

## 7) Speed Display — Exact Coordinates

| Property | Value | Source |
|----------|-------|--------|
| Speed text Y | **232** | `SPEED_Y = 232` (`ui_common.h:56`) |
| Speed text X | **240** (centered, TC_DATUM) | `SCREEN_W / 2 = 240` (`drive_screen.cpp:221`) |
| Text size | **3** (24px tall GLCD) | `tft.setTextSize(3)` (`drive_screen.cpp:219`) |
| Clear rect | X=0, Y=232, W=480, H=24 | `drive_screen.cpp:215` |
| "km/h" label Y | **258** | `SPEED_Y + 26 = 258` (`drive_screen.cpp:134`) |
| "km/h" label X | **240** (centered, TC_DATUM) | `SCREEN_W / 2 = 240` |
| "km/h" text size | **1** (8px tall) | `drive_screen.cpp:132` |
| Perfectly centered horizontally? | **YES** — TC_DATUM at X=240 ✓ |
| Max bottom Y | **266** | 258 + 8 = 266 |
| Within speed zone (230–270)? | **YES** ✓ |

---

## 8) RPM

| Question | Answer |
|----------|--------|
| Any RPM text or variable in code? | **NO** — grep for "RPM" or "rpm" returns zero matches in all UI/screen files |
| Anything drawn on car hood? | **NO** — `CarRenderer::drawStatic()` draws only: body outline, windshield line, rear window line, axle lines, steering circle. No text on car body. |

---

## 9) 360° Label

| Property | Value | Source |
|----------|-------|--------|
| Text exists? | **YES** — `"360"` drawn above steering gauge | `drive_screen.cpp:141` |
| X coordinate | **410** (centered, TC_DATUM) | `STEER_CX = 410` |
| Y coordinate | **124** | `STEER_CY - STEER_RADIUS - 12 = 160 - 24 - 12 = 124` |
| Text color | **COL_CYAN** (0x07FF) | `drive_screen.cpp:138` |
| Text size | **1** (8px tall) | `drive_screen.cpp:139` |
| Within car area (85–230)? | **YES** — Y=124 ≥ 85 ✓ |

---

## 10) Layout Safety — Full Bounds Check

### No element draws outside 0 ≤ X ≤ 479, 0 ≤ Y ≤ 319

| Element | X min | X max | Y min | Y max | In bounds? |
|---------|-------|-------|-------|-------|------------|
| Mode icon 4x4 | 10 | 60 | 6 | 34 | ✓ |
| Mode icon 4x2 | 68 | 118 | 6 | 34 | ✓ |
| Mode icon 360 | 126 | 176 | 6 | 34 | ✓ |
| Battery | 405 | 470 | 6 | 34 | ✓ |
| Sensor label | ~198 | ~282 | 42 | 50 | ✓ |
| Sensor text | 200 | 280 | 54 | 66 | ✓ |
| Sensor bar | 140 | 340 | 68 | 80 | ✓ |
| Wheel FL | 130 | 174 | 95 | 139 | ✓ |
| Wheel FR | 306 | 350 | 95 | 139 | ✓ |
| Wheel RL | 130 | 174 | 180 | 224 | ✓ |
| Wheel RR | 306 | 350 | 180 | 224 | ✓ |
| Car body | 190 | 290 | 105 | 215 | ✓ |
| Steering gauge | 385 | 435 | 135 | 185 | ✓ |
| 360° label | ~401 | ~419 | 124 | 132 | ✓ |
| Speed text | ~170 | ~310 | 232 | 256 | ✓ |
| Speed clear rect | 0 | 480 | 232 | 256 | ✓ |
| "km/h" label | ~228 | ~252 | 258 | 266 | ✓ |
| Pedal label | 10 | ~40 | 272 | 280 | ✓ |
| Pedal bar | 10 | 390 | 284 | 300 | ✓ |
| Pedal text | 400 | 470 | 284 | 300 | ✓ |
| Gear P | 50 | 102 | 300 | 318 | ✓ |
| Gear R | 130 | 182 | 300 | 318 | ✓ |
| Gear N | 210 | 262 | 300 | 318 | ✓ |
| Gear D1 | 290 | 342 | 300 | 318 | ✓ |
| Gear D2 | 370 | 422 | 300 | 318 | ✓ |

**Result: ALL elements within bounds.** ✓

### Zone Overlap Check

| Zone boundary | Top zone bottom | Bottom zone top | Gap | Overlap? |
|---------------|----------------|-----------------|-----|----------|
| Top bar → Sensor | 40 | 40 | 0 | Adjacent, no overlap ✓ |
| Sensor → Car area | 85 | 85 | 0 | Adjacent, no overlap ✓ |
| Car area → Speed | 230 | 232 | 2 px | No overlap ✓ |
| Speed → Pedal | 270 | 272 | 2 px | No overlap ✓ |
| Pedal → Gear | 300 | 300 | 0 | Adjacent, no overlap ✓ |

**Result: NO overlapping zones.** ✓

---

## Summary

All 10 verification points **CONFIRMED** from actual source code coordinates:

1. ✅ Resolution: 480×320 landscape, `setRotation(1)`
2. ✅ Car body: 100px wide, centered at X=240
3. ✅ Wheels: symmetric, all within bounds
4. ✅ Steering: circular gauge only (no text), at (410, 160), R=24
5. ✅ Gears: P/R/N/D1/D2 at x=[50,130,210,290,370], box highlight, within 300–320
6. ✅ Obstacle sensor: label+text+bar all within Y 40–85, centered, no health indicator
7. ✅ Speed: centered at X=240, text size 3, within 230–270
8. ✅ No RPM anywhere, nothing on car hood
9. ✅ "360" label at (410, 124) above steering gauge
10. ✅ All elements within 0–479 X, 0–319 Y, no zone overlaps
