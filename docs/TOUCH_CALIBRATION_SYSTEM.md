# Touch Calibration System (ESP32-S3 HMI)

Persistent calibration of the XPT2046 resistive touch overlay used on the
ST7796 480×320 TFT.  Captured once via an interactive wizard, stored in NVS,
and applied on every subsequent boot — without ever modifying the STM32
firmware or the CAN protocol.

> **Scope**: ESP32-S3 only (PlatformIO env `esp32s3`).  The STM32G474RE,
> the CAN 500 kbps protocol, and all safety logic are completely untouched
> by this feature.

## 1. Overview

| Aspect            | Behaviour                                                                                                |
|-------------------|----------------------------------------------------------------------------------------------------------|
| First boot        | If no valid calibration exists in NVS the wizard launches automatically and the user **must** complete it. |
| Subsequent boots  | Stored calibration is loaded and applied via `tft.setTouch()`; the wizard does **not** re-launch.         |
| Recalibration     | Available on demand from the engineering menu (PIN `8989` → `TOUCH CALIBRATION`).                          |
| Reset             | `RESET TOUCH CAL` entry in the engineering menu erases NVS data and the `first_done` flag.                |
| Fallback          | Compile-time `TOUCH_CALIBRATION` macro (`include/User_Setup.h`) is used whenever stored data is missing or invalid. |
| CAN / STM32       | No effect.  No CAN frames are emitted by the wizard.  No STM32 source files are modified.                 |

## 2. Architecture

### 2.1 Files

| File                                                        | Role                                                                 |
|-------------------------------------------------------------|----------------------------------------------------------------------|
| `esp32/src/touch_calibration.{h,cpp}`                       | NVS persistence (load / save / clear / first-boot flag) + validation |
| `esp32/src/screens/touch_calibration_screen.{h,cpp}`        | Wizard UI: INTRO → COLLECT → CONFIRM → DONE / FAILED                 |
| `esp32/src/screen_manager.{h,cpp}`                          | New `touchCalActive_` flag and `requestTouchWizard(bool firstBoot)`   |
| `esp32/src/screens/engineering_screen.{h,cpp}`              | Two main-menu entries: `TOUCH CALIBRATION` and `RESET TOUCH CAL`     |
| `esp32/src/main.cpp`                                        | Loads NVS calibration in `setup()`; arms first-boot wizard in `renderTask` |
| `esp32/include/User_Setup.h`                                | Untouched — keeps providing the compile-time fallback                |

### 2.2 Touch flow (compatible with existing pipeline)

```
loop() in renderTask  (esp32/src/main.cpp)
   │
   ▼  tft.getTouch(&tx, &ty)         ← TFT_eSPI internally:
                                          · reads XPT2046 over SPI
                                          · filters / averages samples
                                          · applies calibration via the
                                            calData[] previously loaded
                                            via tft.setTouch()
                                          · returns screen-mapped (tx, ty)
   │
   ▼  touch::update(...)             ← debounce / long-press
   │
   ▼  screenManager.onTouch(x, y)    ← dispatched per active screen
```

The wizard does **not** intercept this pipeline.  It simply replaces the
`calData[]` consumed by `tft.setTouch()`.  All other screens (`drive`,
`boot`, `safety`, etc.) are unaffected.

## 3. NVS Storage Layout

Namespace: **`touch_cal`** (Arduino `Preferences` API).  Independent of
the main `hmi_cfg` namespace so that `config_store::factoryReset()`
**does not** erase touch calibration accidentally.

| Key          | Type   | Description                                                                            |
|--------------|--------|----------------------------------------------------------------------------------------|
| `data`       | blob   | Single-record blob (20 bytes) — see record layout below.                                |
| `first_done` | bool   | Latched true once the wizard has completed at least once. Suppresses auto-launch.       |

### 3.1 Record layout

```c
struct Record {              // 20 bytes total
    uint32_t magic;          // 0x54434C31  ("TCL1")
    uint16_t xMin;           // touch_x_min
    uint16_t xMax;           // touch_x_max
    uint16_t yMin;           // touch_y_min
    uint16_t yMax;           // touch_y_max
    uint8_t  rotation;       // must equal TFT_ROTATION (currently 1)
    uint8_t  _pad;           // explicit padding (zeroed)
    uint16_t _pad2;          // explicit padding (zeroed) — keeps crc32 4-byte aligned
    uint32_t crc32;          // CRC-32/ISO-HDLC over the 16 preceding bytes
};
```

The four logical XPT2046 raw-range keys requested in the design
(`touch_x_min`, `touch_x_max`, `touch_y_min`, `touch_y_max`) map directly
to the four `uint16_t` fields above.

### 3.2 Validation

Stored calibration is accepted only if **all** of the following hold:

1. `magic == 0x54434C31`
2. Recomputed CRC-32 matches `crc32`
3. All raw values ∈ `[0, 4095]`
4. `xMax > xMin + 1000` and `yMax > yMin + 1000` (`MIN_RANGE` guard against degenerate captures)
5. `rotation == TFT_ROTATION` (the calibration is rotation-specific)

If any check fails, calibration is treated as missing — the device falls
back to `TOUCH_CALIBRATION` from `User_Setup.h` and **does not**
auto-launch the wizard (anti-loop guarantee — see §5).

## 4. Boot Flow

```
setup() (esp32/src/main.cpp)
  ├─ config_store::init()
  ├─ touch_calibration::init()        ← opens "touch_cal" namespace
  ├─ tft.init()
  ├─ tft.setRotation(TFT_ROTATION)    ← 1 (landscape)
  └─ tft.setTouch(calData)            ← from NVS if loadValid(); else fallback

renderTask first iteration (Core 0)
  └─ if (!touch_calibration::firstBootDone())
        screenManager.requestTouchWizard(firstBoot=true);
```

The display always boots with a **working** touch mapping (the fallback
ensures the wizard itself can be tapped to start).  CAN polling on Core 1
is unaffected; the wizard runs entirely on Core 0.

## 5. First-Boot vs Reset Policy

| Action                                                                | `first_done` | Calibration data | Next boot launches wizard? |
|-----------------------------------------------------------------------|--------------|------------------|----------------------------|
| Device flashed, NVS empty                                             | `false`      | absent           | **Yes**                    |
| Wizard completed (`SAVE`)                                             | `true`       | valid            | No                         |
| Wizard cancelled from engineering menu                                | `true`       | unchanged        | No                         |
| Stored data corrupted (CRC mismatch / magic mismatch)                 | `true`       | invalid          | **No** — fallback only (anti-loop). User must run the wizard manually. |
| `RESET TOUCH CAL` engineering entry                                   | `false`      | absent           | **Yes** — re-arms first-boot wizard                                    |
| `config_store::factoryReset()` (main HMI factory restore)             | unchanged    | unchanged        | No (touch namespace is independent)                                    |

The `first_done` flag exists precisely to **prevent a corruption / power
loss loop** from re-launching the wizard on every boot.  Recalibration
after data corruption is always a deliberate user action via the
engineering menu.

## 6. Wizard UI

| State    | Description                                                                                       |
|----------|---------------------------------------------------------------------------------------------------|
| INTRO    | Instructions + `EMPEZAR` button.  `CANCELAR` is shown only when not in first-boot mode.            |
| COLLECT  | Four corner crosses with per-corner visual feedback (cross fades on accepted sample).  Implemented via `TFT_eSPI::calibrateTouch()`, which performs the rotation-aware fit and returns the standard 5-element calData array. |
| CONFIRM  | Displays the 4 captured raw values.  Buttons: `GUARDAR` / `REINTENTAR` (+ `CANCELAR` when not first-boot). |
| FAILED   | Shown when the captured range is degenerate.  Buttons: `REINTENTAR` (+ `CANCELAR` when not first-boot).    |
| DONE     | Transient — the new calibration is persisted to NVS, applied via `tft.setTouch()`, and the wizard exits. |

### 6.1 First-boot mode

When the wizard is launched automatically on first boot
(`requestTouchWizard(true)`), `CANCELAR` is **hidden** in INTRO / CONFIRM
/ FAILED so the user cannot leave without producing a valid calibration.
On `SAVE`, `first_done` is latched to `true` and the wizard never
auto-launches again.

### 6.2 Engineering-menu mode

When launched from the engineering menu
(`PIN 8989` → `TOUCH CALIBRATION`), `CANCELAR` is available so the
operator can leave without modifying the stored calibration.

## 7. How to Recalibrate (operator)

1. Turn the ignition key ON.
2. Long-press the battery icon on the drive screen → enter PIN `8989`.
3. Tap `TOUCH CALIBRATION` (cyan entry near the bottom).
4. Tap `EMPEZAR`.
5. Tap each of the four crosses precisely as they appear (top-left,
   top-right, bottom-left, bottom-right — order shown by `TFT_eSPI`).
6. On the CONFIRM screen, tap `GUARDAR` to persist.
7. The drive screen comes back with the new calibration applied.

## 8. How to Reset (operator)

- **Soft reset** (recommended for re-calibration during service):
  Engineering menu → `RESET TOUCH CAL`.  The current calibration is
  erased, `first_done` is cleared, and on the next reboot the wizard is
  launched automatically.

- **Hard reset** (full NVS wipe — should rarely be needed): erase the
  `touch_cal` partition entry via `nvs_flash_erase` or full-flash erase.
  Behaviour matches first-boot.

## 9. Fallback Behaviour

If the persistent record is missing or fails any validation in §3.2, the
firmware silently falls back to the compile-time
`TOUCH_CALIBRATION { 256, 3643, 182, 3672, 1 }` macro from
`esp32/include/User_Setup.h` and logs a single line:

```
[TOUCH_CAL] Stored calibration invalid — using fallback
[BOOT][INFO] Touch calibration: User_Setup.h fallback
```

The display remains fully usable; the user can recalibrate at their
discretion via the engineering menu.

## 10. Compatibility Matrix

| Concern                              | Status                                                     |
|--------------------------------------|------------------------------------------------------------|
| STM32G474RE firmware                 | ✅ Untouched (zero source changes in `Core/`)              |
| CAN 500 kbps protocol                | ✅ Untouched (no new IDs, no new frames, no timing change) |
| Existing touch-handler debounce      | ✅ Untouched (`touch_handler.{h,cpp}`)                     |
| Compile-time TOUCH_CALIBRATION       | ✅ Retained as fallback (no behavioural regression)        |
| Display rotation                     | ✅ Stored calibration is rotation-tagged and rejected on mismatch |
| Boot screen / drive screen / safety  | ✅ Untouched                                               |

## 11. Validation Checklist

| Test                                                              | Expected                                                              |
|-------------------------------------------------------------------|-----------------------------------------------------------------------|
| Erase NVS, boot device                                            | Wizard auto-launches; cannot exit without saving                       |
| Reboot after wizard completion                                    | No wizard; touch responds correctly                                   |
| `RESET TOUCH CAL` then reboot                                     | Wizard auto-launches again                                            |
| Inject CRC-corrupt blob (developer-only)                          | Fallback to `TOUCH_CALIBRATION`; wizard does **not** auto-launch       |
| Wizard active                                                     | CAN bus traffic continues at 500 kbps; no STM32 state change observed |
