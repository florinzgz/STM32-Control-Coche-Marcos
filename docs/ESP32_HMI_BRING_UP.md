# ESP32-S3 HMI — First Screen Bring-Up Guide

**Revision:** 1.0  
**Status:** ACTIVE  
**Date:** 2026-02-20  
**Scope:** Ordered steps to get the first live screen working on the ESP32-S3 HMI  
**Prerequisites:** STM32 firmware flashed and running (sends CAN 0x001 heartbeat every 100 ms)

---

## Step 1: Build and flash the ESP32 firmware

```bash
cd esp32
pio run -t upload
pio device monitor -b 115200
```

Expected Serial output on boot:

```
[HMI] Reset reason: PowerOn
[HMI] ESP32 HMI CAN bring-up booted
[PSRAM] Initialized — total: 8388608 bytes, free: ...
[TFT] Display initialized (480x320 landscape)
[CAN] Initialized at 500 kbps
[HMI] heartbeat
```

If `[CAN] Initialization FAILED` appears: check GPIO4 (TX) and GPIO5 (RX) wiring
to the TJA1051 transceiver. Verify 3.3 V power to the transceiver.

---

## Step 2: Observe the Boot screen

The TFT should immediately show:

```
     COCHE
     MARCOS
     HMI v1.0
     CAN: WAITING...
```

The "CAN: WAITING..." line turns green ("CAN: LINKED") as soon as the first
STM32 heartbeat (0x001) is received. This confirms the CAN link is up.

If the screen stays blank after `[TFT] Display initialized`:
- Check SPI wiring (MOSI=13, SCLK=14, CS=15, DC=16, RST=17, BL=42).
- Confirm `ST7796_DRIVER=1` is set in `platformio.ini`.
- Confirm `TFT_BACKLIGHT_ON=1` is active — without it the backlight stays off.

---

## Step 3: Verify CAN reception (minimum messages)

| Priority | CAN ID | Rate | What it enables |
|----------|--------|------|-----------------|
| **1** | 0x001 `HEARTBEAT_STM32` | 100 ms | Screen state machine, CAN-link indicator |
| **2** | 0x200 `STATUS_SPEED` | 100 ms | Speed display on Drive screen |
| **3** | 0x207 `STATUS_BATTERY` | 100 ms | Battery indicator on Drive screen |
| **4** | 0x203 `STATUS_SAFETY` | 100 ms | ABS/TCS flags, safe/error screen transitions |
| — | All others | — | Can be added later without changing architecture |

These four messages are enough to have a fully functional Drive screen.

To confirm reception, add a temporary Serial print inside `can_rx::poll()` for
each `case` label during initial debugging, then remove before production.

---

## Step 4: Trigger the Drive screen

The screen state machine transitions automatically via `HEARTBEAT_STM32` byte 1:

| `systemState` value | Screen shown |
|--------------------|--------------|
| 0 `BOOT` | Boot splash |
| 1 `STANDBY` | Standby (READY + temperatures) |
| 2 `ACTIVE` | **Drive screen (main dashboard)** |
| 3 `DEGRADED` | Drive screen (same, reduced limits) |
| 4 `SAFE` | Safe mode (amber banner) |
| 5 `ERROR` | Error screen (red) |
| 6 `LIMP_HOME` | Drive screen |

Once the STM32 moves to ACTIVE state (after ESP32 heartbeat is detected),
the Drive screen appears automatically — no action required on the ESP32.

---

## Step 5: What the Drive screen displays with minimum CAN data

| Zone | Y (px) | Data source | Shown value when data = 0 |
|------|--------|-------------|--------------------------|
| Top bar — battery | 0–40 | 0x207 `voltageRaw` | 0% (red battery) |
| Obstacle bar | 40–85 | `obstacle.distanceCm` | Gray bar (no sensor) |
| Car + wheels | 85–230 | 0x205 traction, 0x206 temp | All wheels gray, 0% |
| Steering gauge | 85–230 | 0x204 `angleRaw` | Needle at center |
| Speed | 230–270 | 0x200 avg of 4 wheels | `0.0` km/h |
| Pedal bar | 270–300 | average of 0x205 traction | 0% |
| Gear display | 300–320 | Drive screen default | D1 highlighted |

No crash or blank screen should occur with zero data — every widget is safe
with its default-initialized values.

---

## Step 6: Validate each element live

1. **Speed** — spin a wheel encoder by hand or in a test jig; verify the
   value changes on screen.
2. **Battery** — inject a 0x207 frame with a known voltage via a CAN adapter
   (e.g. `candump`/`cansend` on a Linux host with a PEAK or SLCAN adapter);
   verify the battery bar fills.
3. **System state transitions** — force the STM32 to SAFE state (trigger an
   overcurrent); verify the amber "SAFE MODE" screen appears on the ESP32.
4. **Serial heartbeat** — confirm `[HMI] heartbeat` prints every second even
   when the Drive screen is active (proves loop() is not blocking).

---

## What NOT to implement yet

The following features must not be coded until the basic screen validation
(Steps 1–6) is complete, to avoid architectural errors:

| Feature | Why to wait |
|---------|-------------|
| Touch input / CMD_THROTTLE / CMD_STEERING | Requires drive-by-wire safety audit on the STM32 side first. A wrong command sent from the ESP32 will move wheels. |
| Obstacle sensor (ultrasonic / VL53L8CX) | Has its own CAN contract section (0x208, 0x209). Add after the display is confirmed stable. |
| Service mode UI (0x301–0x303) | Service mode requires validated STM32 service mode firmware. Premature UI can mask STM32-side bugs. |
| OTA firmware update | Cannot be added without partitioning the 8 MB flash correctly (factory + OTA_0 + OTA_1). Must be planned before first flash. |
| LVGL or sprite-based rendering | Current direct-TFT rendering is stable and <5 ms/frame. Migrating to LVGL while the display is unvalidated adds risk. |
| Wi-Fi / Bluetooth | Shares radio with the display SPI clock domain on some boards. Validate display stability first. |

---

## Runtime Monitor (built-in, enabled by default)

The `RUNTIME_MONITOR` macro is defined to `1` by default in
`esp32/src/ui/runtime_monitor.h`. It logs performance data every 5 seconds:

```
[RT] fps=20 avg=3200 max=8100 min=900 redraw=12 full=0 can=45
```

- `fps` should be 20 (one frame per 50 ms).
- `avg` and `max` frame times should stay under 5000 µs (5 ms).
- `full=0` means no unintentional full-screen redraws (only on screen transitions).

To activate the debug overlay, hold the touch panel for 3 seconds. It displays
the same statistics graphically on top of the active screen.

To disable the monitor entirely (zero overhead), add to `platformio.ini`:

```ini
build_flags =
    ...
    -DRUNTIME_MONITOR=0
```

---

## Wiring Reference

| Signal | ESP32-S3 GPIO | Connected to |
|--------|--------------|-------------|
| CAN TX | 4 | TJA1051 TXD |
| CAN RX | 5 | TJA1051 RXD |
| TFT MOSI | 13 | ST7796 SDA |
| TFT SCLK | 14 | ST7796 SCL |
| TFT CS | 15 | ST7796 CS |
| TFT DC | 16 | ST7796 DC/RS |
| TFT RST | 17 | ST7796 RESET |
| TFT Backlight | 42 | ST7796 BL (via MOSFET or direct) |
| Touch CS | 21 | XPT2046 CS |

Full connection diagram: `docs/ESP32_S3_DISPLAY_Y_CAN_CONEXIONES.md`

---

## Reference Documents

| Document | Content |
|----------|---------|
| `docs/CAN_CONTRACT_FINAL.md` rev 1.3 | All CAN IDs, payloads, timing |
| `docs/HMI_STATE_MODEL.md` | Screen state machine specification |
| `docs/HMI_RENDERING_STRATEGY.md` | Rendering constraints and partial-redraw strategy |
| `docs/ESP32_FIRMWARE_DESIGN.md` | Framework and architecture decisions |
| `esp32/src/README.md` | Module list and Drive screen layout diagram |
