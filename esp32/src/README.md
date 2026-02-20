# ESP32-S3 HMI — Source Directory

This directory contains C++ source files for the ESP32-S3 HMI firmware.

**Framework:** Arduino only — no ESP-IDF APIs.

## Module Overview

| File | Responsibility |
|------|----------------|
| `main.cpp` | Arduino `setup()` / `loop()` — CAN init, TFT init, heartbeat |
| `can_rx.cpp` / `can_rx.h` | CAN frame reception and decoding (all STM32→ESP32 IDs) |
| `vehicle_data.h` / `vehicle_data.cpp` | Passive telemetry data store (populated by `can_rx`, read by UI) |
| `screen_manager.cpp` / `screen_manager.h` | State machine — selects active screen from `SystemState` |
| `screens/` | One `.cpp`/`.h` pair per screen: boot, standby, drive, safe, error |
| `ui/` | Reusable display widgets: battery indicator, car renderer, gear display, pedal bar, obstacle sensor, steering display, mode icons, runtime monitor, debug overlay |

## Screen State Machine

```
BOOT  →  STANDBY  →  ACTIVE / DEGRADED / LIMP_HOME  →  SAFE / ERROR
```

The `ScreenManager` reads `heartbeat().systemState` (CAN 0x001 byte 1) each
loop and switches screens automatically.

## Drive Screen Layout (480×320)

```
┌─────────────────────────────────────────────────┐
│ [4x4] [4x2] [360°]              [BAT  85%]  Y: 0–40
├─────────────────────────────────────────────────┤
│         ←─── Obstacle: 2.35 m ────→             Y: 40–85
├──────────┬──────────────┬──────────┬────────────┤
│  FL●     │              │     FR●  │            │
│  85%     │    CAR BODY  │     72%  │  Steering  │ Y: 85–230
│  42°C    │              │     38°C │  gauge     │
│  RL●     │              │     RR●  │            │
│  90%     │              │     88%  │            │
│  45°C    │              │     41°C │            │
├──────────┴──────────────┴──────────┴────────────┤
│                   12.5 km/h                      Y: 230–270
├─────────────────────────────────────────────────┤
│ ████████████░░░░░░░░  65%                        Y: 270–300
├─────────────────────────────────────────────────┤
│  [P]   [R]   [N]  ►[D1]◄  [D2]                  Y: 300–320
└─────────────────────────────────────────────────┘
```

## Reference

See `docs/ESP32_FIRMWARE_DESIGN.md` and `docs/HMI_RENDERING_STRATEGY.md` for
architecture decisions. See `docs/CAN_CONTRACT_FINAL.md` rev 1.3 for the CAN
protocol specification. See `docs/ESP32_HMI_BRING_UP.md` for first-screen
validation steps.
