# Engineering Screen

> **Note:** The canonical specification of the hidden engineering screen lives in
> [`ENGINEERING_MENU.md`](ENGINEERING_MENU.md). This file is preserved only as a
> stable entry point because several internal references (changelog, audit
> reports, training notes) historically pointed at `ENGINEERING_SCREEN.md`.
>
> **All content has moved to [`ENGINEERING_MENU.md`](ENGINEERING_MENU.md).**

## Quick map of related documents

| Topic | Document |
|-------|----------|
| Menu layout, access code, submenu list | [`ENGINEERING_MENU.md`](ENGINEERING_MENU.md) |
| Pedal calibration procedure & CAN protocol | [`CALIBRATION.md`](CALIBRATION.md) |
| Full CAN ID / DLC / payload reference | [`CAN_PROTOCOL.md`](CAN_PROTOCOL.md) |
| Persistent flash layout (page 124 = pedal cal) | [`HARDWARE_AND_SENSOR_MAP.md`](HARDWARE_AND_SENSOR_MAP.md) §1.6 |
| Touch calibration wizard | [`TOUCH_CALIBRATION_SYSTEM.md`](TOUCH_CALIBRATION_SYSTEM.md) |

## Implementation

- ESP32-S3 source: `esp32/src/screens/engineering_screen.cpp` / `.h`
- STM32 service handler: `Core/Src/can_handler.c` (`SERVICE_ACTION_PEDAL_CAL = 0xF5`)
- Flash store: `Core/Src/pedal_cal_store.c` (page 124, `0x0807C000`)
