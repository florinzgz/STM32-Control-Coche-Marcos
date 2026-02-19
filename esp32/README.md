# ESP32-S3 HMI Firmware

Human-Machine Interface firmware for the STM32-Control-Coche-Marcos project.

## Overview

This subproject contains the **ESP32-S3** HMI firmware that communicates with
the STM32G474RE safety controller over CAN bus. The ESP32 acts exclusively as
an **intent sender** — the STM32 remains the sole safety authority and decides
what is allowed.

## Technology Stack

| Item | Choice |
|------|--------|
| Language | C++ (C++17) |
| Framework | **Arduino** (no ESP-IDF) |
| Build system | PlatformIO |
| Board | ESP32-S3-DevKitC-1 |
| CAN bitrate | 500 kbps (classic CAN 2.0A) |

> **ESP-IDF is NOT used.** All code relies on the Arduino core and
> Arduino-compatible libraries only.

## Project Structure

```
esp32/
├── platformio.ini          # PlatformIO build configuration
├── include/
│   └── can_ids.h           # CAN IDs and protocol constants (from CAN contract)
├── src/
│   └── main.cpp            # Arduino entry point (setup / loop)
└── README.md               # This file
```

## CAN Protocol

All CAN IDs and payload definitions are mirrored from
[`docs/CAN_CONTRACT_FINAL.md`](../docs/CAN_CONTRACT_FINAL.md) rev 1.0 and
defined as `constexpr` values in `include/can_ids.h`.

The STM32 firmware owns the protocol. The ESP32 must never assume a sent
command was executed — it must verify state via status messages.

## Building

```bash
cd esp32
pio run
```

## Uploading Firmware

```bash
cd esp32
pio run -t upload
```

> **Tip:** PlatformIO auto-detects the serial port.  If it picks the wrong
> one, set `upload_port` in `platformio.ini` (see comments there).

## Troubleshooting Upload Errors

### "Could not open COMx" / PermissionError on Windows

```
A fatal error occurred: Could not open COM3, the port is busy or doesn't exist.
(could not open port 'COM3': PermissionError(13, 'Acceso denegado.', None, 5))
```

**Cause:** The ESP32-S3 DevKitC uses the native USB port for both the serial
monitor and firmware upload.  If another application (serial monitor, PuTTY,
Arduino IDE, etc.) is holding the port open, the upload tool cannot access it.

**Solutions (try in order):**

1. **Close every serial monitor / terminal** connected to that port
   (including the PlatformIO Serial Monitor in VS Code).
2. **Enter download mode manually:**
   hold **BOOT**, press **RESET**, release **BOOT**, then run `pio run -t upload`.
3. **Verify the port:**
   - Windows → *Device Manager → Ports (COM & LPT)*
   - Linux → `ls /dev/ttyACM* /dev/ttyUSB*`
   - macOS → `ls /dev/cu.usbmodem*`

   Set the correct port in `platformio.ini`:
   ```ini
   upload_port = COM5          ; ← your actual port
   ```
4. **Install / update USB drivers:**
   - Windows may need the
     [ESP32-S3 USB driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/establish-serial-connection.html).
   - Linux: add your user to the `dialout` group:
     ```bash
     sudo usermod -aG dialout $USER   # then log out / log in
     ```
5. **Try a different USB cable or port** — some cables are charge-only.

## Authority Model

- **STM32G474RE** — safety authority, controls actuators, enforces fail-safe.
- **ESP32-S3** — HMI only, sends operator intent, displays status.
