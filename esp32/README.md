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

## Authority Model

- **STM32G474RE** — safety authority, controls actuators, enforces fail-safe.
- **ESP32-S3** — HMI only, sends operator intent, displays status.
