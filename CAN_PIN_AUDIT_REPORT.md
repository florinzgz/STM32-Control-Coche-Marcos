# CAN/TWAI Hardware Pin Audit Report

**Date:** 2026-02-19  
**Project:** STM32-Control-Coche-Marcos (Dual-MCU: STM32G474RE + ESP32-S3)  
**Scope:** CAN/TWAI pin usage extraction from source code and configuration

---

## STM32 FDCAN Configuration

**FDCAN instance:** FDCAN1

**TX:** PB9 (GPIO_PIN_9, GPIOB)  
**RX:** PB8 (GPIO_PIN_8, GPIOB)  
**AF:** GPIO_AF9_FDCAN1

**Source files:**
- `/Core/Src/stm32g4xx_hal_msp.c` lines 9-27 (HAL_FDCAN_MspInit)
- `/Core/Inc/main.h` lines 79-81 (pin definitions)
- `/README.md` (pin table documentation)

**Bitrate config:**
```c
Location: /Core/Src/main.c - MX_FDCAN1_Init()
FrameFormat:         FDCAN_FRAME_CLASSIC
Mode:                FDCAN_MODE_NORMAL
ClockDivider:        FDCAN_CLOCK_DIV1
NominalPrescaler:    17
NominalSyncJumpWidth: 1
NominalTimeSeg1:     14
NominalTimeSeg2:     5
AutoRetransmission:  ENABLE
TransmitPause:       DISABLE
ProtocolException:   DISABLE
```

**Calculated bitrate:**
- Nominal bit time = (1 + NominalTimeSeg1 + NominalTimeSeg2) = 1 + 14 + 5 = 20 time quanta
- CAN clock = 170 MHz / ClockDivider(1) = 170 MHz
- Bitrate = 170 MHz / (NominalPrescaler × bit_time) = 170 MHz / (17 × 20) = 500 kbps

---

## ESP32 TWAI Configuration

**TX:** GPIO 4  
**RX:** GPIO 5

**Driver:** ESP32-TWAI-CAN library (v1.0.1)

**Source files:**
- `/esp32/src/main.cpp` lines 23-24 (pin constants)
- `/esp32/platformio.ini` line 8 (documentation comment)

**Bitrate:** 500 kbps

**Configuration code:**
```cpp
Location: /esp32/src/main.cpp - setup()
static constexpr int CAN_TX_PIN = 4;
static constexpr int CAN_RX_PIN = 5;

ESP32Can.setPins(CAN_TX_PIN, CAN_RX_PIN);
ESP32Can.setRxQueueSize(5);
ESP32Can.setTxQueueSize(5);
ESP32Can.begin(ESP32Can.convertSpeed(500));  // 500 kbps
```

**Mode:** Normal mode (default for ESP32Can.begin(), no listen-only or no-ack flags set)

---

## Transceiver Expectations

**STM32 side:**
- No standby pin configured
- No enable pin configured
- No silent mode pin configured
- Transceiver not explicitly specified in code (assumed passive connection)

**ESP32 side:**
- Transceiver: TJA1051 (documented in platformio.ini line 8)
- No standby pin configured
- No enable pin configured
- No silent mode pin configured
- Standard TX/RX connection only

---

## Conflicts

None detected.

**Pin assignment consistency:**
- STM32: PB9 (TX), PB8 (RX) — explicitly configured in HAL MSP
- ESP32: GPIO4 (TX), GPIO5 (RX) — explicitly configured in main.cpp

**Bitrate consistency:**
- Both sides configured for 500 kbps
- STM32: 170 MHz / (17 × 20) = 500 kbps
- ESP32: convertSpeed(500) = 500 kbps

**No duplicate or conflicting definitions found.**

---

## Confidence Level

**HIGH**

**Rationale:**
- All pin assignments are literal values from source code (not inferred)
- STM32 pins traced through HAL_FDCAN_MspInit() in stm32g4xx_hal_msp.c
- STM32 bitrate parameters found in MX_FDCAN1_Init() in main.c
- ESP32 pins found as constexpr literals in main.cpp
- ESP32 bitrate found as literal argument to convertSpeed()
- Pin definitions cross-referenced in main.h and README.md
- No runtime configuration detected
- No conditional compilation affecting these values
- Documentation in platformio.ini and README.md confirms hardware expectations

**Active configuration:**
- Only one FDCAN peripheral initialized (FDCAN1)
- Only one CAN initialization path in ESP32 (setup() function)
- No alternative pin configurations found in codebase
