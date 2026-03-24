# Connector Pinouts — STM32 + ESP32-S3 Vehicle Control System

> **Source of truth:** `Core/Inc/main.h`, `esp32/include/User_Setup.h`, `esp32/src/*.h`  
> **Generated:** 2026-03-19 | **Branch:** `analysis/wiring-docs`

---

## Table of Contents

1. [STM32G474RE Nucleo-64 Pinout](#1-stm32g474re-nucleo-64-pinout)
2. [BTS7960 Motor Driver Connections](#2-bts7960-motor-driver-connections)
3. [CAN Bus Connector](#3-can-bus-connector)
4. [I2C Bus Connector](#4-i2c-bus-connector)
5. [Wheel Speed Sensor Connectors](#5-wheel-speed-sensor-connectors)
6. [Steering Encoder Connector](#6-steering-encoder-connector)
7. [Relay Connections](#7-relay-connections)
8. [ESP32-S3-DevKitC-1 Pinout](#8-esp32-s3-devkitc-1-pinout)
9. [TFT Display Connector](#9-tft-display-connector)
10. [DFPlayer Mini Connector](#10-dfplayer-mini-connector)
11. [MCP23017 Shifter Connector](#11-mcp23017-shifter-connector)
12. [LED Strip Connectors](#12-led-strip-connectors)
13. [Missing Information](#13-missing-information)

---

## 1. STM32G474RE Nucleo-64 Pinout

### Port A Pins

| Pin | GPIO | Alternate Function | Signal | Direction | Notes |
|-----|------|--------------------|--------|-----------|-------|
| PA0 | GPIOA_0 | EXTI0 | WHEEL_FL | Input | Wheel speed sensor FL; rising-edge interrupt |
| PA1 | GPIOA_1 | EXTI1 | WHEEL_FR | Input | Wheel speed sensor FR; rising-edge interrupt |
| PA2 | GPIOA_2 | EXTI2 | WHEEL_RL | Input | Wheel speed sensor RL; rising-edge interrupt |
| PA3 | GPIOA_3 | ADC1_IN4 | PEDAL | Analog In | Hall pedal via 10k/6.8k divider; 0–2.0V range |
| PA5 | GPIOA_5 | — | LED_LD2 | Output | Nucleo-64 user LED (active HIGH) |
| PA6 | GPIOA_6 | TIM3_CH1 | RPWM_STEER | AF Output | Steering motor RPWM (20 kHz center-aligned) |
| PA7 | GPIOA_7 | TIM3_CH2 | LPWM_STEER | AF Output | Steering motor LPWM (20 kHz center-aligned) |
| PA8 | GPIOA_8 | TIM1_CH1 | RPWM_FL | AF Output | Front-left motor RPWM (20 kHz) |
| PA9 | GPIOA_9 | TIM1_CH2 | LPWM_FL | AF Output | Front-left motor LPWM |
| PA10 | GPIOA_10 | TIM1_CH3 | RPWM_FR | AF Output | Front-right motor RPWM |
| PA11 | GPIOA_11 | TIM1_CH4 | LPWM_FR | AF Output | Front-right motor LPWM |
| PA15 | GPIOA_15 | TIM2_CH1 | ENC_A | AF Input | Steering encoder channel A (quadrature) |

### Port B Pins

| Pin | GPIO | Alternate Function | Signal | Direction | Notes |
|-----|------|--------------------|--------|-----------|-------|
| PB0 | GPIOB_0 | — | ONEWIRE | Bidirectional | DS18B20 temperature bus (4.7k pull-up to 3.3V) |
| PB3 | GPIOB_3 | TIM2_CH2 | ENC_B | AF Input | Steering encoder channel B (quadrature) |
| PB4 | GPIOB_4 | EXTI4 | ENC_Z | Input | Steering encoder index pulse (rising edge) |
| PB5 | GPIOB_5 | EXTI5 | STEER_CENTER | Input | LJ12A3 inductive center sensor |
| PB6 | GPIOB_6 | I2C1_SCL | I2C_SCL | AF Open-Drain | I2C clock (400 kHz); 4.7k pull-up to 3.3V |
| PB7 | GPIOB_7 | I2C1_SDA | I2C_SDA | AF Open-Drain | I2C data; 4.7k pull-up to 3.3V |
| PB8 | GPIOB_8 | FDCAN1_RX (AF9) | CAN_RX | AF Input | FDCAN receive from transceiver RXD |
| PB9 | GPIOB_9 | FDCAN1_TX (AF9) | CAN_TX | AF Output | FDCAN transmit to transceiver TXD |
| PB10 | GPIOB_10 | — | RELAY_LED_FRONT | Output | Front WS2812B LED strip power relay |
| PB11 | GPIOB_11 | — | RELAY_LED_REAR | Output | Rear WS2812B LED strip power relay |
| PB15 | GPIOB_15 | EXTI15 | WHEEL_RR | Input | Wheel speed sensor RR; rising-edge interrupt |

### Port C Pins

| Pin | GPIO | Alternate Function | Signal | Direction | Notes |
|-----|------|--------------------|--------|-----------|-------|
| PC0 | GPIOC_0 | — | (FREED) | — | Formerly DIR_FL; leave unconnected |
| PC1 | GPIOC_1 | — | (FREED) | — | Formerly DIR_FR; leave unconnected |
| PC2 | GPIOC_2 | — | (FREED) | — | Formerly DIR_RL; leave unconnected |
| PC3 | GPIOC_3 | — | (FREED) | — | Formerly DIR_RR; leave unconnected |
| PC4 | GPIOC_4 | — | (FREED) | — | Formerly DIR_STEER; leave unconnected |
| PC5 | GPIOC_5 | — | EN_FL | Output | Front-left BTS7960 enable (active HIGH) |
| PC6 | GPIOC_6 | TIM8_CH1 | RPWM_RL | AF Output | Rear-left motor RPWM (20 kHz) |
| PC7 | GPIOC_7 | TIM8_CH2 | LPWM_RL | AF Output | Rear-left motor LPWM |
| PC8 | GPIOC_8 | TIM8_CH3 | RPWM_RR | AF Output | Rear-right motor RPWM |
| PC9 | GPIOC_9 | TIM8_CH4 | LPWM_RR | AF Output | Rear-right motor LPWM |
| PC10 | GPIOC_10 | — | RELAY_MAIN | Output | Main power relay (active HIGH) |
| PC11 | GPIOC_11 | — | RELAY_TRAC | Output | Traction power relay (active HIGH) |
| PC12 | GPIOC_12 | — | RELAY_DIR | Output | Steering power relay (active HIGH) |
| PC13 | GPIOC_13 | — | EN_RR | Output | Rear-right BTS7960 enable (active HIGH) |

---

## 2. BTS7960 Motor Driver Connections

Each BTS7960 module has these pins:

```
BTS7960 Module Pinout:
┌──────────────────────────┐
│  B+   │ Motor power in (from relay/fuse)
│  B-   │ GND (power ground)
│  M+   │ Motor terminal 1
│  M-   │ Motor terminal 2
│  VCC  │ Logic supply 3.3V–5V
│  GND  │ Logic ground
│  R_EN │ Right-side enable (active HIGH)
│  L_EN │ Left-side enable (active HIGH)
│  RPWM │ Right-side PWM input
│  LPWM │ Left-side PWM input
│  R_IS │ Right-side current sense (unused)
│  L_IS │ Left-side current sense (unused)
└──────────────────────────┘
```

### Motor → BTS7960 → STM32 Mapping

| Motor | BTS7960 | RPWM Pin | LPWM Pin | R_EN | L_EN | Timer |
|-------|---------|----------|----------|------|------|-------|
| Front-Left (FL) | DRV1 | PA8 (TIM1_CH1) | PA9 (TIM1_CH2) | PC5 (GPIO) | PC5 (GPIO) | TIM1 |
| Front-Right (FR) | DRV2 | PA10 (TIM1_CH3) | PA11 (TIM1_CH4) | **Tie to 3.3V** | **Tie to 3.3V** | TIM1 |
| Rear-Left (RL) | DRV3 | PC6 (TIM8_CH1) | PC7 (TIM8_CH2) | **Tie to 3.3V** | **Tie to 3.3V** | TIM8 |
| Rear-Right (RR) | DRV4 | PC8 (TIM8_CH3) | PC9 (TIM8_CH4) | PC13 (GPIO) | PC13 (GPIO) | TIM8 |
| Steering | DRV5 | PA6 (TIM3_CH1) | PA7 (TIM3_CH2) | **Tie to 3.3V** | **Tie to 3.3V** | TIM3 |

> **⚠ CRITICAL:** FR, RL, and Steering BTS7960 modules have R_EN and L_EN **tied directly to 3.3V** (not GPIO-controlled). Only FL (PC5) and RR (PC13) have software-controlled enables.

### Power Path per Motor

```
Battery 24V ──► F1 (60A) ──► RELAY_MAIN (PC10)
                                   │
                     ┌─────────────┼─────────────┐
                     ▼             ▼             ▼
              RELAY_TRAC(PC11) RELAY_TRAC     RELAY_DIR(PC12)
                     │             │             │
              ┌──────┤      ┌──────┤             │
              ▼      ▼      ▼      ▼             ▼
           INA226  INA226 INA226 INA226       INA226
          (1.5mΩ) (1.5mΩ)(1.5mΩ)(1.5mΩ)    (1.5mΩ)
              │      │      │      │             │
              ▼      ▼      ▼      ▼             ▼
           BTS FL  BTS FR BTS RL BTS RR       BTS STEER
              │      │      │      │             │
              ▼      ▼      ▼      ▼             ▼
           Motor   Motor  Motor  Motor        Motor
            FL      FR     RL     RR          STEER

Battery INA226 (CH4, 0.75 mΩ) placed BEFORE RELAY_MAIN to always read battery voltage.

Capacitors at each BTS7960 (mandatory):
  B+──[470µF/35V]──GND  (bulk, traction motors; use 470µF/25V for steering 12V bus)
  B+──[100nF/50V]──GND  (bypass, HF filter, as close to IC as possible)
  VCC──[100nF/50V]──GND_logic  (logic decoupling, next to VCC pin)
  M+──[100nF/50V]──M−  (motor terminal snubber, mount at motor body)
```

> **INA226 shunt values (from `Core/Inc/project_config.h`):**
> - Motor channels (CH0–CH3, CH5): **1.5 mΩ** (50A/75mV rating)
> - Battery channel (CH4): **0.75 mΩ** (100A/75mV rating)

---

## 3. CAN Bus Connector

### Recommended Connector: JST-XH 4-pin or DB9

| Pin | Signal | Wire Color | Notes |
|-----|--------|------------|-------|
| 1 | CANH | Yellow | Twisted pair with CANL |
| 2 | CANL | Green | Twisted pair with CANH |
| 3 | GND | Black | Signal ground reference |
| 4 | Shield | Bare/Drain | Shield grounded at ONE end only |

### CAN Bus Topology

```
  STM32 Node                              ESP32 Node
┌─────────────┐                        ┌─────────────┐
│  PB9 (TX)───┤──► TXD  SN65HVD230    │  GPIO4(TX)──┤──► TXD  TJA1051
│  PB8 (RX)◄──┤──◄ RXD     │          │  GPIO5(RX)◄─┤──◄ RXD    │
│             │         CANH├──────────┤         CANH│
│             │         CANL├──────────┤         CANL│
│             │   ┌─120Ω──┤          │   ┌─120Ω──┤
│             │   └───────┘          │   └───────┘
│             │   TVS(PESD2CAN)       │   TVS(PESD2CAN)
└─────────────┘                        └─────────────┘
```

> **Cable:** Shielded twisted pair, 22 AWG, ≤ 5 m length for 500 kbps. Shield drain wire grounded at STM32 end only.

---

## 4. I2C Bus Connector

### I2C Topology via TCA9548A Multiplexer

```
STM32 I2C1 (PB6/PB7, 400kHz)
       │
   ┌───┴───┐
   │TCA9548A│ (Addr: 0x70)
   │  CH0   │──► INA226 #0 (Addr: 0x40) — Motor FL current
   │  CH1   │──► INA226 #1 (Addr: 0x40) — Motor FR current
   │  CH2   │──► INA226 #2 (Addr: 0x40) — Motor RL current
   │  CH3   │──► INA226 #3 (Addr: 0x40) — Motor RR current
   │  CH4   │──► INA226 #4 (Addr: 0x40) — Battery 24V current
   │  CH5   │──► INA226 #5 (Addr: 0x40) — Steering current
   │  CH6   │    (Available)
   │  CH7   │    (Available)
   └────────┘
```

### I2C Connector (JST-XH 4-pin per sensor)

| Pin | Signal | Wire Color | Notes |
|-----|--------|------------|-------|
| 1 | VCC | Red | 3.3V supply |
| 2 | GND | Black | Ground |
| 3 | SCL | Yellow | Clock (4.7k pull-up to 3.3V at bus master) |
| 4 | SDA | Blue | Data (4.7k pull-up to 3.3V at bus master) |

---

## 5. Wheel Speed Sensor Connectors

### Sensor: LJ12A3-4-Z/BX (NPN NO, inductive proximity)

| Sensor | MCU Pin | EXTI Line | Connector (JST-XH 3-pin) |
|--------|---------|-----------|--------------------------|
| Wheel FL | PA0 | EXTI0 | Brown=VCC(6-36V), Blue=GND, Black=Signal→PA0 |
| Wheel FR | PA1 | EXTI1 | Brown=VCC(6-36V), Blue=GND, Black=Signal→PA1 |
| Wheel RL | PA2 | EXTI2 | Brown=VCC(6-36V), Blue=GND, Black=Signal→PA2 |
| Wheel RR | PB15 | EXTI15 | Brown=VCC(6-36V), Blue=GND, Black=Signal→PB15 |

> **⚠ IMPORTANT:** LJ12A3 sensors typically output 6-36V signal levels. If powered at voltages above 3.3V, an optocoupler (PC817) **must** be used between sensor output and the STM32 GPIO to prevent damage. The PC817's 4-6 µs propagation delay is acceptable for wheel speed sensing (max ~1 kHz signal).

---

## 6. Steering Encoder Connector

### Encoder: E6B2-CWZ6C (1200 PPR, Omron)

| Wire Color (Omron standard) | Signal | MCU Pin | Notes |
|-----|--------|---------|-------|
| Brown | VCC | 5V supply | Encoder requires 5V |
| Blue | GND | Ground | — |
| Black | Phase A | PA15 (TIM2_CH1) | Quadrature channel A |
| White | Phase B | PB3 (TIM2_CH2) | Quadrature channel B |
| Orange | Phase Z | PB4 (EXTI4) | Index pulse (1 per revolution) |

> **⚠ LEVEL SHIFTING:** Encoder outputs 5V logic. Use 6N137 high-speed optocouplers (10 Mbps, 75-120 ns delay) for galvanic isolation and level shifting to 3.3V. The TIM2 encoder filter threshold is 282 ns — PC817 (4-6 µs) is **too slow** for encoder signals at high RPM.

---

## 7. Relay Connections

### Power Relay Wiring (PC10, PC11, PC12)

```
MCU Pin (3.3V GPIO) ──► R_gate (1kΩ) ──► 2N7000 Gate
                                          │
                                     R_pd (10kΩ)
                                          │
                                         GND
                                         
2N7000 Drain ──► Relay Coil (−)
                 Relay Coil (+) ──► 12V
                 │         │
                 └─ D_fw ──┘  (1N4007 flyback diode, cathode to +12V)

2N7000 Source ──► GND
```

| Relay | MCU Pin | MOSFET | Flyback Diode | Load |
|-------|---------|--------|---------------|------|
| RELAY_MAIN | PC10 | Q1 (2N7000) | D1 (1N4007) | Master 24V power |
| RELAY_TRAC | PC11 | Q2 (2N7000) | D2 (1N4007) | Traction motor power |
| RELAY_DIR | PC12 | Q3 (2N7000) | D3 (1N4007) | Steering motor power |
| RELAY_LED_F | PB10 | Q4 (2N7000) | D4 (1N4007) | Front LED 5V |
| RELAY_LED_R | PB11 | Q5 (2N7000) | D5 (1N4007) | Rear LED 5V |

---

## 8. ESP32-S3-DevKitC-1 Pinout

### GPIO Usage Map

| GPIO | Function | Module | Direction | Notes |
|------|----------|--------|-----------|-------|
| 4 | CAN_TX | CAN (TWAI) | Output | → TJA1051 TXD |
| 5 | CAN_RX | CAN (TWAI) | Input | ← TJA1051 RXD |
| 8 | I2C_SDA | Shifter (MCP23017) | Bidirectional | 4.7k pull-up |
| 9 | I2C_SCL | Shifter (MCP23017) | Bidirectional | 4.7k pull-up |
| 10 | TFT_CS | Display (SPI) | Output | ST7796 chip select |
| 11 | AUDIO_RELAY | Audio relay | Output | Active LOW relay control |
| 12 | TFT_MISO | Display (SPI) | Input | Shared with touch T_DO |
| 13 | TFT_MOSI | Display (SPI) | Output | SPI data out |
| 14 | TFT_SCLK | Display (SPI) | Output | SPI clock |
| 15 | TRACTION_SW | Traction switch | Input | Pull-up; LOW=4WD, HIGH=2WD |
| 21 | TOUCH_CS | Touch (XPT2046) | Output | Touch controller select |
| 38 | TFT_RST | Display | Output | Display reset |
| 39 | TFT_DC | Display | Output | Data/command select |
| 40 | IGNITION_SENSE | Power manager | Input | Key position (HIGH=ON) |
| 41 | POWER_HOLD | Power manager | Output | Keep power alive (HIGH) |
| 42 | TFT_BL | Display backlight | Output | Backlight enable (HIGH) |
| 43 | DFPLAYER_TX | Audio (UART2) | Output | ESP32 TX → DFPlayer RX |
| 44 | DFPLAYER_RX | Audio (UART2) | Input | DFPlayer TX → ESP32 RX |
| 47 | LED_FRONT | LED strip | Output | WS2812B front data (28 LEDs) |
| 48 | LED_REAR | LED strip | Output | WS2812B rear data (16 LEDs) |

### Reserved/Forbidden GPIOs on ESP32-S3-WROOM-1-N16R8

| GPIO Range | Reason | Risk if Used |
|------------|--------|--------------|
| 22–25 | Do not exist in ESP32-S3 | N/A |
| 26–32 | QSPI Flash interface (internal) | Boot corruption |
| 33–37 | Octal PSRAM interface (N16R8) | Memory corruption / crash |
| 0 | Boot strapping pin | Boot mode change |
| 3 | eFuse download strapping | eFuse corruption |
| 45 | VDD_SPI voltage selection | Flash voltage change |
| 46 | Boot log verbosity | May affect boot |
| 19/20 | USB D−/D+ | USB function loss |

---

## 9. TFT Display Connector

### ST7796 480×320 TFT + XPT2046 Touch (SPI, HSPI/SPI3)

| Display Pin | ESP32 GPIO | Signal | Notes |
|-------------|------------|--------|-------|
| VCC | 3.3V | Power | 3.3V supply (NOT 5V) |
| GND | GND | Ground | — |
| CS | GPIO 10 | TFT_CS | Chip select (active LOW) |
| RESET | GPIO 38 | TFT_RST | Display reset (active LOW) |
| DC/RS | GPIO 39 | TFT_DC | Data/Command select |
| SDI/MOSI | GPIO 13 | TFT_MOSI | SPI data in |
| SCK | GPIO 14 | TFT_SCLK | SPI clock (40 MHz write) |
| LED | GPIO 42 | TFT_BL | Backlight (HIGH = ON) |
| SDO/MISO | GPIO 12 | TFT_MISO | SPI data out (20 MHz read) |
| T_CLK | GPIO 14 | (shared) | Touch uses same SPI clock |
| T_CS | GPIO 21 | TOUCH_CS | Touch chip select |
| T_DIN | GPIO 13 | (shared) | Touch uses same MOSI |
| T_DO | GPIO 12 | (shared) | Touch uses same MISO |

---

## 10. DFPlayer Mini Connector

| DFPlayer Pin | ESP32 GPIO | Signal | Notes |
|--------------|------------|--------|-------|
| VCC | 5V | Power | 5V supply |
| GND | GND | Ground | — |
| RX | GPIO 43 | DFPLAYER_TX | ESP32 TX → DFPlayer RX (via 1kΩ series R) |
| TX | GPIO 44 | DFPLAYER_RX | DFPlayer TX → ESP32 RX |
| SPK1 | Speaker+ | Audio out | Via audio relay to speaker |
| SPK2 | Speaker− | Audio out | Via audio relay to speaker |

> **Note:** Add 1 kΩ series resistor between ESP32 GPIO43 and DFPlayer RX to limit current. DFPlayer UART runs at 9600 baud.

---

## 11. MCP23017 Shifter Connector

### I2C I/O Expander (Address: 0x20, ESP32 I2C on GPIO 8/9)

| MCP23017 Pin | Connection | Notes |
|--------------|------------|-------|
| VDD | 3.3V | Logic supply |
| VSS | GND | Ground |
| SDA | GPIO 8 (ESP32) | I2C data (4.7k pull-up to 3.3V) |
| SCL | GPIO 9 (ESP32) | I2C clock (4.7k pull-up to 3.3V) |
| A0/A1/A2 | GND | Address = 0x20 |
| RESET | 3.3V (via 10k) | Keep out of reset |
| GPA0 | Shifter PARK | Active LOW (internal pull-up enabled) |
| GPA1 | Shifter REVERSE | Active LOW (internal pull-up enabled) |
| GPA2 | Shifter NEUTRAL | Active LOW (internal pull-up enabled) |
| GPA3 | Shifter FORWARD (D1) | Active LOW (internal pull-up enabled) |
| GPA4 | Shifter FORWARD_D2 | Active LOW (internal pull-up enabled) |

---

## 12. LED Strip Connectors

### Front LED Strip (28× WS2812B, GPIO 47)

| Wire | Connection | Notes |
|------|------------|-------|
| Red | 5V (via RELAY_LED_FRONT / PB10) | Power through front relay |
| White/Green | GND | Ground |
| Data | GPIO 47 (via 330Ω series R) | Data signal from ESP32 |

### Rear LED Strip (16× WS2812B, GPIO 48)

| Wire | Connection | Notes |
|------|------------|-------|
| Red | 5V (via RELAY_LED_REAR / PB11) | Power through rear relay |
| White/Green | GND | Ground |
| Data | GPIO 48 (via 330Ω series R) | Data signal from ESP32 |

> **Bulk capacitor:** Place 1000 µF/10V electrolytic across 5V/GND at each LED strip power input to absorb inrush current.

---

## 13. Missing Information

The following data is **not available** in the repository and should be obtained physically:

| Item | What's Needed | How to Obtain |
|------|---------------|---------------|
| Actual connector photos | Physical photos of all installed connectors | Take photos of PCB/wiring |
| Wire routing paths | Physical cable routing in chassis | Measure and photograph |
| Motor specifications | Motor model, rated voltage/current, stall current | Read motor nameplate |
| Chassis ground points | Location of ground bus bars/studs | Inspect chassis |
| Fuse holder locations | Physical fuse box layout | Photograph fuse panel |
| BTS7960 heatsink specs | Thermal resistance, heatsink model | Measure or photograph |
| 5V/12V regulator models | Exact regulator part numbers | Read PCB markings |
| Battery model/capacity | Battery Ah rating, internal resistance | Read battery label |
| Speaker impedance | Speaker ohms for DFPlayer matching | Measure with multimeter |
| Relay module photos | Exact relay module PCBs used | Photograph relay boards |
