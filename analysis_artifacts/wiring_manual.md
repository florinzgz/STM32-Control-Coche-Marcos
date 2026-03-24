# Wiring Manual — STM32 + ESP32-S3 Vehicle Control System

> **Source of truth:** `Core/Inc/main.h` (STM32), `esp32/include/*.h` + `esp32/src/*.h` (ESP32-S3)  
> **Generated:** 2026-03-19 | **Branch:** `analysis/wiring-docs`  
> **Companion documents:** `connector_pinouts.md`, `BOM.csv`, `safety_checks.md`, `findings_hardware.csv`

---

## Executive Summary — 10 Critical Points Before Energizing

> **A technician MUST verify ALL 10 items below before applying any power to the system.**

| # | Critical Check | Risk if Missed | Reference |
|---|----------------|----------------|-----------|
| **1** | **Flyback diodes on ALL 5 relay coils** (1N4007, cathode to +V). Relays: PC10, PC11, PC12, PB10, PB11. | MOSFET driver destruction on relay turn-off (50–100V spike) | §6.1 |
| **2** | **CAN bus 120 Ω termination at BOTH ends** (STM32 node and ESP32 node). Measure 60 Ω CANH-to-CANL. | Communication failure, bus-off events, intermittent operation | §3.1 |
| **3** | **Pedal voltage divider: 10 kΩ + 6.8 kΩ** (NOT 1k/2k from outdated docs). Max Vout = 2.02 V on PA3. | Wrong values → ADC over-range or destroyed GPIO | §4.1 |
| **4** | **BTS7960 EN pins: FR/RL/STEER tied to 3.3V**, only FL (PC5) and RR (PC13) are GPIO-controlled. | Motor driver stays disabled; motors won't spin | §2.2 |
| **5** | **Encoder signals (5V) through 6N137 optocouplers**, NOT direct connection to STM32. PC817 too slow for encoder. | 5V on 3.3V GPIO = MCU damage; PC817 drops encoder edges | §4.2 |
| **6** | **10 kΩ pull-down on every relay MOSFET gate**. During MCU reset, GPIO pins float → relay could energize randomly. | Uncontrolled motor activation during boot/reset | §6.1 |
| **7** | **Battery reverse-polarity protection** (P-MOSFET or ideal diode at battery input). | Reversed battery destroys all electronics instantly | §5.1 |
| **8** | **Test PWM with oscilloscope BEFORE connecting motors**. Verify 20 kHz, 0% idle duty on all channels. | Full-speed motor start with no control = safety hazard | §7.1 |
| **9** | **Ground bus: single-point star topology**, NOT daisy chain. All modules reference same GND. | Ground loops cause CAN errors, sensor noise, erratic behavior | §5.2 |
| **10** | **Do NOT use ESP32-S3 GPIO 26–37** (Flash/PSRAM internal bus). Only GPIO 0–21, 38–48 are available. | Flash corruption = bricked module; PSRAM corruption = crash | §8.1 |

---

## Table of Contents

1. [System Architecture Overview](#1-system-architecture-overview)
2. [Motor Driver Wiring (BTS7960)](#2-motor-driver-wiring-bts7960)
3. [CAN Bus Wiring](#3-can-bus-wiring)
4. [Sensor Wiring](#4-sensor-wiring)
5. [Power Distribution](#5-power-distribution)
6. [Relay Wiring](#6-relay-wiring)
7. [Commissioning Procedures](#7-commissioning-procedures)
8. [ESP32-S3 HMI Wiring](#8-esp32-s3-hmi-wiring)
9. [Master Signal Table](#9-master-signal-table)
10. [Appendix: Calculations](#10-appendix-calculations)

---

## 1. System Architecture Overview

```
┌──────────────┐         CAN Bus 500kbps          ┌──────────────┐
│  STM32G474RE │◄═══════ Shielded Twisted Pair ═══►│ ESP32-S3-N16R8│
│  (Nucleo-64) │         120Ω each end             │ (DevKitC-1)  │
│              │                                    │              │
│  Traction    │    ┌──BTS7960(FL)──Motor FL        │  TFT 480×320 │
│  Steering    │    ├──BTS7960(FR)──Motor FR        │  DFPlayer    │
│  Safety      │    ├──BTS7960(RL)──Motor RL        │  LED strips  │
│  Sensors     │    ├──BTS7960(RR)──Motor RR        │  MCP23017    │
│  Relays      │    └──BTS7960(ST)──Motor STEER     │  Traction SW │
│              │                                    │  Power Mgr   │
│  I2C: INA226 │    Relay MAIN/TRAC/DIR (24V)      │  Audio Relay │
│  ADC: Pedal  │    Relay LED F/R (5V)              │  Touch Input │
│  OneWire:Temp│                                    │              │
│  Encoder     │                                    │              │
│  Wheel Sens  │                                    │              │
└──────────────┘                                    └──────────────┘

Power Architecture:
  Battery 24V ──► F1(60A) ──► RELAY_MAIN(PC10) ──► Traction bus + Steering bus
  12V rail   ──► Relay coils, accessories
  5V rail    ──► LED strips (via LED relays), DFPlayer, sensors
  3.3V rail  ──► MCU logic, I2C, CAN transceivers
```

---

## 2. Motor Driver Wiring (BTS7960)

### 2.1 PWM Signal Assignments

All PWM signals are **center-aligned at 20 kHz** (Period = 4249, Prescaler = 0 at 170 MHz SYSCLK).

| Motor | RPWM | LPWM | Timer | EN Pin | INA226 Channel |
|-------|------|------|-------|--------|----------------|
| Front-Left | PA8 (TIM1_CH1) | PA9 (TIM1_CH2) | TIM1 | PC5 (GPIO) | CH0 (1.5 mΩ) |
| Front-Right | PA10 (TIM1_CH3) | PA11 (TIM1_CH4) | TIM1 | **3.3V (tied)** | CH1 (1.5 mΩ) |
| Rear-Left | PC6 (TIM8_CH1) | PC7 (TIM8_CH2) | TIM8 | **3.3V (tied)** | CH2 (1.5 mΩ) |
| Rear-Right | PC8 (TIM8_CH3) | PC9 (TIM8_CH4) | TIM8 | PC13 (GPIO) | CH3 (1.5 mΩ) |
| Steering | PA6 (TIM3_CH1) | PA7 (TIM3_CH2) | TIM3 | **3.3V (tied)** | CH5 (1.5 mΩ) |

> **Note:** Battery INA226 (CH4) uses **0.75 mΩ** shunt (100A/75mV rating). Motor shunts are **1.5 mΩ** (50A/75mV rating). Source: `INA226_SHUNT_MOHM_MOTOR=1.5f`, `INA226_SHUNT_MOHM_BATTERY=0.75f` in `Core/Inc/project_config.h`.

**Source:** `Core/Inc/project_config.h:208-209`, `Core/Src/main.c:652-842`

### 2.2 BTS7960 Wiring Detail (per module)

```
From STM32:
  RPWM pin ──────────────────► BTS7960 RPWM
  LPWM pin ──────────────────► BTS7960 LPWM
  EN pin (or 3.3V) ─┬───────► BTS7960 R_EN
                     └───────► BTS7960 L_EN

Power path (traction motors, 24V):
  24V bus ──► Fuse(30A) ──► INA226 shunt(1.5mΩ) ──► BTS7960 B+
                                                      BTS7960 B− ──► GND bus
                                                      BTS7960 M+ ──► Motor terminal 1
                                                      BTS7960 M− ──► Motor terminal 2

Power path (steering motor, 12V):
  12V bus ──► Fuse(15A) ──► INA226 shunt(1.5mΩ) ──► BTS7960 STEER B+
                                                      BTS7960 B− ──► GND bus
                                                      BTS7960 M+ ──► Motor STEER terminal 1
                                                      BTS7960 M− ──► Motor STEER terminal 2

Logic supply:
  3.3V ──► BTS7960 VCC
  GND  ──► BTS7960 GND (logic)
```

### 2.3 Motor Protection Components

| Component | Value | Location | Purpose | Calculation / Notes |
|-----------|-------|----------|---------|---------------------|
| Fuse per traction motor | 30A slow-blow | Between relay and INA226 shunt | Overcurrent protection | I_max=25A × 1.25 = 31A → 30A fuse |
| Fuse for steering motor | 15A slow-blow | Between DIR relay and INA226 shunt | Overcurrent protection | 12V motor; adjust to actual motor rating |
| INA226 shunt (motors) | **1.5 mΩ** ±1%, 3W, 2512 | In series with B+ line before each BTS7960 | Current measurement | P_max = 50² × 0.0015 = 3.75W → use 5W rated shunt; `INA226_SHUNT_MOHM_MOTOR=1.5f` |
| INA226 shunt (battery) | **0.75 mΩ** ±1%, 5W, 2512 | Between battery+ and RELAY_MAIN COM | Battery current + voltage | P_max = 100² × 0.00075 = 7.5W → use 10W rated shunt; `INA226_SHUNT_MOHM_BATTERY=0.75f` |
| TVS on B+ (traction) | SMBJ30A (30V standoff) | On B+ line at each BTS7960 module | Transient suppression | Clamp 48.4V @ 1A; protects BTS7960 gate from spikes |
| Bulk cap at BTS (traction) | **470 µF / 35V** electrolytic | Between B+ and GND at each BTS7960 (traction) | Inrush/ripple filtering | Handles motor start surges on 24V bus |
| Bulk cap at BTS (steering) | **470 µF / 25V** electrolytic | Between B+ and GND at BTS7960 STEER | Inrush/ripple filtering | For 12V steering bus; 25V rating provides margin |
| Bypass cap at BTS (power) | **100 nF / 50V** X7R ceramic | Between B+ and GND, next to BTS7960 IC | HF PWM noise decoupling | Filters 20 kHz and harmonics from power bus |
| Bypass cap at BTS (logic) | **100 nF / 50V** X7R ceramic | Between VCC and GND_logic at each BTS7960 | 74HC244 buffer decoupling | Prevents logic glitches at 20 kHz switching |
| Snubber cap at motor | **100 nF / 50V** X7R ceramic | Between M+ and M− terminals at motor body | Motor EMI snubber | Suppresses back-EMF spikes from brushed DC motor; critical near steering encoder |

---

## 3. CAN Bus Wiring

### 3.1 Physical Layer

| Parameter | Value | Source |
|-----------|-------|--------|
| Baud rate | 500 kbps | `can_ids.h:21`, `main.c:590` |
| Frame format | Classic CAN (8-byte) | `main.c:600` |
| Termination | 120 Ω at each end | CAN 2.0B specification |
| Cable | Shielded twisted pair, 22 AWG | Recommendation |
| Max length | 5 m (for 500 kbps) | CAN specification |
| Shield grounding | One end only (STM32 side) | EMI best practice |

### 3.2 CAN Transceiver Wiring

**STM32 side (SN65HVD230 or MCP2551):**
```
STM32 PB9 (TX) ──► Pin 1 (TXD)
STM32 PB8 (RX) ◄── Pin 4 (RXD)
3.3V ──► Pin 3 (VCC)  ┌─ 100nF ─┐
GND  ──► Pin 2 (GND)  └─────────┘
Pin 6 (CANH) ──┬── 120Ω ──┬── Pin 7 (CANL)
               │          │
           TVS (PESD2CAN)
               │          │
          To bus CANH    To bus CANL
```

**ESP32 side (TJA1051T/3):**
```
ESP32 GPIO4 (TX) ──► Pin 1 (TXD)
ESP32 GPIO5 (RX) ◄── Pin 4 (RXD)
3.3V ──► Pin 3 (VCC)  ┌─ 100nF ─┐  ┌─ 10µF ─┐
GND  ──► Pin 2 (GND)  └─────────┘  └────────┘
Pin 6 (CANH) ──┬── 120Ω ──┬── Pin 7 (CANL)
               │          │
           TVS (PESD2CAN)
               │          │
          To bus CANH    To bus CANL
```

### 3.3 CAN Message Summary

| ID | Direction | Signal | DLC | Rate |
|----|-----------|--------|-----|------|
| 0x001 | STM32→ESP32 | Heartbeat (state+faults) | 5 | 100 ms |
| 0x011 | ESP32→STM32 | Heartbeat | 1 | 100 ms |
| 0x100 | ESP32→STM32 | Throttle command | 1 | 50 ms |
| 0x101 | ESP32→STM32 | Steering command | 2 | 50 ms |
| 0x102 | ESP32→STM32 | Mode (gear+traction) | 2 | On-demand |
| 0x103 | STM32→ESP32 | Command ACK | 3 | On-demand |
| 0x110 | ESP32→STM32 | Service command | 2 | On-demand |
| 0x120 | ESP32→STM32 | LED relay command | 2 | On-demand |
| 0x200 | STM32→ESP32 | Wheel speeds | 8 | 100 ms |
| 0x201 | STM32→ESP32 | Motor currents | 8 | 100 ms |
| 0x202 | STM32→ESP32 | Temperatures | 5 | 1000 ms |
| 0x203 | STM32→ESP32 | Safety status | 3 | 100 ms |
| 0x204 | STM32→ESP32 | Steering status | 3 | 100 ms |
| 0x205 | STM32→ESP32 | Traction status | 4 | 100 ms |
| 0x206 | STM32→ESP32 | Temperature map | 5 | 1000 ms |
| 0x207 | STM32→ESP32 | Battery V/I | 4 | 100 ms |
| 0x208 | ESP32→STM32 | Obstacle distance | 5 | 66 ms |
| 0x209 | ESP32→STM32 | Obstacle safety | 4 | 100 ms |
| 0x20A | STM32→ESP32 | Lights status | 2 | 1000 ms |
| 0x300 | Bidirectional | Diagnostic error | 2 | On-demand |
| 0x301–0x305 | STM32→ESP32 | Service/error data | 4–8 | 1000 ms |

---

## 4. Sensor Wiring

### 4.1 Pedal Accelerator (ADC)

**Sensor:** SS1324LUA-T Hall-effect linear sensor (5V supply, 0.3–4.8V output)  
**MCU Pin:** PA3 (ADC1_IN4)

```
Pedal Sensor
  VCC (5V) ────────────────────► 5V rail
  GND ─────────────────────────► GND
  OUT (0.3–4.8V) ──┬── R1 (10kΩ) ──┬── PA3 (ADC1_IN4)
                    │                │
                    │          C_ADC (100nF)
                    │                │
                    │               GND
                    │                │
                    │          R2 (6.8kΩ)
                    │                │
                    │               GND
                    │
                BAT54S clamp
                 (to 3.3V and GND rails)
```

**Calculations:**
- Vout_max = 4.8 × 6.8 / (10 + 6.8) = **1.94 V** (well below 3.3V ADC limit)
- Vout_min = 0.3 × 6.8 / (10 + 6.8) = **0.12 V**
- Current through divider = 5.0 / (10k + 6.8k) = **0.30 mA** (negligible load)
- Power in R1 = 0.30² × 10k = **0.9 mW** (rating: 250 mW)
- Filter cutoff = 1 / (2π × (10k‖6.8k) × 100n) = 1 / (2π × 4.05k × 100n) = **393 Hz**

### 4.2 Steering Encoder (Quadrature)

**Encoder:** E6B2-CWZ6C, 1200 PPR (4800 CPR in quadrature mode)  
**MCU Pins:** PA15 (TIM2_CH1/ENC_A), PB3 (TIM2_CH2/ENC_B), PB4 (EXTI4/ENC_Z)

```
Encoder 5V ──► VCC (Brown wire)
              GND (Blue wire)
              
Phase A (Black) ──► 6N137 Optocoupler ──► PA15 (TIM2_CH1)
Phase B (White) ──► 6N137 Optocoupler ──► PB3  (TIM2_CH2)
Phase Z (Orange) ──► 6N137 Optocoupler ──► PB4  (EXTI4)
```

**6N137 connection per channel:**
```
5V side:                3.3V side:
  Encoder OUT ──► R_if (220Ω) ──► Pin 2 (Anode)
  GND ──────────────────────────► Pin 3 (Cathode)
  
  3.3V ──► R_pull (1kΩ) ──► Pin 6 (Vo) ──► STM32 GPIO
  3.3V ──────────────────► Pin 8 (VCC)
  GND ──────────────────► Pin 5 (GND)
  Pin 7 (VE) ──────────► 3.3V (enable)
```

**Timing verification:**
- 6N137 propagation delay: 75–120 ns
- TIM2 digital filter: 282 ns threshold
- 120 ns < 282 ns → **PASS** (optocoupler edges arrive well within filter window)

### 4.3 Wheel Speed Sensors (EXTI)

**Sensor:** LJ12A3-4-Z/BX (NPN NO, inductive proximity, 6–36V supply)  
**MCU Pins:** PA0, PA1, PA2, PB15

> **⚠ Level shifting required.** LJ12A3 output is open-collector NPN with pull-up to supply voltage (6–36V). Use PC817 optocoupler for isolation.

```
LJ12A3 Sensor:
  Brown (VCC) ──► 12V supply
  Blue  (GND) ──► GND
  Black (Signal) ──► PC817 Anode (via 1kΩ current-limiting R)
                     PC817 Cathode ──► GND (sensor side)
                     
  PC817 Collector ──► STM32 GPIO (PA0/PA1/PA2/PB15)
  PC817 Emitter ──► GND (MCU side)
  10kΩ pull-up from GPIO to 3.3V
```

**PC817 timing:**
- Propagation delay: 4–6 µs
- Wheel speed at max 10 km/h, 6 pulses/rev, ~30 cm wheel circumference → ~55 Hz max
- Period at 55 Hz = 18 ms; 6 µs delay is < 0.04% of period → **Acceptable**

### 4.4 Steering Center Sensor (EXTI)

**Sensor:** LJ12A3-4-Z/BX NPN NO (same as wheel sensors)  
**MCU Pin:** PB5 (EXTI5)

Same wiring as wheel sensors via PC817 optocoupler.

### 4.5 Temperature Sensors (OneWire)

**Sensor:** DS18B20 (×5), powered mode  
**MCU Pin:** PB0

```
PB0 ──┬── 4.7kΩ ──► 3.3V
      │
      ├── DS18B20 #1 (DQ pin)
      ├── DS18B20 #2 (DQ pin)
      ├── DS18B20 #3 (DQ pin)
      ├── DS18B20 #4 (DQ pin)
      └── DS18B20 #5 (DQ pin)
      
Each DS18B20:
  VDD ──► 3.3V
  GND ──► GND
  DQ  ──► PB0 bus
```

### 4.6 Current Sensors (I2C)

**Sensor:** INA226 (×6) via TCA9548A multiplexer  
**MCU Pins:** PB6 (SCL), PB7 (SDA) — I2C1 at 400 kHz

| TCA Channel | INA226 | Shunt | Measurement |
|-------------|--------|-------|-------------|
| CH0 | Motor FL | **1.5 mΩ** / ≥5W / 2512 | 0–50A motor current |
| CH1 | Motor FR | **1.5 mΩ** / ≥5W / 2512 | 0–50A motor current |
| CH2 | Motor RL | **1.5 mΩ** / ≥5W / 2512 | 0–50A motor current |
| CH3 | Motor RR | **1.5 mΩ** / ≥5W / 2512 | 0–50A motor current |
| CH4 | Battery 24V | **0.75 mΩ** / ≥10W / 2512 | 0–100A battery current |
| CH5 | Steering | **1.5 mΩ** / ≥5W / 2512 | 0–50A steering current |

**Placement:** Battery shunt (CH4) is placed BEFORE the main relay (between battery terminal and relay input) so voltage is always readable even when relay is open.

---

## 5. Power Distribution

### 5.1 Battery Input Protection

```
Battery 24V (+) ──► P-MOSFET (IRF9540N) ──► F1 (60A slow-blow) ──► System 24V bus
Battery 24V (−) ──► System GND bus

P-MOSFET reverse polarity protection:
  Source ──► Battery (+)
  Drain  ──► System 24V bus
  Gate   ──► Battery (−) via 10kΩ (ensures MOSFET ON when polarity correct)
  
  R_DS(on) = 0.117 Ω → V_drop at 50A = 5.85V (significant!)
  
  Alternative: Schottky diode (simpler but higher drop)
  Alternative: Ideal diode controller (e.g., LTC4357) for minimal drop
```

**⚠ Note:** At high currents (>30A), the P-MOSFET voltage drop causes significant power dissipation. Consider an ideal diode controller for production systems.

### 5.2 Ground Bus

**Topology:** Star ground — all modules connect to a central ground bus bar.

```
                    Ground Bus Bar
                         │
        ┌────────────────┼────────────────┐
        │                │                │
   STM32 GND       ESP32 GND       Power GND
   (logic)          (logic)        (motor/relay)
```

- Power ground and logic ground meet ONLY at the bus bar
- CAN shield grounded at STM32 end only (no shield loops)
- Each BTS7960 GND connects directly to bus bar (not daisy-chained)

### 5.3 Power Rails

| Rail | Source | Fuse | Consumers | Decoupling |
|------|--------|------|-----------|------------|
| 24V | Battery | F1 (60A) | Motors (via relays), INA226 CH4 | 470 µF/35V × 2 |
| 12V | DC-DC from 24V | F5 (10A) | Relay coils, sensors (LJ12A3) | 100 µF/25V |
| 5V | DC-DC from 12V or 24V | F4 (5A) | LED strips, DFPlayer, encoder, ESP32 (via USB) | 470 µF/10V |
| 3.3V | STM32 Nucleo regulator | — | MCU, I2C bus, CAN transceiver | 100 nF + 10 µF per VDD |
| 3.3V | ESP32 DevKit regulator | — | ESP32 SoC, display, MCP23017 | On-module |

---

## 6. Relay Wiring

### 6.1 Relay Driver Circuit

Each relay is driven by a 2N7000 N-channel MOSFET with flyback protection:

```
MCU GPIO (3.3V) ──► R_gate (1kΩ) ──┬──► 2N7000 Gate
                                    │
                               R_pd (10kΩ)
                                    │
                                   GND

2N7000:
  Drain ──► Relay Coil (−)
  Source ──► GND
  
Relay Coil (+) ──► 12V (or 5V for LED relays)
           ┌────┤         ├────┐
           │    Relay Coil     │
           │    │         │    │
           └──D_fw (1N4007)──┘
              (cathode to +V)
```

**R_gate (1 kΩ):** Limits gate charge current from MCU GPIO. I = 3.3V / 1kΩ = 3.3 mA (within GPIO max of 20 mA).

**R_pd (10 kΩ):** Ensures gate is LOW during MCU reset/boot when GPIO is high-impedance. Without this, the gate could float HIGH and accidentally energize the relay.

**D_fw (1N4007):** Clamps relay coil inductive kick to < 1V above supply. Without it, the spike can exceed 100V and destroy the 2N7000 (rated 60V).

### 6.2 Relay Summary

| Relay | MCU Pin | Coil Voltage | Contact Rating | Load | MOSFET | Diode |
|-------|---------|-------------|----------------|------|--------|-------|
| RELAY_MAIN | PC10 | 12V | 30A @ 24V | Master power | Q1 | D1 (1N4007) |
| RELAY_TRAC | PC11 | 12V | 30A @ 24V | Traction motors | Q2 | D2 (1N4007) |
| RELAY_DIR | PC12 | 12V | 30A @ 24V | Steering motor | Q3 | D3 (1N4007) |
| RELAY_LED_F | PB10 | 5V | 10A @ 5V | Front LED strip | Q4 | D4 (1N4007) |
| RELAY_LED_R | PB11 | 5V | 10A @ 5V | Rear LED strip | Q5 | D5 (1N4007) |

---

## 7. Commissioning Procedures

See `safety_checks.md` for detailed step-by-step procedures including:
- Pre-energization visual inspection (20 items)
- Continuity tests with multimeter (19 items)
- Power rail verification (10 items)
- CAN bus verification (8 items)
- Motor driver tests (10 items)
- Sensor verification (13 items)
- Relay tests (7 items)
- ESP32 peripheral tests (10 items)
- Emergency disconnect procedure

---

## 8. ESP32-S3 HMI Wiring

### 8.1 GPIO Safety Rules (ESP32-S3-WROOM-1-N16R8)

**Available GPIO:** 0–21, 38–48 (with restrictions on strapping pins)

| GPIO | Used For | Conflict Risk |
|------|----------|---------------|
| 0 | **UNUSED** (boot strapping) | Must be floating/HIGH at boot |
| 3 | **UNUSED** (eFuse strapping) | Must be floating at boot |
| 19/20 | **UNUSED** (USB D−/D+) | USB function if used |
| 45 | **UNUSED** (VDD_SPI selection) | Avoid as output |
| 46 | **UNUSED** (boot log verbosity) | Avoid as output |

### 8.2 ESP32 I2C Bus (MCP23017 Shifter)

```
ESP32 GPIO8 (SDA) ──┬── 4.7kΩ ──► 3.3V
                     └── MCP23017 Pin 13 (SDA)
ESP32 GPIO9 (SCL) ──┬── 4.7kΩ ──► 3.3V
                     └── MCP23017 Pin 12 (SCL)

MCP23017:
  VDD (Pin 9) ──► 3.3V
  VSS (Pin 10) ──► GND
  A0/A1/A2 (Pins 15/16/17) ──► GND (address 0x20)
  RESET (Pin 18) ──► 3.3V via 10kΩ
  
Port A inputs (GPA0–GPA4) ──► Shifter contacts (active LOW, internal pull-ups)
```

### 8.3 Audio System

```
ESP32 GPIO43 (TX) ──► 1kΩ ──► DFPlayer RX
ESP32 GPIO44 (RX) ◄────────── DFPlayer TX
DFPlayer VCC ──► 5V
DFPlayer GND ──► GND

Audio Relay (GPIO 11, active LOW):
  ESP32 GPIO11 ──► Optocoupled relay IN
  Relay COM ──► Speaker (+)
  Relay NO ──► DFPlayer SPK1 (active: DFPlayer plays to speaker)
  Relay NC ──► Car radio output (default: radio plays to speaker)
```

### 8.4 LED Strips

```
Front Strip (28× WS2812B):
  5V ──► RELAY_LED_FRONT (PB10, STM32) ──► Strip VCC
  GND ──────────────────────────────────► Strip GND
  ESP32 GPIO47 ──► 330Ω ──────────────► Strip DIN
  1000µF/10V across Strip VCC-GND

Rear Strip (16× WS2812B):
  5V ──► RELAY_LED_REAR (PB11, STM32) ──► Strip VCC
  GND ─────────────────────────────────► Strip GND
  ESP32 GPIO48 ──► 330Ω ──────────────► Strip DIN
  1000µF/10V across Strip VCC-GND
```

---

## 9. Master Signal Table

| Signal | MCU Port/Pin | Connector | Cable Color | Length | Shield | Components In-Line | Justification | Test Before Energize |
|--------|-------------|-----------|-------------|--------|--------|-------------------|---------------|---------------------|
| RPWM_FL | PA8 (TIM1_CH1) | BTS7960-FL RPWM | Orange | ≤30 cm | No | — | Direct 3.3V logic | Oscilloscope: 20kHz, 0% idle |
| LPWM_FL | PA9 (TIM1_CH2) | BTS7960-FL LPWM | Orange/White | ≤30 cm | No | — | Direct 3.3V logic | Oscilloscope: 20kHz |
| RPWM_FR | PA10 (TIM1_CH3) | BTS7960-FR RPWM | Blue | ≤30 cm | No | — | Direct 3.3V logic | Oscilloscope: 20kHz |
| LPWM_FR | PA11 (TIM1_CH4) | BTS7960-FR LPWM | Blue/White | ≤30 cm | No | — | Direct 3.3V logic | Oscilloscope: 20kHz |
| RPWM_RL | PC6 (TIM8_CH1) | BTS7960-RL RPWM | Green | ≤30 cm | No | — | Direct 3.3V logic | Oscilloscope: 20kHz |
| LPWM_RL | PC7 (TIM8_CH2) | BTS7960-RL LPWM | Green/White | ≤30 cm | No | — | Direct 3.3V logic | Oscilloscope: 20kHz |
| RPWM_RR | PC8 (TIM8_CH3) | BTS7960-RR RPWM | Yellow | ≤30 cm | No | — | Direct 3.3V logic | Oscilloscope: 20kHz |
| LPWM_RR | PC9 (TIM8_CH4) | BTS7960-RR LPWM | Yellow/White | ≤30 cm | No | — | Direct 3.3V logic | Oscilloscope: 20kHz |
| RPWM_STEER | PA6 (TIM3_CH1) | BTS7960-ST RPWM | Purple | ≤30 cm | No | — | Direct 3.3V logic | Oscilloscope: 20kHz |
| LPWM_STEER | PA7 (TIM3_CH2) | BTS7960-ST LPWM | Purple/White | ≤30 cm | No | — | Direct 3.3V logic | Oscilloscope: 20kHz |
| EN_FL | PC5 | BTS7960-FL R_EN+L_EN | Red | ≤30 cm | No | — | GPIO active HIGH | Multimeter: 0V at boot |
| EN_RR | PC13 | BTS7960-RR R_EN+L_EN | Red | ≤30 cm | No | — | GPIO active HIGH | Multimeter: 0V at boot |
| RELAY_MAIN | PC10 | Relay K1 MOSFET gate | White | ≤50 cm | No | R_gate 1kΩ + R_pd 10kΩ + D_fw 1N4007 | Gate drive + flyback | Multimeter: 0V at boot |
| RELAY_TRAC | PC11 | Relay K2 MOSFET gate | White | ≤50 cm | No | R_gate 1kΩ + R_pd 10kΩ + D_fw 1N4007 | Gate drive + flyback | Multimeter: 0V at boot |
| RELAY_DIR | PC12 | Relay K3 MOSFET gate | White | ≤50 cm | No | R_gate 1kΩ + R_pd 10kΩ + D_fw 1N4007 | Gate drive + flyback | Multimeter: 0V at boot |
| RELAY_LED_F | PB10 | Relay K4 MOSFET gate | White | ≤50 cm | No | R_gate 1kΩ + R_pd 10kΩ + D_fw 1N4007 | Gate drive + flyback | Multimeter: 0V at boot |
| RELAY_LED_R | PB11 | Relay K5 MOSFET gate | White | ≤50 cm | No | R_gate 1kΩ + R_pd 10kΩ + D_fw 1N4007 | Gate drive + flyback | Multimeter: 0V at boot |
| PEDAL_ADC | PA3 (ADC1_IN4) | Pedal sensor connector | Gray | ≤1 m | Yes (if >50cm) | R1 10kΩ + R2 6.8kΩ + C 100nF + BAT54S | 5V→3.3V divider + filter + clamp | Multimeter: 0.12–1.94V |
| CAN_TX | PB9 (FDCAN1_TX) | CAN transceiver TXD | — | ≤10 cm | No | — | On-board connection | — |
| CAN_RX | PB8 (FDCAN1_RX) | CAN transceiver RXD | — | ≤10 cm | No | — | On-board connection | — |
| CANH | CAN xcvr CANH | Bus connector J1 pin 1 | Yellow | ≤5 m | Twisted+shielded | 120Ω term + PESD2CAN TVS | Bus termination + ESD | Measure 60Ω CANH-CANL |
| CANL | CAN xcvr CANL | Bus connector J1 pin 2 | Green | ≤5 m | Twisted+shielded | 120Ω term + PESD2CAN TVS | Bus termination + ESD | Measure 60Ω CANH-CANL |
| I2C_SCL | PB6 (I2C1_SCL) | TCA9548A SCL | Yellow | ≤30 cm | No | 4.7kΩ pull-up to 3.3V | 400kHz bus | Check 4.7kΩ to 3.3V |
| I2C_SDA | PB7 (I2C1_SDA) | TCA9548A SDA | Blue | ≤30 cm | No | 4.7kΩ pull-up to 3.3V | 400kHz bus | Check 4.7kΩ to 3.3V |
| ENC_A | PA15 (TIM2_CH1) | Encoder Black wire | Black | ≤1 m | Yes | 6N137 opto (R_if 220Ω) | 5V→3.3V isolation | Scope: square wave on rotation |
| ENC_B | PB3 (TIM2_CH2) | Encoder White wire | White | ≤1 m | Yes | 6N137 opto (R_if 220Ω) | 5V→3.3V isolation | Scope: 90° phase from A |
| ENC_Z | PB4 (EXTI4) | Encoder Orange wire | Orange | ≤1 m | Yes | 6N137 opto (R_if 220Ω) | 5V→3.3V isolation | Scope: 1 pulse/revolution |
| STEER_CENTER | PB5 (EXTI5) | Center sensor connector | Brown | ≤1 m | No | PC817 opto + 10kΩ pull-up | Level shift from 12V | Multimeter: 0/3.3V toggle |
| WHEEL_FL | PA0 (EXTI0) | Wheel sensor FL connector | Black | ≤2 m | No | PC817 opto + 10kΩ pull-up | Level shift from 12V | Spin wheel: 6 pulses/rev |
| WHEEL_FR | PA1 (EXTI1) | Wheel sensor FR connector | Black | ≤2 m | No | PC817 opto + 10kΩ pull-up | Level shift from 12V | Spin wheel: 6 pulses/rev |
| WHEEL_RL | PA2 (EXTI2) | Wheel sensor RL connector | Black | ≤2 m | No | PC817 opto + 10kΩ pull-up | Level shift from 12V | Spin wheel: 6 pulses/rev |
| WHEEL_RR | PB15 (EXTI15) | Wheel sensor RR connector | Black | ≤2 m | No | PC817 opto + 10kΩ pull-up | Level shift from 12V | Spin wheel: 6 pulses/rev |
| ONEWIRE | PB0 | DS18B20 bus connector | Green | ≤3 m | No | 4.7kΩ pull-up to 3.3V | OneWire bus | Read temp: ~25°C ambient |
| LED_LD2 | PA5 | Nucleo on-board LED | — | — | — | — | Debug indicator | Visible blink at boot |
| ESP_CAN_TX | GPIO4 | TJA1051 TXD | — | ≤10 cm | No | — | On-board | — |
| ESP_CAN_RX | GPIO5 | TJA1051 RXD | — | ≤10 cm | No | — | On-board | — |
| ESP_I2C_SDA | GPIO8 | MCP23017 SDA | Blue | ≤30 cm | No | 4.7kΩ pull-up to 3.3V | Shifter I2C | Check pull-up |
| ESP_I2C_SCL | GPIO9 | MCP23017 SCL | Yellow | ≤30 cm | No | 4.7kΩ pull-up to 3.3V | Shifter I2C | Check pull-up |
| TFT_CS | GPIO10 | ST7796 CS | — | ≤15 cm | No | — | SPI display | Display initializes |
| AUDIO_RELAY | GPIO11 | Relay module IN | White | ≤30 cm | No | — | Active LOW | Relay clicks on audio |
| TFT_MISO | GPIO12 | ST7796 SDO / T_DO | — | ≤15 cm | No | — | SPI shared | — |
| TFT_MOSI | GPIO13 | ST7796 SDI / T_DIN | — | ≤15 cm | No | — | SPI shared | — |
| TFT_SCLK | GPIO14 | ST7796 SCK / T_CLK | — | ≤15 cm | No | — | SPI shared | — |
| TRACTION_SW | GPIO15 | Rocker switch | Brown | ≤1 m | No | Internal pull-up | HIGH=2WD, LOW=4WD | Multimeter: 0/3.3V |
| TOUCH_CS | GPIO21 | XPT2046 T_CS | — | ≤15 cm | No | — | Touch SPI select | Touch responds |
| TFT_RST | GPIO38 | ST7796 RESET | — | ≤15 cm | No | — | Display reset | Display works |
| TFT_DC | GPIO39 | ST7796 DC/RS | — | ≤15 cm | No | — | Data/command | Display works |
| IGNITION | GPIO40 | Ignition switch | Red | ≤1 m | No | — | HIGH=key ON | Multimeter: 0/3.3V |
| POWER_HOLD | GPIO41 | Power latch relay | Green | ≤50 cm | No | — | HIGH=keep power | Multimeter: 3.3V when running |
| TFT_BL | GPIO42 | ST7796 LED/BL | — | ≤15 cm | No | — | Backlight HIGH=ON | Backlight visible |
| DFPLAYER_TX | GPIO43 | DFPlayer RX | Orange | ≤30 cm | No | 1kΩ series R | UART 9600 baud | Audio plays |
| DFPLAYER_RX | GPIO44 | DFPlayer TX | Orange/White | ≤30 cm | No | — | UART 9600 baud | — |
| LED_FRONT | GPIO47 | WS2812B strip DIN | Green | ≤1 m | No | 330Ω series R | Data signal | LEDs animate |
| LED_REAR | GPIO48 | WS2812B strip DIN | Green | ≤1 m | No | 330Ω series R | Data signal | LEDs animate |

---

## 10. Appendix: Calculations

### 10.1 Pedal Voltage Divider

```
R1 = 10 kΩ, R2 = 6.8 kΩ
Vin_max = 4.8 V (SS1324LUA-T at 5V supply, full deflection)
Vin_min = 0.3 V (SS1324LUA-T at 5V supply, no field)

Vout = Vin × R2 / (R1 + R2)
Vout_max = 4.8 × 6800 / (10000 + 6800) = 4.8 × 0.4048 = 1.943 V
Vout_min = 0.3 × 6800 / (10000 + 6800) = 0.3 × 0.4048 = 0.121 V

Safety margin: Vout_max (1.94V) vs ADC rail (3.3V) → 1.36V margin (41%)

Worst case with 1% tolerance resistors:
  R1_min = 9900, R2_max = 6868 → Vout_max = 4.8 × 6868/16768 = 1.965V
  Still well below 3.3V.

Current through divider:
  I = Vin_max / (R1 + R2) = 4.8 / 16800 = 0.286 mA

Power dissipation:
  P_R1 = I² × R1 = (0.286e-3)² × 10000 = 0.82 mW (rating: 250 mW) ✓
  P_R2 = I² × R2 = (0.286e-3)² × 6800 = 0.56 mW (rating: 250 mW) ✓
```

### 10.2 Motor Fuse Selection

```
Motor rated continuous current: 25 A (per BTS7960 limit setting in firmware)
Peak/stall current (brief): ~50 A

Fuse selection criteria:
  - Must NOT blow at continuous rated load (25A)
  - Must blow before wiring/component damage (>50A sustained)
  - Slow-blow type for motor inrush tolerance

Selected: 30A slow-blow blade fuse
  - Holds 30A continuous (> 25A rated)
  - Blows at ~60A within 10 seconds
  - Passes 25A × 1.25 safety factor = 31.25A margin ✓
```

### 10.3 Current Shunt Power

```
Motor shunt (1.5 mΩ):
  P = I² × R = 50² × 0.0015 = 3.75 W (at absolute max 50A)
  Selected: ≥5W rated → margin = 5/3.75 = 1.33× ✓
  Source: INA226_SHUNT_MOHM_MOTOR = 1.5f (project_config.h)

Battery shunt (0.75 mΩ):
  P = I² × R = 100² × 0.00075 = 7.5 W (at absolute max 100A)
  Selected: ≥10W rated → margin = 10/7.5 = 1.33× ✓
  Source: INA226_SHUNT_MOHM_BATTERY = 0.75f (project_config.h)
```

### 10.4 CAN Bus Termination

```
Standard: ISO 11898 requires 120 Ω at each end of the bus.
Parallel resistance of two terminators: 120 × 120 / (120 + 120) = 60 Ω
Measurement: Between CANH and CANL, with no transceivers powered.

Recessive bus voltage: 2.5V (both CANH and CANL)
Dominant differential: CANH − CANL = 2.0V typical (min 1.5V)
Current during dominant: 2.0V / 60Ω = 33 mA through both terminators
Power per resistor: (33e-3)² × 120 = 0.13 W → 0.25W resistor sufficient ✓
```

### 10.5 I2C Pull-Up Selection

```
Bus speed: 400 kHz (Fast Mode)
Estimated bus capacitance: ~100 pF (short runs, < 30 cm)
Rise time requirement: < 300 ns (I2C FM spec)

Pull-up current (3.3V, 4.7 kΩ):
  I_pull = 3.3 / 4700 = 0.702 mA

RC time constant:
  τ = R × C = 4700 × 100e-12 = 470 ns
  Rise time (10%–90%) ≈ 2.2 × τ = 1034 ns

For 400 kHz bus with ~100 pF, 4.7 kΩ is borderline.
If bus issues occur, reduce to 2.2 kΩ (τ = 220 ns, rise time = 484 ns ✓).
For lengths > 50 cm or more capacitance, use I2C bus buffer (e.g., PCA9600).
```

### 10.6 WS2812B LED Power Budget

```
Front strip: 28 LEDs × 60 mA (full white) = 1680 mA = 1.68 A max
Rear strip: 16 LEDs × 60 mA = 960 mA = 0.96 A max
Total: 2.64 A at full white brightness

Typical operation (colored effects, not full white): ~0.5–1.0 A total
Relay contact rating: 10A → more than adequate ✓

Bulk capacitor sizing:
  1000 µF per strip provides charge reservoir for inrush
  C × dV/dt = I → dV = I × dt / C = 1.68 × 0.001 / 0.001 = 1.68 V
  Acceptable ripple with 1000 µF at this load ✓

330 Ω series data resistor:
  Limits ringing on data line; typical recommended range 100–470 Ω
  At 3.3V logic: I_peak through R = 3.3/330 = 10 mA ✓
```
