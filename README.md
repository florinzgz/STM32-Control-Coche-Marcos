# STM32G474RE Control Car - Marcos

## Project Overview
STM32G474RE-based vehicle control system with 4-wheel independent traction, steering motor control, and comprehensive sensor integration for electric vehicle applications.

## Hardware Specifications

### Microcontroller
- **MCU**: STM32G474RE (ARM Cortex-M4F, 170 MHz)
- **Memory**: 128KB RAM, 512KB Flash
- **Package**: LQFP64

### Motor System
- **Traction Motors**: 4x independent (FL, FR, RL, RR)
- **Steering Motor**: 1x with encoder feedback
- **Motor Drivers**: 5x BTS7960 H-bridge drivers (RPWM/LPWM direct from timers, no external logic)
- **Control**: RPWM/LPWM per motor (direction encoded in channel selection) + Enable signals

### Sensors
- **Wheel Speed**: 4x Hall effect sensors
- **Temperature**: 5x DS18B20 (OneWire protocol)
- **Current**: 6x INA226 (I2C via TCA9548A multiplexer)
- **Pedal Position**: 1x Analog (ADC)
- **Steering Position**: Incremental encoder (A/B/Z channels)

### Communication
- **CAN Bus**: FDCAN1 @ 500 kbps to ESP32
- **Protocol**: Custom message set for vehicle control

### Power Management
- **Relays**: 3x power (Main, Traction, Steering) + 2x LED strip (Front, Rear)
- **Watchdog**: Independent watchdog timer (~500 ms)
- **BREAK2**: TIM1/TIM8 BREAK2 armed to Cortex LOCKUP — hardware PWM kill on CPU fault

## Pin Configuration

### Motor PWM Outputs (RPWM/LPWM — no external logic)
| Pin | Function | Timer | Description |
|-----|----------|-------|-------------|
| PA8 | RPWM_FL | TIM1_CH1 | Front Left Motor — forward |
| PA9 | LPWM_FL | TIM1_CH2 | Front Left Motor — reverse |
| PA10 | RPWM_FR | TIM1_CH3 | Front Right Motor — forward |
| PA11 | LPWM_FR | TIM1_CH4 | Front Right Motor — reverse |
| PC6 | RPWM_RL | TIM8_CH1 | Rear Left Motor — forward |
| PC7 | LPWM_RL | TIM8_CH2 | Rear Left Motor — reverse |
| PC8 | RPWM_RR | TIM8_CH3 | Rear Right Motor — forward |
| PC9 | LPWM_RR | TIM8_CH4 | Rear Right Motor — reverse |
| PA6 | RPWM_STEER | TIM3_CH1 | Steering Motor — forward |
| PA7 | LPWM_STEER | TIM3_CH2 | Steering Motor — reverse |

> **Note:** Direction pins (PC0–PC4) are **freed** and no longer used.
> RPWM/LPWM are generated directly by hardware timers. Only one channel is
> active at a time — never simultaneous non-zero.

### Enable Signals
| Pin | Function | Notes |
|-----|----------|-------|
| PC5 | EN_FL | GPIO output, active HIGH |
| PC13 | EN_RR | GPIO output, active HIGH |

> EN pins for FR, RL and STEER BTS7960 modules: wire R_EN/L_EN directly to 3.3 V.

### Relays
| Pin | Function |
|-----|----------|
| PC10 | RELAY_MAIN |
| PC11 | RELAY_TRAC |
| PC12 | RELAY_DIR |
| PB10 | RELAY_LED (front WS2812B 5 V supply) |
| PB11 | RELAY_LED_REAR (rear WS2812B 5 V supply) |

### Encoder Interface
| Pin | Function | Type | Specification |
|-----|----------|------|---------------|
| PA15 | ENC_A | TIM2_CH1 | E6B2-CWZ6C 1200 PPR |
| PB3  | ENC_B | TIM2_CH2 | Quadrature 4800 counts/rev |
| PB4  | ENC_Z | EXTI4 | Index pulse (LJ12A3-4-Z/BX) |

### Communication
| Pin | Function | Protocol |
|-----|----------|----------|
| PB9 | FDCAN1_TX | CAN (AF9) |
| PB8 | FDCAN1_RX | CAN (AF9) |
| PB6 | I2C1_SCL | I2C |
| PB7 | I2C1_SDA | I2C |
| PB0 | OneWire | DS18B20 |

### Analog Input
| Pin | Function |
|-----|----------|
| PA3 | ADC1_IN4 (Pedal) |

### Wheel Speed Sensors (EXTI)
| Pin | Function |
|-----|----------|
| PA0 | EXTI0 — Wheel FL |
| PA1 | EXTI1 — Wheel FR |
| PA2 | EXTI2 — Wheel RL |
| PB15 | EXTI15 — Wheel RR |

## Software Architecture

### Core Modules
1. **motor_control.c/h**: PWM control, PID steering, Ackermann geometry
2. **can_handler.c/h**: FDCAN communication with ESP32
3. **sensor_manager.c/h**: All sensor data acquisition
4. **safety_system.c/h**: ABS, TCS, protection systems
5. **main.c**: Main control loop and initialization

### CAN Message Protocol (500 kbps)

| ID | Direction | Name | Description | Data Format |
|----|-----------|------|-------------|-------------|
| 0x001 | STM32→ESP32 | Heartbeat STM32 | Alive + fault flags (DLC 5) | [counter, state, faultFlags, errorCode, statusFlags] |
| 0x011 | ESP32→STM32 | Heartbeat ESP32 | Alive counter | [counter] |
| 0x100 | ESP32→STM32 | CMD Throttle | Throttle 0-100% | [throttle%] |
| 0x101 | ESP32→STM32 | CMD Steering | Steering angle | [LSB, MSB] |
| 0x102 | ESP32→STM32 | CMD Mode | Gear + 4WD flag | [flags, gear] |
| 0x103 | STM32→ESP32 | CMD ACK | Command ack | [cmdId, result, systemState] |
| 0x120 | ESP32→STM32 | CMD LED | LED relay toggle | [front, rear] |
| 0x200 | STM32→ESP32 | Status Speed | Wheel speeds | [FL_L, FL_H, FR_L, FR_H, RL_L, RL_H, RR_L, RR_H] |
| 0x201 | STM32→ESP32 | Status Current | Motor currents | [FL_L, FL_H, FR_L, FR_H, RL_L, RL_H, RR_L, RR_H] |
| 0x202 | STM32→ESP32 | Status Temp | Temperatures (°C) | [T1, T2, T3, T4, T5] |
| 0x203 | STM32→ESP32 | Status Safety | ABS/TCS/Errors | [abs, tcs, error_code] |
| 0x204 | STM32→ESP32 | Status Steering | Steering position | [angle_L, angle_H, calibrated] |
| 0x205 | STM32→ESP32 | Status Traction | Per-wheel scale (%) | [FL, FR, RL, RR] |
| 0x206 | STM32→ESP32 | Status TempMap | Mapped temps (°C) | [FL, FR, RL, RR, Ambient] |
| 0x207 | STM32→ESP32 | Status Battery | Bus current/voltage | [I_L, I_H, V_L, V_H] |
| 0x208 | ESP32→STM32 | Obstacle Distance | Distance + zone | [mm_L, mm_H, zone, health, counter] |
| 0x209 | ESP32→STM32 | Obstacle Safety | Obstacle state | [zone, status, stuck] |
| 0x20A | STM32→ESP32 | Status Lights | LED relay states | [front, rear] |
| 0x300 | Both | Diag Error | Diagnostic error | [errorCode, subsystem] |
| 0x301 | STM32→ESP32 | Service Faults | Fault bitmask | [b0, b1, b2, b3] |
| 0x302 | STM32→ESP32 | Service Enabled | Enabled bitmask | [b0, b1, b2, b3] |
| 0x303 | STM32→ESP32 | Service Disabled | Disabled bitmask | [b0, b1, b2, b3] |
| 0x110 | ESP32→STM32 | Service Cmd | Module control | [action, moduleId] |

### Control Features

#### Traction Control
- **Modes**: 4x2 (rear only) or 4x4 (all wheels)
- **Throttle**: 0-100% control
- **Emergency Stop**: Immediate power cut

#### Steering Control
- **Type**: Closed-loop PID control
- **Feedback**: Incremental encoder
- **Ackermann Geometry**: Differential angle calculation for inner/outer wheels
- **Calibration**: Auto-center on startup

#### Safety Systems
- **ABS (Anti-lock Braking)**: Wheel slip detection and mitigation
- **TCS (Traction Control)**: Wheel spin prevention
- **Overcurrent Protection**: Per-motor current monitoring
- **Overtemperature Protection**: Thermal shutdown
- **CAN Timeout**: Emergency stop if ESP32 disconnects
- **Watchdog**: System hang detection

### Timing Configuration
- **Main Loop**: 100 Hz (10ms period)
- **Sensor Read**: 20 Hz (50ms)
- **Safety Check**: 100 Hz (10ms)
- **CAN Heartbeat**: 10 Hz (100ms)
- **PWM Frequency**: 20 kHz

## Build Instructions

### ⚠️ IMPORTANT: First-Time Setup

Before building, you **MUST** generate the STM32 HAL drivers using STM32CubeMX:

1. Install [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
2. Open the project file: `STM32-Control-Coche-Marcos.ioc`
3. Click **Project → Generate Code**
4. The HAL drivers will be automatically downloaded to the `Drivers/` folder

📖 **For detailed setup instructions, see [SETUP.md](SETUP.md)**

### Quick Build (after setup)

```bash
# Using STM32CubeIDE
1. Open project in STM32CubeIDE
2. Project → Build Project (Ctrl+B)
3. Run → Debug (F11) to flash

# Using command line (requires arm-none-eabi-gcc)
make clean && make all
```

## Project Structure
```
STM32-Control-Coche-Marcos/
├── Core/                            ← STM32 firmware (C)
│   ├── Inc/
│   │   ├── main.h
│   │   ├── motor_control.h
│   │   ├── can_handler.h
│   │   ├── sensor_manager.h
│   │   ├── safety_system.h
│   │   └── stm32g4xx_it.h
│   └── Src/
│       ├── main.c
│       ├── motor_control.c
│       ├── can_handler.c
│       ├── sensor_manager.c
│       ├── safety_system.c
│       └── stm32g4xx_it.c
├── Drivers/
│   ├── STM32G4xx_HAL_Driver/
│   └── CMSIS/
├── esp32/                           ← ESP32-S3 HMI firmware (C++)
│   ├── platformio.ini
│   ├── src/
│   └── include/
├── docs/
│   ├── CAN_CONTRACT_FINAL.md
│   ├── HMI_STATE_MODEL.md
│   ├── ESP32_FIRMWARE_DESIGN.md
│   └── ...
└── README.md
```

### ESP32-S3 HMI Firmware

The `esp32/` directory contains the HMI firmware for the ESP32-S3, which communicates with the STM32 via CAN bus. It is built with **PlatformIO** using the **Arduino framework** (C++17). 

#### ESP32-S3 Hardware Documentation

📋 **Complete pin connection guides available:**
- **[ESP32 Pin Documentation Index](docs/ESP32_PIN_DOCUMENTATION_INDEX.md)** - Start here for all ESP32 connection docs
- **[Display & CAN-Bus Connections](docs/ESP32_S3_DISPLAY_Y_CAN_CONEXIONES.md)** - Detailed guide (Spanish)
- **[Visual Pin Diagrams](docs/DIAGRAMA_PINES_VISUAL.md)** - ASCII diagrams for wiring
- **[Quick Reference](docs/CONEXIONES_RAPIDAS_ESP32.md)** - Fast lookup tables

**Display:** TFT ST7796 480×320 (GPIO 10, 12–14, 21, 38, 39, 42)  
**CAN-Bus:** TJA1051 transceiver (GPIO 4, 5)  
**Audio:** DFPlayer Mini (GPIO 43 TX, 44 RX) + audio relay (GPIO 11)  
**Traction switch (2WD/4WD):** GPIO 15  
**Shifter:** MCP23017 I2C (GPIO 8 SDA, 9 SCL)  
**LEDs:** WS2812B front (GPIO 47, 28 LEDs), rear (GPIO 48, 16 LEDs)  
**Power:** Ignition sense (GPIO 40), power hold (GPIO 41)  
**Obstacle:** TOFSense-M LiDAR (GPIO 18 UART1 RX, 921600 bps)

See [`docs/ESP32_FIRMWARE_DESIGN.md`](docs/ESP32_FIRMWARE_DESIGN.md) for the full firmware architecture.

## Features
✅ 4-wheel independent traction control
✅ PID-based steering with encoder feedback
✅ Ackermann steering geometry
✅ ABS and TCS safety systems
✅ Multi-sensor integration (temperature, current, speed)
✅ FDCAN communication with ESP32
✅ Overcurrent and overtemperature protection
✅ Independent watchdog
✅ Configurable 4x2/4x4 drive modes

## Author
**Florin Zgureanu** (@florinzgz)

## License
MIT License