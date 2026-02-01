# STM32G474RE Control Car - Marcos

## Project Overview
STM32G474RE-based vehicle control system with 4-wheel independent traction, steering motor control, and comprehensive sensor integration for electric vehicle applications. This firmware works in conjunction with an ESP32-S3 HMI system via CAN bus communication.

## Hardware Specifications

### Microcontroller
- **MCU**: STM32G474RE (ARM Cortex-M4, 170 MHz)
- **Memory**: 128KB RAM, 512KB Flash
- **Package**: LQFP64
- **Board**: NUCLEO-G474RE recommended

### Motor System
- **Traction Motors**: 4x independent (FL, FR, RL, RR)
- **Steering Motor**: 1x with encoder feedback
- **Motor Drivers**: 5x DRV8825 stepper drivers
- **Control**: PWM + Direction + Enable signals

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
- **Relays**: 3x (Main, Traction, Steering)
- **Watchdog**: Independent watchdog timer

## Pin Configuration

### PWM Outputs
| Pin | Function | Timer | Description |
|-----|----------|-------|-------------|
| PA8 | PWM_FL | TIM1_CH1 | Front Left Motor |
| PA9 | PWM_FR | TIM1_CH2 | Front Right Motor |
| PA10 | PWM_RL | TIM1_CH3 | Rear Left Motor |
| PA11 | PWM_RR | TIM1_CH4 | Rear Right Motor |
| PC8 | PWM_STEER | TIM8_CH3 | Steering Motor |

### Direction Control
| Pin | Function |
|-----|----------|
| PC0 | DIR_FL |
| PC1 | DIR_FR |
| PC2 | DIR_RL |
| PC3 | DIR_RR |
| PC4 | DIR_STEER |

### Enable Signals
| Pin | Function |
|-----|----------|
| PC5 | EN_FL |
| PC6 | EN_FR |
| PC7 | EN_RL |
| PC8 | EN_RR |
| PC9 | EN_STEER |

### Relays
| Pin | Function |
|-----|----------|
| PC10 | RELAY_MAIN |
| PC11 | RELAY_TRAC |
| PC12 | RELAY_DIR |

### Encoder Interface
| Pin | Function | Type |
|-----|----------|------|
| PA0 | ENC_A | TIM2_CH1 |
| PA1 | ENC_B | TIM2_CH2 |
| PA4 | ENC_Z | EXTI4 |

### Communication
| Pin | Function | Protocol |
|-----|----------|----------|
| PB9 | FDCAN1_TX | CAN |
| PB8 | FDCAN1_RX | CAN |
| PB6 | I2C1_SCL | I2C |
| PB7 | I2C1_SDA | I2C |
| PB0 | OneWire | DS18B20 |

**Note:** CAN bus connects to ESP32-S3 HMI via TJA1051T/3 transceivers. See [FULL-FIRMWARE-Coche-Marcos](https://github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos) for ESP32 side implementation.

### Analog Input
| Pin | Function |
|-----|----------|
| PA3 | ADC1_IN4 (Pedal) |

### External Interrupts
| Pin | Function |
|-----|----------|
| PA15 | Key ON |
| PB10 | Wheel RR |

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
| 0x100 | STM32→ESP32 | Heartbeat STM32 | Alive signal | [0x01, ...] |
| 0x101 | ESP32→STM32 | Heartbeat ESP32 | Alive signal | [0x01, ...] |
| 0x200 | ESP32→STM32 | CMD Throttle | Throttle 0-100% | [throttle%, ...] |
| 0x201 | ESP32→STM32 | CMD Steering | Steering angle | [LSB, MSB, ...] |
| 0x300 | STM32→ESP32 | Status Speed | Wheel speeds | [FL_L, FL_H, FR_L, FR_H, ...] |
| 0x301 | STM32→ESP32 | Status Current | Motor currents | [FL_L, FL_H, ...] |
| 0x302 | STM32→ESP32 | Status Temp | Temperatures | [T1, T2, T3, T4, T5] |
| 0x303 | STM32→ESP32 | Status Safety | ABS/TCS/Errors | [abs, tcs, error_code, ...] |
| 0x304 | STM32→ESP32 | Status Steering | Steering position | [angle_L, angle_H, calibrated, ...] |

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

### Prerequisites
- STM32CubeIDE (latest version)
- STM32CubeMX (for hardware configuration)
- ST-Link debugger/programmer

### Steps
1. Clone the repository
2. Install STM32 HAL drivers (see below)
3. Open project in STM32CubeIDE or use Makefile
4. Build the project (Ctrl+B or `make`)
5. Connect ST-Link to STM32G474RE
6. Flash the firmware (F11 or `make flash`)
7. Monitor via Serial Wire Debug (SWD)

### Installing HAL Drivers
The STM32 HAL drivers are not included in this repository to keep it lightweight. You can obtain them in one of these ways:

1. **STM32CubeMX** (Recommended):
   - Open the `.ioc` file in STM32CubeMX
   - Click "Generate Code" to automatically download and install the HAL drivers

2. **Manual Download**:
   - Download [STM32CubeG4](https://www.st.com/en/embedded-software/stm32cubeg4.html) from ST
   - Extract to `Drivers/` directory in the project root

3. **Build System**:
   - The Makefile will guide you if HAL drivers are missing

## Project Structure
```
STM32-Control-Coche-Marcos/
├── Core/
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
└── README.md
```

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