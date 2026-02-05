# Quick Start Guide - 5 Minutes to Running Firmware

Get your STM32G474RE control firmware up and running in 5 simple steps.

## Prerequisites Checklist

- [ ] STM32CubeIDE installed ([Download](https://www.st.com/en/development-tools/stm32cubeide.html))
- [ ] NUCLEO-G474RE board or compatible STM32G474RE hardware
- [ ] USB cable for programming (ST-Link)
- [ ] (Optional) CAN transceiver hardware for testing CAN features

## Step 1: Get the Code (30 seconds)

```bash
git clone https://github.com/florinzgz/STM32-Control-Coche-Marcos.git
cd STM32-Control-Coche-Marcos
```

## Step 2: Generate HAL Drivers (2 minutes)

**Option A - Automatic (Recommended)**:
```bash
# Linux/macOS
chmod +x setup_drivers.sh
./setup_drivers.sh

# Windows PowerShell
.\setup_drivers.bat
```

**Option B - Using STM32CubeMX**:
1. Open `STM32G474RE-Control.ioc` in STM32CubeMX
2. Click **Project → Generate Code**
3. Wait for HAL drivers to download (~100MB)

## Step 3: Import Project (1 minute)

1. Launch STM32CubeIDE
2. **File → Open Projects from File System**
3. Select the cloned repository folder
4. Click **Finish**

## Step 4: Build (1 minute)

```
Project → Build All (Ctrl+B)
```

Expected output:
```
Building target: STM32-Control-Coche-Marcos.elf
arm-none-eabi-size STM32-Control-Coche-Marcos.elf
   text    data     bss     dec     hex filename
  45234    2156   11892   59282    e792 STM32-Control-Coche-Marcos.elf
Finished building target: STM32-Control-Coche-Marcos.elf
```

## Step 5: Flash to Hardware (30 seconds)

1. Connect NUCLEO-G474RE via USB
2. **Run → Debug (F11)**
3. Click **OK** to switch to Debug perspective
4. **Resume (F8)** to start execution

## Verification

### LED Indicators (if using NUCLEO board)
- Green LED should blink at startup
- System enters main control loop

### Serial Monitor (Optional)
Connect UART2 (PA2/PA3) at 115200 baud to see debug output:
```
STM32 Control System v1.0
Initializing peripherals...
FDCAN1: OK
I2C1: OK  
TIM1/TIM8 PWM: OK
System ready - entering main loop
```

### CAN Traffic (if hardware connected)
Use CAN analyzer to verify:
- Heartbeat messages at 0x001 every 100ms
- Status messages when sensors active

## Next Steps

### Basic Configuration

Edit `Core/Inc/main.h` to customize:

```c
/* Motor control parameters */
#define PWM_FREQUENCY_HZ    20000    /* 20 kHz default */
#define MAX_THROTTLE        100      /* 0-100% */

/* Safety thresholds */
#define TEMP_WARNING_C      60       /* Celsius */
#define CURRENT_LIMIT_A     20       /* Amperes */
```

### Testing Individual Subsystems

**Test PWM Output**:
```c
// In main.c, add after initialization:
Motor_SetDutyCycle(MOTOR_FL, 50);  // 50% duty cycle
```

**Test CAN Transmission**:
```c
CAN_SendHeartbeat();  // Sends heartbeat to CAN ID 0x001
```

**Read Sensors**:
```c
uint16_t speed = Wheel_GetSpeed(WHEEL_FL);
int8_t temp = Temperature_Read(TEMP_MOTOR_FL);
```

## Troubleshooting

### Build Errors

**"HAL drivers not found"**:
- Run `setup_drivers.sh` or regenerate code in CubeMX
- Verify `Drivers/` folder exists and contains STM32G4xx_HAL_Driver/

**"undefined reference to HAL_xxx"**:
- Clean project: **Project → Clean**
- Rebuild: **Project → Build All**

**"recipe for target failed"**:
- Check arm-none-eabi-gcc is in PATH
- Verify STM32CubeIDE installation includes toolchain

### Flashing Errors

**"No ST-Link detected"**:
- Check USB cable connection
- Install ST-Link drivers ([Windows](https://www.st.com/en/development-tools/stsw-link009.html))
- Try different USB port

**"Target voltage too low"**:
- Ensure NUCLEO board is powered
- Check JP5 jumper for power selection (ST-LINK or external)

### Runtime Issues

**System hangs at startup**:
- Check clock configuration in `SystemClock_Config()`
- Verify HSI oscillator enabled in RCC settings

**CAN errors**:
- Verify transceiver hardware connected
- Check termination resistors (120Ω at both ends)
- See `ESP32_STM32_CAN_CONNECTION.md` for wiring

**Watchdog resets**:
- Ensure `HAL_IWDG_Refresh(&hiwdg)` called regularly in main loop
- Increase watchdog timeout if needed

## Documentation

For detailed information, see:

- `README.md` - Project overview and architecture
- `docs/PINOUT.md` - Complete pin configuration
- `docs/CAN_PROTOCOL.md` - CAN message specifications  
- `docs/MOTOR_CONTROL.md` - PWM and control algorithms
- `docs/SAFETY_SYSTEMS.md` - ABS, TCS, protection systems
- `docs/ESP32_STM32_CAN_CONNECTION.md` - Hardware wiring guide

## Getting Help

**Issues or Questions?**
- Open an issue on [GitHub](https://github.com/florinzgz/STM32-Control-Coche-Marcos/issues)
- Check existing documentation in `docs/` folder
- Review example code in `Core/Src/` files

---

**Congratulations!** Your STM32 control firmware is now running. Proceed to hardware integration and testing.
