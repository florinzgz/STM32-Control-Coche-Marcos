# 🚀 Setup Guide - STM32-Control-Coche-Marcos

This guide will help you set up the development environment and compile the STM32 firmware.

## Prerequisites

### Required Software
- **STM32CubeIDE** (version 1.14.0 or later) - [Download here](https://www.st.com/en/development-tools/stm32cubeide.html)
- **Git** - For cloning the repository

### Hardware
- STM32G474RE microcontroller (NUCLEO-G474RE board recommended)
- ST-Link programmer/debugger (included in NUCLEO board)
- USB cable

## Quick Start

### 1. Clone the Repository

```bash
git clone https://github.com/florinzgz/STM32-Control-Coche-Marcos.git
cd STM32-Control-Coche-Marcos
```

### 2. Install STM32 HAL Drivers

The STM32 HAL drivers are **NOT** included in the repository to keep it lightweight. You need to add them using one of these methods:

#### Method A: Using STM32CubeMX (Recommended)

1. Open **STM32CubeMX**
2. Open the project file: `STM32-Control-Coche-Marcos.ioc`
3. Click **Project → Generate Code**
4. This will automatically download and install the required HAL drivers in the `Drivers/` folder

#### Method B: Manual Download

1. Download STM32CubeG4 package from [ST website](https://www.st.com/en/embedded-software/stm32cubeg4.html)
2. Extract the package
3. Copy the following folders to your project:
   ```
   STM32Cube_FW_G4_VX.X.X/Drivers/STM32G4xx_HAL_Driver/ → Drivers/STM32G4xx_HAL_Driver/
   STM32Cube_FW_G4_VX.X.X/Drivers/CMSIS/ → Drivers/CMSIS/
   ```

### 3. Import Project into STM32CubeIDE

1. Open **STM32CubeIDE**
2. Go to **File → Open Projects from File System...**
3. Click **Directory...** and select the repository folder
4. Click **Finish**

### 4. Build the Project

#### Using STM32CubeIDE:
1. Right-click on project → **Build Project**
2. Or press **Ctrl+B**

#### Using Command Line (requires arm-none-eabi-gcc):
```bash
make clean
make all
```

### 5. Flash to STM32

1. Connect your STM32G474RE board via USB
2. In STM32CubeIDE: **Run → Debug** (F11)
3. Or **Run → Run** (Ctrl+F11) to flash without debugging

## Project Structure

```
STM32-Control-Coche-Marcos/
├── Core/                          # Application code
│   ├── Inc/                       # Header files
│   │   ├── main.h
│   │   ├── motor_control.h
│   │   ├── can_handler.h
│   │   ├── sensor_manager.h
│   │   ├── safety_system.h
│   │   └── stm32g4xx_hal_conf.h
│   └── Src/                       # Source files
│       ├── main.c
│       ├── motor_control.c
│       ├── can_handler.c
│       ├── sensor_manager.c
│       ├── safety_system.c
│       └── stm32g4xx_hal_msp.c
├── Drivers/                       # ⚠️ NOT in repo - Generate with CubeMX
│   ├── STM32G4xx_HAL_Driver/     # HAL drivers (auto-generated)
│   └── CMSIS/                     # CMSIS headers (auto-generated)
├── docs/                          # Documentation
├── Makefile                       # Build script
├── STM32-Control-Coche-Marcos.ioc # CubeMX configuration
├── STM32G474RETX_FLASH.ld        # Linker script
├── startup_stm32g474retx.s       # Startup code
└── .project                       # Eclipse/CubeIDE project file
```

## Troubleshooting

### "Cannot find HAL headers"
- **Solution**: Generate the HAL drivers using STM32CubeMX as described in Step 2

### "undefined reference to HAL_XXX"
- **Solution**: Make sure the Drivers folder is properly generated and the project includes it in the build path

### "No ST-Link detected"
- **Solution**: 
  1. Check USB cable connection
  2. Install ST-Link drivers (included with STM32CubeIDE)
  3. Try a different USB port

### Build errors after cloning
- **Solution**: Always run STM32CubeMX to generate code first before building

### ESP32-S3: "Could not open COMx" / PermissionError
- **Cause**: The serial port is in use by another program, or the board is not in download mode.
- **Solution**:
  1. Close any serial monitor or terminal that is connected to the ESP32 port.
  2. Enter download mode: hold **BOOT**, press **RESET**, release **BOOT**.
  3. Run `pio run -t upload` from the `esp32/` directory.
  4. If the port is wrong, set `upload_port` in `esp32/platformio.ini`.
  5. On Linux, add your user to the `dialout` group: `sudo usermod -aG dialout $USER`
  - See [`esp32/README.md`](esp32/README.md) for detailed troubleshooting.

## Next Steps

1. ✅ Generate HAL drivers with CubeMX
2. ✅ Build the project
3. ✅ Flash to your STM32 board
4. 📖 Read the documentation in `docs/` folder:
   - [PINOUT.md](docs/PINOUT.md) - Pin configuration
   - [CAN_PROTOCOL.md](docs/CAN_PROTOCOL.md) - CAN communication protocol
   - [MOTOR_CONTROL.md](docs/MOTOR_CONTROL.md) - Motor control details
   - [SAFETY_SYSTEMS.md](docs/SAFETY_SYSTEMS.md) - Safety features
   - [BUILD_GUIDE.md](docs/BUILD_GUIDE.md) - Detailed build instructions

## Support

For issues or questions:
- Open an issue on GitHub
- Check the documentation in the `docs/` folder
- Review the [BUILD_GUIDE.md](docs/BUILD_GUIDE.md) for detailed troubleshooting

## License

MIT License - See [LICENSE](LICENSE) file for details

---

**Author**: florinzgz  
**Project**: STM32-Control-Coche-Marcos  
**Version**: 1.0.0
