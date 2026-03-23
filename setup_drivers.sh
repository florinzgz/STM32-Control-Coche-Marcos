#!/bin/bash
# Script to download and setup STM32 HAL Drivers
# This is a convenience script for users who don't want to use STM32CubeMX

set -e

echo "================================================"
echo "STM32 HAL Drivers Setup Script"
echo "================================================"
echo ""

# Check if Drivers directory already exists
if [ -d "Drivers" ]; then
    echo "⚠️  Drivers directory already exists."
    read -p "Do you want to overwrite it? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Aborted."
        exit 1
    fi
    rm -rf Drivers
fi

echo "📥 This script will download STM32 HAL drivers and CMSIS headers."
echo "   Three packages will be fetched from GitHub (~160 MB total)."
echo ""
read -p "Continue? (y/N): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Aborted."
    exit 1
fi

# Preferred download command
download() {
    if command -v curl &> /dev/null; then
        curl -fsSL -o "$1" "$2"
    elif command -v wget &> /dev/null; then
        wget -q -O "$1" "$2"
    else
        echo "❌ Error: Neither curl nor wget found. Please install one of them."
        exit 1
    fi
}

# Create temp directory
TEMP_DIR=$(mktemp -d)
trap 'rm -rf "$TEMP_DIR"' EXIT

# ---- 1. Main STM32CubeG4 package (provides CMSIS/Include and project skeleton) ----
echo ""
echo "📦 [1/3] Downloading STM32CubeG4 main package..."
CUBE_URL="https://github.com/STMicroelectronics/STM32CubeG4/archive/refs/tags/v1.6.2.zip"
download "$TEMP_DIR/stm32cubeg4.zip" "$CUBE_URL"
echo "📂 Extracting..."
unzip -q "$TEMP_DIR/stm32cubeg4.zip" -d "$TEMP_DIR"
CUBE_DIR=$(find "$TEMP_DIR" -maxdepth 1 -type d -name "STM32CubeG4-*" | head -n 1)

# ---- 2. HAL Driver (submodule in main repo, empty in zip) ----
echo "📦 [2/3] Downloading STM32G4xx HAL Driver..."
HAL_URL="https://github.com/STMicroelectronics/stm32g4xx_hal_driver/archive/refs/tags/v1.2.4.zip"
download "$TEMP_DIR/hal_driver.zip" "$HAL_URL"
echo "📂 Extracting..."
unzip -q "$TEMP_DIR/hal_driver.zip" -d "$TEMP_DIR"
HAL_DIR=$(find "$TEMP_DIR" -maxdepth 1 -type d -name "stm32g4xx*hal*driver*" | head -n 1)

# ---- 3. CMSIS Device (submodule in main repo, empty in zip) ----
echo "📦 [3/3] Downloading CMSIS Device STM32G4xx..."
CMSIS_DEV_URL="https://github.com/STMicroelectronics/cmsis_device_g4/archive/refs/tags/v1.2.4.zip"
download "$TEMP_DIR/cmsis_device.zip" "$CMSIS_DEV_URL"
echo "📂 Extracting..."
unzip -q "$TEMP_DIR/cmsis_device.zip" -d "$TEMP_DIR"
CMSIS_DEV_DIR=$(find "$TEMP_DIR" -maxdepth 1 -type d -name "cmsis*device*g4*" | head -n 1)

# ---- Assemble Drivers/ directory ----
echo ""
echo "📋 Assembling Drivers/ directory..."
mkdir -p Drivers/STM32G4xx_HAL_Driver
mkdir -p Drivers/CMSIS/Device/ST/STM32G4xx
mkdir -p Drivers/CMSIS/Include

# Copy HAL Driver (from dedicated repo, not the empty submodule stub)
cp -r "$HAL_DIR/Inc" Drivers/STM32G4xx_HAL_Driver/
cp -r "$HAL_DIR/Src" Drivers/STM32G4xx_HAL_Driver/
# Copy Legacy headers if present
if [ -d "$HAL_DIR/Inc/Legacy" ]; then
    mkdir -p Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
fi

# Copy CMSIS core headers (from main CubeG4 package)
if [ -d "$CUBE_DIR/Drivers/CMSIS/Include" ]; then
    cp -r "$CUBE_DIR/Drivers/CMSIS/Include/"* Drivers/CMSIS/Include/
elif [ -d "$CUBE_DIR/Drivers/CMSIS/Core/Include" ]; then
    cp -r "$CUBE_DIR/Drivers/CMSIS/Core/Include/"* Drivers/CMSIS/Include/
fi

# Copy CMSIS Device headers (from dedicated repo, not the empty submodule stub)
cp -r "$CMSIS_DEV_DIR/Include" Drivers/CMSIS/Device/ST/STM32G4xx/
cp -r "$CMSIS_DEV_DIR/Source" Drivers/CMSIS/Device/ST/STM32G4xx/

echo "🧹 Cleaning up temporary files..."

# Verify critical files exist
echo "🔍 Verifying installation..."
VERIFY_OK=true
for f in \
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c \
    Drivers/STM32G4xx_HAL_Driver/Inc/stm32g4xx_hal.h \
    Drivers/CMSIS/Device/ST/STM32G4xx/Include/stm32g4xx.h \
    Drivers/CMSIS/Include/core_cm4.h; do
    if [ ! -f "$f" ]; then
        echo "   ❌ Missing: $f"
        VERIFY_OK=false
    fi
done

if [ "$VERIFY_OK" = false ]; then
    echo ""
    echo "❌ ERROR: Some driver files are missing."
    echo "   Try deleting the Drivers/ folder and running this script again."
    exit 1
fi
echo "   ✅ All critical files present."

# Remove stale Debug build directory to prevent circular dependency errors
if [ -d "Debug" ]; then
    echo ""
    echo "🧹 Removing stale Debug/ directory to prevent build errors..."
    rm -rf Debug
fi

# Fix .cproject element IDs if CubeMX has overwritten them
if [ -f "./fix_build.sh" ]; then
    echo ""
    echo "🔧 Verifying .cproject element IDs..."
    chmod +x ./fix_build.sh 2>/dev/null || true
    ./fix_build.sh
fi

echo ""
echo "✅ Success! STM32 HAL Drivers installed."
echo ""
echo "📊 Installed components:"
echo "   - Drivers/STM32G4xx_HAL_Driver/"
echo "   - Drivers/CMSIS/"
echo ""
echo "Next steps:"
echo "1. Open the project in STM32CubeIDE"
echo "2. Right-click the project → Refresh (F5)"
echo "3. Project → Clean... → select the project → Clean"
echo "4. Build the project (Ctrl+B)"
echo "5. Flash to your STM32 board"
echo ""
echo "For more information, see SETUP.md"
echo "================================================"
