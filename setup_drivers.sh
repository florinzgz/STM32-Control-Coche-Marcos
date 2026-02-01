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

echo "📥 This script will download STM32CubeG4 firmware package (~150 MB)"
echo "   and extract only the necessary HAL drivers."
echo ""
read -p "Continue? (y/N): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Aborted."
    exit 1
fi

# Create temp directory
TEMP_DIR=$(mktemp -d)
cd "$TEMP_DIR"

echo ""
echo "📦 Downloading STM32CubeG4 package..."
echo "   This may take a few minutes..."

# Download the latest STM32CubeG4 package
# Note: ST doesn't provide direct download links, so we use the GitHub mirror
CUBE_URL="https://github.com/STMicroelectronics/STM32CubeG4/archive/refs/heads/master.zip"

if command -v curl &> /dev/null; then
    curl -L -o stm32cubeg4.zip "$CUBE_URL"
elif command -v wget &> /dev/null; then
    wget -O stm32cubeg4.zip "$CUBE_URL"
else
    echo "❌ Error: Neither curl nor wget found. Please install one of them."
    exit 1
fi

echo ""
echo "📂 Extracting drivers..."
unzip -q stm32cubeg4.zip

# Find the extracted directory
CUBE_DIR=$(find . -maxdepth 1 -type d -name "STM32CubeG4-*" | head -n 1)

if [ -z "$CUBE_DIR" ]; then
    echo "❌ Error: Could not find extracted STM32CubeG4 directory"
    exit 1
fi

# Go back to project directory
cd - > /dev/null

# Copy only the necessary drivers
echo "📋 Copying HAL drivers to project..."
mkdir -p Drivers

cp -r "$TEMP_DIR/$CUBE_DIR/Drivers/STM32G4xx_HAL_Driver" Drivers/
cp -r "$TEMP_DIR/$CUBE_DIR/Drivers/CMSIS" Drivers/

# Clean up
echo "🧹 Cleaning up..."
rm -rf "$TEMP_DIR"

echo ""
echo "✅ Success! STM32 HAL Drivers installed."
echo ""
echo "📊 Installed components:"
echo "   - Drivers/STM32G4xx_HAL_Driver/"
echo "   - Drivers/CMSIS/"
echo ""
echo "Next steps:"
echo "1. Open the project in STM32CubeIDE"
echo "2. Build the project (Ctrl+B)"
echo "3. Flash to your STM32 board"
echo ""
echo "For more information, see SETUP.md"
echo "================================================"
