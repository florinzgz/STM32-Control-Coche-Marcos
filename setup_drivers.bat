@echo off
REM Script to download and setup STM32 HAL Drivers for Windows
REM This is a convenience script for users who don't want to use STM32CubeMX

echo ================================================
echo STM32 HAL Drivers Setup Script (Windows)
echo ================================================
echo.

REM Check if Drivers directory already exists
if exist "Drivers" (
    echo WARNING: Drivers directory already exists.
    set /p OVERWRITE="Do you want to overwrite it? (y/N): "
    if /i not "%OVERWRITE%"=="y" (
        echo Aborted.
        exit /b 1
    )
    rmdir /s /q Drivers
)

echo.
echo This script will download STM32CubeG4 firmware package (~150 MB^)
echo and extract only the necessary HAL drivers.
echo.
set /p CONTINUE="Continue? (y/N): "
if /i not "%CONTINUE%"=="y" (
    echo Aborted.
    exit /b 1
)

REM Create temp directory
set TEMP_DIR=%TEMP%\stm32_drivers_%RANDOM%
mkdir "%TEMP_DIR%"

echo.
echo Downloading STM32CubeG4 package...
echo This may take a few minutes...
echo.

REM Download using PowerShell
set CUBE_URL=https://github.com/STMicroelectronics/STM32CubeG4/archive/refs/tags/v1.5.1.zip
powershell -Command "& {[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12; Invoke-WebRequest -Uri '%CUBE_URL%' -OutFile '%TEMP_DIR%\stm32cubeg4.zip'}"

if %ERRORLEVEL% neq 0 (
    echo ERROR: Download failed
    rmdir /s /q "%TEMP_DIR%"
    exit /b 1
)

echo.
echo Extracting drivers...
powershell -Command "& {Expand-Archive -Path '%TEMP_DIR%\stm32cubeg4.zip' -DestinationPath '%TEMP_DIR%' -Force}"

if %ERRORLEVEL% neq 0 (
    echo ERROR: Extraction failed
    rmdir /s /q "%TEMP_DIR%"
    exit /b 1
)

REM Find the extracted directory
for /d %%i in ("%TEMP_DIR%\STM32CubeG4-*") do set CUBE_DIR=%%i

if not defined CUBE_DIR (
    echo ERROR: Could not find extracted STM32CubeG4 directory
    rmdir /s /q "%TEMP_DIR%"
    exit /b 1
)

REM Copy only the necessary drivers
echo.
echo Copying HAL drivers to project...
mkdir Drivers
xcopy /E /I /Q "%CUBE_DIR%\Drivers\STM32G4xx_HAL_Driver" "Drivers\STM32G4xx_HAL_Driver"
xcopy /E /I /Q "%CUBE_DIR%\Drivers\CMSIS" "Drivers\CMSIS"

REM Clean up
echo.
echo Cleaning up...
rmdir /s /q "%TEMP_DIR%"

REM Verify critical files exist
echo.
echo Verifying installation...
set VERIFY_OK=1
if not exist "Drivers\STM32G4xx_HAL_Driver\Src\stm32g4xx_hal.c" (
    echo    MISSING: Drivers\STM32G4xx_HAL_Driver\Src\stm32g4xx_hal.c
    set VERIFY_OK=0
)
if not exist "Drivers\STM32G4xx_HAL_Driver\Inc\stm32g4xx_hal.h" (
    echo    MISSING: Drivers\STM32G4xx_HAL_Driver\Inc\stm32g4xx_hal.h
    set VERIFY_OK=0
)
if not exist "Drivers\CMSIS\Device\ST\STM32G4xx\Include\stm32g4xx.h" (
    echo    MISSING: Drivers\CMSIS\Device\ST\STM32G4xx\Include\stm32g4xx.h
    set VERIFY_OK=0
)
if not exist "Drivers\CMSIS\Include\core_cm4.h" (
    echo    MISSING: Drivers\CMSIS\Include\core_cm4.h
    set VERIFY_OK=0
)
if %VERIFY_OK%==0 (
    echo.
    echo ERROR: Some driver files are missing.
    echo    Try deleting the Drivers folder and running this script again.
    exit /b 1
)
echo    All critical files present.

REM Remove stale Debug build directory to prevent circular dependency errors
if exist "Debug" (
    echo.
    echo Removing stale Debug\ directory to prevent build errors...
    rmdir /s /q Debug
)

REM Fix .cproject element IDs if CubeMX has overwritten them
if exist "fix_build.bat" (
    echo.
    echo Verifying .cproject element IDs...
    call fix_build.bat
)

echo.
echo ================================================
echo SUCCESS! STM32 HAL Drivers installed.
echo.
echo Installed components:
echo    - Drivers\STM32G4xx_HAL_Driver\
echo    - Drivers\CMSIS\
echo.
echo Next steps:
echo 1. Open the project in STM32CubeIDE
echo 2. Right-click the project -^> Refresh (F5^)
echo 3. Project -^> Clean... -^> select the project -^> Clean
echo 4. Build the project (Ctrl+B^)
echo 5. Flash to your STM32 board
echo.
echo For more information, see SETUP.md
echo ================================================
pause
