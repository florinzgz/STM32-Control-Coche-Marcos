@echo off
REM Script to download and setup STM32 HAL Drivers for Windows
REM Downloads HAL driver, CMSIS core, and CMSIS device packages separately
REM because the main STM32CubeG4 zip does not include git submodule content.

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
echo This script will download STM32 HAL drivers and CMSIS headers.
echo Three packages will be fetched from GitHub (~160 MB total^).
echo.
set /p CONTINUE="Continue? (y/N): "
if /i not "%CONTINUE%"=="y" (
    echo Aborted.
    exit /b 1
)

REM Create temp directory
set TEMP_DIR=%TEMP%\stm32_drivers_%RANDOM%
mkdir "%TEMP_DIR%"

REM ---- 1. Main STM32CubeG4 package (CMSIS/Include) ----
echo.
echo [1/3] Downloading STM32CubeG4 main package...
set CUBE_URL=https://github.com/STMicroelectronics/STM32CubeG4/archive/refs/tags/v1.6.2.zip
powershell -Command "& {[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12; Invoke-WebRequest -Uri '%CUBE_URL%' -OutFile '%TEMP_DIR%\stm32cubeg4.zip'}"
if %ERRORLEVEL% neq 0 ( echo ERROR: Download failed & rmdir /s /q "%TEMP_DIR%" & exit /b 1 )
echo Extracting...
powershell -Command "& {Expand-Archive -Path '%TEMP_DIR%\stm32cubeg4.zip' -DestinationPath '%TEMP_DIR%' -Force}"
for /d %%i in ("%TEMP_DIR%\STM32CubeG4-*") do set CUBE_DIR=%%i

REM ---- 2. HAL Driver (submodule, empty in main zip) ----
echo [2/3] Downloading STM32G4xx HAL Driver...
set HAL_URL=https://github.com/STMicroelectronics/stm32g4xx_hal_driver/archive/refs/tags/v1.2.4.zip
powershell -Command "& {[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12; Invoke-WebRequest -Uri '%HAL_URL%' -OutFile '%TEMP_DIR%\hal_driver.zip'}"
if %ERRORLEVEL% neq 0 ( echo ERROR: Download failed & rmdir /s /q "%TEMP_DIR%" & exit /b 1 )
echo Extracting...
powershell -Command "& {Expand-Archive -Path '%TEMP_DIR%\hal_driver.zip' -DestinationPath '%TEMP_DIR%' -Force}"
for /d %%i in ("%TEMP_DIR%\stm32g4xx*hal*driver*") do set HAL_DIR=%%i

REM ---- 3. CMSIS Device (submodule, empty in main zip) ----
echo [3/3] Downloading CMSIS Device STM32G4xx...
set CMSIS_DEV_URL=https://github.com/STMicroelectronics/cmsis_device_g4/archive/refs/tags/v1.2.4.zip
powershell -Command "& {[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12; Invoke-WebRequest -Uri '%CMSIS_DEV_URL%' -OutFile '%TEMP_DIR%\cmsis_device.zip'}"
if %ERRORLEVEL% neq 0 ( echo ERROR: Download failed & rmdir /s /q "%TEMP_DIR%" & exit /b 1 )
echo Extracting...
powershell -Command "& {Expand-Archive -Path '%TEMP_DIR%\cmsis_device.zip' -DestinationPath '%TEMP_DIR%' -Force}"
for /d %%i in ("%TEMP_DIR%\cmsis*device*g4*") do set CMSIS_DEV_DIR=%%i

REM ---- Assemble Drivers/ ----
echo.
echo Assembling Drivers\ directory...
mkdir Drivers\STM32G4xx_HAL_Driver
mkdir Drivers\CMSIS\Device\ST\STM32G4xx
mkdir Drivers\CMSIS\Include

xcopy /E /I /Q "%HAL_DIR%\Inc" "Drivers\STM32G4xx_HAL_Driver\Inc"
xcopy /E /I /Q "%HAL_DIR%\Src" "Drivers\STM32G4xx_HAL_Driver\Src"

if exist "%CUBE_DIR%\Drivers\CMSIS\Include" (
    xcopy /E /I /Q "%CUBE_DIR%\Drivers\CMSIS\Include" "Drivers\CMSIS\Include"
) else if exist "%CUBE_DIR%\Drivers\CMSIS\Core\Include" (
    xcopy /E /I /Q "%CUBE_DIR%\Drivers\CMSIS\Core\Include" "Drivers\CMSIS\Include"
)

xcopy /E /I /Q "%CMSIS_DEV_DIR%\Include" "Drivers\CMSIS\Device\ST\STM32G4xx\Include"
xcopy /E /I /Q "%CMSIS_DEV_DIR%\Source" "Drivers\CMSIS\Device\ST\STM32G4xx\Source"

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
