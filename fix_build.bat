@echo off
REM fix_build.bat - Fix circular-dependency build errors in STM32CubeIDE
REM
REM Run this script AFTER STM32CubeMX code generation (which may overwrite
REM .cproject with broken element IDs), or whenever you see build errors like:
REM
REM   make: Circular *.o ^<- *.o dependency dropped.
REM   Building file:
REM   arm-none-eabi-gcc "" ...
REM
REM Root cause: when a .cproject element's "id" attribute equals its
REM "superClass" attribute, Eclipse CDT confuses the instance with the
REM template and generates subdir.mk rules where .o files depend on
REM themselves instead of the corresponding .c source file.

echo ======================================
echo STM32CubeIDE Build Fix Script
echo ======================================
echo.

set FIXED=0

REM ---- Step 1: Clean stale Debug directory ----
if exist "Debug" (
    echo Removing stale Debug\ directory...
    rmdir /s /q Debug
    set FIXED=1
)

REM ---- Step 2: Check .cproject exists ----
if not exist ".cproject" (
    echo ERROR: .cproject not found in %CD%
    exit /b 1
)

echo Checking .cproject for id==superClass conflicts...

REM Use PowerShell to detect and fix id==superClass conflicts in .cproject
powershell -Command ^
  "$xml = [xml](Get-Content '.cproject'); " ^
  "$fixed = 0; " ^
  "$all = $xml.SelectNodes('//*[@id and @superClass]'); " ^
  "foreach ($node in $all) { " ^
  "  if ($node.id -eq $node.superClass) { " ^
  "    $suffix = Get-Random -Minimum 100000000 -Maximum 999999999; " ^
  "    $newId = $node.id + '.' + $suffix; " ^
  "    Write-Host ('   Fixing: ' + $node.id); " ^
  "    Write-Host ('        -> ' + $newId); " ^
  "    $node.id = $newId; " ^
  "    $fixed++; " ^
  "  } " ^
  "} " ^
  "if ($fixed -gt 0) { " ^
  "  $xml.Save((Resolve-Path '.cproject').Path); " ^
  "  Write-Host ('   Fixed ' + $fixed + ' conflict(s).'); " ^
  "} else { " ^
  "  Write-Host '   No id==superClass conflicts found.'; " ^
  "}"

REM ---- Step 3: Check assembler inputType superClass ----
findstr /C:"assembler.input.s" .cproject >nul 2>&1
if %ERRORLEVEL%==0 (
    echo    Fixing assembler inputType superClass
    powershell -Command ^
      "(Get-Content '.cproject') -replace 'assembler\.input\.s\"', 'assembler.input\"' | Set-Content '.cproject'"
    set FIXED=1
)

echo.
echo Done. Next steps:
echo    1. In STM32CubeIDE: right-click project -^> Refresh (F5^)
echo    2. Project -^> Clean... -^> select project -^> Clean
echo    3. Build the project (Ctrl+B^)
echo.
pause
