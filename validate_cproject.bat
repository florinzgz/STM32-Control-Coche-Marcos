@echo off
REM validate_cproject.bat - Comprehensive .cproject health check (Windows)
REM
REM Validates the STM32CubeIDE .cproject file for common corruption
REM patterns that cause circular-dependency build errors.  Run this
REM script after every CubeMX code generation.
REM
REM Checks performed:
REM   1. File existence
REM   2. Elements where id == superClass
REM   3. Duplicate element IDs
REM   4. Wrong assembler inputType superClass
REM   5. Required tool elements present
REM   6. Stale Debug/ directory

echo ======================================
echo STM32CubeIDE .cproject Validation
echo ======================================
echo.

set ERRORS=0

REM ---- Check 0: File exists ----
if not exist ".cproject" (
    echo FAIL: .cproject not found in %CD%
    echo.
    echo    You can safely regenerate it:
    echo    1. Open the .ioc file in STM32CubeMX
    echo    2. Click 'Generate Code' (Alt+K^)
    exit /b 2
)

REM ---- Checks 2-5 via PowerShell ----
powershell -Command ^
  "$cproj = '.cproject'; " ^
  "$errors = 0; " ^
  "" ^
  "# Check 2: id == superClass " ^
  "Write-Host ''; Write-Host '1. id == superClass conflicts'; " ^
  "$xml = [xml](Get-Content $cproj); " ^
  "$all = $xml.SelectNodes('//*[@id and @superClass]'); " ^
  "$conflicts = 0; " ^
  "foreach ($n in $all) { " ^
  "  if ($n.id -eq $n.superClass) { " ^
  "    Write-Host ('   FAIL: ' + $n.id); " ^
  "    $conflicts++; $errors++; " ^
  "  } " ^
  "} " ^
  "if ($conflicts -eq 0) { Write-Host '   PASS: no id==superClass conflicts.' } " ^
  "" ^
  "# Check 3: Duplicate IDs (only managed-build elements with superClass) " ^
  "Write-Host ''; Write-Host '2. Duplicate element IDs'; " ^
  "$ids = $xml.SelectNodes('//*[@id and @superClass]') | ForEach-Object { $_.id }; " ^
  "$dups = $ids | Group-Object | Where-Object { $_.Count -gt 1 }; " ^
  "if ($dups.Count -eq 0) { " ^
  "  Write-Host '   PASS: all managed-build element IDs are unique.'; " ^
  "} else { " ^
  "  foreach ($d in $dups) { " ^
  "    Write-Host ('   FAIL: duplicate id=' + $d.Name + ' (' + $d.Count + 'x)'); " ^
  "    $errors++; " ^
  "  } " ^
  "} " ^
  "" ^
  "# Check 4: Assembler inputType " ^
  "Write-Host ''; Write-Host '3. Assembler inputType superClass'; " ^
  "$content = Get-Content $cproj -Raw; " ^
  "if ($content -match 'assembler\.input\.s') { " ^
  "  Write-Host '   FAIL: uses .input.s instead of .input'; $errors++; " ^
  "} else { " ^
  "  Write-Host '   PASS: assembler inputType is correct.'; " ^
  "} " ^
  "" ^
  "# Check 5: Required tool elements " ^
  "Write-Host ''; Write-Host '4. Required tool elements'; " ^
  "$tools = @( " ^
  "  @('MCU GCC Assembler',  'managedbuild.tool.assembler'), " ^
  "  @('MCU GCC Compiler',   'managedbuild.tool.c.compiler'), " ^
  "  @('MCU G++ Compiler',   'managedbuild.tool.cpp.compiler'), " ^
  "  @('MCU GCC Linker',     'managedbuild.tool.c.linker'), " ^
  "  @('MCU G++ Linker',     'managedbuild.tool.cpp.linker'), " ^
  "  @('Linker script',      'STM32G474RETX_FLASH.ld'), " ^
  "  @('Debug config',       'name=\"\"Debug\"\"') " ^
  "); " ^
  "foreach ($t in $tools) { " ^
  "  if ($content -match [regex]::Escape($t[1])) { " ^
  "    Write-Host ('   PASS: ' + $t[0]); " ^
  "  } else { " ^
  "    Write-Host ('   FAIL: MISSING ' + $t[0]); $errors++; " ^
  "  } " ^
  "} " ^
  "" ^
  "# Check 6: Trailing empty Defaults field " ^
  "Write-Host ''; Write-Host '5. Trailing empty Defaults field'; " ^
  "if ($content -match '\|\|\s*\x22\s*valueType=') { " ^
  "  Write-Host '   FAIL: trailing empty Defaults field'; $errors++; " ^
  "} else { " ^
  "  Write-Host '   PASS: no trailing empty Defaults field.'; " ^
  "} " ^
  "" ^
  "# Summary " ^
  "Write-Host ''; Write-Host '======================================'; " ^
  "if ($errors -eq 0) { " ^
  "  Write-Host 'All checks passed. Your .cproject is healthy.'; " ^
  "} else { " ^
  "  Write-Host ($errors.ToString() + ' error(s) detected.'); " ^
  "  Write-Host ''; " ^
  "  Write-Host 'To fix: run fix_build.bat'; " ^
  "  Write-Host 'Or delete .cproject and regenerate from .ioc'; " ^
  "} " ^
  "exit $errors"

set ERRORS=%ERRORLEVEL%

REM ---- Check 6: Stale Debug/ ----
echo.
echo 6. Stale Debug/ directory
if exist "Debug" (
    echo    WARNING: Debug/ directory exists. Consider deleting it before rebuild.
) else (
    echo    PASS: no Debug/ directory.
)

echo.
if %ERRORS% GTR 0 (
    echo %ERRORS% error(s^) found. Run fix_build.bat to repair.
    exit /b 1
) else (
    echo All validation checks passed.
    exit /b 0
)
