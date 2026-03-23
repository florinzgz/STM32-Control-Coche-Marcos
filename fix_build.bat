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

REM ---- Step 4: Fix trailing empty Defaults field ----
powershell -Command "if ((Get-Content '.cproject' -Raw) -match '\|\|\s*\x22\s*valueType=') { exit 1 } else { exit 0 }"
if %ERRORLEVEL%==1 (
    echo    Fixing trailing empty Defaults field
    powershell -Command ^
      "(Get-Content '.cproject') -replace '\|\|\s*(\x22\s*valueType=)', '$1' | Set-Content '.cproject'"
    set FIXED=1
)

REM ---- Step 5: Fix .mxproject corruption ----
REM Handles: duplicates, .c in LibFiles, .h in SourceFiles, trailing semicolons,
REM bare empty list lines, and duplicate sections.
if exist ".mxproject" (
    echo Checking .mxproject for corruption...
    powershell -Command ^
      "$changed = 0; " ^
      "$lines = Get-Content '.mxproject'; " ^
      "$out = @(); $seenSections = @{}; $skipSection = $false; " ^
      "foreach ($line in $lines) { " ^
      "  if ($line -match '^\[.+\]$') { " ^
      "    if ($seenSections.ContainsKey($line)) { " ^
      "      Write-Host ('   Removing duplicate section: ' + $line); " ^
      "      $changed++; $skipSection = $true; continue; " ^
      "    }; $seenSections[$line] = $true; $skipSection = $false; $out += $line; continue; " ^
      "  }; " ^
      "  if ($skipSection) { continue; }; " ^
      "  if ($line -match '^(HeaderFiles|SourceFiles)=;?$') { " ^
      "    Write-Host ('   Removing bare empty line: ' + $line); $changed++; continue; " ^
      "  }; " ^
      "  if ($line -match '^LibFiles=(.+)$') { " ^
      "    $items = $Matches[1] -split ';' | Where-Object { $_ -ne '' }; " ^
      "    $cFiles = $items | Where-Object { $_ -like '*.c' }; " ^
      "    if ($cFiles) { foreach ($cf in $cFiles) { Write-Host ('   Removing .c from LibFiles: ' + $cf) }; $changed++; }; " ^
      "    $items = $items | Where-Object { $_ -notlike '*.c' }; " ^
      "    $unique = $items | Select-Object -Unique | Sort-Object; " ^
      "    if ($items.Count -ne $unique.Count) { Write-Host '   Deduplicating LibFiles'; $changed++; }; " ^
      "    $out += 'LibFiles=' + ($unique -join ';'); continue; " ^
      "  }; " ^
      "  if ($line -match '^SourceFiles=(.+)$' -and $line -notmatch '^SourceFiles#') { " ^
      "    $items = $Matches[1] -split ';' | Where-Object { $_ -ne '' }; " ^
      "    $hFiles = $items | Where-Object { $_ -like '*.h' }; " ^
      "    if ($hFiles) { foreach ($hf in $hFiles) { Write-Host ('   Removing .h from SourceFiles: ' + $hf) }; $changed++; }; " ^
      "    $items = $items | Where-Object { $_ -notlike '*.h' }; " ^
      "    $unique = $items | Select-Object -Unique | Sort-Object; " ^
      "    if ($items.Count -ne $unique.Count) { Write-Host '   Deduplicating SourceFiles'; $changed++; }; " ^
      "    $out += 'SourceFiles=' + ($unique -join ';'); continue; " ^
      "  }; " ^
      "  if ($line -match '^CDefines=(.+)$') { " ^
      "    $items = $Matches[1] -split ';' | Where-Object { $_ -ne '' }; " ^
      "    $unique = $items | Select-Object -Unique; " ^
      "    if ($items.Count -ne $unique.Count) { Write-Host '   Deduplicating CDefines'; $changed++; }; " ^
      "    $out += 'CDefines=' + ($unique -join ';'); continue; " ^
      "  }; " ^
      "  if ($line -match '^HeaderPath=(.+)$' -and $line -notmatch '^HeaderPath#') { " ^
      "    $items = $Matches[1] -split ';' | Where-Object { $_ -ne '' }; " ^
      "    $unique = $items | Select-Object -Unique; " ^
      "    if ($items.Count -ne $unique.Count) { Write-Host '   Deduplicating HeaderPath'; $changed++; }; " ^
      "    $out += 'HeaderPath=' + ($unique -join ';'); continue; " ^
      "  }; " ^
      "  $out += $line; " ^
      "} " ^
      "if ($changed -gt 0) { " ^
      "  $out | Set-Content '.mxproject'; " ^
      "  Write-Host ('   Fixed ' + $changed + ' issue(s).'); " ^
      "  exit 1; " ^
      "} else { " ^
      "  Write-Host '   No .mxproject corruption found.'; " ^
      "  exit 0; " ^
      "}"
    if %ERRORLEVEL%==1 set FIXED=1
)

REM ---- Step 6: Detect and fix unexpected STM32\ folder structure ----
if exist "STM32" (
    echo Detected unexpected STM32\ directory...
    if exist "STM32\Drivers" (
        echo    Moving STM32\Drivers\ to Drivers\...
        if exist "Drivers" (
            xcopy /e /y /q "STM32\Drivers\*" "Drivers\" >nul 2>&1
        ) else (
            move "STM32\Drivers" "Drivers" >nul 2>&1
        )
        set FIXED=1
    )
    if exist "STM32\Core" (
        echo    Moving STM32\Core\ to Core\...
        if exist "Core" (
            xcopy /e /y /q "STM32\Core\*" "Core\" >nul 2>&1
        ) else (
            move "STM32\Core" "Core" >nul 2>&1
        )
        set FIXED=1
    )
    rmdir /s /q "STM32" 2>nul
    echo    Removed STM32\ directory.
) else (
    echo No unexpected STM32\ directory found.
)

echo.
echo Done. Next steps:
echo    1. In STM32CubeIDE: right-click project -^> Refresh (F5^)
echo    2. Project -^> Clean... -^> select project -^> Clean
echo    3. Build the project (Ctrl+B^)
echo.
pause
