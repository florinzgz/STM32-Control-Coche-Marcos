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
REM
REM This script:
REM   1. Removes the stale Debug/ build directory
REM   2. Scans .cproject for id==superClass conflicts and fixes them
REM   3. Scans for duplicate element IDs among managed-build elements
REM   4. Fixes the assembler inputType superClass if needed
REM   5. Strips trailing empty Defaults fields (|| "") that cause GCC "" errors
REM   5b. Ensures test_*.c files are excluded from .cproject sourceEntries
REM   6. Fixes .mxproject corruption and consistency with .cproject
REM   7. Detects and fixes unexpected STM32\ folder / UnderRoot setting

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
powershell -NoProfile -Command ^
  "try { " ^
  "  $content = Get-Content '.cproject' -Raw -Encoding UTF8; " ^
  "  $xml = [xml]$content; " ^
  "} catch { " ^
  "  Write-Host '   ERROR: Cannot parse .cproject XML - file may be corrupted'; " ^
  "  Write-Host '   Recommendation: delete .cproject and regenerate from .ioc'; " ^
  "  exit 2; " ^
  "} " ^
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
  "  $utf8NoBom = New-Object System.Text.UTF8Encoding($false); " ^
  "  $sw = New-Object System.IO.StreamWriter((Resolve-Path '.cproject').Path, $false, $utf8NoBom); " ^
  "  $xml.Save($sw); $sw.Close(); " ^
  "  Write-Host ('   Fixed ' + $fixed + ' conflict(s).'); " ^
  "  exit 1; " ^
  "} else { " ^
  "  Write-Host '   No id==superClass conflicts found.'; " ^
  "  exit 0; " ^
  "}"
if %ERRORLEVEL%==1 set FIXED=1
if %ERRORLEVEL%==2 goto :summary

REM ---- Step 3: Fix duplicate element IDs among managed-build elements ----
echo Checking .cproject for duplicate element IDs...
powershell -NoProfile -Command ^
  "try { " ^
  "  $xml = [xml](Get-Content '.cproject' -Raw -Encoding UTF8); " ^
  "} catch { exit 0; } " ^
  "$ids = $xml.SelectNodes('//*[@id and @superClass]') | ForEach-Object { $_.id }; " ^
  "$dups = $ids | Group-Object | Where-Object { $_.Count -gt 1 }; " ^
  "$fixed = 0; " ^
  "if ($dups.Count -gt 0) { " ^
  "  foreach ($d in $dups) { " ^
  "    Write-Host ('   Duplicate id: ' + $d.Name + ' (' + $d.Count + 'x)'); " ^
  "    $nodes = $xml.SelectNodes('//*[@id=""' + $d.Name + '""]'); " ^
  "    $skip = $true; " ^
  "    foreach ($node in $nodes) { " ^
  "      if ($skip) { $skip = $false; continue; } " ^
  "      $suffix = Get-Random -Minimum 100000000 -Maximum 999999999; " ^
  "      $newId = $node.id + '.' + $suffix; " ^
  "      Write-Host ('        -> ' + $newId); " ^
  "      $node.id = $newId; " ^
  "      $fixed++; " ^
  "    } " ^
  "  } " ^
  "  $utf8NoBom = New-Object System.Text.UTF8Encoding($false); " ^
  "  $sw = New-Object System.IO.StreamWriter((Resolve-Path '.cproject').Path, $false, $utf8NoBom); " ^
  "  $xml.Save($sw); $sw.Close(); " ^
  "  Write-Host ('   Fixed ' + $fixed + ' duplicate(s).'); " ^
  "  exit 1; " ^
  "} else { " ^
  "  Write-Host '   No duplicate element IDs found.'; " ^
  "  exit 0; " ^
  "}"
if %ERRORLEVEL%==1 set FIXED=1

REM ---- Step 4: Check assembler inputType superClass ----
findstr /C:"assembler.input.s" .cproject >nul 2>&1
if %ERRORLEVEL%==0 (
    echo    Fixing assembler inputType superClass
    powershell -NoProfile -Command ^
      "$c = Get-Content '.cproject' -Raw -Encoding UTF8; " ^
      "$c = $c -replace 'assembler\.input\.s""', 'assembler.input""'; " ^
      "[System.IO.File]::WriteAllText((Resolve-Path '.cproject').Path, $c, (New-Object System.Text.UTF8Encoding($false)))"
    set FIXED=1
)

REM ---- Step 5: Fix trailing empty Defaults field ----
powershell -NoProfile -Command "if ((Get-Content '.cproject' -Raw) -match '\|\|\s*\x22\s*valueType=') { exit 1 } else { exit 0 }"
if %ERRORLEVEL%==1 (
    echo    Fixing trailing empty Defaults field
    powershell -NoProfile -Command ^
      "$c = Get-Content '.cproject' -Raw -Encoding UTF8; " ^
      "$c = $c -replace '\|\|\s*(\x22\s*valueType=)', '$1'; " ^
      "[System.IO.File]::WriteAllText((Resolve-Path '.cproject').Path, $c, (New-Object System.Text.UTF8Encoding($false)))"
    set FIXED=1
)

REM ---- Step 5b: Ensure test_*.c files are excluded from .cproject sourceEntries ----
if exist "Core\Src" (
    echo Checking .cproject for un-excluded test_*.c files...
    powershell -NoProfile -Command ^
      "$testFiles = Get-ChildItem 'Core\Src\test_*.c' -ErrorAction SilentlyContinue | ForEach-Object { $_.Name }; " ^
      "if (-not $testFiles) { Write-Host '   No test_*.c files found.'; exit 0; } " ^
      "$content = Get-Content '.cproject' -Raw -Encoding UTF8; " ^
      "if ($content -match 'excluding=""([^""]*)""[^>]*name=""Core/Src""') { " ^
      "  $curExcl = $Matches[1]; " ^
      "  $curSet = $curExcl -split '\|'; " ^
      "  $missing = @(); " ^
      "  foreach ($tf in $testFiles) { " ^
      "    if ($curSet -notcontains $tf) { $missing += $tf; } " ^
      "  } " ^
      "  if ($missing.Count -gt 0) { " ^
      "    foreach ($m in $missing) { Write-Host ('   Adding missing exclusion: ' + $m); } " ^
      "    $newExcl = ($curSet + $missing | Where-Object { $_ -ne '' } | Sort-Object) -join '|'; " ^
      "    $content = $content.Replace('excluding=""' + $curExcl + '""', 'excluding=""' + $newExcl + '""'); " ^
      "    [System.IO.File]::WriteAllText((Resolve-Path '.cproject').Path, $content, (New-Object System.Text.UTF8Encoding($false))); " ^
      "    exit 1; " ^
      "  } else { Write-Host '   All test_*.c files are excluded.'; exit 0; } " ^
      "} else { Write-Host '   WARNING: Core/Src sourceEntry not found in .cproject'; exit 0; }"
    if %ERRORLEVEL%==1 set FIXED=1
)

REM ---- Step 6: Fix .mxproject corruption and consistency ----
if exist ".mxproject" (
    echo Checking .mxproject for corruption...
    powershell -NoProfile -Command ^
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

REM ---- Step 6b: .mxproject / .cproject consistency ----
REM Remove files from .mxproject SourceFiles that are excluded in .cproject
REM Drivers sourceEntry.  This prevents CubeMX from re-including them on
REM code regeneration, which can trigger circular-dependency build errors.
if exist ".mxproject" if exist ".cproject" (
    echo Checking .mxproject/.cproject source file consistency...
    powershell -NoProfile -Command ^
      "$cproj = Get-Content '.cproject' -Raw -Encoding UTF8; " ^
      "if ($cproj -match 'excluding=""([^""]*)""[^>]*name=""Drivers""') { " ^
      "  $exclRaw = $Matches[1]; " ^
      "  $exclFiles = ($exclRaw -split '\|') | Where-Object { $_ -like '*.c' } | ForEach-Object { $_ -replace '/', '\' }; " ^
      "  $mxLines = Get-Content '.mxproject'; " ^
      "  $changed = 0; $out = @(); " ^
      "  foreach ($line in $mxLines) { " ^
      "    if ($line -match '^SourceFiles=(.+)$' -and $line -notmatch '^SourceFiles#') { " ^
      "      $items = $Matches[1] -split ';' | Where-Object { $_ -ne '' }; " ^
      "      $filtered = @(); " ^
      "      foreach ($item in $items) { " ^
      "        $rel = $item; " ^
      "        if ($rel -like 'Drivers\*') { $rel = $rel.Substring(8); } " ^
      "        $isExcl = $false; " ^
      "        foreach ($ex in $exclFiles) { " ^
      "          if ($rel -eq $ex) { $isExcl = $true; break; } " ^
      "        } " ^
      "        if ($isExcl) { " ^
      "          Write-Host ('   Removing excluded file from .mxproject: ' + $item); " ^
      "          $changed++; " ^
      "        } else { $filtered += $item; } " ^
      "      } " ^
      "      $out += 'SourceFiles=' + ($filtered -join ';'); continue; " ^
      "    } " ^
      "    $out += $line; " ^
      "  } " ^
      "  if ($changed -gt 0) { " ^
      "    $out | Set-Content '.mxproject'; " ^
      "    Write-Host ('   Removed ' + $changed + ' excluded file(s) from .mxproject.'); " ^
      "    exit 1; " ^
      "  } else { " ^
      "    Write-Host '   .mxproject and .cproject are consistent.'; " ^
      "    exit 0; " ^
      "  } " ^
      "} else { " ^
      "  Write-Host '   No Drivers sourceEntry found in .cproject (skipping).'; " ^
      "  exit 0; " ^
      "}"
    if %ERRORLEVEL%==1 set FIXED=1
)

REM ---- Step 7: Detect and fix unexpected STM32\ folder structure ----
REM Check .ioc for UnderRoot=false
for %%f in (*.ioc) do (
    findstr /C:"ProjectManager.UnderRoot=false" "%%f" >nul 2>&1
    if !ERRORLEVEL!==0 (
        echo    Fixing ProjectManager.UnderRoot=false in %%f
        powershell -NoProfile -Command ^
          "(Get-Content '%%f' -Raw) -replace 'ProjectManager\.UnderRoot=false', 'ProjectManager.UnderRoot=true' | Set-Content '%%f' -NoNewline"
        set FIXED=1
    )
)

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
    if not exist "STM32" echo    Removed STM32\ directory.
) else (
    echo No unexpected STM32\ directory found.
)

:summary
echo.
if %FIXED%==0 (
    echo No issues found. Your project should build correctly.
    echo.
    echo If you still have build errors, try:
    echo    1. Delete the Debug\ folder manually
    echo    2. Project -^> Clean... -^> select project -^> Clean
    echo    3. Rebuild ^(Ctrl+B^)
) else (
    echo Fixes applied. Next steps:
    echo    1. In STM32CubeIDE: right-click project -^> Refresh ^(F5^)
    echo    2. Project -^> Clean... -^> select project -^> Clean
    echo    3. Build the project ^(Ctrl+B^)
)
echo.
pause
