# =============================================================================
#  backup_firmware.ps1 — Snapshot del firmware (STM32G474RE + ESP32-S3)
# =============================================================================
#  Equivalente Windows del script bash. Mismo flujo:
#   1) Lee 512 KB de Flash del STM32 (incluye NVM 123-127).
#   2) Lee 16 MB de Flash del ESP32-S3.
#   3) Guarda metadatos + SHA256.
#
#  Requisitos:
#   - STM32CubeProgrammer instalado (STM32_Programmer_CLI.exe en PATH).
#   - esptool.py instalado (`pip install esptool`).
#   - ST-Link conectado al SWD; USB-Serial al ESP32 con BOOT activo.
#
#  Uso:
#     .\backup_firmware.ps1 [-Esp32Port COM5]
# =============================================================================
[CmdletBinding()]
param(
    [string]$Esp32Port = "COM3"
)

$ErrorActionPreference = "Stop"

$RepoRoot = (git rev-parse --show-toplevel).Trim()
$Ts       = (Get-Date).ToUniversalTime().ToString("yyyyMMdd_HHmmssZ")
$OutDir   = Join-Path $RepoRoot "backups\firmware_$Ts"
$GitSha   = (git -C $RepoRoot rev-parse HEAD).Trim()
$GitBr    = (git -C $RepoRoot rev-parse --abbrev-ref HEAD).Trim()

New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
Set-Location $OutDir

Write-Host "============================================================"
Write-Host "  Backup firmware MarcosDashboard v10 Final"
Write-Host "  Destino: $OutDir"
Write-Host "  Commit:  $GitSha ($GitBr)"
Write-Host "  Fecha:   $Ts"
Write-Host "============================================================"

@(
    "timestamp_utc=$Ts",
    "git_sha=$GitSha",
    "git_branch=$GitBr",
    "host=$env:COMPUTERNAME",
    "user=$env:USERNAME"
) | Set-Content -Path metadata.txt

Write-Host "`n[1/2] Leyendo Flash del STM32G474RE (512 KB incluyendo NVM)..."
& STM32_Programmer_CLI -c port=SWD mode=UR -r32 0x08000000 0x80000 stm32_full.bin
& STM32_Programmer_CLI -c port=SWD mode=UR -r32 0x0807B000 0x5000 stm32_nvm_pages_123_127.bin

Write-Host "`n[2/2] Leyendo Flash del ESP32-S3 en $Esp32Port (16 MB)..."
& esptool.py --chip esp32s3 --port $Esp32Port --baud 921600 read_flash 0x0 0x1000000 esp32_full.bin

Write-Host "`nCalculando hashes SHA-256..."
Get-FileHash stm32_full.bin, stm32_nvm_pages_123_127.bin, esp32_full.bin -Algorithm SHA256 |
    ForEach-Object { "$($_.Hash.ToLower())  $(Split-Path -Leaf $_.Path)" } |
    Set-Content -Path SHA256SUMS

Write-Host "`n============================================================"
Write-Host "  COPIA DE SEGURIDAD COMPLETADA"
Write-Host "============================================================"
Get-ChildItem $OutDir | Format-Table Name, Length
Write-Host "`nTag git recomendado:"
Write-Host "  git tag -a pre-remote-control-v10 $GitSha -m 'Snapshot pre-mando RC'"
Write-Host "  git push origin pre-remote-control-v10"
