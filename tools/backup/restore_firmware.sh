#!/usr/bin/env bash
# ============================================================================
#  restore_firmware.sh — Restaura un snapshot creado por backup_firmware.sh
# ============================================================================
#  Uso:
#     ./restore_firmware.sh <ruta_backup> [esp32_port]
#  Ejemplo:
#     ./restore_firmware.sh backups/firmware_20260524_180000Z /dev/ttyUSB0
#
#  Restaura:
#    - Flash STM32 completa (incluyendo NVM y por tanto las calibraciones).
#    - Flash ESP32-S3 completa.
#
#  ATENCIÓN: sobrescribe TODA la Flash. Verifica SHA256 antes y después.
# ============================================================================

set -euo pipefail

BACKUP_DIR="${1:?Falta ruta del backup}"
ESP32_PORT="${2:-/dev/ttyUSB0}"

cd "${BACKUP_DIR}"

echo "Verificando SHA256 del snapshot..."
sha256sum -c SHA256SUMS

echo
echo "Restaurando Flash STM32 (512 KB)..."
if command -v STM32_Programmer_CLI >/dev/null 2>&1; then
  STM32_PROG="STM32_Programmer_CLI"
else
  STM32_PROG="/opt/st/stm32cubeprog/bin/STM32_Programmer_CLI"
fi

"${STM32_PROG}" -c port=SWD mode=UR -e all
"${STM32_PROG}" -c port=SWD mode=UR -w stm32_full.bin 0x08000000 -v

echo
echo "Restaurando Flash ESP32-S3 (16 MB)..."
esptool.py --chip esp32s3 --port "${ESP32_PORT}" --baud 921600 \
  write_flash 0x0 esp32_full.bin

echo
echo "Restauración completada. Verifica arranque del coche."
