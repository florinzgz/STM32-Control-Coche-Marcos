#!/usr/bin/env bash
# ============================================================================
#  backup_firmware.sh — Snapshot completo del firmware grabado en el coche
# ============================================================================
#  Copia de seguridad de los binarios actualmente flasheados en el STM32G474RE
#  y en el ESP32-S3 ANTES de cualquier cambio (p.ej. integración del mando RC).
#
#  Qué hace:
#    1) Crea un directorio fechado en backups/firmware_<UTC>/
#    2) Lee la Flash COMPLETA del STM32 (0x08000000..0x0807FFFF, 512 KB)
#       incluyendo las páginas NVM 123-127 (calibraciones de pedal, mapa de
#       sensores DS18B20, log de errores, calibración de steering y parámetros
#       EPS). Si no se copian, una restauración perdería las calibraciones.
#    3) Lee la Flash COMPLETA del ESP32-S3 (16 MB, modelo N16R8).
#    4) Guarda metadatos: SHA del commit, fecha UTC, versiones de toolchain.
#    5) Genera SHA256 de cada binario.
#
#  Requisitos:
#    - STM32CubeProgrammer instalado (proporciona STM32_Programmer_CLI)
#    - esptool.py (pip install esptool) — para el ESP32-S3
#    - ST-Link V2/V3 conectado al SWD del STM32
#    - USB-Serial al ESP32-S3 con bootloader accesible (BOOT pulsado durante reset)
#
#  Uso:
#    ./backup_firmware.sh [esp32_port]
#       esp32_port por defecto: /dev/ttyUSB0
#
#  IMPORTANTE:
#    Ejecutar en un equipo conectado físicamente al coche, no en el sandbox
#    del agente. El agente NO tiene acceso al hardware.
# ============================================================================

set -euo pipefail

ESP32_PORT="${1:-/dev/ttyUSB0}"
REPO_ROOT="$(git rev-parse --show-toplevel)"
TS="$(date -u +%Y%m%d_%H%M%SZ)"
OUT_DIR="${REPO_ROOT}/backups/firmware_${TS}"
GIT_SHA="$(git -C "${REPO_ROOT}" rev-parse HEAD)"
GIT_BRANCH="$(git -C "${REPO_ROOT}" rev-parse --abbrev-ref HEAD)"

mkdir -p "${OUT_DIR}"
cd "${OUT_DIR}"

echo "============================================================"
echo "  Backup firmware MarcosDashboard v10 Final"
echo "  Destino: ${OUT_DIR}"
echo "  Commit:  ${GIT_SHA} (${GIT_BRANCH})"
echo "  Fecha:   ${TS}"
echo "============================================================"

# ---------------------------------------------------------------------------
# 1) Metadatos
# ---------------------------------------------------------------------------
{
  echo "timestamp_utc=${TS}"
  echo "git_sha=${GIT_SHA}"
  echo "git_branch=${GIT_BRANCH}"
  echo "host=$(uname -a)"
  echo "user=$(whoami)"
} > metadata.txt

# ---------------------------------------------------------------------------
# 2) STM32G474RE — Flash completa incluyendo NVM (512 KB)
# ---------------------------------------------------------------------------
echo
echo "[1/2] Leyendo Flash del STM32G474RE (0x08000000..0x0807FFFF, 512 KB)..."
if command -v STM32_Programmer_CLI >/dev/null 2>&1; then
  STM32_PROG="STM32_Programmer_CLI"
elif [ -x "/opt/st/stm32cubeprog/bin/STM32_Programmer_CLI" ]; then
  STM32_PROG="/opt/st/stm32cubeprog/bin/STM32_Programmer_CLI"
else
  echo "ERROR: STM32_Programmer_CLI no encontrado. Instala STM32CubeProgrammer." >&2
  exit 1
fi

# 0x80000 = 524288 bytes = 512 KB (incluye NVM pages 123-127)
"${STM32_PROG}" -c port=SWD mode=UR -r32 0x08000000 0x80000 stm32_full.bin

# Sub-volcado solo de NVM por comodidad (páginas 123-127, 20 KB)
"${STM32_PROG}" -c port=SWD mode=UR -r32 0x0807B000 0x5000 stm32_nvm_pages_123_127.bin

# ---------------------------------------------------------------------------
# 3) ESP32-S3 — Flash completa (16 MB)
# ---------------------------------------------------------------------------
echo
echo "[2/2] Leyendo Flash del ESP32-S3 en ${ESP32_PORT} (16 MB)..."
if ! command -v esptool.py >/dev/null 2>&1; then
  echo "ERROR: esptool.py no encontrado. Ejecuta: pip install esptool" >&2
  exit 1
fi

esptool.py --chip esp32s3 --port "${ESP32_PORT}" --baud 921600 \
  read_flash 0x0 0x1000000 esp32_full.bin

# ---------------------------------------------------------------------------
# 4) Hashes SHA-256
# ---------------------------------------------------------------------------
echo
echo "Calculando hashes SHA-256..."
sha256sum stm32_full.bin stm32_nvm_pages_123_127.bin esp32_full.bin > SHA256SUMS

echo
echo "============================================================"
echo "  COPIA DE SEGURIDAD COMPLETADA"
echo "============================================================"
ls -lh "${OUT_DIR}"
echo
echo "Tag git recomendado:"
echo "  git tag -a pre-remote-control-v10 ${GIT_SHA} -m 'Snapshot pre-mando RC'"
echo "  git push origin pre-remote-control-v10"
