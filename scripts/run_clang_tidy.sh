#!/usr/bin/env bash

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ARTIFACT_DIR="${REPO_ROOT}/artifacts/static-analysis"
LOG_FILE="${ARTIFACT_DIR}/clang-tidy.txt"
STM32_DB_DIR="${ARTIFACT_DIR}/clang-tidy/stm32"
ESP32_DB_DIR="${ARTIFACT_DIR}/clang-tidy/esp32"

mkdir -p "${ARTIFACT_DIR}" "${STM32_DB_DIR}" "${ESP32_DB_DIR}"

if ! command -v clang-tidy >/dev/null 2>&1; then
  echo "SKIP: clang-tidy is not installed." | tee "${LOG_FILE}"
  exit 0
fi

cd "${REPO_ROOT}"

mapfile -t STM32_OWNED_SOURCES < <(
  find Core/Src -maxdepth 1 -type f -name '*.c' \
    ! -name 'adc.c' \
    ! -name 'fdcan.c' \
    ! -name 'gpio.c' \
    ! -name 'i2c.c' \
    ! -name 'iwdg.c' \
    ! -name 'tim.c' \
    ! -name 'stm32g4xx_hal_msp.c' \
    ! -name 'stm32g4xx_it.c' \
    ! -name 'system_stm32g4xx.c' \
    ! -name 'syscalls.c' \
    ! -name 'sysmem.c' \
    ! -name 'test_*.c' \
    | sort
)

mapfile -t ESP32_OWNED_SOURCES < <(find esp32/src -type f -name '*.cpp' ! -name 'test_*.cpp' | sort)

python3 - "${REPO_ROOT}" "${STM32_DB_DIR}/compile_commands.json" "${STM32_OWNED_SOURCES[@]}" <<'PY'
import json
import pathlib
import shlex
import sys

repo_root = pathlib.Path(sys.argv[1]).resolve()
output = pathlib.Path(sys.argv[2])
files = [pathlib.Path(p) for p in sys.argv[3:]]
base = [
    "clang",
    "-std=gnu11",
    "-DUSE_HAL_DRIVER",
    "-DSTM32G474xx",
    "-ICore/Inc",
    "-isystem", "Drivers/STM32G4xx_HAL_Driver/Inc",
    "-isystem", "Drivers/STM32G4xx_HAL_Driver/Inc/Legacy",
    "-isystem", "Drivers/CMSIS/Device/ST/STM32G4xx/Include",
    "-isystem", "Drivers/CMSIS/Include",
]
entries = []
for file_path in files:
    command = base + ["-c", str(file_path)]
    entries.append({
        "directory": str(repo_root),
        "file": str(repo_root / file_path),
        "command": " ".join(shlex.quote(part) for part in command),
    })
output.write_text(json.dumps(entries, indent=2) + "\n", encoding="utf-8")
PY

{
  echo "clang-tidy $(clang-tidy --version | head -n 1)"
  echo
  echo "=== STM32 owned firmware ==="
} > "${LOG_FILE}"

if ((${#STM32_OWNED_SOURCES[@]} > 0)); then
  clang-tidy -p "${STM32_DB_DIR}" "${STM32_OWNED_SOURCES[@]}" >> "${LOG_FILE}" 2>&1 || true
fi

{
  echo
  echo "=== ESP32 owned firmware ==="
} >> "${LOG_FILE}"

if command -v pio >/dev/null 2>&1; then
  (
    cd esp32
    pio run -e esp32s3 -t compiledb >/dev/null 2>&1 || exit 1
    cp compile_commands.json "${ESP32_DB_DIR}/compile_commands.json"
  ) || echo "SKIP: PlatformIO compile_commands.json generation failed." >> "${LOG_FILE}"
elif command -v platformio >/dev/null 2>&1; then
  (
    cd esp32
    platformio run -e esp32s3 -t compiledb >/dev/null 2>&1 || exit 1
    cp compile_commands.json "${ESP32_DB_DIR}/compile_commands.json"
  ) || echo "SKIP: PlatformIO compile_commands.json generation failed." >> "${LOG_FILE}"
else
  echo "SKIP: PlatformIO is not installed; ESP32 clang-tidy pass not run." >> "${LOG_FILE}"
fi

if [ -f "${ESP32_DB_DIR}/compile_commands.json" ] && ((${#ESP32_OWNED_SOURCES[@]} > 0)); then
  clang-tidy -p "${ESP32_DB_DIR}" "${ESP32_OWNED_SOURCES[@]}" >> "${LOG_FILE}" 2>&1 || true
fi

echo "clang-tidy report written to ${LOG_FILE}"
