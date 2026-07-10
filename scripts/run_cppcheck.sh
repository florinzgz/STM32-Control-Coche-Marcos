#!/usr/bin/env bash

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ARTIFACT_DIR="${REPO_ROOT}/artifacts/static-analysis"
LOG_FILE="${ARTIFACT_DIR}/cppcheck.txt"
SUPPRESSIONS_FILE="${REPO_ROOT}/.cppcheck-suppressions.txt"
CPPCHECK_ERROR_EXITCODE="${CPPCHECK_ERROR_EXITCODE:-0}"

mkdir -p "${ARTIFACT_DIR}"

if ! command -v cppcheck >/dev/null 2>&1; then
  echo "ERROR: cppcheck is not installed." | tee "${LOG_FILE}"
  exit 1
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

mapfile -t STM32_TEST_SOURCES < <(find Core/Src -maxdepth 1 -type f -name 'test_*.c' | sort)
mapfile -t ESP32_OWNED_SOURCES < <(find esp32/src -type f -name '*.cpp' ! -name 'test_*.cpp' | sort)
mapfile -t ESP32_TEST_SOURCES < <(find esp32/src -maxdepth 1 -type f -name 'test_*.cpp' | sort)

run_cppcheck() {
  local label="$1"
  local enable_set="$2"
  shift 2

  {
    echo "=== ${label} ==="
    cppcheck \
      --enable="${enable_set}" \
      --inconclusive \
      --inline-suppr \
      --quiet \
      --check-level=normal \
      --suppress=missingIncludeSystem \
      --suppress=missingInclude \
      --suppress=unmatchedSuppression \
      --suppressions-list="${SUPPRESSIONS_FILE}" \
      --template='[{severity}] {file}:{line} ({id}) {message}' \
      "$@"
    echo
  } >> "${LOG_FILE}" 2>&1
}

: > "${LOG_FILE}"
echo "Cppcheck $(cppcheck --version 2>&1)" >> "${LOG_FILE}"
echo >> "${LOG_FILE}"

if [ "${CPPCHECK_ERROR_EXITCODE}" != "0" ]; then
  CPPCHECK_EXIT_FLAG=(--error-exitcode="${CPPCHECK_ERROR_EXITCODE}")
else
  CPPCHECK_EXIT_FLAG=()
fi

run_cppcheck \
  "STM32 owned firmware" \
  "warning,style,performance,portability,information" \
  "${CPPCHECK_EXIT_FLAG[@]}" \
  --std=c11 \
  -D__GNUC__=13 \
  -D__GNUC_MINOR__=2 \
  -D__GNUC_PATCHLEVEL__=0 \
  -D__GNUC_STDC_INLINE__=1 \
  -DUSE_HAL_DRIVER \
  -DSTM32G474xx \
  -ICore/Inc \
  -IDrivers/STM32G4xx_HAL_Driver/Inc \
  -IDrivers/STM32G4xx_HAL_Driver/Inc/Legacy \
  -IDrivers/CMSIS/Device/ST/STM32G4xx/Include \
  -IDrivers/CMSIS/Include \
  -iDrivers \
  -iMiddlewares \
  -ibuild \
  -iDebug \
  -iRelease \
  "${STM32_OWNED_SOURCES[@]}"

run_cppcheck \
  "STM32 host tests" \
  "warning,performance,portability,information" \
  "${CPPCHECK_EXIT_FLAG[@]}" \
  --std=c11 \
  -DHOST_TEST \
  -D_GNU_SOURCE \
  -ICore/Inc \
  -Ianalysis_artifacts/stubs \
  -ibuild \
  -iDebug \
  -iRelease \
  "${STM32_TEST_SOURCES[@]}"

run_cppcheck \
  "ESP32 owned firmware" \
  "warning,style,performance,portability,information" \
  "${CPPCHECK_EXIT_FLAG[@]}" \
  --language=c++ \
  --std=c++17 \
  -DREMOTE_CONTROL_ENABLED=1 \
  -Iesp32/include \
  -Iesp32/src \
  -Iesp32/src/ui \
  -Iesp32/src/screens \
  -iesp32/.pio \
  "${ESP32_OWNED_SOURCES[@]}"

run_cppcheck \
  "ESP32 host tests" \
  "warning,performance,portability,information" \
  "${CPPCHECK_EXIT_FLAG[@]}" \
  --language=c++ \
  --std=c++17 \
  -DHOST_TEST \
  -DREMOTE_CONTROL_ENABLED=1 \
  -DREMOTE_CONTROL_TEST_HOOKS \
  -Iesp32/include \
  -Iesp32/src \
  -Iesp32/src/ui \
  -Iesp32/test_stubs \
  "${ESP32_TEST_SOURCES[@]}"

echo "Cppcheck report written to ${LOG_FILE}"
