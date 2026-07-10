#!/usr/bin/env bash

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ARTIFACT_DIR="${REPO_ROOT}/artifacts/static-analysis"
SUMMARY_FILE="${ARTIFACT_DIR}/summary.txt"
SCAN_BUILD_LOG="${ARTIFACT_DIR}/scan-build.txt"
SCAN_BUILD_HTML_DIR="${ARTIFACT_DIR}/scan-build-html"
LIZARD_LOG="${ARTIFACT_DIR}/lizard.txt"
FLAWFINDER_LOG="${ARTIFACT_DIR}/flawfinder.txt"
SEMGREP_NOTE="${ARTIFACT_DIR}/semgrep.txt"
OVERALL_STATUS=0

mkdir -p "${ARTIFACT_DIR}"

run_and_capture() {
  local name="$1"
  local logfile="$2"
  shift 2
  local status=0

  {
    echo "=== ${name} ==="
    "$@"
  } > "${logfile}" 2>&1 || status=$?

  if [ "${status}" -ne 0 ]; then
    OVERALL_STATUS="${status}"
  fi

  return "${status}"
}

cd "${REPO_ROOT}"

run_and_capture "cppcheck" "${ARTIFACT_DIR}/cppcheck-driver.txt" ./scripts/run_cppcheck.sh || true

if command -v flawfinder >/dev/null 2>&1; then
  run_and_capture \
    "flawfinder" \
    "${FLAWFINDER_LOG}" \
    flawfinder --minlevel=3 --error-level=3 Core/Src Core/Inc esp32/src esp32/include || true
else
  echo "SKIP: flawfinder is not installed." > "${FLAWFINDER_LOG}"
fi

if command -v lizard >/dev/null 2>&1; then
  run_and_capture \
    "lizard" \
    "${LIZARD_LOG}" \
    bash -lc 'lizard Core/Src Core/Inc esp32/src esp32/include -l c -l cpp -T cyclomatic_complexity=15 -w || true'
else
  echo "SKIP: lizard is not installed." > "${LIZARD_LOG}"
fi

run_and_capture "clang-tidy" "${ARTIFACT_DIR}/clang-tidy-driver.txt" ./scripts/run_clang_tidy.sh || true

if command -v scan-build >/dev/null 2>&1; then
  rm -rf "${SCAN_BUILD_HTML_DIR}"
  mkdir -p "${SCAN_BUILD_HTML_DIR}"
  {
    echo "=== scan-build ==="
    scan-build --keep-going -o "${SCAN_BUILD_HTML_DIR}" \
      bash -lc '
        set -euo pipefail
        cd "'"${REPO_ROOT}"'"
        while IFS= read -r f; do
          clang -std=gnu11 -fsyntax-only \
            -DUSE_HAL_DRIVER -DSTM32G474xx \
            -ICore/Inc \
            -isystem Drivers/STM32G4xx_HAL_Driver/Inc \
            -isystem Drivers/STM32G4xx_HAL_Driver/Inc/Legacy \
            -isystem Drivers/CMSIS/Device/ST/STM32G4xx/Include \
            -isystem Drivers/CMSIS/Include \
            "$f"
        done < <(find Core/Src -maxdepth 1 -type f -name "*.c" \
          ! -name "adc.c" \
          ! -name "fdcan.c" \
          ! -name "gpio.c" \
          ! -name "i2c.c" \
          ! -name "iwdg.c" \
          ! -name "tim.c" \
          ! -name "stm32g4xx_hal_msp.c" \
          ! -name "stm32g4xx_it.c" \
          ! -name "system_stm32g4xx.c" \
          ! -name "syscalls.c" \
          ! -name "sysmem.c" \
          ! -name "test_*.c" | sort)
        while IFS= read -r f; do
          clang++ -std=gnu++17 -fsyntax-only \
            -DREMOTE_CONTROL_ENABLED=1 \
            -Iesp32/include \
            -Iesp32/src \
            -Iesp32/src/ui \
            "$f"
        done < <(find esp32/src -type f -name "*.cpp" ! -name "test_*.cpp" | sort)
      '
  } > "${SCAN_BUILD_LOG}" 2>&1 || true
else
  echo "SKIP: scan-build is not installed." > "${SCAN_BUILD_LOG}"
fi

cat > "${SEMGREP_NOTE}" <<'EOF'
SKIP: Semgrep is intentionally not enabled in this repository yet.
Reason: the current goal is a low-noise C/C++ report-only stack, and the existing
tools already cover syntax, bug-prone patterns, analyzer findings, security smoke
tests, and complexity without introducing another high-noise dependency.
EOF

{
  echo "Static analysis summary"
  echo "======================="
  echo "cppcheck:    ${ARTIFACT_DIR}/cppcheck.txt"
  echo "flawfinder:  ${FLAWFINDER_LOG}"
  echo "lizard:      ${LIZARD_LOG}"
  echo "clang-tidy:  ${ARTIFACT_DIR}/clang-tidy.txt"
  echo "scan-build:  ${SCAN_BUILD_LOG}"
  echo "semgrep:     ${SEMGREP_NOTE}"
} > "${SUMMARY_FILE}"

echo "Static analysis artifacts written to ${ARTIFACT_DIR}"
exit "${OVERALL_STATUS}"
