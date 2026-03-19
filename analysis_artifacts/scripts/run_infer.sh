#!/bin/bash
# Infer Static Analysis Script for STM32-Control-Coche-Marcos
# Prerequisites: infer (https://github.com/facebook/infer/releases), gcc, g++
# Usage: ./run_infer.sh [output_dir]

set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
STUB_DIR="${REPO_ROOT}/analysis_artifacts/stubs"
OUT_DIR="${1:-${REPO_ROOT}/analysis_artifacts/infer}"
mkdir -p "$OUT_DIR"

echo "=== Infer Static Analysis — $(date) ==="
echo "Infer version: $(infer --version 2>&1 | head -1)"
echo ""

# --- STM32 Analysis ---
echo "=== Phase 1: STM32 (C) Analysis ==="
STM32_OUT="${OUT_DIR}/stm32"
rm -rf "$STM32_OUT"
CFLAGS="-std=c11 -DHOST_TEST -D_GNU_SOURCE -I${STUB_DIR} -I${REPO_ROOT}/Core/Inc"

infer capture --results-dir "$STM32_OUT" -- gcc $CFLAGS -c \
  "${REPO_ROOT}/Core/Src/main.c" \
  "${REPO_ROOT}/Core/Src/motor_control.c" \
  "${REPO_ROOT}/Core/Src/can_handler.c" \
  "${REPO_ROOT}/Core/Src/sensor_manager.c" \
  "${REPO_ROOT}/Core/Src/safety_system.c" \
  "${REPO_ROOT}/Core/Src/service_mode.c" \
  "${REPO_ROOT}/Core/Src/ackermann.c" \
  "${REPO_ROOT}/Core/Src/steering_centering.c" \
  "${REPO_ROOT}/Core/Src/boot_validation.c" \
  "${REPO_ROOT}/Core/Src/encoder_reader.c" \
  "${REPO_ROOT}/Core/Src/eps_params.c" \
  "${REPO_ROOT}/Core/Src/error_log.c" \
  "${REPO_ROOT}/Core/Src/steering_cal_store.c" \
  "${REPO_ROOT}/Core/Src/math_safety.c" \
  "${REPO_ROOT}/Core/Src/stm32g4xx_it.c" \
  "${REPO_ROOT}/Core/Src/stm32g4xx_hal_msp.c" \
  "${REPO_ROOT}/Core/Src/system_stm32g4xx.c" \
  "${REPO_ROOT}/Core/Src/syscalls.c"

infer analyze --results-dir "$STM32_OUT" --bufferoverrun --pulse --biabduction
cp "$STM32_OUT/report.json" "$OUT_DIR/infer_stm32_report.json"
echo ""

# --- ESP32 Analysis ---
echo "=== Phase 2: ESP32 (C++) Analysis ==="
ESP32_OUT="${OUT_DIR}/esp32"
rm -rf "$ESP32_OUT"
ESPFLAGS="-std=c++17 -I${REPO_ROOT}/esp32/src -I${REPO_ROOT}/esp32/include -I${REPO_ROOT}/esp32/test_stubs"

infer capture --results-dir "$ESP32_OUT" -- g++ $ESPFLAGS -c \
  "${REPO_ROOT}/esp32/src/relay_audio.cpp" \
  "${REPO_ROOT}/esp32/src/shifter_input.cpp" \
  "${REPO_ROOT}/esp32/src/traction_switch.cpp" \
  "${REPO_ROOT}/esp32/src/sensors/obstacle_sensor.cpp"

infer analyze --results-dir "$ESP32_OUT" --bufferoverrun --pulse --biabduction
cp "$ESP32_OUT/report.json" "$OUT_DIR/infer_esp32_report.json"

echo ""
echo "=== Infer analysis complete. Reports in ${OUT_DIR}/ ==="
