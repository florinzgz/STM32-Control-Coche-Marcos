#!/bin/bash
# CBMC Model Checking Script for STM32-Control-Coche-Marcos
# Prerequisites: cbmc (apt install cbmc), gcc
# Usage: ./run_cbmc.sh [output_dir]

set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
STUB_DIR="${REPO_ROOT}/analysis_artifacts/stubs"
OUT_DIR="${1:-${REPO_ROOT}/analysis_artifacts/cbmc}"
mkdir -p "$OUT_DIR"

CFLAGS="-DHOST_TEST -D_GNU_SOURCE -I${STUB_DIR} -I${REPO_ROOT}/Core/Inc"
CBMC_FLAGS="--bounds-check --pointer-check --div-by-zero-check \
  --signed-overflow-check --unsigned-overflow-check \
  --nan-check --float-overflow-check \
  --unwind 10 --unwinding-assertions --object-bits 10"

echo "=== CBMC Analysis — $(date) ==="
echo "CBMC version: $(cbmc --version 2>&1)"
echo ""

run_cbmc() {
    local file="$1" func="$2" logname="$3"
    echo -n "  Analyzing ${file} :: ${func} ... "
    cbmc --function "$func" $CFLAGS $CBMC_FLAGS "${REPO_ROOT}/${file}" \
        > "${OUT_DIR}/${logname}.log" 2>&1
    local rc=$?
    local total=$(grep -c "^\[" "${OUT_DIR}/${logname}.log" 2>/dev/null || echo 0)
    local fails=$(grep -c "FAILURE" "${OUT_DIR}/${logname}.log" 2>/dev/null || echo 0)
    if [ $rc -eq 0 ]; then
        echo "PASSED (${total} checks, 0 failures)"
    else
        echo "FAILED (${fails}/${total} checks failed)"
    fi
}

run_cbmc "Core/Src/safety_system.c"      "Safety_SetState"          "cbmc_safety_setstate"
run_cbmc "Core/Src/can_handler.c"         "CAN_ProcessMessages"     "cbmc_can_process"
run_cbmc "Core/Src/motor_control.c"       "Traction_Update"         "cbmc_traction_update"
run_cbmc "Core/Src/sensor_manager.c"      "Sensor_Init"             "cbmc_sensor_init"
run_cbmc "Core/Src/steering_centering.c"  "SteeringCentering_Step"  "cbmc_steering"
run_cbmc "Core/Src/eps_params.c"          "EPS_Params_Save"         "cbmc_eps_params"
run_cbmc "Core/Src/error_log.c"           "ErrorLog_Init"           "cbmc_errorlog"

echo ""
echo "=== CBMC analysis complete. Logs in ${OUT_DIR}/ ==="
