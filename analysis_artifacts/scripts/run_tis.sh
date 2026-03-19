#!/bin/bash
# TIS-Analyzer Script (Fallback Notice)
# TrustInSoft Analyzer is commercial software and not available in this environment.
# This script documents what WOULD be run with TIS-Analyzer.
# The actual analysis was performed with CBMC + Infer as fallback.

set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
STUB_DIR="${REPO_ROOT}/analysis_artifacts/stubs"
OUT_DIR="${1:-${REPO_ROOT}/analysis_artifacts/tis}"
mkdir -p "$OUT_DIR"

echo "=== TIS-Analyzer Status ==="
echo "TrustInSoft Analyzer is commercial software (https://trust-in-soft.com/)."
echo "It requires a license and is not available in this CI/CD environment."
echo ""
echo "=== Fallback ==="
echo "Formal verification was performed using:"
echo "  - CBMC 5.95.1 (bounded model checking)"
echo "  - Infer v1.2.0 (abstract interpretation)"
echo "  - AFL++ 4.09c (fuzzing)"
echo ""
echo "Run the following scripts instead:"
echo "  ./analysis_artifacts/scripts/run_cbmc.sh"
echo "  ./analysis_artifacts/scripts/run_infer.sh"
echo "  ./analysis_artifacts/scripts/run_fuzz.sh"
echo ""

# Document what TIS-Analyzer would have been used for:
cat > "$OUT_DIR/tis_would_analyze.txt" << 'EOF'
TIS-Analyzer would verify the following properties on all Core/Src/*.c files:

1. Absence of undefined behavior (C11 standard compliance)
2. No buffer overflows or out-of-bounds access
3. No null pointer dereferences
4. No use of uninitialized variables
5. No signed integer overflow
6. No division by zero
7. No invalid pointer arithmetic
8. No invalid type conversions (especially float/int)
9. No data races in ISR/main-loop shared variables
10. Full value analysis with abstract interpretation

These properties are covered by CBMC (items 1-8) and Infer (items 1-5, 7-8).
Item 9 (data races) was analyzed manually.
Item 10 is partially covered by Infer's abstract domain.

Files that would be analyzed:
  Core/Src/main.c
  Core/Src/motor_control.c
  Core/Src/can_handler.c
  Core/Src/sensor_manager.c
  Core/Src/safety_system.c
  Core/Src/service_mode.c
  Core/Src/ackermann.c
  Core/Src/steering_centering.c
  Core/Src/boot_validation.c
  Core/Src/encoder_reader.c
  Core/Src/eps_params.c
  Core/Src/error_log.c
  Core/Src/steering_cal_store.c
  Core/Src/math_safety.c
  Core/Src/stm32g4xx_it.c
  Core/Src/stm32g4xx_hal_msp.c
  Core/Src/system_stm32g4xx.c
  Core/Src/syscalls.c
EOF

echo "Documentation written to $OUT_DIR/tis_would_analyze.txt"
