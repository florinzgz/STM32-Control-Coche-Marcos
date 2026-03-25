#!/usr/bin/env bash
# Repository integrity checks for STM32-Control-Coche-Marcos
# Validates: Drivers/ immutability, no template .c files, HAL version pinned.
# Usage: ./scripts/check_integrity.sh

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
EXPECTED_HAL_VERSION="V1.2.2"
ERRORS=0

# ── Check 1: Drivers/ has NOT changed vs main ──────────────────────────
echo "=== Check 1: Drivers/ immutability ==="
if git -C "$REPO_ROOT" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  if git -C "$REPO_ROOT" rev-parse --verify origin/main >/dev/null 2>&1; then
    CHANGED=$(git -C "$REPO_ROOT" diff --name-only origin/main...HEAD -- Drivers/ || true)
    if [ -n "$CHANGED" ]; then
      echo "FAIL: Drivers/ has modifications vs origin/main:"
      echo "$CHANGED"
      ERRORS=$((ERRORS + 1))
    else
      echo "PASS: Drivers/ unchanged"
    fi
  else
    echo "SKIP: origin/main not available (not a full clone?)"
  fi
else
  echo "SKIP: not inside a git repository"
fi

# ── Check 2: No template .c files ──────────────────────────────────────
echo ""
echo "=== Check 2: No forbidden template .c files ==="
TEMPLATES=$(find "$REPO_ROOT" -type f -name "*template*.c" -not -path "*/.git/*" || true)
if [ -n "$TEMPLATES" ]; then
  echo "FAIL: Forbidden template .c files found:"
  echo "$TEMPLATES"
  ERRORS=$((ERRORS + 1))
else
  echo "PASS: No template .c files found"
fi

# ── Check 3: HAL version is exactly V1.2.2 ─────────────────────────────
echo ""
echo "=== Check 3: HAL version pinned to $EXPECTED_HAL_VERSION ==="
HAL_FILE="$REPO_ROOT/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c"
if [ -f "$HAL_FILE" ]; then
  if grep -q "STM32G4xx HAL Driver version number $EXPECTED_HAL_VERSION" "$HAL_FILE"; then
    echo "PASS: HAL version is $EXPECTED_HAL_VERSION"
  else
    echo "FAIL: HAL version mismatch (expected $EXPECTED_HAL_VERSION)"
    grep "HAL Driver version number" "$HAL_FILE" || true
    ERRORS=$((ERRORS + 1))
  fi
else
  echo "FAIL: HAL source file not found: $HAL_FILE"
  ERRORS=$((ERRORS + 1))
fi

# ── Summary ────────────────────────────────────────────────────────────
echo ""
if [ "$ERRORS" -gt 0 ]; then
  echo "INTEGRITY CHECK FAILED ($ERRORS error(s))"
  exit 1
else
  echo "ALL INTEGRITY CHECKS PASSED"
fi
