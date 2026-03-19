#!/bin/bash
# Fuzzing Script for STM32-Control-Coche-Marcos
# Prerequisites: afl++ (apt install afl++), gcc
# Usage: ./run_fuzz.sh [duration_seconds] [output_dir]

set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
STUB_DIR="${REPO_ROOT}/analysis_artifacts/stubs"
FUZZ_DIR="${REPO_ROOT}/analysis_artifacts/fuzz"
DURATION="${1:-30}"
OUT_DIR="${2:-${FUZZ_DIR}}"
mkdir -p "$OUT_DIR"

export AFL_I_DONT_CARE_ABOUT_MISSING_CRASHES=1

echo "=== CAN Fuzz Harness Build — $(date) ==="
echo "AFL++ version: $(afl-fuzz --version 2>&1 | head -1)"
echo "Duration: ${DURATION}s"
echo ""

# Build CAN harness with AFL instrumentation
echo "Building CAN fuzz harness..."
afl-gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE -DFUZZ_HARNESS \
    -I"${STUB_DIR}" -I"${REPO_ROOT}/Core/Inc" \
    -g -O1 \
    "${FUZZ_DIR}/fuzz_can_harness.c" \
    "${REPO_ROOT}/Core/Src/can_handler.c" \
    -o /tmp/fuzz_can_afl -lm

# Create seed corpus
CORPUS_DIR="/tmp/fuzz_can_corpus_$$"
mkdir -p "$CORPUS_DIR"
echo -ne '\x00\x00\x01\x42' > "$CORPUS_DIR/heartbeat"
echo -ne '\x00\x01\x01\x50' > "$CORPUS_DIR/throttle"
echo -ne '\x00\x02\x02\x00\x80' > "$CORPUS_DIR/steering"
echo -ne '\x00\x03\x01\x03' > "$CORPUS_DIR/mode"
echo -ne '\x00\x04\x03\x01\xF0\x00' > "$CORPUS_DIR/service"
echo -ne '\x00\x08\x05\x01\x00\xC8\x00\x01' > "$CORPUS_DIR/obstacle_dist"
echo -ne '\x00\x09\x03\x01\x01\x00' > "$CORPUS_DIR/obstacle_safety"

# Run fuzzer
FUZZ_OUT="/tmp/fuzz_can_out_$$"
rm -rf "$FUZZ_OUT"
echo ""
echo "=== Starting AFL++ fuzzing for ${DURATION}s ==="
timeout $((DURATION + 10)) afl-fuzz \
    -i "$CORPUS_DIR" \
    -o "$FUZZ_OUT" \
    -V "$DURATION" \
    -- /tmp/fuzz_can_afl 2>&1 | tail -5

echo ""
echo "=== Results ==="
if [ -d "$FUZZ_OUT/default" ]; then
    crashes=$(ls "$FUZZ_OUT/default/crashes/" 2>/dev/null | grep -v README | wc -l)
    hangs=$(ls "$FUZZ_OUT/default/hangs/" 2>/dev/null | wc -l)
    echo "Crashes: $crashes"
    echo "Hangs: $hangs"
    grep -E "execs_done|paths_total|unique_crashes|unique_hangs" "$FUZZ_OUT/default/fuzzer_stats" 2>/dev/null
    
    # Save results
    mkdir -p "$OUT_DIR/can_results"
    cp "$FUZZ_OUT/default/fuzzer_stats" "$OUT_DIR/can_results/" 2>/dev/null
    if [ "$crashes" -gt 0 ]; then
        cp "$FUZZ_OUT/default/crashes/"* "$OUT_DIR/can_results/" 2>/dev/null
        echo "*** CRASHES FOUND — review $OUT_DIR/can_results/ ***"
    fi
fi

# Cleanup
rm -rf "$CORPUS_DIR" "$FUZZ_OUT" /tmp/fuzz_can_afl

echo ""
echo "=== Fuzzing complete ==="
