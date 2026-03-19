/**
 ****************************************************************************
 * @file    test_obstacle_sensor.cpp
 * @brief   Host-compilable unit tests for obstacle_sensor (TOFSense-M 8×8).
 *
 *          Tests the multi-pixel protocol parsing ONLY.
 *          Frame format: 400 bytes
 *            [0x57][0x01][0xFF][ID][TIME×4][0x40]
 *            [64 × {DIST_L,DIST_H,DIST_UH,STATUS,SIG_L,SIG_H}]
 *            [0xFF×6][CHECKSUM]
 *
 *          Compile and run on host (from esp32/src/):
 *            g++ -std=c++17 -I. -I../test_stubs \
 *                sensors/obstacle_sensor.cpp test_obstacle_sensor.cpp \
 *                -o /tmp/test_obstacle_sensor && /tmp/test_obstacle_sensor
 *
 *          Requirements verified:
 *            1. Frame: header=0x57, function_mark=0x01, 400 bytes
 *            2. Pixel distance: uint24 LE in µm, converted to mm
 *            3. Pixel status: 0x00 = valid
 *            4. Checksum: sum of bytes [0..398] mod 256
 *            5. Statistics: min/max/avg/validCount/dispersion
 *            6. Quality gates: MIN_VALID_PIXELS, MAX_PIXEL_DISPERSION
 *            7. Zone mapping matches STM32 safety tiers
 *            8. Stuck detection, warmup, timeout
 *            9. NLink 0x5A configuration commands
 *           10. Single-point frames (0x57 0x00) must be REJECTED
 ****************************************************************************
 */

#include <Arduino.h>
#include <cstdio>
#include <cstring>
#include "sensors/obstacle_sensor.h"

/* ---- Test harness ------------------------------------------------------- */

static int s_tests_run    = 0;
static int s_tests_failed = 0;

#define ASSERT(cond) do {                                                    \
    s_tests_run++;                                                           \
    if (!(cond)) {                                                           \
        printf("FAIL  %s:%d  Assertion: %s\n", __FILE__, __LINE__, #cond);  \
        s_tests_failed++;                                                    \
    }                                                                        \
} while (0)

#define ASSERT_EQ(a, b) do {                                                 \
    s_tests_run++;                                                           \
    if ((a) != (b)) {                                                        \
        printf("FAIL  %s:%d  %s == %ld (expected %ld)\n",                   \
               __FILE__, __LINE__, #a,                                       \
               (long)(a), (long)(b));                                        \
        s_tests_failed++;                                                    \
    }                                                                        \
} while (0)

/* ---- Multi-pixel frame helpers ------------------------------------------ */

static constexpr uint16_t MP_FRAME_LEN = 400;

// Build a multi-pixel frame with all pixels at the same distance (µm)
// and same signal strength.  Status=0 (valid) for all pixels.
static void buildMPFrame(uint8_t* buf, uint32_t distUm, uint16_t signal) {
    buf[0] = 0x57;  // header
    buf[1] = 0x01;  // multi-pixel function mark
    buf[2] = 0xFF;  // reserved
    buf[3] = 0x00;  // sensor_id
    buf[4] = 0; buf[5] = 0; buf[6] = 0; buf[7] = 0;  // system_time
    buf[8] = 64;    // num_pixels

    for (int px = 0; px < 64; px++) {
        uint16_t off = 9 + px * 6;
        buf[off + 0] = (uint8_t)(distUm & 0xFF);
        buf[off + 1] = (uint8_t)((distUm >> 8) & 0xFF);
        buf[off + 2] = (uint8_t)((distUm >> 16) & 0xFF);
        buf[off + 3] = 0x00;   // status = valid
        buf[off + 4] = (uint8_t)(signal & 0xFF);
        buf[off + 5] = (uint8_t)((signal >> 8) & 0xFF);
    }

    // End marker
    for (int i = 393; i < 399; i++) buf[i] = 0xFF;

    // Checksum
    uint8_t sum = 0;
    for (int i = 0; i < 399; i++) sum += buf[i];
    buf[399] = sum;
}

// Build a multi-pixel frame with per-pixel distances (µm array) and status
static void buildMPFrameVarying(uint8_t* buf, const uint32_t* distUm,
                                 const uint8_t* status, uint16_t signal) {
    buf[0] = 0x57;
    buf[1] = 0x01;
    buf[2] = 0xFF;
    buf[3] = 0x00;
    buf[4] = 0; buf[5] = 0; buf[6] = 0; buf[7] = 0;
    buf[8] = 64;

    for (int px = 0; px < 64; px++) {
        uint16_t off = 9 + px * 6;
        buf[off + 0] = (uint8_t)(distUm[px] & 0xFF);
        buf[off + 1] = (uint8_t)((distUm[px] >> 8) & 0xFF);
        buf[off + 2] = (uint8_t)((distUm[px] >> 16) & 0xFF);
        buf[off + 3] = status[px];
        buf[off + 4] = (uint8_t)(signal & 0xFF);
        buf[off + 5] = (uint8_t)((signal >> 8) & 0xFF);
    }

    for (int i = 393; i < 399; i++) buf[i] = 0xFF;

    uint8_t sum = 0;
    for (int i = 0; i < 399; i++) sum += buf[i];
    buf[399] = sum;
}

/* ---- Integration helper: inject a frame and run update() ---------------- */

static obstacle_sensor::Reading injectMPFrameAndUpdate(const uint8_t* frame) {
    g_uart_inject_reset();
    g_test_millis = 0;

    obstacle_sensor::init();

    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);  // flush warmup

    g_uart_inject(frame, MP_FRAME_LEN);
    obstacle_sensor::update(0.0f);

    return obstacle_sensor::getReading();
}

static obstacle_sensor::Reading injectMPFrameWithConfig(
        const uint8_t* frame, const obstacle_sensor::Config& cfg) {
    g_uart_inject_reset();
    g_test_millis = 0;

    obstacle_sensor::init(cfg);

    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);  // flush warmup

    g_uart_inject(frame, MP_FRAME_LEN);
    obstacle_sensor::update(0.0f);

    return obstacle_sensor::getReading();
}

/* ---- Tests -------------------------------------------------------------- */

// Test 1: Frame layout constants
static void test_frame_constants() {
    printf("  test_frame_constants...\n");
    // 9 header + 64*6 pixel + 6 end + 1 checksum = 400
    ASSERT_EQ(9 + 64*6 + 6 + 1, 400);
    ASSERT_EQ(MP_FRAME_LEN, 400);
}

// Test 2: Checksum computation
static void test_checksum_computation() {
    printf("  test_checksum_computation...\n");
    uint8_t frame[MP_FRAME_LEN];
    buildMPFrame(frame, 1500000, 30);

    uint8_t sum = 0;
    for (int i = 0; i < 399; i++) sum += frame[i];
    ASSERT_EQ(sum, frame[399]);
}

// Test 3: Header bytes
static void test_frame_header_bytes() {
    printf("  test_frame_header_bytes...\n");
    uint8_t frame[MP_FRAME_LEN];
    buildMPFrame(frame, 1000000, 20);
    ASSERT_EQ(frame[0], 0x57);
    ASSERT_EQ(frame[1], 0x01);
    ASSERT_EQ(frame[2], 0xFF);
    ASSERT_EQ(frame[8], 64);
}

// Test 4: Config defaults
static void test_config_defaults() {
    printf("  test_config_defaults...\n");
    obstacle_sensor::Config cfg{};
    ASSERT_EQ(cfg.rxPin, 18);
    ASSERT_EQ(cfg.txPin, -1);
    ASSERT_EQ(cfg.baudRate, 921600UL);
    ASSERT_EQ(cfg.rxBufSize, 1024);
    ASSERT_EQ(cfg.minRangeMm, 20);
    ASSERT_EQ(cfg.maxRangeMm, 4000);
    ASSERT_EQ(cfg.warmupMs, 1000UL);
    ASSERT_EQ(cfg.frameTimeoutMs, 500UL);
}

// Test 5: Init resets state
static void test_init_resets_state() {
    printf("  test_init_resets_state...\n");
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.distance_mm, 0);
    ASSERT_EQ(rd.zone, 0);
    ASSERT_EQ(rd.healthy, false);
    ASSERT_EQ(rd.stuck, false);
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::WAITING));
}

// Test 6: Uniform distance — all 64 pixels at 1500 mm
static void test_uniform_distance() {
    printf("  test_uniform_distance...\n");
    uint8_t frame[MP_FRAME_LEN];
    buildMPFrame(frame, 1500000, 30);

    obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
    ASSERT_EQ(rd.distance_mm, 1500);
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.validCount, 64);
    ASSERT_EQ(rd.minDist_mm, 1500);
    ASSERT_EQ(rd.maxDist_mm, 1500);
    ASSERT_EQ(rd.avgDist_mm, 1500);
    ASSERT_EQ(rd.dispersion_mm, 0);
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
}

// Test 7: Minimum pixel distance used
static void test_minimum_distance_used() {
    printf("  test_minimum_distance_used...\n");
    uint32_t dists[64];
    uint8_t stats[64];
    for (int i = 0; i < 64; i++) {
        dists[i] = 2000000;  // 2000 mm
        stats[i] = 0x00;
    }
    dists[42] = 750000;  // 750 mm
    uint8_t frame[MP_FRAME_LEN];
    buildMPFrameVarying(frame, dists, stats, 30);

    obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
    ASSERT_EQ(rd.distance_mm, 750);
    ASSERT_EQ(rd.minDist_mm, 750);
    ASSERT_EQ(rd.maxDist_mm, 2000);
    ASSERT_EQ(rd.validCount, 64);
    ASSERT_EQ(rd.zone, 2);  // 500–1000 mm = warning
}

// Test 8: Invalid pixels skipped
static void test_invalid_pixels_skipped() {
    printf("  test_invalid_pixels_skipped...\n");
    uint32_t dists[64];
    uint8_t stats[64];
    for (int i = 0; i < 64; i++) {
        dists[i] = 500000;   // 500 mm
        stats[i] = 0x09;     // invalid
    }
    // Only 10 pixels valid at 1200 mm
    for (int i = 0; i < 10; i++) {
        dists[i] = 1200000;
        stats[i] = 0x00;
    }
    uint8_t frame[MP_FRAME_LEN];
    buildMPFrameVarying(frame, dists, stats, 30);

    obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
    ASSERT_EQ(rd.distance_mm, 1200);
    ASSERT_EQ(rd.validCount, 10);
    ASSERT_EQ(rd.healthy, true);
}

// Test 9: All pixels invalid → NO_TARGET → minRangeMm
static void test_all_invalid_no_target() {
    printf("  test_all_invalid_no_target...\n");
    uint32_t dists[64];
    uint8_t stats[64];
    for (int i = 0; i < 64; i++) {
        dists[i] = 1000000;
        stats[i] = 0x09;
    }
    uint8_t frame[MP_FRAME_LEN];
    buildMPFrameVarying(frame, dists, stats, 30);

    obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
    ASSERT_EQ(rd.distance_mm, 20);  // minRangeMm
}

// Test 10: All zero distance → NO_TARGET
static void test_all_zero_distance_no_target() {
    printf("  test_all_zero_distance_no_target...\n");
    uint8_t frame[MP_FRAME_LEN];
    buildMPFrame(frame, 0, 30);

    obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
    ASSERT_EQ(rd.distance_mm, 20);  // minRangeMm
}

// Test 11: Bad checksum rejected
static void test_bad_checksum_rejected() {
    printf("  test_bad_checksum_rejected...\n");
    uint8_t frame[MP_FRAME_LEN];
    buildMPFrame(frame, 1500000, 30);
    frame[399] ^= 0xFF;  // Corrupt checksum

    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    g_uart_inject(frame, MP_FRAME_LEN);
    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.distance_mm, 0);  // No valid frame
}

// Test 12: Emergency zone (< 200 mm)
static void test_emergency_zone() {
    printf("  test_emergency_zone...\n");
    uint8_t frame[MP_FRAME_LEN];
    buildMPFrame(frame, 100000, 30);  // 100 mm

    obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
    ASSERT_EQ(rd.distance_mm, 100);
    ASSERT_EQ(rd.zone, 4);
    ASSERT_EQ(rd.healthy, true);
}

// Test 13: Zone boundary tests
static void test_zone_boundaries() {
    printf("  test_zone_boundaries...\n");

    // 200 mm → zone 3 (critical)
    {
        uint8_t frame[MP_FRAME_LEN];
        buildMPFrame(frame, 200000, 30);
        obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
        ASSERT_EQ(rd.distance_mm, 200);
        ASSERT_EQ(rd.zone, 3);
    }
    // 500 mm → zone 2 (warning)
    {
        uint8_t frame[MP_FRAME_LEN];
        buildMPFrame(frame, 500000, 30);
        obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
        ASSERT_EQ(rd.distance_mm, 500);
        ASSERT_EQ(rd.zone, 2);
    }
    // 1000 mm → zone 1 (caution)
    {
        uint8_t frame[MP_FRAME_LEN];
        buildMPFrame(frame, 1000000, 30);
        obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
        ASSERT_EQ(rd.distance_mm, 1000);
        ASSERT_EQ(rd.zone, 1);
    }
    // 1500 mm → zone 0 (normal)
    {
        uint8_t frame[MP_FRAME_LEN];
        buildMPFrame(frame, 1500000, 30);
        obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
        ASSERT_EQ(rd.distance_mm, 1500);
        ASSERT_EQ(rd.zone, 0);
    }
    // < 200 mm → zone 4 (emergency)
    {
        uint8_t frame[MP_FRAME_LEN];
        buildMPFrame(frame, 199000, 30);
        obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
        ASSERT_EQ(rd.distance_mm, 199);
        ASSERT_EQ(rd.zone, 4);
    }
}

// Test 14: Above max range clamped
static void test_above_max_range_clamped() {
    printf("  test_above_max_range_clamped...\n");
    uint8_t frame[MP_FRAME_LEN];
    buildMPFrame(frame, 6000000, 30);  // 6000 mm

    obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
    ASSERT_EQ(rd.distance_mm, 4000);  // maxRangeMm
    ASSERT_EQ(rd.healthy, true);
}

// Test 15: Below min range clamped
static void test_below_min_range_clamped() {
    printf("  test_below_min_range_clamped...\n");
    uint8_t frame[MP_FRAME_LEN];
    buildMPFrame(frame, 5000, 30);  // 5 mm in µm

    obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
    ASSERT_EQ(rd.distance_mm, 20);  // minRangeMm
}

// Test 16: Small distance µm → mm conversion
static void test_small_distance_um_to_mm() {
    printf("  test_small_distance_um_to_mm...\n");
    // 500 µm → 0 mm → set to 1 mm → clamped to minRangeMm (20 mm)
    uint8_t frame[MP_FRAME_LEN];
    buildMPFrame(frame, 500, 30);

    obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
    ASSERT_EQ(rd.distance_mm, 20);  // minRangeMm
}

// Test 17: Warmup period
static void test_warmup_period() {
    printf("  test_warmup_period...\n");
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();

    // During warmup
    g_test_millis = 500;
    obstacle_sensor::update(0.0f);
    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::WAITING));

    // Just past warmup
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // Send valid frame — should now process
    uint8_t frame[MP_FRAME_LEN];
    buildMPFrame(frame, 1000000, 30);
    g_uart_inject(frame, MP_FRAME_LEN);
    obstacle_sensor::update(0.0f);
    rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.distance_mm, 1000);
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
}

// Test 18: Frame timeout
static void test_frame_timeout() {
    printf("  test_frame_timeout...\n");
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();

    // Past warmup
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // No frame, advance past timeout (warmup end + 500 ms)
    g_test_millis = 1700;
    obstacle_sensor::update(0.0f);
    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::INVALID));
    ASSERT_EQ(rd.healthy, false);
}

// Test 19: No frame ever received → INVALID after warmup + timeout
static void test_no_frame_ever_received_timeout() {
    printf("  test_no_frame_ever_received_timeout...\n");
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();

    // During warmup
    g_test_millis = 500;
    obstacle_sensor::update(0.0f);
    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::WAITING));

    // Just past warmup + timeout
    g_test_millis = 1600;
    obstacle_sensor::update(0.0f);
    rd = obstacle_sensor::getReading();
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::INVALID));
}

// Test 20: Single-point frames (0x57 0x00) REJECTED
static void test_single_point_rejected() {
    printf("  test_single_point_rejected...\n");
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // Inject a 7-byte single-point frame
    uint8_t frame[7];
    frame[0] = 0x57;
    frame[1] = 0x00;  // single-point function mark
    frame[2] = 0xE8;  // dist_L (1000)
    frame[3] = 0x03;  // dist_H
    frame[4] = 0x64;  // sig_L
    frame[5] = 0x00;  // sig_H
    uint8_t sum = 0;
    for (int i = 0; i < 6; i++) sum += frame[i];
    frame[6] = sum;

    g_uart_inject(frame, 7);
    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.distance_mm, 0);  // Rejected — no valid frame
}

// Test 21: updateCount increments on each valid frame
static void test_update_count_increments() {
    printf("  test_update_count_increments...\n");
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    uint8_t frame[MP_FRAME_LEN];
    buildMPFrame(frame, 1500000, 30);

    g_uart_inject(frame, MP_FRAME_LEN);
    obstacle_sensor::update(0.0f);
    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.updateCount, 1UL);

    g_uart_inject(frame, MP_FRAME_LEN);
    obstacle_sensor::update(0.0f);
    rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.updateCount, 2UL);
}

// Test 22: Mixed pixel validity
static void test_mixed_validity() {
    printf("  test_mixed_validity...\n");
    uint32_t dists[64];
    uint8_t stats[64];
    for (int i = 0; i < 64; i++) {
        dists[i] = 3000000;  // 3000 mm
        stats[i] = 0x00;     // valid
    }
    stats[0] = 0x01;   // invalid
    stats[1] = 0xFF;   // invalid
    dists[2] = 0;       // zero distance
    dists[32] = 400000; // 400 mm — closest

    uint8_t frame[MP_FRAME_LEN];
    buildMPFrameVarying(frame, dists, stats, 30);

    obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
    ASSERT_EQ(rd.distance_mm, 400);
    ASSERT_EQ(rd.zone, 3);  // 200–500 = critical
    ASSERT_EQ(rd.validCount, 61); // 64 - 2 invalid - 1 zero
}

// Test 23: Dispersion calculation
static void test_dispersion_calculation() {
    printf("  test_dispersion_calculation...\n");
    uint32_t dists[64];
    uint8_t stats[64];
    for (int i = 0; i < 64; i++) {
        dists[i] = 1000000;  // 1000 mm
        stats[i] = 0x00;
    }
    dists[0] = 500000;   // 500 mm
    dists[63] = 2500000;  // 2500 mm

    uint8_t frame[MP_FRAME_LEN];
    buildMPFrameVarying(frame, dists, stats, 30);

    obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
    ASSERT_EQ(rd.minDist_mm, 500);
    ASSERT_EQ(rd.maxDist_mm, 2500);
    ASSERT_EQ(rd.dispersion_mm, 2000);
    ASSERT_EQ(rd.distance_mm, 500);  // min used
}

// Test 24: Too few valid pixels → fallback to minRangeMm
static void test_too_few_pixels() {
    printf("  test_too_few_pixels...\n");
    uint32_t dists[64];
    uint8_t stats[64];
    for (int i = 0; i < 64; i++) {
        dists[i] = 1000000;
        stats[i] = 0x09;  // invalid
    }
    // Only 3 valid pixels (below MIN_VALID_PIXELS = 4)
    for (int i = 0; i < 3; i++) {
        dists[i] = 1000000;
        stats[i] = 0x00;
    }

    uint8_t frame[MP_FRAME_LEN];
    buildMPFrameVarying(frame, dists, stats, 30);

    obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
    ASSERT_EQ(rd.distance_mm, 20);  // minRangeMm (emergency fallback)
}

// Test 25: Exactly MIN_VALID_PIXELS valid → OK
static void test_exactly_min_valid_pixels() {
    printf("  test_exactly_min_valid_pixels...\n");
    uint32_t dists[64];
    uint8_t stats[64];
    for (int i = 0; i < 64; i++) {
        dists[i] = 1000000;
        stats[i] = 0x09;  // invalid
    }
    // Exactly 4 valid pixels
    for (int i = 0; i < 4; i++) {
        dists[i] = 800000;  // 800 mm
        stats[i] = 0x00;
    }

    uint8_t frame[MP_FRAME_LEN];
    buildMPFrameVarying(frame, dists, stats, 30);

    obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
    ASSERT_EQ(rd.distance_mm, 800);
    ASSERT_EQ(rd.validCount, 4);
    ASSERT_EQ(rd.healthy, true);
}

// Test 26: High dispersion (>3000mm) — still reports min distance but marked frame
static void test_high_dispersion() {
    printf("  test_high_dispersion...\n");
    uint32_t dists[64];
    uint8_t stats[64];
    for (int i = 0; i < 64; i++) {
        dists[i] = 1000000;  // 1000 mm
        stats[i] = 0x00;
    }
    dists[0] = 100000;    // 100 mm
    dists[63] = 3500000;  // 3500 mm → dispersion = 3400 > 3000

    uint8_t frame[MP_FRAME_LEN];
    buildMPFrameVarying(frame, dists, stats, 30);

    obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
    // HIGH_DISPERSION: driver uses minDist as conservative safety measure
    ASSERT_EQ(rd.distance_mm, 100);
    ASSERT_EQ(rd.dispersion_mm, 3400);
}

// Test 27: Unknown function mark rejected
static void test_unknown_function_mark_rejected() {
    printf("  test_unknown_function_mark_rejected...\n");
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // Build a frame with function mark 0x02
    uint8_t frame[MP_FRAME_LEN];
    buildMPFrame(frame, 1500000, 30);
    frame[1] = 0x02;  // Unknown function mark
    // Recompute checksum
    uint8_t sum = 0;
    for (int i = 0; i < 399; i++) sum += frame[i];
    frame[399] = sum;

    g_uart_inject(frame, MP_FRAME_LEN);
    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.distance_mm, 0);
}

// Test 28: Wrong pixel count rejected
static void test_wrong_pixel_count_rejected() {
    printf("  test_wrong_pixel_count_rejected...\n");
    uint8_t frame[MP_FRAME_LEN];
    buildMPFrame(frame, 1500000, 30);
    frame[8] = 16;  // Wrong: should be 64
    // Recompute checksum
    uint8_t sum = 0;
    for (int i = 0; i < 399; i++) sum += frame[i];
    frame[399] = sum;

    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    g_uart_inject(frame, MP_FRAME_LEN);
    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.distance_mm, 0);
}

// Test 29: NLink buildCommand
static void test_build_command_range_mode() {
    printf("  test_build_command_range_mode...\n");
    uint8_t buf[16];
    uint8_t payload = 0x01;  // LONG_RANGE
    uint16_t len = obstacle_sensor::buildCommand(
        static_cast<uint8_t>(obstacle_sensor::CmdId::SET_RANGE_MODE),
        &payload, 1, buf, sizeof(buf));

    ASSERT_EQ(len, 5);
    ASSERT_EQ(buf[0], 0x5A);
    ASSERT_EQ(buf[1], 5);
    ASSERT_EQ(buf[2], 0x07);  // SET_RANGE_MODE
    ASSERT_EQ(buf[3], 0x01);  // LONG_RANGE

    uint8_t sum = 0;
    for (int i = 0; i < 4; i++) sum += buf[i];
    ASSERT_EQ(buf[4], sum);
}

// Test 30: NLink buildCommand — save config
static void test_build_command_save_config() {
    printf("  test_build_command_save_config...\n");
    uint8_t buf[16];
    uint16_t len = obstacle_sensor::buildCommand(
        static_cast<uint8_t>(obstacle_sensor::CmdId::SAVE_CONFIG),
        nullptr, 0, buf, sizeof(buf));

    ASSERT_EQ(len, 4);
    ASSERT_EQ(buf[0], 0x5A);
    ASSERT_EQ(buf[1], 4);
    ASSERT_EQ(buf[2], 0x08);

    uint8_t sum = 0;
    for (int i = 0; i < 3; i++) sum += buf[i];
    ASSERT_EQ(buf[3], sum);
}

// Test 31: buildCommand buffer too small
static void test_build_command_buffer_too_small() {
    printf("  test_build_command_buffer_too_small...\n");
    uint8_t buf[3];
    uint8_t payload = 0x01;
    uint16_t len = obstacle_sensor::buildCommand(0x07, &payload, 1, buf, sizeof(buf));
    ASSERT_EQ(len, 0);
}

// Test 32: buildCommand null payload with len > 0 rejected
static void test_build_command_null_payload_rejected() {
    printf("  test_build_command_null_payload_rejected...\n");
    uint8_t buf[16];
    uint16_t len = obstacle_sensor::buildCommand(0x07, nullptr, 1, buf, sizeof(buf));
    ASSERT_EQ(len, 0);
}

// Test 33: buildCommand null payload with len=0 OK
static void test_build_command_null_payload_zero_len_ok() {
    printf("  test_build_command_null_payload_zero_len_ok...\n");
    uint8_t buf[16];
    uint16_t len = obstacle_sensor::buildCommand(0x08, nullptr, 0, buf, sizeof(buf));
    ASSERT_EQ(len, 4);
}

// Test 34: Send command fails without TX pin
static void test_send_command_fails_without_tx_pin() {
    printf("  test_send_command_fails_without_tx_pin...\n");
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::Config cfg;
    cfg.txPin = -1;
    obstacle_sensor::init(cfg);

    bool ok = obstacle_sensor::setRangeMode(obstacle_sensor::RangeMode::LONG_RANGE);
    ASSERT_EQ(ok, false);
}

// Test 35: configureLongRange fails without TX
static void test_configure_long_range_fails_without_tx() {
    printf("  test_configure_long_range_fails_without_tx...\n");
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::Config cfg;
    cfg.txPin = -1;
    obstacle_sensor::init(cfg);

    bool ok = obstacle_sensor::configureLongRange();
    ASSERT_EQ(ok, false);
}

// Test 36: configureLongRange succeeds with TX
static void test_configure_long_range_succeeds_with_tx() {
    printf("  test_configure_long_range_succeeds_with_tx...\n");
    g_uart_inject_reset();
    g_uart_tx_reset();
    g_test_millis = 0;
    obstacle_sensor::Config cfg;
    cfg.txPin = 17;
    obstacle_sensor::init(cfg);

    bool ok = obstacle_sensor::configureLongRange();
    ASSERT_EQ(ok, true);
    ASSERT(g_uart_tx_len > 0);
}

// Test 37: setFrameRate(0) rejected
static void test_set_frame_rate_zero_rejected() {
    printf("  test_set_frame_rate_zero_rejected...\n");
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::Config cfg;
    cfg.txPin = 17;
    obstacle_sensor::init(cfg);

    bool ok = obstacle_sensor::setFrameRate(0);
    ASSERT_EQ(ok, false);
}

// Test 38: Real-world packet from NAssistant
static void test_real_world_packet() {
    printf("  test_real_world_packet...\n");

    const char* hex =
        "57 01 ff 00 5f ba 00 00 40 92 00 13 00 1e 00 d7 33 16 00 22 00 "
        "63 6a 17 00 31 00 fe ef 16 00 27 00 cf d5 17 00 22 00 7c f9 17 "
        "00 20 00 d0 bf 16 00 1d 00 00 e7 18 00 14 00 4e bf 15 00 27 00 "
        "5c 64 17 00 33 00 b4 ca 17 00 3d 00 ff b4 17 00 28 00 03 10 18 "
        "00 2b 00 cf 52 18 00 22 00 0f f5 17 00 22 00 07 d5 17 00 16 00 "
        "f8 2d 17 00 29 00 f0 3f 17 00 2f 00 84 18 18 00 39 00 9c 6e 18 "
        "00 3b 00 9c 1b 17 00 2c 00 ab a3 18 00 1f 00 ca 26 19 00 1e 00 "
        "6f fe 17 00 1c 00 5c 79 17 00 2c 00 88 e3 16 00 34 00 20 09 18 "
        "00 3c 00 d8 7d 18 00 36 00 f6 78 18 00 2d 00 73 bd 18 00 25 00 "
        "7c f3 18 00 20 00 fe 98 18 00 1b 00 63 9c 17 00 25 00 6a 0a 18 "
        "00 2a 00 b8 6c 17 00 3a 00 58 bc 18 00 36 00 84 12 19 00 39 00 "
        "7c 76 18 00 20 00 58 44 19 00 1d 00 fb 7f 19 00 19 00 37 1e 17 "
        "00 22 00 d0 04 18 00 29 00 40 1c 18 00 29 00 13 9b 18 00 31 00 "
        "f4 f2 18 00 35 00 b6 6c 19 00 27 00 b7 50 19 00 22 00 ce b3 19 "
        "00 15 00 07 b5 14 00 22 00 74 0b 18 00 26 00 a0 89 18 00 29 00 "
        "46 de 17 00 27 00 b8 6f 19 00 32 00 6e 5f 19 00 2d 00 70 c3 18 "
        "00 1d 00 c8 ba 13 00 0b 00 ec d0 11 00 1a 00 42 1f 17 00 1e 00 "
        "ae 9d 18 00 27 00 97 e9 18 00 28 00 5a 92 18 00 1e 00 87 a3 19 "
        "00 2e 00 ac e5 19 00 20 00 50 05 1a 00 17 00 ff ff ff ff ff ff 08";

    uint8_t frame[MP_FRAME_LEN];
    int idx = 0;
    const char* p = hex;
    while (*p && idx < MP_FRAME_LEN) {
        while (*p == ' ') p++;
        if (*p == '\0') break;
        unsigned int val = 0;
        for (int d = 0; d < 2 && *p; d++, p++) {
            val <<= 4;
            if (*p >= '0' && *p <= '9') val |= (*p - '0');
            else if (*p >= 'a' && *p <= 'f') val |= (*p - 'a' + 10);
            else if (*p >= 'A' && *p <= 'F') val |= (*p - 'A' + 10);
        }
        frame[idx++] = (uint8_t)val;
    }
    ASSERT_EQ(idx, MP_FRAME_LEN);

    obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
    // Should parse successfully with all valid pixels producing meaningful distance
    ASSERT(rd.distance_mm > 0);
    ASSERT(rd.healthy == true);
    ASSERT(rd.validCount > 0);
    ASSERT(rd.zone <= 4);
}

// Test 39: Resync after corrupt data
static void test_resync_after_corrupt() {
    printf("  test_resync_after_corrupt...\n");
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // Inject garbage followed by a valid frame
    uint8_t garbage[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
    g_uart_inject(garbage, sizeof(garbage));

    uint8_t frame[MP_FRAME_LEN];
    buildMPFrame(frame, 2000000, 30);
    g_uart_inject(frame, MP_FRAME_LEN);

    obstacle_sensor::update(0.0f);
    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.distance_mm, 2000);
    ASSERT_EQ(rd.healthy, true);
}

// Test 40: Buffer overflow protection
static void test_rxbuf_overflow_protection() {
    printf("  test_rxbuf_overflow_protection...\n");
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // Inject a lot of junk (more than rx buffer)
    for (int i = 0; i < 500; i++) {
        uint8_t junk = 0xAA;
        g_uart_inject(&junk, 1);
    }
    obstacle_sensor::update(0.0f);

    // Now inject a valid frame — should still work
    uint8_t frame[MP_FRAME_LEN];
    buildMPFrame(frame, 1500000, 30);
    g_uart_inject(frame, MP_FRAME_LEN);
    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.distance_mm, 1500);
}

// Test 41: Average distance calculation
static void test_avg_distance() {
    printf("  test_avg_distance...\n");
    uint32_t dists[64];
    uint8_t stats[64];
    for (int i = 0; i < 64; i++) {
        stats[i] = 0x00;
    }
    // Half at 1000mm, half at 2000mm → avg = 1500mm
    for (int i = 0; i < 32; i++) dists[i] = 1000000;
    for (int i = 32; i < 64; i++) dists[i] = 2000000;

    uint8_t frame[MP_FRAME_LEN];
    buildMPFrameVarying(frame, dists, stats, 30);

    obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
    ASSERT_EQ(rd.avgDist_mm, 1500);
    ASSERT_EQ(rd.minDist_mm, 1000);
    ASSERT_EQ(rd.maxDist_mm, 2000);
    ASSERT_EQ(rd.validCount, 64);
}

// Test 42: Stuck detection
static void test_stuck_detection() {
    printf("  test_stuck_detection...\n");
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    uint8_t frame[MP_FRAME_LEN];
    buildMPFrame(frame, 1000000, 30);

    // Send same distance repeatedly at speed > 1 km/h
    // Reset inject buffer before each frame to avoid overflow (4096-byte buffer)
    for (int i = 0; i < 15; i++) {
        g_uart_inject_reset();
        g_uart_inject(frame, MP_FRAME_LEN);
        g_test_millis += 100;
        obstacle_sensor::update(5.0f);
    }

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.stuck, true);
    ASSERT_EQ(rd.healthy, false);
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::INVALID));
}

// Test 43: Stuck clears when distance changes
static void test_stuck_clears() {
    printf("  test_stuck_clears...\n");
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    uint8_t frame[MP_FRAME_LEN];
    buildMPFrame(frame, 1000000, 30);

    // Get stuck
    for (int i = 0; i < 15; i++) {
        g_uart_inject_reset();
        g_uart_inject(frame, MP_FRAME_LEN);
        g_test_millis += 100;
        obstacle_sensor::update(5.0f);
    }
    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.stuck, true);

    // Now change distance significantly
    buildMPFrame(frame, 2000000, 30);
    g_uart_inject_reset();
    g_uart_inject(frame, MP_FRAME_LEN);
    g_test_millis += 100;
    obstacle_sensor::update(5.0f);

    rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.stuck, false);
    ASSERT_EQ(rd.healthy, true);
}

// Test 44: Stuck not triggered at low speed
static void test_stuck_not_at_low_speed() {
    printf("  test_stuck_not_at_low_speed...\n");
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    uint8_t frame[MP_FRAME_LEN];
    buildMPFrame(frame, 1000000, 30);

    for (int i = 0; i < 15; i++) {
        g_uart_inject_reset();
        g_uart_inject(frame, MP_FRAME_LEN);
        g_test_millis += 100;
        obstacle_sensor::update(0.5f);  // Below threshold
    }

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.stuck, false);
    ASSERT_EQ(rd.healthy, true);
}

// Test 45: Multiple queued frames — last one wins
static void test_multiple_frames_last_wins() {
    printf("  test_multiple_frames_last_wins...\n");
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    uint8_t frame1[MP_FRAME_LEN];
    buildMPFrame(frame1, 1000000, 30);
    uint8_t frame2[MP_FRAME_LEN];
    buildMPFrame(frame2, 2000000, 30);

    g_uart_inject(frame1, MP_FRAME_LEN);
    g_uart_inject(frame2, MP_FRAME_LEN);
    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.distance_mm, 2000);
}

// Test 46: Custom config — different range
static void test_custom_config_range() {
    printf("  test_custom_config_range...\n");
    obstacle_sensor::Config cfg;
    cfg.maxRangeMm = 2000;
    cfg.minRangeMm = 50;

    uint8_t frame[MP_FRAME_LEN];
    buildMPFrame(frame, 3000000, 30);  // 3000 mm > maxRangeMm

    obstacle_sensor::Reading rd = injectMPFrameWithConfig(frame, cfg);
    ASSERT_EQ(rd.distance_mm, 2000);  // Clamped to maxRangeMm
}

// Test 47: 4000 mm distance (max range)
static void test_4000mm_max_range() {
    printf("  test_4000mm_max_range...\n");
    uint8_t frame[MP_FRAME_LEN];
    buildMPFrame(frame, 4000000, 30);

    obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
    ASSERT_EQ(rd.distance_mm, 4000);
    ASSERT_EQ(rd.zone, 0);
    ASSERT_EQ(rd.healthy, true);
}

// Test 48: 20 mm distance (min range)
static void test_20mm_min_range() {
    printf("  test_20mm_min_range...\n");
    uint8_t frame[MP_FRAME_LEN];
    buildMPFrame(frame, 20000, 30);  // 20 mm = 20000 µm

    obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
    ASSERT_EQ(rd.distance_mm, 20);
    ASSERT_EQ(rd.zone, 4);  // Emergency
}

// Test 49: Pixel data at various distances
static void test_pixel_various_distances() {
    printf("  test_pixel_various_distances...\n");
    uint32_t dists[64];
    uint8_t stats[64];
    for (int i = 0; i < 64; i++) {
        stats[i] = 0x00;
    }
    // Simulate real scene: mostly 1.5m, one corner at 0.3m
    for (int i = 0; i < 64; i++) dists[i] = 1500000;
    dists[0]  = 300000;   // 0.3m
    dists[7]  = 400000;   // 0.4m
    dists[56] = 1800000;  // 1.8m
    dists[63] = 2000000;  // 2.0m

    uint8_t frame[MP_FRAME_LEN];
    buildMPFrameVarying(frame, dists, stats, 30);

    obstacle_sensor::Reading rd = injectMPFrameAndUpdate(frame);
    ASSERT_EQ(rd.distance_mm, 300);   // min
    ASSERT_EQ(rd.maxDist_mm, 2000);   // max
    ASSERT_EQ(rd.validCount, 64);
    ASSERT_EQ(rd.zone, 3);            // 200–500 = critical
}

/* ---- Main --------------------------------------------------------------- */

int main() {
    printf("=== test_obstacle_sensor (TOFSense-M 8x8 multi-pixel only) ===\n");

    test_frame_constants();
    test_checksum_computation();
    test_frame_header_bytes();
    test_config_defaults();
    test_init_resets_state();
    test_uniform_distance();
    test_minimum_distance_used();
    test_invalid_pixels_skipped();
    test_all_invalid_no_target();
    test_all_zero_distance_no_target();
    test_bad_checksum_rejected();
    test_emergency_zone();
    test_zone_boundaries();
    test_above_max_range_clamped();
    test_below_min_range_clamped();
    test_small_distance_um_to_mm();
    test_warmup_period();
    test_frame_timeout();
    test_no_frame_ever_received_timeout();
    test_single_point_rejected();
    test_update_count_increments();
    test_mixed_validity();
    test_dispersion_calculation();
    test_too_few_pixels();
    test_exactly_min_valid_pixels();
    test_high_dispersion();
    test_unknown_function_mark_rejected();
    test_wrong_pixel_count_rejected();
    test_build_command_range_mode();
    test_build_command_save_config();
    test_build_command_buffer_too_small();
    test_build_command_null_payload_rejected();
    test_build_command_null_payload_zero_len_ok();
    test_send_command_fails_without_tx_pin();
    test_configure_long_range_fails_without_tx();
    test_configure_long_range_succeeds_with_tx();
    test_set_frame_rate_zero_rejected();
    test_real_world_packet();
    test_resync_after_corrupt();
    test_rxbuf_overflow_protection();
    test_avg_distance();
    test_stuck_detection();
    test_stuck_clears();
    test_stuck_not_at_low_speed();
    test_multiple_frames_last_wins();
    test_custom_config_range();
    test_4000mm_max_range();
    test_20mm_min_range();
    test_pixel_various_distances();

    printf("\n%d tests run, %d failed\n", s_tests_run, s_tests_failed);
    return s_tests_failed > 0 ? 1 : 0;
}
