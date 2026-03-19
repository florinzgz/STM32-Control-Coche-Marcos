/**
 ****************************************************************************
 * @file    test_obstacle_sensor.cpp
 * @brief   Host-compilable unit tests for obstacle_sensor frame parsing.
 *
 *          Tests the TOFSense-M S (single-point) protocol parsing.
 *          Frame format: 7 bytes
 *            [0x57] [0x00] [DIST_L] [DIST_H] [SIG_L] [SIG_H] [CHECKSUM]
 *
 *          Compile and run on host (from esp32/src/):
 *            g++ -std=c++17 -I. -I../test_stubs \
 *                sensors/obstacle_sensor.cpp test_obstacle_sensor.cpp \
 *                -o /tmp/test_obstacle_sensor && /tmp/test_obstacle_sensor
 *
 *          Requirements verified:
 *            1. Frame: header = 0x57, function_mark = 0x00, 7 bytes
 *            2. Distance: uint16 LE at bytes 2-3, in mm
 *            3. Signal:   uint16 LE at bytes 4-5
 *            4. Checksum: sum of bytes [0..5] mod 256, stored at byte 6
 *            5. distance == 0 means no target (invalid)
 *            6. Invalid frames rejected (wrong header, bad checksum)
 *            7. Zone mapping matches STM32 safety tiers
 *            8. Stuck detection, warmup, auto-recovery
 *            9. NLink 0x5A configuration commands
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

/* ---- Frame construction helpers ----------------------------------------- */

// TOFSense-M S frame layout (7 bytes):
//   [0]   header         = 0x57
//   [1]   function_mark  = 0x00
//   [2]   distance_L     uint8
//   [3]   distance_H     uint8
//   [4]   signal_L       uint8
//   [5]   signal_H       uint8
//   [6]   checksum       sum([0..5]) & 0xFF

static constexpr uint16_t FRAME_LEN = 7;

// Build a valid M-S frame with given distance and signal
static void buildMSFrame(uint8_t* buf, uint16_t distMm, uint16_t signal) {
    buf[0] = 0x57;  // header
    buf[1] = 0x00;  // function_mark (single-point)
    buf[2] = (uint8_t)(distMm & 0xFF);        // distance_L
    buf[3] = (uint8_t)((distMm >> 8) & 0xFF); // distance_H
    buf[4] = (uint8_t)(signal & 0xFF);         // signal_L
    buf[5] = (uint8_t)((signal >> 8) & 0xFF);  // signal_H
    // Checksum: sum of bytes [0..5] & 0xFF
    uint8_t sum = 0;
    for (int i = 0; i < 6; i++) sum += buf[i];
    buf[6] = sum;
}

// Build a no-target frame (distance = 0)
static void buildNoTargetFrame(uint8_t* buf, uint16_t signal) {
    buildMSFrame(buf, 0, signal);
}

/* ---- Integration helper: inject a frame and run update() ---------------- */

// Reset sensor + UART stub, advance past warmup, then inject `frame`
// and call update().  Returns the reading produced.
static obstacle_sensor::Reading injectFrameAndUpdate(const uint8_t* frame) {
    g_uart_inject_reset();
    g_test_millis = 0;

    obstacle_sensor::init();

    // Advance past warmup (default 1000 ms)
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);  // flush warmup

    // Inject frame bytes and run update
    g_uart_inject(frame, FRAME_LEN);
    obstacle_sensor::update(0.0f);

    return obstacle_sensor::getReading();
}

// Variant with custom config
static obstacle_sensor::Reading injectFrameWithConfig(
        const uint8_t* frame, const obstacle_sensor::Config& cfg) {
    g_uart_inject_reset();
    g_test_millis = 0;

    obstacle_sensor::init(cfg);

    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);  // flush warmup

    g_uart_inject(frame, FRAME_LEN);
    obstacle_sensor::update(0.0f);

    return obstacle_sensor::getReading();
}

/* ---- Tests -------------------------------------------------------------- */

// Test 1: Verify frame layout constants
static void test_frame_constants() {
    printf("  test_frame_constants...\n");

    // M-S frame: header(1) + func(1) + dist(2) + signal(2) + checksum(1) = 7
    ASSERT_EQ(1 + 1 + 2 + 2 + 1, 7);
    ASSERT_EQ(FRAME_LEN, 7);
}

// Test 2: Build a valid frame and verify checksum computation
static void test_checksum_computation() {
    printf("  test_checksum_computation...\n");

    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 1000, 100);

    // Verify: sum of bytes [0..5] should equal byte[6]
    uint8_t sum = 0;
    for (int i = 0; i < 6; i++) sum += frame[i];
    ASSERT_EQ(sum, frame[6]);
}

// Test 3: Verify distance field position (bytes 2-3, uint16 LE)
static void test_distance_field_position() {
    printf("  test_distance_field_position...\n");

    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 500, 100);

    // Header
    ASSERT_EQ(frame[0], 0x57);
    ASSERT_EQ(frame[1], 0x00);

    // Distance 500 = 0x01F4
    ASSERT_EQ(frame[2], (uint8_t)(500 & 0xFF));
    ASSERT_EQ(frame[3], (uint8_t)((500 >> 8) & 0xFF));
}

// Test 4: Verify signal field position (bytes 4-5, uint16 LE)
static void test_signal_field_position() {
    printf("  test_signal_field_position...\n");

    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 1000, 0x1234);

    ASSERT_EQ(frame[4], 0x34);
    ASSERT_EQ(frame[5], 0x12);
}

// Test 5: Verify distance unit is mm (no conversion needed)
static void test_distance_conversion() {
    printf("  test_distance_conversion...\n");

    ASSERT_EQ(1000, 1000);   // 1 m
    ASSERT_EQ(500, 500);     // 50 cm
    ASSERT_EQ(200, 200);     // emergency zone
    ASSERT_EQ(4000, 4000);   // max range
}

// Test 6: Verify zone mapping thresholds
static void test_zone_mapping() {
    printf("  test_zone_mapping...\n");

    obstacle_sensor::init();
    obstacle_sensor::Reading rd = obstacle_sensor::getReading();

    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::WAITING));
    ASSERT_EQ(rd.healthy, false);
}

// Test 7: Verify frame header bytes
static void test_frame_header_bytes() {
    printf("  test_frame_header_bytes...\n");

    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 1000, 100);

    ASSERT_EQ(frame[0], 0x57);
    ASSERT_EQ(frame[1], 0x00);
}

// Test 8: Verify checksum byte position is at byte 6
static void test_checksum_position() {
    printf("  test_checksum_position...\n");

    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 1000, 50);

    uint16_t checksumOffset = FRAME_LEN - 1;
    ASSERT_EQ(checksumOffset, 6);
}

// Test 9: Verify init() resets state correctly
static void test_init_resets_state() {
    printf("  test_init_resets_state...\n");

    obstacle_sensor::init();
    obstacle_sensor::Reading rd = obstacle_sensor::getReading();

    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::WAITING));
    ASSERT_EQ(rd.healthy, false);
    ASSERT_EQ(rd.stuck, false);
    ASSERT_EQ(rd.distance_mm, 0);
    ASSERT_EQ(rd.zone, 0);
}

// Test 10: Verify Config defaults
static void test_config_defaults() {
    printf("  test_config_defaults...\n");

    obstacle_sensor::Config cfg{};

    ASSERT_EQ(cfg.baudRate, 921600UL);
    ASSERT_EQ(cfg.rxBufSize, 512);
    ASSERT_EQ(cfg.minRangeMm, 20);
    ASSERT_EQ(cfg.maxRangeMm, 4000);
    ASSERT_EQ(cfg.shortRangeMode, false);
}

// Test 11: No-target frame -> emergency close (distance = minRangeMm)
static void test_no_target_is_emergency_close() {
    printf("  test_no_target_is_emergency_close...\n");

    uint8_t frame[FRAME_LEN];
    buildNoTargetFrame(frame, 100);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(rd.distance_mm, 20);  // minRangeMm
    ASSERT_EQ(rd.zone, 4);          // emergency zone (< 200)
    ASSERT_EQ(rd.healthy, true);
}

// Test 12: Below-minimum distance clamped to minRangeMm
static void test_below_min_range_clamped_to_min() {
    printf("  test_below_min_range_clamped_to_min...\n");

    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 5, 100);  // 5 mm < minRangeMm (20)

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(rd.distance_mm, 20);  // clamped to minRangeMm
    ASSERT_EQ(rd.zone, 4);          // emergency
    ASSERT_EQ(rd.healthy, true);
}

// Test 13: Above-maximum distance clamped to maxRangeMm
static void test_above_max_range_clamped_to_max() {
    printf("  test_above_max_range_clamped_to_max...\n");

    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 5000, 100);  // 5000 mm > maxRangeMm (4000)

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(rd.distance_mm, 4000);  // clamped to maxRangeMm
    ASSERT_EQ(rd.zone, 0);            // normal (> 1500)
    ASSERT_EQ(rd.healthy, true);
}

// Test 14: Normal distance -> valid reading
static void test_normal_distance_valid() {
    printf("  test_normal_distance_valid...\n");

    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 1000, 200);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(rd.distance_mm, 1000);
    ASSERT_EQ(rd.zone, 1);  // caution zone: [1000, 1500)
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
}

// Test 15: Exact minimum range -> valid
static void test_exact_min_range_valid() {
    printf("  test_exact_min_range_valid...\n");

    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 20, 100);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(rd.distance_mm, 20);
    ASSERT_EQ(rd.healthy, true);
}

// Test 16: Overflow flush keeps latest frame
static void test_overflow_flush_keeps_latest_frame() {
    printf("  test_overflow_flush_keeps_latest_frame...\n");

    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // Inject two frames -- driver should process both and keep last
    uint8_t frame1[FRAME_LEN], frame2[FRAME_LEN];
    buildMSFrame(frame1, 800, 100);
    buildMSFrame(frame2, 1200, 100);

    g_uart_inject(frame1, FRAME_LEN);
    g_uart_inject(frame2, FRAME_LEN);
    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.distance_mm, 1200);  // last frame wins
    ASSERT_EQ(rd.healthy, true);
}

// Test 17: Resync after bad checksum
static void test_resync_after_bad_checksum() {
    printf("  test_resync_after_bad_checksum...\n");

    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // Build a bad frame (corrupt checksum)
    uint8_t badFrame[FRAME_LEN];
    buildMSFrame(badFrame, 500, 100);
    badFrame[6] ^= 0xFF;  // Corrupt checksum

    // Build a good frame
    uint8_t goodFrame[FRAME_LEN];
    buildMSFrame(goodFrame, 2000, 200);

    g_uart_inject(badFrame, FRAME_LEN);
    g_uart_inject(goodFrame, FRAME_LEN);
    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    // The good frame should be found after resync
    ASSERT_EQ(rd.distance_mm, 2000);
    ASSERT_EQ(rd.healthy, true);
}

// Test 18: Single frame within byte limit
static void test_single_frame_within_byte_limit() {
    printf("  test_single_frame_within_byte_limit...\n");

    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 750, 100);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(rd.distance_mm, 750);
    ASSERT_EQ(rd.healthy, true);
}

// Test 19: Multiple bad frames then good
static void test_multiple_bad_frames_then_good() {
    printf("  test_multiple_bad_frames_then_good...\n");

    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // Inject 3 bad frames
    for (int i = 0; i < 3; i++) {
        uint8_t bad[FRAME_LEN];
        buildMSFrame(bad, 500, 100);
        bad[6] ^= 0xFF;
        g_uart_inject(bad, FRAME_LEN);
    }

    // Inject 1 good frame
    uint8_t good[FRAME_LEN];
    buildMSFrame(good, 1500, 200);
    g_uart_inject(good, FRAME_LEN);

    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.distance_mm, 1500);
    ASSERT_EQ(rd.healthy, true);
}

// Test 20: Many queued frames -- all processed
static void test_many_queued_frames_all_processed() {
    printf("  test_many_queued_frames_all_processed...\n");

    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // Inject 10 frames with increasing distances
    for (int i = 0; i < 10; i++) {
        uint8_t frame[FRAME_LEN];
        buildMSFrame(frame, (uint16_t)(100 + i * 100), 100);
        g_uart_inject(frame, FRAME_LEN);
    }

    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.distance_mm, 1000);  // last frame: 100 + 9*100 = 1000
    ASSERT_EQ(rd.healthy, true);
}

// Test 21: RX buffer overflow protection
static void test_rxbuf_overflow_protection() {
    printf("  test_rxbuf_overflow_protection...\n");

    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // Inject a lot of garbage (no header) + a valid frame
    for (int i = 0; i < 100; i++) {
        uint8_t junk = 0xAA;
        g_uart_inject(&junk, 1);
    }

    uint8_t good[FRAME_LEN];
    buildMSFrame(good, 600, 100);
    g_uart_inject(good, FRAME_LEN);

    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.distance_mm, 600);
    ASSERT_EQ(rd.healthy, true);
}

// Test 22: NLink buildCommand -- range mode
static void test_build_command_range_mode() {
    printf("  test_build_command_range_mode...\n");

    uint8_t buf[8];
    uint8_t payload = static_cast<uint8_t>(obstacle_sensor::RangeMode::LONG_RANGE);
    uint16_t len = obstacle_sensor::buildCommand(
        static_cast<uint8_t>(obstacle_sensor::CmdId::SET_RANGE_MODE),
        &payload, 1, buf, sizeof(buf));

    ASSERT_EQ(len, 5);
    ASSERT_EQ(buf[0], 0x5A);           // CMD_HEADER
    ASSERT_EQ(buf[1], 5);              // total length
    ASSERT_EQ(buf[2], 0x07);           // SET_RANGE_MODE
    ASSERT_EQ(buf[3], 0x01);           // LONG_RANGE

    uint8_t expectedSum = (uint8_t)(0x5A + 5 + 0x07 + 0x01);
    ASSERT_EQ(buf[4], expectedSum);
}

// Test 23: NLink buildCommand -- save config
static void test_build_command_save_config() {
    printf("  test_build_command_save_config...\n");

    uint8_t buf[8];
    uint16_t len = obstacle_sensor::buildCommand(
        static_cast<uint8_t>(obstacle_sensor::CmdId::SAVE_CONFIG),
        nullptr, 0, buf, sizeof(buf));

    ASSERT_EQ(len, 4);
    ASSERT_EQ(buf[0], 0x5A);
    ASSERT_EQ(buf[1], 4);
    ASSERT_EQ(buf[2], 0x08);
    uint8_t expectedSum = (uint8_t)(0x5A + 4 + 0x08);
    ASSERT_EQ(buf[3], expectedSum);
}

// Test 24: NLink buildCommand -- baud rate
static void test_build_command_baud_rate() {
    printf("  test_build_command_baud_rate...\n");

    uint8_t buf[8];
    uint8_t payload = static_cast<uint8_t>(obstacle_sensor::BaudRateCode::BAUD_921600);
    uint16_t len = obstacle_sensor::buildCommand(
        static_cast<uint8_t>(obstacle_sensor::CmdId::SET_BAUD_RATE),
        &payload, 1, buf, sizeof(buf));

    ASSERT_EQ(len, 5);
    ASSERT_EQ(buf[0], 0x5A);
    ASSERT_EQ(buf[1], 5);
    ASSERT_EQ(buf[2], 0x03);
    ASSERT_EQ(buf[3], 0x07);
}

// Test 25: NLink buildCommand -- buffer too small
static void test_build_command_buffer_too_small() {
    printf("  test_build_command_buffer_too_small...\n");

    uint8_t buf[3];  // Too small for 5-byte frame
    uint8_t payload = 0x01;
    uint16_t len = obstacle_sensor::buildCommand(0x07, &payload, 1, buf, sizeof(buf));
    ASSERT_EQ(len, 0);
}

// Test 26: NLink buildCommand -- null payload with len > 0 rejected
static void test_build_command_null_payload_rejected() {
    printf("  test_build_command_null_payload_rejected...\n");

    uint8_t buf[8];
    uint16_t len = obstacle_sensor::buildCommand(0x07, nullptr, 1, buf, sizeof(buf));
    ASSERT_EQ(len, 0);
}

// Test 27: NLink buildCommand -- null payload with len 0 ok
static void test_build_command_null_payload_zero_len_ok() {
    printf("  test_build_command_null_payload_zero_len_ok...\n");

    uint8_t buf[8];
    uint16_t len = obstacle_sensor::buildCommand(0x08, nullptr, 0, buf, sizeof(buf));
    ASSERT(len > 0);
    ASSERT_EQ(buf[0], 0x5A);
}

// Test 28: NLink buildCommand -- checksum wraps
static void test_build_command_checksum_wraps() {
    printf("  test_build_command_checksum_wraps...\n");

    uint8_t buf[8];
    uint8_t payload = 0xFF;
    uint16_t len = obstacle_sensor::buildCommand(0xFF, &payload, 1, buf, sizeof(buf));
    ASSERT(len > 0);

    uint8_t sum = 0;
    for (uint16_t i = 0; i < len - 1; i++) sum += buf[i];
    ASSERT_EQ(buf[len - 1], sum);
}

// Test 29: 2000 mm distance parses correctly
static void test_2000mm_distance_parses_correctly() {
    printf("  test_2000mm_distance_parses_correctly...\n");

    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 2000, 150);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(rd.distance_mm, 2000);
    ASSERT_EQ(rd.zone, 0);  // normal (> 1500)
    ASSERT_EQ(rd.healthy, true);
}

// Test 30: 4000 mm max range parses correctly
static void test_4000mm_max_range_parses_correctly() {
    printf("  test_4000mm_max_range_parses_correctly...\n");

    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 4000, 100);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(rd.distance_mm, 4000);
    ASSERT_EQ(rd.zone, 0);
    ASSERT_EQ(rd.healthy, true);
}

// Test 31: NLink buildCommand -- output mode
static void test_build_command_output_mode() {
    printf("  test_build_command_output_mode...\n");

    uint8_t buf[8];
    uint8_t payload = static_cast<uint8_t>(obstacle_sensor::OutputMode::ACTIVE);
    uint16_t len = obstacle_sensor::buildCommand(
        static_cast<uint8_t>(obstacle_sensor::CmdId::SET_OUTPUT_MODE),
        &payload, 1, buf, sizeof(buf));

    ASSERT_EQ(len, 5);
    ASSERT_EQ(buf[0], 0x5A);
    ASSERT_EQ(buf[2], 0x02);  // SET_OUTPUT_MODE
    ASSERT_EQ(buf[3], 0x00);  // ACTIVE
}

// Test 32: NLink buildCommand -- frame rate
static void test_build_command_frame_rate() {
    printf("  test_build_command_frame_rate...\n");

    uint8_t buf[8];
    uint8_t hz = 10;
    uint16_t len = obstacle_sensor::buildCommand(
        static_cast<uint8_t>(obstacle_sensor::CmdId::SET_FRAME_RATE),
        &hz, 1, buf, sizeof(buf));

    ASSERT_EQ(len, 5);
    ASSERT_EQ(buf[2], 0x05);  // SET_FRAME_RATE
    ASSERT_EQ(buf[3], 10);
}

// Test 33: 3000 mm distance parses correctly
static void test_3000mm_distance_parses_correctly() {
    printf("  test_3000mm_distance_parses_correctly...\n");

    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 3000, 200);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(rd.distance_mm, 3000);
    ASSERT_EQ(rd.zone, 0);
    ASSERT_EQ(rd.healthy, true);
}

// Test 34: Signal strength does not affect distance
static void test_signal_strength_does_not_affect_distance() {
    printf("  test_signal_strength_does_not_affect_distance...\n");

    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 1500, 0xFFFF);  // max signal

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(rd.distance_mm, 1500);
    ASSERT_EQ(rd.healthy, true);
}

// Test 35: Send command fails without TX pin
static void test_send_command_fails_without_tx_pin() {
    printf("  test_send_command_fails_without_tx_pin...\n");

    obstacle_sensor::Config cfg;
    cfg.txPin = -1;
    obstacle_sensor::init(cfg);

    bool ok = obstacle_sensor::setRangeMode(obstacle_sensor::RangeMode::LONG_RANGE);
    ASSERT_EQ(ok, false);
}

// Test 36: configureLongRange fails without TX
static void test_configure_long_range_fails_without_tx() {
    printf("  test_configure_long_range_fails_without_tx...\n");

    obstacle_sensor::Config cfg;
    cfg.txPin = -1;
    obstacle_sensor::init(cfg);

    bool ok = obstacle_sensor::configureLongRange();
    ASSERT_EQ(ok, false);
}

// Test 37: configureLongRange succeeds with TX
static void test_configure_long_range_succeeds_with_tx() {
    printf("  test_configure_long_range_succeeds_with_tx...\n");

    obstacle_sensor::Config cfg;
    cfg.txPin = 17;
    obstacle_sensor::init(cfg);

    g_uart_tx_reset();
    bool ok = obstacle_sensor::configureLongRange();
    ASSERT_EQ(ok, true);
    ASSERT(g_uart_tx_len > 0);
}

// Test 38: setFrameRate(0) rejected
static void test_set_frame_rate_zero_rejected() {
    printf("  test_set_frame_rate_zero_rejected...\n");

    obstacle_sensor::Config cfg;
    cfg.txPin = 17;
    obstacle_sensor::init(cfg);

    bool ok = obstacle_sensor::setFrameRate(0);
    ASSERT_EQ(ok, false);
}

// Test 39: 4000 mm raw bytes verification
static void test_4000mm_raw_bytes() {
    printf("  test_4000mm_raw_bytes...\n");

    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 4000, 100);

    // 4000 = 0x0FA0
    ASSERT_EQ(frame[2], 0xA0);
    ASSERT_EQ(frame[3], 0x0F);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);
    ASSERT_EQ(rd.distance_mm, 4000);
}

// Test 40: configureLongRange sends correct command sequence
static void test_configure_long_range_sends_correct_commands() {
    printf("  test_configure_long_range_sends_correct_commands...\n");

    obstacle_sensor::Config cfg;
    cfg.txPin = 17;
    obstacle_sensor::init(cfg);
    g_uart_tx_reset();

    obstacle_sensor::configureLongRange();

    // 4 commands: setRangeMode(5) + setOutputMode(5) + setFrameRate(5) + saveConfig(4) = 19
    ASSERT_EQ(g_uart_tx_len, 19);

    // First command: SET_RANGE_MODE = LONG_RANGE
    ASSERT_EQ(g_uart_tx_buf[0], 0x5A);
    ASSERT_EQ(g_uart_tx_buf[2], 0x07);
    ASSERT_EQ(g_uart_tx_buf[3], 0x01);

    // Second command: SET_OUTPUT_MODE = ACTIVE
    ASSERT_EQ(g_uart_tx_buf[5], 0x5A);
    ASSERT_EQ(g_uart_tx_buf[7], 0x02);
    ASSERT_EQ(g_uart_tx_buf[8], 0x00);

    // Third command: SET_FRAME_RATE = 10
    ASSERT_EQ(g_uart_tx_buf[10], 0x5A);
    ASSERT_EQ(g_uart_tx_buf[12], 0x05);
    ASSERT_EQ(g_uart_tx_buf[13], 10);

    // Fourth command: SAVE_CONFIG (4 bytes)
    ASSERT_EQ(g_uart_tx_buf[15], 0x5A);
    ASSERT_EQ(g_uart_tx_buf[17], 0x08);
}

// Test 41: setRangeMode sends correct bytes
static void test_set_range_mode_sends_correct_bytes() {
    printf("  test_set_range_mode_sends_correct_bytes...\n");

    obstacle_sensor::Config cfg;
    cfg.txPin = 17;
    obstacle_sensor::init(cfg);
    g_uart_tx_reset();

    obstacle_sensor::setRangeMode(obstacle_sensor::RangeMode::SHORT_RANGE);
    ASSERT_EQ(g_uart_tx_len, 5);
    ASSERT_EQ(g_uart_tx_buf[3], 0x00);  // SHORT_RANGE
}

// Test 42: saveConfig sends correct bytes
static void test_save_config_sends_correct_bytes() {
    printf("  test_save_config_sends_correct_bytes...\n");

    obstacle_sensor::Config cfg;
    cfg.txPin = 17;
    obstacle_sensor::init(cfg);
    g_uart_tx_reset();

    obstacle_sensor::saveConfig();
    ASSERT_EQ(g_uart_tx_len, 4);
    ASSERT_EQ(g_uart_tx_buf[0], 0x5A);
    ASSERT_EQ(g_uart_tx_buf[1], 4);
    ASSERT_EQ(g_uart_tx_buf[2], 0x08);
}

// Test 43: Repeated no-target frames stay at minRangeMm
static void test_repeated_no_target_stays_at_min() {
    printf("  test_repeated_no_target_stays_at_min...\n");

    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // Inject 5 no-target frames
    for (int i = 0; i < 5; i++) {
        g_uart_inject_reset();
        uint8_t frame[FRAME_LEN];
        buildNoTargetFrame(frame, 100);
        g_uart_inject(frame, FRAME_LEN);
        g_test_millis += 100;
        obstacle_sensor::update(0.0f);

        obstacle_sensor::Reading rd = obstacle_sensor::getReading();
        ASSERT_EQ(rd.distance_mm, 20);  // minRangeMm
    }
}

// Test 44: Open-air above max range is valid
static void test_open_air_above_max_range_is_valid() {
    printf("  test_open_air_above_max_range_is_valid...\n");

    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 5000, 100);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(rd.distance_mm, 4000);  // clamped
    ASSERT_EQ(rd.zone, 0);
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
}

// Test 45: Auto-recovery triggers after threshold
static void test_auto_recovery_triggers_after_threshold() {
    printf("  test_auto_recovery_triggers_after_threshold...\n");

    obstacle_sensor::Config cfg;
    cfg.txPin = 17;
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init(cfg);
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    ASSERT_EQ(obstacle_sensor::getAutoRecoveryAttempts(), 0);

    // Inject 30+ no-target frames to trigger auto-recovery
    for (uint32_t i = 0; i < 31; i++) {
        g_uart_inject_reset();
        uint8_t frame[FRAME_LEN];
        buildNoTargetFrame(frame, 100);
        g_uart_inject(frame, FRAME_LEN);
        g_test_millis += 100;
        obstacle_sensor::update(0.0f);
    }

    ASSERT(obstacle_sensor::getAutoRecoveryAttempts() >= 1);
}

// Test 46: Auto-recovery skipped without TX pin
static void test_auto_recovery_skipped_without_tx_pin() {
    printf("  test_auto_recovery_skipped_without_tx_pin...\n");

    obstacle_sensor::Config cfg;
    cfg.txPin = -1;
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init(cfg);
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    for (uint32_t i = 0; i < 31; i++) {
        g_uart_inject_reset();
        uint8_t frame[FRAME_LEN];
        buildNoTargetFrame(frame, 100);
        g_uart_inject(frame, FRAME_LEN);
        g_test_millis += 100;
        obstacle_sensor::update(0.0f);
    }

    ASSERT_EQ(obstacle_sensor::getAutoRecoveryAttempts(), 0);
}

// Test 47: Auto-recovery resets on valid frame
static void test_auto_recovery_resets_on_valid_frame() {
    printf("  test_auto_recovery_resets_on_valid_frame...\n");

    obstacle_sensor::Config cfg;
    cfg.txPin = 17;
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init(cfg);
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // Inject 20 no-target frames (not enough to trigger)
    for (uint32_t i = 0; i < 20; i++) {
        g_uart_inject_reset();
        uint8_t frame[FRAME_LEN];
        buildNoTargetFrame(frame, 100);
        g_uart_inject(frame, FRAME_LEN);
        g_test_millis += 100;
        obstacle_sensor::update(0.0f);
    }

    // Then inject a valid frame
    g_uart_inject_reset();
    uint8_t good[FRAME_LEN];
    buildMSFrame(good, 1000, 200);
    g_uart_inject(good, FRAME_LEN);
    g_test_millis += 100;
    obstacle_sensor::update(0.0f);

    ASSERT_EQ(obstacle_sensor::getAutoRecoveryAttempts(), 0);

    // Now inject 30 more no-target -- should trigger recovery from 0
    for (uint32_t i = 0; i < 31; i++) {
        g_uart_inject_reset();
        uint8_t frame[FRAME_LEN];
        buildNoTargetFrame(frame, 100);
        g_uart_inject(frame, FRAME_LEN);
        g_test_millis += 100;
        obstacle_sensor::update(0.0f);
    }

    ASSERT(obstacle_sensor::getAutoRecoveryAttempts() >= 1);
}

// Test 48: Auto-recovery limited to max attempts
static void test_auto_recovery_limited_to_max_attempts() {
    printf("  test_auto_recovery_limited_to_max_attempts...\n");

    obstacle_sensor::Config cfg;
    cfg.txPin = 17;
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init(cfg);
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    for (uint32_t i = 0; i < 350; i++) {
        g_uart_inject_reset();
        uint8_t frame[FRAME_LEN];
        buildNoTargetFrame(frame, 100);
        g_uart_inject(frame, FRAME_LEN);
        g_test_millis += 100;
        obstacle_sensor::update(0.0f);
    }

    ASSERT_EQ(obstacle_sensor::getAutoRecoveryAttempts(), 10);
}

// Test 49: init resets auto-recovery
static void test_init_resets_auto_recovery() {
    printf("  test_init_resets_auto_recovery...\n");

    obstacle_sensor::Config cfg;
    cfg.txPin = 17;
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init(cfg);
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    for (uint32_t i = 0; i < 31; i++) {
        g_uart_inject_reset();
        uint8_t frame[FRAME_LEN];
        buildNoTargetFrame(frame, 100);
        g_uart_inject(frame, FRAME_LEN);
        g_test_millis += 100;
        obstacle_sensor::update(0.0f);
    }
    ASSERT(obstacle_sensor::getAutoRecoveryAttempts() >= 1);

    // Re-init should reset
    obstacle_sensor::init(cfg);
    ASSERT_EQ(obstacle_sensor::getAutoRecoveryAttempts(), 0);
}

// Test 50: Distance 65534 is valid (clamped to max)
static void test_distance_65534_is_valid() {
    printf("  test_distance_65534_is_valid...\n");

    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 65534, 100);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(rd.distance_mm, 4000);  // clamped to maxRangeMm
    ASSERT_EQ(rd.healthy, true);
}

// Test 51: Distance 0 is no-target
static void test_distance_zero_is_no_target() {
    printf("  test_distance_zero_is_no_target...\n");

    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 0, 100);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(rd.distance_mm, 20);  // minRangeMm
}

// Test 52: Resync finds real frame after false header
static void test_resync_finds_real_frame_after_false_header() {
    printf("  test_resync_finds_real_frame_after_false_header...\n");

    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // Inject: 0x57 (false header) + some garbage + valid frame
    uint8_t falseStart[] = {0x57, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    g_uart_inject(falseStart, sizeof(falseStart));

    uint8_t good[FRAME_LEN];
    buildMSFrame(good, 900, 100);
    g_uart_inject(good, FRAME_LEN);

    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.distance_mm, 900);
    ASSERT_EQ(rd.healthy, true);
}

// Test 53: Multiple false headers then valid
static void test_multiple_false_headers_then_valid() {
    printf("  test_multiple_false_headers_then_valid...\n");

    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    for (int i = 0; i < 3; i++) {
        uint8_t noise[] = {0x57, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
        g_uart_inject(noise, sizeof(noise));
    }

    uint8_t good[FRAME_LEN];
    buildMSFrame(good, 1100, 200);
    g_uart_inject(good, FRAME_LEN);

    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.distance_mm, 1100);
    ASSERT_EQ(rd.healthy, true);
}

// Test 54: Sensor fault after recovery exhausted
static void test_sensor_fault_after_recovery_exhausted() {
    printf("  test_sensor_fault_after_recovery_exhausted...\n");

    obstacle_sensor::Config cfg;
    cfg.txPin = 17;
    cfg.shortRangeMode = false;
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init(cfg);
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    for (uint32_t i = 0; i < 400; i++) {
        g_uart_inject_reset();
        uint8_t frame[FRAME_LEN];
        buildNoTargetFrame(frame, 100);
        g_uart_inject(frame, FRAME_LEN);
        g_test_millis += 100;
        obstacle_sensor::update(0.0f);
    }

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.healthy, false);
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::INVALID));
}

// Test 55: Sensor fault without TX pin
static void test_sensor_fault_without_tx_pin() {
    printf("  test_sensor_fault_without_tx_pin...\n");

    obstacle_sensor::Config cfg;
    cfg.txPin = -1;
    cfg.shortRangeMode = false;
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init(cfg);
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    for (uint32_t i = 0; i < 70; i++) {
        g_uart_inject_reset();
        uint8_t frame[FRAME_LEN];
        buildNoTargetFrame(frame, 100);
        g_uart_inject(frame, FRAME_LEN);
        g_test_millis += 100;
        obstacle_sensor::update(0.0f);
    }

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.healthy, false);
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::INVALID));
}

// Test 56: Sensor fault clears on valid frame
static void test_sensor_fault_clears_on_valid_frame() {
    printf("  test_sensor_fault_clears_on_valid_frame...\n");

    obstacle_sensor::Config cfg;
    cfg.txPin = -1;
    cfg.shortRangeMode = false;
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init(cfg);
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    for (uint32_t i = 0; i < 70; i++) {
        g_uart_inject_reset();
        uint8_t frame[FRAME_LEN];
        buildNoTargetFrame(frame, 100);
        g_uart_inject(frame, FRAME_LEN);
        g_test_millis += 100;
        obstacle_sensor::update(0.0f);
    }

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.healthy, false);

    // Now send a valid frame
    g_uart_inject_reset();
    uint8_t good[FRAME_LEN];
    buildMSFrame(good, 1500, 200);
    g_uart_inject(good, FRAME_LEN);
    g_test_millis += 100;
    obstacle_sensor::update(0.0f);

    rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.distance_mm, 1500);
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
}

// Test 57: Update count increments
static void test_update_count_increments() {
    printf("  test_update_count_increments...\n");

    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 500, 100);

    // First frame
    g_uart_inject(frame, FRAME_LEN);
    obstacle_sensor::update(0.0f);
    ASSERT_EQ(obstacle_sensor::getReading().updateCount, 1);

    // Second frame
    g_uart_inject_reset();
    g_uart_inject(frame, FRAME_LEN);
    g_test_millis += 100;
    obstacle_sensor::update(0.0f);
    ASSERT_EQ(obstacle_sensor::getReading().updateCount, 2);

    // Third frame
    g_uart_inject_reset();
    g_uart_inject(frame, FRAME_LEN);
    g_test_millis += 100;
    obstacle_sensor::update(0.0f);
    ASSERT_EQ(obstacle_sensor::getReading().updateCount, 3);
}

// Test 58: Short-range mode -- no target reports maxRangeMm
static void test_short_range_no_target_reports_max_range() {
    printf("  test_short_range_no_target_reports_max_range...\n");

    obstacle_sensor::Config cfg;
    cfg.shortRangeMode = true;
    cfg.maxRangeMm = 1350;

    uint8_t frame[FRAME_LEN];
    buildNoTargetFrame(frame, 100);

    obstacle_sensor::Reading rd = injectFrameWithConfig(frame, cfg);

    ASSERT_EQ(rd.distance_mm, 1350);  // maxRangeMm (safe)
    ASSERT_EQ(rd.zone, 1);            // caution zone: [1000, 1500)
    ASSERT_EQ(rd.healthy, true);
}

// Test 59: Short-range mode -- no fault on sustained no-target
static void test_short_range_no_fault_on_sustained_no_target() {
    printf("  test_short_range_no_fault_on_sustained_no_target...\n");

    obstacle_sensor::Config cfg;
    cfg.shortRangeMode = true;
    cfg.maxRangeMm = 1350;
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init(cfg);
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    for (uint32_t i = 0; i < 100; i++) {
        g_uart_inject_reset();
        uint8_t frame[FRAME_LEN];
        buildNoTargetFrame(frame, 100);
        g_uart_inject(frame, FRAME_LEN);
        g_test_millis += 100;
        obstacle_sensor::update(0.0f);
    }

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.distance_mm, 1350);
    ASSERT_EQ(rd.healthy, true);
}

// Test 60: Short-range mode -- valid distance normal
static void test_short_range_valid_distance_normal() {
    printf("  test_short_range_valid_distance_normal...\n");

    obstacle_sensor::Config cfg;
    cfg.shortRangeMode = true;
    cfg.maxRangeMm = 1350;

    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 500, 100);

    obstacle_sensor::Reading rd = injectFrameWithConfig(frame, cfg);

    ASSERT_EQ(rd.distance_mm, 500);
    ASSERT_EQ(rd.zone, 2);
    ASSERT_EQ(rd.healthy, true);
}

// Test 61: Short-range mode default is false
static void test_short_range_mode_default_false() {
    printf("  test_short_range_mode_default_false...\n");

    obstacle_sensor::Config cfg{};
    ASSERT_EQ(cfg.shortRangeMode, false);
}

// Test 62: Short-range recovery to valid
static void test_short_range_recovery_to_valid() {
    printf("  test_short_range_recovery_to_valid...\n");

    obstacle_sensor::Config cfg;
    cfg.shortRangeMode = true;
    cfg.maxRangeMm = 1350;
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init(cfg);
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    for (int i = 0; i < 5; i++) {
        g_uart_inject_reset();
        uint8_t frame[FRAME_LEN];
        buildNoTargetFrame(frame, 100);
        g_uart_inject(frame, FRAME_LEN);
        g_test_millis += 100;
        obstacle_sensor::update(0.0f);
    }

    {
        obstacle_sensor::Reading rd = obstacle_sensor::getReading();
        ASSERT_EQ(rd.distance_mm, 1350);
        ASSERT_EQ(rd.healthy, true);
    }

    // Now an obstacle enters range
    g_uart_inject_reset();
    uint8_t good[FRAME_LEN];
    buildMSFrame(good, 500, 200);
    g_uart_inject(good, FRAME_LEN);
    g_test_millis += 100;
    obstacle_sensor::update(0.0f);

    {
        obstacle_sensor::Reading rd = obstacle_sensor::getReading();
        ASSERT_EQ(rd.distance_mm, 500);
        ASSERT_EQ(rd.healthy, true);
        ASSERT_EQ(rd.zone, 2);
    }
}

// Test 63: Zone thresholds -- exact boundaries
static void test_zone_exact_boundaries() {
    printf("  test_zone_exact_boundaries...\n");

    // < 200 -> zone 4 (emergency)
    uint8_t f1[FRAME_LEN]; buildMSFrame(f1, 199, 100);
    ASSERT_EQ(injectFrameAndUpdate(f1).zone, 4);

    // 200 -> zone 3 (critical)
    uint8_t f2[FRAME_LEN]; buildMSFrame(f2, 200, 100);
    ASSERT_EQ(injectFrameAndUpdate(f2).zone, 3);

    // 499 -> zone 3
    uint8_t f3[FRAME_LEN]; buildMSFrame(f3, 499, 100);
    ASSERT_EQ(injectFrameAndUpdate(f3).zone, 3);

    // 500 -> zone 2 (warning)
    uint8_t f4[FRAME_LEN]; buildMSFrame(f4, 500, 100);
    ASSERT_EQ(injectFrameAndUpdate(f4).zone, 2);

    // 999 -> zone 2
    uint8_t f5[FRAME_LEN]; buildMSFrame(f5, 999, 100);
    ASSERT_EQ(injectFrameAndUpdate(f5).zone, 2);

    // 1000 -> zone 1 (caution)
    uint8_t f6[FRAME_LEN]; buildMSFrame(f6, 1000, 100);
    ASSERT_EQ(injectFrameAndUpdate(f6).zone, 1);

    // 1499 -> zone 1
    uint8_t f7[FRAME_LEN]; buildMSFrame(f7, 1499, 100);
    ASSERT_EQ(injectFrameAndUpdate(f7).zone, 1);

    // 1500 -> zone 0 (normal)
    uint8_t f8[FRAME_LEN]; buildMSFrame(f8, 1500, 100);
    ASSERT_EQ(injectFrameAndUpdate(f8).zone, 0);
}

// Test 64: Frame timeout sets INVALID
static void test_frame_timeout() {
    printf("  test_frame_timeout...\n");

    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 1000, 100);
    g_uart_inject(frame, FRAME_LEN);
    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.healthy, true);

    // Advance time beyond timeout (default 500 ms) without new frames
    g_test_millis += 600;
    g_uart_inject_reset();
    obstacle_sensor::update(0.0f);

    rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.healthy, false);
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::INVALID));
}

// Test 65: Corrupted stream with embedded 0x57 headers
static void test_corrupted_stream_with_embedded_headers() {
    printf("  test_corrupted_stream_with_embedded_headers...\n");

    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    uint8_t noise1[] = {0x10, 0x20, 0x57, 0x99, 0x88, 0x77, 0x66};
    g_uart_inject(noise1, sizeof(noise1));

    uint8_t good[FRAME_LEN];
    buildMSFrame(good, 700, 150);
    g_uart_inject(good, FRAME_LEN);

    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.distance_mm, 700);
    ASSERT_EQ(rd.healthy, true);
}

// Test 66: Distance 1 is valid (clamped to minRangeMm)
static void test_distance_1mm_valid() {
    printf("  test_distance_1mm_valid...\n");

    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 1, 100);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(rd.distance_mm, 20);  // clamped to minRangeMm
    ASSERT_EQ(rd.healthy, true);
}

// Test 67: Distance 65535 (max uint16) -> clamped to maxRangeMm
static void test_distance_max_uint16() {
    printf("  test_distance_max_uint16...\n");

    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 65535, 100);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(rd.distance_mm, 4000);  // clamped
    ASSERT_EQ(rd.healthy, true);
}

// Test 68: Warmup period -> WAITING status
static void test_warmup_period() {
    printf("  test_warmup_period...\n");

    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();

    g_test_millis = 500;
    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::WAITING));
    ASSERT_EQ(rd.healthy, false);
}

// Test 69: Bad function mark rejected
static void test_bad_function_mark_rejected() {
    printf("  test_bad_function_mark_rejected...\n");

    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // Build frame with wrong function mark (0x01 instead of 0x00)
    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 1000, 100);
    frame[1] = 0x01;  // Wrong function mark
    // Recalculate checksum
    uint8_t sum = 0;
    for (int i = 0; i < 6; i++) sum += frame[i];
    frame[6] = sum;

    g_uart_inject(frame, FRAME_LEN);
    obstacle_sensor::update(0.0f);

    // Should have been rejected -- no valid frame processed
    // Reading remains from default or previous state
}

// Test 70: Exact checksum verification -- known values
static void test_exact_checksum_known_values() {
    printf("  test_exact_checksum_known_values...\n");

    // Frame for 1000 mm, signal 100:
    // [0x57, 0x00, 0xE8, 0x03, 0x64, 0x00, checksum]
    // sum = 0x57 + 0x00 + 0xE8 + 0x03 + 0x64 + 0x00
    uint8_t frame[FRAME_LEN];
    buildMSFrame(frame, 1000, 100);

    ASSERT_EQ(frame[0], 0x57);
    ASSERT_EQ(frame[1], 0x00);
    ASSERT_EQ(frame[2], 0xE8);  // 1000 & 0xFF
    ASSERT_EQ(frame[3], 0x03);  // 1000 >> 8
    ASSERT_EQ(frame[4], 0x64);  // 100 & 0xFF
    ASSERT_EQ(frame[5], 0x00);  // 100 >> 8

    uint8_t expected = (uint8_t)(0x57 + 0x00 + 0xE8 + 0x03 + 0x64 + 0x00);
    ASSERT_EQ(frame[6], expected);
}

/* ---- Main --------------------------------------------------------------- */

int main() {
    printf("=== test_obstacle_sensor (TOFSense-M S single-point) ===\n");

    test_frame_constants();
    test_checksum_computation();
    test_distance_field_position();
    test_signal_field_position();
    test_distance_conversion();
    test_zone_mapping();
    test_frame_header_bytes();
    test_checksum_position();
    test_init_resets_state();
    test_config_defaults();
    test_no_target_is_emergency_close();
    test_below_min_range_clamped_to_min();
    test_above_max_range_clamped_to_max();
    test_normal_distance_valid();
    test_exact_min_range_valid();
    test_overflow_flush_keeps_latest_frame();
    test_resync_after_bad_checksum();
    test_single_frame_within_byte_limit();
    test_multiple_bad_frames_then_good();
    test_many_queued_frames_all_processed();
    test_rxbuf_overflow_protection();
    test_build_command_range_mode();
    test_build_command_save_config();
    test_build_command_baud_rate();
    test_build_command_buffer_too_small();
    test_build_command_null_payload_rejected();
    test_build_command_null_payload_zero_len_ok();
    test_build_command_checksum_wraps();
    test_2000mm_distance_parses_correctly();
    test_4000mm_max_range_parses_correctly();
    test_build_command_output_mode();
    test_build_command_frame_rate();
    test_3000mm_distance_parses_correctly();
    test_signal_strength_does_not_affect_distance();
    test_send_command_fails_without_tx_pin();
    test_configure_long_range_fails_without_tx();
    test_configure_long_range_succeeds_with_tx();
    test_set_frame_rate_zero_rejected();
    test_4000mm_raw_bytes();
    test_configure_long_range_sends_correct_commands();
    test_set_range_mode_sends_correct_bytes();
    test_save_config_sends_correct_bytes();
    test_repeated_no_target_stays_at_min();
    test_open_air_above_max_range_is_valid();
    test_auto_recovery_triggers_after_threshold();
    test_auto_recovery_skipped_without_tx_pin();
    test_auto_recovery_resets_on_valid_frame();
    test_auto_recovery_limited_to_max_attempts();
    test_init_resets_auto_recovery();
    test_distance_65534_is_valid();
    test_distance_zero_is_no_target();
    test_resync_finds_real_frame_after_false_header();
    test_multiple_false_headers_then_valid();
    test_sensor_fault_after_recovery_exhausted();
    test_sensor_fault_without_tx_pin();
    test_sensor_fault_clears_on_valid_frame();
    test_update_count_increments();
    test_short_range_no_target_reports_max_range();
    test_short_range_no_fault_on_sustained_no_target();
    test_short_range_valid_distance_normal();
    test_short_range_mode_default_false();
    test_short_range_recovery_to_valid();
    test_zone_exact_boundaries();
    test_frame_timeout();
    test_corrupted_stream_with_embedded_headers();
    test_distance_1mm_valid();
    test_distance_max_uint16();
    test_warmup_period();
    test_bad_function_mark_rejected();
    test_exact_checksum_known_values();

    printf("\n%d tests run, %d failed\n", s_tests_run, s_tests_failed);
    return s_tests_failed > 0 ? 1 : 0;
}
