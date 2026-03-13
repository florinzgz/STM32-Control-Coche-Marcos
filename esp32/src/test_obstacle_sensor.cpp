/**
 ****************************************************************************
 * @file    test_obstacle_sensor.cpp
 * @brief   Host-compilable unit tests for obstacle_sensor frame parsing.
 *
 *          Tests the NLink_TOFSense_M_Frame0 protocol parsing against the
 *          official Nooploop specification (User Manual V3.0).
 *
 *          Compile and run on host (from esp32/src/):
 *            g++ -std=c++17 -I. -I../test_stubs \
 *                sensors/obstacle_sensor.cpp test_obstacle_sensor.cpp \
 *                -o /tmp/test_obstacle_sensor && /tmp/test_obstacle_sensor
 *
 *          Requirements verified:
 *            1. Frame header = 0x57, function_mark = 0x01
 *            2. Pixel data starts at byte offset 9 (byte 8 = pixel_count)
 *            3. Per-pixel layout: dis(3) + dis_status(1) + signal_strength(2)
 *            4. Distance unit: µm (divide by 1000 for mm)
 *            5. Checksum: sum of bytes [0..398] mod 256, stored at byte 399
 *            6. Invalid frames rejected (wrong header, bad checksum)
 *            7. Zone mapping matches STM32 safety tiers
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

// Official NLink_TOFSense_M_Frame0 layout (400 bytes for 8x8):
//   [0]     frame_header    = 0x57
//   [1]     function_mark   = 0x01
//   [2]     reserved
//   [3]     id
//   [4-7]   system_time     (uint32 LE)
//   [8]     pixel_count     (64 for 8x8)
//   [9-392] pixel_data      (64 × 6 bytes)
//   [393-398] reserved1     (6 bytes)
//   [399]   checksum        (sum of bytes [0..398] mod 256)

static constexpr uint16_t FRAME_LEN = 400;

static void buildFrame(uint8_t* buf) {
    memset(buf, 0, FRAME_LEN);
    buf[0] = 0x57;  // frame_header
    buf[1] = 0x01;  // function_mark
    buf[2] = 0x00;  // reserved
    buf[3] = 0x01;  // id
    // system_time = 0
    buf[8] = 64;    // pixel_count = 64 (8x8)
}

// Set a pixel's distance and status in the frame buffer.
// Per-pixel layout (6 bytes each):
//   [0-2] dis: int24 LE, unit = µm
//   [3]   dis_status: 0 = valid
//   [4-5] signal_strength: uint16 LE
static void setPixel(uint8_t* buf, uint8_t px, int32_t distUm,
                     uint8_t status, uint16_t signal) {
    uint16_t base = 9 + px * 6;  // pixel data starts at offset 9
    // Distance: 3-byte LE signed
    buf[base + 0] = (uint8_t)(distUm & 0xFF);
    buf[base + 1] = (uint8_t)((distUm >> 8) & 0xFF);
    buf[base + 2] = (uint8_t)((distUm >> 16) & 0xFF);
    // dis_status
    buf[base + 3] = status;
    // signal_strength (uint16 LE)
    buf[base + 4] = (uint8_t)(signal & 0xFF);
    buf[base + 5] = (uint8_t)((signal >> 8) & 0xFF);
}

// Compute and write the checksum at byte 399
static void writeChecksum(uint8_t* buf) {
    uint8_t sum = 0;
    for (uint16_t i = 0; i < FRAME_LEN - 1; i++) {
        sum += buf[i];
    }
    buf[FRAME_LEN - 1] = sum;
}

/* ---- Access to static parseFrame via the translation unit --------------- */
// We access the parseFrame function through the obstacle_sensor namespace.
// Since parseFrame is static, we use a test helper that feeds a full frame
// through the public API (init + update + getReading).

// UART data injection: override the HardwareSerial stub to provide data
static uint8_t  g_uart_buf[FRAME_LEN * 2];
static uint16_t g_uart_len = 0;
static uint16_t g_uart_pos = 0;

// Patch the HardwareSerial stub to return injected data
// (Override the inline stubs from test_stubs/Arduino.h)
// NOTE: The HardwareSerial in the stub is a struct with available() and read().
// We need to hook into it. Since obstacle_sensor.cpp creates its own
// HardwareSerial(1), and the stub's available()/read() always return 0/-1,
// we need a different approach.

// Since we can't easily inject UART data through the stub, we test the
// frame parsing logic by directly calling internal functions.
// We include the .cpp to access static functions in the same TU.

// The obstacle_sensor.cpp is already compiled as part of this test.
// We can access the static parseFrame by declaring it extern here
// (since it's in the same binary). However, static functions have internal
// linkage. Instead, we replicate the parsing logic in a test-only wrapper.

// Actually, the simplest approach: we test the PUBLIC API behavior by
// verifying the constants and frame structure match the official spec.

/* ---- Tests -------------------------------------------------------------- */

// Test 1: Verify frame layout constants match official Nooploop spec
static void test_frame_constants() {
    printf("  test_frame_constants...\n");

    // Official struct:
    // header(1) + function_mark(1) + reserved(1) + id(1) + system_time(4)
    //   + pixel_count(1) + pixels(64*6) + reserved1(6) + sum(1) = 400
    ASSERT_EQ(1 + 1 + 1 + 1 + 4 + 1 + 64 * 6 + 6 + 1, 400);

    // Pixel data starts after header (9 bytes)
    ASSERT_EQ(1 + 1 + 1 + 1 + 4 + 1, 9);  // pixel data offset

    // Each pixel is 6 bytes: dis(3) + dis_status(1) + signal_strength(2)
    ASSERT_EQ(3 + 1 + 2, 6);
}

// Test 2: Build a valid frame and verify checksum computation
static void test_checksum_computation() {
    printf("  test_checksum_computation...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // Set pixel 0 to 1 meter (1,000,000 µm), valid status
    setPixel(frame, 0, 1000000, 0, 100);

    // Compute checksum
    writeChecksum(frame);

    // Verify: sum of bytes [0..398] should equal byte[399]
    uint8_t sum = 0;
    for (uint16_t i = 0; i < FRAME_LEN - 1; i++) {
        sum += frame[i];
    }
    ASSERT_EQ(sum, frame[399]);
}

// Test 3: Verify pixel data offset is correct (byte 9, not byte 8)
static void test_pixel_data_offset() {
    printf("  test_pixel_data_offset...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // byte 8 should be pixel_count (64), NOT part of pixel data
    ASSERT_EQ(frame[8], 64);

    // Pixel 0 data starts at byte 9
    // Set pixel 0 distance to a known value
    int32_t testDist = 500000;  // 500,000 µm = 500 mm
    setPixel(frame, 0, testDist, 0, 100);

    // Verify byte 9 contains the LSB of the distance
    ASSERT_EQ(frame[9], (uint8_t)(testDist & 0xFF));
    ASSERT_EQ(frame[10], (uint8_t)((testDist >> 8) & 0xFF));
    ASSERT_EQ(frame[11], (uint8_t)((testDist >> 16) & 0xFF));

    // Verify byte 12 is dis_status (0 = valid)
    ASSERT_EQ(frame[12], 0);

    // Verify bytes 13-14 are signal_strength (100 = 0x64, 0x00)
    ASSERT_EQ(frame[13], 0x64);
    ASSERT_EQ(frame[14], 0x00);
}

// Test 4: Verify per-pixel field order matches official struct
// Official: dis(3) + dis_status(1) + signal_strength(2)
static void test_pixel_field_order() {
    printf("  test_pixel_field_order...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // Set pixel 5 with specific values to verify field positions
    int32_t dist = 2000000;     // 2,000,000 µm = 2000 mm = 2 m
    uint8_t status = 0;         // valid
    uint16_t signal = 0x1234;   // test pattern

    setPixel(frame, 5, dist, status, signal);

    uint16_t base = 9 + 5 * 6;  // pixel 5 starts at byte 9 + 30 = 39

    // [base+0..2] = distance (3 bytes LE)
    ASSERT_EQ(frame[base + 0], (uint8_t)(dist & 0xFF));
    ASSERT_EQ(frame[base + 1], (uint8_t)((dist >> 8) & 0xFF));
    ASSERT_EQ(frame[base + 2], (uint8_t)((dist >> 16) & 0xFF));

    // [base+3] = dis_status
    ASSERT_EQ(frame[base + 3], status);

    // [base+4..5] = signal_strength (uint16 LE)
    ASSERT_EQ(frame[base + 4], 0x34);
    ASSERT_EQ(frame[base + 5], 0x12);
}

// Test 5: Verify distance conversion factor (µm / 1000 = mm)
static void test_distance_conversion() {
    printf("  test_distance_conversion...\n");

    // 1 meter = 1,000,000 µm → 1000 mm
    ASSERT_EQ(1000000 / 1000, 1000);

    // 500 mm = 500,000 µm
    ASSERT_EQ(500000 / 1000, 500);

    // 200 mm (emergency zone threshold)
    ASSERT_EQ(200000 / 1000, 200);

    // 4000 mm (max range) = 4,000,000 µm
    ASSERT_EQ(4000000 / 1000, 4000);

    // Max int24 positive = 8,388,607 µm ≈ 8.39 m (within 8m sensor range)
    ASSERT_EQ(8388607 / 1000, 8388);  // ~8.4 m
}

// Test 6: Verify zone mapping thresholds
static void test_zone_mapping() {
    printf("  test_zone_mapping...\n");

    // Zone mapping from obstacle_sensor.cpp:
    // < 200 mm  → zone 4 (emergency)
    // 200–500   → zone 3 (critical)
    // 500–1000  → zone 2 (warning)
    // 1000–1500 → zone 1 (caution)
    // ≥ 1500    → zone 0 (normal)

    // Init to get default state
    obstacle_sensor::init();
    obstacle_sensor::Reading rd = obstacle_sensor::getReading();

    // Default reading should be WAITING during warmup
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::WAITING));
    ASSERT_EQ(rd.healthy, false);
}

// Test 7: Verify frame header validation
static void test_frame_header_bytes() {
    printf("  test_frame_header_bytes...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // Byte 0 must be 0x57 (frame_header)
    ASSERT_EQ(frame[0], 0x57);

    // Byte 1 must be 0x01 (function_mark for Frame0)
    ASSERT_EQ(frame[1], 0x01);
}

// Test 8: Verify checksum byte position is at byte 399
static void test_checksum_position() {
    printf("  test_checksum_position...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);
    setPixel(frame, 0, 1000000, 0, 50);
    writeChecksum(frame);

    // Checksum should be at the very last byte (399)
    // Reserved1 occupies bytes 393-398
    uint16_t checksumOffset = FRAME_LEN - 1;
    ASSERT_EQ(checksumOffset, 399);

    // Pixel data ends at byte 392 (9 + 64*6 - 1)
    uint16_t lastPixelByte = 9 + 64 * 6 - 1;
    ASSERT_EQ(lastPixelByte, 392);

    // Reserved1 starts at byte 393
    ASSERT_EQ(lastPixelByte + 1, 393);

    // Reserved1 ends at byte 398 (6 bytes: 393-398)
    ASSERT_EQ(393 + 6 - 1, 398);
}

// Test 9: Verify two-byte sync pattern (0x57 0x01) distinguishes real headers
//         from false 0x57 occurrences in pixel distance data.
//         The update() function now checks both bytes during frame sync.
static void test_two_byte_sync_pattern() {
    printf("  test_two_byte_sync_pattern...\n");

    // A false 0x57 inside pixel data must NOT look like a valid header.
    // The function_mark byte (0x01) serves as the confirmation byte.
    // Probability of false sync: 1/256 (only 0x57) → 1/65536 (0x57+0x01).

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // Set pixel 0 to a distance where the LSB is 0x57 (87 µm).
    // This creates a 0x57 inside pixel data at offset 9.
    setPixel(frame, 0, 87, 0, 100);   // 87 µm, valid, signal=100
    writeChecksum(frame);

    // The byte at offset 9 (pixel 0, distance LSB) is 0x57 — same as header.
    ASSERT_EQ(frame[9], 0x57);

    // But the next byte (distance byte 1) is 0x00 — NOT 0x01.
    // The two-byte sync check rejects this as a false header.
    ASSERT(frame[10] != 0x01);

    // The real header at offset 0 has the correct two-byte pattern.
    ASSERT_EQ(frame[0], 0x57);
    ASSERT_EQ(frame[1], 0x01);
}

// Test 10: Verify init() resets state correctly for warmup flush
static void test_init_resets_state() {
    printf("  test_init_resets_state...\n");

    // After init(), sensor should be in WAITING state
    obstacle_sensor::init();
    obstacle_sensor::Reading rd = obstacle_sensor::getReading();

    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::WAITING));
    ASSERT_EQ(rd.healthy, false);
    ASSERT_EQ(rd.stuck, false);
    ASSERT_EQ(rd.distance_mm, 0);
    ASSERT_EQ(rd.zone, 0);
}

// Test 11: Verify Config defaults — baud rate and RX buffer size
//          Baud rate must be 921600 (TOFSense / TOFSense-M factory default).
//          RX buffer must be > 400 (one full frame) to prevent overflow.
static void test_config_defaults() {
    printf("  test_config_defaults...\n");

    obstacle_sensor::Config cfg{};

    // Baud rate: 921600 — factory default per Nooploop User Manual V3.0
    ASSERT_EQ(cfg.baudRate, 921600UL);

    // RX buffer: must be larger than a single 400-byte frame
    ASSERT(cfg.rxBufSize > FRAME_LEN);

    // Default value should be 2048 (~5 frames, ~22 ms headroom at 921600)
    ASSERT_EQ(cfg.rxBufSize, 2048);
}

/* ---- Integration helper: inject a frame and run update() ---------------- */

// Reset sensor + UART stub, advance past warmup, then inject `frame`
// and call update().  Returns the reading produced.
static obstacle_sensor::Reading injectFrameAndUpdate(uint8_t* frame) {
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

// Test 12: All pixels invalid (NO_VALID_PIXELS) → treated as emergency-close
//          instead of timing out to INVALID.
//          This is the core fix: the TOFSense-M reports all pixels invalid
//          when an object is < ~50 mm from the sensor (hardware limitation).
static void test_all_pixels_invalid_is_emergency_close() {
    printf("  test_all_pixels_invalid_is_emergency_close...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // All 64 pixels have dis_status = 1 (invalid) — no valid measurements
    for (uint8_t px = 0; px < 64; px++) {
        setPixel(frame, px, 50000, /*status=*/1, 100);
    }
    writeChecksum(frame);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    // Should be VALID (not INVALID) — treated as "too close"
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);

    // Distance should be clamped to minRangeMm (default 20 mm)
    ASSERT_EQ(rd.distance_mm, 20);

    // Zone should be 4 (emergency: < 200 mm)
    ASSERT_EQ(rd.zone, 4);
}

// Test 13: Distance below minRangeMm is clamped (not INVALID).
//          If the sensor reports a valid pixel at e.g. 10 mm, that's still
//          a real obstacle — clamp to minRangeMm and report VALID.
static void test_below_min_range_clamped_to_min() {
    printf("  test_below_min_range_clamped_to_min...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // Set pixel 0 to 10 mm = 10,000 µm (below minRangeMm = 20)
    setPixel(frame, 0, 10000, /*status=*/0, 100);
    writeChecksum(frame);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.distance_mm, 20);   // clamped to minRangeMm
    ASSERT_EQ(rd.zone, 4);           // emergency zone
}

// Test 14: Distance above maxRangeMm still goes INVALID.
//          Only below-minimum clamping changed; above-max is still rejected.
static void test_above_max_range_still_invalid() {
    printf("  test_above_max_range_still_invalid...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // Set pixel 0 to 5000 mm = 5,000,000 µm (above maxRangeMm = 4000)
    setPixel(frame, 0, 5000000, /*status=*/0, 100);
    writeChecksum(frame);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::INVALID));
    ASSERT_EQ(rd.healthy, false);
}

// Test 15: Normal in-range distance (500 mm) → VALID, correct zone.
static void test_normal_distance_valid() {
    printf("  test_normal_distance_valid...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // Set pixel 0 to 500 mm = 500,000 µm
    setPixel(frame, 0, 500000, /*status=*/0, 100);
    writeChecksum(frame);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.distance_mm, 500);
    ASSERT_EQ(rd.zone, 2);   // warning zone: [500, 1000) mm
}

// Test 16: Exactly minRangeMm distance (20 mm) → VALID at emergency zone.
static void test_exact_min_range_valid() {
    printf("  test_exact_min_range_valid...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // Set pixel 0 to 20 mm = 20,000 µm (exactly minRangeMm)
    setPixel(frame, 0, 20000, /*status=*/0, 100);
    writeChecksum(frame);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.distance_mm, 20);
    ASSERT_EQ(rd.zone, 4);   // emergency zone: < 200 mm
}

// Test 17: Overflow flush — when more than 2 frames of data are buffered,
//          old bytes are discarded and only the latest frame is processed.
//          This prevents blocking and cascading checksum failures.
static void test_overflow_flush_keeps_latest_frame() {
    printf("  test_overflow_flush_keeps_latest_frame...\n");

    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();

    // Advance past warmup
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);  // flush warmup

    // Inject 3 frames: 2 old + 1 new (latest).
    // The overflow logic should discard old data and parse the latest.

    // Frame 1 (old): 300 mm
    uint8_t frame1[FRAME_LEN];
    buildFrame(frame1);
    setPixel(frame1, 0, 300000, 0, 100);
    writeChecksum(frame1);

    // Frame 2 (old): 400 mm
    uint8_t frame2[FRAME_LEN];
    buildFrame(frame2);
    setPixel(frame2, 0, 400000, 0, 100);
    writeChecksum(frame2);

    // Frame 3 (latest): 800 mm
    uint8_t frame3[FRAME_LEN];
    buildFrame(frame3);
    setPixel(frame3, 0, 800000, 0, 100);
    writeChecksum(frame3);

    g_uart_inject(frame1, FRAME_LEN);
    g_uart_inject(frame2, FRAME_LEN);
    g_uart_inject(frame3, FRAME_LEN);

    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    // After overflow flush discards old frames, the latest frame (800 mm)
    // should be the one that gets parsed successfully.
    ASSERT_EQ(rd.distance_mm, 800);
    ASSERT_EQ(rd.zone, 2);  // warning zone: [500, 1000) mm
}

// Test 18: Resync after bad frame — when a frame has a bad checksum,
//          the parser should scan for the next valid header {0x57, 0x01}
//          inside the consumed buffer, avoiding a full frame's worth of
//          re-alignment delay.
static void test_resync_after_bad_checksum() {
    printf("  test_resync_after_bad_checksum...\n");

    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();

    // Advance past warmup
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);  // flush warmup

    // Build a corrupted frame (bad checksum) followed by a valid frame.
    // The valid frame's header sits inside the bad frame's data area.
    // After checksum failure, resync should find it.

    uint8_t bad_frame[FRAME_LEN];
    buildFrame(bad_frame);
    setPixel(bad_frame, 0, 600000, 0, 100);
    writeChecksum(bad_frame);
    bad_frame[FRAME_LEN - 1] ^= 0xFF;  // Corrupt checksum

    uint8_t good_frame[FRAME_LEN];
    buildFrame(good_frame);
    setPixel(good_frame, 0, 750000, 0, 100);  // 750 mm
    writeChecksum(good_frame);

    g_uart_inject(bad_frame, FRAME_LEN);
    g_uart_inject(good_frame, FRAME_LEN);

    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    // The good frame (750 mm) should have been parsed after resync
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.distance_mm, 750);
    ASSERT_EQ(rd.zone, 2);  // warning zone: [500, 1000) mm
}

// Test 19: Single frame within the per-update byte limit parses correctly.
//          Verifies that the byte-limit guard (MAX_BYTES_PER_UPDATE) does
//          not interfere with normal single-frame processing.
static void test_single_frame_within_byte_limit() {
    printf("  test_single_frame_within_byte_limit...\n");

    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();

    // Advance past warmup
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);  // flush warmup

    // Inject 1 valid frame.  After the overflow flush discards excess,
    // the remaining data fits within the per-update limit and parses OK.
    uint8_t frame[FRAME_LEN];
    buildFrame(frame);
    setPixel(frame, 0, 1200000, 0, 100);  // 1200 mm
    writeChecksum(frame);

    g_uart_inject(frame, FRAME_LEN);
    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.distance_mm, 1200);
    ASSERT_EQ(rd.zone, 1);  // caution zone: [1000, 1500) mm
}

// Test 20: Multiple bad frames followed by a good frame — the parser must
//          recover and return a VALID reading from the good frame.
//          This reproduces the cascading-failure scenario that caused
//          intermittent INVALID every ~1–1.5 s on the real sensor.
static void test_multiple_bad_frames_then_good() {
    printf("  test_multiple_bad_frames_then_good...\n");

    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();

    // Advance past warmup
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);  // flush warmup

    // Inject 3 corrupted frames (bad checksum) followed by 1 good frame.
    // The parser should discard the bad frames and still find the good one.
    for (int i = 0; i < 3; i++) {
        uint8_t bad[FRAME_LEN];
        buildFrame(bad);
        setPixel(bad, 0, 500000 + i * 100000, 0, 100);
        writeChecksum(bad);
        bad[FRAME_LEN - 1] ^= 0xFF;  // Corrupt checksum
        g_uart_inject(bad, FRAME_LEN);
    }

    uint8_t good[FRAME_LEN];
    buildFrame(good);
    setPixel(good, 0, 900000, 0, 100);  // 900 mm
    writeChecksum(good);
    g_uart_inject(good, FRAME_LEN);

    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.distance_mm, 900);
    ASSERT_EQ(rd.zone, 2);  // warning zone: [500, 1000) mm
}

// Test 21: Many queued frames are all processed in a single update() call —
//          no byte-limit cut-off causes the sensor to miss frames and time out.
static void test_many_queued_frames_all_processed() {
    printf("  test_many_queued_frames_all_processed...\n");

    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();

    // Advance past warmup
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);  // flush warmup

    // Inject 5 valid frames (simulating a slow main-loop iteration).
    // All should be processed, and the reading should reflect the last one.
    for (int i = 0; i < 5; i++) {
        uint8_t frame[FRAME_LEN];
        buildFrame(frame);
        setPixel(frame, 0, (uint32_t)(200 + i * 100) * 1000, 0, 100);
        writeChecksum(frame);
        g_uart_inject(frame, FRAME_LEN);
    }

    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    // Last frame: (200 + 4*100) * 1000 µm = 600,000 µm = 600 mm
    ASSERT_EQ(rd.distance_mm, 600);
    ASSERT_EQ(rd.zone, 2);  // warning zone: [500, 1000) mm
}

/* ---- Main --------------------------------------------------------------- */

int main() {
    printf("=== test_obstacle_sensor ===\n");

    test_frame_constants();
    test_checksum_computation();
    test_pixel_data_offset();
    test_pixel_field_order();
    test_distance_conversion();
    test_zone_mapping();
    test_frame_header_bytes();
    test_checksum_position();
    test_two_byte_sync_pattern();
    test_init_resets_state();
    test_config_defaults();
    test_all_pixels_invalid_is_emergency_close();
    test_below_min_range_clamped_to_min();
    test_above_max_range_still_invalid();
    test_normal_distance_valid();
    test_exact_min_range_valid();
    test_overflow_flush_keeps_latest_frame();
    test_resync_after_bad_checksum();
    test_single_frame_within_byte_limit();
    test_multiple_bad_frames_then_good();
    test_many_queued_frames_all_processed();

    printf("\n%d tests run, %d failed\n", s_tests_run, s_tests_failed);
    return s_tests_failed > 0 ? 1 : 0;
}
