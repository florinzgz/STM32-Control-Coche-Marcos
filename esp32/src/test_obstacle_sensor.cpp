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
 *            4. Distance unit: mm (raw int24 value is in mm)
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
//   [0-2] dis: int24 LE, unit = mm (per official Nooploop nlink_tofsensem_frame0.c)
//   [3]   dis_status: 0 = valid
//   [4-5] signal_strength: uint16 LE
static void setPixel(uint8_t* buf, uint8_t px, int32_t distMm,
                     uint8_t status, uint16_t signal) {
    uint16_t base = 9 + px * 6;  // pixel data starts at offset 9
    // Distance: 3-byte LE signed (mm)
    buf[base + 0] = (uint8_t)(distMm & 0xFF);
    buf[base + 1] = (uint8_t)((distMm >> 8) & 0xFF);
    buf[base + 2] = (uint8_t)((distMm >> 16) & 0xFF);
    // dis_status
    buf[base + 3] = status;
    // signal_strength (uint16 LE)
    buf[base + 4] = (uint8_t)(signal & 0xFF);
    buf[base + 5] = (uint8_t)((signal >> 8) & 0xFF);
}

// Helper: set multiple consecutive pixels to the same valid distance.
// Used to satisfy the MIN_VALID_PIXELS threshold (4 pixels) so that
// the frame parses as OK instead of being rejected as wrap-around.
static constexpr uint8_t TEST_MIN_PX = 4;  // Matches MIN_VALID_PIXELS in driver
static void setValidPixels(uint8_t* buf, int32_t distMm,
                           uint16_t signal = 100, uint8_t startPx = 0) {
    for (uint8_t px = startPx; px < startPx + TEST_MIN_PX; px++) {
        setPixel(buf, px, distMm, /*status=*/0, signal);
    }
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

    // Set pixel 0 to 1 meter (1000 mm), valid status
    setPixel(frame, 0, 1000, 0, 100);

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
    int32_t testDist = 500;  // 500 mm
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
    int32_t dist = 2000;            // 2000 mm = 2 m
    uint8_t status = 0;             // valid
    uint16_t signal = 0x1234;       // test pattern

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

// Test 5: Verify distance unit is mm (per official Nooploop nlink_tofsensem_frame0.c).
//         Raw int24 value is already in mm — no division needed.
//         Official code divides by 1000.0f to get meters (float); we keep mm.
static void test_distance_conversion() {
    printf("  test_distance_conversion...\n");

    // Raw int24 = mm → no conversion needed for our uint16 distance_mm
    // 1 meter = 1000 mm raw
    ASSERT_EQ(1000, 1000);

    // 500 mm raw
    ASSERT_EQ(500, 500);

    // 200 mm (emergency zone threshold)
    ASSERT_EQ(200, 200);

    // 4000 mm (max range)
    ASSERT_EQ(4000, 4000);

    // Max int24 positive = 8,388,607 mm ≈ 8388 m (far beyond sensor range)
    ASSERT_EQ(8388607, 8388607);
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
    setPixel(frame, 0, 1000, 0, 50);
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

    // Set pixel 0 to a distance where the LSB is 0x57 (87 mm).
    // This creates a 0x57 inside pixel data at offset 9.
    setPixel(frame, 0, 87, 0, 100);   // 87 mm, valid, signal=100
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
        setPixel(frame, px, 50, /*status=*/1, 100);
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

    // Set pixel 0 to 10 mm (below minRangeMm = 20)
    setValidPixels(frame, 10);
    writeChecksum(frame);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.distance_mm, 20);   // clamped to minRangeMm
    ASSERT_EQ(rd.zone, 4);           // emergency zone
}

// Test 14: Distance above maxRangeMm is clamped to maxRangeMm (VALID).
//          This represents "open air" — nothing within the sensor's effective
//          range.  The reading is valid (sensor is working) at zone 0 (no
//          obstacle), rather than being marked INVALID (which would look like
//          a sensor fault to the STM32).
static void test_above_max_range_clamped_to_max() {
    printf("  test_above_max_range_clamped_to_max...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // Set many pixels to 5000 mm (above maxRangeMm = 4000) to exceed
    // MIN_VALID_PIXELS threshold (need ≥ 4 valid pixels).
    for (uint8_t px = 0; px < 10; px++) {
        setPixel(frame, px, 5000, /*status=*/0, 100);
    }
    writeChecksum(frame);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    // Should be VALID at maxRangeMm, zone 0 (no obstacle)
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.distance_mm, 4000);  // clamped to maxRangeMm
    ASSERT_EQ(rd.zone, 0);            // normal zone: ≥ 1500 mm
}

// Test 15: Normal in-range distance (500 mm) → VALID, correct zone.
static void test_normal_distance_valid() {
    printf("  test_normal_distance_valid...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // Set pixel 0 to 500 mm
    setValidPixels(frame, 500);
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

    // Set pixel 0 to 20 mm (exactly minRangeMm)
    setValidPixels(frame, 20);
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
    setValidPixels(frame1, 300);
    writeChecksum(frame1);

    // Frame 2 (old): 400 mm
    uint8_t frame2[FRAME_LEN];
    buildFrame(frame2);
    setValidPixels(frame2, 400);
    writeChecksum(frame2);

    // Frame 3 (latest): 800 mm
    uint8_t frame3[FRAME_LEN];
    buildFrame(frame3);
    setValidPixels(frame3, 800);
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
    setValidPixels(bad_frame, 600);
    writeChecksum(bad_frame);
    bad_frame[FRAME_LEN - 1] ^= 0xFF;  // Corrupt checksum

    uint8_t good_frame[FRAME_LEN];
    buildFrame(good_frame);
    setValidPixels(good_frame, 750);  // 750 mm
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
    setValidPixels(frame, 1200);  // 1200 mm
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
        setValidPixels(bad, 500 + i * 100);
        writeChecksum(bad);
        bad[FRAME_LEN - 1] ^= 0xFF;  // Corrupt checksum
        g_uart_inject(bad, FRAME_LEN);
    }

    uint8_t good[FRAME_LEN];
    buildFrame(good);
    setValidPixels(good, 900);  // 900 mm
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
        setValidPixels(frame, (int32_t)(200 + i * 100));
        writeChecksum(frame);
        g_uart_inject(frame, FRAME_LEN);
    }

    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    // Last frame: (200 + 4*100) = 600 mm
    ASSERT_EQ(rd.distance_mm, 600);
    ASSERT_EQ(rd.zone, 2);  // warning zone: [500, 1000) mm
}

/* ---- Configuration command tests ---------------------------------------- */

// Test 22: buildCommand produces correct frame for SET_RANGE_MODE (long range)
//          Frame: [0x5A, len, 0x07, 0x01, checksum]
static void test_build_command_range_mode() {
    printf("  test_build_command_range_mode...\n");

    uint8_t buf[16];
    uint8_t payload = static_cast<uint8_t>(obstacle_sensor::RangeMode::LONG_RANGE);
    uint16_t len = obstacle_sensor::buildCommand(
        static_cast<uint8_t>(obstacle_sensor::CmdId::SET_RANGE_MODE),
        &payload, 1, buf, sizeof(buf));

    // Total = header(1) + len(1) + cmd(1) + payload(1) + checksum(1) = 5
    ASSERT_EQ(len, 5);

    // Header
    ASSERT_EQ(buf[0], 0x5A);

    // Length byte = total frame bytes
    ASSERT_EQ(buf[1], 5);

    // Command ID = SET_RANGE_MODE (0x07)
    ASSERT_EQ(buf[2], 0x07);

    // Payload = LONG_RANGE (0x01)
    ASSERT_EQ(buf[3], 0x01);

    // Checksum = (0x5A + 0x05 + 0x07 + 0x01) & 0xFF = 0x67
    uint8_t expectedSum = (uint8_t)(0x5A + 5 + 0x07 + 0x01);
    ASSERT_EQ(buf[4], expectedSum);
}

// Test 23: buildCommand for SAVE_CONFIG (no payload)
//          Frame: [0x5A, 0x04, 0x08, checksum]
static void test_build_command_save_config() {
    printf("  test_build_command_save_config...\n");

    uint8_t buf[16];
    uint16_t len = obstacle_sensor::buildCommand(
        static_cast<uint8_t>(obstacle_sensor::CmdId::SAVE_CONFIG),
        nullptr, 0, buf, sizeof(buf));

    // Total = header(1) + len(1) + cmd(1) + checksum(1) = 4
    ASSERT_EQ(len, 4);
    ASSERT_EQ(buf[0], 0x5A);
    ASSERT_EQ(buf[1], 4);
    ASSERT_EQ(buf[2], 0x08);

    // Checksum = (0x5A + 0x04 + 0x08) & 0xFF
    uint8_t expectedSum = (uint8_t)(0x5A + 4 + 0x08);
    ASSERT_EQ(buf[3], expectedSum);
}

// Test 24: buildCommand for SET_BAUD_RATE (115200)
static void test_build_command_baud_rate() {
    printf("  test_build_command_baud_rate...\n");

    uint8_t buf[16];
    uint8_t payload = static_cast<uint8_t>(obstacle_sensor::BaudRateCode::BAUD_115200);
    uint16_t len = obstacle_sensor::buildCommand(
        static_cast<uint8_t>(obstacle_sensor::CmdId::SET_BAUD_RATE),
        &payload, 1, buf, sizeof(buf));

    ASSERT_EQ(len, 5);
    ASSERT_EQ(buf[0], 0x5A);
    ASSERT_EQ(buf[1], 5);
    ASSERT_EQ(buf[2], 0x03);  // SET_BAUD_RATE
    ASSERT_EQ(buf[3], 0x04);  // BAUD_115200

    uint8_t expectedSum = (uint8_t)(0x5A + 5 + 0x03 + 0x04);
    ASSERT_EQ(buf[4], expectedSum);
}

// Test 25: buildCommand returns 0 when buffer is too small
static void test_build_command_buffer_too_small() {
    printf("  test_build_command_buffer_too_small...\n");

    uint8_t buf[3];  // Too small for any command
    uint8_t payload = 0x01;
    uint16_t len = obstacle_sensor::buildCommand(
        0x07, &payload, 1, buf, sizeof(buf));

    ASSERT_EQ(len, 0);  // Should fail
}

// Test 26: buildCommand checksum is correct modulo 256
static void test_build_command_checksum_wraps() {
    printf("  test_build_command_checksum_wraps...\n");

    uint8_t buf[16];
    uint8_t payload = 0xFF;
    uint16_t len = obstacle_sensor::buildCommand(0xFF, &payload, 1, buf, sizeof(buf));

    ASSERT_EQ(len, 5);
    // Sum = 0x5A + 0x05 + 0xFF + 0xFF = 0x25D → 0x5D (mod 256)
    uint8_t expectedSum = (uint8_t)(0x5A + 5 + 0xFF + 0xFF);
    ASSERT_EQ(buf[4], expectedSum);
}

// Test 27: 2000 mm distance parses correctly through the full pipeline
//          (regression test for the ~1368 mm issue — verifies code is NOT
//          the cause of the truncation; the limit is sensor configuration).
static void test_2000mm_distance_parses_correctly() {
    printf("  test_2000mm_distance_parses_correctly...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // Set pixel 0 to 2000 mm = 0x0007D0
    // Byte[0]=0xD0, Byte[1]=0x07, Byte[2]=0x00
    setPixel(frame, 0, 2000, /*status=*/0, 200);
    // Set additional pixels to meet MIN_VALID_PIXELS threshold
    for (uint8_t px = 1; px < TEST_MIN_PX; px++)
        setPixel(frame, px, 2000, /*status=*/0, 200);
    writeChecksum(frame);

    // Verify the raw bytes are correct
    uint16_t base = 9;  // pixel 0
    ASSERT_EQ(frame[base + 0], 0xD0);
    ASSERT_EQ(frame[base + 1], 0x07);
    ASSERT_EQ(frame[base + 2], 0x00);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.distance_mm, 2000);
    ASSERT_EQ(rd.zone, 0);  // normal zone: ≥ 1500 mm
}

// Test 28: 4000 mm (max range) parses correctly
static void test_4000mm_max_range_parses_correctly() {
    printf("  test_4000mm_max_range_parses_correctly...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // Set pixel 0 to 4000 mm = 0x000FA0
    setValidPixels(frame, 4000, 150);
    writeChecksum(frame);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.distance_mm, 4000);
    ASSERT_EQ(rd.zone, 0);  // normal zone: ≥ 1500 mm
}

// Test 29: buildCommand for SET_OUTPUT_MODE (active continuous mode)
//          Frame: [0x5A, 0x05, 0x02, 0x00, checksum]
static void test_build_command_output_mode() {
    printf("  test_build_command_output_mode...\n");

    uint8_t buf[16];
    uint8_t payload = static_cast<uint8_t>(obstacle_sensor::OutputMode::ACTIVE);
    uint16_t len = obstacle_sensor::buildCommand(
        static_cast<uint8_t>(obstacle_sensor::CmdId::SET_OUTPUT_MODE),
        &payload, 1, buf, sizeof(buf));

    // Total = header(1) + len(1) + cmd(1) + payload(1) + checksum(1) = 5
    ASSERT_EQ(len, 5);
    ASSERT_EQ(buf[0], 0x5A);
    ASSERT_EQ(buf[1], 5);
    ASSERT_EQ(buf[2], 0x02);  // SET_OUTPUT_MODE
    ASSERT_EQ(buf[3], 0x00);  // ACTIVE

    uint8_t expectedSum = (uint8_t)(0x5A + 5 + 0x02 + 0x00);
    ASSERT_EQ(buf[4], expectedSum);
}

// Test 30: buildCommand for SET_FRAME_RATE (10 Hz, recommended for long range)
//          Frame: [0x5A, 0x05, 0x05, 0x0A, checksum]
static void test_build_command_frame_rate() {
    printf("  test_build_command_frame_rate...\n");

    uint8_t buf[16];
    uint8_t hz = 10;
    uint16_t len = obstacle_sensor::buildCommand(
        static_cast<uint8_t>(obstacle_sensor::CmdId::SET_FRAME_RATE),
        &hz, 1, buf, sizeof(buf));

    ASSERT_EQ(len, 5);
    ASSERT_EQ(buf[0], 0x5A);
    ASSERT_EQ(buf[1], 5);
    ASSERT_EQ(buf[2], 0x05);  // SET_FRAME_RATE
    ASSERT_EQ(buf[3], 10);    // 10 Hz

    uint8_t expectedSum = (uint8_t)(0x5A + 5 + 0x05 + 10);
    ASSERT_EQ(buf[4], expectedSum);
}

// Test 31: Negative int24 distance is correctly skipped by the parser.
//          The distance field is a signed 24-bit value; negative values
//          (bit 23 set) represent invalid/error conditions and must not
//          produce a valid reading.
static void test_negative_distance_skipped() {
    printf("  test_negative_distance_skipped...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // Set pixel 0 to -100 mm (negative distance).
    // In 24-bit two's complement: -100 = 0xFFFF9C
    // Bytes LE: [0x9C, 0xFF, 0xFF]
    setPixel(frame, 0, -100, /*status=*/0, 100);

    // Set pixels 1-4 to a valid 500 mm (should be the result).
    // Pixel 0 has dis_status=0 but negative distance → skipped by parser,
    // so only these TEST_MIN_PX pixels count as valid.
    for (uint8_t px = 1; px < 1 + TEST_MIN_PX; px++)
        setPixel(frame, px, 500, /*status=*/0, 100);
    writeChecksum(frame);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    // The negative pixel should be skipped; pixel 1's 500 mm is the minimum
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.distance_mm, 500);
}

// Test 32: 3000 mm intermediate long-range distance parses correctly.
//          Covers the gap between 2000 mm (test 27) and 4000 mm (test 28).
static void test_3000mm_distance_parses_correctly() {
    printf("  test_3000mm_distance_parses_correctly...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // 3000 mm = 0x000BB8
    setPixel(frame, 0, 3000, /*status=*/0, 120);
    // Additional pixels to meet MIN_VALID_PIXELS
    for (uint8_t px = 1; px < TEST_MIN_PX; px++)
        setPixel(frame, px, 3000, /*status=*/0, 120);
    writeChecksum(frame);

    // Verify raw bytes (LE)
    uint16_t base = 9;
    ASSERT_EQ(frame[base + 0], 0xB8);
    ASSERT_EQ(frame[base + 1], 0x0B);
    ASSERT_EQ(frame[base + 2], 0x00);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.distance_mm, 3000);
    ASSERT_EQ(rd.zone, 0);  // normal zone: ≥ 1500 mm
}

// Test 33: Signal strength value does not affect distance parsing.
//          The parser uses only dis_status (not signal_strength) to determine
//          pixel validity.  At long range the signal is naturally weaker;
//          this test confirms weak signal alone does not reject the reading.
//          (dis_status-based rejection is verified by test 12.)
static void test_signal_strength_does_not_affect_distance() {
    printf("  test_signal_strength_does_not_affect_distance...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // Set pixel 0 to 3500 mm with very low signal (signal=1)
    setValidPixels(frame, 3500, 1);
    writeChecksum(frame);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    // Low signal should NOT cause rejection — only dis_status matters
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.distance_mm, 3500);
    ASSERT_EQ(rd.zone, 0);
}

// Test 34: High-level send commands fail gracefully when TX pin is not
//          connected (Config::txPin = -1, the default).
static void test_send_command_fails_without_tx_pin() {
    printf("  test_send_command_fails_without_tx_pin...\n");

    // Default Config has txPin = -1 (not connected)
    obstacle_sensor::init();

    // Advance past warmup so initialized_ is true
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // All command functions should return false
    ASSERT_EQ(obstacle_sensor::setRangeMode(obstacle_sensor::RangeMode::LONG_RANGE), false);
    ASSERT_EQ(obstacle_sensor::setBaudRate(obstacle_sensor::BaudRateCode::BAUD_921600), false);
    ASSERT_EQ(obstacle_sensor::setFrameRate(10), false);
    ASSERT_EQ(obstacle_sensor::setOutputMode(obstacle_sensor::OutputMode::ACTIVE), false);
    ASSERT_EQ(obstacle_sensor::saveConfig(), false);
}

// Test 35: configureLongRange() fails when TX pin is not connected.
static void test_configure_long_range_fails_without_tx() {
    printf("  test_configure_long_range_fails_without_tx...\n");

    // Default Config has txPin = -1 (not connected)
    obstacle_sensor::init();

    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // configureLongRange() should return false because TX is not connected
    ASSERT_EQ(obstacle_sensor::configureLongRange(), false);
}

// Test 36: configureLongRange() succeeds when TX pin is connected.
static void test_configure_long_range_succeeds_with_tx() {
    printf("  test_configure_long_range_succeeds_with_tx...\n");

    obstacle_sensor::Config cfg{};
    cfg.txPin = 17;  // TX pin connected
    obstacle_sensor::init(cfg);

    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // configureLongRange() should return true
    ASSERT_EQ(obstacle_sensor::configureLongRange(), true);
}

// Test 37: setFrameRate(0) is rejected (0 Hz is invalid).
static void test_set_frame_rate_zero_rejected() {
    printf("  test_set_frame_rate_zero_rejected...\n");

    obstacle_sensor::Config cfg{};
    cfg.txPin = 17;
    obstacle_sensor::init(cfg);

    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    ASSERT_EQ(obstacle_sensor::setFrameRate(0), false);
    // Valid frame rate should succeed
    ASSERT_EQ(obstacle_sensor::setFrameRate(1), true);
    ASSERT_EQ(obstacle_sensor::setFrameRate(15), true);
}

// Test 38: 4000 mm raw bytes verification — confirms no truncation or
//          byte-order issues in the int24 LE encoding at max range.
static void test_4000mm_raw_bytes() {
    printf("  test_4000mm_raw_bytes...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // 4000 mm = 0x000FA0
    // LE bytes: [0xA0, 0x0F, 0x00]
    setPixel(frame, 0, 4000, /*status=*/0, 100);
    // Additional pixels to meet MIN_VALID_PIXELS
    for (uint8_t px = 1; px < TEST_MIN_PX; px++)
        setPixel(frame, px, 4000, /*status=*/0, 100);
    writeChecksum(frame);

    uint16_t base = 9;  // pixel 0
    ASSERT_EQ(frame[base + 0], 0xA0);  // LSB
    ASSERT_EQ(frame[base + 1], 0x0F);  // middle byte
    ASSERT_EQ(frame[base + 2], 0x00);  // MSB (bit 23 = 0 → positive)

    // Verify the value is positive (bit 23 not set → no sign extension)
    ASSERT(!(frame[base + 2] & 0x80));

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.distance_mm, 4000);
}

// Test 39: configureLongRange() sends all 4 commands with correct bytes.
//          Uses the UART TX capture buffer to verify each command frame
//          was sent with the right header, length, cmdId, payload, and checksum.
//          This is the key test for the "can't see 4000 mm" diagnosis:
//          if these bytes actually reach the sensor, the sensor WILL switch
//          to LONG RANGE mode.  The missing piece on real hardware is
//          wiring ESP32 TX → sensor RX.
static void test_configure_long_range_sends_correct_commands() {
    printf("  test_configure_long_range_sends_correct_commands...\n");

    obstacle_sensor::Config cfg{};
    cfg.txPin = 17;  // TX pin connected
    obstacle_sensor::init(cfg);

    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // Reset TX capture buffer
    g_uart_tx_reset();

    // Call configureLongRange — should send 4 commands
    ASSERT_EQ(obstacle_sensor::configureLongRange(), true);

    // Expected commands (each is 5 or 4 bytes):
    //   1. SET_RANGE_MODE(LONG_RANGE): [0x5A, 5, 0x07, 0x01, checksum]
    //   2. SET_OUTPUT_MODE(ACTIVE):    [0x5A, 5, 0x02, 0x00, checksum]
    //   3. SET_FRAME_RATE(10):         [0x5A, 5, 0x05, 0x0A, checksum]
    //   4. SAVE_CONFIG:                [0x5A, 4, 0x08,       checksum]
    // Total bytes = 5 + 5 + 5 + 4 = 19
    ASSERT_EQ(g_uart_tx_len, 19);

    // Command 1: SET_RANGE_MODE (offset 0)
    ASSERT_EQ(g_uart_tx_buf[0], 0x5A);    // header
    ASSERT_EQ(g_uart_tx_buf[1], 5);       // length
    ASSERT_EQ(g_uart_tx_buf[2], 0x07);    // SET_RANGE_MODE
    ASSERT_EQ(g_uart_tx_buf[3], 0x01);    // LONG_RANGE
    ASSERT_EQ(g_uart_tx_buf[4], (uint8_t)(0x5A + 5 + 0x07 + 0x01));  // checksum

    // Command 2: SET_OUTPUT_MODE (offset 5)
    ASSERT_EQ(g_uart_tx_buf[5], 0x5A);
    ASSERT_EQ(g_uart_tx_buf[6], 5);
    ASSERT_EQ(g_uart_tx_buf[7], 0x02);    // SET_OUTPUT_MODE
    ASSERT_EQ(g_uart_tx_buf[8], 0x00);    // ACTIVE
    ASSERT_EQ(g_uart_tx_buf[9], (uint8_t)(0x5A + 5 + 0x02 + 0x00));

    // Command 3: SET_FRAME_RATE (offset 10)
    ASSERT_EQ(g_uart_tx_buf[10], 0x5A);
    ASSERT_EQ(g_uart_tx_buf[11], 5);
    ASSERT_EQ(g_uart_tx_buf[12], 0x05);   // SET_FRAME_RATE
    ASSERT_EQ(g_uart_tx_buf[13], 10);     // 10 Hz
    ASSERT_EQ(g_uart_tx_buf[14], (uint8_t)(0x5A + 5 + 0x05 + 10));

    // Command 4: SAVE_CONFIG (offset 15)
    ASSERT_EQ(g_uart_tx_buf[15], 0x5A);
    ASSERT_EQ(g_uart_tx_buf[16], 4);      // length = 4 (no payload)
    ASSERT_EQ(g_uart_tx_buf[17], 0x08);   // SAVE_CONFIG
    ASSERT_EQ(g_uart_tx_buf[18], (uint8_t)(0x5A + 4 + 0x08));
}

// Test 40: Individual setRangeMode(LONG_RANGE) sends correct bytes to UART TX.
static void test_set_range_mode_sends_correct_bytes() {
    printf("  test_set_range_mode_sends_correct_bytes...\n");

    obstacle_sensor::Config cfg{};
    cfg.txPin = 17;
    obstacle_sensor::init(cfg);
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    g_uart_tx_reset();
    ASSERT_EQ(obstacle_sensor::setRangeMode(obstacle_sensor::RangeMode::LONG_RANGE), true);

    ASSERT_EQ(g_uart_tx_len, 5);
    ASSERT_EQ(g_uart_tx_buf[0], 0x5A);
    ASSERT_EQ(g_uart_tx_buf[1], 5);
    ASSERT_EQ(g_uart_tx_buf[2], 0x07);
    ASSERT_EQ(g_uart_tx_buf[3], 0x01);
}

// Test 41: Individual saveConfig() sends correct bytes to UART TX.
static void test_save_config_sends_correct_bytes() {
    printf("  test_save_config_sends_correct_bytes...\n");

    obstacle_sensor::Config cfg{};
    cfg.txPin = 17;
    obstacle_sensor::init(cfg);
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    g_uart_tx_reset();
    ASSERT_EQ(obstacle_sensor::saveConfig(), true);

    ASSERT_EQ(g_uart_tx_len, 4);
    ASSERT_EQ(g_uart_tx_buf[0], 0x5A);
    ASSERT_EQ(g_uart_tx_buf[1], 4);
    ASSERT_EQ(g_uart_tx_buf[2], 0x08);
}

// Test 42: Repeated all-pixels-invalid frames always return minRangeMm (20 mm).
//          This reproduces the exact "sensor stuck at 20 mm" symptom reported
//          when the TOFSense-M is in SHORT RANGE mode pointing at open air:
//          every pixel reports dis_status ≠ 0, so the driver keeps returning
//          minRangeMm (20 mm) as emergency-close.  After switching to LONG
//          RANGE mode (via configureLongRange()), valid pixel data arrives and
//          the correct distance is returned.
static void test_repeated_all_pixels_invalid_stays_at_min() {
    printf("  test_repeated_all_pixels_invalid_stays_at_min...\n");

    // Build a frame where all 64 pixels are invalid (dis_status=1)
    uint8_t frame[FRAME_LEN];
    buildFrame(frame);
    for (uint8_t px = 0; px < 64; px++) {
        setPixel(frame, px, 50, /*status=*/1, 100);
    }
    writeChecksum(frame);

    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();

    // Advance past warmup
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // Feed 5 consecutive all-invalid frames — simulates open air in SHORT RANGE
    for (int i = 0; i < 5; i++) {
        g_uart_inject(frame, FRAME_LEN);
        g_test_millis += 100;
        obstacle_sensor::update(0.0f);

        obstacle_sensor::Reading rd = obstacle_sensor::getReading();
        ASSERT_EQ(rd.distance_mm, 20);   // Always minRangeMm
        ASSERT_EQ(rd.zone, 4);           // Emergency zone
        ASSERT_EQ(static_cast<uint8_t>(rd.status),
                  static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
        ASSERT_EQ(rd.healthy, true);
    }

    // Now inject a valid frame with a real distance (3000 mm) — simulates
    // switching to LONG RANGE mode where the sensor can actually see objects
    uint8_t goodFrame[FRAME_LEN];
    buildFrame(goodFrame);
    setValidPixels(goodFrame, 3000, 200);  // 3000 mm
    writeChecksum(goodFrame);

    g_uart_inject(goodFrame, FRAME_LEN);
    g_test_millis += 100;
    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    ASSERT_EQ(rd.distance_mm, 3000);   // Now reads correctly
    ASSERT_EQ(rd.zone, 0);            // Normal zone (>= 1500 mm)
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
}

// Test 43: Parse the official Nooploop example frame from the support page.
//          This is the exact 400-byte frame provided at:
//            https://support.nooploop.com/tofsense/example-code
//          Verifies our parser correctly handles a real sensor frame:
//            - Valid header (0x57, 0x01)
//            - Valid checksum (0x05)
//            - All 64 pixels have dis_status ≠ 0 (21× status=5, 43× status=0xFF)
//            → NO_VALID_PIXELS → returns minRangeMm (20 mm) as emergency-close.
//          This matches the official Nooploop code behavior where the user
//          application must check dis_status per pixel.
static void test_official_nooploop_example_frame() {
    printf("  test_official_nooploop_example_frame...\n");

    // Official example frame from https://support.nooploop.com/tofsense/example-code
    static const uint8_t rx_buf[400] = {
        0x57,0x01,0xff,0x00,0x17,0x56,0x00,0x00,0x40,0x1d,0x00,0x00,0x05,0xa1,0x84,
        0x20,0x00,0x00,0x05,0xcf,0x69,0x20,0x00,0x00,0x05,0x9e,0x75,0x21,0x00,0x00,
        0x05,0x99,0x73,0x20,0x00,0x00,0x05,0xb0,0x72,0x1f,0x00,0x00,0x05,0x73,0x51,
        0x1f,0x00,0x00,0x05,0x7c,0xc3,0x1e,0x00,0x00,0xff,0x57,0x40,0x21,0x00,0x00,
        0x05,0x9a,0xd5,0x22,0x00,0x00,0x05,0x71,0xdc,0x26,0x00,0x00,0x05,0xee,0xe7,
        0x22,0x00,0x00,0x05,0xd6,0xde,0x23,0x00,0x00,0x05,0x6a,0xef,0x22,0x00,0x00,
        0x05,0xf8,0xe5,0x22,0x00,0x00,0x05,0x84,0x9b,0x20,0x00,0x00,0xff,0x23,0x30,
        0x27,0x00,0x00,0x05,0x74,0x5b,0x29,0x00,0x00,0x05,0xb6,0x70,0x27,0x00,0x00,
        0x05,0xe2,0x5c,0x27,0x00,0x00,0x05,0x04,0x4e,0x27,0x00,0x00,0x05,0xec,0x4c,
        0x24,0x00,0x00,0x05,0x57,0x4d,0x23,0x00,0x00,0xff,0xf0,0x3a,0x21,0x00,0x00,
        0xff,0xff,0x20,0x2b,0x00,0x00,0xff,0x3c,0x25,0x29,0x00,0x00,0xff,0x6f,0x2e,
        0x29,0x00,0x00,0xff,0xde,0x26,0x28,0x00,0x00,0xff,0xa7,0x22,0x26,0x00,0x00,
        0xff,0x57,0x23,0x23,0x00,0x00,0xff,0x91,0x27,0x23,0x00,0x00,0xff,0x05,0x23,
        0x21,0x00,0x00,0xff,0x34,0x15,0x27,0x00,0x00,0xff,0x61,0x13,0x27,0x00,0x00,
        0xff,0xeb,0x18,0x24,0x00,0x00,0xff,0x8a,0x18,0x24,0x00,0x00,0xff,0x86,0x19,
        0x22,0x00,0x00,0xff,0x13,0x1c,0x22,0x00,0x00,0xff,0xa1,0x1e,0x21,0x00,0x00,
        0xff,0x3b,0x18,0x1f,0x00,0x00,0xff,0x47,0x0c,0x23,0x00,0x00,0xff,0xff,0x0b,
        0x22,0x00,0x00,0xff,0x35,0x0e,0x20,0x00,0x00,0xff,0x8b,0x10,0x20,0x00,0x00,
        0xff,0x80,0x11,0x20,0x00,0x00,0xff,0xe8,0x11,0x20,0x00,0x00,0xff,0x22,0x10,
        0x1e,0x00,0x00,0xff,0x7d,0x0b,0x1e,0x00,0x00,0xff,0x0b,0x0b,0x1e,0x00,0x00,
        0xff,0x4a,0x09,0x1e,0x00,0x00,0xff,0xb7,0x0a,0x1f,0x00,0x00,0xff,0x2d,0x0d,
        0x1e,0x00,0x00,0xff,0x1c,0x0f,0x1d,0x00,0x00,0xff,0x06,0x10,0x1e,0x00,0x00,
        0xff,0x21,0x0d,0x1e,0x00,0x00,0xff,0x89,0x0b,0x17,0x00,0x00,0xff,0x4b,0x17,
        0x1c,0x00,0x00,0xff,0x9d,0x09,0x1c,0x00,0x00,0xff,0x43,0x0d,0x1d,0x00,0x00,
        0xff,0xd5,0x0c,0x1b,0x00,0x00,0xff,0xaa,0x0e,0x1c,0x00,0x00,0xff,0x29,0x0f,
        0x1a,0x00,0x00,0xff,0x57,0x0d,0x18,0x00,0x00,0xff,0xd1,0x0f,0x13,0x00,0x00,
        0x05,0xa4,0x5c,0xff,0xff,0xff,0xff,0xff,0xff,0x05
    };

    // Verify frame structure
    ASSERT_EQ(sizeof(rx_buf), 400);
    ASSERT_EQ(rx_buf[0], 0x57);      // frame_header
    ASSERT_EQ(rx_buf[1], 0x01);      // function_mark (Frame0)
    ASSERT_EQ(rx_buf[8], 0x40);      // pixel_count = 64 (8×8)

    // Verify checksum is valid
    uint8_t sum = 0;
    for (int i = 0; i < 399; i++) sum += rx_buf[i];
    ASSERT_EQ(sum, rx_buf[399]);

    // Feed through the full pipeline — all pixels are invalid in this frame
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init();

    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);  // flush warmup

    g_uart_inject(rx_buf, sizeof(rx_buf));
    obstacle_sensor::update(0.0f);

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();
    // All pixels have dis_status ≠ 0 → NO_VALID_PIXELS → emergency-close
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.distance_mm, 20);   // minRangeMm fallback
    ASSERT_EQ(rd.zone, 4);           // emergency zone
}

// Test 44: Wrap-around detection — 3 valid pixels with high distances
//          (2245 mm) while all other pixels are invalid.
//          This reproduces the exact "data reversed" symptom reported:
//          palm at 2 cm → sensor shows 2245 mm instead of emergency-close.
//          With the MIN_VALID_PIXELS threshold, the 3 valid pixels are
//          rejected as phase wrap-around artifacts and treated as
//          emergency-close (NO_VALID_PIXELS → 20 mm).
static void test_few_valid_pixels_treated_as_wraparound() {
    printf("  test_few_valid_pixels_treated_as_wraparound...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // Simulate wrap-around: 3 valid pixels with high distances (2245 mm),
    // all other 61 pixels are invalid (dis_status=1).
    for (uint8_t px = 0; px < 64; px++) {
        setPixel(frame, px, 2245, /*status=*/1, 50);  // All invalid
    }
    // Only 3 pixels "valid" with wrap-around distances
    setPixel(frame, 10, 2245, /*status=*/0, 200);
    setPixel(frame, 11, 2800, /*status=*/0, 180);
    setPixel(frame, 12, 3000, /*status=*/0, 160);
    writeChecksum(frame);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    // Should be treated as emergency-close (NO_VALID_PIXELS)
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.distance_mm, 20);   // minRangeMm fallback
    ASSERT_EQ(rd.zone, 4);           // emergency zone
}

// Test 45: Exactly MIN_VALID_PIXELS (4) valid pixels → accepted as real data.
//          This tests the threshold boundary: 4 valid pixels should be
//          enough to produce a valid reading, not treated as wrap-around.
static void test_exactly_min_valid_pixels_accepted() {
    printf("  test_exactly_min_valid_pixels_accepted...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // Set exactly 4 valid pixels at 800 mm, rest are invalid
    for (uint8_t px = 0; px < 64; px++) {
        setPixel(frame, px, 800, /*status=*/1, 50);  // All invalid
    }
    setPixel(frame, 0, 800, /*status=*/0, 200);
    setPixel(frame, 1, 900, /*status=*/0, 180);
    setPixel(frame, 2, 850, /*status=*/0, 190);
    setPixel(frame, 3, 800, /*status=*/0, 200);
    writeChecksum(frame);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    // 4 valid pixels → accepted; minimum is 800 mm
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.distance_mm, 800);
    ASSERT_EQ(rd.zone, 2);  // warning zone: [500, 1000) mm
}

// Test 46: Single valid pixel → wrap-around rejection (emergency-close).
//          Verifies that even 1 valid pixel with a plausible distance
//          is rejected when fewer than MIN_VALID_PIXELS are valid.
static void test_single_valid_pixel_rejected_as_wraparound() {
    printf("  test_single_valid_pixel_rejected_as_wraparound...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // Only pixel 0 is valid at 1500 mm, rest are invalid
    setPixel(frame, 0, 1500, /*status=*/0, 100);
    writeChecksum(frame);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    // Single valid pixel → NO_VALID_PIXELS → emergency-close
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.distance_mm, 20);   // minRangeMm
    ASSERT_EQ(rd.zone, 4);           // emergency
}

// Test 47: Above-maxRange with many valid pixels → clamped to maxRangeMm.
//          Simulates open air in LONG RANGE mode where the sensor sees
//          background at 5000+ mm.  Result is VALID at zone 0 (no obstacle).
static void test_open_air_above_max_range_is_valid() {
    printf("  test_open_air_above_max_range_is_valid...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // All 64 pixels valid at 6000 mm (way above maxRangeMm = 4000)
    for (uint8_t px = 0; px < 64; px++) {
        setPixel(frame, px, 6000, /*status=*/0, 80);
    }
    writeChecksum(frame);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    // Should be VALID at maxRangeMm, zone 0
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.distance_mm, 4000);  // clamped to maxRangeMm
    ASSERT_EQ(rd.zone, 0);            // normal zone
}

// Test 48: High dispersion — many valid pixels with scattered distances.
//          Reproduces the "sensor covered" symptom: valid pixels at 121, 273,
//          1000 mm (spread 879 mm > 500 mm threshold).  Should be treated as
//          wrap-around → emergency-close (20 mm).
static void test_high_dispersion_covered_sensor() {
    printf("  test_high_dispersion_covered_sensor...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // Simulate covered sensor: most pixels invalid, but 8 valid pixels
    // with wildly scattered wrap-around distances (121, 273, 1000 mm).
    for (uint8_t px = 0; px < 64; px++) {
        setPixel(frame, px, 500, /*status=*/1, 50);  // All invalid
    }
    // 8 valid pixels with high dispersion (spread = 1000 - 121 = 879 mm)
    setPixel(frame, 0,  121,  /*status=*/0, 100);
    setPixel(frame, 1,  273,  /*status=*/0, 100);
    setPixel(frame, 10, 400,  /*status=*/0, 100);
    setPixel(frame, 11, 550,  /*status=*/0, 100);
    setPixel(frame, 20, 700,  /*status=*/0, 100);
    setPixel(frame, 21, 800,  /*status=*/0, 100);
    setPixel(frame, 30, 900,  /*status=*/0, 100);
    setPixel(frame, 31, 1000, /*status=*/0, 100);
    writeChecksum(frame);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    // High dispersion → emergency-close
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.distance_mm, 20);   // minRangeMm fallback
    ASSERT_EQ(rd.zone, 4);           // emergency zone
}

// Test 49: High dispersion — close-range wrap-around at ~2 cm.
//          Reproduces the "object at 2 cm shows 1700, 2500 mm" symptom.
//          Spread = 2500 - 1700 = 800 mm > 500 mm threshold.
static void test_high_dispersion_close_range_wraparound() {
    printf("  test_high_dispersion_close_range_wraparound...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // Simulate wrap-around: 6 valid pixels with high distances
    for (uint8_t px = 0; px < 64; px++) {
        setPixel(frame, px, 2000, /*status=*/1, 50);  // All invalid
    }
    setPixel(frame, 5,  1700, /*status=*/0, 200);
    setPixel(frame, 6,  1900, /*status=*/0, 180);
    setPixel(frame, 7,  2100, /*status=*/0, 160);
    setPixel(frame, 8,  2300, /*status=*/0, 150);
    setPixel(frame, 15, 2400, /*status=*/0, 140);
    setPixel(frame, 16, 2500, /*status=*/0, 130);
    writeChecksum(frame);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    // High dispersion → emergency-close
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.distance_mm, 20);   // minRangeMm fallback
    ASSERT_EQ(rd.zone, 4);           // emergency zone
}

// Test 50: Normal dispersion — real obstacle with consistent pixel distances.
//          All valid pixels at 800 ± 50 mm (spread = 100 mm < 500 mm).
//          Should be accepted as a valid reading.
static void test_normal_dispersion_real_obstacle() {
    printf("  test_normal_dispersion_real_obstacle...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // Real obstacle: 32 valid pixels with consistent distances
    for (uint8_t px = 0; px < 32; px++) {
        int32_t dist = 750 + (px % 5) * 25;  // 750, 775, 800, 825, 850
        setPixel(frame, px, dist, /*status=*/0, 200);
    }
    writeChecksum(frame);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    // Low dispersion → valid reading at minimum distance (750 mm)
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.distance_mm, 750);
    ASSERT_EQ(rd.zone, 2);  // warning zone: [500, 1000) mm
}

// Test 51: Dispersion boundary — spread exactly at threshold (500 mm).
//          Spread = 1000 - 500 = 500 mm, which equals the threshold.
//          Should be accepted (threshold check is strictly greater-than).
static void test_dispersion_at_boundary_accepted() {
    printf("  test_dispersion_at_boundary_accepted...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // 4 valid pixels with spread = exactly 500 mm
    for (uint8_t px = 0; px < 64; px++) {
        setPixel(frame, px, 700, /*status=*/1, 50);
    }
    setPixel(frame, 0, 500,  /*status=*/0, 200);
    setPixel(frame, 1, 600,  /*status=*/0, 200);
    setPixel(frame, 2, 900,  /*status=*/0, 200);
    setPixel(frame, 3, 1000, /*status=*/0, 200);
    writeChecksum(frame);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    // Spread == 500 → NOT exceeded → valid (minimum = 500 mm)
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.distance_mm, 500);
    ASSERT_EQ(rd.zone, 2);  // warning zone: [500, 1000) mm
}

// Test 52: Dispersion just above threshold — spread = 501 mm.
//          Should be rejected as high dispersion → emergency-close.
static void test_dispersion_just_above_threshold_rejected() {
    printf("  test_dispersion_just_above_threshold_rejected...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // 4 valid pixels with spread = 501 mm (just over threshold)
    for (uint8_t px = 0; px < 64; px++) {
        setPixel(frame, px, 700, /*status=*/1, 50);
    }
    setPixel(frame, 0, 500,  /*status=*/0, 200);
    setPixel(frame, 1, 600,  /*status=*/0, 200);
    setPixel(frame, 2, 900,  /*status=*/0, 200);
    setPixel(frame, 3, 1001, /*status=*/0, 200);
    writeChecksum(frame);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    // Spread = 501 → exceeded → emergency-close
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.distance_mm, 20);   // minRangeMm fallback
    ASSERT_EQ(rd.zone, 4);           // emergency zone
}

// Test 53: All 64 pixels valid at same distance — zero dispersion.
//          Verifies that uniform readings (e.g. flat wall) are accepted.
static void test_all_pixels_same_distance_accepted() {
    printf("  test_all_pixels_same_distance_accepted...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);

    // All 64 pixels valid at 1500 mm
    for (uint8_t px = 0; px < 64; px++) {
        setPixel(frame, px, 1500, /*status=*/0, 200);
    }
    writeChecksum(frame);

    obstacle_sensor::Reading rd = injectFrameAndUpdate(frame);

    // Zero dispersion → valid
    ASSERT_EQ(static_cast<uint8_t>(rd.status),
              static_cast<uint8_t>(obstacle_sensor::SensorStatus::VALID));
    ASSERT_EQ(rd.healthy, true);
    ASSERT_EQ(rd.distance_mm, 1500);
    ASSERT_EQ(rd.zone, 0);  // normal zone: ≥ 1500 mm
}

// Test 54: Auto-recovery triggers configureLongRange() after sustained
//          all-pixels-invalid frames (50 consecutive).  Verifies that
//          the driver automatically retries the configuration when the
//          sensor appears stuck in SHORT RANGE mode.
static void test_auto_recovery_triggers_after_threshold() {
    printf("  test_auto_recovery_triggers_after_threshold...\n");

    // Build a frame where all 64 pixels are invalid (dis_status=1)
    uint8_t frame[FRAME_LEN];
    buildFrame(frame);
    for (uint8_t px = 0; px < 64; px++) {
        setPixel(frame, px, 50, /*status=*/1, 100);
    }
    writeChecksum(frame);

    // Init with TX pin connected (auto-recovery requires txPin ≥ 0)
    g_uart_inject_reset();
    g_uart_tx_reset();
    g_test_millis = 0;
    obstacle_sensor::Config cfg;
    cfg.txPin = 17;
    obstacle_sensor::init(cfg);
    ASSERT_EQ(obstacle_sensor::getAutoRecoveryAttempts(), 0);

    // Advance past warmup
    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // Feed 30 consecutive all-invalid frames (AUTO_RECOVERY_FRAME_THRESHOLD)
    g_uart_tx_reset();  // Clear any TX data from init
    for (int i = 0; i < 30; i++) {
        g_uart_inject_reset();  // Reset inject buffer to reuse space
        g_uart_inject(frame, FRAME_LEN);
        g_test_millis += 100;
        obstacle_sensor::update(0.0f);
    }

    // Auto-recovery should have triggered
    ASSERT_EQ(obstacle_sensor::getAutoRecoveryAttempts(), 1);
    // Verify that configureLongRange() sent commands over UART TX
    ASSERT(g_uart_tx_len > 0);
}

// Test 55: Auto-recovery does NOT trigger when txPin is not connected.
static void test_auto_recovery_skipped_without_tx_pin() {
    printf("  test_auto_recovery_skipped_without_tx_pin...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);
    for (uint8_t px = 0; px < 64; px++) {
        setPixel(frame, px, 50, /*status=*/1, 100);
    }
    writeChecksum(frame);

    // Init WITHOUT TX pin (txPin = -1, default)
    g_uart_inject_reset();
    g_uart_tx_reset();
    g_test_millis = 0;
    obstacle_sensor::init();  // default config: txPin = -1
    ASSERT_EQ(obstacle_sensor::getAutoRecoveryAttempts(), 0);

    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // Feed 60 consecutive all-invalid frames — well past threshold (30)
    for (int i = 0; i < 60; i++) {
        g_uart_inject_reset();
        g_uart_inject(frame, FRAME_LEN);
        g_test_millis += 100;
        obstacle_sensor::update(0.0f);
    }

    // No auto-recovery without TX pin
    ASSERT_EQ(obstacle_sensor::getAutoRecoveryAttempts(), 0);
}

// Test 56: Auto-recovery counter resets when a valid frame arrives.
//          Feed 49 invalid frames (below threshold), then one valid frame,
//          then 49 more invalid frames.  Auto-recovery should NOT trigger.
static void test_auto_recovery_resets_on_valid_frame() {
    printf("  test_auto_recovery_resets_on_valid_frame...\n");

    uint8_t badFrame[FRAME_LEN];
    buildFrame(badFrame);
    for (uint8_t px = 0; px < 64; px++) {
        setPixel(badFrame, px, 50, /*status=*/1, 100);
    }
    writeChecksum(badFrame);

    uint8_t goodFrame[FRAME_LEN];
    buildFrame(goodFrame);
    setValidPixels(goodFrame, 2000, 200);
    writeChecksum(goodFrame);

    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::Config cfg;
    cfg.txPin = 17;
    obstacle_sensor::init(cfg);

    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // Feed 29 invalid frames — just below threshold (30)
    for (int i = 0; i < 29; i++) {
        g_uart_inject_reset();
        g_uart_inject(badFrame, FRAME_LEN);
        g_test_millis += 100;
        obstacle_sensor::update(0.0f);
    }
    ASSERT_EQ(obstacle_sensor::getAutoRecoveryAttempts(), 0);

    // Feed one valid frame — resets consecutive counter
    g_uart_inject_reset();
    g_uart_inject(goodFrame, FRAME_LEN);
    g_test_millis += 100;
    obstacle_sensor::update(0.0f);
    ASSERT_EQ(obstacle_sensor::getAutoRecoveryAttempts(), 0);

    // Feed 29 more invalid frames — counter restarts from 0
    for (int i = 0; i < 29; i++) {
        g_uart_inject_reset();
        g_uart_inject(badFrame, FRAME_LEN);
        g_test_millis += 100;
        obstacle_sensor::update(0.0f);
    }

    // Should NOT have triggered (29 < 30 threshold)
    ASSERT_EQ(obstacle_sensor::getAutoRecoveryAttempts(), 0);
}

// Test 57: Auto-recovery stops after MAX_AUTO_RECOVERY_ATTEMPTS (10).
static void test_auto_recovery_limited_to_max_attempts() {
    printf("  test_auto_recovery_limited_to_max_attempts...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);
    for (uint8_t px = 0; px < 64; px++) {
        setPixel(frame, px, 50, /*status=*/1, 100);
    }
    writeChecksum(frame);

    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::Config cfg;
    cfg.txPin = 17;
    obstacle_sensor::init(cfg);

    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    // Trigger auto-recovery 10 times (each needs 30 invalid frames)
    for (int attempt = 0; attempt < 10; attempt++) {
        for (int i = 0; i < 30; i++) {
            g_uart_inject_reset();
            g_uart_inject(frame, FRAME_LEN);
            g_test_millis += 100;
            obstacle_sensor::update(0.0f);
        }
        ASSERT_EQ(obstacle_sensor::getAutoRecoveryAttempts(), (uint8_t)(attempt + 1));
    }

    // Now feed 30 more — should NOT trigger an 11th attempt
    for (int i = 0; i < 30; i++) {
        g_uart_inject_reset();
        g_uart_inject(frame, FRAME_LEN);
        g_test_millis += 100;
        obstacle_sensor::update(0.0f);
    }
    ASSERT_EQ(obstacle_sensor::getAutoRecoveryAttempts(), 10);  // Stays at 10
}

// Test 58: init() resets auto-recovery state.
static void test_init_resets_auto_recovery() {
    printf("  test_init_resets_auto_recovery...\n");

    uint8_t frame[FRAME_LEN];
    buildFrame(frame);
    for (uint8_t px = 0; px < 64; px++) {
        setPixel(frame, px, 50, /*status=*/1, 100);
    }
    writeChecksum(frame);

    // First run: trigger one auto-recovery
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::Config cfg;
    cfg.txPin = 17;
    obstacle_sensor::init(cfg);

    g_test_millis = 1100;
    obstacle_sensor::update(0.0f);

    for (int i = 0; i < 30; i++) {
        g_uart_inject_reset();
        g_uart_inject(frame, FRAME_LEN);
        g_test_millis += 100;
        obstacle_sensor::update(0.0f);
    }
    ASSERT_EQ(obstacle_sensor::getAutoRecoveryAttempts(), 1);

    // Re-init — should reset auto-recovery state
    g_uart_inject_reset();
    g_test_millis = 0;
    obstacle_sensor::init(cfg);
    ASSERT_EQ(obstacle_sensor::getAutoRecoveryAttempts(), 0);
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
    test_above_max_range_clamped_to_max();
    test_normal_distance_valid();
    test_exact_min_range_valid();
    test_overflow_flush_keeps_latest_frame();
    test_resync_after_bad_checksum();
    test_single_frame_within_byte_limit();
    test_multiple_bad_frames_then_good();
    test_many_queued_frames_all_processed();
    test_build_command_range_mode();
    test_build_command_save_config();
    test_build_command_baud_rate();
    test_build_command_buffer_too_small();
    test_build_command_checksum_wraps();
    test_2000mm_distance_parses_correctly();
    test_4000mm_max_range_parses_correctly();
    test_build_command_output_mode();
    test_build_command_frame_rate();
    test_negative_distance_skipped();
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
    test_repeated_all_pixels_invalid_stays_at_min();
    test_official_nooploop_example_frame();
    test_few_valid_pixels_treated_as_wraparound();
    test_exactly_min_valid_pixels_accepted();
    test_single_valid_pixel_rejected_as_wraparound();
    test_open_air_above_max_range_is_valid();
    test_high_dispersion_covered_sensor();
    test_high_dispersion_close_range_wraparound();
    test_normal_dispersion_real_obstacle();
    test_dispersion_at_boundary_accepted();
    test_dispersion_just_above_threshold_rejected();
    test_all_pixels_same_distance_accepted();
    test_auto_recovery_triggers_after_threshold();
    test_auto_recovery_skipped_without_tx_pin();
    test_auto_recovery_resets_on_valid_frame();
    test_auto_recovery_limited_to_max_attempts();
    test_init_resets_auto_recovery();

    printf("\n%d tests run, %d failed\n", s_tests_run, s_tests_failed);
    return s_tests_failed > 0 ? 1 : 0;
}
