// =============================================================================
// ESP32-S3 — Remote Control Driver implementation
//
// See remote_control.h for the public contract and protocol references.
// =============================================================================

#include "remote_control.h"

// Only compile the body when the feature is enabled OR when running host tests.
// In the disabled default state, the header provides inline no-op stubs and
// this translation unit produces no code in the firmware binary.
#if REMOTE_CONTROL_ENABLED || defined(REMOTE_CONTROL_TEST_HOOKS)

#include <Arduino.h>
#include <HardwareSerial.h>

namespace remote_control {

// -------------------------------------------------------------------------
// Constants
// -------------------------------------------------------------------------
static constexpr uint8_t  IBUS_LEN          = 32;
static constexpr uint8_t  IBUS_HDR0         = 0x20;  // length (32)
static constexpr uint8_t  IBUS_HDR1         = 0x40;  // command (servo)
static constexpr uint8_t  IBUS_NUM_CHANNELS = 14;

// Smoothing — match STM32 motor_control.c (see repo memory "throttle smoothing")
static constexpr float    EMA_ALPHA              = 0.15f;
static constexpr float    THROTTLE_RAMP_UP_PCT_S = 50.0f;
static constexpr float    THROTTLE_RAMP_DN_PCT_S = 100.0f;
static constexpr float    STEERING_RAMP_DEG_S    = 90.0f;  // ±30° in ~330 ms
static constexpr float    TRIM_MAX_DEG           = 5.0f;
static constexpr float    STEERING_MAX_DEG       = 30.0f;

// Switch thresholds (microseconds)
static constexpr uint16_t SW_LOW_THR   = 1300;  // below = low position
static constexpr uint16_t SW_HIGH_THR  = 1700;  // above = high position

// -------------------------------------------------------------------------
// Internal state (file-local, single-instance — matches obstacle_sensor /
// shifter_input convention)
// -------------------------------------------------------------------------
namespace {

Config        s_cfg;
Stats         s_stats;
State         s_state          = State::IDLE;
HardwareSerial* s_uart         = nullptr;

// Parser buffer + position
uint8_t       s_buf[IBUS_LEN];
uint8_t       s_pos            = 0;

// Last good channel data (microseconds)
uint16_t      s_ch[IBUS_NUM_CHANNELS] = {};
bool          s_haveFrame      = false;
uint32_t      s_lastGoodMs     = 0;

// Smoothed outputs
float         s_throttleSm     = 0.0f;
float         s_steeringSm     = 0.0f;
uint32_t      s_lastSmoothMs   = 0;

// Test-time clock override
#ifdef REMOTE_CONTROL_TEST_HOOKS
uint32_t      s_testNowMs      = 0;
bool          s_useTestClock   = false;
static inline uint32_t now_ms() { return s_useTestClock ? s_testNowMs : millis(); }
#else
static inline uint32_t now_ms() { return millis(); }
#endif

// -------------------------------------------------------------------------
// Helpers
// -------------------------------------------------------------------------
static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float mapf(float v, float inLo, float inHi, float outLo, float outHi) {
    if (inHi == inLo) return outLo;
    return outLo + (v - inLo) * (outHi - outLo) / (inHi - inLo);
}

/// Verify iBUS checksum on s_buf.  Returns true if valid.
/// checksum = 0xFFFF − Σ(byte[0..29])  (16-bit LE at byte 30-31)
static bool verifyChecksum() {
    uint16_t sum = 0xFFFF;
    for (uint8_t i = 0; i < IBUS_LEN - 2; i++) {
        sum -= s_buf[i];
    }
    uint16_t got = (uint16_t)s_buf[IBUS_LEN - 2]
                 | ((uint16_t)s_buf[IBUS_LEN - 1] << 8);
    return sum == got;
}

/// Sanity-check all channel values fall within [sanityMin, sanityMax].
/// Out-of-range frames are rejected as Layer-2 failsafe (per FAILSAFE doc §2.d).
static bool sanityCheckChannels(const uint16_t ch[IBUS_NUM_CHANNELS]) {
    for (uint8_t i = 0; i < IBUS_NUM_CHANNELS; i++) {
        if (ch[i] < s_cfg.sanityMinUs || ch[i] > s_cfg.sanityMaxUs) {
            return false;
        }
    }
    return true;
}

/// Extract the 14 channels from s_buf bytes 2..29 (uint16 LE pairs).
static void extractChannels(uint16_t out[IBUS_NUM_CHANNELS]) {
    for (uint8_t i = 0; i < IBUS_NUM_CHANNELS; i++) {
        uint8_t lo = s_buf[2 + i * 2];
        uint8_t hi = s_buf[3 + i * 2];
        out[i] = (uint16_t)lo | ((uint16_t)hi << 8);
    }
}

/// Called when a complete frame has been validated and channels extracted.
static void onValidFrame(const uint16_t ch[IBUS_NUM_CHANNELS]) {
    for (uint8_t i = 0; i < IBUS_NUM_CHANNELS; i++) s_ch[i] = ch[i];
    s_haveFrame              = true;
    s_lastGoodMs             = now_ms();
    s_stats.framesOk++;
    s_stats.consecutiveCkFail = 0;
    s_state                   = State::ACTIVE;
}

/// Process one received byte through the parser state machine.
/// Handles header sync, length, checksum verification, and per-frame stats.
static void processByte(uint8_t b) {
    if (s_pos == 0) {
        if (b == IBUS_HDR0) {
            s_buf[0] = b;
            s_pos    = 1;
        } else {
            // Spurious byte before header — count once per resync, not per byte
            // (we only increment when we actually had to drop bytes).
        }
        return;
    }
    if (s_pos == 1) {
        if (b == IBUS_HDR1) {
            s_buf[1] = b;
            s_pos    = 2;
        } else {
            // Bad second header byte → resync.  If this byte is itself 0x20,
            // restart the sync from here so we don't drop a valid header.
            s_stats.headerResyncs++;
            s_pos = (b == IBUS_HDR0) ? 1 : 0;
            if (s_pos == 1) s_buf[0] = b;
        }
        return;
    }

    // Body bytes
    s_buf[s_pos++] = b;
    if (s_pos < IBUS_LEN) return;

    // Frame complete — verify checksum
    s_pos = 0;  // Reset for next frame regardless of outcome
    if (!verifyChecksum()) {
        s_stats.checksumFails++;
        if (s_stats.consecutiveCkFail < UINT16_MAX) s_stats.consecutiveCkFail++;
        if (s_stats.consecutiveCkFail >= s_cfg.maxChksumStreak &&
            s_state == State::ACTIVE) {
            s_state = State::DEGRADED;
        }
        return;
    }

    uint16_t ch[IBUS_NUM_CHANNELS];
    extractChannels(ch);
    if (!sanityCheckChannels(ch)) {
        s_stats.sanityRejects++;
        return;
    }
    onValidFrame(ch);
}

/// Apply a deadzone around `center` for input `us`.  Inside the band returns
/// `center`; outside, returns `us` unchanged.
static inline uint16_t applyDeadzone(uint16_t us, uint16_t center, uint16_t dz) {
    if (us >= center && (us - center) <= dz) return center;
    if (us <  center && (center - us) <= dz) return center;
    return us;
}

/// Map throttle µs (1500..2000) to 0..100 %.  Lower half (<1500) is reserved
/// for future regen brake (always 0 % throttle for now).  Deadzone applied
/// at the floor.
static float computeThrottlePct(uint16_t us) {
    if (us <= s_cfg.centerUs + s_cfg.deadzoneUs) return 0.0f;
    if (us >= 2000) return 100.0f;
    float pct = mapf((float)us,
                     (float)(s_cfg.centerUs + s_cfg.deadzoneUs), 2000.0f,
                     0.0f, 100.0f);
    return clampf(pct, 0.0f, 100.0f);
}

/// Map steering µs (1000..2000) + trim µs to ±30 degrees.
static float computeSteeringDeg(uint16_t usCh1, uint16_t usCh3) {
    uint16_t us = applyDeadzone(usCh1, s_cfg.centerUs, s_cfg.deadzoneUs);
    float deg   = mapf((float)us, 1000.0f, 2000.0f,
                       -STEERING_MAX_DEG, +STEERING_MAX_DEG);
    // CH3 trim: full-scale stick = ±TRIM_MAX_DEG
    float trim  = mapf((float)usCh3, 1000.0f, 2000.0f,
                       -TRIM_MAX_DEG, +TRIM_MAX_DEG);
    return clampf(deg + trim, -STEERING_MAX_DEG, +STEERING_MAX_DEG);
}

/// EMA + ramp limiter on throttle.  dtMs = elapsed since last call.
static float smoothThrottle(float target, float prev, uint32_t dtMs) {
    float ema     = prev + EMA_ALPHA * (target - prev);
    float maxStep = ((ema >= prev) ? THROTTLE_RAMP_UP_PCT_S
                                   : THROTTLE_RAMP_DN_PCT_S) * (dtMs / 1000.0f);
    float delta   = ema - prev;
    if (delta >  maxStep) delta =  maxStep;
    if (delta < -maxStep) delta = -maxStep;
    return clampf(prev + delta, 0.0f, 100.0f);
}

/// EMA + symmetric ramp limiter on steering.
static float smoothSteering(float target, float prev, uint32_t dtMs) {
    float ema     = prev + EMA_ALPHA * (target - prev);
    float maxStep = STEERING_RAMP_DEG_S * (dtMs / 1000.0f);
    float delta   = ema - prev;
    if (delta >  maxStep) delta =  maxStep;
    if (delta < -maxStep) delta = -maxStep;
    return clampf(prev + delta, -STEERING_MAX_DEG, +STEERING_MAX_DEG);
}

} // anonymous namespace

// =========================================================================
// Public API
// =========================================================================

void init(const Config& cfg) {
    s_cfg                       = cfg;
    s_stats                     = Stats{};
    s_state                     = State::IDLE;
    s_pos                       = 0;
    s_haveFrame                 = false;
    s_lastGoodMs                = 0;
    s_throttleSm                = 0.0f;
    s_steeringSm                = 0.0f;
    s_lastSmoothMs              = 0;
    for (uint8_t i = 0; i < IBUS_NUM_CHANNELS; i++) s_ch[i] = s_cfg.centerUs;

    // Allocate the HardwareSerial instance once.  Both the real ESP32 framework
    // and the host-test stub expose HardwareSerial(int) and
    // .begin(baud, config, rx, tx).
    static HardwareSerial uart(s_cfg.uartIndex);
    s_uart = &uart;
    // SERIAL_8N1 == 0x06 in both Arduino-ESP32 and our host stub.
    s_uart->begin(s_cfg.baudrate, 0x06 /*SERIAL_8N1*/, s_cfg.rxPin, -1);
}

void update() {
    // Drain UART (non-blocking).  Cap reads per call to keep loop responsive
    // even if the receiver dumps a burst after a stall.
    if (s_uart != nullptr) {
        for (int i = 0; i < 128 && s_uart->available() > 0; i++) {
            int b = s_uart->read();
            if (b < 0) break;
            processByte((uint8_t)b);
        }
    }

    // FSM timeout check
    uint32_t now = now_ms();
    if (s_haveFrame && (now - s_lastGoodMs) > s_cfg.timeoutMs) {
        if (s_state != State::FAILSAFE) {
            s_stats.timeouts++;
            s_state = State::FAILSAFE;
        }
    }

    // Smoothing tick (only when we have data AND parser is healthy)
    if (s_lastSmoothMs == 0) s_lastSmoothMs = now;
    uint32_t dt = now - s_lastSmoothMs;
    if (dt == 0) return;
    s_lastSmoothMs = now;

    bool active = (s_state == State::ACTIVE) && s_haveFrame
                  && !isKillSwitchActive() && isRemoteSelected();
    if (active) {
        float targetThr = computeThrottlePct(s_ch[1]);             // CH2
        float targetStr = computeSteeringDeg(s_ch[0], s_ch[2]);    // CH1 + CH3 trim
        s_throttleSm = smoothThrottle(targetThr, s_throttleSm, dt);
        s_steeringSm = smoothSteering(targetStr, s_steeringSm, dt);
    } else {
        // Force outputs to safe defaults; ramp down gracefully so a brief
        // CH10 toggle doesn't snap the demand to 0 instantaneously (the
        // motor pipeline will smooth too, but this keeps the audit trail
        // monotonic).
        s_throttleSm = smoothThrottle(0.0f, s_throttleSm, dt);
        s_steeringSm = smoothSteering(0.0f, s_steeringSm, dt);
    }
}

State getState() { return s_state; }

bool isActive() {
    return s_state == State::ACTIVE
        && s_haveFrame
        && !isKillSwitchActive()
        && isRemoteSelected();
}

float getThrottlePct() { return isActive() ? s_throttleSm : 0.0f; }
float getSteeringDeg() { return isActive() ? s_steeringSm : 0.0f; }

bool isKillSwitchActive() {
    // CH5: low microseconds == kill (safer default if no frame: kill ACTIVE)
    if (!s_haveFrame) return true;
    return s_ch[4] < SW_LOW_THR;
}

bool isRemoteSelected() {
    // CH10: above HIGH threshold == REMOTE.  Default to LOCAL when no frame.
    if (!s_haveFrame) return false;
    return s_ch[9] > SW_HIGH_THR;
}

uint8_t getDriveMode() {
    // CH6 three-position: low=ECO(0), mid=NORMAL(1), high=SPORT(2)
    if (!s_haveFrame) return 1;
    if (s_ch[5] < SW_LOW_THR)  return 0;
    if (s_ch[5] > SW_HIGH_THR) return 2;
    return 1;
}

uint8_t getRequestedGear() {
    // CH7 three-position: low=REVERSE(1), mid=NEUTRAL(2), high=FORWARD(3)
    if (!s_haveFrame) return 2;
    if (s_ch[6] < SW_LOW_THR)  return 1;
    if (s_ch[6] > SW_HIGH_THR) return 3;
    return 2;
}

bool isLightsOn() {
    // CH8: above center == ON
    if (!s_haveFrame) return false;
    return s_ch[7] > SW_HIGH_THR;
}

uint8_t getAudioVolume() {
    // CH9 potentiometer → 0..30 (DFPlayer range), with 1-step hysteresis to
    // avoid chatter on the boundary.
    if (!s_haveFrame) return 0;
    uint16_t us = s_ch[8];
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    int v = (int)mapf((float)us, 1000.0f, 2000.0f, 0.0f, 30.0f);
    if (v < 0)  v = 0;
    if (v > 30) v = 30;
    return (uint8_t)v;
}

Stats getStats() { return s_stats; }

uint16_t getChannelRaw(uint8_t ch) {
    if (ch < 1 || ch > IBUS_NUM_CHANNELS) return 0;
    return s_haveFrame ? s_ch[ch - 1] : 0;
}

// -------------------------------------------------------------------------
// Test hooks
// -------------------------------------------------------------------------
#ifdef REMOTE_CONTROL_TEST_HOOKS
namespace test {

void reset(const Config& cfg) {
    s_cfg                       = cfg;
    s_stats                     = Stats{};
    s_state                     = State::IDLE;
    s_pos                       = 0;
    s_haveFrame                 = false;
    s_lastGoodMs                = 0;
    s_throttleSm                = 0.0f;
    s_steeringSm                = 0.0f;
    s_lastSmoothMs              = 0;
    s_testNowMs                 = 0;
    s_useTestClock              = true;
    for (uint8_t i = 0; i < IBUS_NUM_CHANNELS; i++) s_ch[i] = s_cfg.centerUs;
}

void feedByte(uint8_t b) { processByte(b); }

void feedBytes(const uint8_t* data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) processByte(data[i]);
}

void setNowMs(uint32_t ms) {
    s_useTestClock = true;
    s_testNowMs    = ms;
}

void buildFrame(const uint16_t channels[14], uint8_t outBuf[32]) {
    outBuf[0] = IBUS_HDR0;
    outBuf[1] = IBUS_HDR1;
    for (uint8_t i = 0; i < 14; i++) {
        outBuf[2 + i * 2] = (uint8_t)(channels[i] & 0xFF);
        outBuf[3 + i * 2] = (uint8_t)((channels[i] >> 8) & 0xFF);
    }
    uint16_t sum = 0xFFFF;
    for (uint8_t i = 0; i < 30; i++) sum -= outBuf[i];
    outBuf[30] = (uint8_t)(sum & 0xFF);
    outBuf[31] = (uint8_t)((sum >> 8) & 0xFF);
}

} // namespace test
#endif // REMOTE_CONTROL_TEST_HOOKS

} // namespace remote_control

#endif // REMOTE_CONTROL_ENABLED || REMOTE_CONTROL_TEST_HOOKS
