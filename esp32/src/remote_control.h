// =============================================================================
// ESP32-S3 — Remote Control Driver (FlySky FS-iA6B iBUS receiver)
//
// Non-blocking iBUS-Servo parser + 4-state failsafe FSM.  This module is the
// ESP32 side (Layer 2) of the 4-layer failsafe described in
// docs/REMOTE_CONTROL_FAILSAFE.md.  It only OBSERVES the receiver and produces
// validated channel values; CAN emission and arbitration are decided by the
// caller in main.cpp.
//
// Protocol: docs/REMOTE_CONTROL_ARCHITECTURE.md §5
//   - UART 115200 8N1, 3.3 V TTL (no level shifter)
//   - 32-byte frame every ~7 ms, header 0x20 0x40
//   - 14 uint16 LE channels (1000–2000 µs nominal)
//   - 16-bit LE checksum at bytes 30-31, = 0xFFFF − Σ(byte[0..29])
//
// Channel mapping (docs/REMOTE_CONTROL_FULL_PLAN.md §1):
//   CH1  → steering ±30°               (analog, → CAN 0x101)
//   CH2  → throttle 0..100 %           (analog, → CAN 0x100)
//   CH3  → steering trim ±5°           (analog, local, NVS-persistable)
//   CH4  → reserved (ignored)
//   CH5  → kill switch / enable        (digital, ESP32-local gate)
//   CH6  → mode flags 2WD/4WD/(4WD+tank) (discrete, optional)
//   CH7  → gear D/N/R                  (discrete, optional, coordinated w/ MCP23017 shifter)
//   CH8  → lights ON/OFF               (digital, optional)
//   CH9  → audio volume 0..30          (analog, ESP32 audio::setVolume)
//   CH10 → LOCAL / REMOTE sovereignty  (master gate: LOCAL disables RC authority;
//                                       CAN status / 0x10A override frames may still
//                                       be emitted with override_flag=0)
//
// **Disabled in the bench build** via `-DREMOTE_CONTROL_ENABLED=0` in
// platformio.ini (restored to 1 only in a follow-up after bench validation).
// When compiled with `-DREMOTE_CONTROL_ENABLED=0` this header still compiles
// and exposes stub-safe inline no-ops so main.cpp can reference the API
// without conditional compilation everywhere.  The real implementation in
// remote_control.cpp links in whenever the flag is non-zero.
//
// **STM32 invariants preserved**: this module never sends frames out of range,
// never modifies STM32 firmware, never changes safety paths.  The STM32
// remains the single safety authority.
// =============================================================================

#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <cstdint>

#ifndef REMOTE_CONTROL_ENABLED
#define REMOTE_CONTROL_ENABLED 0
#endif

namespace remote_control {

// -------------------------------------------------------------------------
// Failsafe FSM states (docs/REMOTE_CONTROL_FAILSAFE.md §2)
// -------------------------------------------------------------------------
enum class State : uint8_t {
    IDLE      = 0,  ///< Just initialized, no frame seen yet
    ACTIVE    = 1,  ///< Receiving valid frames within timeout
    DEGRADED  = 2,  ///< >5 consecutive checksum fails, but recent ACTIVE
    FAILSAFE  = 3,  ///< >150 ms without valid frame OR receiver lost RF
};

// -------------------------------------------------------------------------
// Module configuration
// -------------------------------------------------------------------------
struct Config {
    int      rxPin       = 16;        ///< ESP32-S3 GPIO for iBUS RX (TX unused)
    int      uartIndex   = 0;         ///< HardwareSerial index (UART0 HW free; USB-CDC uses native USB)
    uint32_t baudrate    = 115200;    ///< iBUS-Servo standard
    uint32_t timeoutMs   = 150;       ///< No valid frame → FAILSAFE (~21 frames @ 7 ms)
    uint16_t centerUs    = 1500;      ///< Stick center microseconds
    uint16_t deadzoneUs  = 30;        ///< ±deadzone around center for steering / throttle floor
    uint16_t sanityMinUs = 900;       ///< Frame rejected if any channel < this
    uint16_t sanityMaxUs = 2100;      ///< Frame rejected if any channel > this
    uint8_t  maxChksumStreak = 5;     ///< Consecutive checksum fails → DEGRADED
};

// -------------------------------------------------------------------------
// Diagnostic counters (cumulative since init)
// -------------------------------------------------------------------------
struct Stats {
    uint32_t framesOk          = 0;
    uint32_t checksumFails     = 0;
    uint32_t headerResyncs     = 0;
    uint32_t sanityRejects     = 0;
    uint32_t timeouts          = 0;   ///< Number of times we entered FAILSAFE
    uint16_t consecutiveCkFail = 0;
};

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

#if REMOTE_CONTROL_ENABLED

/// Initialize UART and reset internal FSM.  Idempotent.
void init(const Config& cfg = Config{});

/// Drain pending UART bytes and run the parser FSM.  Non-blocking; must be
/// called frequently from loop() (no minimum cadence, but ≥ once per loop).
/// Typical execution time: < 100 µs even at full iBUS rate.
void update();

/// Current failsafe state.
State getState();

/// True if state == ACTIVE AND CH5 ≠ kill AND CH10 == REMOTE.  This is the
/// gate main.cpp uses to decide whether to emit 0x100 / 0x101 CAN frames.
bool isActive();

/// Throttle 0..100 (after deadzone, EMA α=0.15, ramp 50%/s up / 100%/s down).
/// Returns 0.0f when not active.  Matches the smoothing pipeline already
/// applied in STM32 motor_control.c so the demand path is consistent.
float getThrottlePct();

/// Steering −30..+30 degrees (after deadzone + EMA + ramp, plus CH3 trim).
/// Returns 0.0f when not active.
float getSteeringDeg();

/// True when CH5 reads as "kill" (below center).  Overrides everything else.
bool isKillSwitchActive();

/// True when CH10 reads as REMOTE (above mid threshold).  When false, the
/// caller MUST NOT emit CAN commands from the remote control.
bool isRemoteSelected();

/// Mode flags from CH6 (three-position switch):
///   low  -> 0x00 (2WD, tank OFF)
///   mid  -> 0x01 (4WD, tank OFF)
///   high -> 0x03 (4WD, tank ON)
/// Matches CMD_MODE bit layout (bit0=4x4, bit1=tank turn).
uint8_t getDriveMode();

/// Requested gear from CH7 three-position switch.  0=PARK, 1=REVERSE,
/// 2=NEUTRAL, 3=FORWARD (matches shifter::Gear / GearPosition_t).
/// The caller decides whether to honor this vs the physical MCP23017 shifter.
uint8_t getRequestedGear();

/// True when CH8 reads as ON (above center).
bool isLightsOn();

/// Audio volume 0..30 from CH9 potentiometer.
uint8_t getAudioVolume();

/// Diagnostic counters snapshot.
Stats getStats();

/// Raw last microsecond value of a channel (1..14).  Returns 0 if no frame
/// has been received or index out of range.  For diagnostics only.
uint16_t getChannelRaw(uint8_t ch);

#else  // REMOTE_CONTROL_ENABLED == 0  (default — module compiled out)

inline void     init(const Config& = Config{}) {}
inline void     update() {}
inline State    getState()                  { return State::IDLE; }
inline bool     isActive()                  { return false; }
inline float    getThrottlePct()            { return 0.0f; }
inline float    getSteeringDeg()            { return 0.0f; }
inline bool     isKillSwitchActive()        { return true; }   // Fail-safe default
inline bool     isRemoteSelected()          { return false; }
inline uint8_t  getDriveMode()              { return 0; }      // 2WD + tank OFF (safe default when RC disabled)
inline uint8_t  getRequestedGear()          { return 2; }      // NEUTRAL
inline bool     isLightsOn()                { return false; }
inline uint8_t  getAudioVolume()            { return 0; }
inline Stats    getStats()                  { return Stats{}; }
inline uint16_t getChannelRaw(uint8_t)      { return 0; }

#endif // REMOTE_CONTROL_ENABLED

// -------------------------------------------------------------------------
// Test hooks (only compiled when REMOTE_CONTROL_TEST_HOOKS is defined).
// Allow host-side tests to drive the parser without a real UART.  Not used
// in production firmware.
// -------------------------------------------------------------------------
#ifdef REMOTE_CONTROL_TEST_HOOKS
namespace test {
    /// Reset all internal state (counters, FSM, smoothing) as if just init'd.
    void reset(const Config& cfg = Config{});
    /// Feed a single byte directly into the parser (bypasses UART).
    void feedByte(uint8_t b);
    /// Feed multiple bytes.
    void feedBytes(const uint8_t* data, uint16_t len);
    /// Advance the parser's "now" reference (uses millis() in production).
    void setNowMs(uint32_t ms);
    /// Build a complete valid 32-byte iBUS frame into outBuf from a
    /// channel array of length 14 (microseconds).  Computes checksum.
    void buildFrame(const uint16_t channels[14], uint8_t outBuf[32]);
}
#endif // REMOTE_CONTROL_TEST_HOOKS

} // namespace remote_control

#endif // REMOTE_CONTROL_H
