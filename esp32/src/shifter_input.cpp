// =============================================================================
// ESP32-S3 — Shifter Input Driver (MCP23017 I2C) — Implementation
//
// Reads gear selector position from MCP23017 Port A pins (active-low).
//
// The lever has DRY CONTACTS (no internal voltage) and 5 wires:
//   1 common wire (blue) — connect to GND
//   4 signal wires       — connect to MCP23017 GPA pins (pull-ups provide bias)
//
// Physical lever order (top → bottom) and confirmed wire colours:
//   GPA0 — P  (Park)    — blue + purple
//   GPA1 — D2 (Drive 2) — blue + green
//   GPA2 — D1 (Drive 1) — blue + yellow
//   GPA3 — R  (Reverse) — blue + white
//
// NEUTRAL has NO dedicated contact wire.  It is detected implicitly:
// when none of the four contacts are closed all four GPA pins stay HIGH
// (pulled up) and decodeGear() returns NEUTRAL.
//
// Reference: docs/CAN_CONTRACT_FINAL.md §4.5
// =============================================================================

#include "shifter_input.h"
#include <Wire.h>
#include <Arduino.h>

namespace shifter {

// MCP23017 register addresses (BANK=0 mode, default)
static constexpr uint8_t REG_IODIRA   = 0x00;  // I/O Direction A
static constexpr uint8_t REG_GPPUA    = 0x0C;  // Pull-Up A
static constexpr uint8_t REG_GPIOA    = 0x12;  // GPIO A read
static constexpr uint8_t REG_GPIOB    = 0x13;  // GPIO B read (diagnostic only)

// Pin masks for each gear position on Port A — confirmed wire colours
// Common wire (blue) → GND.  Signal wires → GPA0-GPA3 with pull-ups.
static constexpr uint8_t PIN_PARK     = (1 << 0);  // GPA0 — blue+purple  (P)
static constexpr uint8_t PIN_FWD_D2   = (1 << 1);  // GPA1 — blue+green   (D2)
static constexpr uint8_t PIN_FORWARD  = (1 << 2);  // GPA2 — blue+yellow  (D1)
static constexpr uint8_t PIN_REVERSE  = (1 << 3);  // GPA3 — blue+white   (R)
// Neutral has no contact wire — detected when no pin is active (see decodeGear)
static constexpr uint8_t GEAR_MASK    = 0x0F;       // Bits 0-3 only

// I2C error handling constants
static constexpr uint8_t  ERROR_THRESHOLD  = 5;     // Consecutive errors before backoff
                                                     // (5 × 50ms pollMs = 250ms to trigger)
static constexpr uint32_t BACKOFF_POLL_MS  = 1000;  // Poll interval during backoff (ms)

// Hard per-transaction I2C timeout.  Bounds the worst-case time the main
// (Core-1) loop can spend inside a single MCP23017 access so that a stuck or
// floating bus can never block CAN reception longer than the heartbeat
// period — preventing the false "CAN LINK LOST" caused by an I2C stall.
static constexpr uint16_t WIRE_TIMEOUT_MS  = 25;

// Module state
static Config       cfg_;
static Gear         currentGear_   = Gear::NEUTRAL;
static Gear         pendingGear_   = Gear::NEUTRAL;        // F5: candidate awaiting confirmation
static uint8_t      pendingCount_  = 0;                     // F5: consecutive identical samples
static constexpr uint8_t DEBOUNCE_SAMPLES = 2;              // F5: required identical samples
static unsigned long lastPollMs_   = 0;
static bool         initialized_   = false;
static uint8_t      errorCount_    = 0;               // Consecutive I2C error counter
static bool         connected_     = false;            // MCP23017 responding on I2C

// Diagnostic snapshot state (cached on the main loop, read by Engineering UI).
// None of these are touched by getDiag(); they are written only here so the
// Engineering screen never has to drive the shared Wire bus itself.
static uint8_t       lastGpioRaw_   = 0xFF;            // Last raw GPIOA read (0xFF = error)
static unsigned long lastValidMs_   = 0;               // millis() of last good GPIOA read
static uint16_t      recoveryCount_ = 0;               // I2C bus-recovery attempts
static uint8_t       cfgIodirA_     = 0;               // Last IODIRA value written
static uint8_t       cfgGppuA_      = 0;               // Last GPPUA value written

// Extended instrumentation (problem statement §1.F/§1.G).  Written only on
// the main loop alongside the existing diagnostic snapshot; read by getDiag().
static uint8_t       lastGpiobRaw_     = 0xFF;         // Last raw GPIOB read (0xFF = not read/error)
static uint16_t      validReads_       = 0;            // Count of valid GPIOA reads
static uint16_t      invalidReads_     = 0;            // Count of rejected GPIOA reads
static uint8_t       lastValidPattern_ = 0xFF;         // Last GPIOA byte from a valid read
static RejectReason  rejectReason_     = RejectReason::NONE; // Why the last read was rejected

// Failure stage of the most recent readReg() call.  Set inside readReg() and
// consumed by update() to publish the exact rejection reason without guessing.
static RejectReason  lastReadStage_    = RejectReason::NONE;

// -------------------------------------------------------------------------
// Write a single register to MCP23017
// -------------------------------------------------------------------------
static bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(cfg_.i2cAddr);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

// -------------------------------------------------------------------------
// Read a single register from MCP23017
// Returns 0xFF on error.  Sets connected_ to false on I2C failure.
//
// Records the exact transaction stage that failed in lastReadStage_ so the
// caller can publish the precise rejection reason (problem statement §1.F):
//   ADDR_NACK — the register-pointer write was not ACKed (endTransmission!=0)
//   NO_DATA   — requestFrom returned no byte (slave did not clock out data)
//   NONE      — success
// -------------------------------------------------------------------------
static uint8_t readReg(uint8_t reg) {
    Wire.beginTransmission(cfg_.i2cAddr);
    Wire.write(reg);
    uint8_t err = Wire.endTransmission(false);
    if (err != 0) {
        lastReadStage_ = RejectReason::ADDR_NACK;
        return 0xFF;  // Error — skip requestFrom to avoid log spam
    }
    uint8_t n = Wire.requestFrom(cfg_.i2cAddr, (uint8_t)1);
    if (n == 0 || !Wire.available()) {
        lastReadStage_ = RejectReason::NO_DATA;
        return 0xFF;  // Error — no data received
    }
    lastReadStage_ = RejectReason::NONE;
    return Wire.read();
}

// -------------------------------------------------------------------------
// I2C bus recovery — free a bus stuck because a slave (or a shorted/floating
// shifter input) is holding SDA low.
//
// Technique (per the I2C-bus specification, §3.1.16): release SDA and pulse
// SCL up to 9 times so the slave finishes its byte and releases SDA, then
// issue a manual STOP condition.  Finally the Wire driver is re-initialised.
//
// This is intentionally short (≈9 × 10 µs ≪ the CAN heartbeat period) so it
// can never starve CAN reception on the shared Core-1 loop, and it is only
// invoked when the device first crosses the consecutive-error threshold.
// -------------------------------------------------------------------------
static void recoverBus() {
    Wire.end();

    // Drive SCL, let SDA float high (external/internal pull-up restores it).
    pinMode(cfg_.sdaPin, INPUT_PULLUP);
    pinMode(cfg_.sclPin, OUTPUT);
    digitalWrite(cfg_.sclPin, HIGH);
    delayMicroseconds(5);

    for (int i = 0; i < 9; ++i) {
        digitalWrite(cfg_.sclPin, LOW);
        delayMicroseconds(5);
        digitalWrite(cfg_.sclPin, HIGH);
        delayMicroseconds(5);
        if (digitalRead(cfg_.sdaPin) == HIGH) {
            break;  // Slave released SDA — bus is free
        }
    }

    // Manual STOP: SDA low→high while SCL is high.
    pinMode(cfg_.sdaPin, OUTPUT);
    digitalWrite(cfg_.sdaPin, LOW);
    delayMicroseconds(5);
    digitalWrite(cfg_.sclPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(cfg_.sdaPin, HIGH);
    delayMicroseconds(5);

    // Re-arm the Wire driver with the project timing/timeout.
    Wire.begin(cfg_.sdaPin, cfg_.sclPin);
    Wire.setClock(400000);
    Wire.setTimeOut(WIRE_TIMEOUT_MS);

    if (recoveryCount_ < 0xFFFF) recoveryCount_++;
}

// -------------------------------------------------------------------------
// Decode gear from port value (active-low, one-hot)
//
// Returns NEUTRAL when no pin is active — this is the normal resting state
// for the N position (no physical contact wire).
// -------------------------------------------------------------------------
static Gear decodeGear(uint8_t portVal) {
    // Invert (active-low) and mask relevant bits
    uint8_t active = (~portVal) & GEAR_MASK;

    // Zero active bits = Neutral (lever in N, no contact closed)
    // More than one active bit = invalid/transition → default to Neutral
    if (__builtin_popcount(active) != 1) {
        return Gear::NEUTRAL;
    }

    if (active & PIN_PARK)    return Gear::PARK;
    if (active & PIN_FWD_D2)  return Gear::FORWARD_D2;
    if (active & PIN_FORWARD) return Gear::FORWARD;
    if (active & PIN_REVERSE) return Gear::REVERSE;

    return Gear::NEUTRAL;
}

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

void init(const Config& cfg) {
    cfg_ = cfg;

    Wire.begin(cfg_.sdaPin, cfg_.sclPin);
    Wire.setClock(400000);  // 400 kHz I2C (Fast Mode)
    // Hard timeout so no single MCP23017 transaction can stall the Core-1
    // loop (and therefore CAN reception) longer than the heartbeat period.
    Wire.setTimeOut(WIRE_TIMEOUT_MS);

    // Probe the MCP23017: try to configure Port A
    bool ok = writeReg(REG_IODIRA, GEAR_MASK);
    if (ok) {
        ok = writeReg(REG_GPPUA, GEAR_MASK);
    }

    // Cache the configured register values for the Engineering diagnostic.
    // GPPUA = GEAR_MASK enables the MCP23017 internal pull-ups on GPA0-3, so
    // an open contact (and the implicit NEUTRAL position) reads HIGH and a
    // closed contact (common wire → GND) reads LOW (active-low). decodeGear()
    // enforces a one-hot result and falls back to NEUTRAL for 0 or >1 active
    // bits, so a floating/shorted input can never latch a spurious gear.
    cfgIodirA_ = GEAR_MASK;
    cfgGppuA_  = GEAR_MASK;

    connected_   = ok;
    errorCount_  = ok ? 0 : ERROR_THRESHOLD;
    initialized_ = true;
    lastPollMs_  = 0;
    lastGpioRaw_ = 0xFF;
    lastValidMs_ = 0;
    lastGpiobRaw_     = 0xFF;
    validReads_       = 0;
    invalidReads_     = 0;
    lastValidPattern_ = 0xFF;
    rejectReason_     = RejectReason::NONE;
    lastReadStage_    = RejectReason::NONE;
    currentGear_ = Gear::NEUTRAL;
    pendingGear_  = Gear::NEUTRAL;   // F5: reset debounce state
    pendingCount_ = 0;

    if (ok) {
        Serial.println("[SHIFTER] MCP23017 initialized");
    } else {
        Serial.println("[SHIFTER] MCP23017 NOT detected — backoff active");
    }
}

// -------------------------------------------------------------------------
// Record the outcome of a GPIOA read for the diagnostic snapshot.
//
// On a valid read: bump the valid counter, latch the bit pattern, clear the
// reject reason and refresh GPIOB (one extra transaction, only when the bus
// is already known good so it never adds load during an error/backoff).
// On a rejected read: bump the invalid counter and publish the exact stage
// that failed (captured by readReg in lastReadStage_).
// -------------------------------------------------------------------------
static void recordRead(uint8_t portVal) {
    lastGpioRaw_ = portVal;
    if (portVal != 0xFF) {
        if (validReads_ < 0xFFFF) validReads_++;
        lastValidPattern_ = portVal;
        rejectReason_     = RejectReason::NONE;
        // Refresh GPIOB only after a good GPIOA read (bus confirmed healthy).
        lastGpiobRaw_ = readReg(REG_GPIOB);
    } else {
        if (invalidReads_ < 0xFFFF) invalidReads_++;
        rejectReason_ = lastReadStage_;  // ADDR_NACK or NO_DATA
    }
}

void update() {
    if (!initialized_) return;

    unsigned long now = millis();

    // Use longer poll interval when the device is in error backoff
    uint32_t interval = (errorCount_ >= ERROR_THRESHOLD) ? BACKOFF_POLL_MS : cfg_.pollMs;
    if (now - lastPollMs_ < interval) return;
    lastPollMs_ = now;

    // When in backoff, use address-only probe (endTransmission with STOP)
    // instead of readReg() to avoid ESP32 Wire.requestFrom() error spam
    // when the MCP23017 is not connected.
    if (errorCount_ >= ERROR_THRESHOLD) {
        Wire.beginTransmission(cfg_.i2cAddr);
        if (Wire.endTransmission() != 0) {
            currentGear_ = Gear::NEUTRAL;
            pendingGear_  = Gear::NEUTRAL;   // F5: reset debounce on fail-safe
            pendingCount_ = 0;
            rejectReason_ = RejectReason::BACKOFF;  // device absent — no read attempted
            return;  // Device still absent — stay in backoff
        }
        // Device responded — re-initialize and resume
        if (!writeReg(REG_IODIRA, GEAR_MASK) || !writeReg(REG_GPPUA, GEAR_MASK)) {
            currentGear_ = Gear::NEUTRAL;
            pendingGear_  = Gear::NEUTRAL;   // F5: reset debounce on fail-safe
            pendingCount_ = 0;
            return;  // Re-init failed — stay in backoff
        }
        errorCount_ = 0;
        connected_  = true;
        Serial.println("[SHIFTER] MCP23017 reconnected");
        uint8_t portVal = readReg(REG_GPIOA);
        recordRead(portVal);
        if (portVal != 0xFF) lastValidMs_ = now;
        // F5: after reconnect, force the NEUTRAL fail-safe and restart the
        // debounce window — never publish a non-NEUTRAL gear on a single
        // post-recovery sample.
        currentGear_ = Gear::NEUTRAL;
        pendingGear_  = (portVal != 0xFF) ? decodeGear(portVal) : Gear::NEUTRAL;
        pendingCount_ = 1;
        return;
    }

    uint8_t portVal = readReg(REG_GPIOA);
    recordRead(portVal);   // cache raw read + counters/pattern/reason for diag

    if (portVal == 0xFF) {
        // I2C read failed
        if (errorCount_ < ERROR_THRESHOLD) {
            errorCount_++;
            if (errorCount_ == ERROR_THRESHOLD) {
                // Transition to disconnected — log once
                connected_ = false;
                // Attempt a one-shot bus recovery before entering backoff:
                // frees SDA if a shorted/floating shifter input or an
                // unresponsive MCP23017 is clamping the line low.
                recoverBus();
                Serial.println("[SHIFTER] MCP23017 I2C error — bus recovery + backoff");
            }
        }
        // Set currentGear_ to NEUTRAL during error and clear debounce state
        currentGear_  = Gear::NEUTRAL;
        pendingGear_  = Gear::NEUTRAL;
        pendingCount_ = 0;
        return;
    }

    // Successful read — recover from minor error streak
    lastValidMs_ = now;
    if (errorCount_ > 0) {
        errorCount_ = 0;
        connected_  = true;
    }

    // F5 — true debounce:
    //   Only commit a decoded gear to currentGear_ after observing
    //   DEBOUNCE_SAMPLES (=2) consecutive identical samples.  Any
    //   sample that differs from the pending candidate restarts the
    //   counter.  This filters single-poll glitches without any heap
    //   allocation, RAM cost = 2 bytes (pendingGear_ + pendingCount_),
    //   and preserves the 50 ms polling cadence and the one-hot
    //   NEUTRAL fail-safe in decodeGear().
    Gear sample = decodeGear(portVal);
    if (sample == pendingGear_) {
        if (pendingCount_ < DEBOUNCE_SAMPLES) {
            pendingCount_++;
        }
    } else {
        pendingGear_  = sample;
        pendingCount_ = 1;
    }
    if (pendingCount_ >= DEBOUNCE_SAMPLES) {
        currentGear_ = pendingGear_;
    }
}

Gear getGear() {
    if (!initialized_) return Gear::NEUTRAL;
    return currentGear_;
}

uint8_t getGearRaw() {
    if (!initialized_) return static_cast<uint8_t>(Gear::NEUTRAL);
    return static_cast<uint8_t>(currentGear_);
}

bool isConnected() {
    return initialized_ && connected_;
}

Diag getDiag() {
    Diag d;
    d.i2cAddr       = cfg_.i2cAddr;
    d.initialized   = initialized_;
    d.connected     = initialized_ && connected_;
    d.iodirA        = cfgIodirA_;
    d.gppuA         = cfgGppuA_;
    d.gpioRaw       = lastGpioRaw_;
    d.gearDecoded   = static_cast<uint8_t>(currentGear_);
    d.errorCount    = errorCount_;
    d.recoveryCount = recoveryCount_;
    d.lastValidMs   = lastValidMs_;
    d.gpiobRaw         = lastGpiobRaw_;
    d.validReads       = validReads_;
    d.invalidReads     = invalidReads_;
    d.lastValidPattern = lastValidPattern_;
    d.rejectReason     = rejectReason_;
    return d;
}

} // namespace shifter
