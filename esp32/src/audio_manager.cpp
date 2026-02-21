// =============================================================================
// ESP32-S3 HMI — Audio Manager Implementation (DFPlayer Mini)
//
// DFPlayer Mini communication protocol: 10-byte serial commands at 9600 baud.
// Frame format: 0x7E FF 06 CMD 00 PARAM1 PARAM2 CHKLO CHKHI 0xEF
//
// Non-blocking: commands are sent via UART, no delay() calls.
// Minimum command interval: 100 ms (DFPlayer processing time).
// =============================================================================

#include "audio_manager.h"
#include <Arduino.h>
#include <HardwareSerial.h>

namespace audio {

// DFPlayer uses UART2
static HardwareSerial dfSerial(2);

static bool     initialized   = false;
static bool     playing       = false;
static Priority currentPri    = Priority::LOW;
static unsigned long lastCmdMs = 0;

// Minimum interval between DFPlayer commands
static constexpr unsigned long CMD_INTERVAL_MS = 100;

// Pending sound (simple single-slot queue)
static bool     pendingValid  = false;
static Sound    pendingSound  = Sound::WELCOME;
static Priority pendingPri    = Priority::LOW;

// Maximum assumed playback duration (ms) — DFPlayer doesn't report end reliably
static constexpr unsigned long MAX_PLAY_DURATION_MS = 5000;
static unsigned long playStartMs = 0;

// -------------------------------------------------------------------------
// DFPlayer serial command builder
// -------------------------------------------------------------------------
static void sendCommand(uint8_t cmd, uint8_t param1, uint8_t param2) {
    uint8_t buf[10];
    buf[0] = 0x7E;   // Start
    buf[1] = 0xFF;   // Version
    buf[2] = 0x06;   // Length
    buf[3] = cmd;    // Command
    buf[4] = 0x00;   // Feedback (0 = no)
    buf[5] = param1; // Parameter high
    buf[6] = param2; // Parameter low

    // Checksum: -(sum of bytes 1..6)
    int16_t sum = 0;
    for (uint8_t i = 1; i <= 6; i++) {
        sum += buf[i];
    }
    int16_t chk = -sum;
    buf[7] = (uint8_t)((chk >> 8) & 0xFF);
    buf[8] = (uint8_t)(chk & 0xFF);
    buf[9] = 0xEF;   // End

    dfSerial.write(buf, 10);
    lastCmdMs = millis();
}

// DFPlayer command codes
static constexpr uint8_t DF_CMD_PLAY_TRACK  = 0x03;  // Play specific track
static constexpr uint8_t DF_CMD_SET_VOLUME  = 0x06;  // Set volume (0-30)
static constexpr uint8_t DF_CMD_RESET       = 0x0C;  // Reset module

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

void init() {
    dfSerial.begin(9600, SERIAL_8N1, PIN_DFPLAYER_RX, PIN_DFPLAYER_TX);
    delay(100);  // Allow UART to stabilize

    // Reset DFPlayer module
    sendCommand(DF_CMD_RESET, 0, 0);
    delay(500);  // DFPlayer needs ~500ms after reset

    // Set default volume (20 out of 30)
    sendCommand(DF_CMD_SET_VOLUME, 0, 20);

    initialized = true;
    playing     = false;
    currentPri  = Priority::LOW;

    Serial.println("[AUDIO] DFPlayer initialized (UART2, 9600 baud)");
}

void update() {
    if (!initialized) return;

    unsigned long now = millis();

    // Check if current playback has timed out
    if (playing && (now - playStartMs) >= MAX_PLAY_DURATION_MS) {
        playing    = false;
        currentPri = Priority::LOW;
    }

    // Process pending sound if interval has elapsed
    if (pendingValid && (now - lastCmdMs) >= CMD_INTERVAL_MS) {
        // Send play command
        sendCommand(DF_CMD_PLAY_TRACK, 0, static_cast<uint8_t>(pendingSound));
        playing      = true;
        currentPri   = pendingPri;
        playStartMs  = now;
        pendingValid = false;
    }
}

void play(Sound sound, Priority priority) {
    if (!initialized) return;

    // If higher or equal priority than current, queue it
    if (!playing || static_cast<uint8_t>(priority) >= static_cast<uint8_t>(currentPri)) {
        pendingSound = sound;
        pendingPri   = priority;
        pendingValid = true;
    }
}

void setVolume(uint8_t vol) {
    if (!initialized) return;
    if (vol > 30) vol = 30;
    sendCommand(DF_CMD_SET_VOLUME, 0, vol);
}

bool isPlaying() {
    return playing;
}

} // namespace audio
