// =============================================================================
// ESP32-S3 HMI — Audio Manager (DFPlayer Mini)
//
// Non-blocking audio playback via DFPlayer Mini module over UART.
// Priority-based queue: higher priority sounds preempt lower ones.
// The DFPlayer reads MP3 files from an SD card (numbered 001–255).
//
// UART2 on GPIO 43 (TX) / GPIO 44 (RX), 9600 baud.
//
// Sound file mapping (on SD card):
//   001.mp3 — Welcome / startup chime
//   002.mp3 — Farewell / shutdown chime
//   003.mp3 — Obstacle warning beep
//   004.mp3 — Error alert
//   005.mp3 — Low battery warning
//   006.mp3 — Gear change confirmation click
//
// Reference: FIRMWARE_MIGRATION_AUDIT.md Step 7
// =============================================================================

#ifndef AUDIO_MANAGER_H
#define AUDIO_MANAGER_H

#include <cstdint>

namespace audio {

/// GPIO assignments for DFPlayer Mini UART
inline constexpr int PIN_DFPLAYER_TX = 43;   // ESP32 TX → DFPlayer RX
inline constexpr int PIN_DFPLAYER_RX = 44;   // DFPlayer TX → ESP32 RX

/// Sound file indices (must match SD card file numbering)
enum class Sound : uint8_t {
    WELCOME         = 1,
    FAREWELL        = 2,
    OBSTACLE_WARN   = 3,
    ERROR_ALERT     = 4,
    BATTERY_LOW     = 5,
    GEAR_CHANGE     = 6
};

/// Priority levels (higher number = higher priority, preempts lower)
enum class Priority : uint8_t {
    LOW     = 0,   // Gear clicks, info beeps
    MEDIUM  = 1,   // Obstacle warnings, battery alerts
    HIGH    = 2    // Error, welcome, farewell
};

/// Initialize UART and DFPlayer module
void init();

/// Update audio state machine — call from main loop
/// Handles DFPlayer busy detection and queue processing.
void update();

/// Play a sound. If a higher or equal priority sound is already playing,
/// the new sound is queued. If a lower priority sound is playing, it is
/// preempted.
void play(Sound sound, Priority priority = Priority::MEDIUM);

/// Set master volume (0–30, DFPlayer range)
void setVolume(uint8_t vol);

/// Returns true if audio is currently playing
bool isPlaying();

} // namespace audio

#endif // AUDIO_MANAGER_H
