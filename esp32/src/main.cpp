// =============================================================================
// ESP32-S3 HMI Firmware — CAN Bring-Up + TFT Display
//
// Framework:  Arduino (C++17)
// Board:      ESP32-S3-DevKitC-1
// Display:    480×320 TFT via TFT_eSPI (ST7796, landscape rotation 1)
// Reference:  docs/CAN_CONTRACT_FINAL.md rev 1.3
//             docs/HMI_RENDERING_STRATEGY.md
// =============================================================================

#include <Arduino.h>
#include <climits>
#include <esp_system.h>
#include <ESP32-TWAI-CAN.hpp>
#include <TFT_eSPI.h>
#include "can_ids.h"
#include "can_rx.h"
#include "vehicle_data.h"
#include "screen_manager.h"
#include "ui/runtime_monitor.h"
#include "ui/debug_overlay.h"
#include "ui/led_toggle.h"
#include "ui/mode_icons.h"
#include "sensors/obstacle_sensor.h"
#include "can/can_obstacle.h"
#include "led_controller.h"
#include "power_manager.h"
#include "audio_manager.h"
#include "shifter_input.h"
#include "touch_handler.h"
#include "config_store.h"
#include "traction_switch.h"

// CAN transceiver pins (TJA1051 — see platformio.ini header)
static constexpr int CAN_TX_PIN = 4;
static constexpr int CAN_RX_PIN = 5;

// Global TFT instance — used by all screens via extern
TFT_eSPI tft = TFT_eSPI();

static vehicle::VehicleData vehicleData;
static ScreenManager screenManager;

static uint8_t  heartbeatCounter = 0;
static unsigned long lastHeartbeatMs  = 0;
static unsigned long lastSerialMs     = 0;
#if RUNTIME_MONITOR
static unsigned long lastRtMonMs      = 0;
#endif

// ---- LED relay state tracking ----
static bool     frontLedLocalState = false;  // front LED relay desired state
static bool     rearLedLocalState  = false;  // rear LED relay desired state

// ---- Shifter gear tracking ----
static uint8_t  lastSentGear      = 0xFF;    // last gear value sent to STM32
static unsigned long lastGearSendMs = 0;     // debounce for gear CAN sends
static constexpr unsigned long GEAR_SEND_DEBOUNCE_MS = 100;
static constexpr uint8_t GEAR_REVERSE = 1;   // shifter raw value for Reverse

// ---- Mode state tracking ----
static uint8_t  currentModeFlags  = 0;       // Current mode flags (bit 0=4x4, bit 1=tank)

// ---- Power/Audio state tracking ----
static bool     welcomePlayed     = false;
static bool     farewellPlayed    = false;

// ---- Audio event tracking ----
static uint8_t  lastAudioSystemState = 0;     // detect state transitions for error alert
static bool     batteryLowPlayed  = false;     // one-shot for battery warning
static bool     batteryCritPlayed = false;     // one-shot for battery critical
static constexpr uint16_t BATTERY_LOW_THRESHOLD_RAW = 2000;  // 20.00 V in 0.01 V units
static constexpr uint16_t BATTERY_CRIT_THRESHOLD_RAW = 1800; // 18.00 V in 0.01 V units
static unsigned long lastObstacleWarnMs = 0;   // debounce obstacle warning beeps
static constexpr unsigned long OBSTACLE_WARN_INTERVAL_MS = 2000;

// ---- Temperature audio tracking ----
static bool     tempHighPlayed = false;        // one-shot for temp high warning
static constexpr int8_t TEMP_HIGH_THRESHOLD = 85;  // °C — play alert above this
static constexpr int8_t TEMP_HIGH_HYSTERESIS = 5;  // °C — hysteresis to prevent oscillation

// ---- ABS/TCS audio tracking ----
static uint8_t  lastAbsActive = 0;
static uint8_t  lastTcsActive = 0;

// ---- Overcurrent audio tracking ----
static uint8_t  lastSafetyError = 0;           // last SafetyError code for transition detect

// ---- Lights audio tracking ----
static bool     lastFrontRelayOn  = false;
static bool     lastRearRelayOn   = false;
static bool     lightsAudioInit   = false;     // skip first transition on startup

// ---- Multi-error burst arbitration ----
// When >1 error/warning fires within the burst window, play the highest-severity
// sound instead of collapsing to ERROR_GENERAL (Phase 1 AUDIO_MITIGATION_PLAN).
// Severity order: EMERGENCY > OVERCURRENT > BATTERY_CRITICAL > SENSOR errors > ERROR_GENERAL
static uint8_t  errorBurstCount = 0;
static unsigned long errorBurstStartMs = 0;
static constexpr unsigned long ERROR_BURST_WINDOW_MS = 2000;  // 2 s window
static constexpr uint8_t ERROR_BURST_THRESHOLD = 2;           // >=2 errors = burst
static audio::Sound    burstDominantSound = audio::Sound::ERROR_GENERAL;
static audio::Priority burstDominantPri   = audio::Priority::HI;

/// Burst severity ranking — higher value = more important for the driver.
/// Determines which sound wins when multiple errors fire within the burst window.
static uint8_t burstSeverity(audio::Sound s) {
    switch (s) {
        case audio::Sound::EMERGENCY:            return 6;
        case audio::Sound::OVERCURRENT:          return 5;
        case audio::Sound::BATTERY_CRITICAL:     return 4;
        case audio::Sound::SENSOR_TEMP_ERROR:    return 3;
        case audio::Sound::SENSOR_CURRENT_ERROR: return 3;
        case audio::Sound::SENSOR_SPEED_ERROR:   return 3;
        case audio::Sound::ENCODER_ERROR:        return 3;
        case audio::Sound::TEMP_HIGH:            return 3;
        case audio::Sound::BATTERY_LOW:          return 2;
        case audio::Sound::OBSTACLE_WARN:        return 2;
        case audio::Sound::ERROR_GENERAL:        return 1;
        default:                                 return 0;
    }
}

// ---- LED brake/reverse detection thresholds ----
// Speed sum threshold: 4 wheels × 0.5 km/h × 10 (0.1 km/h units) = 20
static constexpr uint32_t LED_SPEED_SUM_THRESHOLD = 20;
// Traction average below which we consider braking (0–100% scale)
static constexpr uint8_t LED_TRACTION_BRAKING_THRESHOLD = 5;

// ---- NVS flush interval ----
static unsigned long lastNvsFlushMs = 0;
static constexpr unsigned long NVS_FLUSH_INTERVAL_MS = 10000;  // 10 seconds

// ---- STM32 heartbeat liveness monitoring (SAFETY FIX) ----
// Detect whether the STM32 alive counter (heartbeat byte 0) is actually
// incrementing.  A frozen STM32 (IWDG not kicking, or timer firing without
// the main loop running) would keep sending the same counter value.
// After STM32_HB_FREEZE_COUNT consecutive identical counter values the ESP32
// considers the STM32 non-responsive and inhibits further motion commands.
static constexpr uint8_t  STM32_HB_FREEZE_COUNT = 5;   // 5 × 100 ms = 500 ms
static uint8_t  stm32HbLastCounter   = 0xFFU;           // Init to impossible value
static uint8_t  stm32HbSameCount     = 0;
static bool     stm32IsAlive         = false;

// ---- Gear re-sync after STM32 restart (SAFETY FIX) ----
// When the STM32 restarts it re-initialises to GEAR_FORWARD regardless of
// what gear the ESP32 (or driver) had selected.  The ESP32 detects the restart
// via the startup_inhibit bit (bit 0 of heartbeat statusFlags) transitioning
// from 0→1, sets a flag, and re-sends the current physical shifter gear as soon
// as the inhibit clears and the STM32 enters ACTIVE state.
static bool     stm32StartupSeen     = false;  // true while startup_inhibit was last 1
static bool     gearResyncPending    = false;  // true = need to re-send gear on ACTIVE

// ---- Command ACK tracking (Phase 13) ----
// Non-blocking: records when a command was sent and checks for ACK arrival.
// UI state is only updated once ACK is received or timeout expires.
// Design: no automatic retry — bounded timeout only, no infinite loops.

static bool     ackPending     = false;   // true while waiting for ACK
static uint8_t  ackExpectedCmd = 0;       // low byte of the command CAN ID we sent
static unsigned long ackSentMs = 0;       // timestamp when the command was sent
static bool     ackTimedOut    = false;   // set true if ACK_TIMEOUT_MS elapsed

/// Call before sending a command that expects ACK (CMD_MODE, SERVICE_CMD).
__attribute__((unused))
static void ackBeginWait(uint8_t cmdIdLow) {
    ackPending     = true;
    ackExpectedCmd = cmdIdLow;
    ackSentMs      = millis();
    ackTimedOut    = false;
}

/// Call from loop() after can_rx::poll() to check for ACK arrival or timeout.
static void ackCheck(vehicle::VehicleData& data) {
    if (!ackPending) return;

    // Check if matching ACK arrived
    const auto& ad = data.ack();
    if (ad.timestampMs >= ackSentMs && ad.cmdIdLow == ackExpectedCmd) {
        ackPending  = false;
        ackTimedOut = false;
        if (ad.result != can::AckResult::OK) {
            Serial.printf("[ACK] cmd 0x%02X result=%u state=%u\n",
                          ad.cmdIdLow, static_cast<uint8_t>(ad.result),
                          static_cast<uint8_t>(ad.systemState));
        }
        return;
    }

    // Check timeout (bounded, no retry loop)
    if (millis() - ackSentMs >= can::ACK_TIMEOUT_MS) {
        ackPending  = false;
        ackTimedOut = true;
        data.setAckTimeout(millis());
        Serial.printf("[ACK] TIMEOUT waiting for cmd 0x%02X\n", ackExpectedCmd);
    }
}

/// Send LED relay command to STM32 via CAN 0x120.
/// @param front  true = turn on front relay, false = turn off
/// @param rear   true = turn on rear relay,  false = turn off
static void sendLedCommand(bool front, bool rear) {
    CanFrame frame = {};
    frame.identifier       = can::CMD_LED;
    frame.extd             = 0;
    frame.data_length_code = 2;
    frame.data[0]          = front ? 1 : 0;
    frame.data[1]          = rear  ? 1 : 0;
    ESP32Can.writeFrame(frame);
    ackBeginWait(can::CMD_LED & 0xFF);  // Low byte of 0x120 = 0x20
}

/// Send gear change command to STM32 via CAN 0x102 (CMD_MODE).
/// @param gear  Gear value (0=P, 1=R, 2=N, 3=D1, 4=D2)
static void sendGearCommand(uint8_t gear) {
    CanFrame frame = {};
    frame.identifier       = can::CMD_MODE;
    frame.extd             = 0;
    frame.data_length_code = 2;
    frame.data[0]          = currentModeFlags;  // Include current mode flags
    frame.data[1]          = gear;
    ESP32Can.writeFrame(frame);
    ackBeginWait(can::CMD_MODE & 0xFF);  // Low byte of 0x102 = 0x02
}

/// Send drive mode command to STM32 via CAN 0x102 (CMD_MODE).
/// @param modeFlags  Mode flags byte (bit 0 = 4x4, bit 1 = tank turn)
static void sendModeCommand(uint8_t modeFlags) {
    CanFrame frame = {};
    frame.identifier       = can::CMD_MODE;
    frame.extd             = 0;
    frame.data_length_code = 2;
    frame.data[0]          = modeFlags;
    frame.data[1]          = shifter::getGearRaw();  // Include current gear
    ESP32Can.writeFrame(frame);
    ackBeginWait(can::CMD_MODE & 0xFF);
}

// ---------------------------------------------------------------------------
// setup() — called once at power-on
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);

    // Reset cause reporting
    esp_reset_reason_t reason = esp_reset_reason();
    Serial.printf("[HMI] Reset reason: %s\n",
        reason == ESP_RST_POWERON  ? "PowerOn" :
        reason == ESP_RST_SW       ? "Software" :
        reason == ESP_RST_PANIC    ? "Panic" :
        reason == ESP_RST_INT_WDT  ? "Watchdog(INT)" :
        reason == ESP_RST_TASK_WDT ? "Watchdog(TASK)" :
        reason == ESP_RST_WDT      ? "Watchdog(OTHER)" :
        reason == ESP_RST_BROWNOUT ? "Brownout" :
        reason == ESP_RST_SDIO     ? "SDIO" :
        reason == ESP_RST_DEEPSLEEP ? "DeepSleep" :
                                      "Unknown");

    Serial.println("[HMI] ESP32 HMI CAN bring-up booted");

    // Initialize NVS config store
    config_store::init();

    // Initialize PSRAM
    if (psramInit()) {
        // Give system a moment to complete PSRAM initialization
        delay(10);
        Serial.printf("[PSRAM] Initialized — total: %u bytes, free: %u bytes\n",
                      ESP.getPsramSize(), ESP.getFreePsram());
    } else {
        Serial.println("[PSRAM] CRITICAL: Initialization FAILED — check board_build.arduino.memory_type");
        // Consider halting or entering degraded mode if PSRAM is required
        // while(1) { delay(1000); }
    }

    // Initialize TFT display
    tft.init();
    tft.setRotation(1);  // Landscape mode (480×320)

    // Apply touch calibration (XPT2046) — values from User_Setup.h
    // Run TFT_eSPI/examples/Generic/Touch_calibrate to get your own values.
    uint16_t calData[5] = TOUCH_CALIBRATION;
    tft.setTouch(calData);

    tft.fillScreen(0x2104);  // Dark gray background
    tft.setTextColor(0xFFFF, 0x2104);
    tft.setTextSize(1);
    Serial.println("[TFT] Display initialized (480x320 landscape)");

    ESP32Can.setPins(CAN_TX_PIN, CAN_RX_PIN);
    ESP32Can.setRxQueueSize(5);
    ESP32Can.setTxQueueSize(5);

    if (ESP32Can.begin(ESP32Can.convertSpeed(500))) {
        Serial.println("[CAN] Initialized at 500 kbps");
    } else {
        Serial.println("[CAN] Initialization FAILED");
    }

    // Initialize obstacle sensor driver (HC-SR04 on GPIO 6/7)
    obstacle_sensor::init();

    // Initialize CAN TX for obstacle distance frame (0x208)
    can_obstacle::init();

    // Initialize WS2812B LED controller (front GPIO 47 + rear GPIO 48)
    led_ctrl::init();

    // Initialize power manager (ignition key on GPIO 40/41)
    power_mgr::init();

    // Initialize DFPlayer audio (UART2 on GPIO 43/44)
    audio::init();

    // Initialize MCP23017 shifter input (I2C on GPIO 8/9)
    shifter::init();

    // Initialize traction switch (DPDT rocker on GPIO 15)
    traction_sw::init();

    // Initialize centralized touch handler
    touch::init();

    // Apply saved configuration
    {
        const auto& cfg = config_store::get();
        // Traction mode is now read from physical switch, not NVS
        currentModeFlags   = traction_sw::getModeFlag()
                           | (cfg.driveMode & can::MODE_FLAG_TANK_TURN);
        frontLedLocalState = cfg.frontLedEnabled;
        rearLedLocalState  = cfg.rearLedEnabled;

        // Apply saved audio volume to DFPlayer
        audio::setVolume(cfg.audioVolume);

        // Populate vehicleData with initial mode flags
        vehicle::ModeData md;
        md.modeFlags   = currentModeFlags;
        md.timestampMs = millis();
        vehicleData.setMode(md);
    }
}

// ---------------------------------------------------------------------------
// loop() — called repeatedly
// ---------------------------------------------------------------------------
void loop() {
    unsigned long now = millis();

    // Poll CAN RX and decode incoming frames
    RTMON_CAN_BEGIN();
    can_rx::poll(vehicleData);
    RTMON_CAN_END();

    // Check for pending ACK (non-blocking)
    ackCheck(vehicleData);

    // ---- Sync mode flags from STM32 heartbeat echo ----
    // The STM32 echoes the active mode flags in the heartbeat status_flags
    // (bits 1-2).  This allows the ESP32 to confirm the STM32 actually
    // applied the requested mode, even if the ACK was lost.
    // Guard: only sync after first valid heartbeat has been received
    // (timestampMs is 0 until the first CAN heartbeat is decoded).
    {
        const auto& hb = vehicleData.heartbeat();
        if (hb.timestampMs > 0) {
            uint8_t confirmedFlags = (hb.statusFlags >> 1) & 0x03;
            if (confirmedFlags != currentModeFlags) {
                currentModeFlags = confirmedFlags;
                vehicle::ModeData md;
                md.modeFlags   = currentModeFlags;
                md.timestampMs = millis();
                vehicleData.setMode(md);
            }
        }
    }

    // ---- STM32 heartbeat liveness monitoring (SAFETY FIX) ----
    // Verify the STM32 alive counter is incrementing each heartbeat period.
    // A frozen STM32 (timer ISR alive but main loop stuck) would send the same
    // counter value indefinitely — treat it as non-responsive after the freeze
    // threshold and inhibit further motion commands from this node.
    {
        const auto& hb = vehicleData.heartbeat();
        if (hb.timestampMs > 0) {
            uint8_t counter = hb.aliveCounter;
            if (counter != stm32HbLastCounter) {
                stm32HbLastCounter = counter;
                stm32HbSameCount   = 0;
                stm32IsAlive       = true;
            } else {
                if (stm32HbSameCount < STM32_HB_FREEZE_COUNT) {
                    stm32HbSameCount++;
                }
                stm32IsAlive = (stm32HbSameCount < STM32_HB_FREEZE_COUNT);
                if (!stm32IsAlive) {
                    Serial.println("[SAFETY] STM32 heartbeat counter frozen — inhibiting commands");
                }
            }

            // ---- Gear re-sync after STM32 restart (SAFETY FIX) ----
            // Detect startup_inhibit bit (statusFlags bit 0): 0→1 = STM32 just restarted.
            // When the bit later clears (0) AND state is ACTIVE, re-send the current gear
            // so the STM32 and shifter are synchronised.  Guards against the default
            // GEAR_FORWARD mis-match if the driver had previously selected PARK or REVERSE.
            bool startupInhibitNow = (hb.statusFlags & 0x01u) != 0;
            if (startupInhibitNow && !stm32StartupSeen) {
                // Rising edge of startup_inhibit → STM32 just booted/reset
                stm32StartupSeen  = true;
                gearResyncPending = true;
                Serial.println("[SAFETY] STM32 restart detected — gear resync pending");
            }
            if (gearResyncPending && !startupInhibitNow &&
                hb.systemState == can::SystemState::ACTIVE && stm32IsAlive) {
                // STM32 is now in ACTIVE with no startup inhibit — send current gear
                uint8_t curGear = shifter::getGearRaw();
                sendGearCommand(curGear);
                lastSentGear    = curGear;  // prevent duplicate send from shifter loop
                gearResyncPending = false;
                Serial.printf("[SAFETY] Gear resync sent: gear=%u\n", curGear);
            }
            if (!startupInhibitNow) {
                stm32StartupSeen = false;  // inhibit cleared, ready for next detection
            }
        }
    }

    // Update obstacle sensor and transmit CAN 0x208
    {
        // Compute average vehicle speed (0.1 km/h units → km/h)
        uint32_t speedSum = 0;
        for (uint8_t i = 0; i < 4; ++i) {
            speedSum += vehicleData.speed().raw[i];
        }
        float speedKmh = static_cast<float>(speedSum) / 4.0f * 0.1f;
        obstacle_sensor::update(speedKmh);
        can_obstacle::update();

        // Update VehicleData with latest sensor reading for HMI display
        obstacle_sensor::Reading rd = obstacle_sensor::getReading();
        vehicle::ObstacleData od;
        od.distanceCm  = rd.distance_mm / 10;
        od.timestampMs  = millis();
        vehicleData.setObstacle(od);
    }

    // Update HMI screen based on current vehicle state
    RTMON_UI_BEGIN();
    screenManager.update(vehicleData);
    RTMON_UI_END();

    // ---- Shifter gear update ----
    // Poll MCP23017, send CAN 0x102 on gear change.
    // SAFETY FIX: Gate gear commands on STM32 liveness.  If the STM32
    // heartbeat counter is frozen the vehicle controller is not responsive;
    // sending gear changes to a non-responding node could cause stale
    // commands to execute when the node recovers.
    {
        shifter::update();
        uint8_t gear = shifter::getGearRaw();
        if (gear != lastSentGear &&
            (now - lastGearSendMs) >= GEAR_SEND_DEBOUNCE_MS &&
            stm32IsAlive) {
            lastSentGear   = gear;
            lastGearSendMs = now;
            sendGearCommand(gear);
            // Play specific gear announcement
            switch (static_cast<shifter::Gear>(gear)) {
                case shifter::Gear::PARK:       audio::play(audio::Sound::GEAR_PARK,    audio::Priority::LO); break;
                case shifter::Gear::REVERSE:    audio::play(audio::Sound::GEAR_REVERSE, audio::Priority::LO); break;
                case shifter::Gear::NEUTRAL:    audio::play(audio::Sound::GEAR_NEUTRAL, audio::Priority::LO); break;
                case shifter::Gear::FORWARD:    audio::play(audio::Sound::GEAR_D1,      audio::Priority::LO); break;
                case shifter::Gear::FORWARD_D2: audio::play(audio::Sound::GEAR_D2,      audio::Priority::LO); break;
            }
            Serial.printf("[SHIFTER] Gear → %u\n", gear);
        }
    }

    // ---- Traction switch update ----
    // Poll DPDT rocker switch for 2WD/4WD selection.
    // CAN CMD_MODE is sent only when the switch state changes.
    // Speed gate is handled internally by traction_sw::update().
    {
        // Compute average vehicle speed (0.1 km/h units → km/h)
        uint32_t swSpeedSum = 0;
        for (uint8_t i = 0; i < 4; ++i) {
            swSpeedSum += vehicleData.speed().raw[i];
        }
        float swSpeedKmh = static_cast<float>(swSpeedSum) / 4.0f * 0.1f;

        traction_sw::update(swSpeedKmh);

        if (traction_sw::hasChanged() && stm32IsAlive) {
            // Update mode flags: preserve tank turn bit, set 4x4 from switch
            uint8_t tractionBit = traction_sw::getModeFlag();
            currentModeFlags = (currentModeFlags & can::MODE_FLAG_TANK_TURN)
                             | tractionBit;
            sendModeCommand(currentModeFlags);
            // Only persist tank turn to NVS; traction comes from physical switch
            config_store::setDriveMode(currentModeFlags & can::MODE_FLAG_TANK_TURN);
            {
                vehicle::ModeData md;
                md.modeFlags   = currentModeFlags;
                md.timestampMs = millis();
                vehicleData.setMode(md);
            }
            // Play traction audio feedback
            if (traction_sw::is4WD()) {
                audio::play(audio::Sound::TRACTION_4X4, audio::Priority::LO);
            } else {
                audio::play(audio::Sound::TRACTION_4X2, audio::Priority::LO);
            }
            Serial.printf("[TRACTION] Switch → %s, flags=0x%02X\n",
                          traction_sw::is4WD() ? "4WD" : "2WD",
                          currentModeFlags);
            traction_sw::clearChanged();
        }
    }

    // ---- Centralized touch handling ----
    {
        uint16_t tx = 0, ty = 0;
        bool isTouched = tft.getTouch(&tx, &ty);
        touch::update(isTouched, static_cast<int16_t>(tx),
                      static_cast<int16_t>(ty));

        touch::TouchEvent evt = touch::getEvent();

        if (evt.type == touch::EventType::LONG_PRESS) {
            // Long press on battery icon → open PIN entry screen
            screenManager.onLongPress(evt.x, evt.y);
        }

        if (evt.type == touch::EventType::TAP) {
            // Forward to screen manager (PIN / engineering dispatch)
            screenManager.onTouch(evt.x, evt.y);

            // LED toggle and mode icons are suppressed while PIN/engineering
            // overlay is active to avoid unintended commands
            if (!screenManager.isBlockingInput()) {
                // Front LED toggle
                if (ui::LedToggle::hitTestFront(evt.x, evt.y)) {
                    frontLedLocalState = !frontLedLocalState;
                    sendLedCommand(frontLedLocalState, rearLedLocalState);
                    config_store::setFrontLedEnabled(frontLedLocalState);
                    audio::play(frontLedLocalState ? audio::Sound::LIGHTS_ON
                                                   : audio::Sound::LIGHTS_OFF,
                                audio::Priority::LO);
                    Serial.printf("[LED] Front → %s\n",
                                  frontLedLocalState ? "ON" : "OFF");
                }

                // Rear LED toggle
                if (ui::LedToggle::hitTestRear(evt.x, evt.y)) {
                    rearLedLocalState = !rearLedLocalState;
                    sendLedCommand(frontLedLocalState, rearLedLocalState);
                    config_store::setRearLedEnabled(rearLedLocalState);
                    audio::play(rearLedLocalState ? audio::Sound::LIGHTS_ON
                                                  : audio::Sound::LIGHTS_OFF,
                                audio::Priority::LO);
                    Serial.printf("[LED] Rear → %s\n",
                                  rearLedLocalState ? "ON" : "OFF");
                }

                // Mode icons — 360° tank turn only (touch selectable)
                // 4x4/4x2 traction is now controlled by the physical switch.
                // SAFETY FIX: Only send mode command when STM32 is alive.
                uint8_t modeHit = ui::ModeIcons::hitTest(evt.x, evt.y);
                if (modeHit == 3 && stm32IsAlive) {
                    // 360° toggle: flip tank turn bit, preserve 4x4 from switch
                    currentModeFlags ^= can::MODE_FLAG_TANK_TURN;
                    audio::play(audio::Sound::BEEP, audio::Priority::LO);
                    sendModeCommand(currentModeFlags);
                    // Only persist tank turn to NVS; traction comes from physical switch
                    config_store::setDriveMode(currentModeFlags & can::MODE_FLAG_TANK_TURN);
                    {
                        vehicle::ModeData md;
                        md.modeFlags   = currentModeFlags;
                        md.timestampMs = millis();
                        vehicleData.setMode(md);
                    }
                    Serial.printf("[MODE] Flags → 0x%02X\n", currentModeFlags);
                }
            }
        }
    }

    // ---- Power management ----
    power_mgr::update();

    // Welcome audio on startup
    if (power_mgr::isRunning() && !welcomePlayed) {
        audio::play(audio::Sound::WELCOME, audio::Priority::HI);
        welcomePlayed  = true;
        farewellPlayed = false;
    }

    // Farewell audio on shutdown
    if (power_mgr::getState() == power_mgr::PowerState::SHUTTING_DOWN &&
        !farewellPlayed) {
        config_store::flush();  // Persist any unsaved changes before shutdown
        audio::play(audio::Sound::FAREWELL, audio::Priority::HI);
        farewellPlayed = true;
        welcomePlayed  = false;
        // Turn off LEDs during shutdown
        if (frontLedLocalState || rearLedLocalState) {
            frontLedLocalState = false;
            rearLedLocalState  = false;
            sendLedCommand(false, false);
        }
    }

    // ---- CAN-triggered audio events ----
    // Multi-error burst arbitration: when multiple errors fire within the
    // burst window, play the highest-severity sound (dominant fault) instead
    // of collapsing to ERROR_GENERAL.  (Phase 1 AUDIO_MITIGATION_PLAN)
    {
        auto st = static_cast<uint8_t>(vehicleData.heartbeat().systemState);

        // --- Helper lambda: queue an error/warning sound with burst arbitration ---
        auto playWarning = [&](audio::Sound sound, audio::Priority pri) {
            // Track burst window
            if (errorBurstCount == 0) {
                errorBurstStartMs = now;
                burstDominantSound = sound;
                burstDominantPri   = pri;
            }
            ++errorBurstCount;

            // Track highest-severity sound in burst
            if (burstSeverity(sound) > burstSeverity(burstDominantSound)) {
                burstDominantSound = sound;
                burstDominantPri   = pri;
            }

            if ((now - errorBurstStartMs) < ERROR_BURST_WINDOW_MS &&
                errorBurstCount >= ERROR_BURST_THRESHOLD) {
                // Multiple errors in short window → play dominant fault
                audio::play(burstDominantSound, burstDominantPri);
            } else {
                audio::play(sound, pri);
            }
        };

        // Reset burst counter when window expires
        if (errorBurstCount > 0 &&
            (now - errorBurstStartMs) >= ERROR_BURST_WINDOW_MS) {
            errorBurstCount = 0;
        }

        // --- System state transitions ---
        if (st != lastAudioSystemState) {
            // Emergency / safe state
            if (st == static_cast<uint8_t>(can::SystemState::SAFE) ||
                st == static_cast<uint8_t>(can::SystemState::ERROR)) {
                playWarning(audio::Sound::EMERGENCY, audio::Priority::HI);
            }
            // Degraded / limp-home
            else if (st == static_cast<uint8_t>(can::SystemState::DEGRADED) ||
                     st == static_cast<uint8_t>(can::SystemState::LIMP_HOME)) {
                playWarning(audio::Sound::ERROR_GENERAL, audio::Priority::HI);
            }
            // Recovery to ACTIVE from error state
            else if (st == static_cast<uint8_t>(can::SystemState::ACTIVE) &&
                     (lastAudioSystemState == static_cast<uint8_t>(can::SystemState::SAFE) ||
                      lastAudioSystemState == static_cast<uint8_t>(can::SystemState::ERROR))) {
                audio::play(audio::Sound::SAFETY_RESET, audio::Priority::MEDIUM);
            }
            lastAudioSystemState = st;
        }

        // --- Battery warnings (with separate low and critical thresholds) ---
        uint16_t battVoltRaw = vehicleData.battery().voltageRaw;
        if (battVoltRaw > 0 && battVoltRaw < BATTERY_CRIT_THRESHOLD_RAW) {
            if (!batteryCritPlayed) {
                playWarning(audio::Sound::BATTERY_CRITICAL, audio::Priority::HI);
                batteryCritPlayed = true;
                batteryLowPlayed  = true;  // skip low if critical already played
            }
        } else if (battVoltRaw > 0 && battVoltRaw < BATTERY_LOW_THRESHOLD_RAW) {
            if (!batteryLowPlayed) {
                playWarning(audio::Sound::BATTERY_LOW, audio::Priority::MEDIUM);
                batteryLowPlayed = true;
            }
        } else {
            batteryLowPlayed  = false;  // reset when voltage recovers
            batteryCritPlayed = false;
        }

        // --- Temperature warnings ---
        {
            int8_t maxTemp = INT8_MIN;
            for (uint8_t i = 0; i < vehicle::NUM_TEMP_SENSORS; ++i) {
                int8_t t = vehicleData.temp().temps[i];
                if (t > maxTemp) maxTemp = t;
            }
            if (maxTemp > TEMP_HIGH_THRESHOLD) {
                if (!tempHighPlayed) {
                    playWarning(audio::Sound::TEMP_HIGH, audio::Priority::MEDIUM);
                    tempHighPlayed = true;
                }
            } else if (tempHighPlayed && maxTemp < (TEMP_HIGH_THRESHOLD - TEMP_HIGH_HYSTERESIS)) {
                audio::play(audio::Sound::TEMP_NORMAL, audio::Priority::LO);
                tempHighPlayed = false;
            }
        }

        // --- ABS / TCS activation announcements ---
        {
            uint8_t absNow = vehicleData.safety().absActive;
            uint8_t tcsNow = vehicleData.safety().tcsActive;
            if (absNow != lastAbsActive) {
                audio::play(absNow ? audio::Sound::ABS_ON : audio::Sound::ABS_OFF,
                            audio::Priority::LO);
                lastAbsActive = absNow;
            }
            if (tcsNow != lastTcsActive) {
                audio::play(tcsNow ? audio::Sound::TCS_ON : audio::Sound::TCS_OFF,
                            audio::Priority::LO);
                lastTcsActive = tcsNow;
            }
        }

        // --- Overcurrent / sensor fault from safety error code ---
        {
            uint8_t safeErr = vehicleData.safety().errorCode;
            if (safeErr != lastSafetyError && safeErr != 0) {
                auto se = static_cast<can::SafetyError>(safeErr);
                switch (se) {
                    case can::SafetyError::OVERCURRENT:
                        playWarning(audio::Sound::OVERCURRENT, audio::Priority::HI);
                        break;
                    case can::SafetyError::OVERTEMP:
                        playWarning(audio::Sound::SENSOR_TEMP_ERROR, audio::Priority::MEDIUM);
                        break;
                    case can::SafetyError::SENSOR_FAULT:
                        playWarning(audio::Sound::SENSOR_SPEED_ERROR, audio::Priority::MEDIUM);
                        break;
                    case can::SafetyError::CENTERING:
                        playWarning(audio::Sound::ENCODER_ERROR, audio::Priority::MEDIUM);
                        break;
                    case can::SafetyError::I2C_FAILURE:
                        playWarning(audio::Sound::SENSOR_CURRENT_ERROR, audio::Priority::MEDIUM);
                        break;
                    case can::SafetyError::OBSTACLE:
                        playWarning(audio::Sound::OBSTACLE_WARN, audio::Priority::MEDIUM);
                        break;
                    default:
                        break;
                }
                lastSafetyError = safeErr;
            } else if (safeErr == 0) {
                lastSafetyError = 0;
            }
        }

        // --- Lights status from STM32 CAN echo ---
        {
            bool frontNow = vehicleData.lights().frontRelayOn;
            bool rearNow  = vehicleData.lights().rearRelayOn;
            if (lightsAudioInit) {
                if (frontNow != lastFrontRelayOn || rearNow != lastRearRelayOn) {
                    bool anyOn = frontNow || rearNow;
                    audio::play(anyOn ? audio::Sound::LIGHTS_ON
                                      : audio::Sound::LIGHTS_OFF,
                                audio::Priority::LO);
                }
            }
            lastFrontRelayOn = frontNow;
            lastRearRelayOn  = rearNow;
            lightsAudioInit  = true;
        }

        // --- Obstacle proximity warning — beep when zone ≥ 3 (critical/emergency) ---
        obstacle_sensor::Reading obsRd = obstacle_sensor::getReading();
        if (obsRd.healthy && obsRd.zone >= 3 &&
            (now - lastObstacleWarnMs) >= OBSTACLE_WARN_INTERVAL_MS) {
            audio::play(audio::Sound::OBSTACLE_WARN, audio::Priority::MEDIUM);
            lastObstacleWarnMs = now;
        }
    }

    // ---- Audio update ----
    audio::update();

    // ---- Periodic NVS flush (mitigate write wear) ----
    if ((now - lastNvsFlushMs) >= NVS_FLUSH_INTERVAL_MS) {
        lastNvsFlushMs = now;
        config_store::flush();
    }

    // ---- WS2812B LED update ----
    {
        auto st = vehicleData.heartbeat().systemState;
        bool frontEnabled = vehicleData.lights().frontRelayOn;

        // Enable/disable LED system based on front relay and system state
        // (front relay controls the WS2812B power for decorative effects)
        if (!frontEnabled || st == can::SystemState::BOOT || st == can::SystemState::STANDBY) {
            led_ctrl::setEnabled(false);
        } else {
            led_ctrl::setEnabled(true);
        }

        // Reverse detection from physical shifter
        bool reverse = (shifter::getGearRaw() == GEAR_REVERSE);

        // Braking & throttle detection: traction average near zero while
        // speed > 0 indicates dynamic braking or throttle release at speed.
        bool braking = false;
        float throttlePct;
        {
            uint32_t trSum = 0;
            for (uint8_t i = 0; i < 4; ++i) {
                trSum += vehicleData.traction().scale[i];
            }
            uint8_t trAvg = static_cast<uint8_t>(trSum / 4);
            throttlePct = static_cast<float>(trAvg);

            uint32_t spSum = 0;
            for (uint8_t i = 0; i < 4; ++i) {
                spSum += vehicleData.speed().raw[i];
            }
            braking = (spSum > LED_SPEED_SUM_THRESHOLD && trAvg <= LED_TRACTION_BRAKING_THRESHOLD);
        }

        // ---- Front LED mode from vehicle state ----
        if (st == can::SystemState::SAFE || st == can::SystemState::ERROR) {
            led_ctrl::startEmergencyFlash(5);
        } else if (st == can::SystemState::LIMP_HOME) {
            led_ctrl::setFrontMode(led_ctrl::FrontMode::KITT_IDLE);
        } else if (vehicleData.safety().absActive) {
            led_ctrl::setFrontMode(led_ctrl::FrontMode::ABS_ALERT);
        } else if (vehicleData.safety().tcsActive) {
            led_ctrl::setFrontMode(led_ctrl::FrontMode::TCS_ALERT);
        } else if (reverse) {
            led_ctrl::setFrontMode(led_ctrl::FrontMode::REVERSE);
        } else {
            // Throttle-reactive front patterns
            led_ctrl::setFrontFromThrottle(throttlePct);
        }

        // ---- Rear LED mode from driving state ----
        if (st == can::SystemState::SAFE || st == can::SystemState::ERROR) {
            led_ctrl::setRearMode(led_ctrl::RearMode::BRAKE_EMERGENCY);
        } else if (reverse) {
            led_ctrl::setRearMode(led_ctrl::RearMode::REVERSE);
        } else if (braking) {
            led_ctrl::setRearMode(led_ctrl::RearMode::BRAKE);
        } else {
            led_ctrl::setRearMode(led_ctrl::RearMode::POSITION);
        }

        // ---- Turn-signal derivation from steering angle (0.1° units) ----
        // Activate at 15° to signal the beginning of a turn.
        // Deactivate at 10° (5° hysteresis) to avoid flickering when the
        // wheel hovers around the threshold.
        // SAFE/ERROR states override with hazard flash.
        {
            static constexpr int16_t TURN_ON_RAW  = 150;  // 15.0° — activate
            static constexpr int16_t TURN_OFF_RAW = 100;  // 10.0° — deactivate (hysteresis)

            int16_t angle = vehicleData.steering().angleRaw;
            static led_ctrl::TurnSignal prevTurn = led_ctrl::TurnSignal::OFF;

            led_ctrl::TurnSignal turn = prevTurn;  // start from last state

            if (st == can::SystemState::SAFE || st == can::SystemState::ERROR) {
                turn = led_ctrl::TurnSignal::HAZARD;
                // Don't persist HAZARD into prevTurn — reset to OFF so that
                // when the system recovers, the hysteresis starts clean.
                prevTurn = led_ctrl::TurnSignal::OFF;
                led_ctrl::setTurnSignal(turn);
            } else {
                if (angle < -TURN_ON_RAW) {
                    turn = led_ctrl::TurnSignal::LEFT;
                } else if (angle > TURN_ON_RAW) {
                    turn = led_ctrl::TurnSignal::RIGHT;
                } else if (angle > -TURN_OFF_RAW && angle < TURN_OFF_RAW) {
                    turn = led_ctrl::TurnSignal::OFF;
                }
                // Between OFF_RAW and ON_RAW: keep previous state (hysteresis band)

                prevTurn = turn;
                led_ctrl::setTurnSignal(turn);
            }
        }

        // Single update call drives all animation + FastLED.show()
        led_ctrl::update();
    }

#if RUNTIME_MONITOR
    // Debug overlay — detect long touch (3 seconds) to toggle
    uint16_t touchX = 0, touchY = 0;
    bool touched = tft.getTouch(&touchX, &touchY);
    RTMON_OVERLAY_UPDATE(touched);
    RTMON_OVERLAY_DRAW(tft);
#endif

    // Send heartbeat 0x011 every 100 ms
    if (now - lastHeartbeatMs >= can::HEARTBEAT_INTERVAL_MS) {
        lastHeartbeatMs = now;

        CanFrame frame = {};
        frame.identifier       = can::HEARTBEAT_ESP32;
        frame.extd             = 0;
        frame.data_length_code = 1;
        frame.data[0]          = heartbeatCounter++;

        ESP32Can.writeFrame(frame);
    }

    // Serial heartbeat every ~1 second
    if (now - lastSerialMs >= 1000) {
        lastSerialMs = now;
        Serial.println("[HMI] heartbeat");
    }

#if RUNTIME_MONITOR
    // Runtime monitor serial log every LOG_INTERVAL_MS
    if (now - lastRtMonMs >= rtmon::LOG_INTERVAL_MS) {
        lastRtMonMs = now;
        RTMON_LOG();
    }
#endif
}
