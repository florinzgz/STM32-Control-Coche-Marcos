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

// ---- LED toggle touch tracking ----
static bool     ledLocalState     = false;   // local desired state (sent to STM32)
static unsigned long lastLedTouchMs = 0;     // debounce for touch
static constexpr unsigned long LED_TOUCH_DEBOUNCE_MS = 300;

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
static constexpr uint16_t BATTERY_LOW_THRESHOLD_RAW = 2000; // 20.00 V in 0.01 V units
static unsigned long lastObstacleWarnMs = 0;   // debounce obstacle warning beeps
static constexpr unsigned long OBSTACLE_WARN_INTERVAL_MS = 2000;

// ---- LED brake/reverse detection thresholds ----
// Speed sum threshold: 4 wheels × 0.5 km/h × 10 (0.1 km/h units) = 20
static constexpr uint32_t LED_SPEED_SUM_THRESHOLD = 20;
// Traction average below which we consider braking (0–100% scale)
static constexpr uint8_t LED_TRACTION_BRAKING_THRESHOLD = 5;

// ---- NVS flush interval ----
static unsigned long lastNvsFlushMs = 0;
static constexpr unsigned long NVS_FLUSH_INTERVAL_MS = 10000;  // 10 seconds

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
static void ackCheck(const vehicle::VehicleData& data) {
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
        Serial.printf("[ACK] TIMEOUT waiting for cmd 0x%02X\n", ackExpectedCmd);
    }
}

/// Send LED relay command to STM32 via CAN 0x120.
/// @param on  true = turn on, false = turn off
static void sendLedCommand(bool on) {
    CanFrame frame = {};
    frame.identifier       = can::CMD_LED;
    frame.extd             = 0;
    frame.data_length_code = 1;
    frame.data[0]          = on ? 1 : 0;
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

    // Initialize WS2812B LED controller (GPIO 38, 44 LEDs)
    led_ctrl::init();

    // Initialize power manager (ignition key on GPIO 40/41)
    power_mgr::init();

    // Initialize DFPlayer audio (UART2 on GPIO 43/44)
    audio::init();

    // Initialize MCP23017 shifter input (I2C on GPIO 8/9)
    shifter::init();

    // Initialize centralized touch handler
    touch::init();

    // Apply saved configuration
    {
        const auto& cfg = config_store::get();
        currentModeFlags = cfg.driveMode;
        ledLocalState    = cfg.ledEnabled;

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
    // Poll MCP23017, send CAN 0x102 on gear change
    {
        shifter::update();
        uint8_t gear = shifter::getGearRaw();
        if (gear != lastSentGear &&
            (now - lastGearSendMs) >= GEAR_SEND_DEBOUNCE_MS) {
            lastSentGear   = gear;
            lastGearSendMs = now;
            sendGearCommand(gear);
            audio::play(audio::Sound::GEAR_CHANGE, audio::Priority::LOW);
            Serial.printf("[SHIFTER] Gear → %u\n", gear);
        }
    }

    // ---- Centralized touch handling ----
    {
        uint16_t tx = 0, ty = 0;
        bool isTouched = tft.getTouch(&tx, &ty);
        touch::update(isTouched, static_cast<int16_t>(tx),
                      static_cast<int16_t>(ty));

        touch::TouchEvent evt = touch::getEvent();
        if (evt.type == touch::EventType::TAP) {
            // Forward to screen manager for secret code / engineering screen
            screenManager.onTouch(evt.x, evt.y);

            // LED toggle
            if (ui::LedToggle::hitTest(evt.x, evt.y)) {
                ledLocalState = !ledLocalState;
                sendLedCommand(ledLocalState);
                config_store::setLedEnabled(ledLocalState);
                Serial.printf("[LED] Toggle → %s\n",
                              ledLocalState ? "ON" : "OFF");
            }

            // Mode icons (4x4 / 4x2 / 360°)
            uint8_t modeHit = ui::ModeIcons::hitTest(evt.x, evt.y);
            if (modeHit > 0) {
                switch (modeHit) {
                    case 1:  // 4x4 → set 4x4 flag, clear tank
                        currentModeFlags = can::MODE_FLAG_4X4;
                        break;
                    case 2:  // 4x2 → clear both flags
                        currentModeFlags = 0;
                        break;
                    case 3:  // 360° → set tank turn flag
                        currentModeFlags = can::MODE_FLAG_TANK_TURN;
                        break;
                }
                sendModeCommand(currentModeFlags);
                config_store::setDriveMode(currentModeFlags);
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

    // ---- Power management ----
    power_mgr::update();

    // Welcome audio on startup
    if (power_mgr::isRunning() && !welcomePlayed) {
        audio::play(audio::Sound::WELCOME, audio::Priority::HIGH);
        welcomePlayed  = true;
        farewellPlayed = false;
    }

    // Farewell audio on shutdown
    if (power_mgr::getState() == power_mgr::PowerState::SHUTTING_DOWN &&
        !farewellPlayed) {
        config_store::flush();  // Persist any unsaved changes before shutdown
        audio::play(audio::Sound::FAREWELL, audio::Priority::HIGH);
        farewellPlayed = true;
        welcomePlayed  = false;
        // Turn off LEDs during shutdown
        if (ledLocalState) {
            ledLocalState = false;
            sendLedCommand(false);
        }
    }

    // ---- CAN-triggered audio events ----
    {
        auto st = static_cast<uint8_t>(vehicleData.heartbeat().systemState);

        // Error / Safe state alert — play once on transition
        if ((st == 4 || st == 5) && lastAudioSystemState != st) {
            audio::play(audio::Sound::ERROR_ALERT, audio::Priority::HIGH);
        }
        lastAudioSystemState = st;

        // Battery low warning — play once when voltage drops below threshold
        uint16_t battVoltRaw = vehicleData.battery().voltageRaw;
        if (battVoltRaw > 0 && battVoltRaw < BATTERY_LOW_THRESHOLD_RAW) {
            if (!batteryLowPlayed) {
                audio::play(audio::Sound::BATTERY_LOW, audio::Priority::MEDIUM);
                batteryLowPlayed = true;
            }
        } else {
            batteryLowPlayed = false;  // reset when voltage recovers
        }

        // Obstacle proximity warning — beep when zone ≥ 3 (critical/emergency)
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

        // Reverse detection from physical shifter
        bool reverse = (shifter::getGearRaw() == GEAR_REVERSE);

        // Braking detection: traction average near zero while speed > 0
        // indicates dynamic braking or throttle release at speed.
        bool braking = false;
        {
            uint32_t trSum = 0;
            for (uint8_t i = 0; i < 4; ++i) {
                trSum += vehicleData.traction().scale[i];
            }
            uint8_t trAvg = static_cast<uint8_t>(trSum / 4);

            uint32_t spSum = 0;
            for (uint8_t i = 0; i < 4; ++i) {
                spSum += vehicleData.speed().raw[i];
            }
            braking = (spSum > LED_SPEED_SUM_THRESHOLD && trAvg <= LED_TRACTION_BRAKING_THRESHOLD);
        }

        bool ledEnabled = vehicleData.lights().relayOn;
        led_ctrl::update(static_cast<uint8_t>(st), braking, reverse, ledEnabled);
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
