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
#include <ESP32-TWAI-CAN.hpp>
#include <TFT_eSPI.h>
#include "can_ids.h"
#include "can_rx.h"
#include "vehicle_data.h"
#include "screen_manager.h"
#include "ui/runtime_monitor.h"
#include "ui/debug_overlay.h"

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

// ---- Command ACK tracking (Phase 13) ----
// Non-blocking: records when a command was sent and checks for ACK arrival.
// UI state is only updated once ACK is received or timeout expires.
// Design: no automatic retry — bounded timeout only, no infinite loops.

static bool     ackPending     = false;   // true while waiting for ACK
static uint8_t  ackExpectedCmd = 0;       // low byte of the command CAN ID we sent
static unsigned long ackSentMs = 0;       // timestamp when the command was sent
static bool     ackTimedOut    = false;   // set true if ACK_TIMEOUT_MS elapsed

/// Call before sending a command that expects ACK (CMD_MODE, SERVICE_CMD).
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

// ---------------------------------------------------------------------------
// setup() — called once at power-on
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("[HMI] ESP32 HMI CAN bring-up booted");

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

    // Update HMI screen based on current vehicle state
    RTMON_UI_BEGIN();
    screenManager.update(vehicleData);
    RTMON_UI_END();

#if RUNTIME_MONITOR
    // Debug overlay — detect long touch (3 seconds) to toggle
    bool touched = tft.getTouch(nullptr, nullptr);
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
