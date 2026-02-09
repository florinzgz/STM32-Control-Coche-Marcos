// =============================================================================
// ESP32-S3 HMI Firmware — CAN Bring-Up
//
// Framework:  Arduino (C++17)
// Board:      ESP32-S3-DevKitC-1
// Reference:  docs/CAN_CONTRACT_FINAL.md rev 1.0
// =============================================================================

#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include "can_ids.h"
#include "can_rx.h"
#include "vehicle_data.h"

// CAN transceiver pins (TJA1051 — see platformio.ini header)
static constexpr int CAN_TX_PIN = 4;
static constexpr int CAN_RX_PIN = 5;

static vehicle::VehicleData vehicleData;

static uint8_t  heartbeatCounter = 0;
static unsigned long lastHeartbeatMs  = 0;
static unsigned long lastSerialMs     = 0;

// ---------------------------------------------------------------------------
// setup() — called once at power-on
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("[HMI] ESP32 HMI CAN bring-up booted");

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
    can_rx::poll(vehicleData);

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
}
