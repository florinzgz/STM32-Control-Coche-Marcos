// =============================================================================
// ESP32-S3 HMI Firmware — Main Entry Point
//
// Framework:  Arduino (C++17)
// Board:      ESP32-S3-DevKitC-1
// Reference:  docs/CAN_CONTRACT_FINAL.md rev 1.0
// =============================================================================

#include <Arduino.h>

// ---------------------------------------------------------------------------
// setup() — called once at power-on
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    Serial.println("[HMI] ESP32-S3 HMI firmware starting...");
    Serial.println("[HMI] Framework: Arduino");
    Serial.println("[HMI] Role: HMI (intent sender) — STM32 is safety authority");
}

// ---------------------------------------------------------------------------
// loop() — called repeatedly
// ---------------------------------------------------------------------------
void loop() {
    // Placeholder — CAN communication and HMI logic will be added
    // in subsequent development phases.
    delay(1000);
}
