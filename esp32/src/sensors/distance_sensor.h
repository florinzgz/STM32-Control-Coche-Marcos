// =============================================================================
// ESP32-S3 — Distance Sensor Interface (Abstract)
//
// Common interface for UART-based distance/LiDAR sensors used for obstacle
// detection.  Concrete implementations should provide:
//
//   init()          — Configure UART and sensor hardware
//   update(speed)   — Read and parse sensor data (non-blocking)
//   getReading()    — Return latest validated Reading struct
//
// Current implementations:
//   - TOFSense-M 8×8 (Nooploop) — obstacle_sensor.h  [DISABLED: hardware removed]
//
// Planned implementations:
//   - Benewake TF-Mini Plus (UART LiDAR, 115200 baud, 9-byte frame)
//
// All implementations share the same obstacle_sensor::Reading struct and
// SensorStatus enum, so CAN, UI, and safety modules require NO changes
// when swapping sensor hardware.
//
// ---- TF-MINI PLUS INTEGRATION GUIDE ----------------------------------------
//
// Benewake TF-Mini Plus specifications:
//   - Interface: UART, 115200 baud, 8N1
//   - Frame: 9 bytes [0x59 0x59 DIST_L DIST_H STR_L STR_H TEMP_L TEMP_H CHK]
//   - Distance: uint16 LE (cm), range 10–1200 cm
//   - Strength: uint16 LE (signal quality, reject if < 100 or > 65535)
//   - Checksum: low byte of sum of bytes [0..7]
//   - Output rate: configurable (1–1000 Hz, default 100 Hz)
//
// Integration steps:
//
//   1. UART CONFIGURATION
//      In obstacle_sensor.h Config struct:
//        - baudRate = 115200  (not 921600)
//        - rxBufSize = 256    (9-byte frames, much smaller than TOFSense-M)
//        - rxPin = same GPIO 18 (or remap as needed)
//        - maxRangeMm = 12000 (TF-Mini Plus range = 12 m)
//
//   2. FRAME PARSING
//      In obstacle_sensor.cpp, add a new parse function:
//        - parseTfMiniFrame(buf, len, outDistMm)
//        - Header: two consecutive 0x59 bytes
//        - Distance in cm (bytes 2-3, uint16 LE) → convert to mm
//        - Strength check (bytes 4-5): reject if < 100
//        - Checksum: sum of bytes [0..7] & 0xFF == byte[8]
//
//   3. UPDATE LOOP
//      In obstacle_sensor.cpp update():
//        - Change frame length to 9 bytes (not 400)
//        - Sync on 0x59 0x59 double-header (not 0x57 0x01)
//        - MAX_BYTES_PER_UPDATE can be much smaller (~100 bytes)
//        - No pixel statistics (single distance value, not 8×8 matrix)
//        - Stuck detection and zone mapping remain the same
//
//   4. COMPILE-TIME SELECTION
//      In obstacle_sensor.h:
//        - Set OBSTACLE_SENSOR_ENABLED to 1
//        - Add a new flag: SENSOR_TYPE_TFMINI (vs SENSOR_TYPE_TOFSENSE)
//        - Use #if to select the correct parser and UART config
//
//   5. CAN / UI / SAFETY INTEGRATION
//      No changes needed — the Reading struct and getReading() API remain
//      identical.  CAN frames 0x208/0x209 use distance_mm, zone, healthy,
//      and stuck fields, which are populated by any sensor implementation.
//
//   6. HARDWARE WIRING
//      TF-Mini Plus pinout (4-pin JST-GH):
//        Pin 1 = +5V (requires 5V, ~140mA peak)
//        Pin 2 = GND
//        Pin 3 = TX → ESP32 RX (Config.rxPin, default GPIO 18)
//        Pin 4 = RX ← ESP32 TX (Config.txPin, optional for commands)
//      UART levels: 3.3V compatible — NO voltage divider needed
//      (unlike TOFSense-M which outputs 3.5–3.6V)
//
// ---- RISKS AND CONSIDERATIONS ----------------------------------------------
//
//   - TF-Mini Plus is a single-point sensor (1 distance value), NOT a matrix.
//     The pixel statistics fields (minDist, maxDist, avgDist, validCount,
//     dispersion) will all report the same single distance value.
//     Callers that rely on these fields should handle this gracefully.
//
//   - The baud rate change (921600 → 115200) means rxBufSize can be reduced
//     significantly (256 bytes vs 4096), saving RAM.
//
//   - TF-Mini Plus distance is in cm (not µm like TOFSense-M). The parser
//     must convert to mm for the Reading struct.
//
//   - Signal strength filtering is critical: reject readings with
//     strength < 100 (unreliable) or strength == 65535 (saturation).
//
//   - At 100 Hz output rate, one 9-byte frame arrives every 10 ms.
//     At 115200 baud, each byte takes ~87 µs, so a frame takes ~0.8 ms.
//     No risk of buffer overflow with even a 256-byte buffer.
//
// =============================================================================

#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

// This header documents the distance sensor abstraction.
// The actual API is in obstacle_sensor.h (namespace obstacle_sensor):
//
//   void init(const Config& cfg);     // Configure and start sensor
//   void update(float speedKmh);      // Read/parse data (non-blocking)
//   Reading getReading();             // Get latest validated reading
//
// The Reading struct, SensorStatus enum, and Config struct are defined
// in obstacle_sensor.h.  All sensor implementations populate these
// same types, ensuring transparent swappability for CAN, UI, and
// safety modules.
//
// To add a new sensor type:
//   1. Add a SENSOR_TYPE_xxx define in obstacle_sensor.h
//   2. Implement the parse function in obstacle_sensor.cpp
//   3. Select the parser in update() via #if SENSOR_TYPE_xxx
//   4. Adjust Config defaults (baudRate, rxBufSize, maxRangeMm)
//   5. Set OBSTACLE_SENSOR_ENABLED to 1

#endif // DISTANCE_SENSOR_H
