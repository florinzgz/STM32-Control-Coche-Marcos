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
#include <esp_idf_version.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <ESP32-TWAI-CAN.hpp>
#include <driver/twai.h>
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
#include "touch_calibration.h"
#include "config_store.h"
#include "traction_switch.h"

// =============================================================================
// PSRAM Diagnostic — Verifica que la PSRAM OPI de 8MB está activa y funcional
// =============================================================================
static void psram_diagnostic() {
    Serial.println("=== PSRAM DIAGNOSTIC ===");

    if (psramInit()) {
        Serial.println("PSRAM OK: inicializada correctamente");
    } else {
        Serial.println("PSRAM ERROR: no detectada o fallo de inicialización");
        return;
    }

    size_t total = ESP.getPsramSize();
    size_t libre = ESP.getFreePsram();

    Serial.printf("PSRAM total: %u bytes\n", (unsigned)total);
    Serial.printf("PSRAM libre: %u bytes\n", (unsigned)libre);

    // Reservar la mitad de la PSRAM disponible (seguro para N16R8: 8MB)
    size_t testSize = total / 2;
    uint8_t* block = (uint8_t*) ps_malloc(testSize);

    if (block) {
        Serial.printf("PSRAM TEST: asignación de %u bytes OK\n", (unsigned)testSize);
        free(block);
    } else {
        Serial.printf("PSRAM TEST: fallo asignando %u bytes\n", (unsigned)testSize);
    }

    Serial.println("==========================");
}

// =============================================================================
// Runtime Overlay — Muestra PSRAM, RAM, FPS y tiempo de render en pantalla
// =============================================================================
static void draw_runtime_overlay(TFT_eSPI& tft, uint32_t frameTimeMs) {
    // Fondo semitransparente
    tft.fillRect(0, 0, 160, 60, TFT_BLACK);

    // PSRAM y RAM — stack-only formatting (no heap String class)
    char line[32];
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextDatum(TL_DATUM);

    snprintf(line, sizeof(line), "PSRAM: %u KB",
             (unsigned)(ESP.getFreePsram() / 1024));
    tft.drawString(line, 4, 4);

    snprintf(line, sizeof(line), "RAM:   %u KB",
             (unsigned)(ESP.getFreeHeap() / 1024));
    tft.drawString(line, 4, 20);

    // FPS y tiempo de render
    uint32_t fps = frameTimeMs > 0 ? 1000 / frameTimeMs : 0;
    snprintf(line, sizeof(line), "FPS: %lu", (unsigned long)fps);
    tft.drawString(line, 4, 36);
}

// CAN transceiver pins (SN65HVD230, 3.3V — module Rs/SLNT pin tied to GND)
static constexpr int CAN_TX_PIN = 4;
static constexpr int CAN_RX_PIN = 5;

// Global TFT instance — used by all screens via extern
TFT_eSPI tft = TFT_eSPI();

static vehicle::VehicleData vehicleData;
static ScreenManager screenManager;

static uint8_t  heartbeatCounter = 0;
static unsigned long lastHeartbeatMs  = 0;
static unsigned long lastSerialMs     = 0;
static unsigned long lastCanDiagMs    = 0;   // TWAI bus diagnostic interval
static unsigned long lastBusOffCheckMs = 0;  // TWAI bus-off recovery check
static uint8_t       busOffRecoveryCount = 0;

/* Error-passive recovery state.
 * When TEC reaches 128 the TWAI controller enters error-passive but keeps
 * state = RUNNING — the existing bus-off recovery never fires.  If the
 * condition persists for ERROR_PASSIVE_TIMEOUT_MS we do a full driver
 * reinit (stop → uninstall → install → start) to clear error counters.
 *
 * Two-phase strategy:
 *   Phase 1 (fast):  up to ERROR_PASSIVE_MAX_FAST attempts every 3 s.
 *   Phase 2 (slow):  unlimited attempts every 30 s — avoids hammering the
 *                     bus while still recovering if the remote node (STM32)
 *                     comes online later. */
static constexpr uint32_t ERROR_PASSIVE_TIMEOUT_MS      = 3000;   // fast-phase interval
static constexpr uint32_t ERROR_PASSIVE_SLOW_TIMEOUT_MS = 30000;  // slow-phase interval
static constexpr uint8_t  ERROR_PASSIVE_MAX_FAST        = 10;     // fast-phase attempts
static unsigned long errorPassiveSince  = 0;  // 0 = not in error-passive
static uint8_t       errorPassiveResets = 0;
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
// Brownout/reboot guard: minimum interval between WELCOME sounds.
// On rapid MCU resets (brownout, watchdog) the ESP32 may restart
// multiple times in quick succession.  This timestamp prevents audio
// spam by enforcing a cooldown.  NOTE: static RAM is NOT guaranteed
// to persist across all reset types on all ESP32 variants.  On a
// full power cycle, lastWelcomePlayMs reinitialises to 0 (desired:
// fresh boot always plays welcome).  On brownout or SW reset, the
// per-sound cooldown in audio_manager.cpp (SOUND_COOLDOWN_MS=4s)
// provides a secondary safety net even if this variable is lost.
static unsigned long lastWelcomePlayMs = 0;
static constexpr unsigned long WELCOME_COOLDOWN_MS = 5000;  // 5 s minimum gap

// ---- Runtime overlay state ----
static bool     showOverlay       = true;    // overlay visible on initial screen only
static uint32_t lastFrameStart    = 0;

// ---- Audio event tracking ----
static uint8_t  lastAudioSystemState = 0;     // detect state transitions for error alert
static bool     batteryLowPlayed  = false;     // one-shot for battery warning
static bool     batteryCritPlayed = false;     // one-shot for battery critical
static constexpr uint16_t BATTERY_LOW_THRESHOLD_RAW = 2000;  // 20.00 V in 0.01 V units
static constexpr uint16_t BATTERY_CRIT_THRESHOLD_RAW = 1800; // 18.00 V in 0.01 V units
static unsigned long lastObstacleWarnMs = 0;   // debounce obstacle warning beeps
static constexpr unsigned long OBSTACLE_WARN_INTERVAL_MS = 2000;

// ---- Runtime counter for maintenance tracking ----
static unsigned long lastRuntimeTickMs = 0;
static uint32_t      runtimeAccumMs    = 0;       // partial seconds accumulated in RAM (§2.2)
static constexpr unsigned long RUNTIME_COMMIT_INTERVAL_MS = 60000;  // commit to NVS every 60 s
static bool     maintenanceWarnPlayed = false;

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
// Traction average below which we consider regen deceleration (above braking).
// Range: braking < trAvg ≤ regen threshold, with speed > 0.
// This triggers the blue-pulse regen indicator on the rear centre zone.
static constexpr uint8_t LED_TRACTION_REGEN_THRESHOLD = 20;

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
static bool     tempMapResyncPending = false;  // true = need to re-send DS18B20 map after STM32 restart

// ---- Render task (Core 0) — offloads TFT + touch from main loop ----
static SemaphoreHandle_t vdMutex = nullptr;
static vehicle::VehicleData renderVD;  // shared copy for render task
static QueueHandle_t touchActionQueue = nullptr;

enum class TouchAction : uint8_t {
    FRONT_LED_TOGGLE,
    REAR_LED_TOGGLE,
    TANK_MODE_TOGGLE,
};

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
    ESP32Can.writeFrame(frame, 0);  // Non-blocking: drop if TX queue full
    ackBeginWait(can::CMD_LED & 0xFF);  // Low byte of 0x120 = 0x20
}

/// Send pre-power-cut safe-state request to STM32 via CAN 0x130.
/// Called once on entry to SHUTTING_DOWN, before the external delay
/// relay physically removes power.  The STM32 reuses existing primitives
/// (Traction_EmergencyStop / Steering_Neutralize / Relay_PowerDown) to
/// force PWM=0, EN=LOW, relays OFF.  No ACK is expected.  If the frame
/// is lost on the bus, behaviour is unchanged — the hardware delay
/// relay still cuts power exactly as before.
static void sendSystemShutdown() {
    CanFrame frame = {};
    frame.identifier       = can::CMD_SYSTEM_SHUTDOWN;
    frame.extd             = 0;
    frame.data_length_code = 0;     // Empty payload (STM32 ignores any byte)
    ESP32Can.writeFrame(frame, 0);  // Non-blocking: drop if TX queue full
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
    ESP32Can.writeFrame(frame, 0);  // Non-blocking: drop if TX queue full
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
    ESP32Can.writeFrame(frame, 0);  // Non-blocking: drop if TX queue full
    ackBeginWait(can::CMD_MODE & 0xFF);
}

// ---------------------------------------------------------------------------
// renderTask() — FreeRTOS task pinned to Core 0
//
// Handles all TFT display and touch operations, keeping the main loop()
// on Core 1 free for CAN, sensors, audio, and LEDs without SPI blocking.
// The TFT SPI bus (~28 ms per frame) no longer stalls CAN polling or
// safety-critical processing.
//
// FRAME LATCH ARCHITECTURE (deterministic render):
//   1. Acquire mutex → copy renderVD into localVD (frozen snapshot)
//   2. Pass localVD to screenManager.update(localVD):
//      a. screen.update(localVD) copies data into cur_* members
//      b. frameLimiter gates screen.draw() to 20 FPS
//      c. screen.draw() reads only from cur_*/prev_* — NOT from localVD
//   3. Result: each rendered frame represents a single, consistent time instant
//      Even if CAN updates arrive mid-render, the displayed frame is atomic.
// ---------------------------------------------------------------------------
static void renderTask(void* /*param*/) {
    vehicle::VehicleData localVD;

    // First-boot policy: if the touch-calibration wizard has never been
    // completed on this device, launch it once on the very first render
    // tick.  After SAVE the `first_done` NVS flag prevents this from
    // firing again — the wizard is reachable only via the engineering
    // menu thereafter.  The check is cheap (single NVS read) and runs
    // exactly once per boot.
    bool firstBootCheckPending = !touch_calibration::firstBootDone();

    for (;;) {
        // 1. Copy latest vehicle data from main loop
        if (xSemaphoreTake(vdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            localVD = renderVD;
            xSemaphoreGive(vdMutex);
        }

        // First-boot wizard trigger (executed at most once per boot, on
        // the first iteration after the screen manager is alive).  The
        // engineering-menu "TOUCH CALIBRATION" entry is handled atomically
        // inside ScreenManager::update() (right after engineering exit)
        // so we don't need to poll for it here.
        if (firstBootCheckPending) {
            firstBootCheckPending = false;
            screenManager.requestTouchWizard(/*firstBoot=*/true);
        }

        // 2. Update & render screen (frame limiter inside screenManager)
        lastFrameStart = millis();
        screenManager.update(localVD);

        // 3. Runtime overlay on boot screen only
        {
            uint32_t frameTime = millis() - lastFrameStart;
            if (showOverlay && screenManager.isInitialScreen()) {
                draw_runtime_overlay(tft, frameTime);
            } else {
                showOverlay = false;
            }
        }

        // 4. Centralized touch handling (same SPI bus as TFT)
        {
            uint16_t tx = 0, ty = 0;
            bool isTouched = tft.getTouch(&tx, &ty);
            touch::update(isTouched, static_cast<int16_t>(tx),
                          static_cast<int16_t>(ty));

            touch::TouchEvent evt = touch::getEvent();

            if (evt.type == touch::EventType::LONG_PRESS) {
                // Block long-press while tank confirm modal is active (§3.1)
                if (!screenManager.isTankConfirmActive()) {
                    screenManager.onLongPress(evt.x, evt.y);
                }
            }

            if (evt.type == touch::EventType::TAP) {
                screenManager.onTouch(evt.x, evt.y);

                if (!screenManager.isBlockingInput()) {
                    // ---- Tank turn confirmation dialog active ----
                    // If the confirm bar is visible, route ALL taps to it first.
                    if (screenManager.isTankConfirmActive()) {
                        uint8_t confirmResult = screenManager.handleTankConfirmTouch(evt.x, evt.y);
                        if (confirmResult == 1) {
                            // User confirmed YES → queue the actual toggle
                            TouchAction act = TouchAction::TANK_MODE_TOGGLE;
                            xQueueSend(touchActionQueue, &act, 0);
                        }
                        // confirmResult 2 = NO (dismissed), 0 = consumed
                        // In all cases, skip normal touch handling this frame
                    } else {
                        // Normal touch handling (no confirm dialog)
                        if (ui::LedToggle::hitTestFront(evt.x, evt.y)) {
                            TouchAction act = TouchAction::FRONT_LED_TOGGLE;
                            xQueueSend(touchActionQueue, &act, 0);
                        }
                        if (ui::LedToggle::hitTestRear(evt.x, evt.y)) {
                            TouchAction act = TouchAction::REAR_LED_TOGGLE;
                            xQueueSend(touchActionQueue, &act, 0);
                        }
                        uint8_t modeHit = ui::ModeIcons::hitTest(evt.x, evt.y);
                        if (modeHit == 3) {
                            // Show confirmation dialog instead of immediate toggle
                            screenManager.showTankConfirm();
                        }
                    }
                }
            }

#if RUNTIME_MONITOR
            // Debug overlay toggle (3 s hold) and draw
            // Uses lastFrameStart (captured once per frame at line 367)
            // to comply with frame time contract — no direct millis() in UI.
            RTMON_OVERLAY_UPDATE(isTouched, lastFrameStart);
            RTMON_OVERLAY_DRAW(tft, lastFrameStart);
#endif
        }

        vTaskDelay(1);  // yield to other Core 0 tasks (WiFi, BT, etc.)
    }
}

// ---------------------------------------------------------------------------
// twaiInit() — (re-)install and start the TWAI driver with project timing.
//
// Returns true on success.  Called from setup() for the initial bring-up and
// from the error-passive recovery path when the controller is stuck with
// TEC ≥ 128 but never reaches bus-off.
// ---------------------------------------------------------------------------
static bool twaiInit() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        static_cast<gpio_num_t>(CAN_TX_PIN),
        static_cast<gpio_num_t>(CAN_RX_PIN),
        TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 5;
    g_config.tx_queue_len = 5;

    /* Start from the standard 500 kbps macro so that clk_src (and
     * quanta_resolution_hz on ESP-IDF ≥ 5.x) are initialised to the
     * correct platform defaults.  The previous memset(&t_config,0,...)
     * zeroed clk_src, which is NOT TWAI_CLK_SRC_DEFAULT on ESP-IDF 5.x
     * — causing twai_driver_install() to fail or to select the wrong
     * clock, producing a baud-rate mismatch with the STM32 FDCAN.
     *
     * We then override brp/tseg/sjw to achieve an 87.5 % sample point
     * (CiA 301 recommended) that closely matches the STM32's 88.2 %. */
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

    /* CRITICAL FIX: On ESP-IDF ≥ 5.0, TWAI_TIMING_CONFIG_500KBITS()
     * sets quanta_resolution_hz = 10 000 000 (10 MHz) with brp = 0.
     * When quanta_resolution_hz > 0, twai_driver_install() calculates:
     *   brp = APB_CLK / quanta_resolution_hz = 80 MHz / 10 MHz = 8
     * This produces:
     *   bit time = (1+13+2) × 8/80 MHz = 1600 ns → 625 kbps
     *
     * The STM32 FDCAN is configured for 500 kbps (170 MHz / 10 / 34).
     * A 25 % baud-rate mismatch makes communication IMPOSSIBLE — every
     * frame fails bit-level validation (stuff/CRC errors, no ACKs).
     *
     * Fix: clear quanta_resolution_hz so the driver uses the manually
     * specified brp = 10 directly:
     *   bit time = (1+13+2) × 10/80 MHz = 2000 ns → 500 kbps  ✓
     *
     * The clk_src field (set by the macro) is preserved, ensuring the
     * correct APB clock source is selected on all ESP-IDF versions.   */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    t_config.quanta_resolution_hz = 0;
#endif
    t_config.brp            = 10;
    t_config.tseg_1         = 13;
    t_config.tseg_2         = 2;
    t_config.sjw            = 2;
    t_config.triple_sampling = false;

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        return false;
    }
    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// setup() — called once at power-on
// ---------------------------------------------------------------------------
void setup() {
    Serial.setTxBufferSize(512);   // Prevent Serial.printf() blocking on long diagnostics
    Serial.begin(115200);
    delay(500);
    psram_diagnostic();

    // Reset cause reporting
    esp_reset_reason_t reason = esp_reset_reason();
    Serial.printf("[BOOT][INFO] Reset reason: %s\n",
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

    Serial.println("[BOOT][INFO] ESP32 HMI CAN bring-up booted");

    // Initialize NVS config store
    config_store::init();

    // Initialize persistent touch-calibration NVS namespace.  Must be done
    // BEFORE tft.setTouch() so the loaded calibration (if valid) takes
    // precedence over the compile-time TOUCH_CALIBRATION fallback.
    touch_calibration::init();

    // Initialize PSRAM
    if (psramInit()) {
        // Give system a moment to complete PSRAM initialization
        delay(10);
        Serial.printf("[BOOT][INFO] PSRAM initialized — total: %u bytes, free: %u bytes\n",
                      (unsigned)ESP.getPsramSize(), (unsigned)ESP.getFreePsram());
    } else {
        Serial.println("[BOOT][ERR] PSRAM initialization FAILED — check board_build.arduino.memory_type");
        // Consider halting or entering degraded mode if PSRAM is required
        // while(1) { delay(1000); }
    }

    // Initialize TFT display
    tft.init();
    tft.setRotation(1);  // Landscape mode (480×320)

    // Apply touch calibration (XPT2046).
    // Order of preference:
    //   1. Persistent calibration from NVS (touch_calibration namespace)
    //      — set by the wizard on first boot or recalibration.
    //   2. Compile-time TOUCH_CALIBRATION from User_Setup.h — used as the
    //      safe fallback so the display ALWAYS boots with a working touch
    //      mapping (the wizard itself relies on this fallback while
    //      capturing new corners on the very first boot).
    {
        uint16_t calData[5] = TOUCH_CALIBRATION;
        if (touch_calibration::loadValid(calData)) {
            Serial.println("[BOOT][INFO] Touch calibration: NVS (persistent)");
        } else {
            Serial.println("[BOOT][INFO] Touch calibration: User_Setup.h fallback");
        }
        tft.setTouch(calData);
    }

    tft.fillScreen(0x2104);  // Dark gray background
    tft.setTextColor(0xFFFF, 0x2104);
    tft.setTextSize(1);
    Serial.println("[BOOT][INFO] Display initialized (480x320 landscape)");

    /* ---- TWAI initialization with CiA 301 optimal timing ----
     *
     * The ESP32-TWAI-CAN library default 500 kbps timing uses a sample
     * point of 80.0 % (BRP=8, TSEG1=15, TSEG2=4).  The STM32 FDCAN is
     * configured at 88.2 %.  The 8.2 % difference is right at the limit
     * of what SJW can compensate — combined with the STM32's HSI ±1 %
     * clock tolerance, this causes persistent BERR / ACK errors.
     *
     * Fix: use custom timing with 87.5 % sample point (CiA 301
     * recommended for 500 kbps), closely matching the STM32's 88.2 %.
     *
     * APB clock  = 80 MHz
     * BRP        = 10    → TQ = 10 / 80 MHz = 125 ns
     * Bit time   = 1 (sync) + 13 (seg1) + 2 (seg2) = 16 TQ = 2 µs
     * Baud rate  = 80 MHz / 10 / 16 = 500 kbps
     * Sample pt  = (1 + 13) / 16 = 87.5 %
     * SJW        = 2  → ±12.5 % oscillator tolerance                  */
    if (twaiInit()) {
        Serial.printf("[CAN][INFO] Initialized at 500 kbps (SP=87.5%%, BRP=10, "
                      "TSEG1=13, TSEG2=2, SJW=2)\n");
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        Serial.printf("[CAN][INFO] ESP-IDF %d.%d.%d — quanta_resolution_hz cleared "
                      "to force BRP mode\n",
                      ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR,
                      ESP_IDF_VERSION_PATCH);
#endif
    } else {
        Serial.println("[CAN][ERR] Initialization FAILED");
    }

    // Initialize obstacle sensor driver (TF-Mini Plus on UART1, GPIO 18)
    //   RX = GPIO 18 (sensor TX → ESP32 RX, 3.3V TTL direct connection)
    //   TX = NOT CONNECTED (sensor RX not wired to ESP32 TX)
    //
    // READ-ONLY MODE: The sensor's RX pin is not connected to the ESP32's
    // TX, so no configuration commands can reach the sensor.
    // Config defaults are selected at compile time based on SENSOR_TYPE.
    {
        obstacle_sensor::Config obsCfg;
        obsCfg.txPin = -1;   // TX not connected — read-only
        obstacle_sensor::init(obsCfg);
    }

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

    // Create render task on Core 0 — offloads all TFT and touch operations
    // from the main loop, preventing SPI blocking on Core 1.
    vdMutex = xSemaphoreCreateMutex();
    touchActionQueue = xQueueCreate(8, sizeof(TouchAction));
    if (vdMutex == nullptr || touchActionQueue == nullptr) {
        Serial.println("[BOOT][ERR] Failed to create render task resources");
        // Fall through — render task won't be created, TFT won't update.
        // System is still safe (CAN, safety, LEDs continue on Core 1).
    } else {
        BaseType_t rc = xTaskCreatePinnedToCore(
            renderTask, "Render", 16384, nullptr, 1, nullptr, 0);
        if (rc != pdPASS) {
            Serial.println("[BOOT][ERR] Failed to create render task");
        } else {
            Serial.println("[BOOT][INFO] Render task started on Core 0");
        }
    }
}

// ---------------------------------------------------------------------------
// loop() — called repeatedly
// ---------------------------------------------------------------------------
void loop() {
    RTMON_LOOP_BEGIN();
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
    // §1 CAN state validation: this is the SOLE source of truth for UI mode
    // state.  Tank turn and traction toggles only send CAN commands — the UI
    // updates here after the STM32 confirms via heartbeat echo.
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
                // Persist confirmed tank turn state to NVS
                config_store::setDriveMode(currentModeFlags & can::MODE_FLAG_TANK_TURN);
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
                    Serial.println("[SAFETY][WARN] STM32 heartbeat counter frozen — inhibiting commands");
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
                stm32StartupSeen    = true;
                gearResyncPending   = true;
                tempMapResyncPending = true;
                Serial.println("[SAFETY][INFO] STM32 restart detected — gear + temp-map resync pending");
            }
            if (gearResyncPending && !startupInhibitNow &&
                hb.systemState == can::SystemState::ACTIVE && stm32IsAlive) {
                // STM32 is now in ACTIVE with no startup inhibit — send current gear
                uint8_t curGear = shifter::getGearRaw();
                sendGearCommand(curGear);
                lastSentGear    = curGear;  // prevent duplicate send from shifter loop
                gearResyncPending = false;
                Serial.printf("[SAFETY][INFO] Gear resync sent: gear=%u\n", curGear);
            }
            // Re-send the DS18B20 physIdx→role mapping after STM32 restart so
            // the STM32's in-RAM map matches the user's saved assignment.
            // Sent in STANDBY or ACTIVE (STM32 can receive it in either state).
            if (tempMapResyncPending && !startupInhibitNow && stm32IsAlive) {
                const auto& cfg = config_store::get();
                CanFrame frame = {};
                frame.identifier       = can::CMD_SENSOR_MAP_TEMP;
                frame.extd             = 0;
                frame.data_length_code = config_store::NUM_TEMP_SENS;
                for (uint8_t i = 0; i < config_store::NUM_TEMP_SENS; ++i) {
                    frame.data[i] = cfg.tempSensorMap[i];
                }
                ESP32Can.writeFrame(frame, 0);
                tempMapResyncPending = false;
                Serial.println("[SAFETY][INFO] DS18B20 sensor map resync sent to STM32");
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

        // Update VehicleData with latest sensor reading for HMI display.
        // distanceCm = 0 maps to "---" in the UI, signalling no valid reading.
        // Use 0 when the sensor is unhealthy (fault/stuck/timeout) so the
        // display shows "---" instead of a misleading "0.02 m" fallback value.
        obstacle_sensor::Reading rd = obstacle_sensor::getReading();
        vehicle::ObstacleData od;
        od.distanceCm  = rd.healthy ? (rd.distance_mm / 10) : 0;
        od.timestampMs  = millis();
        vehicleData.setObstacle(od);
    }

    // ---- Update shared vehicle data for render task (Core 0) ----
    // Non-blocking: skip if render task holds the mutex (stale-by-one-frame is OK).
    if (vdMutex != nullptr && xSemaphoreTake(vdMutex, 0) == pdTRUE) {
        renderVD = vehicleData;
        xSemaphoreGive(vdMutex);
    }

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
            // NVS persist deferred to heartbeat sync (§1) — only confirmed
            // state is persisted to avoid stale data on power loss.
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

    // ---- Process touch actions from render task (Core 0) ----
    // Touch reading and screen dispatch happen on Core 0 (same SPI bus as TFT).
    // Actions that require CAN sends or state changes are queued here.
    if (touchActionQueue != nullptr) {
        TouchAction act;
        while (xQueueReceive(touchActionQueue, &act, 0) == pdTRUE) {
            switch (act) {
                case TouchAction::FRONT_LED_TOGGLE:
                    frontLedLocalState = !frontLedLocalState;
                    sendLedCommand(frontLedLocalState, rearLedLocalState);
                    config_store::setFrontLedEnabled(frontLedLocalState);
                    audio::play(frontLedLocalState ? audio::Sound::LIGHTS_ON
                                                   : audio::Sound::LIGHTS_OFF,
                                audio::Priority::LO);
                    Serial.printf("[LED] Front → %s\n",
                                  frontLedLocalState ? "ON" : "OFF");
                    break;

                case TouchAction::REAR_LED_TOGGLE:
                    rearLedLocalState = !rearLedLocalState;
                    sendLedCommand(frontLedLocalState, rearLedLocalState);
                    config_store::setRearLedEnabled(rearLedLocalState);
                    audio::play(rearLedLocalState ? audio::Sound::LIGHTS_ON
                                                  : audio::Sound::LIGHTS_OFF,
                                audio::Priority::LO);
                    Serial.printf("[LED] Rear → %s\n",
                                  rearLedLocalState ? "ON" : "OFF");
                    break;

                case TouchAction::TANK_MODE_TOGGLE:
                    if (stm32IsAlive) {
                        currentModeFlags ^= can::MODE_FLAG_TANK_TURN;
                        audio::play(audio::Sound::BEEP, audio::Priority::LO);
                        sendModeCommand(currentModeFlags);
                        // §1 CAN state validation: do NOT update vehicleData here.
                        // The heartbeat sync (lines 693-700) updates UI state
                        // ONLY after the STM32 confirms via statusFlags echo.
                        // NVS persistence also deferred to heartbeat confirmation.
                        Serial.printf("[MODE] Tank toggle requested → 0x%02X\n",
                                      currentModeFlags);
                    }
                    break;
            }
        }
    }

    // ---- Power management ----
    power_mgr::update();

    // Welcome audio on startup — with brownout/reboot cooldown
    if (power_mgr::isRunning() && !welcomePlayed) {
        if (lastWelcomePlayMs == 0 ||
            (now - lastWelcomePlayMs) >= WELCOME_COOLDOWN_MS) {
            audio::play(audio::Sound::WELCOME, audio::Priority::HI);
            lastWelcomePlayMs = now;
        }
        welcomePlayed  = true;
        farewellPlayed = false;
    }

    // Farewell audio on shutdown
    if (power_mgr::getState() == power_mgr::PowerState::SHUTTING_DOWN &&
        !farewellPlayed) {
        // Pre-power-cut safe-state handshake: tell the STM32 to enter a
        // commanded safe state (PWM=0, EN=LOW, relays OFF) BEFORE the
        // external delay relay physically removes power.  The 100 ms
        // pause gives the STM32 a deterministic window to react before
        // we continue with the existing shutdown sequence.  This does
        // NOT alter SHUTDOWN_DELAY_MS, GPIO 41, or the LED-OFF below;
        // if the frame is lost on the bus, behaviour is unchanged.
        sendSystemShutdown();
        delay(100);

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
                    case can::SafetyError::BATTERY_OV_WARN:
                        // No dedicated overvoltage sound exists; BATTERY_LOW is the
                        // closest available warning tone.  A future audio track can be
                        // added and this mapping updated without any logic change.
                        playWarning(audio::Sound::BATTERY_LOW, audio::Priority::MEDIUM);
                        break;
                    case can::SafetyError::BATTERY_OV_CRIT:
                        playWarning(audio::Sound::BATTERY_CRITICAL, audio::Priority::HI);
                        break;
                    case can::SafetyError::RELAY_OPEN:
                        playWarning(audio::Sound::ERROR_GENERAL, audio::Priority::HI);
                        break;
                    default:
                        break;
                }

                // ---- DTC Fault Log: persist error event to NVS ----
                {
                    config_store::FaultLogEntry fte;
                    fte.uptimeMs    = now;
                    fte.errorCode   = safeErr;
                    fte.faultFlags  = vehicleData.heartbeat().faultFlags;
                    fte.subsystem   = vehicleData.diag().subsystem;
                    fte.systemState = static_cast<uint8_t>(vehicleData.heartbeat().systemState);
                    config_store::logFault(fte);
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

    // ---- Runtime counter for maintenance tracking (§2.1, §2.2) ----
    // Accumulate milliseconds in RAM continuously while operating.
    // Commit full seconds to NVS periodically (every ~60 s) to reduce flash wear.
    // Delta-based: uses real elapsed time, not frame count.
    {
        auto sysState = vehicleData.heartbeat().systemState;
        bool isOperating = (sysState == can::SystemState::ACTIVE ||
                            sysState == can::SystemState::DEGRADED ||
                            sysState == can::SystemState::LIMP_HOME);
        if (isOperating) {
            // Accumulate real elapsed ms since last tick (§2.1 — delta-based)
            // Unsigned subtraction handles millis() wrap correctly (~49.7 days).
            // Clamp to a reasonable max to guard against stale lastRuntimeTickMs.
            uint32_t elapsed = static_cast<uint32_t>(now - lastRuntimeTickMs);
            if (elapsed > 10000) elapsed = 10000;  // sanity: cap at 10 s per tick
            runtimeAccumMs += elapsed;

            // Commit accumulated whole seconds to NVS periodically (§2.2)
            if (runtimeAccumMs >= RUNTIME_COMMIT_INTERVAL_MS) {
                uint32_t wholeSec = runtimeAccumMs / 1000;
                runtimeAccumMs -= wholeSec * 1000;  // keep fractional ms
                uint32_t cur = config_store::get().runtimeSeconds;
                config_store::setRuntimeSeconds(cur + wholeSec);
            }
        } else {
            // Not operating: commit any accumulated partial seconds before discarding
            if (runtimeAccumMs >= 1000) {
                uint32_t wholeSec = runtimeAccumMs / 1000;
                uint32_t cur = config_store::get().runtimeSeconds;
                config_store::setRuntimeSeconds(cur + wholeSec);
            }
            runtimeAccumMs = 0;
        }
        lastRuntimeTickMs = now;

        // Play maintenance reminder once per power cycle when threshold crossed (§2.3)
        if (config_store::isMaintenanceDue() && !maintenanceWarnPlayed &&
            !config_store::get().maintAcknowledged) {
            audio::play(audio::Sound::TEST_SYSTEM, audio::Priority::LO);
            maintenanceWarnPlayed = true;
            Serial.println("[MAINT] Maintenance reminder — service due");
        }
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

        // ---- One-shot emergency flash on ERROR state transition ----
        // When the system enters ERROR (not SAFE — SAFE is a controlled
        // shutdown), fire 3 aggressive red/black flash cycles across both
        // strips to grab the driver's attention.  After the flash completes,
        // the sustained BRAKE_EMERGENCY + HAZARD indication takes over.
        {
            static can::SystemState prevLedState = can::SystemState::BOOT;
            if (st == can::SystemState::ERROR && prevLedState != can::SystemState::ERROR) {
                led_ctrl::startEmergencyFlash(3);
            }
            prevLedState = st;
        }

        // Reverse detection from physical shifter
        bool reverse = (shifter::getGearRaw() == GEAR_REVERSE);

        // Braking & throttle detection: traction average near zero while
        // speed > 0 indicates dynamic braking or throttle release at speed.
        // Regen zone: low throttle (5–20%) at speed — lighter deceleration
        // than full braking, triggers blue-pulse regen indicator on rear.
        bool braking = false;
        bool regen   = false;
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
            regen   = (spSum > LED_SPEED_SUM_THRESHOLD
                       && trAvg > LED_TRACTION_BRAKING_THRESHOLD
                       && trAvg <= LED_TRACTION_REGEN_THRESHOLD);
        }

        // ---- Front LED mode from vehicle state ----
        if (st == can::SystemState::SAFE || st == can::SystemState::ERROR) {
            // Safe Mode: KITT scanner on centre, HAZARD indicators on 4 corners
            led_ctrl::setFrontMode(led_ctrl::FrontMode::KITT_IDLE);
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
        } else if (regen) {
            led_ctrl::setRearMode(led_ctrl::RearMode::REGEN_ACTIVE);
        } else {
            led_ctrl::setRearMode(led_ctrl::RearMode::POSITION);
        }

        // ---- Turn-signal derivation from steering angle (0.1° units) ----
        //
        // ⚠ TURN SIGNAL TIMING & DETERMINISM CONTRACT:
        //
        // This logic is fully deterministic with the following guarantees:
        //   - Runs once per render loop iteration (20 Hz cadence)
        //   - Uses 'now' (millis()) captured once per frame at the top of
        //     the render loop — identical across all consumers
        //   - State transitions: prevTurn, candidateTurn, candidateStartMs
        //     are static locals — no dynamic allocation, no race conditions
        //   - Single-threaded: runs only on Core 0 render task
        //
        // Anti-jitter stack (three independent layers):
        //   1. Hysteresis: 5° dead band (±10° off → ±15° on)
        //      Prevents chatter when steering oscillates near threshold
        //   2. Time filter: new state must persist ≥100 ms (2 frames at 20 Hz)
        //      Prevents micro-oscillation from PWM-coupled steering noise
        //   3. HAZARD override: SAFE/ERROR → immediate hazard (no filter)
        //      Safety-critical indication bypasses all debounce logic
        //
        // Edge oscillation at low speed:
        //   At low speed, steering angle can jitter ±2-5° due to encoder
        //   resolution and motor vibration.  The 5° hysteresis band
        //   (10°→15°) means the signal won't toggle unless the angle
        //   swings by 5°.  The 100 ms persistence filter adds a second
        //   layer: even if the angle briefly crosses 15° and returns,
        //   the turn signal won't activate unless it stays above 15°
        //   for at least 100 ms.
        //
        // Activate at 15° to signal the beginning of a turn.
        // Deactivate at 10° (5° hysteresis) to avoid flickering when the
        // wheel hovers around the threshold.
        // Time filter: a new turn state must persist for ≥100 ms before
        // being applied, preventing rapid left↔right flicker from small
        // steering oscillations at low speed.
        // SAFE/ERROR states override with hazard flash (immediate).
        {
            static constexpr int16_t TURN_ON_RAW  = 150;  // 15.0° — activate
            static constexpr int16_t TURN_OFF_RAW = 100;  // 10.0° — deactivate (hysteresis)
            static constexpr unsigned long TURN_PERSIST_MS = 100;  // anti-jitter time filter

            int16_t angle = vehicleData.steering().angleRaw;
            static led_ctrl::TurnSignal prevTurn = led_ctrl::TurnSignal::OFF;
            static led_ctrl::TurnSignal candidateTurn = led_ctrl::TurnSignal::OFF;
            static unsigned long candidateStartMs = 0;

            if (st == can::SystemState::SAFE || st == can::SystemState::ERROR) {
                // Hazard override — immediate, no time filter
                led_ctrl::setTurnSignal(led_ctrl::TurnSignal::HAZARD);
                // Don't persist HAZARD into prevTurn — reset to OFF so that
                // when the system recovers, the hysteresis starts clean.
                prevTurn = led_ctrl::TurnSignal::OFF;
                candidateTurn = led_ctrl::TurnSignal::OFF;
                candidateStartMs = 0;
            } else {
                // Compute desired turn from angle + hysteresis
                led_ctrl::TurnSignal desired = prevTurn;
                if (angle < -TURN_ON_RAW) {
                    desired = led_ctrl::TurnSignal::LEFT;
                } else if (angle > TURN_ON_RAW) {
                    desired = led_ctrl::TurnSignal::RIGHT;
                } else if (angle > -TURN_OFF_RAW && angle < TURN_OFF_RAW) {
                    desired = led_ctrl::TurnSignal::OFF;
                }
                // Between OFF_RAW and ON_RAW: keep previous state (hysteresis band)

                // Time filter: require new state to persist ≥100 ms
                if (desired != prevTurn) {
                    if (desired != candidateTurn) {
                        candidateTurn = desired;
                        candidateStartMs = now;
                    } else if ((now - candidateStartMs) >= TURN_PERSIST_MS) {
                        prevTurn = desired;
                        candidateStartMs = 0;
                    }
                } else {
                    // Stable — reset candidate
                    candidateTurn = prevTurn;
                    candidateStartMs = 0;
                }

                led_ctrl::setTurnSignal(prevTurn);
            }
        }

        // Single update call drives all animation + FastLED.show()
        led_ctrl::update();
    }

    // Send heartbeat 0x011 every 100 ms
    if (now - lastHeartbeatMs >= can::HEARTBEAT_INTERVAL_MS) {
        lastHeartbeatMs = now;

        CanFrame frame = {};
        frame.identifier       = can::HEARTBEAT_ESP32;
        frame.extd             = 0;
        frame.data_length_code = 1;
        frame.data[0]          = heartbeatCounter++;

        ESP32Can.writeFrame(frame, 0);  // Non-blocking: drop if TX queue full
    }

    // Serial heartbeat every ~1 second
    if (now - lastSerialMs >= 1000) {
        lastSerialMs = now;
        Serial.println("[HMI] heartbeat");
    }

    // TWAI bus diagnostic — log error counters every 5 seconds
    if (now - lastCanDiagMs >= 5000) {
        lastCanDiagMs = now;
        twai_status_info_t status;
        if (twai_get_status_info(&status) == ESP_OK) {
            const char* stateStr = "UNKNOWN";
            switch (status.state) {
                case TWAI_STATE_STOPPED:    stateStr = "STOPPED";    break;
                case TWAI_STATE_RUNNING:    stateStr = "RUNNING";    break;
                case TWAI_STATE_BUS_OFF:    stateStr = "BUS_OFF";    break;
                case TWAI_STATE_RECOVERING: stateStr = "RECOVERING"; break;
            }
            Serial.printf("[CAN][DIAG] state=%s tx_err=%lu rx_err=%lu "
                          "tx_fail=%lu rx_miss=%lu arb_lost=%lu bus_err=%lu\n",
                          stateStr,
                          (unsigned long)status.tx_error_counter,
                          (unsigned long)status.rx_error_counter,
                          (unsigned long)status.tx_failed_count,
                          (unsigned long)status.rx_missed_count,
                          (unsigned long)status.arb_lost_count,
                          (unsigned long)status.bus_error_count);
            if (status.tx_error_counter >= 128 &&
                status.state == TWAI_STATE_RUNNING) {
                Serial.println("[CAN][WARN] Error-passive "
                               "(tx_err>=128). Check CAN bus wiring "
                               "and termination.");
            }
        }
    }

    /* ---- TWAI bus-off / error-passive recovery ----
     * If the ESP32 transmits before the STM32 starts (or after a bus
     * glitch), the TWAI controller enters BUS_OFF after TEC reaches 256.
     * Without recovery the ESP32 stays dead permanently.
     *
     * Recovery sequence (per ESP-IDF TWAI API):
     *   BUS_OFF → twai_initiate_recovery() → RECOVERING
     *   RECOVERING → (128×11 recessive bits) → STOPPED
     *   STOPPED → twai_start() → RUNNING
     *
     * Error-passive recovery (two-phase):
     *   When TEC saturates at 128 (error-passive) the TWAI state remains
     *   RUNNING so bus-off recovery never triggers.  If error-passive
     *   persists for the timeout period, a full driver reinit
     *   (stop → uninstall → install → start) clears error counters.
     *   Phase 1: first 10 attempts at 3 s intervals (fast).
     *   Phase 2: unlimited attempts at 30 s intervals (slow periodic).
     *
     * Check every 250 ms; bus-off limited to 10 attempts. */
    if (now - lastBusOffCheckMs >= 250) {
        lastBusOffCheckMs = now;
        twai_status_info_t sts;
        if (twai_get_status_info(&sts) == ESP_OK) {
            if (sts.state == TWAI_STATE_BUS_OFF) {
                if (busOffRecoveryCount < 10) {
                    Serial.printf("[CAN][ERR] BUS_OFF detected — recovery attempt %u/10\n",
                                  busOffRecoveryCount + 1);
                    twai_initiate_recovery();
                    busOffRecoveryCount++;
                }
                errorPassiveSince = 0;  // not error-passive while bus-off
            } else if (sts.state == TWAI_STATE_STOPPED) {
                /* Recovery completed — restart the driver */
                if (twai_start() == ESP_OK) {
                    Serial.println("[CAN][INFO] Recovery complete — TWAI restarted");
                    busOffRecoveryCount = 0;
                }
                errorPassiveSince = 0;
            } else if (sts.state == TWAI_STATE_RUNNING) {
                busOffRecoveryCount = 0;

                /* ---- Error-passive detection (two-phase) ----
                 * TEC ≥ 128 means the controller is error-passive.  It can
                 * still receive but transmissions are degraded.  If this
                 * persists, a full driver reinit is the only way to clear
                 * the error counters (twai_initiate_recovery() is only
                 * valid from BUS_OFF state).
                 *
                 * Phase 1 (fast):  first ERROR_PASSIVE_MAX_FAST attempts
                 *                  at ERROR_PASSIVE_TIMEOUT_MS intervals.
                 * Phase 2 (slow):  unlimited attempts at
                 *                  ERROR_PASSIVE_SLOW_TIMEOUT_MS intervals. */
                if (sts.tx_error_counter >= 128) {
                    bool slowPhase = (errorPassiveResets >= ERROR_PASSIVE_MAX_FAST);
                    uint32_t timeout = slowPhase
                                     ? ERROR_PASSIVE_SLOW_TIMEOUT_MS
                                     : ERROR_PASSIVE_TIMEOUT_MS;

                    if (errorPassiveSince == 0) {
                        errorPassiveSince = now;
                    } else if ((now - errorPassiveSince) >= timeout) {
                        if (slowPhase) {
                            Serial.printf("[CAN][WARN] Error-passive (tx_err=%lu) "
                                          "— slow-periodic reinit (%lu s cycle)\n",
                                          (unsigned long)sts.tx_error_counter,
                                          (unsigned long)(ERROR_PASSIVE_SLOW_TIMEOUT_MS / 1000));
                        } else {
                            Serial.printf("[CAN][WARN] Error-passive (tx_err=%lu) for >3s "
                                          "— reinit attempt %u/%u\n",
                                          (unsigned long)sts.tx_error_counter,
                                          errorPassiveResets + 1,
                                          ERROR_PASSIVE_MAX_FAST);
                        }
                        esp_err_t stop_res = twai_stop();
                        if (stop_res != ESP_OK &&
                            stop_res != ESP_ERR_INVALID_STATE) {
                            Serial.printf("[CAN][ERR] twai_stop() failed: 0x%x\n",
                                          (unsigned)stop_res);
                        } else {
                            esp_err_t uninst_res = twai_driver_uninstall();
                            if (uninst_res != ESP_OK &&
                                uninst_res != ESP_ERR_INVALID_STATE) {
                                Serial.printf("[CAN][ERR] twai_driver_uninstall() "
                                              "failed: 0x%x\n",
                                              (unsigned)uninst_res);
                            } else if (twaiInit()) {
                                Serial.println("[CAN][INFO] Error-passive recovery "
                                               "complete — TWAI reinitialized");
                            } else {
                                Serial.println("[CAN][ERR] Error-passive recovery "
                                               "FAILED — TWAI reinit error");
                            }
                        }
                        errorPassiveResets++;
                        errorPassiveSince = 0;
                    }
                } else {
                    /* TEC below error-passive threshold — healthy */
                    errorPassiveSince  = 0;
                    errorPassiveResets = 0;
                }
            }
        }
    }

#if RUNTIME_MONITOR
    // Runtime monitor serial log every LOG_INTERVAL_MS
    if (now - lastRtMonMs >= rtmon::LOG_INTERVAL_MS) {
        lastRtMonMs = now;
        RTMON_LOG();
    }
#endif
    RTMON_LOOP_END();

    vTaskDelay(1);  // Yield Core 1 — let TWAI driver, idle task, and system housekeeping run
}
