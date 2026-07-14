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
#include <Preferences.h>
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
#include "remote_control.h"
#include "touch_handler.h"
#include "touch_calibration.h"
#include "config_store.h"
#include "traction_switch.h"
#include "mode_sync.h"
#include "display_backlight.h"
#include "stm32_liveness.h"
#include "twai_recovery.h"
#include "boot_diag.h"
#include "display_supervisor.h"
#include "display_recovery.h"
#include "ack_tracker.h"

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

// CAN transceiver pins (TJA1051T/3, 3.3V logic with VIO=3.3V)
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
// TWAI BUS_OFF recovery — stable-heartbeat policy (pure, host-tested module).
// Recovery is confirmed ONLY after sustained advancing STM32 heartbeats; the
// controller merely reporting RUNNING is treated as probation, never as
// healthy.  Preserves the lifetime BUS_OFF count and retries fast-then-slow so
// the node always recovers after a physical reconnection.
static twai_recovery::TwaiBusOffRecovery g_twaiRecovery;

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
// One-shot NVS relay restore after boot (retries until the STM32 echoes the
// desired relay state, so a dropped CMD_LED frame can't leave the strips off).
static bool          ledBootRestoreDone     = false;
static uint8_t       ledBootRestoreAttempts = 0;
static unsigned long ledBootRestoreLastMs   = 0;
static constexpr uint8_t       LED_BOOT_RESTORE_MAX_ATTEMPTS = 10;
static constexpr unsigned long LED_BOOT_RESTORE_RETRY_MS     = 500;

// ---- Shifter gear tracking ----
static uint8_t  lastSentGear      = 0xFF;    // last gear value sent to STM32
static unsigned long lastGearSendMs = 0;     // debounce for gear CAN sends
static constexpr unsigned long GEAR_SEND_DEBOUNCE_MS = 100;
static constexpr uint8_t GEAR_REVERSE = 1;   // shifter raw value for Reverse

// ---- Mode state tracking ----
static uint8_t  currentModeFlags  = 0;       // Current mode flags (bit 0=4x4, bit 1=tank)

// ---- Drive-mode sync FSM (physical 4x4 selector = source of truth) ----
// The debounced selector latches its state into g_modeSync.setDesired(); the
// FSM defers the actual CMD_MODE transmission until the STM32 heartbeat is
// confirmed and (re)transmits until an ACK arrives or a bounded retry budget
// is exhausted.  Logic lives in the pure, host-tested mode_sync.h.
static constexpr uint8_t MODE_SYNC_MAX_RETRIES = 3;
static ModeSync      g_modeSync(can::ACK_TIMEOUT_MS, MODE_SYNC_MAX_RETRIES);
static unsigned long g_modeSendMs   = 0;     // millis() of last FSM-driven send
static unsigned long g_lastModeAckTs = 0;    // last CMD_MODE ACK consumed by FSM
static unsigned long g_lastModeEchoTs = 0;   // last heartbeat frame fed as mode echo

// ---- Power/Audio state tracking ----
static bool     welcomePlayed     = false;
static bool     farewellPlayed    = false;
static uint8_t  lastAppliedAudioVolume = 15;
static unsigned long lastRemoteVolumeSetMs = 0;
static constexpr unsigned long REMOTE_VOLUME_DEBOUNCE_MS = 250;
static constexpr uint8_t       REMOTE_VOLUME_HYSTERESIS  = 2;
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

// ---- Render heartbeat (audit P2) ----
// Monotonic millis() timestamp published by renderTask at the END of every
// fully completed frame, i.e. AFTER screenManager.update(), all TFT drawing,
// the centralized touch read, and the implicit release of the shared SPI bus.
// Core-1 (loop) reads this to feed the DisplaySupervisor; the heartbeat only
// advances when a frame truly finished, so a hung render/SPI transaction is
// observable as a stalled timestamp.  volatile: written on Core 0, read on
// Core 1 (a single aligned 32-bit word — atomic on the ESP32-S3).
static volatile uint32_t renderHeartbeatMs = 0;
static volatile uint32_t renderFrameCounter = 0;

// ---- Core-1 → Core-0 TFT recovery request (audit P2 §4) ----
// The white-screen fault must be recovered with EXCLUSIVE bus ownership from
// the Core-0 render task; Core 1 must NEVER call TFT_eSPI.  Core 1 (loop /
// supervisor) only posts a request into this mailbox, and Core 0 executes the
// hardware choreography (display_recovery.h) and reports the result back.  The
// pure handshake logic lives in display::RecoveryRequestMailbox (host-tested in
// test_display_recovery.cpp); the portMUX below makes the two-core access safe.
static display::RecoveryRequestMailbox g_recoveryMailbox;
static portMUX_TYPE g_recoveryMux = portMUX_INITIALIZER_UNLOCKED;

// Bench/manual "the panel went white" request (audit §4.2).  Because the white
// screen is NOT automatically observable with the current wiring (no reliable
// readback, no reset-signal latch), an operator/bench path can explicitly
// request a recovery.  Set from Core 1 (serial command / future HMI button).
static volatile bool g_manualRecoveryRequest = false;

// Optional latched TFT reset-signal evidence (audit §4.3).  Remains false
// unless a real hardware latch on TFT_RST/GPIO38 is wired and sets it; the
// firmware never fabricates this signal, so white is never asserted from it
// unless the hardware truly observed a reset.
static volatile bool g_tftResetSignalLatched = false;

// Core-1 entry point to request a recovery.  Thread-safe; posts into the
// mailbox only if none is pending / in flight.  Returns true if accepted.  The
// current time is latched so Core 1 can later detect a render task that never
// consumes the request (audit P2.1).
static bool requestDisplayRecovery(display::RecoveryTrigger trigger) {
    bool accepted;
    taskENTER_CRITICAL(&g_recoveryMux);
    accepted = g_recoveryMailbox.request(trigger, (uint32_t)millis());
    taskEXIT_CRITICAL(&g_recoveryMux);
    return accepted;
}

// ---- Core-1 → Core-0 post-recovery banner text mailbox (audit P2.4) ----
// Core 1 assembles the banner (with REAL supervisor counters) and posts the
// formatted text here; Core 0 (render task) shows it as a ScreenManager overlay
// so a subsequent full redraw cannot erase it.
static portMUX_TYPE g_bannerMux = portMUX_INITIALIZER_UNLOCKED;
static char          g_bannerText[320] = {0};
static volatile bool g_bannerReady = false;

static void postRecoveryBanner(const char* text) {
    taskENTER_CRITICAL(&g_bannerMux);
    strncpy(g_bannerText, text, sizeof(g_bannerText) - 1);
    g_bannerText[sizeof(g_bannerText) - 1] = '\0';
    g_bannerReady = true;
    taskEXIT_CRITICAL(&g_bannerMux);
}

static bool takeRecoveryBanner(char* out, size_t n) {
    bool have = false;
    taskENTER_CRITICAL(&g_bannerMux);
    if (g_bannerReady) {
        strncpy(out, g_bannerText, n - 1);
        out[n - 1] = '\0';
        g_bannerReady = false;
        have = true;
    }
    taskEXIT_CRITICAL(&g_bannerMux);
    return have;
}

// Audit P2 final — persisted render-blocked reboot surfaced ON THE HMI once.
// setup() (Core 1) reads the persisted cause and, if a reboot is still pending,
// latches it here BEFORE the render task exists.  The render task (Core 0) then
// publishes it as a ScreenManager overlay on its first frames and clears ONLY
// rb_pending, keeping rb_count as the historical counter.  Written once by
// Core 1 before task creation (happens-before), then owned by Core 0.
static volatile bool             s_rebootBannerPending = false;
static display::RebootRecoveryInfo s_rebootBannerInfo   = {};

// Clear ONLY the rb_pending flag after the banner has been published, keeping
// rb_cause / rb_uptime / rb_count intact as the historical record so later
// normal boots do NOT repeat the banner.
static void clearRenderBlockedPending(void) {
    Preferences p;
    if (p.begin("disp_fault", /*readOnly=*/false)) {
        p.putBool("rb_pending", false);
        p.end();
    }
}

// Audit P2.1 — last resort when the render task is genuinely blocked and never
// consumes/finishes the recovery request.  Core 1 persists the cause to NVS and
// performs a controlled ESP32 restart.  Core 1 NEVER touches the TFT/SPI here.
static void persistRenderBlockedAndReboot(display::RecoveryTrigger trigger) {
    Preferences p;
    if (p.begin("disp_fault", /*readOnly=*/false)) {
        p.putUChar("rb_cause",  (uint8_t)trigger);
        p.putUInt ("rb_uptime", (uint32_t)millis());
        p.putUInt ("rb_count",  p.getUInt("rb_count", 0) + 1U);
        // audit P2 final — mark the fault pending so the NEXT boot surfaces it
        // on the HMI (not only on the serial log) exactly once.
        p.putBool ("rb_pending", true);
        p.end();
    }
    Serial.printf("[DISPLAY][RECOVERY] RENDER BLOCKED — persisted cause '%s'; "
                  "controlled ESP32 restart\n",
                  display::recoveryTriggerText(trigger));
    Serial.flush();
    delay(50);
    esp_restart();
}

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
static constexpr uint8_t  STM32_HB_FREEZE_COUNT = 5;   // 5 × HEARTBEAT_INTERVAL_MS (see STM32_HB_FREEZE_TIME_MS below)
// Tie the freeze-detection window to the heartbeat period at compile time so
// that changing one constant cannot silently desynchronise the other.
static constexpr uint32_t STM32_HB_FREEZE_TIME_MS =
    (uint32_t)STM32_HB_FREEZE_COUNT * can::HEARTBEAT_INTERVAL_MS;
static_assert(STM32_HB_FREEZE_TIME_MS < can::CAN_LOSS_TIMEOUT_MS,
              "STM32 freeze detection must trigger before CAN-loss timeout");
// Absence timeout: if no NEW heartbeat frame arrives within this window the
// STM32 is treated as not-alive independently of the counter value.  Kept
// below CAN_LOSS_TIMEOUT_MS so the dedicated CAN-loss error screen still wins.
static constexpr uint32_t STM32_HB_ABSENCE_TIMEOUT_MS = STM32_HB_FREEZE_TIME_MS;
static Stm32Liveness stm32Liveness(STM32_HB_FREEZE_COUNT, STM32_HB_ABSENCE_TIMEOUT_MS);
// Rate-limit the "frozen" warning so a stuck STM32 cannot spam Serial.
static unsigned long stm32FrozenWarnLastMs = 0;
static constexpr unsigned long STM32_FROZEN_WARN_INTERVAL_MS = 1000;
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
// R-3 hardening: capture the render task handle so loop() can query
// its stack high-watermark via uxTaskGetStackHighWaterMark().  Was
// previously discarded (nullptr passed as 6th arg of xTaskCreate*).
static TaskHandle_t renderTaskHandle = nullptr;

enum class TouchAction : uint8_t {
    FRONT_LED_TOGGLE,
    REAR_LED_TOGGLE,
    TANK_MODE_TOGGLE,
};

// ---- Command ACK tracking (Phase 13, audit §Entrega 3-D) ----
// Non-blocking, bounded, INDEPENDENT per-command tracking: several distinct
// commands (CMD_MODE, CMD_LED, SERVICE_CMD, CMD_SENSOR_MAP_TEMP) can be in
// flight at once without one overwriting another.  No automatic retry — each
// entry has an independent bounded timeout only, so there are no infinite
// loops.  See ack_tracker.h / test_ack_tracker.cpp.

static ack::Tracker<8> ackTracker(can::ACK_TIMEOUT_MS);

/// Call before sending a command that expects ACK (CMD_MODE, CMD_LED,
/// SERVICE_CMD, CMD_SENSOR_MAP_TEMP).  Registers an independent pending entry.
__attribute__((unused))
static void ackBeginWait(uint8_t cmdIdLow) {
    if (!ackTracker.begin(cmdIdLow, millis())) {
        // Table full — should never happen with only a handful of command
        // classes, but surface it instead of silently dropping the wait.
        Serial.printf("[ACK] table full, cannot track cmd 0x%02X\n", cmdIdLow);
    }
}

/// Call from loop() after can_rx::poll() to consume newly-arrived ACKs and
/// expire any independently timed-out commands.
static void ackCheck(vehicle::VehicleData& data) {
    // Drain EVERY ACK queued during can_rx::poll() and feed each to the tracker
    // exactly once.  can_rx::poll() can decode several CMD_ACK frames in a
    // single call, so consuming only the last-seen ack_ slot would lose the
    // rest and let their pending commands time out even though the ACK arrived.
    // The FIFO is bounded and holds every received ACK in order (see
    // VehicleData::pushAck / popAck).  No timestamp de-duplication is used, so
    // the very first ACK (millis() == 0) is handled correctly too.
    vehicle::AckData ad;
    while (data.popAck(ad)) {
        if (ackTracker.onAck(ad.cmdIdLow) == ack::MatchResult::MATCHED) {
            if (ad.result != can::AckResult::OK) {
                Serial.printf("[ACK] cmd 0x%02X result=%u state=%u\n",
                              ad.cmdIdLow, static_cast<uint8_t>(ad.result),
                              static_cast<uint8_t>(ad.systemState));
            }
        }
        // Unexpected ACKs (no matching pending entry) are ignored.
    }

    // Expire any commands that never got acknowledged (bounded, no retry).
    unsigned long now = millis();
    for (;;) {
        ack::TimeoutInfo to = ackTracker.drainTimeout(now);
        if (!to.valid) break;
        data.setAckTimeout(now);
        Serial.printf("[ACK] TIMEOUT waiting for cmd 0x%02X\n", to.cmdIdLow);
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
// readTftStatus() — attempt an ST7796 status/ID readback (audit §4.1).
//
// Runs on Core 0 only (owns the SPI bus).  Reads the panel ID (RDDID 0x04) at
// SPI_READ_FREQUENCY via TFT_eSPI.  The ST7796 wiring here is documented as
// having unreliable MISO readback, so this is treated conservatively:
//   - an all-zero / all-ones / clearly implausible response  -> UNSUPPORTED
//     (we refuse to assert a controller state we cannot trust), and
//   - a stable, plausible ID                                  -> VALID.
// It NEVER returns INVALID from a single dubious read, so a flaky MISO can not
// fabricate a "controller lost" event.  If a future panel proves reliable
// readback, INVALID can be returned here to drive TFT_STATUS_LOST.
static display::StatusRead readTftStatus() {
    // RDDID returns 3 ID bytes (manufacturer / version / driver).  index 1..3.
    uint8_t id1 = tft.readcommand8(0x04, 1);
    uint8_t id2 = tft.readcommand8(0x04, 2);
    uint8_t id3 = tft.readcommand8(0x04, 3);
    const uint32_t id = ((uint32_t)id1 << 16) | ((uint32_t)id2 << 8) | id3;
    // 0x000000 and 0xFFFFFF are the classic "bus floating / no MISO" patterns.
    if (id == 0x000000UL || id == 0xFFFFFFUL) {
        return display::StatusRead::UNSUPPORTED;
    }
    return display::StatusRead::VALID;
}

// ---------------------------------------------------------------------------
// executeDisplayRecovery() — the REAL Core-0 TFT recovery choreography.
//
// Runs exclusively inside the render task (Core 0), which is the ONLY task that
// touches the TFT SPI bus, so bus exclusion is inherent — no other task can
// interleave a transaction.  Executes the exact ordered steps proven by
// display_recovery.h / test_display_recovery.cpp.  Returns the TRISTATE result
// (VERIFIED / UNVERIFIED_READBACK_UNSUPPORTED / FAILED); it does NOT draw the
// banner — Core 1 assembles it with real counters and shows it as a
// ScreenManager overlay (audit P2.3/P2.4).
// ---------------------------------------------------------------------------
// Recovery hardware ops (Core 0 only).  These are the REAL implementations of
// the injected display::RecoveryOps callbacks; display::runRecovery() drives
// them in the canonical order with the retry/final-failure policy that
// test_display_recovery.cpp verifies with mocks.
namespace {
struct RecoveryCtx {
    int  tftCs          = 10;
    int  touchCs        = 21;
    int  tftRst         = 38;
    bool status_invalid = false;  // reliable readback (future) reported INVALID
    bool observable     = false;  // whiteScreenObservable() at last verify
};

void rec_stopRenderTouch(void*) { /* we are the sole SPI owner on Core 0 */ }
void rec_blockBus(void*)        { /* bus exclusion is inherent on Core 0 */ }
void rec_tftCsHigh(void* v)     { auto* c = static_cast<RecoveryCtx*>(v);
                                  pinMode(c->tftCs, OUTPUT);  digitalWrite(c->tftCs, HIGH); }
void rec_touchCsHigh(void* v)   { auto* c = static_cast<RecoveryCtx*>(v);
                                  pinMode(c->touchCs, OUTPUT); digitalWrite(c->touchCs, HIGH); }
void rec_closeSpi(void*)        { tft.endWrite(); }
void rec_pulseReset(void* v)    { auto* c = static_cast<RecoveryCtx*>(v);
                                  pinMode(c->tftRst, OUTPUT);
                                  digitalWrite(c->tftRst, HIGH); delay(5);
                                  digitalWrite(c->tftRst, LOW);  delay(20);
                                  digitalWrite(c->tftRst, HIGH); delay(150);
                                  g_tftResetSignalLatched = false; }
void rec_tftInit(void*)         { tft.init(); }
void rec_setRotation(void*)     { tft.setRotation(1); }
void rec_restoreTouchCal(void*) { uint16_t calData[5] = TOUCH_CALIBRATION;
                                  (void)touch_calibration::loadValid(calData);
                                  tft.setTouch(calData); }
void rec_restoreBacklight(void*){ const auto& cfg = config_store::get();
                                  display_backlight::apply(cfg.brightness); }
void rec_invalidateCaches(void*){ tft.fillScreen(0x2104); }
void rec_forceFullRedraw(void*) { screenManager.forceFullRedraw(); }
display::RecoveryResult rec_verify(void* v)  { auto* c = static_cast<RecoveryCtx*>(v);
                                  const display::StatusRead st = readTftStatus();
                                  c->status_invalid = (st == display::StatusRead::INVALID);
                                  c->observable = display::whiteScreenObservable(
                                      st != display::StatusRead::UNSUPPORTED,
                                      /*reset_latch_available=*/false);
                                  // Tristate (audit P2.3): a reliable INVALID is a
                                  // real FAILED; a genuine VALID readback is VERIFIED;
                                  // with no readback support we CANNOT claim success,
                                  // so the honest result is UNVERIFIED (never a
                                  // fabricated "pantalla blanca confirmada").
                                  if (st == display::StatusRead::INVALID)
                                      return display::RecoveryResult::FAILED;
                                  if (st == display::StatusRead::VALID)
                                      return display::RecoveryResult::VERIFIED;
                                  return display::RecoveryResult::UNVERIFIED_READBACK_UNSUPPORTED; }
void rec_resumeRenderTouch(void*) { /* control returns to render loop */ }
}  // namespace

// Runs the choreography and returns the tristate verification result plus the
// measured duration.  It does NOT draw the banner directly (audit P2.4): the
// banner is assembled on Core 1 with the real supervisor counters and shown as
// a ScreenManager overlay so the next full redraw cannot erase it.
static display::RecoveryResult executeDisplayRecovery(display::RecoveryTrigger trigger,
                                                      uint32_t& duration_ms_out,
                                                      uint8_t&  attempts_out) {
    const uint32_t t0 = millis();
    Serial.printf("[DISPLAY][RECOVERY] start — trigger: %s\n",
                  display::recoveryTriggerText(trigger));

    RecoveryCtx ctx;
#if defined(TFT_CS)
    ctx.tftCs = TFT_CS;
#endif
#if defined(TOUCH_CS)
    ctx.touchCs = TOUCH_CS;
#endif
#if defined(TFT_RST)
    ctx.tftRst = TFT_RST;
#endif

    display::RecoveryOps ops{};
    ops.stopRenderTouch   = rec_stopRenderTouch;
    ops.blockBus          = rec_blockBus;
    ops.tftCsHigh         = rec_tftCsHigh;
    ops.touchCsHigh       = rec_touchCsHigh;
    ops.closeSpi          = rec_closeSpi;
    ops.pulseReset        = rec_pulseReset;
    ops.tftInit           = rec_tftInit;
    ops.setRotation       = rec_setRotation;
    ops.restoreTouchCal   = rec_restoreTouchCal;
    ops.restoreBacklight  = rec_restoreBacklight;
    ops.invalidateCaches  = rec_invalidateCaches;
    ops.forceFullRedraw   = rec_forceFullRedraw;
    ops.verify            = rec_verify;
    ops.resumeRenderTouch = rec_resumeRenderTouch;
    ops.onStep            = nullptr;

    // Up to 3 reset→init→redraw→verify cycles.  With UNSUPPORTED readback the
    // first verify() returns the terminal UNVERIFIED result (1 attempt); a
    // future reliable readback that keeps reporting INVALID drives the honest
    // "reintentos y fallo final" path.
    const display::RecoveryOutcome oc = display::runRecovery(ops, &ctx, /*maxAttempts=*/3);

    duration_ms_out = millis() - t0;
    attempts_out    = oc.attempts;
    Serial.printf("[DISPLAY][RECOVERY] done — %s | resultado=%s | intentos=%u | %lu ms\n",
                  ctx.observable ? "observable" : "no observable",
                  display::recoveryResultText(oc.result),
                  (unsigned)oc.attempts,
                  (unsigned long)duration_ms_out);

    return oc.result;
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
        // 0. Core-1 recovery request check (audit P2 §4).  Runs BEFORE any
        // rendering so the choreography owns the SPI bus exclusively this
        // iteration.  Only Core 0 ever touches the TFT, so taking the request
        // here guarantees no interleaved transaction.
        {
            display::RecoveryTrigger trig = display::RecoveryTrigger::NONE;
            bool have = false;
            const uint32_t tnow = millis();
            taskENTER_CRITICAL(&g_recoveryMux);
            have = g_recoveryMailbox.take(trig, tnow);
            taskEXIT_CRITICAL(&g_recoveryMux);
            if (have) {
                uint32_t dur = 0; uint8_t attempts = 0;
                const display::RecoveryResult res =
                    executeDisplayRecovery(trig, dur, attempts);
                taskENTER_CRITICAL(&g_recoveryMux);
                g_recoveryMailbox.done(res, dur);
                taskEXIT_CRITICAL(&g_recoveryMux);
                // Publish a fresh heartbeat so the supervisor sees the render
                // path alive again immediately after recovery.
                renderFrameCounter = renderFrameCounter + 1;
                renderHeartbeatMs  = millis();
                vTaskDelay(1);
                continue;
            }
        }

        // 0b. Consume a post-recovery banner posted by Core 1 (audit P2.4) and
        // show it as a ScreenManager overlay (redrawn on top of every frame for
        // ~9 s, surviving the recovery's full redraw).  Only Core 0 touches the
        // TFT, so drawing here is safe.
        {
            char banner[320];
            if (takeRecoveryBanner(banner, sizeof(banner))) {
                screenManager.showRecoveryBanner(banner, millis());
            }
        }

        // 0c. Surface a persisted render-blocked reboot on the HMI (audit P2
        // final).  setup() latched the persisted cause here (Core 1) before
        // this task existed; publish it once as a ScreenManager overlay now
        // that the render path/ScreenManager are alive, then clear ONLY
        // rb_pending (keeping rb_count as the historical counter) so later
        // normal boots do not repeat the banner.
        if (s_rebootBannerPending) {
            char banner[320];
            display::formatRebootRecoveryBanner(banner, sizeof(banner),
                                                s_rebootBannerInfo);
            screenManager.showRecoveryBanner(banner, millis());
            s_rebootBannerPending = false;
            clearRenderBlockedPending();
        }

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

            if (evt.type == touch::EventType::CORNER_LONG_PRESS) {
                // Robust Engineering-access fallback (top-right / battery icon).
                // Works even in LIMP_HOME / CAN-LOST; gated to parked vehicle
                // inside the screen manager.  Suppressed during tank confirm.
                if (!screenManager.isTankConfirmActive()) {
                    screenManager.onCornerLongPress(evt.x, evt.y);
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
            // Debug overlay (perf stats) — toggled explicitly from the
            // Engineering menu, NOT from the global long-press gesture.
            // Never paint it over a blocking screen (PIN / Engineering /
            // touch-cal) so it can't be superimposed on those menus.
            // Uses lastFrameStart (captured once per frame at line 367)
            // to comply with frame time contract — no direct millis() in UI.
            if (!screenManager.isBlockingInput()) {
                RTMON_OVERLAY_UPDATE(isTouched, lastFrameStart);
                RTMON_OVERLAY_DRAW(tft, lastFrameStart);
            }
#endif
        }

        // 5. Publish render heartbeat (audit P2).  Reaching this point means
        // the whole frame completed: screenManager.update(), TFT drawing, the
        // touch read and the implicit SPI transaction release all finished
        // without blocking.  A stalled timestamp therefore signals a hung
        // render/SPI path to the Core-1 supervisor.
        renderFrameCounter = renderFrameCounter + 1;
        renderHeartbeatMs  = millis();

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
    boot_diag::ResetClass bootRc =
        reason == ESP_RST_POWERON   ? boot_diag::ResetClass::POWER_ON  :
        reason == ESP_RST_SW        ? boot_diag::ResetClass::SOFTWARE  :
        reason == ESP_RST_PANIC     ? boot_diag::ResetClass::PANIC     :
        reason == ESP_RST_INT_WDT   ? boot_diag::ResetClass::WATCHDOG  :
        reason == ESP_RST_TASK_WDT  ? boot_diag::ResetClass::WATCHDOG  :
        reason == ESP_RST_WDT       ? boot_diag::ResetClass::WATCHDOG  :
        reason == ESP_RST_BROWNOUT  ? boot_diag::ResetClass::BROWNOUT  :
        reason == ESP_RST_DEEPSLEEP ? boot_diag::ResetClass::DEEPSLEEP :
        reason == ESP_RST_SDIO      ? boot_diag::ResetClass::EXTERNAL_PIN :
        reason == ESP_RST_EXT       ? boot_diag::ResetClass::EXTERNAL_PIN :
                                      boot_diag::ResetClass::UNKNOWN;
    Serial.printf("[BOOT][INFO] Reset reason: %s%s\n",
                  boot_diag::name(bootRc),
                  boot_diag::isAbnormal(bootRc) ? " (ABNORMAL)" : "");

    Serial.println("[BOOT][INFO] ESP32 HMI CAN bring-up booted");

    // Initialize NVS config store
    config_store::init();

    // Audit P2.1 — if the previous session ended with a controlled restart due
    // to a genuinely blocked render task, surface the persisted cause on boot
    // so the fault is not silently lost.
    {
        Preferences p;
        if (p.begin("disp_fault", /*readOnly=*/true)) {
            const uint32_t rbCount = p.getUInt("rb_count", 0);
            if (rbCount > 0) {
                const uint8_t  cause  = p.getUChar("rb_cause", 0);
                const uint32_t uptime = p.getUInt("rb_uptime", 0);
                const bool     pending = p.getBool("rb_pending", false);
                Serial.printf("[BOOT][DISPLAY] previous render-blocked restart — "
                              "cause=%s, at %lu ms uptime, total=%lu, pending=%s\n",
                              display::recoveryTriggerText(
                                  (display::RecoveryTrigger)cause),
                              (unsigned long)uptime, (unsigned long)rbCount,
                              pending ? "si" : "no");
                // audit P2 final — latch the banner for the render task to show
                // once (only when still pending).  We do NOT draw here: the TFT
                // and ScreenManager are not initialised yet at this point.
                if (display::shouldShowRebootBanner(pending, rbCount)) {
                    s_rebootBannerInfo.trigger   = (display::RecoveryTrigger)cause;
                    s_rebootBannerInfo.uptime_ms = uptime;
                    s_rebootBannerInfo.count     = rbCount;
                    s_rebootBannerPending        = true;
                }
            }
            p.end();
        }
    }

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

    // Initialize TFT backlight PWM and apply persisted brightness (clamped to
    // a safe visible range so the screen never appears off after reboot).
    {
        const auto& cfg = config_store::get();
        const uint8_t applied = display_backlight::apply(cfg.brightness);
        if (applied != cfg.brightness) {
            config_store::setBrightness(applied);
            config_store::flush();
        }
    }

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

    // Initialize remote control parser (FlySky FS-iA6B iBUS, GPIO 16 RX).
    // Enabled by default (REMOTE_CONTROL_ENABLED=1 in platformio.ini);
    // compiles to an inline no-op when the flag is set to 0.  See
    // docs/REMOTE_CONTROL_IMPLEMENTATION_PLAN.md Phase 2.
    remote_control::init();

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
        lastAppliedAudioVolume = cfg.audioVolume;

        // Restore saved LED decorative mode
        led_ctrl::setDecorMode(static_cast<led_ctrl::DecorMode>(cfg.ledMode));

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
            renderTask, "Render", 16384, nullptr, 1, &renderTaskHandle, 0);
        if (rc != pdPASS) {
            Serial.println("[BOOT][ERR] Failed to create render task");
        } else {
            Serial.println("[BOOT][INFO] Render task started on Core 0");
        }
    }

    // Boot-diagnostics telemetry — single machine-parseable line emitted once
    // per boot on the serial channel (the ESP32 HMI is CAN receive-only, so it
    // has no diagnostic-frame TX path).  Captures the reset cause plus the heap
    // low-water mark and both task stack high-watermarks so brownouts, panics,
    // watchdog resets, heap exhaustion and stack pressure are all observable
    // from the boot log.  Formatting/classification lives in the host-tested
    // boot_diag module; this is pure instrumentation.
    {
        boot_diag::HeapStats hs;
        hs.freeNow      = (uint32_t)esp_get_free_heap_size();
        hs.minFreeEver  = (uint32_t)esp_get_minimum_free_heap_size();
        hs.totalSize    = (uint32_t)ESP.getHeapSize();
        hs.largestBlock = (uint32_t)ESP.getMaxAllocHeap();
        uint32_t loopHwm   = (uint32_t)uxTaskGetStackHighWaterMark(nullptr);
        uint32_t renderHwm = (renderTaskHandle != nullptr)
                           ? (uint32_t)uxTaskGetStackHighWaterMark(renderTaskHandle)
                           : 0u;
        char bootLine[192];
        boot_diag::format(bootLine, sizeof(bootLine), bootRc, hs, loopHwm, renderHwm);
        Serial.println(bootLine);
    }
}

// ---------------------------------------------------------------------------
// loop() — called repeatedly
// ---------------------------------------------------------------------------
void loop() {
    RTMON_LOOP_BEGIN();
    unsigned long now = millis();

    // R-3 hardening: emit stack high-watermark for both FreeRTOS tasks
    // every 5 s.  Pure instrumentation — uxTaskGetStackHighWaterMark
    // is non-blocking and reports the minimum free stack (in words on
    // ESP-IDF FreeRTOS) seen since task creation.  No stack sizes are
    // changed; this only makes silent overflow risk observable.
    {
        static uint32_t s_lastStackHwmLogMs = 0;
        if ((now - s_lastStackHwmLogMs) >= 5000UL) {
            s_lastStackHwmLogMs = now;
            UBaseType_t loopHwm   = uxTaskGetStackHighWaterMark(nullptr);
            UBaseType_t renderHwm = (renderTaskHandle != nullptr)
                                  ? uxTaskGetStackHighWaterMark(renderTaskHandle)
                                  : 0;
            Serial.printf("[STACK] loop_hwm=%u render_hwm=%u\n",
                          (unsigned)loopHwm, (unsigned)renderHwm);
        }
    }

    // ---- Bench/manual TFT recovery request (audit P2 §4.2) ----
    // The white screen is NOT automatically observable with the current wiring
    // (no reliable readback, no reset-signal latch), so an operator on the
    // bench can explicitly request a recovery by sending 'R' (or 'r') over the
    // USB serial console.  This sets a flag consumed by the supervisor block
    // below, which forces a confirmed loss and drives the Core-0 choreography.
    while (Serial.available() > 0) {
        int c = Serial.read();
        if (c == 'R' || c == 'r') {
            g_manualRecoveryRequest = true;
            Serial.println("[DISPLAY][RECOVERY] manual/bench request received");
        }
    }

    // ---- Display supervisor + recovery orchestration (audit P2) ----
    // Core-1 does DETECTION only and, when a recovery is warranted, REQUESTS it
    // from the Core-0 render task (which owns the SPI bus).  Core 1 NEVER calls
    // TFT_eSPI.  Readback is reported as UNSUPPORTED here because the ST7796
    // panel cannot be read back reliably with the current wiring, so a stalled
    // render heartbeat is honestly classified as RENDER_STALLED (an observable
    // fact) — never asserted as "pantalla blanca confirmada".  A bench/manual
    // request (g_manualRecoveryRequest) exists precisely because the white
    // screen is NOT automatically observable with this hardware.
    {
        static display::Supervisor s_dispSup;
        static uint32_t s_lastDispEvalMs = 0;
        static display::State s_lastDispState = display::State::OK;
        static bool s_recoveryPosted = false;
        static display::RecoveryTrigger s_lastTrigger = display::RecoveryTrigger::NONE;
        const uint32_t kRenderStaleMs = 500;  // mirrors Config::render_stale_ms
        // Bounded wait for the render task to consume+finish a recovery request
        // before we treat it as genuinely blocked (audit P2.1).
        const uint32_t kRenderBlockedMs = 3000;

        // First, finalise any recovery the render task has already completed so
        // the supervisor's counters / backoff stay consistent.
        {
            display::RecoveryResult res = display::RecoveryResult::FAILED;
            uint32_t dur = 0;
            bool haveResult;
            taskENTER_CRITICAL(&g_recoveryMux);
            haveResult = g_recoveryMailbox.takeResult(res, dur);
            taskEXIT_CRITICAL(&g_recoveryMux);
            if (haveResult) {
                // VERIFIED and UNVERIFIED both map to a non-failed completion:
                // the choreography ran; only a reliable INVALID readback is a
                // real FAILED (audit P2.3 tristate).
                const bool ok = (res != display::RecoveryResult::FAILED);
                const uint8_t attemptsDone = s_dispSup.attempts();
                // Walk the pure FSM through its hardware stages, then complete.
                s_dispSup.advanceRecovery((uint32_t)now);   // -> RESETTING
                s_dispSup.advanceRecovery((uint32_t)now);   // -> REINITIALIZING
                s_dispSup.advanceRecovery((uint32_t)now);   // -> REDRAWING
                s_dispSup.completeRecovery((uint32_t)now, ok);
                s_recoveryPosted = false;

                // Assemble the banner with the REAL supervisor counters (audit
                // P2.4/P2.5 — no hardcoded total_recoveries / render_continued)
                // and post it to Core 0 to be shown as a managed overlay.
                display::RecoveryBannerInfo info{};
                info.cause            = display::recoveryTriggerToFault(s_lastTrigger);
                info.trigger          = s_lastTrigger;
                info.duration_ms      = dur;
                info.attempts         = attemptsDone;
                info.verify_result    = res;
                info.render_continued = true;   // render consumed+finished (no reboot)
                info.esp32_rebooted   = false;
                info.free_heap        = (uint32_t)ESP.getFreeHeap();
                info.total_recoveries = s_dispSup.recoveryCount();
                char banner[320];
                display::formatRecoveryBanner(banner, sizeof(banner), info);
                postRecoveryBanner(banner);

                Serial.printf("[DISPLAY][RECOVERY] finalised — %s (%lu ms) | "
                              "estado=%s recuperaciones=%lu fallidas=%lu\n",
                              display::recoveryResultText(res), (unsigned long)dur,
                              display::stateText(s_dispSup.state()),
                              (unsigned long)s_dispSup.recoveryCount(),
                              (unsigned long)s_dispSup.recoveryFailCount());
            }
        }

        // Audit P2.1 — if a recovery was posted but the render task never
        // consumed/finished it within the bounded window, the render path is
        // genuinely blocked.  Persist the cause and perform a controlled ESP32
        // restart as a last resort.  Core 1 must NOT touch the TFT/SPI here.
        if (s_recoveryPosted) {
            bool stalled;
            taskENTER_CRITICAL(&g_recoveryMux);
            stalled = g_recoveryMailbox.renderStalled((uint32_t)now, kRenderBlockedMs);
            taskEXIT_CRITICAL(&g_recoveryMux);
            if (stalled) {
                persistRenderBlockedAndReboot(s_lastTrigger);  // does not return
            }
        }

        if ((now - s_lastDispEvalMs) >= 100UL) {
            s_lastDispEvalMs = now;

            const bool manual = g_manualRecoveryRequest;
            const bool reset_latched = g_tftResetSignalLatched;

            // A manual/bench request forces a confirmed loss so the FSM opens a
            // recovery attempt (the white screen is otherwise not observable).
            if (manual && !s_dispSup.recovering() &&
                s_dispSup.state() != display::State::RECOVERY_FAILED) {
                s_dispSup.forceFault(
                    (uint32_t)now,
                    display::recoveryTriggerToFault(
                        display::RecoveryTrigger::RECOVERY_MANUAL_REQUEST));
            }
            g_manualRecoveryRequest = false;

            display::State st = s_dispSup.update(
                (uint32_t)now,
                (uint32_t)renderHeartbeatMs,
                display::StatusRead::UNSUPPORTED,
                (uint32_t)ESP.getFreeHeap());

            // When the FSM asks for a recovery, classify the trigger honestly
            // and post the request to Core 0 (once per attempt).
            if (st == display::State::RECOVERY_REQUESTED && !s_recoveryPosted) {
                const bool render_stale =
                    (uint32_t)(now - renderHeartbeatMs) >= kRenderStaleMs;
                display::RecoveryTrigger trig = display::classifyRecoveryTrigger(
                    manual, reset_latched,
                    display::StatusRead::UNSUPPORTED, render_stale);
                if (requestDisplayRecovery(trig)) {
                    s_recoveryPosted = true;
                    s_lastTrigger    = trig;
                    Serial.printf("[DISPLAY][RECOVERY] requested from Core 1 — "
                                  "trigger: %s | observable=%s\n",
                                  display::recoveryTriggerText(trig),
                                  display::whiteScreenObservable(false, reset_latched)
                                      ? "si" : "no");
                }
            }

            if (st != s_lastDispState) {
                if (st == display::State::CONFIRMED_LOST) {
                    display::Fault f = s_dispSup.lastFault();
                    display::RecoveryBannerInfo info{};
                    info.cause            = f;
                    info.duration_ms      = 0;
                    info.attempts         = s_dispSup.attempts();
                    info.render_continued = false;
                    info.esp32_rebooted   = false;
                    info.free_heap        = (uint32_t)ESP.getFreeHeap();
                    info.total_recoveries = s_dispSup.recoveryCount();
                    Serial.printf("[DISPLAY] fault suspected: %s | %s\n",
                                  display::faultText(f),
                                  display::recoveryActionText(info));
                } else if (st == display::State::RECOVERY_FAILED) {
                    Serial.printf("[DISPLAY][RECOVERY] FAILED after %u attempts — "
                                  "manual intervention required\n",
                                  (unsigned)s_dispSup.attempts());
                }
                s_lastDispState = st;
            }
        }
    }

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
            // Evaluate the alive counter only when a NEW heartbeat frame has
            // been decoded (tracked via hb.timestampMs).  This prevents the
            // same sample from being counted repeatedly while loop() spins far
            // faster than the ~100 ms heartbeat period (root cause of the
            // FALSE "STM32 frozen" detection).  A time-based absence timeout
            // covers the "no new heartbeat at all" case.
            stm32Liveness.update((uint32_t)hb.timestampMs, hb.aliveCounter, (uint32_t)now);
            stm32IsAlive = stm32Liveness.isAlive();
            if (stm32Liveness.consumeFrozenEdge() &&
                (now - stm32FrozenWarnLastMs) >= STM32_FROZEN_WARN_INTERVAL_MS) {
                stm32FrozenWarnLastMs = now;
                Serial.println("[SAFETY][WARN] STM32 heartbeat counter frozen — inhibiting commands");
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
                // The STM32 restarted → whatever drive mode it last confirmed
                // is stale.  Invalidate the ModeSync confirmation so the
                // physical selector's mode is RE-TRANSMITTED even if it equals
                // the previously-confirmed value (root cause of a confirmed
                // 4x4 never being resent after an STM32 restart).
                g_modeSync.invalidateConfirmed();
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

            // ---- Drive-mode confirmation from the heartbeat echo ----
            // The STM32 heartbeat echoes its applied drive mode (statusFlags
            // bit1 = 4x4, bit2 = tank turn).  Feed it into ModeSync as remote
            // confirmation so a selected mode still confirms even when the
            // CMD_ACK for our CMD_MODE was lost, and so a silent STM32 mode
            // revert is re-synchronised.  Processed once per NEW heartbeat
            // frame (tracked via hb.timestampMs) and only while the link is
            // proven alive.  Map STM32 statusFlags bits → CMD_MODE flag bits.
            if (stm32IsAlive &&
                (uint32_t)hb.timestampMs != (uint32_t)g_lastModeEchoTs) {
                g_lastModeEchoTs = (unsigned long)hb.timestampMs;
                uint8_t echoFlags = 0;
                if (hb.statusFlags & 0x02u) echoFlags |= can::MODE_FLAG_4X4;
                if (hb.statusFlags & 0x04u) echoFlags |= can::MODE_FLAG_TANK_TURN;
                g_modeSync.onHeartbeatModeEcho(echoFlags);
            }

            // ---- One-shot LED relay restore from NVS after boot ----
            // If the front/rear LED strips were saved as ENABLED, power them
            // up automatically once the STM32 is alive and out of BOOT — the
            // user should NOT have to press the button after boot.  An explicit
            // OFF state (both false) is respected: nothing is sent.  Only the
            // LED relays are touched (CMD_LED); traction/steering relays are
            // never affected here.  Retries (rate-limited, bounded) until the
            // STM32 echoes the desired relay state so a dropped CAN frame can't
            // silently leave the strips off.
            if (!ledBootRestoreDone && stm32IsAlive &&
                (hb.systemState == can::SystemState::STANDBY ||
                 hb.systemState == can::SystemState::ACTIVE)) {
                if (!frontLedLocalState && !rearLedLocalState) {
                    ledBootRestoreDone = true;  // OFF saved — respect it, nothing to do
                } else if (vehicleData.lights().frontRelayOn == frontLedLocalState &&
                           vehicleData.lights().rearRelayOn  == rearLedLocalState) {
                    ledBootRestoreDone = true;  // STM32 already reflects saved state
                } else if (ledBootRestoreAttempts >= LED_BOOT_RESTORE_MAX_ATTEMPTS) {
                    ledBootRestoreDone = true;  // give up after bounded retries
                } else if ((now - ledBootRestoreLastMs) >= LED_BOOT_RESTORE_RETRY_MS) {
                    sendLedCommand(frontLedLocalState, rearLedLocalState);
                    ledBootRestoreLastMs = now;
                    ledBootRestoreAttempts++;
                    Serial.printf("[LED] Boot restore from NVS (try %u) → front=%s rear=%s\n",
                                  ledBootRestoreAttempts,
                                  frontLedLocalState ? "ON" : "OFF",
                                  rearLedLocalState ? "ON" : "OFF");
                }
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

    // Poll iBUS parser (no-op only when REMOTE_CONTROL_ENABLED=0) and compute
    // unified LOCAL/REMOTE arbitration gates for all RC-driven subsystems.
    remote_control::update();
    const bool remoteAuthorityActive =
        stm32IsAlive &&
        remote_control::isRemoteSelected() &&
        remote_control::getState() == remote_control::State::ACTIVE;
    const bool remoteMotionAuthorityActive =
        remoteAuthorityActive && !remote_control::isKillSwitchActive();

    // ---- Shifter gear update ----
    // Poll shifter + RC CH7 and send CAN 0x102 on authoritative source change.
    {
        shifter::update();
        uint8_t gear = remoteMotionAuthorityActive
                     ? remote_control::getRequestedGear()
                     : shifter::getGearRaw();
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
            Serial.printf(remoteMotionAuthorityActive // flawfinder: ignore
                              ? "[REMOTE] Gear(CH7) → %u\n"
                              : "[SHIFTER] Gear → %u\n",
                          gear);
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

        if (remoteMotionAuthorityActive) {
            uint8_t remoteModeFlags = remote_control::getDriveMode();
            if (remoteModeFlags != currentModeFlags && stm32IsAlive) {
                currentModeFlags = remoteModeFlags;
                sendModeCommand(currentModeFlags);
                vehicle::ModeData md;
                md.modeFlags   = currentModeFlags;
                md.timestampMs = millis();
                vehicleData.setMode(md);
                Serial.printf("[REMOTE] Mode(CH6) → flags=0x%02X\n", currentModeFlags);
            }
        } else if (traction_sw::hasChanged() && stm32IsAlive) {
            // Physical 4x4 selector is the SOURCE OF TRUTH.  Latch the
            // debounced selection into the sync FSM; the actual CMD_MODE
            // transmission (heartbeat-gated, ACK + bounded retry) is driven
            // by g_modeSync below instead of a single fire-and-forget send.
            uint8_t tractionBit = traction_sw::getModeFlag();
            currentModeFlags = (currentModeFlags & can::MODE_FLAG_TANK_TURN)
                             | tractionBit;
            g_modeSync.setDesired(currentModeFlags);
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

        // ---- Drive-mode sync FSM tick (physical selector path only) ----
        // Feeds observed CMD_MODE ACKs into the FSM, then advances it: the FSM
        // (re)transmits the selected mode until the STM32 confirms it or the
        // bounded retry budget is exhausted.  Skipped while the remote holds
        // motion authority so the two paths never fight over CMD_MODE.
        if (!remoteMotionAuthorityActive) {
            const auto& ad = vehicleData.ack();
            if (g_modeSync.pending()
                && ad.cmdIdLow == (can::CMD_MODE & 0xFF)
                && ModeSync::ackIsAtOrAfterSend((uint32_t)ad.timestampMs,
                                                (uint32_t)g_modeSendMs)
                && ad.timestampMs != g_lastModeAckTs) {
                g_lastModeAckTs = ad.timestampMs;
                bool ackOk = (ad.result == can::AckResult::OK);
                g_modeSync.onAck(ackOk ? ModeSync::AckResult::OK
                                     : ModeSync::AckResult::REJECTED);
                // [MODESYNC] diagnostic — last ACK + requested/confirmed/retry.
                Serial.printf(
                    "[MODESYNC] ack=%s req=0x%02X conf=0x%02X retries=%u%s\n",
                    ackOk ? "OK" : "REJECT",
                    g_modeSync.desired(), g_modeSync.confirmed(),
                    (unsigned)g_modeSync.retries(),
                    g_modeSync.failed() ? " -> FAILED" : "");
            }
            if (g_modeSync.update(millis(), stm32IsAlive)
                    == ModeSync::Action::SEND) {
                g_modeSendMs = millis();
                sendModeCommand(g_modeSync.sendMode());
                // [MODESYNC] diagnostic — each (re)transmission attempt.
                Serial.printf(
                    "[MODESYNC] send req=0x%02X conf=0x%02X attempt=%u/%u\n",
                    g_modeSync.desired(), g_modeSync.confirmed(),
                    (unsigned)g_modeSync.retries() + 1u,
                    (unsigned)MODE_SYNC_MAX_RETRIES + 1u);
            }
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
                    if (remoteAuthorityActive) break;
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
                    if (remoteAuthorityActive) break;
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
                    if (stm32IsAlive && !remoteAuthorityActive) {
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

    // ---- Remote lights (CH8) ----
    // In REMOTE authority mode, CH8 owns both front/rear light relays.
    if (remoteAuthorityActive) {
        bool lightsOn = remote_control::isLightsOn();
        if (frontLedLocalState != lightsOn || rearLedLocalState != lightsOn) {
            frontLedLocalState = lightsOn;
            rearLedLocalState  = lightsOn;
            sendLedCommand(frontLedLocalState, rearLedLocalState);
            config_store::setFrontLedEnabled(frontLedLocalState);
            config_store::setRearLedEnabled(rearLedLocalState);
            audio::play(lightsOn ? audio::Sound::LIGHTS_ON
                                 : audio::Sound::LIGHTS_OFF,
                        audio::Priority::LO);
            Serial.printf("[REMOTE] Lights(CH8) → %s\n", lightsOn ? "ON" : "OFF");
        }
    }

    // ---- Remote audio volume (CH9) ----
    // Debounce + hysteresis to avoid rapid volume chatter from RC jitter.
    if (remoteAuthorityActive) {
        uint8_t remoteVolume = remote_control::getAudioVolume();
        uint8_t delta = (remoteVolume > lastAppliedAudioVolume)
                      ? (remoteVolume - lastAppliedAudioVolume)
                      : (lastAppliedAudioVolume - remoteVolume);
        if (delta >= REMOTE_VOLUME_HYSTERESIS &&
            (now - lastRemoteVolumeSetMs) >= REMOTE_VOLUME_DEBOUNCE_MS) {
            audio::setVolume(remoteVolume);
            config_store::setAudioVolume(remoteVolume);
            lastAppliedAudioVolume = remoteVolume;
            lastRemoteVolumeSetMs  = now;
            Serial.printf("[REMOTE] Volume(CH9) → %u\n", remoteVolume);
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

        // Enable/disable LED system based on front relay and system state.
        // The front relay controls WS2812B power; when it is OFF (or the user
        // disabled the strips via NVS, reflected in frontRelayOn) the strips
        // stay dark.  BOOT keeps the strips off while the STM32 initialises.
        // STANDBY is intentionally NOT a disable condition: with the front
        // relay ON the driver must see the KITT scanner as soon as the strips
        // are powered, even before the system leaves STANDBY into ACTIVE.
        // (Traction/steering relays and safety are untouched by this.)
        if (!frontEnabled || st == can::SystemState::BOOT) {
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
            // NORMAL base: the front strip must ALWAYS show the red KITT /
            // "coche fantástico" scanner.  It must NEVER turn into a
            // rainbow/green pattern from throttle (traction scale) — only the
            // safety states above (ABS/TCS/REVERSE) may override it.
            led_ctrl::setFrontMode(led_ctrl::FrontMode::KITT_IDLE);
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

#if REMOTE_CONTROL_ENABLED
    // ------------------------------------------------------------------
    // Remote control → CAN bridge — Modo Control Remoto Clásico
    //
    // The ESP32 NEVER writes to 0x100/0x101 on behalf of the RC: those
    // IDs remain reserved for the local pedal/steering pipeline.
    //
    // Instead, every 50 ms the ESP32 emits CMD_RC_OVERRIDE (0x10A) with
    // raw stick values and a flag indicating whether the operator wants
    // RC to take control (CH10 == REMOTE && parser ACTIVE && CH5 not
    // killed).  The STM32 arbiter (rc_arbiter.c) decides which source
    // feeds Safety_ValidateThrottle / Safety_ValidateSteering using a
    // strict 200 ms watchdog.  Failsafe is automatic: if we stop sending
    // 0x10A (RC off, RF lost, ESP32 dead), the STM32 reverts to local
    // pedal control within 200 ms with no further action.
    // ------------------------------------------------------------------
    static uint32_t lastRcCmdMs = 0;
    static uint8_t  rcOverrideSeq = 0;
    if ((now - lastRcCmdMs) >= can::CMD_THROTTLE_RATE_MS) {
        lastRcCmdMs = now;

        // Override-active gate combines: parser fresh (≤150 ms),
        // CH5 kill switch released, CH10 selector in REMOTE, and STM32
        // alive.  Anything failing here → override_flag=0 → STM32
        // returns control to the pedal at the next arbiter check.
        bool overrideActive = remoteMotionAuthorityActive;

        // Sanitize stick values regardless of override gate so the
        // STM32 always sees in-range numbers (avoids spurious arbiter
        // rejections when override_flag=0).
        float thrPct = remote_control::getThrottlePct();
        if (thrPct < 0.0f)   thrPct = 0.0f;
        if (thrPct > 100.0f) thrPct = 100.0f;

        float strDeg = remote_control::getSteeringDeg();
        if (strDeg < -30.0f) strDeg = -30.0f;
        if (strDeg >  30.0f) strDeg =  30.0f;
        int16_t deg10 = static_cast<int16_t>(strDeg * 10.0f);

        CanFrame frRc = {};
        frRc.identifier       = can::CMD_RC_OVERRIDE;
        frRc.extd             = 0;
        frRc.data_length_code = 5;
        frRc.data[0]          = overrideActive ? 0x01 : 0x00;
        frRc.data[1]          = static_cast<uint8_t>(thrPct + 0.5f);
        frRc.data[2]          = static_cast<uint8_t>(deg10 & 0xFF);
        frRc.data[3]          = static_cast<uint8_t>((deg10 >> 8) & 0xFF);
        frRc.data[4]          = rcOverrideSeq++;
        ESP32Can.writeFrame(frRc, 0);
    }
#endif // REMOTE_CONTROL_ENABLED

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
     *   STOPPED → twai_start() → RUNNING (probation)
     *   RUNNING → sustained advancing STM32 heartbeats → recovery CONFIRMED
     *
     * The bus-off recovery decision is delegated to the pure, host-tested
     * twai_recovery::TwaiBusOffRecovery FSM (esp32/src/twai_recovery.h): the
     * controller merely reporting RUNNING is treated as PROBATION, not health,
     * so the photographed BUS_OFF↔RUNNING oscillation cannot falsely reset the
     * counters.  Retries are fast-then-slow (unbounded) so the node always
     * recovers after a physical reconnection, and the lifetime BUS_OFF count is
     * preserved.
     *
     * Error-passive recovery (two-phase):
     *   When TEC saturates at 128 (error-passive) the TWAI state remains
     *   RUNNING so bus-off recovery never triggers.  If error-passive
     *   persists for the timeout period, a full driver reinit
     *   (stop → uninstall → install → start) clears error counters.
     *   Phase 1: first 10 attempts at 3 s intervals (fast).
     *   Phase 2: unlimited attempts at 30 s intervals (slow periodic).
     *
     * Check every 250 ms. */
    if (now - lastBusOffCheckMs >= 250) {
        lastBusOffCheckMs = now;
        twai_status_info_t sts;
        if (twai_get_status_info(&sts) == ESP_OK) {
            // Map the ESP-IDF controller state to the pure-logic enum.
            twai_recovery::TwaiState st;
            switch (sts.state) {
                case TWAI_STATE_BUS_OFF:    st = twai_recovery::TwaiState::BUS_OFF;    break;
                case TWAI_STATE_RECOVERING: st = twai_recovery::TwaiState::RECOVERING; break;
                case TWAI_STATE_STOPPED:    st = twai_recovery::TwaiState::STOPPED;    break;
                default:                    st = twai_recovery::TwaiState::RUNNING;    break;
            }

            // Stable-heartbeat recovery FSM: RUNNING alone never confirms and
            // never resets counters — only sustained advancing STM32 heartbeats
            // (stm32IsAlive) confirm recovery.
            twai_recovery::TwaiAction act = g_twaiRecovery.update(now, st, stm32IsAlive);
            switch (act) {
                case twai_recovery::TwaiAction::INITIATE_RECOVERY:
                    Serial.printf("[CAN][ERR] BUS_OFF — recovery attempt %u "
                                  "(lifetime bus-off #%lu)\n",
                                  g_twaiRecovery.attempts(),
                                  (unsigned long)g_twaiRecovery.lifetimeBusOffCount());
                    twai_initiate_recovery();
                    break;
                case twai_recovery::TwaiAction::START:
                    if (twai_start() == ESP_OK) {
                        Serial.println("[CAN][INFO] Driver restarted — awaiting "
                                       "sustained STM32 heartbeats to confirm recovery");
                    }
                    break;
                case twai_recovery::TwaiAction::CONFIRMED:
                    Serial.printf("[CAN][INFO] BUS_OFF recovery CONFIRMED — "
                                  "stable heartbeat window sustained (recoveries=%lu)\n",
                                  (unsigned long)g_twaiRecovery.confirmedRecoveries());
                    break;
                case twai_recovery::TwaiAction::NONE:
                default:
                    break;
            }

            if (sts.state != TWAI_STATE_RUNNING) {
                errorPassiveSince = 0;  // not error-passive unless RUNNING
            } else {
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
