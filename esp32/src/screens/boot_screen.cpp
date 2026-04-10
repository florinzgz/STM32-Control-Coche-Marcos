// =============================================================================
// ESP32-S3 HMI — Boot Screen Implementation
// =============================================================================

#include "boot_screen.h"
#include "ui/ui_common.h"
#include "ui/ui_config.h"
#include "ui/render_trace.h"
#include "hmi/obstacle_indicator.h"
#include "sensors/obstacle_sensor.h"
#include <TFT_eSPI.h>
#include <driver/twai.h>

extern TFT_eSPI tft;

// RX frame flag bits — which STM32 frame types have been received recently
static constexpr uint16_t DIAG_RX_HB   = (1 << 0);
static constexpr uint16_t DIAG_RX_SPD  = (1 << 1);
static constexpr uint16_t DIAG_RX_CUR  = (1 << 2);
static constexpr uint16_t DIAG_RX_SAF  = (1 << 3);
static constexpr uint16_t DIAG_RX_STR  = (1 << 4);
static constexpr uint16_t DIAG_RX_TRC  = (1 << 5);
static constexpr uint16_t DIAG_RX_BAT  = (1 << 6);

// Diagnostic area layout (below obstacle indicator)
static constexpr int16_t DIAG_SEP_Y    = ui::SCREEN_H / 2 + 104;  // separator line
static constexpr int16_t DIAG_LINE_H   = 10;                       // line spacing
static constexpr int16_t DIAG_LINE1_Y  = DIAG_SEP_Y + 4;          // ESP32 bus status
static constexpr int16_t DIAG_LINE2_Y  = DIAG_LINE1_Y + DIAG_LINE_H;  // STM32 heartbeat
static constexpr int16_t DIAG_LINE3_Y  = DIAG_LINE2_Y + DIAG_LINE_H;  // RX frame flags
static constexpr int16_t DIAG_LINE4_Y  = DIAG_LINE3_Y + DIAG_LINE_H;  // error counts
static constexpr int16_t DIAG_LINE5_Y  = DIAG_LINE4_Y + DIAG_LINE_H;  // diagnostic verdict
static constexpr int16_t DIAG_MARGIN_X = 10;

// Timeout for considering a frame "recently received"
static constexpr unsigned long DIAG_RX_RECENT_MS = 2000;

// Text buffer sizes (sized for worst-case snprintf output)
static constexpr int DIAG_BUF_SIZE  = 56;   // "STM32: FROZEN cnt:255 St:LIMP F:FF E:FF" + NUL
static constexpr int DIAG_RX_BUF    = 40;   // "HB SPD CUR SAF STR TRC BAT " (28) + margin

// Freeze detection: if alive counter hasn't changed for 1 s, STM32 main loop is stuck
static constexpr unsigned long DIAG_FREEZE_MS = 1000;

// RX flag-to-label mapping for compact iteration
struct RxFlagLabel { uint16_t flag; const char* label; };
static constexpr RxFlagLabel RX_FLAG_TABLE[] = {
    { DIAG_RX_HB,  "HB "  },
    { DIAG_RX_SPD, "SPD " },
    { DIAG_RX_CUR, "CUR " },
    { DIAG_RX_SAF, "SAF " },
    { DIAG_RX_STR, "STR " },
    { DIAG_RX_TRC, "TRC " },
    { DIAG_RX_BAT, "BAT " },
};

// STM32 system state to short abbreviation (for diagnostic display)
static const char* stm32StateStr(uint8_t state) {
    switch (state) {
        case 0: return "BOOT";
        case 1: return "STBY";
        case 2: return "ACT";
        case 3: return "DEG";
        case 4: return "SAFE";
        case 5: return "ERR";
        case 6: return "LIMP";
        default: return "??";
    }
}

void BootScreen::onEnter() {
    RTRACE_BEGIN_SCREEN("boot");
    needsRedraw_ = true;
    canLinked_    = false;
    prevCanLinked_ = false;
    sensorStatus_     = obstacle_sensor::SensorStatus::WAITING;
    prevSensorStatus_ = obstacle_sensor::SensorStatus::WAITING;
    sensorDistanceMm_     = 0;
    prevSensorDistanceMm_ = 0;

    // Reset diagnostic state
    diagNeedsRedraw_ = true;
    diagBusState_    = 0xFF;
    diagTxErr_       = 0;
    diagRxErr_       = 0;
    diagTxFail_      = 0;
    diagRxMiss_      = 0;
    diagBusErr_      = 0;
    diagRxFlags_     = 0;
    diagObsActive_   = false;

    // Reset STM32 heartbeat diagnostic state
    diagStm32HbValid_      = false;
    diagStm32Frozen_       = false;
    diagStm32Alive_        = 0;
    diagStm32PrevAlive_    = 0xFF;
    diagStm32AliveChangedMs_ = 0;
    diagStm32State_        = 0xFF;
    diagStm32Faults_       = 0;
    diagStm32Error_        = 0;

    // Initialize tile regions
    tiles_.setRect(BTILE_CAN_STATUS,  0, ui::SCREEN_H / 2 + 60, ui::SCREEN_W, 20);
    tiles_.setRect(BTILE_SENSOR,      0, ui::SCREEN_H / 2 + 80, ui::SCREEN_W, 20);
    tiles_.setRect(BTILE_DIAGNOSTICS, 0, DIAG_SEP_Y - 1,
                   ui::SCREEN_W, ui::SCREEN_H - DIAG_SEP_Y + 1);
    tiles_.invalidateAll();
}

void BootScreen::onExit() {}

void BootScreen::update(const vehicle::VehicleData& data) {
    // CAN link is considered active if heartbeat timestamp is recent
    unsigned long now = millis();
    canLinked_ = (data.heartbeat().timestampMs > 0 &&
                  (now - data.heartbeat().timestampMs) < 500);

    // Obstacle sensor status
    obstacle_sensor::Reading sensorReading = obstacle_sensor::getReading();
    sensorStatus_      = sensorReading.status;
    sensorDistanceMm_  = sensorReading.distance_mm;

    // ---- CAN diagnostics ----

    // TWAI bus status (read-only register query, no side effects)
    twai_status_info_t twaiStatus;
    if (twai_get_status_info(&twaiStatus) == ESP_OK) {
        if (static_cast<uint8_t>(twaiStatus.state) != diagBusState_ ||
            twaiStatus.tx_error_counter != diagTxErr_ ||
            twaiStatus.rx_error_counter != diagRxErr_ ||
            twaiStatus.tx_failed_count  != diagTxFail_ ||
            twaiStatus.rx_missed_count  != diagRxMiss_ ||
            twaiStatus.bus_error_count  != diagBusErr_) {
            diagBusState_ = static_cast<uint8_t>(twaiStatus.state);
            diagTxErr_    = twaiStatus.tx_error_counter;
            diagRxErr_    = twaiStatus.rx_error_counter;
            diagTxFail_   = twaiStatus.tx_failed_count;
            diagRxMiss_   = twaiStatus.rx_missed_count;
            diagBusErr_   = twaiStatus.bus_error_count;
            diagNeedsRedraw_ = true;
        }
    }

    // RX frame flags — which STM32 frame types have been received recently
    uint16_t rxFlags = 0;
    if (data.heartbeat().timestampMs > 0 && (now - data.heartbeat().timestampMs) < DIAG_RX_RECENT_MS)
        rxFlags |= DIAG_RX_HB;
    if (data.speed().timestampMs > 0 && (now - data.speed().timestampMs) < DIAG_RX_RECENT_MS)
        rxFlags |= DIAG_RX_SPD;
    if (data.current().timestampMs > 0 && (now - data.current().timestampMs) < DIAG_RX_RECENT_MS)
        rxFlags |= DIAG_RX_CUR;
    if (data.safety().timestampMs > 0 && (now - data.safety().timestampMs) < DIAG_RX_RECENT_MS)
        rxFlags |= DIAG_RX_SAF;
    if (data.steering().timestampMs > 0 && (now - data.steering().timestampMs) < DIAG_RX_RECENT_MS)
        rxFlags |= DIAG_RX_STR;
    if (data.traction().timestampMs > 0 && (now - data.traction().timestampMs) < DIAG_RX_RECENT_MS)
        rxFlags |= DIAG_RX_TRC;
    if (data.battery().timestampMs > 0 && (now - data.battery().timestampMs) < DIAG_RX_RECENT_MS)
        rxFlags |= DIAG_RX_BAT;

    if (rxFlags != diagRxFlags_) {
        diagRxFlags_ = rxFlags;
        diagNeedsRedraw_ = true;
    }

    // Obstacle sensor active? (affects TX line display)
    bool obsActive = (sensorStatus_ != obstacle_sensor::SensorStatus::WAITING);
    if (obsActive != diagObsActive_) {
        diagObsActive_ = obsActive;
        diagNeedsRedraw_ = true;
    }

    // ---- STM32 heartbeat details (freeze detection + status) ----
    const auto& hb = data.heartbeat();
    bool hbValid = (hb.timestampMs > 0 && (now - hb.timestampMs) < DIAG_RX_RECENT_MS);

    // Track alive counter changes for freeze detection
    if (hbValid && hb.aliveCounter != diagStm32PrevAlive_) {
        diagStm32PrevAlive_    = hb.aliveCounter;
        diagStm32AliveChangedMs_ = now;
    }
    bool frozen = hbValid && (diagStm32AliveChangedMs_ > 0) &&
                  (now - diagStm32AliveChangedMs_) > DIAG_FREEZE_MS;

    if (hbValid != diagStm32HbValid_ || frozen != diagStm32Frozen_ ||
        hb.aliveCounter != diagStm32Alive_ ||
        static_cast<uint8_t>(hb.systemState) != diagStm32State_ ||
        hb.faultFlags != diagStm32Faults_ || hb.errorCode != diagStm32Error_) {
        diagStm32HbValid_ = hbValid;
        diagStm32Frozen_  = frozen;
        diagStm32Alive_   = hb.aliveCounter;
        diagStm32State_   = static_cast<uint8_t>(hb.systemState);
        diagStm32Faults_  = hb.faultFlags;
        diagStm32Error_   = hb.errorCode;
        diagNeedsRedraw_  = true;
    }

    // ---- Compute tile hashes ----
    tiles_.updateHash(BTILE_CAN_STATUS, ui::tileHashVal(canLinked_));
    {
        ui::TileHash sh = ui::tileHashVal(sensorStatus_);
        sh = ui::tileHashFeed(sh, sensorDistanceMm_);
        tiles_.updateHash(BTILE_SENSOR, sh);
    }
    // Diagnostics tile uses diagNeedsRedraw_ as event-driven dirty flag
    if (diagNeedsRedraw_) {
        tiles_.markDirty(BTILE_DIAGNOSTICS);
    }
}

void BootScreen::draw() {
    if (needsRedraw_) {
        needsRedraw_ = false;

        RTRACE_SET_LAYER(0);
        tft.fillScreen(ui::COL_BG);
        RTRACE_FILL_SCREEN(ui::COL_BG);

        RTRACE_SET_LAYER(1);

        // Title
        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        tft.setTextSize(3);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("COCHE", ui::SCREEN_W / 2, ui::SCREEN_H / 2 - 40);
        RTRACE_TEXT(ui::SCREEN_W / 2, ui::SCREEN_H / 2 - 40, "COCHE",
                    ui::COL_WHITE, ui::COL_BG, 3, MC_DATUM);
        tft.drawString("MARCOS", ui::SCREEN_W / 2, ui::SCREEN_H / 2);
        RTRACE_TEXT(ui::SCREEN_W / 2, ui::SCREEN_H / 2, "MARCOS",
                    ui::COL_WHITE, ui::COL_BG, 3, MC_DATUM);

        // Subtitle
        tft.setTextSize(1);
        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.drawString("HMI v1.0", ui::SCREEN_W / 2, ui::SCREEN_H / 2 + 40);
        RTRACE_TEXT(ui::SCREEN_W / 2, ui::SCREEN_H / 2 + 40, "HMI v1.0",
                    ui::COL_GRAY, ui::COL_BG, 1, MC_DATUM);

        prevCanLinked_ = !canLinked_;  // Force status redraw
        prevSensorStatus_ = (sensorStatus_ == obstacle_sensor::SensorStatus::WAITING)
                            ? obstacle_sensor::SensorStatus::VALID
                            : obstacle_sensor::SensorStatus::WAITING;  // Force redraw
        prevSensorDistanceMm_ = ~sensorDistanceMm_;  // Force distance redraw (always differs)
        diagNeedsRedraw_ = true;  // Force diagnostic redraw

        tiles_.markAllDirty();
    }

    RTRACE_SET_LAYER(2);

    // ---- TILE: CAN link status ----
    if (tiles_.isDirty(BTILE_CAN_STATUS)) {
        prevCanLinked_ = canLinked_;

        int16_t statusY = ui::SCREEN_H / 2 + 70;

        tft.setTextSize(1);
        tft.setTextDatum(MC_DATUM);
        tft.setTextPadding(ui::cfg::PAD_BOOT_CAN_STATUS);
        if (canLinked_) {
            tft.setTextColor(ui::COL_GREEN, ui::COL_BG);
            tft.drawString("CAN: LINKED", ui::SCREEN_W / 2, statusY);
            RTRACE_TEXT(ui::SCREEN_W / 2, statusY, "CAN: LINKED",
                        ui::COL_GREEN, ui::COL_BG, 1, MC_DATUM);
        } else {
            tft.setTextColor(ui::COL_RED, ui::COL_BG);
            tft.drawString("CAN: WAITING...", ui::SCREEN_W / 2, statusY);
            RTRACE_TEXT(ui::SCREEN_W / 2, statusY, "CAN: WAITING...",
                        ui::COL_RED, ui::COL_BG, 1, MC_DATUM);
        }
        tft.setTextPadding(0);
        tft.setTextDatum(TL_DATUM);
        tiles_.markClean(BTILE_CAN_STATUS);
    }

    // ---- TILE: Obstacle sensor status ----
    if (tiles_.isDirty(BTILE_SENSOR)) {
        hmi::ObstacleIndicator::draw(tft, sensorStatus_, prevSensorStatus_,
                                     sensorDistanceMm_, prevSensorDistanceMm_);
        prevSensorStatus_      = sensorStatus_;
        prevSensorDistanceMm_  = sensorDistanceMm_;
        tiles_.markClean(BTILE_SENSOR);
    }

    // ---- TILE: CAN diagnostics panel ----
    if (tiles_.isDirty(BTILE_DIAGNOSTICS)) {
        diagNeedsRedraw_ = false;

        RTRACE_SET_LAYER(2);

        // Clear diagnostic area
        tft.fillRect(0, DIAG_SEP_Y - 1, ui::SCREEN_W,
                     ui::SCREEN_H - DIAG_SEP_Y + 1, ui::COL_BG);
        RTRACE_FILL_RECT(0, DIAG_SEP_Y - 1, ui::SCREEN_W,
                         ui::SCREEN_H - DIAG_SEP_Y + 1, ui::COL_BG);

        // Thin separator line
        tft.drawFastHLine(40, DIAG_SEP_Y, ui::SCREEN_W - 80, ui::COL_DARK_GRAY);
        RTRACE_LINE(40, DIAG_SEP_Y, ui::SCREEN_W - 40, DIAG_SEP_Y, ui::COL_DARK_GRAY);

        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);

        // Line 1: ESP32 TWAI bus status + error counters
        {
            const char* stateStr = "UNKNOWN";
            uint16_t stateCol = ui::COL_YELLOW;
            switch (diagBusState_) {
                case TWAI_STATE_STOPPED:    stateStr = "STOPPED";    stateCol = ui::COL_RED;    break;
                case TWAI_STATE_RUNNING:    stateStr = "RUNNING";    stateCol = ui::COL_GREEN;  break;
                case TWAI_STATE_BUS_OFF:    stateStr = "BUS_OFF";    stateCol = ui::COL_RED;    break;
                case TWAI_STATE_RECOVERING: stateStr = "RECOVERING"; stateCol = ui::COL_YELLOW; break;
                default: break;
            }
            char buf[DIAG_BUF_SIZE];
            snprintf(buf, sizeof(buf), "ESP32: %s  TxE:%lu RxE:%lu",
                     stateStr,
                     (unsigned long)diagTxErr_,
                     (unsigned long)diagRxErr_);
            tft.setTextColor(stateCol, ui::COL_BG);
            tft.drawString(buf, DIAG_MARGIN_X, DIAG_LINE1_Y);
            RTRACE_TEXT(DIAG_MARGIN_X, DIAG_LINE1_Y, buf,
                        stateCol, ui::COL_BG, 1, TL_DATUM);
        }

        // Line 2: STM32 heartbeat status (alive counter, state, faults, error)
        {
            char buf[DIAG_BUF_SIZE];
            uint16_t stm32Col;
            if (!diagStm32HbValid_) {
                snprintf(buf, sizeof(buf), "STM32: NO HEARTBEAT");
                stm32Col = ui::COL_RED;
            } else if (diagStm32Frozen_) {
                snprintf(buf, sizeof(buf), "STM32: FROZEN cnt:%u St:%s F:%02X E:%02X",
                         diagStm32Alive_, stm32StateStr(diagStm32State_),
                         diagStm32Faults_, diagStm32Error_);
                stm32Col = ui::COL_ORANGE;
            } else {
                snprintf(buf, sizeof(buf), "STM32: OK cnt:%u St:%s F:%02X E:%02X",
                         diagStm32Alive_, stm32StateStr(diagStm32State_),
                         diagStm32Faults_, diagStm32Error_);
                stm32Col = (diagStm32Faults_ > 0) ? ui::COL_YELLOW : ui::COL_GREEN;
            }
            tft.setTextColor(stm32Col, ui::COL_BG);
            tft.drawString(buf, DIAG_MARGIN_X, DIAG_LINE2_Y);
            RTRACE_TEXT(DIAG_MARGIN_X, DIAG_LINE2_Y, buf,
                        stm32Col, ui::COL_BG, 1, TL_DATUM);
        }

        // Line 3: STM32→ESP32 (what STM32 transmits = what ESP32 receives)
        {
            char rxBuf[DIAG_RX_BUF] = "";
            int pos = 0;
            for (const auto& entry : RX_FLAG_TABLE) {
                if (diagRxFlags_ & entry.flag) {
                    int remain = static_cast<int>(sizeof(rxBuf)) - pos;
                    if (remain <= 1) break;
                    pos += snprintf(rxBuf + pos, remain, "%s", entry.label);
                }
            }
            if (pos == 0) snprintf(rxBuf, sizeof(rxBuf), "-- (waiting)");

            char buf[DIAG_BUF_SIZE];
            snprintf(buf, sizeof(buf), "STM32->ESP32: %s", rxBuf);
            uint16_t rxCol = (pos > 0) ? ui::COL_GREEN : ui::COL_YELLOW;
            tft.setTextColor(rxCol, ui::COL_BG);
            tft.drawString(buf, DIAG_MARGIN_X, DIAG_LINE3_Y);
            RTRACE_TEXT(DIAG_MARGIN_X, DIAG_LINE3_Y, buf,
                        rxCol, ui::COL_BG, 1, TL_DATUM);
        }

        // Line 4: Additional error counters
        {
            char buf[DIAG_BUF_SIZE];
            snprintf(buf, sizeof(buf), "Fail:%lu Miss:%lu BErr:%lu",
                     (unsigned long)diagTxFail_,
                     (unsigned long)diagRxMiss_,
                     (unsigned long)diagBusErr_);
            bool hasErrors = (diagTxFail_ > 0 || diagRxMiss_ > 0 || diagBusErr_ > 0);
            uint16_t errCol = hasErrors ? ui::COL_ORANGE : ui::COL_GRAY;
            tft.setTextColor(errCol, ui::COL_BG);
            tft.drawString(buf, DIAG_MARGIN_X, DIAG_LINE4_Y);
            RTRACE_TEXT(DIAG_MARGIN_X, DIAG_LINE4_Y, buf,
                        errCol, ui::COL_BG, 1, TL_DATUM);
        }

        // Line 5: Diagnostic verdict — quick summary of which side has the problem
        {
            const char* verdict;
            uint16_t verdictCol;
            bool espOk = (diagBusState_ == TWAI_STATE_RUNNING);

            if (!espOk) {
                verdict    = ">> ESP32 CAN bus error";
                verdictCol = ui::COL_RED;
            } else if (!diagStm32HbValid_) {
                verdict    = ">> STM32 not responding - check STM32/wiring";
                verdictCol = ui::COL_RED;
            } else if (diagStm32Frozen_) {
                verdict    = ">> STM32 main loop frozen!";
                verdictCol = ui::COL_ORANGE;
            } else if (diagStm32Faults_ > 0) {
                verdict    = ">> STM32 reporting faults";
                verdictCol = ui::COL_YELLOW;
            } else {
                verdict    = ">> Both sides OK";
                verdictCol = ui::COL_GREEN;
            }
            tft.setTextColor(verdictCol, ui::COL_BG);
            tft.drawString(verdict, DIAG_MARGIN_X, DIAG_LINE5_Y);
            RTRACE_TEXT(DIAG_MARGIN_X, DIAG_LINE5_Y, verdict,
                        verdictCol, ui::COL_BG, 1, TL_DATUM);
        }
        tiles_.markClean(BTILE_DIAGNOSTICS);
    }

    RTRACE_DUMP_IF_PENDING();
}
