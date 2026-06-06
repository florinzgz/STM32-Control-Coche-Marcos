// =============================================================================
// ESP32-S3 HMI — Boot Screen Implementation
// =============================================================================

#include "boot_screen.h"
#include "ui/ui_common.h"
#include "ui/ui_config.h"
#include "ui/render_trace.h"
#include "ui/runtime_monitor.h"
#include "hmi/obstacle_indicator.h"
#include "sensors/obstacle_sensor.h"
#include <TFT_eSPI.h>
#include <driver/twai.h>
#include <Arduino.h>
#include <cmath>
#include <cstring>

extern TFT_eSPI tft;

// RX frame flag bits — which STM32 frame types have been received recently
static constexpr uint16_t DIAG_RX_HB   = (1 << 0);
static constexpr uint16_t DIAG_RX_SPD  = (1 << 1);
static constexpr uint16_t DIAG_RX_CUR  = (1 << 2);
static constexpr uint16_t DIAG_RX_SAF  = (1 << 3);
static constexpr uint16_t DIAG_RX_STR  = (1 << 4);
static constexpr uint16_t DIAG_RX_TRC  = (1 << 5);
static constexpr uint16_t DIAG_RX_BAT  = (1 << 6);

// Diagnostic area layout (below obstacle indicator) — from ui_config.h
static constexpr int16_t DIAG_SEP_Y    = ui::cfg::BTILE_DIAG_SEP_Y;
static constexpr int16_t DIAG_LINE_H   = ui::cfg::BTILE_DIAG_LINE_H;
static constexpr int16_t DIAG_LINE1_Y  = DIAG_SEP_Y + 4;          // ESP32 bus status
static constexpr int16_t DIAG_LINE2_Y  = DIAG_LINE1_Y + DIAG_LINE_H;  // STM32 heartbeat
static constexpr int16_t DIAG_LINE3_Y  = DIAG_LINE2_Y + DIAG_LINE_H;  // RX frame flags
static constexpr int16_t DIAG_LINE4_Y  = DIAG_LINE3_Y + DIAG_LINE_H;  // error counts
static constexpr int16_t DIAG_LINE5_Y  = DIAG_LINE4_Y + DIAG_LINE_H;  // diagnostic verdict
static constexpr int16_t DIAG_MARGIN_X = ui::cfg::BTILE_DIAG_MARGIN_X;

// Timeout for considering a frame "recently received"
static constexpr unsigned long DIAG_RX_RECENT_MS = ui::cfg::BTILE_DIAG_RX_RECENT_MS;

// Text buffer sizes (sized for worst-case snprintf output)
static constexpr int DIAG_RX_BUF    = 40;   // "HB SPD CUR SAF STR TRC BAT " (28) + margin

// Freeze detection: if alive counter hasn't changed for configured duration, STM32 main loop is stuck
static constexpr unsigned long DIAG_FREEZE_MS = ui::cfg::BTILE_DIAG_FREEZE_MS;
static constexpr unsigned long DIAG_UPDATE_MS = ui::cfg::BTILE_DIAG_UPDATE_MS;

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

static uint16_t blend565(uint16_t a, uint16_t b, uint8_t t) {
    uint8_t ar = (a >> 11) & 0x1F;
    uint8_t ag = (a >> 5) & 0x3F;
    uint8_t ab = a & 0x1F;
    uint8_t br = (b >> 11) & 0x1F;
    uint8_t bg = (b >> 5) & 0x3F;
    uint8_t bb = b & 0x1F;
    uint8_t r = static_cast<uint8_t>((ar * (255 - t) + br * t) / 255);
    uint8_t g = static_cast<uint8_t>((ag * (255 - t) + bg * t) / 255);
    uint8_t bl = static_cast<uint8_t>((ab * (255 - t) + bb * t) / 255);
    return static_cast<uint16_t>((r << 11) | (g << 5) | bl);
}

static void drawMercedesEmblem(int16_t cx, int16_t cy, int16_t radius) {
    constexpr uint16_t COL_CHROME_DARK  = 0x7BEF;
    constexpr uint16_t COL_CHROME_MID   = 0xBDF7;
    constexpr uint16_t COL_CHROME_LIGHT = 0xE71C;
    constexpr uint16_t COL_SHADOW       = 0x2104;

    // Soft drop shadow for depth
    tft.fillCircle(cx + 2, cy + 2, radius + 1, COL_SHADOW);

    // Metallic ring (radial gradient)
    for (int16_t r = radius; r >= radius - 5; --r) {
        uint8_t t = static_cast<uint8_t>((radius - r) * 255 / 5);
        tft.drawCircle(cx, cy, r, blend565(COL_CHROME_LIGHT, COL_CHROME_DARK, t));
    }
    for (int16_t r = radius - 6; r >= radius - 9; --r) {
        uint8_t t = static_cast<uint8_t>((radius - 6 - r) * 255 / 3);
        tft.drawCircle(cx, cy, r, blend565(COL_CHROME_DARK, COL_CHROME_MID, t));
    }

    // Three-pointed star spokes
    constexpr float kDegToRad = 3.14159265f / 180.0f;
    constexpr int angles[3] = {-90, 30, 150};
    for (int i = 0; i < 3; ++i) {
        float ang = angles[i] * kDegToRad;
        int16_t ex = cx + static_cast<int16_t>(std::cos(ang) * (radius - 10));
        int16_t ey = cy + static_cast<int16_t>(std::sin(ang) * (radius - 10));
        float nx = -std::sin(ang);
        float ny =  std::cos(ang);
        for (int8_t w = -1; w <= 1; ++w) {
            int16_t ox = static_cast<int16_t>(nx * w);
            int16_t oy = static_cast<int16_t>(ny * w);
            uint16_t col = (w == 0) ? COL_CHROME_LIGHT : COL_CHROME_MID;
            tft.drawLine(cx + ox, cy + oy, ex + ox, ey + oy, col);
        }
    }

    // Hub and highlights
    tft.fillCircle(cx, cy, 5, COL_CHROME_MID);
    tft.fillCircle(cx - 2, cy - 2, 2, COL_CHROME_LIGHT);
    tft.drawCircle(cx, cy, radius - 11, COL_CHROME_MID);
}

static void drawBootStaticSplash() {
    RTRACE_SET_LAYER(1);

    // Main title
    tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
    tft.setTextSize(3);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("COCHE", ui::SCREEN_W / 2, ui::SCREEN_H / 2 - 40);
    RTRACE_TEXT(ui::SCREEN_W / 2, ui::SCREEN_H / 2 - 40, "COCHE",
                ui::COL_WHITE, ui::COL_BG, 3, MC_DATUM);
    tft.drawString("MARCOS", ui::SCREEN_W / 2, ui::SCREEN_H / 2);
    RTRACE_TEXT(ui::SCREEN_W / 2, ui::SCREEN_H / 2, "MARCOS",
                ui::COL_WHITE, ui::COL_BG, 3, MC_DATUM);

    // Decorative frame
    tft.drawRoundRect(130, 96, 220, 86, 10, ui::COL_DARK_GRAY);
    tft.drawRoundRect(132, 98, 216, 82, 10, ui::COL_GRAY);

    // Premium Mercedes-style emblems (vector only)
    const int16_t cy = ui::cfg::BOOT_EMBLEM_CENTER_Y;
    const int16_t cxL = ui::SCREEN_W / 2 - ui::cfg::BOOT_EMBLEM_OFFSET_X;
    const int16_t cxR = ui::SCREEN_W / 2 + ui::cfg::BOOT_EMBLEM_OFFSET_X;
    const int16_t radius = ui::cfg::BOOT_EMBLEM_RADIUS;
    drawMercedesEmblem(cxL, cy, radius);
    drawMercedesEmblem(cxR, cy, radius);

    // Subtitle
    tft.setTextSize(1);
    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    tft.drawString("HMI v1.0", ui::SCREEN_W / 2, ui::SCREEN_H / 2 + 40);
    RTRACE_TEXT(ui::SCREEN_W / 2, ui::SCREEN_H / 2 + 40, "HMI v1.0",
                ui::COL_GRAY, ui::COL_BG, 1, MC_DATUM);

    // Static diagnostic separator
    tft.drawFastHLine(40, DIAG_SEP_Y, ui::SCREEN_W - 80, ui::COL_DARK_GRAY);
    RTRACE_LINE(40, DIAG_SEP_Y, ui::SCREEN_W - 40, DIAG_SEP_Y, ui::COL_DARK_GRAY);
}

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
    diagBusState_    = 0xFF;
    diagTxErr_       = 0;
    diagRxErr_       = 0;
    diagTxFail_      = 0;
    diagRxMiss_      = 0;
    diagBusErr_      = 0;
    diagRxFlags_     = 0;

    // Reset STM32 heartbeat diagnostic state
    diagStm32HbValid_      = false;
    diagStm32Frozen_       = false;
    diagStm32Alive_        = 0;
    diagStm32PrevAlive_    = 0xFF;
    diagStm32AliveChangedMs_ = 0;
    diagStm32State_        = 0xFF;
    diagStm32Faults_       = 0;
    diagStm32Error_        = 0;
    diagNextUpdateMs_      = 0;
    diagDirtyMask_         = 0x1F;
    memset(diagLineText_, 0, sizeof(diagLineText_));
    memset(diagLineColor_, 0, sizeof(diagLineColor_));

    // Initialize tile regions
    tiles_.setRect(BTILE_CAN_STATUS,  0, ui::SCREEN_H / 2 + 60, ui::SCREEN_W, 20);
    tiles_.setRect(BTILE_SENSOR,      0, ui::SCREEN_H / 2 + 80, ui::SCREEN_W, 20);
    tiles_.setRect(BTILE_DIAGNOSTICS, 0, DIAG_SEP_Y - 1,
                   ui::SCREEN_W, ui::SCREEN_H - DIAG_SEP_Y + 1);
    tiles_.invalidateAll();
}

void BootScreen::onExit() {}

void BootScreen::update(const vehicle::VehicleData& data, unsigned long frameTimeMs) {
    // CAN link is considered active if heartbeat timestamp is recent
    canLinked_ = (data.heartbeat().timestampMs > 0 &&
                  (frameTimeMs - data.heartbeat().timestampMs) < 500);

    // Obstacle sensor status
    obstacle_sensor::Reading sensorReading = obstacle_sensor::getReading();
    sensorStatus_      = sensorReading.status;
    sensorDistanceMm_  = sensorReading.distance_mm;

    // ---- Throttled diagnostics (8 Hz) ----
    bool sampleDiag = (diagNextUpdateMs_ == 0) ||
                      ((frameTimeMs - diagNextUpdateMs_) < 0x80000000UL);
    if (sampleDiag) {
        diagNextUpdateMs_ = frameTimeMs + DIAG_UPDATE_MS;

        // TWAI bus status (read-only register query, no side effects)
        twai_status_info_t twaiStatus;
        if (twai_get_status_info(&twaiStatus) == ESP_OK) {
            diagBusState_ = static_cast<uint8_t>(twaiStatus.state);
            diagTxErr_    = twaiStatus.tx_error_counter;
            diagRxErr_    = twaiStatus.rx_error_counter;
            diagTxFail_   = twaiStatus.tx_failed_count;
            diagRxMiss_   = twaiStatus.rx_missed_count;
            diagBusErr_   = twaiStatus.bus_error_count;
        }

        // RX frame flags — which STM32 frame types have been received recently
        uint16_t rxFlags = 0;
        if (data.heartbeat().timestampMs > 0 && (frameTimeMs - data.heartbeat().timestampMs) < DIAG_RX_RECENT_MS)
            rxFlags |= DIAG_RX_HB;
        if (data.speed().timestampMs > 0 && (frameTimeMs - data.speed().timestampMs) < DIAG_RX_RECENT_MS)
            rxFlags |= DIAG_RX_SPD;
        if (data.current().timestampMs > 0 && (frameTimeMs - data.current().timestampMs) < DIAG_RX_RECENT_MS)
            rxFlags |= DIAG_RX_CUR;
        if (data.safety().timestampMs > 0 && (frameTimeMs - data.safety().timestampMs) < DIAG_RX_RECENT_MS)
            rxFlags |= DIAG_RX_SAF;
        if (data.steering().timestampMs > 0 && (frameTimeMs - data.steering().timestampMs) < DIAG_RX_RECENT_MS)
            rxFlags |= DIAG_RX_STR;
        if (data.traction().timestampMs > 0 && (frameTimeMs - data.traction().timestampMs) < DIAG_RX_RECENT_MS)
            rxFlags |= DIAG_RX_TRC;
        if (data.battery().timestampMs > 0 && (frameTimeMs - data.battery().timestampMs) < DIAG_RX_RECENT_MS)
            rxFlags |= DIAG_RX_BAT;
        diagRxFlags_ = rxFlags;

        // STM32 heartbeat details (freeze detection + status)
        const auto& hb = data.heartbeat();
        bool hbValid = (hb.timestampMs > 0 && (frameTimeMs - hb.timestampMs) < DIAG_RX_RECENT_MS);
        if (hbValid && hb.aliveCounter != diagStm32PrevAlive_) {
            diagStm32PrevAlive_ = hb.aliveCounter;
            diagStm32AliveChangedMs_ = frameTimeMs;
        }
        bool frozen = hbValid && (diagStm32AliveChangedMs_ > 0) &&
                      (frameTimeMs - diagStm32AliveChangedMs_) > DIAG_FREEZE_MS;
        diagStm32HbValid_ = hbValid;
        diagStm32Frozen_  = frozen;
        diagStm32Alive_   = hb.aliveCounter;
        diagStm32State_   = static_cast<uint8_t>(hb.systemState);
        diagStm32Faults_  = hb.faultFlags;
        diagStm32Error_   = hb.errorCode;

        // Compose diagnostics lines and mark changed lines only
        char nextLines[DIAG_LINE_COUNT][DIAG_TEXT_MAX] = {};
        uint16_t nextColors[DIAG_LINE_COUNT] = {};

        const char* stateStr = "UNKNOWN";
        uint16_t stateCol = ui::COL_YELLOW;
        switch (diagBusState_) {
            case TWAI_STATE_STOPPED:    stateStr = "STOPPED";    stateCol = ui::COL_RED;    break;
            case TWAI_STATE_RUNNING:    stateStr = "RUNNING";    stateCol = ui::COL_GREEN;  break;
            case TWAI_STATE_BUS_OFF:    stateStr = "BUS_OFF";    stateCol = ui::COL_RED;    break;
            case TWAI_STATE_RECOVERING: stateStr = "RECOVERING"; stateCol = ui::COL_YELLOW; break;
            default: break;
        }
        snprintf(nextLines[0], DIAG_TEXT_MAX, "ESP32: %s TxE:%lu RxE:%lu",
                 stateStr, (unsigned long)diagTxErr_, (unsigned long)diagRxErr_);
        nextColors[0] = stateCol;

        if (!diagStm32HbValid_) {
            snprintf(nextLines[1], DIAG_TEXT_MAX, "STM32: NO HEARTBEAT");
            nextColors[1] = ui::COL_RED;
        } else if (diagStm32Frozen_) {
            snprintf(nextLines[1], DIAG_TEXT_MAX, "STM32: FROZEN cnt:%u St:%s F:%02X E:%02X",
                     diagStm32Alive_, stm32StateStr(diagStm32State_),
                     diagStm32Faults_, diagStm32Error_);
            nextColors[1] = ui::COL_ORANGE;
        } else {
            snprintf(nextLines[1], DIAG_TEXT_MAX, "STM32: OK cnt:%u St:%s F:%02X E:%02X",
                     diagStm32Alive_, stm32StateStr(diagStm32State_),
                     diagStm32Faults_, diagStm32Error_);
            nextColors[1] = (diagStm32Faults_ > 0) ? ui::COL_YELLOW : ui::COL_GREEN;
        }

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
        snprintf(nextLines[2], DIAG_TEXT_MAX, "STM32->ESP32: %s", rxBuf);
        nextColors[2] = (pos > 0) ? ui::COL_GREEN : ui::COL_YELLOW;

        snprintf(nextLines[3], DIAG_TEXT_MAX, "Fail:%lu Miss:%lu BErr:%lu",
                 (unsigned long)diagTxFail_,
                 (unsigned long)diagRxMiss_,
                 (unsigned long)diagBusErr_);
        nextColors[3] = (diagTxFail_ > 0 || diagRxMiss_ > 0 || diagBusErr_ > 0)
                        ? ui::COL_ORANGE : ui::COL_GRAY;

        const char* verdict = ">> Both sides OK";
        uint16_t verdictCol = ui::COL_GREEN;
        bool espOk = (diagBusState_ == TWAI_STATE_RUNNING);
        if (!espOk) {
            verdict = ">> ESP32 CAN bus error";
            verdictCol = ui::COL_RED;
        } else if (!diagStm32HbValid_) {
            verdict = ">> STM32 not responding";
            verdictCol = ui::COL_RED;
        } else if (diagStm32Frozen_) {
            verdict = ">> STM32 main loop frozen!";
            verdictCol = ui::COL_ORANGE;
        } else if (diagStm32Faults_ > 0) {
            verdict = ">> STM32 reporting faults";
            verdictCol = ui::COL_YELLOW;
        }
        auto stats = rtmon::RuntimeMonitor::getStats();
        unsigned long ramKb = ESP.getFreeHeap() / 1024UL;
        unsigned long psramKb = ESP.getFreePsram() / 1024UL;
        snprintf(nextLines[4], DIAG_TEXT_MAX, "FPS:%u RAM:%luk PS:%luk %s",
                 static_cast<unsigned>(stats.fps),
                 ramKb, psramKb, verdict);
        nextColors[4] = verdictCol;

        for (uint8_t i = 0; i < DIAG_LINE_COUNT; ++i) {
            if (strcmp(nextLines[i], diagLineText_[i]) != 0 || nextColors[i] != diagLineColor_[i]) {
                strncpy(diagLineText_[i], nextLines[i], DIAG_TEXT_MAX - 1);
                diagLineText_[i][DIAG_TEXT_MAX - 1] = '\0';
                diagLineColor_[i] = nextColors[i];
                diagDirtyMask_ |= static_cast<uint8_t>(1U << i);
            }
        }
    }

    // ---- Compute tile hashes ----
    tiles_.updateHash(BTILE_CAN_STATUS, ui::tileHashVal(canLinked_));
    {
        ui::TileHash sh = ui::tileHashVal(sensorStatus_);
        sh = ui::tileHashFeed(sh, sensorDistanceMm_);
        tiles_.updateHash(BTILE_SENSOR, sh);
    }
    // Diagnostics tile uses per-line dirty bitmask
    if (diagDirtyMask_ != 0) {
        tiles_.markDirty(BTILE_DIAGNOSTICS);
    }
}

void BootScreen::draw() {
    if (needsRedraw_) {
        needsRedraw_ = false;

        RTRACE_SET_LAYER(0);
        tft.fillScreen(ui::COL_BG);
        RTRACE_FILL_SCREEN(ui::COL_BG);

        drawBootStaticSplash();

        prevCanLinked_ = !canLinked_;  // Force status redraw
        prevSensorStatus_ = (sensorStatus_ == obstacle_sensor::SensorStatus::WAITING)
                            ? obstacle_sensor::SensorStatus::VALID
                            : obstacle_sensor::SensorStatus::WAITING;  // Force redraw
        prevSensorDistanceMm_ = ~sensorDistanceMm_;  // Force distance redraw (always differs)
        diagDirtyMask_ = 0x1F;  // Force all diagnostic lines redraw

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
        RTRACE_SET_LAYER(2);

        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);
        static constexpr int16_t kDiagLineY[DIAG_LINE_COUNT] = {
            DIAG_LINE1_Y, DIAG_LINE2_Y, DIAG_LINE3_Y, DIAG_LINE4_Y, DIAG_LINE5_Y
        };
        constexpr int16_t clearX = DIAG_MARGIN_X - 2;
        constexpr int16_t clearW = ui::SCREEN_W - (clearX * 2);
        for (uint8_t i = 0; i < DIAG_LINE_COUNT; ++i) {
            uint8_t bit = static_cast<uint8_t>(1U << i);
            if ((diagDirtyMask_ & bit) == 0) continue;

            const int16_t y = kDiagLineY[i];
            tft.fillRect(clearX, y - 1, clearW, DIAG_LINE_H, ui::COL_BG);
            RTRACE_FILL_RECT(clearX, y - 1, clearW, DIAG_LINE_H, ui::COL_BG);

            tft.setTextColor(diagLineColor_[i], ui::COL_BG);
            tft.drawString(diagLineText_[i], DIAG_MARGIN_X, y);
            RTRACE_TEXT(DIAG_MARGIN_X, y, diagLineText_[i],
                        diagLineColor_[i], ui::COL_BG, 1, TL_DATUM);

        }
        tiles_.markClean(BTILE_DIAGNOSTICS);
        diagDirtyMask_ = 0;
    }

    RTRACE_DUMP_IF_PENDING();
}
