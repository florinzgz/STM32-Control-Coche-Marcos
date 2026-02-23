// =============================================================================
// ESP32-S3 HMI — Render Trace Implementation
//
// Accumulates TFT draw calls and dumps them as JSON to Serial.
// Zero heap allocation — all storage is in static BSS arrays.
// JSON is output incrementally via Serial.print() to avoid large buffers.
//
// Output format per screen entry:
//
//   [TRACE:boot]
//   {"meta":{"screen_width":480,"screen_height":320,"rotation":1,
//             "spi_speed":40000000,"sprite_usage":"no"},
//    "screen":"boot","draws":[
//     {"order":0,"type":"fill_screen","x":0,"y":0,"w":480,"h":320,
//      "color":"0x2104","bg_color":"0x0000","font":0,"text":"",
//      "alignment":"TL_DATUM","layer":0,"screen_name":"boot",
//      "dynamic":false,"refresh_ms":0},
//     ...
//   ]}
//   [/TRACE:boot]
//
// =============================================================================

#include "render_trace.h"

#if RENDER_TRACE

#include <cstring>

namespace trace {

// -------------------------------------------------------------------------
// Static member storage
// -------------------------------------------------------------------------
DrawEntry RenderTrace::buf_[TRACE_MAX];
uint16_t  RenderTrace::cnt_         = 0;
uint8_t   RenderTrace::layer_       = 1;
bool      RenderTrace::dumpPending_ = false;
char      RenderTrace::screen_[TRACE_SCREEN_LEN] = "unknown";

// -------------------------------------------------------------------------
// Context setters
// -------------------------------------------------------------------------
void RenderTrace::beginScreen(const char* name) {
    cnt_         = 0;
    // Start at layer 1 (static). Callers use setLayer(0) before fillScreen
    // and setLayer(2) before dynamic partial-update sections.
    layer_       = 1;
    dumpPending_ = true;
    strncpy(screen_, name ? name : "unknown", TRACE_SCREEN_LEN - 1);
    screen_[TRACE_SCREEN_LEN - 1] = '\0';
}

void RenderTrace::setLayer(uint8_t layer) {
    layer_ = layer;
}

// -------------------------------------------------------------------------
// Internal: append entry to buffer (silently drops when full)
// -------------------------------------------------------------------------
void RenderTrace::push(const DrawEntry& e) {
    if (cnt_ >= TRACE_MAX) return;
    buf_[cnt_++] = e;
}

// -------------------------------------------------------------------------
// Draw call recorders
// -------------------------------------------------------------------------
void RenderTrace::recFillScreen(uint16_t color) {
    DrawEntry e{};
    e.type            = DrawType::FILL_SCREEN;
    e.x               = 0;
    e.y               = 0;
    e.w               = TRACE_SCREEN_W;
    e.h               = TRACE_SCREEN_H;
    e.color           = color;
    e.bg_color        = 0x0000;
    e.font            = 0;
    e.alignment       = 0;
    e.layer           = 0;   // always background layer
    e.is_dynamic      = 0;
    e.refresh_rate_ms = 0;
    strncpy(e.screen_name, screen_, TRACE_SCREEN_LEN - 1);
    e.screen_name[TRACE_SCREEN_LEN - 1] = '\0';
    e.text[0] = '\0';
    push(e);
}

void RenderTrace::recRect(bool filled,
                          int16_t x, int16_t y, int16_t w, int16_t h,
                          uint16_t color) {
    DrawEntry e{};
    e.type            = filled ? DrawType::FILL_RECT : DrawType::DRAW_RECT;
    e.x               = x;
    e.y               = y;
    e.w               = w;
    e.h               = h;
    e.color           = color;
    e.bg_color        = 0x0000;
    e.font            = 0;
    e.alignment       = 0;
    e.layer           = layer_;
    e.is_dynamic      = (layer_ == 2) ? 1 : 0;
    e.refresh_rate_ms = (layer_ == 2) ? 50 : 0;
    strncpy(e.screen_name, screen_, TRACE_SCREEN_LEN - 1);
    e.screen_name[TRACE_SCREEN_LEN - 1] = '\0';
    e.text[0] = '\0';
    push(e);
}

void RenderTrace::recLine(int16_t x0, int16_t y0,
                          int16_t x1, int16_t y1, uint16_t color) {
    DrawEntry e{};
    e.type            = DrawType::LINE;
    e.x               = x0;
    e.y               = y0;
    e.w               = x1;   // end-point stored in w/h
    e.h               = y1;
    e.color           = color;
    e.bg_color        = 0x0000;
    e.font            = 0;
    e.alignment       = 0;
    e.layer           = layer_;
    e.is_dynamic      = (layer_ == 2) ? 1 : 0;
    e.refresh_rate_ms = (layer_ == 2) ? 50 : 0;
    strncpy(e.screen_name, screen_, TRACE_SCREEN_LEN - 1);
    e.screen_name[TRACE_SCREEN_LEN - 1] = '\0';
    e.text[0] = '\0';
    push(e);
}

void RenderTrace::recCircle(int16_t cx, int16_t cy, int16_t r,
                             uint16_t color) {
    DrawEntry e{};
    e.type            = DrawType::CIRCLE;
    e.x               = cx;
    e.y               = cy;
    e.w               = r;
    e.h               = 0;
    e.color           = color;
    e.bg_color        = 0x0000;
    e.font            = 0;
    e.alignment       = 0;
    e.layer           = layer_;
    e.is_dynamic      = (layer_ == 2) ? 1 : 0;
    e.refresh_rate_ms = (layer_ == 2) ? 50 : 0;
    strncpy(e.screen_name, screen_, TRACE_SCREEN_LEN - 1);
    e.screen_name[TRACE_SCREEN_LEN - 1] = '\0';
    e.text[0] = '\0';
    push(e);
}

void RenderTrace::recFillCircle(int16_t cx, int16_t cy, int16_t r,
                                 uint16_t color) {
    DrawEntry e{};
    e.type            = DrawType::FILL_CIRCLE;
    e.x               = cx;
    e.y               = cy;
    e.w               = r;
    e.h               = 0;
    e.color           = color;
    e.bg_color        = 0x0000;
    e.font            = 0;
    e.alignment       = 0;
    e.layer           = layer_;
    e.is_dynamic      = (layer_ == 2) ? 1 : 0;
    e.refresh_rate_ms = (layer_ == 2) ? 50 : 0;
    strncpy(e.screen_name, screen_, TRACE_SCREEN_LEN - 1);
    e.screen_name[TRACE_SCREEN_LEN - 1] = '\0';
    e.text[0] = '\0';
    push(e);
}

void RenderTrace::recText(int16_t x, int16_t y, const char* text,
                          uint16_t color, uint16_t bgColor,
                          uint8_t font, uint8_t datum) {
    DrawEntry e{};
    e.type            = DrawType::TEXT;
    e.x               = x;
    e.y               = y;
    e.w               = 0;
    e.h               = 0;
    e.color           = color;
    e.bg_color        = bgColor;
    e.font            = font;
    e.alignment       = datum;
    e.layer           = layer_;
    e.is_dynamic      = (layer_ == 2) ? 1 : 0;
    e.refresh_rate_ms = (layer_ == 2) ? 50 : 0;
    strncpy(e.screen_name, screen_, TRACE_SCREEN_LEN - 1);
    e.screen_name[TRACE_SCREEN_LEN - 1] = '\0';
    strncpy(e.text, text ? text : "", TRACE_TEXT_LEN - 1);
    e.text[TRACE_TEXT_LEN - 1] = '\0';
    push(e);
}

// -------------------------------------------------------------------------
// String helpers for JSON output
// -------------------------------------------------------------------------
const char* RenderTrace::typeName(DrawType t) {
    switch (t) {
        case DrawType::FILL_SCREEN:    return "fill_screen";
        case DrawType::TEXT:           return "text";
        case DrawType::FILL_RECT:      return "fill_rect";
        case DrawType::DRAW_RECT:      return "rect";
        case DrawType::FILL_ROUNDRECT: return "fill_roundrect";
        case DrawType::DRAW_ROUNDRECT: return "roundrect";
        case DrawType::LINE:           return "line";
        case DrawType::CIRCLE:         return "circle";
        case DrawType::FILL_CIRCLE:    return "fill_circle";
        case DrawType::SPRITE:         return "sprite";
        case DrawType::ICON:           return "icon";
        case DrawType::IMAGE:          return "image";
        default:                       return "unknown";
    }
}

const char* RenderTrace::datumName(uint8_t d) {
    switch (d) {
        case 0: return "TL_DATUM";
        case 1: return "TC_DATUM";
        case 2: return "TR_DATUM";
        case 3: return "ML_DATUM";
        case 4: return "MC_DATUM";
        case 5: return "MR_DATUM";
        case 6: return "BL_DATUM";
        case 7: return "BC_DATUM";
        case 8: return "BR_DATUM";
        default: return "TL_DATUM";
    }
}

void RenderTrace::printHex(uint16_t v) {
    static const char hex[] = "0123456789ABCDEF";
    char buf[8];
    buf[0] = '"';
    buf[1] = '0';
    buf[2] = 'x';
    buf[3] = hex[(v >> 12) & 0xF];
    buf[4] = hex[(v >>  8) & 0xF];
    buf[5] = hex[(v >>  4) & 0xF];
    buf[6] = hex[ v        & 0xF];
    buf[7] = '"';
    Serial.write(reinterpret_cast<const uint8_t*>(buf), 8);
}

// -------------------------------------------------------------------------
// JSON dump to Serial
// -------------------------------------------------------------------------
void RenderTrace::dumpJSON() {
    // Framing markers make it easy to extract one screen's trace from a log
    Serial.print("\n[TRACE:");
    Serial.print(screen_);
    Serial.println("]");

    // Opening: metadata + screen name
    Serial.print("{\"meta\":{");
    Serial.print("\"screen_width\":");  Serial.print(TRACE_SCREEN_W);
    Serial.print(",\"screen_height\":"); Serial.print(TRACE_SCREEN_H);
    Serial.print(",\"rotation\":");      Serial.print(TRACE_ROTATION);
    Serial.print(",\"spi_speed\":");     Serial.print(TRACE_SPI_SPEED);
    Serial.print(",\"sprite_usage\":\"no\"");
    Serial.print("},\"screen\":\"");
    Serial.print(screen_);
    Serial.print("\",\"draws\":[");

    for (uint16_t i = 0; i < cnt_; ++i) {
        if (i > 0) {
            Serial.print(',');
        }
        Serial.println();

        const DrawEntry& e = buf_[i];

        Serial.print("{\"order\":");        Serial.print(i);
        Serial.print(",\"type\":\"");       Serial.print(typeName(e.type));
        Serial.print("\",\"x\":");          Serial.print(e.x);
        Serial.print(",\"y\":");            Serial.print(e.y);
        Serial.print(",\"w\":");            Serial.print(e.w);
        Serial.print(",\"h\":");            Serial.print(e.h);
        Serial.print(",\"color\":");        printHex(e.color);
        Serial.print(",\"bg_color\":");     printHex(e.bg_color);
        Serial.print(",\"font\":");         Serial.print(e.font);
        Serial.print(",\"text\":\"");       Serial.print(e.text);
        Serial.print("\",\"alignment\":\""); Serial.print(datumName(e.alignment));
        Serial.print("\",\"layer\":");      Serial.print(e.layer);
        Serial.print(",\"screen_name\":\""); Serial.print(e.screen_name);
        Serial.print("\",\"dynamic\":");    Serial.print(e.is_dynamic ? "true" : "false");
        Serial.print(",\"refresh_ms\":");   Serial.print(e.refresh_rate_ms);
        Serial.print('}');

        // Yield every 16 entries to drain the USB-CDC Serial buffer
        // and prevent watchdog resets during long dumps.
        static constexpr uint16_t YIELD_INTERVAL_MASK = 0x0F;
        if ((i & YIELD_INTERVAL_MASK) == YIELD_INTERVAL_MASK) {
            Serial.flush();
            yield();
        }
    }

    Serial.println();
    Serial.println("]}");
    Serial.print("[/TRACE:");
    Serial.print(screen_);
    Serial.println("]");
    Serial.flush();
}

// -------------------------------------------------------------------------
// Public output trigger
// -------------------------------------------------------------------------
void RenderTrace::dumpIfPending() {
    if (!dumpPending_) return;
    dumpPending_ = false;
    dumpJSON();
}

}  // namespace trace

#endif  // RENDER_TRACE
