// =============================================================================
// ESP32-S3 HMI — Render Trace
//
// Instruments the display layer and outputs a deterministic UI layout trace
// as JSON via Serial for every screen (boot, drive, standby, safe, error,
// engineering/calibration).
//
// For each TFT draw call captured:
//   type, x, y, w, h, color, bg_color, font, text, alignment, layer,
//   screen_name, dynamic/static, refresh_rate_ms
//
// Screen-level metadata included in each dump:
//   screen_width, screen_height, rotation, spi_speed, sprite_usage
//
// Draw order matches the real code execution order.
//
// Enable  : #define RENDER_TRACE 1  (default)
// Disable : #define RENDER_TRACE 0  or  -DRENDER_TRACE=0 in platformio.ini
//
// Usage:
//   1. Call RTRACE_BEGIN_SCREEN("name") from each screen's onEnter().
//   2. Set RTRACE_SET_LAYER(0/1/2) around draw code (0=bg, 1=static, 2=dynamic).
//   3. Add RTRACE_* macros alongside every tft.drawXxx() / tft.fillXxx() call.
//   4. Call RTRACE_DUMP_IF_PENDING() at the end of each screen's draw().
//      → Outputs JSON once after the first frame of each screen entry.
//
// Reference: docs/HMI_RENDERING_STRATEGY.md
// =============================================================================

#ifndef RENDER_TRACE_H
#define RENDER_TRACE_H

// Master switch — defaults to enabled, matches RUNTIME_MONITOR convention
#ifndef RENDER_TRACE
#define RENDER_TRACE 1
#endif

#include <cstdint>
#include <cstring>

#if RENDER_TRACE

#include <Arduino.h>

namespace trace {

// -------------------------------------------------------------------------
// Hardware constants (must match platformio.ini / ui_common.h)
// -------------------------------------------------------------------------
inline constexpr uint32_t TRACE_SPI_SPEED = 40000000;  // SPI_FREQUENCY
inline constexpr uint8_t  TRACE_ROTATION  = 1;          // landscape
inline constexpr int16_t  TRACE_SCREEN_W  = 480;
inline constexpr int16_t  TRACE_SCREEN_H  = 320;

// -------------------------------------------------------------------------
// Draw call type identifiers
// -------------------------------------------------------------------------
enum class DrawType : uint8_t {
    FILL_SCREEN     = 0,
    TEXT            = 1,
    FILL_RECT       = 2,
    DRAW_RECT       = 3,
    FILL_ROUNDRECT  = 4,
    DRAW_ROUNDRECT  = 5,
    LINE            = 6,
    CIRCLE          = 7,
    FILL_CIRCLE     = 8,
    SPRITE          = 9,
    ICON            = 10,
    IMAGE           = 11
};

// -------------------------------------------------------------------------
// Per-draw-call record (fixed-size, stack/BSS allocated, no heap)
// -------------------------------------------------------------------------
inline constexpr uint8_t  TRACE_TEXT_LEN   = 32;   // max text string length
inline constexpr uint8_t  TRACE_SCREEN_LEN = 24;   // max screen name length
// 200 entries × ~75 bytes = ~15 KB — well within ESP32-S3 SRAM (520 KB).
// Covers the most complex screen (drive: ~90 draw calls per full frame).
inline constexpr uint16_t TRACE_MAX        = 200;  // max draw calls per screen

struct DrawEntry {
    DrawType type;
    int16_t  x, y;              // draw origin (or x0,y0 for lines; cx,cy for circles)
    int16_t  w, h;              // size (or x1,y1 for lines; r,0 for circles; 0,0 for text)
    uint16_t color;             // primary draw color (RGB565)
    uint16_t bg_color;          // background color (TEXT only; 0x0000 for shapes)
    uint8_t  font;              // textSize 1–3 for TEXT; 0 for shapes
    uint8_t  alignment;         // TFT datum constant for TEXT; 0 for shapes
    uint8_t  layer;             // 0=background, 1=static, 2=dynamic
    uint8_t  is_dynamic;        // 0=static/one-shot, 1=redrawn each frame
    uint16_t refresh_rate_ms;   // 0=static, 50=20 FPS dynamic
    char     screen_name[TRACE_SCREEN_LEN];
    char     text[TRACE_TEXT_LEN];
};

// -------------------------------------------------------------------------
// RenderTrace — static class, zero heap allocation
//
// Layer semantics:
//   0 — fillScreen (background clear)
//   1 — static elements drawn once per screen entry (onEnter / needsRedraw)
//   2 — dynamic elements redrawn each frame when data changes
// -------------------------------------------------------------------------
class RenderTrace {
public:
    // ----- Context setters -----

    /// Reset accumulator and schedule first-frame JSON dump.
    /// Call from each screen's onEnter().
    static void beginScreen(const char* name);

    /// Set layer for all subsequent draw calls (0/1/2).
    static void setLayer(uint8_t layer);

    // ----- Draw call recorders -----

    /// tft.fillScreen(color)
    static void recFillScreen(uint16_t color);

    /// tft.fillRect() or tft.drawRect()
    static void recRect(bool filled,
                        int16_t x, int16_t y, int16_t w, int16_t h,
                        uint16_t color);

    /// tft.drawLine(x0, y0, x1, y1, color)
    /// Stored as: x=x0, y=y0, w=x1, h=y1
    static void recLine(int16_t x0, int16_t y0,
                        int16_t x1, int16_t y1, uint16_t color);

    /// tft.drawCircle(cx, cy, r, color) — outline only
    /// Stored as: x=cx, y=cy, w=r, h=0
    static void recCircle(int16_t cx, int16_t cy, int16_t r, uint16_t color);

    /// tft.fillCircle(cx, cy, r, color)
    /// Stored as: x=cx, y=cy, w=r, h=0
    static void recFillCircle(int16_t cx, int16_t cy, int16_t r,
                               uint16_t color);

    /// tft.drawString(text, x, y) with active font/datum at time of call.
    /// @param font   textSize value (1–3) active when drawString was called.
    ///              Values outside 1–3 are stored as-is; TFT_eSPI silently
    ///              clamps them when rendering.
    /// @param datum  TFT datum constant active when drawString was called
    static void recText(int16_t x, int16_t y, const char* text,
                        uint16_t color, uint16_t bgColor,
                        uint8_t font, uint8_t datum);

    // ----- Output -----

    /// Dump JSON to Serial if a dump is pending (set by beginScreen()).
    /// Clears the pending flag after dump. Call at end of each draw().
    static void dumpIfPending();

private:
    static DrawEntry  buf_[TRACE_MAX];
    static uint16_t   cnt_;
    static uint8_t    layer_;
    static bool       dumpPending_;
    static char       screen_[TRACE_SCREEN_LEN];

    static void push(const DrawEntry& e);

    static const char* typeName(DrawType t);
    static const char* datumName(uint8_t d);
    static void        printHex(uint16_t v);
    static void        dumpJSON();
};

}  // namespace trace

// -------------------------------------------------------------------------
// Convenience macros — compile to nothing when RENDER_TRACE == 0
// -------------------------------------------------------------------------
#define RTRACE_BEGIN_SCREEN(n)          trace::RenderTrace::beginScreen(n)
#define RTRACE_SET_LAYER(l)             trace::RenderTrace::setLayer(l)
#define RTRACE_FILL_SCREEN(c)           trace::RenderTrace::recFillScreen(c)
#define RTRACE_FILL_RECT(x,y,w,h,c)    trace::RenderTrace::recRect(true,(x),(y),(w),(h),(c))
#define RTRACE_DRAW_RECT(x,y,w,h,c)    trace::RenderTrace::recRect(false,(x),(y),(w),(h),(c))
#define RTRACE_LINE(x0,y0,x1,y1,c)     trace::RenderTrace::recLine((x0),(y0),(x1),(y1),(c))
#define RTRACE_CIRCLE(cx,cy,r,c)        trace::RenderTrace::recCircle((cx),(cy),(r),(c))
#define RTRACE_FILL_CIRCLE(cx,cy,r,c)  trace::RenderTrace::recFillCircle((cx),(cy),(r),(c))
#define RTRACE_TEXT(x,y,t,c,b,f,d)     trace::RenderTrace::recText((x),(y),(t),(c),(b),(f),(d))
#define RTRACE_DUMP_IF_PENDING()        trace::RenderTrace::dumpIfPending()

#else  // RENDER_TRACE == 0

#define RTRACE_BEGIN_SCREEN(n)          ((void)0)
#define RTRACE_SET_LAYER(l)             ((void)0)
#define RTRACE_FILL_SCREEN(c)           ((void)0)
#define RTRACE_FILL_RECT(x,y,w,h,c)    ((void)0)
#define RTRACE_DRAW_RECT(x,y,w,h,c)    ((void)0)
#define RTRACE_LINE(x0,y0,x1,y1,c)     ((void)0)
#define RTRACE_CIRCLE(cx,cy,r,c)        ((void)0)
#define RTRACE_FILL_CIRCLE(cx,cy,r,c)  ((void)0)
#define RTRACE_TEXT(x,y,t,c,b,f,d)     ((void)0)
#define RTRACE_DUMP_IF_PENDING()        ((void)0)

#endif  // RENDER_TRACE

#endif  // RENDER_TRACE_H
