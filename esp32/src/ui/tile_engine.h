// =============================================================================
// ESP32-S3 HMI — Tile-Based Dirty Region Engine
//
// Core infrastructure for tile-based rendering. Each screen is divided into
// fixed logical regions (tiles). A tile is only redrawn when its content
// changes, determined by comparing a compact FNV-1a hash of the tile's
// source data against the previously rendered hash.
//
// Design principles:
//   - Zero heap allocation: all tiles are statically allocated
//   - Hash-based comparison: faster than field-by-field comparison for
//     complex tiles (e.g. 4 wheels × torque + temp = 8 bytes → 1 hash)
//   - Tile skip: if hash matches last render, the tile is not touched
//   - Overlay restore: overlay tiles can invalidate underlying tiles
//     when they appear or disappear
//
// PIPELINE (RENDER CONTRACT — ABSOLUTE RULE):
//
//   UPDATE PHASE (screen.update(data, frameTimeMs)):
//     - Compute all derived state (filters, thresholds, hysteresis)
//     - Populate cur_* members and precomputed draw values
//     - Call updateHash() per tile → marks dirty if hash changed
//     - All timing logic MUST use injected frameTimeMs, NOT millis()
//     - NO DRAW CALLS. NO SPI TRANSACTIONS.
//
//   DRAW PHASE (screen.draw()):
//     - Consume ONLY precomputed state from update phase
//     - Render dirty tiles to TFT via SPI
//     - MUST NOT modify derived state or recompute business logic
//     - Event flags (ackIndicatorDirty_, etc.) cleared ONLY after
//       the corresponding tile renders successfully (flag safety §16)
//     - After render: markClean() + copy cur→prev for next frame
//
//   This separation guarantees render is functionally pure w.r.t. frame state.
//   same (VehicleData, frameTimeMs) ⇒ same derived state ALWAYS.
//
// FRAME TIME CONTRACT (V10):
//
//   frameTimeMs is captured ONCE per frame in ScreenManager::update() via
//   millis() and injected into screen.update(data, frameTimeMs).
//
//   Guarantees:
//     1. SINGLE-SAMPLED: frameTimeMs is read exactly once per frame. All
//        timing decisions within the frame use the same value.
//     2. MONOTONIC: millis() on ESP32 is monotonically non-decreasing (wraps
//        at 2^32 ≈ 49.7 days). A debug assertion detects backward jumps.
//     3. OVERFLOW-SAFE: All delta calculations use unsigned subtraction
//        (frameTimeMs - refTimeMs) which is overflow-safe in C/C++ for
//        unsigned types. A wrap produces the correct elapsed time as long as
//        the true delta is less than 2^32 ms (~49.7 days).
//     4. DETERMINISTIC: identical (VehicleData, frameTimeMs) input must
//        produce identical derived state — no hidden time dependencies.
//
// RENDER ATOMICITY CONTRACT (V10):
//
//   The update/draw cycle runs entirely on Core 0 in a single FreeRTOS task
//   with no yield points during execution. This guarantees:
//
//     1. NO PARTIAL STATE: Within a single draw() call, the sequence
//        render → overlay_invalidation → markClean → flag_clear is atomic.
//        No concurrent code can observe intermediate dirty/clean state.
//     2. NO MID-FRAME MUTATION: CAN RX runs on Core 1 and writes to a
//        shared VehicleData behind a mutex. The renderTask latches localVD
//        once per frame BEFORE calling update(). During update()+draw(),
//        the snapshot is frozen — CAN updates do not affect the current frame.
//     3. FLAG SAFETY: Event flags (e.g. ackIndicatorDirty_) are set in
//        update() and cleared in draw() ONLY AFTER the corresponding tile
//        renders AND markClean() completes. A flag is never lost.
//
// Z-ORDER LAYER MODEL (formal overlay compositing):
//
//   Layer 0 — STATIC: background fill, outlines, labels (drawn once in onEnter)
//   Layer 1 — BASE: regular data tiles (speed, battery, wheels, pedal, gear...)
//   Layer 2 — OVERLAY: conditional overlays (ACK, FAULTS, DEGRADED, CAN LOST)
//   Layer 3 — SYSTEM: reserved for future system-level indicators
//
//   Rules:
//     - Overlays may physically overlap base tiles
//     - When an overlay becomes invisible, all overlapped base tiles MUST be
//       marked dirty to restore their content ("no persistent artifact" rule)
//     - Overlay visibility MUST be computed in update(), not draw()
//     - Render order: base tiles first, then overlay tiles
//
// OVERLAY INVALIDATION CONTRACT:
//
//   When an overlay tile transitions visible→invisible:
//     1. The overlay's draw area is cleared to background color
//     2. All base tiles whose rects intersect the overlay rect are markDirty()
//     3. On the next frame, those base tiles repaint their content
//
//   This guarantees zero visual artifacts from overlay lifecycle.
//   Each screen's draw() documents its specific invalidation chain.
//
// HASH FAILSAFE SYSTEM (safety-critical tiles):
//
//   FNV-1a 32-bit has a non-zero collision probability. For safety-critical
//   tiles (SPEED, FAULT, WARNING, BATTERY), a periodic forceRedraw() is
//   triggered every HASH_FAILSAFE_INTERVAL frames (see ui_config.h).
//   This guarantees that even if a hash collision causes a missed update,
//   the tile will be repainted within a bounded time window.
//
//   forceRedraw(idx): zeroes the stored hash → next updateHash() always dirty.
//
// HASH FAILSAFE DISTRIBUTION (V10):
//
//   To avoid SPI spikes (multiple critical tiles redrawing on the same frame),
//   forced redraws are STAGGERED across the failsafe interval:
//     - Each critical tile is assigned a different frame offset within the
//       HASH_FAILSAFE_INTERVAL cycle.
//     - Example (DriveScreen, 4 tiles, interval=100):
//       SPEED at frame 0, FAULTS at 25, DEGRADED at 50, BATTERY at 75.
//     - This distributes the SPI load evenly across time.
//
// CRITICAL TILE POLICY (V10):
//
//   Tiles classified as CRITICAL (SPEED, FAULTS, WARNINGS) receive special
//   treatment under fault conditions:
//     - When curFaultFlags_ != 0, critical tiles are force-redrawn EVERY frame
//       (hash suppression override). This ensures fault-related information
//       is always visually current regardless of hash collisions.
//     - Under normal operation (no faults), the standard staggered failsafe
//       interval applies.
//
// Usage:
//   1. Define an enum for tile indices (e.g. DriveTile::SPEED)
//   2. Instantiate TileSet<N> with the number of tiles
//   3. Initialize tile rects in onEnter()
//   4. In update(), call updateHash() for each tile with new data hash
//   5. In draw(), iterate tiles and only render dirty ones
//
// Reference: docs/HMI_RENDERING_STRATEGY.md
// =============================================================================

#ifndef TILE_ENGINE_H
#define TILE_ENGINE_H

#include <cstdint>
#include <cstddef>
#include <cstring>
#include "ui_common.h"

// Enable debug assertions for tile engine (set to 1 for development builds).
// When enabled, out-of-bounds coordinates or invalid tile indices trigger
// Serial.printf diagnostics instead of silent clamping.
#ifndef UI_TILE_DEBUG
#define UI_TILE_DEBUG 0
#endif

#if UI_TILE_DEBUG
#include <cstdio>
#define TILE_ASSERT(cond, msg, ...) \
    do { if (!(cond)) { Serial.printf("[TILE] ASSERT: " msg "\n", ##__VA_ARGS__); } } while (0)
#else
#define TILE_ASSERT(cond, msg, ...) ((void)0)
#endif

namespace ui {

// -------------------------------------------------------------------------
// Tile Z-order layers — formal compositing hierarchy
//
// Used for documentation and render trace classification.
// The actual render order is determined by the tile iteration order in
// each screen's draw() method, which MUST follow this layering.
// -------------------------------------------------------------------------
enum class TileLayer : uint8_t {
    STATIC  = 0,    // Background, outlines, labels (drawn once in onEnter)
    BASE    = 1,    // Regular data tiles (speed, battery, wheels, etc.)
    OVERLAY = 2,    // Conditional overlays (ACK, FAULTS, DEGRADED)
    SYSTEM  = 3     // Reserved: system-level indicators
};

// -------------------------------------------------------------------------
// Overlay composition mode — defines how an overlay interacts with base tiles
//
// Each overlay tile declares its mode (documented per-screen below).
// The mode determines the invalidation strategy when the overlay disappears.
//
// DriveScreen overlay registry:
//   DTILE_DEGRADED → REPLACE, overlaps DTILE_OBSTACLE
//   DTILE_FAULTS   → REPLACE, overlaps DTILE_MODE_ICONS + DTILE_LED_TOGGLE + DTILE_BATTERY
//   DTILE_ACK      → REPLACE, overlaps DTILE_LED_TOGGLE
//
// ErrorScreen: no overlays (all tiles are base layer, screen is self-contained)
// SafeScreen:  no overlays
// StandbyScreen: no overlays
// BootScreen: no overlays
// -------------------------------------------------------------------------
enum class OverlayMode : uint8_t {
    REPLACE = 0,    // Overlay completely covers base tile region.
                    // On removal: clear overlay area → markDirty all overlapped base tiles.
    MERGE   = 1     // Overlay coexists with base tile (partial overlap / transparency).
                    // On removal: markDirty overlapped base tiles (content preserved).
};

// -------------------------------------------------------------------------
// Tile rectangle — defines a screen region
// -------------------------------------------------------------------------
struct TileRect {
    int16_t x = 0;
    int16_t y = 0;
    int16_t w = 0;
    int16_t h = 0;
};

// -------------------------------------------------------------------------
// Tile hash type — 32-bit FNV-1a
// -------------------------------------------------------------------------
using TileHash = uint32_t;

/// Initial FNV-1a offset basis
inline constexpr TileHash FNV_OFFSET = 2166136261u;

/// FNV-1a prime
inline constexpr TileHash FNV_PRIME  = 16777619u;

/// Compute FNV-1a hash over a byte buffer
inline TileHash tileHash(const void* data, size_t len) {
    TileHash h = FNV_OFFSET;
    const uint8_t* p = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < len; ++i) {
        h ^= p[i];
        h *= FNV_PRIME;
    }
    return h;
}

/// Hash a single POD value
template<typename T>
inline TileHash tileHashVal(const T& v) {
    return tileHash(&v, sizeof(v));
}

/// Combine two hashes (boost-style hash combine)
inline TileHash tileHashCombine(TileHash a, TileHash b) {
    a ^= b + 0x9e3779b9u + (a << 6) + (a >> 2);
    return a;
}

/// Feed a POD value into an existing running hash
template<typename T>
inline TileHash tileHashFeed(TileHash h, const T& v) {
    return tileHashCombine(h, tileHashVal(v));
}

// -------------------------------------------------------------------------
// TileSet — fixed-size collection of tiles with dirty tracking
//
// Template parameter N: number of tiles in this set (compile-time)
// -------------------------------------------------------------------------
template<uint8_t N>
class TileSet {
public:
    /// Initialize all tiles to clean with zero hash
    TileSet() {
        memset(dirty_,  0, sizeof(dirty_));
        memset(hashes_, 0, sizeof(hashes_));
    }

    /// Define the screen region for a tile.
    /// Coordinates are clamped to screen bounds (SCREEN_W × SCREEN_H).
    /// Negative or out-of-bounds values are silently corrected (asserted in debug).
    void setRect(uint8_t idx, int16_t x, int16_t y, int16_t w, int16_t h) {
        if (idx >= N) {
            TILE_ASSERT(false, "setRect: idx=%u >= N=%u", idx, N);
            return;
        }

        // Debug: warn about negative or out-of-bounds inputs before clamping
        TILE_ASSERT(x >= 0 && y >= 0, "setRect[%u]: negative origin (%d,%d)", idx, x, y);
        TILE_ASSERT(w > 0 && h > 0, "setRect[%u]: non-positive size (%d×%d)", idx, w, h);
        TILE_ASSERT(x + w <= SCREEN_W && y + h <= SCREEN_H,
                    "setRect[%u]: exceeds screen (%d+%d > %d || %d+%d > %d)",
                    idx, x, w, SCREEN_W, y, h, SCREEN_H);

        // Clamp origin to screen area
        if (x < 0) { w += x; x = 0; }
        if (y < 0) { h += y; y = 0; }

        // Clamp dimensions to non-negative
        if (w < 0) w = 0;
        if (h < 0) h = 0;

        // Clamp to screen extents
        if (x + w > SCREEN_W) w = SCREEN_W - x;
        if (y + h > SCREEN_H) h = SCREEN_H - y;

        rects_[idx] = {x, y, w, h};
    }

    /// Get the rect for a tile
    const TileRect& rect(uint8_t idx) const {
        return rects_[idx];
    }

    /// Mark a tile as needing redraw
    void markDirty(uint8_t idx) {
        if (idx < N) dirty_[idx] = true;
    }

    /// Mark a tile as clean (after rendering)
    void markClean(uint8_t idx) {
        if (idx < N) dirty_[idx] = false;
    }

    /// Mark all tiles dirty (e.g. on screen enter / full redraw)
    void markAllDirty() {
        for (uint8_t i = 0; i < N; ++i) dirty_[i] = true;
    }

    /// Mark all tiles clean
    void markAllClean() {
        for (uint8_t i = 0; i < N; ++i) dirty_[i] = false;
    }

    /// Check if a tile needs redraw
    bool isDirty(uint8_t idx) const {
        return (idx < N) && dirty_[idx];
    }

    /// Check if any tile is dirty
    bool anyDirty() const {
        for (uint8_t i = 0; i < N; ++i) {
            if (dirty_[i]) return true;
        }
        return false;
    }

    /// Count of dirty tiles (useful for metrics)
    uint8_t dirtyCount() const {
        uint8_t count = 0;
        for (uint8_t i = 0; i < N; ++i) {
            if (dirty_[i]) ++count;
        }
        return count;
    }

    /// Update tile hash and mark dirty if changed.
    /// Returns true if the tile was marked dirty (hash changed).
    bool updateHash(uint8_t idx, TileHash newHash) {
        if (idx >= N) return false;
        if (newHash != hashes_[idx]) {
            hashes_[idx] = newHash;
            dirty_[idx]  = true;
            return true;
        }
        return false;
    }

    /// Get the last stored hash for a tile
    TileHash lastHash(uint8_t idx) const {
        return (idx < N) ? hashes_[idx] : 0;
    }

    /// Reset all hashes to zero and mark all dirty (full invalidation)
    void invalidateAll() {
        memset(hashes_, 0, sizeof(hashes_));
        markAllDirty();
    }

    /// Force-redraw a specific tile (hash failsafe).
    /// Zeroes the stored hash so the next updateHash() will always mark dirty.
    /// Use for critical tiles (SPEED, FAULT) as a safety net against
    /// hash collisions or missed invalidation.
    void forceRedraw(uint8_t idx) {
        if (idx < N) {
            hashes_[idx] = 0;
            dirty_[idx]  = true;
        }
    }

    /// Number of tiles in this set
    static constexpr uint8_t size() { return N; }

private:
    TileRect  rects_[N]  = {};
    bool      dirty_[N]  = {};
    TileHash  hashes_[N] = {};
};

} // namespace ui

#endif // TILE_ENGINE_H
