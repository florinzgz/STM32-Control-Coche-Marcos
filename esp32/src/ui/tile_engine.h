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
// PIPELINE:
//   CAN RX → VehicleData → snapshot latch → TileSet::updateHash() per tile
//   → dirty list → render only dirty tiles → display
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

namespace ui {

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

    /// Define the screen region for a tile
    void setRect(uint8_t idx, int16_t x, int16_t y, int16_t w, int16_t h) {
        if (idx < N) {
            rects_[idx] = {x, y, w, h};
        }
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

    /// Number of tiles in this set
    static constexpr uint8_t size() { return N; }

private:
    TileRect  rects_[N]  = {};
    bool      dirty_[N]  = {};
    TileHash  hashes_[N] = {};
};

} // namespace ui

#endif // TILE_ENGINE_H
