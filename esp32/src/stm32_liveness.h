/**
 ****************************************************************************
 * @file    stm32_liveness.h
 * @brief   Pure, host-testable STM32 heartbeat liveness tracker.
 *
 *          SAFETY FIX (§2): the previous implementation lived inline in
 *          main.cpp loop() and re-evaluated the STM32 alive counter on EVERY
 *          loop iteration.  Because loop() runs every few milliseconds while
 *          the STM32 heartbeat only arrives every ~100 ms, the SAME stored
 *          heartbeat sample was processed dozens of times before the next
 *          frame arrived.  The "same counter" branch therefore incremented on
 *          each pass and the STM32 was declared frozen FALSELY.
 *
 *          This tracker evaluates the alive counter ONLY when a brand-new
 *          heartbeat frame has been decoded (detected via a monotonically
 *          updated frame timestamp), and additionally uses a time-based
 *          absence timeout to distinguish "no new heartbeat at all" from
 *          "new heartbeats arriving with a stuck counter".
 *
 *          It is intentionally free of any Arduino/ESP-IDF dependency so it
 *          can be linked directly into host unit tests (no re-implementation
 *          of the logic in the test file).
 ****************************************************************************
 */
#ifndef STM32_LIVENESS_H
#define STM32_LIVENESS_H

#include <cstdint>

class Stm32Liveness {
public:
    /**
     * @param freezeCount        Number of consecutive NEW heartbeat frames
     *                           carrying an identical alive counter before the
     *                           STM32 is declared frozen.
     * @param absenceTimeoutMs   If no NEW heartbeat frame has been observed for
     *                           longer than this, the STM32 is declared not
     *                           alive (absent), independent of the counter.
     */
    Stm32Liveness(uint8_t freezeCount, uint32_t absenceTimeoutMs)
        : freezeCount_(freezeCount), absenceTimeoutMs_(absenceTimeoutMs) {}

    /**
     * Advance the tracker.  Call once per loop.
     *
     * @param frameTimestampMs  Timestamp (millis) recorded when the LAST
     *                          heartbeat frame was decoded.  0 until the first
     *                          frame is received.  A change in this value is
     *                          how a genuinely new frame is detected.
     * @param aliveCounter      Heartbeat byte 0 (wraps 255->0).
     * @param nowMs             Current millis().
     * @return true if a brand-new frame was processed on this call.
     */
    bool update(uint32_t frameTimestampMs, uint8_t aliveCounter, uint32_t nowMs) {
        bool processedNew = false;

        // Evaluate the alive counter ONLY when a new frame has arrived.  A new
        // frame is one whose recorded timestamp differs from the last one we
        // processed.  This prevents the same sample from being counted many
        // times while loop() spins faster than the heartbeat period.
        if (frameTimestampMs != 0U && frameTimestampMs != lastProcessedTs_) {
            lastProcessedTs_ = frameTimestampMs;
            processedNew     = true;

            if (!seen_) {
                // First ever valid frame establishes the baseline; alive only
                // becomes true after a valid, coherent frame.
                seen_        = true;
                lastCounter_ = aliveCounter;
                sameCount_   = 0;
                alive_       = true;
            } else if (aliveCounter != lastCounter_) {
                // Counter advanced (or wrapped 255->0): STM32 main loop alive.
                lastCounter_ = aliveCounter;
                sameCount_   = 0;
                alive_       = true;
            } else {
                // A NEW frame carried the same counter -> possible freeze.
                if (sameCount_ < freezeCount_) {
                    sameCount_++;
                }
                if (sameCount_ >= freezeCount_) {
                    if (alive_) {
                        frozenEdge_ = true;  // one-shot for rate-limited logging
                    }
                    alive_ = false;
                }
            }
        }

        // Time-based absence: if no new frame has been seen for longer than the
        // absence timeout, declare the STM32 not alive regardless of counter.
        if (seen_ && (uint32_t)(nowMs - frameTimestampMs) > absenceTimeoutMs_) {
            alive_ = false;
        }

        return processedNew;
    }

    bool    isAlive()   const { return alive_; }
    uint8_t sameCount() const { return sameCount_; }
    bool    hasSeen()   const { return seen_; }

    /** One-shot: true exactly once when the tracker transitions to frozen. */
    bool consumeFrozenEdge() {
        bool e      = frozenEdge_;
        frozenEdge_ = false;
        return e;
    }

private:
    const uint8_t  freezeCount_;
    const uint32_t absenceTimeoutMs_;
    uint32_t       lastProcessedTs_ = 0;
    uint8_t        lastCounter_     = 0;
    uint8_t        sameCount_       = 0;
    bool           seen_            = false;
    bool           alive_           = false;
    bool           frozenEdge_      = false;
};

#endif  // STM32_LIVENESS_H
