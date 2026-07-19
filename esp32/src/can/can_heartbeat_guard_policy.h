#ifndef CAN_HEARTBEAT_GUARD_POLICY_H
#define CAN_HEARTBEAT_GUARD_POLICY_H

#include <cstdint>

namespace can_heartbeat {

/*
 * Pure scheduling + bookkeeping for the single ESP32 -> STM32 safety heartbeat
 * (0x011) producer.
 *
 * There is exactly ONE producer: the dedicated FreeRTOS task.  It transmits
 * 0x011 unconditionally every 100 ms (anchored on absolute time with
 * vTaskDelayUntil) and, when the TWAI driver rejects a frame, performs one
 * bounded 10 ms retry.  The legacy Arduino loop() producer has been removed,
 * so this policy owns the single rolling counter as well.
 *
 * The class is HAL-free (no FreeRTOS/TWAI), so the exact cadence, counter and
 * diagnostic accounting can be verified by host tests.
 *
 * All time arithmetic uses unsigned subtraction, so millis() rollover is
 * handled naturally.
 */
struct HeartbeatConfig {
    uint32_t periodMs = 100U;   // unconditional transmit cadence
    uint32_t retryMs  = 10U;    // bounded retry delay after a rejected frame
};

/* Coherent diagnostic snapshot for serial/engineering telemetry. */
struct HeartbeatDiag {
    bool     everAccepted    = false;   // at least one frame accepted
    uint32_t lastAcceptedMs  = 0U;      // millis() of last accepted transmit
    uint32_t currentGapMs    = 0U;      // now - lastAcceptedMs
    uint32_t maxGapMs        = 0U;      // worst accepted-to-accepted gap
    uint32_t txAcceptedCount = 0U;      // frames the driver queued (ESP_OK)
    uint32_t txRejectedCount = 0U;      // twai_transmit rejections (software)
    uint32_t retryCount      = 0U;      // bounded retries triggered
    uint32_t hwTxFailedCount = 0U;      // twai_status_info_t.tx_failed_count
    uint8_t  queueDepth      = 0U;      // twai_status_info_t.msgs_to_tx
    uint8_t  twaiState       = 0U;      // twai_state_t
    bool     busOff          = false;   // controller BUS_OFF
    bool     errorPassive    = false;   // TEC/REC >= 128 (error-passive)
    uint32_t resetReason     = 0U;      // esp_reset_reason()
    uint8_t  counter         = 0U;      // next rolling-counter value to send
};

class HeartbeatProducer {
public:
    explicit HeartbeatProducer(const HeartbeatConfig& cfg = HeartbeatConfig{})
        : cfg_(cfg) {}

    void reset(uint32_t nowMs) {
        counter_        = 0U;
        lastAcceptedMs_ = nowMs;
        lastAttemptMs_  = nowMs;
        retryPending_   = false;
        everAccepted_   = false;
        diag_           = HeartbeatDiag{};
    }

    /* Value that must be placed on the wire for the next attempt.  It only
     * advances after the driver accepts the frame, so a retry re-sends the
     * same counter value (no sequence gaps for the STM32). */
    uint8_t counter() const { return counter_; }

    /* Is a transmit attempt due at nowMs?  Unconditional 100 ms cadence, or a
     * 10 ms bounded retry when the previous frame was rejected. */
    bool due(uint32_t nowMs) const {
        const uint32_t minGap = retryPending_ ? cfg_.retryMs : cfg_.periodMs;
        return (nowMs - lastAttemptMs_) >= minGap;
    }

    bool retryPending() const { return retryPending_; }

    /* Record the result of a single twai_transmit attempt.  The rolling
     * counter advances ONLY when the frame was accepted. */
    void onResult(uint32_t nowMs, bool accepted) {
        lastAttemptMs_ = nowMs;
        if (accepted) {
            if (everAccepted_) {
                const uint32_t gap = nowMs - lastAcceptedMs_;
                if (gap > diag_.maxGapMs) diag_.maxGapMs = gap;
            }
            lastAcceptedMs_ = nowMs;
            everAccepted_   = true;
            retryPending_   = false;
            ++counter_;                 // uint8_t wraps 255 -> 0 naturally
            ++diag_.txAcceptedCount;
        } else {
            ++diag_.txRejectedCount;
            if (!retryPending_) ++diag_.retryCount;
            retryPending_ = true;
        }
    }

    /* Fold in the latest TWAI controller snapshot for diagnostics. */
    void observe(uint8_t queueDepth, uint8_t twaiState, bool busOff,
                 bool errorPassive, uint32_t hwTxFailed) {
        diag_.queueDepth      = queueDepth;
        diag_.twaiState       = twaiState;
        diag_.busOff          = busOff;
        diag_.errorPassive    = errorPassive;
        diag_.hwTxFailedCount = hwTxFailed;
    }

    void setResetReason(uint32_t reason) { diag_.resetReason = reason; }

    /* Coherent snapshot including the live current gap at nowMs. */
    HeartbeatDiag snapshot(uint32_t nowMs) const {
        HeartbeatDiag d = diag_;
        d.everAccepted   = everAccepted_;
        d.lastAcceptedMs = lastAcceptedMs_;
        d.currentGapMs   = everAccepted_ ? (nowMs - lastAcceptedMs_) : 0U;
        d.counter        = counter_;
        return d;
    }

    /* Live gap since the last accepted frame (0 before the first one). */
    uint32_t currentGap(uint32_t nowMs) const {
        return everAccepted_ ? (nowMs - lastAcceptedMs_) : 0U;
    }

    uint32_t maxGap() const { return diag_.maxGapMs; }

private:
    HeartbeatConfig cfg_{};
    HeartbeatDiag   diag_{};
    uint8_t  counter_        = 0U;
    uint32_t lastAcceptedMs_ = 0U;
    uint32_t lastAttemptMs_  = 0U;
    bool     retryPending_   = false;
    bool     everAccepted_   = false;
};

}  // namespace can_heartbeat

#endif  // CAN_HEARTBEAT_GUARD_POLICY_H
