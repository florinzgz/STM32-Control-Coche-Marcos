#include "can_heartbeat_guard.h"

#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "can_ids.h"

namespace can_heartbeat {
namespace {

static constexpr uint32_t TASK_PERIOD_MS = 10U;
static constexpr uint32_t TX_WAIT_MS = 5U;
static constexpr uint32_t STATS_LOG_MS = 10000U;
static constexpr uint32_t TASK_STACK_BYTES = 3072U;
static constexpr UBaseType_t TASK_PRIORITY = 4U;
static constexpr BaseType_t TASK_CORE = 1;

static GuardPolicy g_policy;
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t g_taskHandle = nullptr;
static uint8_t g_backupCounter = 0x80U;
static uint32_t g_lastTxFailedCount = 0U;

static GuardStats snapshotLocked() {
    taskENTER_CRITICAL(&g_mux);
    const GuardStats copy = g_policy.stats();
    taskEXIT_CRITICAL(&g_mux);
    return copy;
}

static bool sendBackupHeartbeat(uint32_t nowMs) {
    CanFrame frame = {};
    frame.identifier = can::HEARTBEAT_ESP32;
    frame.extd = 0;
    frame.data_length_code = 1;
    frame.data[0] = g_backupCounter;

    bool sent = ESP32Can.writeFrame(frame, TX_WAIT_MS);
    taskENTER_CRITICAL(&g_mux);
    g_policy.noteAttempt(nowMs, sent);
    taskEXIT_CRITICAL(&g_mux);

    if (!sent) {
        vTaskDelay(pdMS_TO_TICKS(10U));
        const uint32_t retryMs = millis();
        taskENTER_CRITICAL(&g_mux);
        g_policy.noteRetry();
        taskEXIT_CRITICAL(&g_mux);
        sent = ESP32Can.writeFrame(frame, TX_WAIT_MS);
        taskENTER_CRITICAL(&g_mux);
        g_policy.noteAttempt(retryMs, sent);
        taskEXIT_CRITICAL(&g_mux);
    }

    if (sent) {
        ++g_backupCounter;  // advance only after TWAI accepted the frame
    }
    return sent;
}

static void heartbeatGuardTask(void*) {
    TickType_t wake = xTaskGetTickCount();
    uint32_t lastStatsLogMs = millis();

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(TASK_PERIOD_MS));
        const uint32_t nowMs = millis();

        // Read the driver state once per guard tick.  When the controller is
        // STOPPED/BUS_OFF/RECOVERING (or not installed yet), do not hammer
        // twai_transmit(); the existing twai_recovery module remains the sole
        // authority for controller recovery.  Pending policy evidence is kept
        // and will be serviced as soon as TWAI returns to RUNNING.
        twai_status_info_t twaiInfo{};
        if (twai_get_status_info(&twaiInfo) != ESP_OK) {
            continue;
        }

        bool due = false;
        taskENTER_CRITICAL(&g_mux);
        g_policy.observeQueue(static_cast<uint8_t>(
            twaiInfo.msgs_to_tx > 255U ? 255U : twaiInfo.msgs_to_tx));

        // Catch failures from every CAN producer, including the legacy 0x011
        // sender whose writeFrame() result is not currently exposed.  A rising
        // TWAI tx_failed_count latches congestion evidence for a priority
        // heartbeat once the queue/controller is healthy again.
        if (twaiInfo.tx_failed_count != g_lastTxFailedCount) {
            g_lastTxFailedCount = twaiInfo.tx_failed_count;
            g_policy.notifyTxDrop();
        }

        if (twaiInfo.state == TWAI_STATE_RUNNING) {
            due = g_policy.shouldAttempt(nowMs);
        }
        taskEXIT_CRITICAL(&g_mux);

        if (due) {
            (void)sendBackupHeartbeat(nowMs);
        }

        if ((nowMs - lastStatsLogMs) >= STATS_LOG_MS) {
            lastStatsLogMs = nowMs;
            const GuardStats s = snapshotLocked();
            if (s.loopStallEvents != 0U || s.congestionEvents != 0U ||
                s.backupFailures != 0U) {
                Serial.printf("[CAN][HB-GUARD] stall=%lu congestion=%lu "
                              "ok=%lu fail=%lu retry=%lu maxLoopGap=%lu ms\n",
                              (unsigned long)s.loopStallEvents,
                              (unsigned long)s.congestionEvents,
                              (unsigned long)s.backupSuccess,
                              (unsigned long)s.backupFailures,
                              (unsigned long)s.retries,
                              (unsigned long)s.maxObservedLoopGapMs);
            }
        }
    }
}

}  // namespace

bool init() {
    taskENTER_CRITICAL(&g_mux);
    if (g_taskHandle != nullptr) {
        taskEXIT_CRITICAL(&g_mux);
        return true;
    }
    g_policy.reset(millis());
    g_lastTxFailedCount = 0U;
    taskEXIT_CRITICAL(&g_mux);

    const BaseType_t rc = xTaskCreatePinnedToCore(
        heartbeatGuardTask,
        "CanHeartbeatGuard",
        TASK_STACK_BYTES,
        nullptr,
        TASK_PRIORITY,
        &g_taskHandle,
        TASK_CORE);

    if (rc != pdPASS) {
        taskENTER_CRITICAL(&g_mux);
        g_taskHandle = nullptr;
        taskEXIT_CRITICAL(&g_mux);
        Serial.println("[CAN][ERR] Failed to create heartbeat guard task");
        return false;
    }

    Serial.println("[CAN][INFO] Heartbeat guard active (Core 1, priority 4)");
    return true;
}

void notifyLoopAlive(uint32_t nowMs) {
    taskENTER_CRITICAL(&g_mux);
    g_policy.notifyLoopAlive(nowMs);
    taskEXIT_CRITICAL(&g_mux);
}

void notifyTxDrop() {
    taskENTER_CRITICAL(&g_mux);
    g_policy.notifyTxDrop();
    taskEXIT_CRITICAL(&g_mux);
}

GuardStats stats() {
    return snapshotLocked();
}

}  // namespace can_heartbeat
