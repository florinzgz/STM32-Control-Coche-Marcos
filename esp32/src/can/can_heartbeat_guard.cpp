#include "can_heartbeat_guard.h"

#include <Arduino.h>
#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "can_ids.h"

namespace can_heartbeat {
namespace {

static constexpr uint32_t TASK_TICK_MS      = 5U;
static constexpr uint32_t TX_WAIT_MS        = 5U;
static constexpr uint32_t TASK_STACK_BYTES  = 3072U;
static constexpr UBaseType_t TASK_PRIORITY  = 5U;
static constexpr BaseType_t TASK_CORE       = 1;

static_assert(can::HEARTBEAT_INTERVAL_MS < can::HEARTBEAT_TIMEOUT_MS,
              "Heartbeat period must remain below the STM32 watchdog");
static_assert(120U < can::HEARTBEAT_TIMEOUT_MS,
              "Backup stall threshold must remain below the STM32 watchdog");

static GuardPolicy g_policy;
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t g_taskHandle = nullptr;
/* Deliberately separated from the normal loop producer's 0-based counter.
 * It is used only during a measured loop stall/congestion event. */
static uint8_t g_counter = 0x80U;
static uint32_t g_lastTxFailedCount = 0U;

static GuardStats snapshotLocked() {
    taskENTER_CRITICAL(&g_mux);
    const GuardStats copy = g_policy.stats();
    taskEXIT_CRITICAL(&g_mux);
    return copy;
}

/* Transmit exactly one failover heartbeat.  The counter advances only when the
 * ESP-IDF TWAI driver accepts the frame into its TX queue. */
static bool transmitHeartbeat(uint32_t nowMs) {
    twai_message_t msg{};
    msg.identifier = can::HEARTBEAT_ESP32;
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = 1;
    msg.data[0] = g_counter;

    const esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(TX_WAIT_MS));
    const bool sent = (err == ESP_OK);

    taskENTER_CRITICAL(&g_mux);
    g_policy.noteAttempt(nowMs, sent);
    taskEXIT_CRITICAL(&g_mux);

    if (sent) {
        ++g_counter;
    }
    return sent;
}

/* Backup producer independent from Arduino loop().  It stays silent while the
 * normal loop producer is alive.  NVS/LED/audio/main-loop stalls therefore get
 * a bounded 0x011 failover without introducing a permanent second producer. */
static void heartbeatTask(void*) {
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(TASK_TICK_MS));
        const uint32_t nowMs = millis();

        twai_status_info_t info{};
        if (twai_get_status_info(&info) != ESP_OK) {
            taskENTER_CRITICAL(&g_mux);
            g_policy.noteSkippedNotRunning();
            taskEXIT_CRITICAL(&g_mux);
            continue;
        }

        taskENTER_CRITICAL(&g_mux);
        g_policy.observeQueue(static_cast<uint8_t>(info.msgs_to_tx));
        if (info.tx_failed_count != g_lastTxFailedCount) {
            g_lastTxFailedCount = info.tx_failed_count;
            g_policy.notifyTxDrop();
        }
        const bool due = g_policy.shouldAttempt(nowMs);
        taskEXIT_CRITICAL(&g_mux);

        if (!due) {
            continue;
        }

        if (info.state != TWAI_STATE_RUNNING) {
            taskENTER_CRITICAL(&g_mux);
            g_policy.noteSkippedNotRunning();
            taskEXIT_CRITICAL(&g_mux);
            continue;
        }

        (void)transmitHeartbeat(nowMs);
        /* Failed attempts are retried by the same absolute task after 10 ms.
         * No unbounded loop and no long blocking wait are used here. */
    }
}

}  // namespace

bool init() {
    taskENTER_CRITICAL(&g_mux);
    if (g_taskHandle != nullptr) {
        taskEXIT_CRITICAL(&g_mux);
        return true;
    }

    const uint32_t nowMs = millis();
    g_policy.reset(nowMs);
    twai_status_info_t info{};
    g_lastTxFailedCount =
        (twai_get_status_info(&info) == ESP_OK) ? info.tx_failed_count : 0U;
    taskEXIT_CRITICAL(&g_mux);

    const BaseType_t rc = xTaskCreatePinnedToCore(
        heartbeatTask,
        "CanHbBackup",
        TASK_STACK_BYTES,
        nullptr,
        TASK_PRIORITY,
        &g_taskHandle,
        TASK_CORE);

    if (rc != pdPASS) {
        taskENTER_CRITICAL(&g_mux);
        g_taskHandle = nullptr;
        taskEXIT_CRITICAL(&g_mux);
        Serial.println("[CAN][ERR] Failed to create heartbeat failover task");
        return false;
    }

    Serial.println("[CAN][INFO] Heartbeat failover active: loop stall=120 ms, backup=80 ms");
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
