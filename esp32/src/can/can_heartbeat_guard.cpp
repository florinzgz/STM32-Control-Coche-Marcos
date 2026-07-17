#include "can_heartbeat_guard.h"

#include <Arduino.h>
#include <driver/twai.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "can_ids.h"

namespace can_heartbeat {
namespace {

// Unconditional transmit cadence and bounded retry, both below the STM32
// 250 ms watchdog so a single missed slot can never trip CAN_TIMEOUT.
static constexpr uint32_t PERIOD_MS         = can::HEARTBEAT_INTERVAL_MS;  // 100
static constexpr uint32_t RETRY_MS          = 10U;
static constexpr uint32_t TX_WAIT_MS        = 5U;
static constexpr uint32_t DIAG_LOG_MS       = 1000U;
static constexpr uint32_t TASK_STACK_BYTES  = 3072U;
static constexpr UBaseType_t TASK_PRIORITY  = 6U;   // above app tasks
static constexpr BaseType_t TASK_CORE       = 1;
static constexpr uint8_t ERROR_PASSIVE_TEC  = 128U;

static_assert(PERIOD_MS < can::HEARTBEAT_TIMEOUT_MS,
              "Heartbeat period must remain below the STM32 watchdog");
static_assert(PERIOD_MS + RETRY_MS < can::HEARTBEAT_TIMEOUT_MS,
              "Period + one bounded retry must remain below the STM32 watchdog");

static HeartbeatProducer g_producer;
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t g_taskHandle = nullptr;

// Transmit exactly one 0x011 frame carrying the current rolling counter.
// Returns true only when the ESP-IDF TWAI driver accepts it into the queue.
static bool transmitFrame(uint8_t counter) {
    twai_message_t msg{};
    msg.identifier       = can::HEARTBEAT_ESP32;
    msg.extd             = 0;
    msg.rtr              = 0;
    msg.data_length_code = 1;
    msg.data[0]          = counter;

    return twai_transmit(&msg, pdMS_TO_TICKS(TX_WAIT_MS)) == ESP_OK;
}

// One transmit attempt with full diagnostic accounting.
static bool attempt(uint32_t nowMs) {
    taskENTER_CRITICAL(&g_mux);
    const uint8_t counter = g_producer.counter();
    taskEXIT_CRITICAL(&g_mux);

    const bool accepted = transmitFrame(counter);

    taskENTER_CRITICAL(&g_mux);
    g_producer.onResult(nowMs, accepted);
    taskEXIT_CRITICAL(&g_mux);
    return accepted;
}

static void snapshotController() {
    twai_status_info_t info{};
    if (twai_get_status_info(&info) != ESP_OK) {
        return;
    }
    const bool busOff = (info.state == TWAI_STATE_BUS_OFF);
    const bool errPassive = (info.tx_error_counter >= ERROR_PASSIVE_TEC) ||
                            (info.rx_error_counter >= ERROR_PASSIVE_TEC);

    taskENTER_CRITICAL(&g_mux);
    g_producer.observe(static_cast<uint8_t>(info.msgs_to_tx),
                       static_cast<uint8_t>(info.state),
                       busOff, errPassive,
                       static_cast<uint32_t>(info.tx_failed_count));
    taskEXIT_CRITICAL(&g_mux);
}

static void logDiagnostics(uint32_t nowMs) {
    taskENTER_CRITICAL(&g_mux);
    const HeartbeatDiag d = g_producer.snapshot(nowMs);
    taskEXIT_CRITICAL(&g_mux);

    // last accepted send · current/max gap · tx_failed_count · queue depth ·
    // TWAI state · BUS_OFF/error-passive · reset reason.
    Serial.printf("[CAN_HB] last=%lums gap=%lu/%lums txAcc=%lu txRej=%lu "
                  "retry=%lu hwFail=%lu q=%u state=%u busoff=%u errpass=%u "
                  "rst=%lu seq=%u\n",
                  (unsigned long)d.lastAcceptedMs,
                  (unsigned long)d.currentGapMs,
                  (unsigned long)d.maxGapMs,
                  (unsigned long)d.txAcceptedCount,
                  (unsigned long)d.txRejectedCount,
                  (unsigned long)d.retryCount,
                  (unsigned long)d.hwTxFailedCount,
                  (unsigned)d.queueDepth,
                  (unsigned)d.twaiState,
                  (unsigned)d.busOff,
                  (unsigned)d.errorPassive,
                  (unsigned long)d.resetReason,
                  (unsigned)d.counter);
}

// The single 0x011 producer.  Unconditional 100 ms cadence anchored on
// absolute time; one bounded 10 ms retry on rejection.  Fully independent from
// Arduino loop(), so NVS/LED/audio/TFT stalls can never delay the heartbeat.
static void heartbeatTask(void*) {
    TickType_t wake = xTaskGetTickCount();
    uint32_t lastLogMs = millis();

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_MS));

        snapshotController();

        const uint32_t nowMs = millis();
        const bool accepted = attempt(nowMs);

        if (!accepted) {
            // Bounded retry: a single re-attempt 10 ms later re-sends the same
            // counter value.  No unbounded loop, no long blocking wait.
            vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
            (void)attempt(millis());
        }

        const uint32_t logNow = millis();
        if (logNow - lastLogMs >= DIAG_LOG_MS) {
            lastLogMs = logNow;
            logDiagnostics(logNow);
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
    g_producer.reset(millis());
    g_producer.setResetReason(static_cast<uint32_t>(esp_reset_reason()));
    taskEXIT_CRITICAL(&g_mux);

    const BaseType_t rc = xTaskCreatePinnedToCore(
        heartbeatTask,
        "CanHeartbeat",
        TASK_STACK_BYTES,
        nullptr,
        TASK_PRIORITY,
        &g_taskHandle,
        TASK_CORE);

    if (rc != pdPASS) {
        taskENTER_CRITICAL(&g_mux);
        g_taskHandle = nullptr;
        taskEXIT_CRITICAL(&g_mux);
        Serial.println("[CAN][ERR] Failed to create heartbeat producer task");
        return false;
    }

    Serial.println("[CAN][INFO] Heartbeat 0x011 producer active: 100 ms "
                   "unconditional (single owner)");
    return true;
}

HeartbeatDiag diag() {
    taskENTER_CRITICAL(&g_mux);
    const HeartbeatDiag d = g_producer.snapshot(millis());
    taskEXIT_CRITICAL(&g_mux);
    return d;
}

}  // namespace can_heartbeat
