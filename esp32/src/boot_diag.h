// boot_diag.h — Pure, host-testable ESP32 boot-diagnostics formatter.
//
// The ESP32 HMI is CAN receive-only (it never transmits diagnostic frames),
// so the boot telemetry channel is the serial log.  This module keeps the
// *classification* and *formatting* of the boot report free of any ESP-IDF
// dependency so it can be unit-tested on the host.  The thin hardware glue in
// main.cpp maps esp_reset_reason() onto boot_diag::ResetClass and reads the
// live heap / stack figures, then hands them to boot_diag::format().
//
// Instrumentation only — nothing here changes control or safety behaviour.
#ifndef BOOT_DIAG_H
#define BOOT_DIAG_H

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

namespace boot_diag {

// Compact reset categories.  Kept independent of esp_reset_reason_t so the
// module builds on the host; main.cpp performs the (hardware-specific)
// esp_reset_reason_t -> ResetClass mapping.
enum class ResetClass : uint8_t {
    POWER_ON  = 0,   // clean cold boot (ESP_RST_POWERON)
    SOFTWARE  = 1,   // esp_restart() / ESP_RST_SW
    PANIC     = 2,   // exception / abort (ESP_RST_PANIC)
    WATCHDOG  = 3,   // INT/TASK/other watchdog (ESP_RST_*_WDT)
    BROWNOUT  = 4,   // supply sag (ESP_RST_BROWNOUT)
    DEEPSLEEP = 5,   // wake from deep sleep (ESP_RST_DEEPSLEEP)
    EXTERNAL  = 6,   // external pin / SDIO (ESP_RST_EXT / ESP_RST_SDIO)
    UNKNOWN   = 7,   // ESP_RST_UNKNOWN or anything unmapped
};

// Live memory / stack figures captured at boot.  All values are best-effort
// snapshots; 0 is a valid "not available" sentinel for the stack fields.
struct HeapStats {
    uint32_t freeNow;      // esp_get_free_heap_size()
    uint32_t minFreeEver;  // esp_get_minimum_free_heap_size() (low-water mark)
    uint32_t totalSize;    // ESP.getHeapSize()
    uint32_t largestBlock; // ESP.getMaxAllocHeap() (fragmentation indicator)
};

inline const char* name(ResetClass rc) {
    switch (rc) {
        case ResetClass::POWER_ON:  return "PowerOn";
        case ResetClass::SOFTWARE:  return "Software";
        case ResetClass::PANIC:     return "Panic";
        case ResetClass::WATCHDOG:  return "Watchdog";
        case ResetClass::BROWNOUT:  return "Brownout";
        case ResetClass::DEEPSLEEP: return "DeepSleep";
        case ResetClass::EXTERNAL:  return "External";
        case ResetClass::UNKNOWN:   default: return "Unknown";
    }
}

// True for resets that indicate a fault the operator should notice: a firmware
// panic, a watchdog timeout, or a supply brownout.  Power-on / software /
// deep-sleep / external resets are expected and return false.
inline bool isAbnormal(ResetClass rc) {
    return rc == ResetClass::PANIC ||
           rc == ResetClass::WATCHDOG ||
           rc == ResetClass::BROWNOUT;
}

// Formats a single-line boot report into buf.  Returns the number of
// characters written (excluding the NUL), or 0 if buf is null/empty.  The
// format is stable and machine-parseable:
//   [BOOTDIAG] reset=<name> abnormal=<0|1> heap_free=<b> heap_min=<b>
//              heap_size=<b> heap_maxblk=<b> stack_loop=<w> stack_render=<w>
inline int format(char* buf, size_t n, ResetClass rc, const HeapStats& h,
                  uint32_t loopStackHwm, uint32_t renderStackHwm) {
    if (buf == nullptr || n == 0) {
        return 0;
    }
    int w = snprintf(buf, n,
        "[BOOTDIAG] reset=%s abnormal=%d heap_free=%u heap_min=%u "
        "heap_size=%u heap_maxblk=%u stack_loop=%u stack_render=%u",
        name(rc), isAbnormal(rc) ? 1 : 0,
        (unsigned)h.freeNow, (unsigned)h.minFreeEver,
        (unsigned)h.totalSize, (unsigned)h.largestBlock,
        (unsigned)loopStackHwm, (unsigned)renderStackHwm);
    if (w < 0) {
        buf[0] = '\0';
        return 0;
    }
    // snprintf returns the would-be length; clamp to the truncated size.
    return (size_t)w >= n ? (int)(n - 1) : w;
}

} // namespace boot_diag

#endif // BOOT_DIAG_H
