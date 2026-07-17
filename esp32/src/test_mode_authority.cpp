// =============================================================================
// Host integration tests for the CLOSED drive-mode synchronisation (§1-§5).
//
// These go beyond the pure ModeSync unit tests (test_mode_sync.cpp): they wire
// the real ModeSync FSM and the real mode_authority::arbitrate() into a small
// simulated STM32 that models BOTH the ACTIVE mode-change gate AND the §3
// STANDBY logical-mode-sync gate, so the full path is exercised end-to-end:
//
//   source arbitration → desiredModeFlags → g_modeSync.setDesired()
//     → ModeSync::update() → CMD_MODE → STM32 gate → ACK/heartbeat
//     → appliedModeFlags.
//
// The design guarantees checked here:
//   - DESIRED (driver intent) and APPLIED (STM32-confirmed) never alias.
//   - There is a SINGLE CMD_MODE owner (the FSM); selector, remote and tank
//     all reach it only through setDesired().
//   - The HMI mode reflects APPLIED, never the mere request.
//   - The STM32 §3 gate lets a logical mode sync in STANDBY (when safe) so the
//     vehicle never starts moving in the wrong (default 4x2) mode.
//
// Compile (from repo root):
//   g++ -std=c++17 -Wall -Wextra -Iesp32/src
//       esp32/src/test_mode_authority.cpp -o /tmp/test_mode_authority
//   /tmp/test_mode_authority
// =============================================================================

#include "mode_sync.h"
#include "mode_authority.h"

#include <cstdio>
#include <cstdint>

// ---- Minimal test harness ---------------------------------------------------
static int g_tests_run    = 0;
static int g_tests_failed = 0;

#define ASSERT(cond) do {                                                   \
    g_tests_run++;                                                          \
    if (!(cond)) {                                                          \
        std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);         \
        g_tests_failed++;                                                   \
    }                                                                       \
} while (0)

#define ASSERT_EQ(a, b) do {                                                \
    g_tests_run++;                                                          \
    long _va = (long)(a); long _vb = (long)(b);                            \
    if (_va != _vb) {                                                       \
        std::printf("FAIL %s:%d  %s == %s (got %ld vs %ld)\n",             \
                    __FILE__, __LINE__, #a, #b, _va, _vb);                 \
        g_tests_failed++;                                                   \
    }                                                                       \
} while (0)

static constexpr uint32_t ACK_TIMEOUT_MS = 200;
static constexpr uint8_t  MAX_RETRIES    = 3;
static constexpr uint32_t BACKOFF_MS     = 1000;

using Action = ModeSync::Action;
using AckRes = ModeSync::AckResult;

static constexpr uint8_t F_4X4  = mode_authority::FLAG_4X4;
static constexpr uint8_t F_TANK = mode_authority::FLAG_TANK_TURN;

// ---- Simulated STM32 (mode authority) ---------------------------------------
// Replicates the STM32 CMD_MODE decision, including the §3 (Option A) STANDBY
// logical-mode gate added to can_handler.c / safety_system.c.
struct SimStm32 {
    enum State { BOOT, STANDBY, ACTIVE, SAFE };
    State   state        = STANDBY;
    uint8_t applied      = 0;      // logical mode the STM32 holds (heartbeat echo)
    bool    pedalPressed = false;  // pedal above rest threshold
    bool    moving       = false;  // wheel speed above the mode-change gate

    // Mirrors Safety_IsCommandAllowed(): motion commands only in ACTIVE/DEGRADED.
    bool commandAllowed() const { return state == ACTIVE; }

    // Mirrors Safety_IsStandbyModeSyncAllowed(): STANDBY + pedal released +
    // zero PWM + safe speed.  (Zero PWM is implied here because STANDBY never
    // energises traction.)
    bool standbyModeSyncAllowed() const {
        return state == STANDBY && !pedalPressed && !moving;
    }

    // Process a CMD_MODE(modeFlags).  Returns the ACK code the ESP32 observes.
    AckRes onCmdMode(uint8_t modeFlags) {
        if (!commandAllowed()) {
            if (standbyModeSyncAllowed()) {
                applied = modeFlags & mode_authority::FLAG_MASK;  // logical only
                return AckRes::OK;
            }
            return AckRes::BLOCKED;   // e.g. STANDBY with pedal pressed, or SAFE
        }
        if (moving) return AckRes::REJECTED;   // ACTIVE speed gate
        applied = modeFlags & mode_authority::FLAG_MASK;
        return AckRes::OK;
    }

    uint8_t heartbeatEcho() const { return applied; }
};

// ---- Simulated ESP32 loop (desired/applied separation, single CMD_MODE path)-
struct SimEsp32 {
    ModeSync ms{ACK_TIMEOUT_MS, MAX_RETRIES, BACKOFF_MS};

    uint8_t desired       = 0;     // driver intent (arbitrated)
    uint8_t appliedHmi    = 0;     // HMI/vehicleData value = STM32-confirmed mode

    // Arbitration inputs.
    bool    localSel4x4   = false;
    bool    localTank     = false;
    bool    remoteActive  = false;
    uint8_t remoteMode    = 0;

    int      sendCount    = 0;     // CMD_MODE frames emitted (ONLY via the FSM)
    AckRes   lastAckSent  = AckRes::OK;

    // One main-loop tick.  hbAlive models a proven STM32 heartbeat link.
    void tick(uint32_t now, SimStm32& stm, bool hbAlive) {
        // (1) SOLE writer of APPLIED: the STM32 heartbeat echo.
        if (hbAlive) {
            appliedHmi = stm.heartbeatEcho();
            ms.onHeartbeatModeEcho(appliedHmi);
        }
        // (2) SINGLE arbitration → desired → setDesired().
        desired = mode_authority::arbitrate(remoteActive, remoteMode,
                                            localSel4x4, localTank);
        ms.setDesired(desired);
        // (3) SINGLE CMD_MODE owner: the FSM.  No other path transmits.
        if (ms.update(now, hbAlive) == Action::SEND) {
            AckRes r = stm.onCmdMode(ms.sendMode());
            ++sendCount;
            lastAckSent = r;
            ms.onAck(r);
        }
    }

    // Run n ticks spaced dt apart starting at t0; returns the final time.
    uint32_t run(uint32_t t0, uint32_t dt, int n, SimStm32& stm, bool hbAlive) {
        uint32_t t = t0;
        for (int i = 0; i < n; ++i) { tick(t, stm, hbAlive); t += dt; }
        return t;
    }
};

// ---- §2 pure arbitration ----------------------------------------------------

static void test_arbitrate_local_selector() {
    // LOCAL authority: selector 4x4 bit + tank toggle compose the desire.
    ASSERT_EQ(mode_authority::arbitrate(false, 0, false, false), 0);
    ASSERT_EQ(mode_authority::arbitrate(false, 0, true,  false), F_4X4);
    ASSERT_EQ(mode_authority::arbitrate(false, 0, false, true),  F_TANK);
    ASSERT_EQ(mode_authority::arbitrate(false, 0, true,  true),  F_4X4 | F_TANK);
    // The remote payload is IGNORED while LOCAL holds authority.
    ASSERT_EQ(mode_authority::arbitrate(false, F_4X4 | F_TANK, false, false), 0);
}

static void test_arbitrate_remote_authority() {
    // REMOTE authority: the remote's requested mode is the desire; the local
    // selector/tank are ignored.
    ASSERT_EQ(mode_authority::arbitrate(true, F_4X4, false, false), F_4X4);
    ASSERT_EQ(mode_authority::arbitrate(true, 0, true, true), 0);
    ASSERT_EQ(mode_authority::arbitrate(true, F_4X4 | F_TANK, false, false),
              F_4X4 | F_TANK);
    // Undefined high bits are masked off.
    ASSERT_EQ(mode_authority::arbitrate(true, 0xFF, false, false),
              F_4X4 | F_TANK);
}

// ---- §1/§3/§5 integration: boot with selector 4x4 ---------------------------

// Selector at 4x4 before power-on, driver does NOT touch it.  First heartbeat
// echoes the STM32 power-on default 4x2 (applied=0) while desired stays 4x4;
// the §3 STANDBY gate then logically applies 4x4 BEFORE the vehicle can move,
// so there is never a moment of traction in the wrong mode.
static void test_boot_selector_4x4_syncs_in_standby() {
    SimStm32 stm; stm.state = SimStm32::STANDBY; stm.applied = 0; // power-on 4x2
    SimEsp32 esp; esp.localSel4x4 = true;                          // selector 4x4

    // Very first live tick: desired is 4x4 but the HMI still shows the confirmed
    // power-on 4x2 (the echo is read BEFORE the §3 gate applies this tick).
    esp.tick(0, stm, /*hbAlive=*/true);
    ASSERT_EQ(esp.desired, F_4X4);       // driver intent
    ASSERT_EQ(esp.appliedHmi, 0);        // HMI = confirmed 4x2 (not the request)
    ASSERT_EQ(stm.applied, F_4X4);       // §3 gate applied it logically in STANDBY

    // Selector never physically moved; keep ticking — the HMI catches up to the
    // confirmed 4x4 and stays in sync.
    esp.run(1, 10, 20, stm, true);
    ASSERT_EQ(esp.desired, F_4X4);
    ASSERT_EQ(stm.applied, F_4X4);       // STM32 logical mode = 4x4
    ASSERT_EQ(esp.appliedHmi, F_4X4);    // HMI shows applied 4x4
    ASSERT(esp.ms.inSync());

    // Now the driver presses the pedal and the vehicle goes ACTIVE: the mode is
    // ALREADY 4x4, so it never moved in the default 4x2.
    stm.state = SimStm32::ACTIVE; stm.pedalPressed = true;
    esp.run(300, 10, 5, stm, true);
    ASSERT_EQ(stm.applied, F_4X4);
    ASSERT(esp.ms.inSync());
}

// The very first heartbeat reports applied=4x2 while desired=4x4: DESIRED must
// NOT be overwritten by APPLIED (they are independent variables).
static void test_applied_never_overwrites_desired() {
    SimStm32 stm; stm.state = SimStm32::ACTIVE; stm.moving = true; // mode change gated
    SimEsp32 esp; esp.localSel4x4 = true;                          // desired 4x4

    esp.run(0, 10, 5, stm, true);
    // STM32 keeps REJECTING (moving), so it stays 4x2; desired remains 4x4.
    ASSERT_EQ(esp.appliedHmi, 0);
    ASSERT_EQ(esp.desired, F_4X4);
    ASSERT(!esp.ms.inSync());
    ASSERT(esp.lastAckSent == AckRes::REJECTED);
}

// §3: CMD_MODE is BLOCKED in STANDBY when it is NOT safe (pedal pressed), then
// applied AUTOMATICALLY once the pedal is released — no selector toggle.
static void test_standby_blocked_then_applies_when_safe() {
    SimStm32 stm; stm.state = SimStm32::STANDBY; stm.pedalPressed = true;
    SimEsp32 esp; esp.localSel4x4 = true;

    esp.tick(0, stm, true);
    ASSERT(esp.lastAckSent == AckRes::BLOCKED);   // unsafe → blocked
    ASSERT_EQ(stm.applied, 0);
    ASSERT(esp.ms.blocked());
    ASSERT(esp.ms.blockCount() >= 1);             // §4 visible in diagnostics

    // Pedal released while still in STANDBY; the pending block retries after the
    // cooldown and the §3 gate now applies the logical mode.  Space the ticks
    // well beyond the block cooldown so a retransmission actually occurs.
    stm.pedalPressed = false;
    esp.run(BACKOFF_MS, BACKOFF_MS, 6, stm, true);
    ASSERT_EQ(stm.applied, F_4X4);
    ASSERT_EQ(esp.appliedHmi, F_4X4);
    ASSERT(esp.ms.inSync());
}

// §4: a long-pending temporary block is never latched FAILED but IS visible:
// block counter increments and the request age grows.
static void test_block_diagnostics_visible() {
    SimStm32 stm; stm.state = SimStm32::SAFE;      // always blocks
    SimEsp32 esp; esp.localSel4x4 = true;

    esp.tick(0, stm, true);
    uint16_t firstCount = esp.ms.blockCount();
    ASSERT(firstCount >= 1);
    ASSERT(!esp.ms.failed());                      // temporary, never FAILED
    ASSERT(esp.ms.blockReason() == AckRes::BLOCKED);

    esp.run(BACKOFF_MS + 1, BACKOFF_MS + 1, 3, stm, true);
    ASSERT(esp.ms.blockCount() > firstCount);      // keeps counting the retries
    ASSERT(esp.ms.requestAgeMs(5 * BACKOFF_MS) > 0);
    ASSERT(!esp.ms.failed());
}

// ---- §5 tank turn + remote share the SAME ModeSync --------------------------

// Tank turn only flips the DESIRED tank bit; it is synced by the same FSM and
// the HMI updates only after the STM32 confirms.
static void test_tank_turn_uses_modesync() {
    SimStm32 stm; stm.state = SimStm32::ACTIVE;
    SimEsp32 esp; esp.localSel4x4 = true;

    esp.run(0, 10, 5, stm, true);
    ASSERT_EQ(esp.appliedHmi, F_4X4);
    int sendsBefore = esp.sendCount;

    // Driver toggles tank turn on the HMI.
    esp.localTank = true;
    esp.run(200, 10, 5, stm, true);
    ASSERT_EQ(esp.desired, F_4X4 | F_TANK);
    ASSERT_EQ(stm.applied, F_4X4 | F_TANK);
    ASSERT_EQ(esp.appliedHmi, F_4X4 | F_TANK);
    ASSERT(esp.sendCount > sendsBefore);           // went through the FSM
    ASSERT(esp.ms.inSync());
}

// Remote authority drives the SAME ModeSync; switching LOCAL↔REMOTE recomputes
// the desired mode and resynchronises through the single CMD_MODE owner.
static void test_local_remote_switch_uses_modesync() {
    SimStm32 stm; stm.state = SimStm32::ACTIVE;
    SimEsp32 esp; esp.localSel4x4 = true;          // LOCAL desires 4x4

    esp.run(0, 10, 5, stm, true);
    ASSERT_EQ(stm.applied, F_4X4);

    // Remote takes authority and requests 4x2 (no 4x4 bit).
    esp.remoteActive = true; esp.remoteMode = 0;
    esp.run(200, 10, 5, stm, true);
    ASSERT_EQ(esp.desired, 0);
    ASSERT_EQ(stm.applied, 0);
    ASSERT(esp.ms.inSync());

    // Authority returns to LOCAL: the physical selector (still 4x4) wins again.
    esp.remoteActive = false;
    esp.run(400, 10, 5, stm, true);
    ASSERT_EQ(esp.desired, F_4X4);
    ASSERT_EQ(stm.applied, F_4X4);
    ASSERT(esp.ms.inSync());
}

// ---- §5 robustness: STM32 restart, CAN loss, lost ACK -----------------------

// STM32 restart invalidates the confirmation; the selector mode is re-sent and
// re-confirmed without the driver touching anything.
static void test_stm32_restart_resyncs() {
    SimStm32 stm; stm.state = SimStm32::ACTIVE;
    SimEsp32 esp; esp.localSel4x4 = true;
    esp.run(0, 10, 5, stm, true);
    ASSERT(esp.ms.inSync());

    // STM32 restarts: it reverts to the 4x2 default and its confirmation is
    // stale (the ESP32 detects the startup_inhibit edge → invalidateConfirmed).
    stm.applied = 0;
    esp.ms.invalidateConfirmed();
    ASSERT(!esp.ms.inSync());

    esp.run(200, 10, 5, stm, true);
    ASSERT_EQ(stm.applied, F_4X4);                 // re-applied automatically
    ASSERT(esp.ms.inSync());
}

// CAN loss then recovery: nothing is transmitted into a dead link, and the mode
// is resynchronised once the heartbeat returns.
static void test_can_loss_and_recovery() {
    SimStm32 stm; stm.state = SimStm32::ACTIVE;
    SimEsp32 esp; esp.localSel4x4 = true;
    esp.run(0, 10, 5, stm, true);
    ASSERT(esp.ms.inSync());

    int sendsBefore = esp.sendCount;
    // Link drops: STM32 also restarts unseen (reverts to 4x2).
    esp.run(200, 10, 5, stm, /*hbAlive=*/false);
    ASSERT_EQ(esp.sendCount, sendsBefore);         // no traffic into a dead link
    stm.applied = 0;

    // Link returns: the mode is resent and re-applied.
    esp.run(400, 10, 5, stm, true);
    ASSERT_EQ(stm.applied, F_4X4);
    ASSERT(esp.ms.inSync());
}

// A CMD_MODE that is applied but whose ACK is LOST is still confirmed by the
// heartbeat echo alone (no infinite resend).
static void test_lost_ack_confirmed_by_heartbeat() {
    SimStm32 stm; stm.state = SimStm32::ACTIVE;
    ModeSync ms{ACK_TIMEOUT_MS, MAX_RETRIES, BACKOFF_MS};
    ms.setDesired(F_4X4);
    ASSERT(ms.update(0, true) == Action::SEND);
    // STM32 applies it, but the ACK never reaches the ESP32 (no onAck()).
    stm.onCmdMode(ms.sendMode());
    // Heartbeat echoes the applied 4x4 → confirmation without the ACK.
    ms.onHeartbeatModeEcho(stm.heartbeatEcho());
    ASSERT(ms.inSync());
    ASSERT_EQ(ms.confirmed(), F_4X4);
    ASSERT(ms.update(10 * ACK_TIMEOUT_MS, true) == Action::NONE);  // no resend
}

// ---- §1/§5 structural: HMI shows applied; single CMD_MODE owner -------------

// The HMI value tracks APPLIED, never the request: while a 4x4 request is still
// pending the HMI keeps showing the confirmed 4x2.
static void test_hmi_shows_applied_not_desired() {
    SimStm32 stm; stm.state = SimStm32::ACTIVE; stm.moving = true;  // gate mode change
    SimEsp32 esp; esp.localSel4x4 = true;
    esp.run(0, 10, 5, stm, true);
    ASSERT_EQ(esp.desired, F_4X4);      // driver asked for 4x4
    ASSERT_EQ(esp.appliedHmi, 0);       // HMI still shows confirmed 4x2
    ASSERT(!esp.ms.inSync());
}

// Once in sync, a steady state produces NO further CMD_MODE traffic — the FSM
// is the sole owner and does not spam the bus.
static void test_single_owner_no_spam_when_in_sync() {
    SimStm32 stm; stm.state = SimStm32::ACTIVE;
    SimEsp32 esp; esp.localSel4x4 = true;
    esp.run(0, 10, 5, stm, true);
    ASSERT(esp.ms.inSync());
    int sends = esp.sendCount;
    esp.run(200, 10, 50, stm, true);   // nothing changes
    ASSERT_EQ(esp.sendCount, sends);   // no extra frames
}

int main() {
    test_arbitrate_local_selector();
    test_arbitrate_remote_authority();
    test_boot_selector_4x4_syncs_in_standby();
    test_applied_never_overwrites_desired();
    test_standby_blocked_then_applies_when_safe();
    test_block_diagnostics_visible();
    test_tank_turn_uses_modesync();
    test_local_remote_switch_uses_modesync();
    test_stm32_restart_resyncs();
    test_can_loss_and_recovery();
    test_lost_ack_confirmed_by_heartbeat();
    test_hmi_shows_applied_not_desired();
    test_single_owner_no_spam_when_in_sync();

    std::printf("\n--- mode_authority tests: %d run, %d failed ---\n",
                g_tests_run, g_tests_failed);
    return g_tests_failed ? 1 : 0;
}
