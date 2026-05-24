// RfScheduler — the per-tick RF coordinator (core 1).
//
// Owns packet cadence and slot timing. On every timer tick it: decides the slot, advances
// the FHSS index, asks the codec to build/parse the frame for that slot, and drives the
// PHY's async TX/RX with the correct turnaround. It feeds LinkStats. This is the central
// authority that keeps Timing / FHSS / OTA / PHY from racing each other.
//
// Distinct from Link (the connection LIFECYCLE: bind → connect → connected → failsafe,
// which runs at a slower, event-driven cadence). The scheduler is microsecond-critical;
// Link is not.
//
// Turnaround budget: a Telemetry slot must arm RX *before* the expected downlink arrival,
// and the SX1280 TX↔RX mode switch (tens of µs) must fit inside the slot. That constraint
// caps the max packet rate and lives here.
#pragma once
#include <stdint.h>
#include <atomic>
#include "timing/Pfd.h"
#include "timing/HwTimer.h"
#include "link/RateConfig.h"

namespace xlrs {

enum class Slot : uint8_t { Uplink, Telemetry, Sync, Bind, Idle };

// NOTE: the authoritative per-tick slot decision is Link::slotForTick (FHSS-position-based).
// A separate tick-cadence helper used to live here; it was removed to keep a single source of
// truth and avoid the two diverging on Sync semantics.

// Forward-declared collaborators keep this header dependency-light.
class IRadioPhy;
class Link;

class RfScheduler {
public:
    static const uint32_t TX_GUARD_US = 150;

    // If poll() ever finds more than this many timer ticks waiting (core 1 stalled — e.g. a
    // long flash write), it stops replaying the backlog slot-by-slot (each replay drives
    // blocking SPI + a TX_GUARD sleep for a now-stale FHSS slot) and fast-forwards to the
    // latest tick instead. See poll().
    static const uint32_t MAX_TICK_CATCHUP = 16;
    static const uint32_t PHY_RECOVERY_BACKOFF_US = 100000;

    RfScheduler();
    ~RfScheduler();

    // Wire in the PHY, link instance, and the active rate (sets cadence + slot ratios).
    // (FHSS lives inside Link — the scheduler reads frequency via Link::freqForTick.)
    bool begin(IRadioPhy* phy, Link* link, uint8_t rateIndex);

    // Called from the hardware-timer ISR context path (core 1). Must stay short:
    // it sequences the slot and kicks async PHY ops — no blocking, no logging, no alloc.
    void onTick(uint32_t tick);

    // Called from the RF task after a DIO RxDone/TxDone flag is observed.
    void onRxDone();
    void onTxDone();

    // Background polling loop for Core 1 (task context) to process tiny ISR events.
    void poll();

    Slot currentSlot() const;

    uint32_t processedTick() const { return _tick; }
    uint32_t tickEvents() const { return _tickEvents.load(std::memory_order_relaxed); }
    uint32_t rxDoneEvents() const { return _rxDoneEvents.load(std::memory_order_relaxed); }
    uint32_t txDoneEvents() const { return _txDoneEvents.load(std::memory_order_relaxed); }
    int32_t  pfdPhaseErrorUs() const { return _pfd.phaseError(); }
    uint32_t timerIntervalUs() const {
        return _timer ? _timer->intervalUs() : _rate.intervalUs;
    }

private:
    bool recoverPhyIfNeeded();
    void syncPhyIdentity(bool force = false);

    std::atomic<uint32_t> _tickEvents{0};
    std::atomic<uint32_t> _rxDoneEvents{0};
    std::atomic<uint32_t> _txDoneEvents{0};
    // Wall-clock (us) latched IN the timer ISR at each fire. onTick() uses this as the PFD's
    // "expected tick" reference so the phase error is measured against the true fire time, not
    // against when the core-1 task got around to processing the tick (which adds wake-latency
    // jitter). See docs/troubleshooting/index.md §1.1.
    std::atomic<uint32_t> _lastTickFireUs{0};

private:
    IRadioPhy*   _phy = nullptr;
    Link*        _link = nullptr;
    RateConfig   _rate{};
    uint8_t      _rateIndex = 0;
    Slot         _currentSlot = Slot::Idle;
    uint32_t     _tick = 0;
    uint32_t     _tickStartUs = 0;
    uint32_t     _nextPhyRecoveryAttemptUs = 0;
    // The (tick, pos, tickStart) the radio was ARMED with this slot. onRxDone() processes a
    // drained packet against THIS — not the current scheduler tick — so a packet drained after
    // a tick advance (late packet / poll() fast-forward) is still decoded against its own slot.
    uint32_t     _armedTick = 0;
    uint16_t     _armedPos = 0;
    uint32_t     _armedTickStartUs = 0;
    Pfd          _pfd{};
    HwTimer*     _timer = nullptr;
    uint32_t     _syncedIdentityRevision = 0;
    bool         _identitySynced = false;

    uint32_t _lastProcessedTickEvent = 0;
    uint32_t _lastProcessedRxEvent = 0;
    uint32_t _lastProcessedTxEvent = 0;
};

} // namespace xlrs
