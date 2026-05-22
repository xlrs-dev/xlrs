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

// Pure slot-selection: which slot does this packet tick belong to?
// Precedence: Sync > Telemetry > Uplink. Host-testable.
//   syncEveryNTicks: emit a Sync beacon every N ticks (0 = never).
//   tlmRatioDenom:   telemetry slot ratio 1:N (0 = no telemetry).
inline Slot slotForTick(uint32_t tick, uint8_t tlmRatioDenom, uint16_t syncEveryNTicks) {
    if (syncEveryNTicks && (tick % syncEveryNTicks) == 0) return Slot::Sync;
    if (tlmRatioDenom    && (tick % tlmRatioDenom)    == 0) return Slot::Telemetry;
    return Slot::Uplink;
}

// Forward-declared collaborators keep this header dependency-light.
class IRadioPhy;
class Fhss;
class Link;

class RfScheduler {
public:
    static const uint32_t TX_GUARD_US = 150;

    RfScheduler();
    ~RfScheduler();

    // Wire in the lower layers, link instance, and the active rate (sets cadence + slot ratios).
    bool begin(IRadioPhy* phy, Fhss* fhss, Link* link, uint8_t rateIndex);

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

private:
    bool recoverPhyIfNeeded();

    std::atomic<uint32_t> _tickEvents{0};
    std::atomic<uint32_t> _rxDoneEvents{0};
    std::atomic<uint32_t> _txDoneEvents{0};

private:
    IRadioPhy*   _phy = nullptr;
    Fhss*        _fhss = nullptr;
    Link*        _link = nullptr;
    RateConfig   _rate{};
    uint8_t      _rateIndex = 0;
    Slot         _currentSlot = Slot::Idle;
    uint32_t     _tick = 0;
    uint32_t     _tickStartUs = 0;
    Pfd          _pfd{};
    HwTimer*     _timer = nullptr;

    uint32_t _lastProcessedTickEvent = 0;
    uint32_t _lastProcessedRxEvent = 0;
    uint32_t _lastProcessedTxEvent = 0;
};

} // namespace xlrs
