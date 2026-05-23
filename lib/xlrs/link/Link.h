// Link — connection LIFECYCLE state machine (bind → connect → connected → failsafe).
//
// Slow, event-driven. The per-tick RF sequencing (slot/FHSS-advance/turnaround) lives in
// RfScheduler, NOT here — keeping the two timescales separate is deliberate (see
// RfScheduler.h). Link consumes the scheduler's LinkStats and drives high-level state +
// failsafe; it never touches the microsecond-critical path.
//
// This header defines the contract and the pure helpers (LqTracker, stats/enums); the
// implementation (Link.cpp) lands in M3 once PHY and timing exist.
#pragma once
#include <stdint.h>
#include "phy/IRadioPhy.h"
#include "link/RateConfig.h"
#include "link/DynamicPower.h"
#include "crypto/ICipher.h"
#include "fhss/Fhss.h"
#include "fhss/channels_2g4.h"
#include "link/StubbornTelemetry.h"
#include "link/RfScheduler.h"

namespace xlrs {

enum class Role         : uint8_t { Tx, Rx };
enum class LinkState    : uint8_t { Disconnected, Binding, Connecting, Connected, Failsafe };
enum class FailsafeMode : uint8_t { NoPulses, Hold };   // default NoPulses (stop CRSF → FC failsafe); Hold is configurable

struct LinkStats {
    uint8_t  lqUp;            // uplink LQ 0..100 (RX-measured over uplink slots only)
    uint8_t  lqDown;          // downlink LQ 0..100 (TX-measured)
    int16_t  rssiDbm;         // smoothed (EMA) RSSI
    int8_t   snr;             // LoRa only; 0 in FLRC
    uint8_t  rateIndex;       // index into kRates
    uint16_t missedDeadlines; // slot prep that missed the RF slot deadline (timing health)
    uint16_t rxQueueDrops;    // RfToApp ring overflow count
    uint16_t phyRecoveries;   // radio backend recovered/reinitialized
    uint16_t phyRecoveryFailures;
    uint16_t telemetryMisses;  // expected telemetry slots without a downlink packet
    uint8_t  fhssIndex;        // current sequence position for diagnostics
    int8_t   txPowerDbm;       // currently requested TX output power
};

// Sliding-window link quality: received vs. expected over the last `Window` slots.
// Replaces the legacy linear RSSI→LQ guess with a true packet-success ratio.
template <uint8_t Window = 100>
class LqTracker {
public:
    void reset() {
        for (uint8_t i = 0; i < Window; ++i) {
            _hist[i] = false;
        }
        _pos = _count = _filled = 0;
    }

    // Call once per expected slot; `received` = a valid packet arrived in that slot.
    void update(bool received) {
        if (_hist[_pos] && !received) {
            _count--;
        } else if (!_hist[_pos] && received) {
            _count++;
        }
        _hist[_pos] = received;
        _pos = (uint8_t)((_pos + 1) % Window);
        if (_filled < Window) {
            _filled++;
        }
    }

    uint8_t lq() const { return _filled ? (uint8_t)((uint16_t)_count * 100 / _filled) : 0; }

private:
    bool    _hist[Window] = {};
    uint8_t _pos = 0, _count = 0, _filled = 0;
};

// Orchestrator (M3: fixed frequency; FHSS/slotting deepen at M4/M5). Defined in Link.cpp.
//
// Per-slot driver: the RfScheduler (or the sim test) calls onSlot() to do the scheduled
// air action, then service() after the exchange to drain the PHY and run the state machine.
class Link {
public:
    static constexpr uint8_t  RC_CHANNELS   = 8;   // M3: 8 packed 11-bit channels / RC frame
    static constexpr uint16_t FAILSAFE_MISS = 10;  // consecutive missed uplink slots → Failsafe

    void begin(Role role, const uint8_t uid[8], uint8_t rateIndex, int8_t maxPowerDbm = 10, bool useDynamicPower = true);
    void setRegion(FhssRegion region) { _region = region; }
    void setFailsafeMode(FailsafeMode mode) { _fsMode = mode; }
    void requestRate(uint8_t rateIndex);   // TX: switch rate (conveyed to RX via the Sync beacon)
    void setCipher(ICipher* c) { _cipher = c; }              // opt-in AEAD over the RC payload
    void setSessionSalt(uint32_t salt) { _sessionSalt = salt; }  // negotiated at Connect (real); fixed in sim

    void onTick(uint32_t tick);
    float freqForTick(uint32_t tick) const;
    Slot slotForTick(uint32_t tick) const;
    bool getTxPayload(uint32_t tick, uint16_t pos, uint8_t* outBuf, uint8_t& outLen);
    bool processRxPayload(uint32_t tick, uint16_t pos, const uint8_t* data, uint8_t len, int16_t rssi, int8_t snr);
    void service(uint32_t tick);   // timing-independent timeout, LQ and state machine updater
    void notePhyRecovery(bool success);
    void noteMissedDeadlines(uint16_t n);  // scheduler fell behind by n slots (saturating count)
    void noteSchedulerOverrun(uint16_t n); // scheduler skipped stale slots; RX must re-acquire

    void    setChannels(const uint16_t* ch, uint8_t n);        // TX input (11-bit, 0..2047)
    uint8_t getChannels(uint16_t* ch, uint8_t maxN) const;     // RX output; returns count
    bool    outputActive() const;  // RX: emit CRSF? false = NoPulses failsafe / not connected

    LinkState        state() const { return _state; }
    const LinkStats& stats() const { return _stats; }
    bool             isLocked() const { return _locked; }
    uint16_t         rxPos() const { return _rxPos; }
    uint16_t         txPos(uint32_t tick) const { return (uint16_t)((tick / hopInterval()) % _fhss.count()); }
    uint8_t          tlmRatioDenom() const { return _rate.tlmRatioDenom; }
    uint16_t         syncEveryNTicks() const { return _fhss.count() * hopInterval(); }
    uint16_t         syncWord() const { return _syncWord; }
    int8_t           txPowerDbm() const { return _txPowerDbm; }
    Role             role() const { return _role; }

    bool queueTelemetry(const uint8_t* data, size_t len) { return _tlmSender.queuePayload(data, len); }
    bool getTelemetry(uint8_t* outBuf, size_t& outLen) { return _tlmReceiver.getPayload(outBuf, outLen); }

private:
    enum class SlotKind : uint8_t { Sync, Telemetry, Uplink };
    uint16_t hopInterval() const;
    SlotKind slotKind(uint32_t tick, uint16_t pos) const;
    float    freqForPos(uint16_t pos) const;

    Role         _role      = Role::Rx;
    RateConfig   _rate{};
    uint8_t      _rateIndex = 0;
    uint8_t      _nextRateIndex = 0;
    uint32_t     _rateSwitchCycle = 0;
    uint32_t     _switchCyclesRemaining = 0;
    uint32_t     _tick = 0;
    DynamicPower _power;                // TX-side dynamic power
    uint16_t     _syncWord  = 0;
    ICipher*     _cipher    = nullptr;  // null ⇒ NullCipher (plaintext); opt-in AEAD (M8)
    uint32_t     _sessionSalt = 0;      // nonce salt; negotiated at Connect on real hardware
    Fhss         _fhss;                 // UID-seeded hop sequence (M4)
    FhssRegion   _region   = FhssRegion::US_FCC;
    uint8_t      _uidCrc   = 0;
    bool         _locked   = false;     // RX: hop-locked to the TX sequence?
    bool         _syncSeen = false;
    uint16_t     _rxPos    = 0;         // RX: current sequence position (from Sync beacon)
    FailsafeMode _fsMode   = FailsafeMode::NoPulses;
    LinkState    _state    = LinkState::Disconnected;
    uint16_t     _ch[RC_CHANNELS] = {0};
    uint32_t     _lastRxTick = 0;
    bool         _everRx     = false;
    uint32_t     _txCounter  = 0;
    LqTracker<100> _lq;       // RX uplink LQ (uplink slots only)
    LqTracker<100> _lqDown;   // TX downlink LQ (telemetry slots only)
    LinkStats    _stats{};
    int8_t       _txPowerDbm = 10;
    bool         _gotRcThisTick = false;
    bool         _gotSyncThisTick = false;
    bool         _gotTlmThisTick = false;
    bool         _gotValidUplinkThisTick = false;
    bool         _gotValidTelemetryThisTick = false;
    uint32_t     _consecutiveMissedUplinks = 0;
    uint32_t     _consecutiveMissedTelemetry = 0;
    StubbornSender   _tlmSender;
    StubbornReceiver _tlmReceiver;
    uint8_t          _receivedAckSeq = 0;
    uint8_t          _localAckSeq = 0;
    bool             _txUseCompact = false;     // compact-frame hysteresis state (TX)
    uint16_t         _auxCenteredFrames = 0;    // consecutive RC frames eligible for compact pack
};

} // namespace xlrs
