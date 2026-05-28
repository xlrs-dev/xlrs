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
    int16_t  downlinkRssiDbm; // TX-local RSSI of RX->TX downlink telemetry packets
    int8_t   downlinkSnr;     // TX-local SNR of RX->TX downlink telemetry packets
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
    static constexpr uint16_t FAILSAFE_MISS = 10;  // consecutive missed uplink (RX) or telemetry (TX) slots
    // Sliding-window LQ above this suppresses consecutive-miss failsafe (async HW decode skew).
    static constexpr uint8_t  FAILSAFE_LQ_HOLDOFF = 25;
    // Post-connect grace: flaky bench links need time for PFD/phase to converge before failsafe.
    static constexpr uint16_t FAILSAFE_GRACE_TELEMETRY_SLOTS = 96;
    static constexpr uint16_t FAILSAFE_GRACE_UPLINK_SLOTS    = 48;
    // Valid uplink frames required before RX leaves Failsafe (prevents 3↔4 bounce).
    static constexpr uint8_t  FAILSAFE_RECOVERY_UPLINKS = 2;

    void begin(Role role, const uint8_t uid[8], uint8_t rateIndex, int8_t maxPowerDbm = 10, bool useDynamicPower = true);
    void setRegion(FhssRegion region) { _region = region; }
    void setFailsafeMode(FailsafeMode mode) { _fsMode = mode; }
    // Bench TX: stay Connected from downlink telemetry alone (no controller RC required).
    void setBenchTxMode(bool on) { _benchTxMode = on; }
    bool benchTxMode() const { return _benchTxMode; }
    void requestRate(uint8_t rateIndex);   // TX: switch rate (conveyed to RX via the Sync beacon)
    void setCipher(ICipher* c) { _cipher = c; }              // opt-in AEAD over the RC payload
    void setSessionSalt(uint32_t salt) { _sessionSalt = salt; }  // negotiated at Connect (real); fixed in sim
    void setLinkUid(const uint8_t uid[LINK_UID_SIZE]);
    void startBindTransmit(const uint8_t targetUid[LINK_UID_SIZE]);
    void startBindScan();
    // Leave bind-scan listen mode and restore the bound identity without wiping acquisition.
    void endBindScan(const uint8_t uid[LINK_UID_SIZE]);
    bool bindTransmitActive() const { return _bindTransmitActive; }
    bool bindScanActive() const { return _bindScanActive; }
    bool takeReceivedBindUid(uint8_t uid[LINK_UID_SIZE]);
    // RX: one-shot TX tick from the latest acquisition Sync (scheduler resync).
    bool takeSyncResyncTick(uint32_t& txTick, bool& resetPfd);
    // RX: set when Connected→Failsafe so the scheduler restores nominal timer/PFD state.
    bool takeRxTimingResetPending();
    void snapSchedulerTick(uint32_t txTick, uint32_t localSchedulerTick);

    void onTick(uint32_t tick);
    float freqForTick(uint32_t tick) const;
    Slot slotForTick(uint32_t tick) const;
    bool getTxPayload(uint32_t tick, uint16_t pos, uint8_t* outBuf, uint8_t& outLen);
    bool processRxPayload(uint32_t tick, uint16_t pos, const uint8_t* data, uint8_t len,
                          int16_t rssi, int8_t snr, uint32_t rxPacketStartUs = 0);
    // True after processRxPayload() when this RX packet carried a valid uplink/RC frame.
    bool decodedValidUplinkThisRx() const { return _decodedValidUplinkThisRx; }
    // RX: true when a valid RC/uplink decode landed within ~2 packet periods.
    bool hasFreshRc() const;
    void service(uint32_t tick, bool recordLq = true);
    void refreshRxPosForTick(uint32_t localTick);
#if defined(XLRS_PICO_SDK)
    void advanceHwUplinkLqSlot(uint32_t tick);
    // TOCK: close tick's uplink LQ slot after pending RX for that tick is drained.
    void closeHwUplinkLqSlotAtTock(uint32_t tick);
    void noteHwUplinkDecode(uint32_t tick, bool received);
    void advanceHwTelemetryLqSlot(uint32_t tick);
    void noteHwTelemetryDecode(uint32_t tick, bool received);
#endif
    void notePhyRecovery(bool success);
    void noteMissedDeadlines(uint16_t n);  // scheduler fell behind by n slots (saturating count)
    void noteSchedulerOverrun(uint16_t n); // scheduler skipped stale slots; RX must re-acquire

    void    setChannels(const uint16_t* ch, uint8_t n);        // TX input (11-bit, 0..2047)
    uint8_t getChannels(uint16_t* ch, uint8_t maxN) const;     // RX output; returns count
    bool    outputActive() const;  // RX: emit CRSF? false = NoPulses failsafe / not connected

    LinkState        state() const { return _state; }
    const LinkStats& stats() const { return _stats; }
    bool             isLocked() const { return _locked; }
    bool             syncSeen() const { return _syncSeen; }
    int8_t           syncFhssSkew() const { return _syncFhssSkew; }
    uint32_t         lastRxTick() const { return _lastRxTick; }
    uint16_t         rxPos() const { return _rxPos; }
    uint16_t         txPos(uint32_t tick) const { return (uint16_t)((tick / hopInterval()) % _fhss.count()); }
    uint32_t         effectiveTxTick(uint32_t localTick) const;
    uint8_t          tlmRatioDenom() const { return _rate.tlmRatioDenom; }
    uint16_t         syncEveryNTicks() const { return _fhss.count() * hopInterval(); }
    uint16_t         syncWord() const { return _syncWord; }
    uint32_t         identityRevision() const { return _identityRevision; }
    int8_t           txPowerDbm() const { return _txPowerDbm; }
    Role             role() const { return _role; }

    bool queueTelemetry(const uint8_t* data, size_t len) { return _tlmSender.queuePayload(data, len); }
    bool getTelemetry(uint8_t* outBuf, size_t& outLen) { return _tlmReceiver.getPayload(outBuf, outLen); }

private:
    enum class SlotKind : uint8_t { Sync, Telemetry, Uplink };
    uint16_t hopInterval() const;
    SlotKind slotKind(uint32_t tick, uint16_t pos) const;
    float    freqForPos(uint16_t pos) const;
    uint32_t txTickForNonce(uint32_t rxPacketStartUs) const;
    void     configureIdentity(const uint8_t uid[LINK_UID_SIZE]);
    void     resetAcquisition(LinkState state);
#if defined(XLRS_PICO_SDK)
    void     finalizeHwUplinkLqSlot(uint32_t tick, bool received);
#endif
    void     onRxEnterConnected();

    Role         _role      = Role::Rx;
    RateConfig   _rate{};
    uint8_t      _rateIndex = 0;
    uint8_t      _nextRateIndex = 0;
    uint32_t     _rateSwitchCycle = 0;
    uint32_t     _switchCyclesRemaining = 0;
    uint32_t     _tick = 0;
    DynamicPower _power;                // TX-side dynamic power
    uint16_t     _syncWord  = 0;
    uint32_t     _identityRevision = 0;
    ICipher*     _cipher    = nullptr;  // null ⇒ NullCipher (plaintext); opt-in AEAD (M8)
    uint32_t     _sessionSalt = 0;      // nonce salt; negotiated at Connect on real hardware
    Fhss         _fhss;                 // UID-seeded hop sequence (M4)
    FhssRegion   _region   = FhssRegion::US_FCC;
    uint8_t      _uidCrc   = 0;
    bool         _locked   = false;     // RX: hop-locked to the TX sequence?
    bool         _syncSeen = false;
    int8_t       _syncFhssSkew = 0;
    uint16_t     _rxPos    = 0;         // RX: hop index (tick-derived when locked)
    bool         _syncResyncResetPfd = false;
    uint32_t     _syncAnchorTxTick = 0;
    uint32_t     _syncAnchorLocalTick = 0;
    FailsafeMode _fsMode   = FailsafeMode::NoPulses;
    bool         _benchTxMode = false;
    LinkState    _state    = LinkState::Disconnected;
    uint16_t     _ch[RC_CHANNELS] = {0};
    uint32_t     _lastRxTick = 0;
    uint32_t     _lastValidRcTick = 0;
#if defined(XLRS_PICO_SDK)
    uint32_t     _lastValidRcMs = 0;
#endif
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
    bool         _decodedValidUplinkThisRx = false;  // per-packet; cleared in processRxPayload()
    uint32_t     _consecutiveMissedUplinks = 0;
    uint32_t     _consecutiveMissedTelemetry = 0;
    uint16_t     _telemetryGraceSlotsRemaining = 0;
    uint16_t     _uplinkGraceSlotsRemaining = 0;
    uint8_t      _failsafeRecoveryUplinks = 0;
    uint32_t     _failsafeRecoveryLastUplinkTick = 0;
#if defined(XLRS_PICO_SDK)
    uint32_t     _failsafeEnteredMs = 0;
#endif
    StubbornSender   _tlmSender;
    StubbornReceiver _tlmReceiver;
    uint8_t          _receivedAckSeq = 0;
    uint8_t          _localAckSeq = 0;
    bool             _txUseCompact = false;     // compact-frame hysteresis state (TX)
    uint16_t         _auxCenteredFrames = 0;    // consecutive RC frames eligible for compact pack
    bool             _bindTransmitActive = false;
    bool             _bindScanActive = false;
    uint8_t          _bindTargetUid[LINK_UID_SIZE] = {};
    uint8_t          _receivedBindUid[LINK_UID_SIZE] = {};
    bool             _receivedBindUidReady = false;
    bool             _syncResyncPending = false;
    uint32_t         _pendingTxTickResync = 0;
    bool             _rxTimingResetPending = false;
#if defined(XLRS_PICO_SDK)
    uint32_t         _hwUplinkLqTick = UINT32_MAX;
    bool             _hwUplinkLqClosed = true;
    uint32_t         _hwUplinkDecodeTick = UINT32_MAX;
    bool             _hwUplinkDecodeOk = false;
    uint32_t         _hwTlmLqTick = UINT32_MAX;
    bool             _hwTlmLqClosed = true;
    uint32_t         _hwTlmDecodeTick = UINT32_MAX;
    bool             _hwTlmDecodeOk = false;
#endif
};

} // namespace xlrs
