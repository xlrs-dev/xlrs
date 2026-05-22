// Link — lifecycle + slotted TDM (RC uplink / Sync beacon / telemetry downlink) + FHSS.
// Host-safe (no Arduino): runs in the native sim against MockPhy and on hardware against
// RadioLibPhy. The cipher (M8) layers on top.
//
// Slotting (both ends agree per tick): pos = (tick/hop) % seqLen.
//   pos == 0                       → Sync slot      (TX→RX beacon, on the acquisition channel)
//   else tick % tlmRatioDenom == 0 → Telemetry slot (RX→TX downlink)
//   else                           → Uplink slot    (TX→RC→RX)
// Each slot has exactly one sender and one receiver, both computed identically once locked —
// so uplink and downlink never collide (no holdoff hack). Driver is 3-phase: slotArm (all
// receivers arm), slotSend (all senders transmit), service (all drain) — guaranteeing the
// receiver is listening before the sender transmits, regardless of slot direction.
#include "link/Link.h"
#include "link/Uid.h"
#include "ota/OtaCodec.h"
#include "fhss/channels_2g4.h"
#include <string.h>

namespace xlrs {

static NullCipher s_nullCipher;   // default when no cipher is set (plaintext, integrity = PHY CRC)

void Link::begin(Role role, const uint8_t uid[8], uint8_t rateIndex, int8_t maxPowerDbm, bool useDynamicPower) {
    _role     = role;
    if (rateIndex >= kNumRates) rateIndex = 0;
    _rateIndex = rateIndex;
    _nextRateIndex = rateIndex;
    _rateSwitchTick = 0;
    _tick = 0;
    _rate     = kRates[rateIndex];
    _syncWord = syncWordFromUid(uid);
    _fhss.generate(fhssSeedFromUid(uid), kNumFhssChannels2g4, kNumFhssChannels2g4);
    _fsMode   = FailsafeMode::NoPulses;
    _state    = LinkState::Connecting;
    _locked   = false;
    _rxPos    = 0;
    _lastRxTick = 0;
    _everRx     = false;
    _txCounter  = 0;
    _consecutiveMissedUplinks = 0;
    _consecutiveMissedTelemetry = 0;
    _lq.reset();
    _lqDown.reset();
    _stats = LinkStats{};
    _stats.rateIndex = _rateIndex;
    for (uint8_t i = 0; i < RC_CHANNELS; ++i) _ch[i] = 1024;   // mid (11-bit)
    _txPowerDbm = maxPowerDbm;
    if (_role == Role::Tx) {
        if (useDynamicPower) {
            _power.begin(-18, maxPowerDbm, maxPowerDbm);            // dynamic power starts at max
        } else {
            _power.begin(maxPowerDbm, maxPowerDbm, maxPowerDbm);            // static power at max
        }
    }
}

void Link::requestRate(uint8_t rateIndex) {
    if (rateIndex >= kNumRates) return;
    if (rateIndex == _rateIndex) {
        _nextRateIndex = _rateIndex;
        _rateSwitchTick = 0;
        return;
    }
    _nextRateIndex = rateIndex;
    // Schedule the switch to happen in the future, aligned to an FHSS sequence boundary (pos == 0).
    // An FHSS sequence period in ticks is hopInterval() * seqLen.
    const uint16_t seqPeriod = hopInterval() * _fhss.count();
    _rateSwitchTick = ((_tick / seqPeriod) + 3) * seqPeriod;
}

uint16_t Link::hopInterval() const { return _rate.fhssHopInterval ? _rate.fhssHopInterval : 1; }

Link::SlotKind Link::slotKind(uint32_t tick, uint16_t pos) const {
    if (pos == 0) return SlotKind::Sync;
    const uint8_t tlm = _rate.tlmRatioDenom;
    if (tlm && (tick % tlm) == 0) return SlotKind::Telemetry;
    return SlotKind::Uplink;
}

float Link::freqForPos(uint16_t pos) const {
    return fhssFreqForIndex(_fhss.at((uint8_t)(pos % _fhss.count())));
}

void Link::onTick(uint32_t tick) {
    _tick = tick;
    
    // Check if scheduled rate switch activation boundary is reached
    if (_rateSwitchTick > 0 && tick >= _rateSwitchTick) {
        _rateIndex = _nextRateIndex;
        _rate = kRates[_rateIndex];
        _stats.rateIndex = _rateIndex;
        _rateSwitchTick = 0;
    }

    const uint16_t hop = hopInterval();
    const uint8_t seqLen = _fhss.count();
    if (_role == Role::Rx && _locked && (tick % hop) == 0) {
        _rxPos = (uint16_t)((_rxPos + 1) % seqLen);
    }
}

float Link::freqForTick(uint32_t tick) const {
    if (_role == Role::Tx) {
        return freqForPos(txPos(tick));
    } else {
        if (!_locked) return freqForPos(0);
        return freqForPos(_rxPos);
    }
}

Slot Link::slotForTick(uint32_t tick) const {
    const uint16_t hop = hopInterval();
    const uint8_t seqLen = _fhss.count();
    uint16_t pos = (_role == Role::Tx) ? (uint16_t)((tick / hop) % seqLen) : _rxPos;
    if (pos == 0) return Slot::Sync;
    if (_rate.tlmRatioDenom && (tick % _rate.tlmRatioDenom) == 0) return Slot::Telemetry;
    return Slot::Uplink;
}

bool Link::getTxPayload(uint32_t tick, uint16_t pos, uint8_t* outBuf, uint8_t& outLen) {
    if (_role == Role::Tx) {
        SlotKind k = slotKind(tick, pos);
        if (k == SlotKind::Sync) {
            SyncPayload s{};
            s.fhssIndex     = (uint8_t)pos;
            s.rateIndex     = _rateIndex;
            s.nextRateIndex = _nextRateIndex;
            s.switchTick    = _rateSwitchTick;
            s.tlmRatioDenom = _rate.tlmRatioDenom;
            s.uidCrc        = 0;
            outLen = otaEncodeSync(s, outBuf);
            ++_txCounter;
            return true;
        } else if (k == SlotKind::Uplink) {
            ICipher* c = _cipher ? _cipher : &s_nullCipher;
            const uint8_t p = packedSize(RC_CHANNELS);
            otaEncodeRc(_ch, RC_CHANNELS, outBuf);                  // outBuf[0]=hdr, outBuf[1..p]=payload
            c->seal(outBuf + 1, p, Nonce96::build(_sessionSalt, tick, pos));
            outLen = (uint8_t)(1 + p + c->overhead());
            ++_txCounter;
            return true;
        }
        return false;
    } else {
        if (!_locked) return false;
        if (slotKind(tick, pos) == SlotKind::Telemetry) {
            outLen = otaEncodeTlmDown(_stats.lqUp, _stats.rssiDbm, _stats.snr, outBuf);
            return true;
        }
        return false;
    }
}

bool Link::processRxPayload(uint32_t tick, uint16_t pos, const uint8_t* data, uint8_t len, int16_t rssi, int8_t snr) {
    SyncPayload s{};
    uint16_t rxch[RC_CHANNELS];
    uint8_t  upLq;  int16_t upRssi;  int8_t upSnr;

    if (otaDecodeSync(data, len, s)) {
        _gotSyncThisTick = true;
        if (_role == Role::Rx) {
            if (!_locked) { _rxPos = s.fhssIndex; _locked = true; }
            _lastRxTick = tick; _everRx = true;
            _stats.rssiDbm = rssi; _stats.snr = snr;

            // Handle coordinated rate switch from sync packet
            if (s.switchTick > 0 && s.nextRateIndex < kNumRates) {
                _nextRateIndex = s.nextRateIndex;
                _rateSwitchTick = s.switchTick;
            } else {
                _nextRateIndex = s.rateIndex;
                _rateSwitchTick = 0;
            }

            // Fallback: If RX boots late or was completely out of sync, adopt TX active rate immediately
            if (s.rateIndex < kNumRates && s.rateIndex != _rateIndex && s.switchTick <= tick) {
                _rateIndex = s.rateIndex; _rate = kRates[s.rateIndex];
                _stats.rateIndex = s.rateIndex;
                _nextRateIndex = s.rateIndex;
                _rateSwitchTick = 0;
            }
        }
        return true;
    } else if (otaDecodeTlmDown(data, len, &upLq, &upRssi, &upSnr)) {
        _gotTlmThisTick = true;
        if (_role == Role::Tx) {                   // RX-reported uplink view
            _stats.lqUp = upLq; _stats.rssiDbm = upRssi; _stats.snr = upSnr;
            _txPowerDbm = _power.update(upLq, upRssi);
        }
        return true;
    } else if (otaType(data[0]) == OtaType::Rc && _role == Role::Rx) {
        ICipher* c = _cipher ? _cipher : &s_nullCipher;
        const uint8_t p = packedSize(RC_CHANNELS);
        if (len >= (uint8_t)(1 + p + c->overhead())) {
            uint8_t decBuf[32];
            memcpy(decBuf, data, len);
            if (c->open(decBuf + 1, p, Nonce96::build(_sessionSalt, tick, pos))) {
                if (otaDecodeRc(decBuf, len, rxch, RC_CHANNELS)) {
                    _gotRcThisTick = true;
                    memcpy(_ch, rxch, sizeof(uint16_t) * RC_CHANNELS);
                    _lastRxTick = tick; _everRx = true;
                    _stats.rssiDbm = rssi; _stats.snr = snr;
                    return true;
                }
            }
        }
    }
    return false;
}

void Link::service(uint32_t tick) {
    const uint16_t hop    = hopInterval();
    const uint8_t  seqLen = _fhss.count();

    if (_role == Role::Tx) {
        const uint16_t txPos = (uint16_t)((tick / hop) % seqLen);
        if (slotKind(tick, txPos) == SlotKind::Telemetry) {     // downlink LQ over tlm slots
            _lqDown.update(_gotTlmThisTick);
            _stats.lqDown = _lqDown.lq();
            if (_gotTlmThisTick) {
                _consecutiveMissedTelemetry = 0;
            } else {
                _consecutiveMissedTelemetry++;
                if (_consecutiveMissedTelemetry >= 2) {
                    _txPowerDbm = _power.update(0, 0, true); // ELRS standard: jump to max power immediately on telemetry loss
                }
            }
        }
    } else {
        // RX uplink LQ: count UPLINK slots only (Sync/Telemetry excluded) — once locked.
        if (_locked && slotKind(tick, _rxPos) == SlotKind::Uplink) {
            _lq.update(_gotRcThisTick);
            _stats.lqUp = _lq.lq();
        }
    }

    const bool     got    = _gotRcThisTick || _gotSyncThisTick;
    
    if (got) {
        _consecutiveMissedUplinks = 0;
    } else if (_role == Role::Rx && _locked && slotKind(tick, _rxPos) == SlotKind::Uplink) {
        _consecutiveMissedUplinks++;
    }

    const uint32_t misses = (_role == Role::Rx) 
                            ? _consecutiveMissedUplinks 
                            : (_everRx ? (tick - _lastRxTick) : (FAILSAFE_MISS + 1));
    switch (_state) {
        case LinkState::Disconnected:
        case LinkState::Connecting:
            if (got) _state = LinkState::Connected;
            break;
        case LinkState::Connected:
            if (misses >= FAILSAFE_MISS) { _state = LinkState::Failsafe; _locked = false; }
            break;
        case LinkState::Failsafe:
            if (got) _state = LinkState::Connected;
            break;
        case LinkState::Binding:
            break;
    }

    // Reset tick flags
    _gotRcThisTick = false;
    _gotSyncThisTick = false;
    _gotTlmThisTick = false;
}

void Link::notePhyRecovery(bool success) {
    if (success) {
        if (_stats.phyRecoveries != UINT16_MAX) _stats.phyRecoveries++;
    } else {
        if (_stats.phyRecoveryFailures != UINT16_MAX) _stats.phyRecoveryFailures++;
        if (_role == Role::Rx) {
            _state = LinkState::Failsafe;
            _locked = false;
        }
    }
}

void Link::setChannels(const uint16_t* ch, uint8_t n) {
    if (!ch) return;
    if (n > RC_CHANNELS) n = RC_CHANNELS;
    for (uint8_t i = 0; i < n; ++i) _ch[i] = ch[i] & 0x7FF;
}

uint8_t Link::getChannels(uint16_t* ch, uint8_t maxN) const {
    if (!ch) return 0;
    uint8_t n = (maxN < RC_CHANNELS) ? maxN : RC_CHANNELS;
    for (uint8_t i = 0; i < n; ++i) ch[i] = _ch[i];
    return n;
}

bool Link::outputActive() const {
    if (_role != Role::Rx) return false;
    if (_state == LinkState::Connected) return true;
    if (_state == LinkState::Failsafe && _fsMode == FailsafeMode::Hold) return true;
    return false;   // NoPulses (default): stop CRSF when not connected
}

} // namespace xlrs
