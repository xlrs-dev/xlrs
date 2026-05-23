// Link — lifecycle + slotted TDM (RC uplink / Sync beacon / telemetry downlink) + FHSS.
// Host-safe core: runs in the native sim against MockPhy and on hardware against
// Sx1280NativePhy. The cipher (M8) layers on top.
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
#include "ota/OtaFrameShrink.h"
#include <cstring>

namespace xlrs {

static NullCipher s_nullCipher;   // default when no cipher is set (plaintext, integrity = PHY CRC)
static constexpr uint8_t BIND_FRAME_MAGIC[3] = {'X', 'B', 1};
static constexpr uint8_t BIND_FRAME_LEN = 1 + sizeof(BIND_FRAME_MAGIC) + LINK_UID_SIZE + 1;

static uint8_t uidCrc8(const uint8_t uid[8]) {
    uint8_t crc = 0xA5;
    for (uint8_t i = 0; i < 8; ++i) {
        crc ^= uid[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

void Link::configureIdentity(const uint8_t uid[LINK_UID_SIZE]) {
    _syncWord = syncWordFromUid(uid);
    _uidCrc = uidCrc8(uid);
    _fhss.generate(fhssSeedFromUid(uid), kNumFhssChannels2g4, kNumFhssChannels2g4);
}

void Link::resetAcquisition(LinkState state) {
    _state = state;
    _locked = false;
    _syncSeen = false;
    _rxPos = 0;
    _lastRxTick = 0;
    _everRx = false;
    _consecutiveMissedUplinks = 0;
    _consecutiveMissedTelemetry = 0;
    _gotRcThisTick = false;
    _gotSyncThisTick = false;
    _gotTlmThisTick = false;
    _gotValidUplinkThisTick = false;
    _gotValidTelemetryThisTick = false;
    _lq.reset();
    _lqDown.reset();
    _stats.lqUp = 0;
    _stats.lqDown = 0;
    _stats.fhssIndex = 0;
}

void Link::begin(Role role, const uint8_t uid[8], uint8_t rateIndex, int8_t maxPowerDbm, bool useDynamicPower) {
    _role     = role;
    if (rateIndex >= kNumRates) rateIndex = 0;
    _rateIndex = rateIndex;
    _nextRateIndex = rateIndex;
    _rateSwitchCycle = 0;
    _switchCyclesRemaining = 0;
    _tick = 0;
    _rate     = kRates[rateIndex];
    configureIdentity(uid);
    _fsMode   = FailsafeMode::NoPulses;
    resetAcquisition(LinkState::Connecting);
    _txCounter  = 0;
    _stats = LinkStats{};
    _stats.rateIndex = _rateIndex;
    for (uint8_t i = 0; i < RC_CHANNELS; ++i) _ch[i] = 1024;   // mid (11-bit)
    _txUseCompact = false;
    _auxCenteredFrames = 0;
    _bindTransmitActive = false;
    _bindScanActive = false;
    _receivedBindUidReady = false;
    _txPowerDbm = maxPowerDbm;
    _stats.txPowerDbm = _txPowerDbm;
    if (_role == Role::Tx) {
        if (useDynamicPower) {
            _power.begin(-18, maxPowerDbm, maxPowerDbm);            // dynamic power starts at max
        } else {
            _power.begin(maxPowerDbm, maxPowerDbm, maxPowerDbm);            // static power at max
        }
    }
}

void Link::setLinkUid(const uint8_t uid[LINK_UID_SIZE]) {
    if (!uid) return;
    configureIdentity(uid);
    _bindTransmitActive = false;
    _bindScanActive = false;
    _receivedBindUidReady = false;
    resetAcquisition(LinkState::Connecting);
}

void Link::startBindTransmit(const uint8_t targetUid[LINK_UID_SIZE]) {
    if (_role != Role::Tx || !targetUid) return;
    uint8_t bindUid[LINK_UID_SIZE];
    bindModeUid(bindUid);
    configureIdentity(bindUid);
    memcpy(_bindTargetUid, targetUid, LINK_UID_SIZE);
    _bindTransmitActive = true;
    _bindScanActive = false;
    resetAcquisition(LinkState::Binding);
}

void Link::startBindScan() {
    if (_role != Role::Rx) return;
    uint8_t bindUid[LINK_UID_SIZE];
    bindModeUid(bindUid);
    configureIdentity(bindUid);
    _bindTransmitActive = false;
    _bindScanActive = true;
    _receivedBindUidReady = false;
    resetAcquisition(LinkState::Binding);
}

bool Link::takeReceivedBindUid(uint8_t uid[LINK_UID_SIZE]) {
    if (!uid || !_receivedBindUidReady) return false;
    memcpy(uid, _receivedBindUid, LINK_UID_SIZE);
    _receivedBindUidReady = false;
    return true;
}

void Link::requestRate(uint8_t rateIndex) {
    if (rateIndex >= kNumRates) return;
    if (rateIndex == _rateIndex) {
        _nextRateIndex = _rateIndex;
        _rateSwitchCycle = 0;
        _switchCyclesRemaining = 0;
        return;
    }
    _nextRateIndex = rateIndex;
    // Schedule by FHSS sequence cycle, not raw local tick. The TX announces a
    // countdown in Sync, and the RX decrements it on its own synchronized pos-0
    // boundaries, so independent boot-time tick offsets do not mistime the switch.
    const uint16_t seqPeriod = hopInterval() * _fhss.count();
    _rateSwitchCycle = (_tick / seqPeriod) + 3;
}

uint16_t Link::hopInterval() const { return _rate.fhssHopInterval ? _rate.fhssHopInterval : 1; }

Link::SlotKind Link::slotKind(uint32_t tick, uint16_t pos) const {
    if (pos == 0) return SlotKind::Sync;
    const uint8_t tlm = _rate.tlmRatioDenom;
    if (tlm && (tick % tlm) == 0) return SlotKind::Telemetry;
    return SlotKind::Uplink;
}

float Link::freqForPos(uint16_t pos) const {
    return fhssFreqForIndex(_fhss.at((uint8_t)(pos % _fhss.count())), _region);
}

void Link::onTick(uint32_t tick) {
    _tick = tick;

    const uint16_t hop = hopInterval();
    const uint8_t seqLen = _fhss.count();
    const bool atHopBoundary = (tick % hop) == 0;

    if (_role == Role::Rx && _locked && atHopBoundary) {
        _rxPos = (uint16_t)((_rxPos + 1) % seqLen);
    }

    const bool atSequenceBoundary = atHopBoundary &&
        ((_role == Role::Tx) ? (txPos(tick) == 0) : (_locked && _rxPos == 0));

    if (_role == Role::Tx) {
        const uint16_t seqPeriod = hop * seqLen;
        const uint32_t cycle = tick / seqPeriod;
        if (_rateSwitchCycle > 0 && atSequenceBoundary && cycle >= _rateSwitchCycle) {
            _rateIndex = _nextRateIndex;
            _rate = kRates[_rateIndex];
            _stats.rateIndex = _rateIndex;
            _rateSwitchCycle = 0;
            _switchCyclesRemaining = 0;
        }
    } else if (_locked && atSequenceBoundary && _switchCyclesRemaining > 0) {
        _switchCyclesRemaining--;
        if (_switchCyclesRemaining == 0 && _nextRateIndex < kNumRates && _nextRateIndex != _rateIndex) {
            _rateIndex = _nextRateIndex;
            _rate = kRates[_rateIndex];
            _stats.rateIndex = _rateIndex;
        }
    }

    _stats.fhssIndex = (_role == Role::Tx) ? (uint8_t)txPos(tick) : (uint8_t)_rxPos;
    _stats.txPowerDbm = _txPowerDbm;
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
        if (_bindTransmitActive) {
            if (k == SlotKind::Sync || k == SlotKind::Uplink) {
                outBuf[0] = otaMakeHeader(OtaType::Bind);
                memcpy(outBuf + 1, BIND_FRAME_MAGIC, sizeof(BIND_FRAME_MAGIC));
                memcpy(outBuf + 1 + sizeof(BIND_FRAME_MAGIC), _bindTargetUid, LINK_UID_SIZE);
                outBuf[BIND_FRAME_LEN - 1] = uidCrc8(_bindTargetUid);
                outLen = BIND_FRAME_LEN;
                ++_txCounter;
                return true;
            }
            return false;
        }
        if (k == SlotKind::Sync) {
            SyncPayload s{};
            s.fhssIndex     = (uint8_t)pos;
            s.rateIndex     = _rateIndex;
            s.nextRateIndex = _nextRateIndex;
            s.switchTick    = 0;
            if (_rateSwitchCycle > 0) {
                const uint16_t seqPeriod = hopInterval() * _fhss.count();
                const uint32_t cycle = tick / seqPeriod;
                s.switchTick = (_rateSwitchCycle > cycle) ? (_rateSwitchCycle - cycle) : 1;
            }
            s.tlmRatioDenom = _rate.tlmRatioDenom;
            s.uidCrc        = _uidCrc;
            outLen = otaEncodeSync(s, outBuf);
            ++_txCounter;
            return true;
        } else if (k == SlotKind::Uplink) {
            ICipher* c = _cipher ? _cipher : &s_nullCipher;

            // Interleave stubborn telemetry (MSP) on 25% of uplink frames if queued
            TelemetryChunk chunk{};
            if (!_tlmSender.idle() && (tick % 4 == 0) && _tlmSender.getNextChunk(chunk)) {
                outBuf[0] = otaMakeHeader(OtaType::Msp);
                outBuf[1] = chunk.seq;
                outBuf[2] = chunk.length;
                memcpy(outBuf + 3, chunk.data, 13);
                c->seal(outBuf + 1, 15, Nonce96::build(_sessionSalt, tick, pos));
                outLen = (uint8_t)(1 + 15 + c->overhead());
                ++_txCounter;
                return true;
            }

            // Otherwise, send standard or compact RC frame. HYSTERESIS on the compact decision:
            // eligibility (aux ch5-7 centered AND all channels even → lossless 10-bit pack) can
            // toggle frame-to-frame as a stick's LSB flips or an aux hovers near center, which
            // would flip compact(10-bit)↔full(11-bit) every frame and jitter the primary channels'
            // resolution. So enter compact only after COMPACT_ENTER_FRAMES consecutive eligible RC
            // frames, and drop to full immediately on the first ineligible frame.
            constexpr uint16_t COMPACT_ENTER_FRAMES = 25;
            bool compactEligible = true;
            for (int i = 0; i < RC_CHANNELS; ++i) {
                if (i >= 5 && _ch[i] != 1024) { compactEligible = false; break; }  // aux must be centered
                if ((_ch[i] & 0x01) != 0)     { compactEligible = false; break; }  // odd → 10-bit loses LSB
            }
            if (!compactEligible) {
                _auxCenteredFrames = 0;
                _txUseCompact = false;                          // instant exit to full
            } else if (_auxCenteredFrames < COMPACT_ENTER_FRAMES) {
                if (++_auxCenteredFrames >= COMPACT_ENTER_FRAMES) _txUseCompact = true;
            }
            bool useCompact = _txUseCompact;

            if (useCompact) {
                // Pack into compact 8-byte frame, piggybacking _localAckSeq in the sequence field
                packChannels8Byte(_ch, _localAckSeq, outBuf);
                c->seal(outBuf + 1, 7, Nonce96::build(_sessionSalt, tick, pos));
                outLen = (uint8_t)(8 + c->overhead());
            } else {
                const uint8_t p = packedSize(RC_CHANNELS);
                otaEncodeRc(_ch, RC_CHANNELS, outBuf);
                c->seal(outBuf + 1, p, Nonce96::build(_sessionSalt, tick, pos));
                outLen = (uint8_t)(1 + p + c->overhead());
            }
            ++_txCounter;
            return true;
        }
        return false;
    } else {
        if (!_locked) return false;
        SlotKind k = slotKind(tick, pos);
        if (k == SlotKind::Telemetry) {
            ICipher* c = _cipher ? _cipher : &s_nullCipher;
            TelemetryChunk chunk{};
            if (!_tlmSender.idle() && _tlmSender.getNextChunk(chunk)) {
                // Send RX->TX stubborn telemetry chunk
                outBuf[0] = otaMakeHeader(OtaType::Msp);
                outBuf[1] = chunk.seq;
                outBuf[2] = chunk.length;
                memcpy(outBuf + 3, chunk.data, 13);
                c->seal(outBuf + 1, 15, Nonce96::build(_sessionSalt, tick, pos));
                outLen = (uint8_t)(1 + 15 + c->overhead());
                return true;
            }

            // Fallback to standard telemetry downlink, appending _localAckSeq as a 5th byte
            outLen = otaEncodeTlmDown(_stats.lqUp, _stats.rssiDbm, _stats.snr, outBuf);
            outBuf[outLen] = _localAckSeq;
            outLen += 1;
            return true;
        }
        return false;
    }
}

bool Link::processRxPayload(uint32_t tick, uint16_t pos, const uint8_t* data, uint8_t len, int16_t rssi, int8_t snr) {
    if (!data || len == 0) return false;

    SyncPayload s{};
    uint16_t rxch[RC_CHANNELS];
    uint8_t  upLq;  int16_t upRssi;  int8_t upSnr;

    if (_role == Role::Rx && _bindScanActive && len >= BIND_FRAME_LEN &&
        otaVersion(data[0]) == OTA_VERSION && otaType(data[0]) == OtaType::Bind) {
        if (memcmp(data + 1, BIND_FRAME_MAGIC, sizeof(BIND_FRAME_MAGIC)) != 0) return false;
        const uint8_t* offeredUid = data + 1 + sizeof(BIND_FRAME_MAGIC);
        if (data[BIND_FRAME_LEN - 1] != uidCrc8(offeredUid)) return false;
        memcpy(_receivedBindUid, offeredUid, LINK_UID_SIZE);
        _receivedBindUidReady = true;
        _gotValidUplinkThisTick = true;
        _lastRxTick = tick;
        _everRx = true;
        _stats.rssiDbm = rssi;
        _stats.snr = snr;
        _state = LinkState::Binding;
        return true;
    }

    if (otaDecodeSync(data, len, s)) {
        if (s.uidCrc != _uidCrc) return false;
        _gotSyncThisTick = true;
        if (_role == Role::Rx) {
            if (!_locked) { _rxPos = s.fhssIndex; _locked = true; }
            _syncSeen = true;
            _lastRxTick = tick; _everRx = true;
            _stats.rssiDbm = rssi; _stats.snr = snr;

            // Handle coordinated rate switch from sync packet
            if (s.switchTick > 0 && s.nextRateIndex < kNumRates) {
                _nextRateIndex = s.nextRateIndex;
                _switchCyclesRemaining = s.switchTick;
            } else {
                _nextRateIndex = s.rateIndex;
                _switchCyclesRemaining = 0;
            }

            // Fallback: If RX boots late or was completely out of sync, adopt TX active rate immediately.
            if (s.rateIndex < kNumRates && s.rateIndex != _rateIndex && s.switchTick == 0) {
                _rateIndex = s.rateIndex; _rate = kRates[s.rateIndex];
                _stats.rateIndex = s.rateIndex;
                _nextRateIndex = s.rateIndex;
                _switchCyclesRemaining = 0;
            }
        }
        return true;
    } else if (otaDecodeTlmDown(data, len, &upLq, &upRssi, &upSnr)) {
        _gotTlmThisTick = true;
        _gotValidTelemetryThisTick = true;
        if (_role == Role::Tx) {                   // RX-reported uplink view
            _stats.lqUp = upLq; _stats.rssiDbm = upRssi; _stats.snr = upSnr;
            _lastRxTick = tick; _everRx = true;
            _txPowerDbm = _power.update(upLq, upRssi);
            if (len >= 5) {
                _receivedAckSeq = data[4];
                _tlmSender.receiveAck(_receivedAckSeq);
            }
        }
        return true;
    } else if (otaType(data[0]) == OtaType::Msp) {
        ICipher* c = _cipher ? _cipher : &s_nullCipher;
        if (len >= (uint8_t)(1 + 15 + c->overhead())) {
            uint8_t decBuf[64];
            memcpy(decBuf, data, len);
            if (c->open(decBuf + 1, 15, Nonce96::build(_sessionSalt, tick, pos))) {
                TelemetryChunk chunk{};
                chunk.seq = decBuf[1];
                chunk.length = decBuf[2];
                memcpy(chunk.data, decBuf + 3, 13);
                uint8_t ackSeq = 0;
                _tlmReceiver.processChunk(chunk, ackSeq);
                _localAckSeq = ackSeq;
                if (_role == Role::Rx && slotKind(tick, pos) == SlotKind::Uplink) {
                    _gotValidUplinkThisTick = true;
                } else if (_role == Role::Tx && slotKind(tick, pos) == SlotKind::Telemetry) {
                    _gotValidTelemetryThisTick = true;
                }
                _lastRxTick = tick; _everRx = true;
                return true;
            }
        }
    } else if (otaType(data[0]) == OtaType::Rc && _role == Role::Rx) {
        if (!_locked || !_syncSeen) return false;
        ICipher* c = _cipher ? _cipher : &s_nullCipher;

        // Try compact 8-byte frame first
        if (len >= (uint8_t)(8 + c->overhead())) {
            uint8_t decBuf[32];
            memcpy(decBuf, data, len);
            if (c->open(decBuf + 1, 7, Nonce96::build(_sessionSalt, tick, pos))) {
                uint8_t decSeq = 0;
                if (unpackChannels8Byte(decBuf, rxch, decSeq)) {
                    _gotRcThisTick = true;
                    _gotValidUplinkThisTick = true;
                    memcpy(_ch, rxch, sizeof(uint16_t) * RC_CHANNELS);
                    _lastRxTick = tick; _everRx = true;
                    _stats.rssiDbm = rssi; _stats.snr = snr;
                    _receivedAckSeq = decSeq;
                    _tlmSender.receiveAck(_receivedAckSeq);
                    return true;
                }
            }
        }

        // Fallback to standard 12-byte frame
        const uint8_t p = packedSize(RC_CHANNELS);
        if (len >= (uint8_t)(1 + p + c->overhead())) {
            uint8_t decBuf[32];
            memcpy(decBuf, data, len);
            if (c->open(decBuf + 1, p, Nonce96::build(_sessionSalt, tick, pos))) {
                if (otaDecodeRc(decBuf, len, rxch, RC_CHANNELS)) {
                    _gotRcThisTick = true;
                    _gotValidUplinkThisTick = true;
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
            _lqDown.update(_gotValidTelemetryThisTick);
            _stats.lqDown = _lqDown.lq();
            if (_gotValidTelemetryThisTick) {
                _consecutiveMissedTelemetry = 0;
            } else {
                _consecutiveMissedTelemetry++;
                if (_stats.telemetryMisses != UINT16_MAX) _stats.telemetryMisses++;
                if (_consecutiveMissedTelemetry >= 2) {
                    _txPowerDbm = _power.update(0, 0, true); // ELRS standard: jump to max power immediately on telemetry loss
                }
            }
        }
    } else {
        // RX uplink LQ: count UPLINK slots only (Sync/Telemetry excluded) — once locked.
        if (_locked && slotKind(tick, _rxPos) == SlotKind::Uplink) {
            _lq.update(_gotValidUplinkThisTick);
            _stats.lqUp = _lq.lq();
        }
    }

    const bool got = _gotRcThisTick || _gotSyncThisTick ||
                     _gotValidUplinkThisTick || _gotValidTelemetryThisTick;
    
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
            if (_role == Role::Rx) {
                if (_locked && _syncSeen && _gotRcThisTick) _state = LinkState::Connected;
            } else if (got) {
                _state = LinkState::Connected;
            }
            break;
        case LinkState::Connected:
            if (misses >= FAILSAFE_MISS) { _state = LinkState::Failsafe; _locked = false; }
            break;
        case LinkState::Failsafe:
            if (_role == Role::Rx) {
                if (_locked && _syncSeen && _gotRcThisTick) _state = LinkState::Connected;
            } else if (got) {
                _state = LinkState::Connected;
            }
            break;
        case LinkState::Binding:
            break;
    }

    // Reset tick flags
    _gotRcThisTick = false;
    _gotSyncThisTick = false;
    _gotTlmThisTick = false;
    _gotValidUplinkThisTick = false;
    _gotValidTelemetryThisTick = false;
    _stats.txPowerDbm = _txPowerDbm;
}

void Link::notePhyRecovery(bool success) {
    if (success) {
        if (_stats.phyRecoveries != UINT16_MAX) _stats.phyRecoveries++;
    } else {
        if (_stats.phyRecoveryFailures != UINT16_MAX) _stats.phyRecoveryFailures++;
        if (_role == Role::Rx) {
            _state = LinkState::Failsafe;
            _locked = false;
            _syncSeen = false;
        }
    }
}

void Link::noteMissedDeadlines(uint16_t n) {
    uint32_t v = (uint32_t)_stats.missedDeadlines + n;
    _stats.missedDeadlines = (v > UINT16_MAX) ? UINT16_MAX : (uint16_t)v;
}

void Link::noteSchedulerOverrun(uint16_t n) {
    noteMissedDeadlines(n);
    if (_role == Role::Rx) {
        _state = LinkState::Connecting;
        _locked = false;
        _syncSeen = false;
        _rxPos = 0;
        _consecutiveMissedUplinks = 0;
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
    if (_state == LinkState::Connected && _locked && _syncSeen) return true;
    if (_state == LinkState::Failsafe && _fsMode == FailsafeMode::Hold) return true;
    return false;   // NoPulses (default): stop CRSF when not connected
}

} // namespace xlrs
