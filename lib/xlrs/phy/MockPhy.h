// MockPhy — in-memory IRadioPhy test double (host-testable, no hardware).
//
// Enables sim-testing the entire link (RfScheduler, Link, FHSS, telemetry, failsafe)
// without an SX1280: connect two MockPhys and a TX delivers to the peer's RX *only when
// the peer is listening on the same frequency* — which models FHSS channel isolation, so
// hop desync shows up as packet loss in the sim exactly as it would on air.
#pragma once
#include <stdint.h>
#include <string.h>
#include "phy/IRadioPhy.h"

namespace xlrs {

class MockPhy : public IRadioPhy {
public:
    bool init(const PhyConfig& cfg) override { _cfg = cfg; return true; }

    void startRx(float freqMHz) override { _rxFreq = freqMHz; _listening = true; }

    void startTx(float freqMHz, const uint8_t* data, uint8_t len) override {
        _listening = false;
        // Deliver only if the peer is listening on the same frequency AND has a matching
        // sync word — models both FHSS channel isolation and SX1280 sync-word filtering, so
        // a mismatched-phrase link (different UID → different sync word) is rejected.
        if (_peer && _peer->_listening && _peer->_rxFreq == freqMHz &&
            _peer->_cfg.syncWord == _cfg.syncWord) {
            _peer->deliver(data, len, _rssiDbm, _snr, _clockUs);
        }
        if (_onTxDone) _onTxDone();
    }

    bool readRx(RxPacket& out) override {
        if (!_hasRx) return false;
        out = _rx;
        _hasRx = false;
        return true;
    }

    void setOutputPowerDbm(int8_t dbm) override { _cfg.powerDbm = dbm; }
    bool setSyncWord(uint16_t w) override {
        if (_syncWordFailuresRemaining > 0) {
            --_syncWordFailuresRemaining;
            return false;
        }
        _cfg.syncWord = w;
        return true;
    }
    void reconfigure(const PhyConfig& cfg) override { _cfg = cfg; }
    uint32_t txLatencyUs() const override { return 0; }
    bool txInProgress() const override { return false; }
    bool healthy() const override { return !_forceFault; }
    bool recover() override {
        if (_recoverFailsRemaining > 0) { --_recoverFailsRemaining; return false; }  // models a radio that needs retries
        _forceFault = false; _listening = false; _hasRx = false; return true;
    }
    void setOnTxDone(PhyIsrCallback cb) override { _onTxDone = cb; }
    void setOnRxDone(PhyIsrCallback cb) override { _onRxDone = cb; }

    // ---- test controls ----
    static void connect(MockPhy& a, MockPhy& b) { a._peer = &b; b._peer = &a; }
    void setClockUs(uint32_t t)  { _clockUs = t; }   // arrival timestamp the peer will see
    void setRssi(int16_t r)      { _rssiDbm = r; }
    void setSnr(int8_t s)        { _snr = s; }
    void forceFault()            { _forceFault = true; }
    void setRecoveryFailures(int n) { _recoverFailsRemaining = n; }  // recover() fails n times, then succeeds
    void setSyncWordFailures(int n) { _syncWordFailuresRemaining = n; }
    void corruptNextDeliveries(int n) { _corruptRemaining = n; }     // flip a byte on the next n incoming frames (on-air bit errors)
    bool listening() const       { return _listening; }
    int8_t outputPowerDbm() const { return _cfg.powerDbm; }   // observe dynamic-power output

private:
    void deliver(const uint8_t* d, uint8_t len, int16_t rssi, int8_t snr, uint32_t ts) {
        _rx.len = len > sizeof(_rx.data) ? (uint8_t)sizeof(_rx.data) : len;
        memcpy(_rx.data, d, _rx.len);
        if (_corruptRemaining > 0 && _rx.len > 0) {
            _rx.data[0] ^= 0xFF;                     // simulate an on-air bit error → CRC/AEAD must reject
            --_corruptRemaining;
        }
        _rx.rssiDbm = rssi;
        _rx.snr = snr;
        _rx.timestampUs = ts;
        _hasRx = true;
        _listening = false;                      // one packet, then re-arm needed
        if (_onRxDone) _onRxDone();
    }

    PhyConfig _cfg{};
    MockPhy*  _peer = nullptr;
    bool      _listening = false;
    bool      _hasRx = false;
    bool      _forceFault = false;
    int       _recoverFailsRemaining = 0;
    int       _syncWordFailuresRemaining = 0;
    int       _corruptRemaining = 0;
    float     _rxFreq = 0.0f;
    uint32_t  _clockUs = 0;
    int16_t   _rssiDbm = -50;
    int8_t    _snr = 0;
    RxPacket  _rx{};
    PhyIsrCallback _onTxDone = nullptr;
    PhyIsrCallback _onRxDone = nullptr;
};

} // namespace xlrs
