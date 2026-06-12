// DynamicPower — LQ-first, RSSI-second TX power control with hysteresis (M6).
//
// Per docs/developer/configuration.md: link quality is the primary signal (raise power when LQ sags), RSSI is
// the secondary gate (only reduce power when LQ is high AND there's RSSI margin). Hysteresis
// (N consecutive readings before a step, plus a deadband between the raise/lower thresholds)
// prevents oscillation — RSSI-only control would be weak and twitchy. Pure / host-testable.
#pragma once
#include <stdint.h>

namespace xlrs {

class DynamicPower {
public:
    void begin(int8_t minDbm, int8_t maxDbm, int8_t startDbm) {
        _min = minDbm; _max = maxDbm; _dbm = startDbm; _up = 0; _down = 0;
    }

    // Feed periodically with the current uplink LQ (0..100) and RSSI (dBm). Returns new power.
    int8_t update(uint8_t lq, int16_t rssiDbm, bool telemetryLost = false) {
        if (telemetryLost || lq < CRITICAL_LQ) {
            _up = 0; _down = 0;
            _dbm = _max;  // ELRS safety standard: instant jump to max power
            return _dbm;
        }

        if (lq < WEAK_LQ) {                                    // struggling → raise power
            _down = 0;
            if (++_up >= HYST) { _dbm = clamp(_dbm + STEP); _up = 0; }
        } else if (lq >= STRONG_LQ && rssiDbm > STRONG_RSSI) { // strong + margin → lower power
            _up = 0;
            if (++_down >= HYST) { _dbm = clamp(_dbm - STEP); _down = 0; }
        } else {                                               // deadband → hold
            _up = 0; _down = 0;
        }
        return _dbm;
    }

    int8_t powerDbm() const { return _dbm; }

private:
    static constexpr uint8_t CRITICAL_LQ = 50;
    static constexpr uint8_t WEAK_LQ     = 80;
    static constexpr uint8_t STRONG_LQ   = 99;
    static constexpr uint8_t HYST        = 3;
    static constexpr int16_t STRONG_RSSI = -65;
    static constexpr int8_t  STEP        = 3;

    int8_t clamp(int v) const { return v < _min ? _min : (v > _max ? _max : (int8_t)v); }

    int8_t  _min = 0, _max = 10, _dbm = 10;
    uint8_t _up = 0, _down = 0;
};

} // namespace xlrs
