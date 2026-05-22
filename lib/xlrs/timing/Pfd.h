// Pfd — phase-frequency detector for RX→TX timer locking.
//
// The RX hardware timer should fire exactly when a packet is expected. After each
// reception we measure the offset (us) between the actual arrival and the timer tick and
// feed it here; update() returns the correction to apply to the next timer period.
//
// PI loop (Kp = 1/4, Ki = 1/256, with anti-windup):
//   - Proportional term locks PHASE.
//   - Integral term tracks the persistent crystal FREQUENCY mismatch (±10–30 ppm typical).
//     A proportional-only loop would settle at a constant steady-state offset of
//     ~4 × drift-per-interval; the integral drives that to ~0.
// Lock quality ultimately depends on oscillator stability (a TCXO helps).
//
// Pure integer math — host-testable, no platform intrinsics.
#pragma once
#include <stdint.h>

namespace xlrs {

class Pfd {
public:
    void begin(int32_t intervalUs) {
        _interval  = intervalUs ? intervalUs : 1;
        _phaseErr  = 0;
        _freqAccum = 0;
    }

    // offsetUs = actual arrival - expected tick. Returns timer adjustment for next period.
    int32_t update(int32_t offsetUs) {
        const int32_t half = _interval / 2;            // wrap into [-half, half]
        while (offsetUs >  half) offsetUs -= _interval;
        while (offsetUs < -half) offsetUs += _interval;
        _phaseErr = offsetUs;

        _freqAccum += offsetUs;                         // integral tracks freq mismatch
        const int32_t cap = _interval * 4;              // anti-windup clamp
        if (_freqAccum >  cap) _freqAccum =  cap;
        if (_freqAccum < -cap) _freqAccum = -cap;

        return (offsetUs / 4) + (_freqAccum / 256);    // PI: Kp = 1/4, Ki = 1/256
    }

    int32_t phaseError() const { return _phaseErr; }

private:
    int32_t _interval  = 4000;
    int32_t _phaseErr  = 0;
    int32_t _freqAccum = 0;   // integral state — tracks crystal frequency mismatch
};

} // namespace xlrs
