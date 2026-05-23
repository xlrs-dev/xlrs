// HwTimer — periodic tick source for the RF scheduler (M2 contract).
//
// The RF scheduler runs off a hardware timer firing every RateConfig.intervalUs. On
// RP2040/RP2350 this is a Pico SDK repeating alarm bound to core 1; its ISR latches the
// tick time and signals the RF task. The PFD nudges the period via setIntervalUs() to keep
// the RX timer locked to the TX.
//
// The RP2040 implementation and the PFD-driven lock are validated on hardware for jitter,
// convergence, and drift acceptance (architecture.md §M2). In the native sim the
// test loop drives ticks directly (synchronous delivery ⇒ measured offset ≈ 0), so the timer
// impl isn't needed there; the PFD math itself is verified (timing/Pfd.h).
#pragma once
#include <stdint.h>

namespace xlrs {

using TimerTickCallback = void (*)();

class HwTimer {
public:
    virtual ~HwTimer() = default;
    virtual bool     begin(uint32_t intervalUs, TimerTickCallback onTick) = 0;
    virtual void     setIntervalUs(uint32_t intervalUs) = 0;  // PFD nudges this each period
    virtual uint32_t nowUs() const = 0;
    virtual void     stop() = 0;
};

HwTimer* createHwTimer();

#if !defined(XLRS_PICO_SDK) && !defined(PICO_BOARD)
void setSimulatedTimeUs(uint32_t us);
uint32_t getSimulatedIntervalUs();
// Sim-only: invoke the registered timer callback once (mirrors one hardware timer ISR), so a
// test can drive the real poll() event path instead of calling onTick()/service() directly.
void fireSimTimerTick();
#endif

} // namespace xlrs
