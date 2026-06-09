// LinkStatusLed — GPIO status LED patterns for TX/RX link-state bench debugging.
//
// Output on STATUS_LED_PIN (CMake: XLRS_STATUS_LED_PIN, default GP10 / Pico pin 13).
// Polarity: XLRS_STATUS_LED_ACTIVE_LOW (default ON = active low, LED sinks to GPIO).
// Core 0 only — do not call from the RF core.
#pragma once

#include <stdint.h>

#include "link/Link.h"

#if defined(XLRS_PICO_SDK)
#include <hardware/gpio.h>
#include "hal/Time.h"
#endif

#ifndef XLRS_STATUS_LED_ACTIVE_LOW
#define XLRS_STATUS_LED_ACTIVE_LOW 1
#endif

namespace xlrs {
namespace app {

struct LinkStatusLedFlags {
    bool hardwareError = false;
    bool configFault = false;
    bool bindScanOpen = false;       // RX: listening for OTA bind frames
    bool bindPacketReceived = false; // RX: valid bind frame received
    bool bindTransmitActive = false; // TX: OTA bind transmit window
    bool outputActive = false;       // RX: CRSF output enabled
    bool requireOutputForConnected = false; // RX: solid Connected only when outputActive
};

#if defined(XLRS_PICO_SDK)
inline void linkStatusLedWrite(uint8_t pin, bool on) {
    const bool level = XLRS_STATUS_LED_ACTIVE_LOW ? !on : on;
    gpio_put(pin, level);
}
#endif

inline void linkStatusLedBlinkBlocking(uint8_t pin, uint8_t pulses, uint32_t onMs, uint32_t offMs) {
#if defined(XLRS_PICO_SDK)
    for (uint8_t i = 0; i < pulses; ++i) {
        linkStatusLedWrite(pin, true);
        hal::sleepMs(onMs);
        linkStatusLedWrite(pin, false);
        hal::sleepMs(offMs);
    }
#else
    (void)pin;
    (void)pulses;
    (void)onMs;
    (void)offMs;
#endif
}

inline void linkStatusLedInit(uint8_t pin) {
#if defined(XLRS_PICO_SDK)
    gpio_init(pin);
    gpio_set_function(pin, GPIO_FUNC_SIO);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_set_pulls(pin, false, false);
    linkStatusLedWrite(pin, false);
    // Boot self-test: five slow blinks so wiring/polarity is obvious at power-up.
    linkStatusLedBlinkBlocking(pin, 5, 150, 150);
#else
    (void)pin;
#endif
}

inline void linkStatusLedUpdate(uint8_t pin, LinkState state, const LinkStatusLedFlags& flags) {
#if defined(XLRS_PICO_SDK)
    static uint32_t lastUpdateMs = 0;
    static uint32_t msCounter = 0;
    static constexpr uint32_t kUpdateIntervalMs = 50;

    const uint32_t now = hal::nowMs();
    if (now - lastUpdateMs < kUpdateIntervalMs) return;
    lastUpdateMs = now;
    msCounter += kUpdateIntervalMs;

    bool on = false;
    if (flags.configFault || flags.hardwareError) {
        on = true;
    } else if (flags.bindPacketReceived) {
        on = (msCounter % 100) < 75;
    } else if (flags.bindScanOpen) {
        const uint32_t phase = msCounter % 600;
        on = phase < 70 || (phase >= 140 && phase < 210);
    } else if (state == LinkState::Binding || flags.bindTransmitActive) {
        on = (msCounter % 100) < 50;
    } else if (state == LinkState::Connected) {
        if (!flags.requireOutputForConnected || flags.outputActive) {
            on = true;
        } else {
            on = (msCounter % 500) < 250;
        }
    } else if (state == LinkState::Failsafe) {
        const uint32_t phase = msCounter % 600;
        on = phase < 80 || (phase >= 160 && phase < 240);
    } else if (state == LinkState::Connecting) {
        on = (msCounter % 500) < 250;
    } else {
        on = (msCounter % 1000) < 500;
    }

    linkStatusLedWrite(pin, on);
#else
    (void)pin;
    (void)state;
    (void)flags;
#endif
}

} // namespace app
} // namespace xlrs
