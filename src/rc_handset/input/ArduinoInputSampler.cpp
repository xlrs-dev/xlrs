#include "rc_handset/input/ArduinoInputSampler.h"

#if !defined(UNIT_TEST)

#include <Arduino.h>
#include <hardware/gpio.h>

namespace rc_handset {
namespace input {

ArduinoInputSampler::ArduinoInputSampler()
    : config_(defaultArduinoInputSamplerConfig()),
      filteredAdc_{},
      filteredAdcState_{},
      filterReady_(false) {
}

ArduinoInputPins defaultRp2350InputPins() {
    ArduinoInputPins pins{};
    pins.analog[0] = 26;
    pins.analog[1] = 27;
    pins.analog[2] = 28;
    pins.analog[3] = 29;
    const uint8_t toggles[kMaxThreePositionInputs * 2] = {1, 2, 3, 6, 7, 10, 11, 12};
    for (uint8_t i = 0; i < kMaxThreePositionInputs * 2; ++i) {
        pins.threePosition[i] = toggles[i];
    }
    return pins;
}

ArduinoInputSamplerConfig defaultArduinoInputSamplerConfig() {
    ArduinoInputSamplerConfig config{};
    config.pins = defaultRp2350InputPins();
    config.adcSamples = 8;
    config.smoothingPercent = 25;
    config.highPassPercent = 0;
    return config;
}

void ArduinoInputSampler::begin(const ArduinoInputSamplerConfig& config) {
    configure(config);
    for (uint8_t pin : config_.pins.threePosition) {
        pinMode(pin, INPUT_PULLUP);
    }
    analogReadResolution(12);
    // Explicitly disable the digital input buffer and pull resistors on each
    // analog pin -- leaving either enabled is a documented RP2040/RP2350 ADC
    // noise source (the digital input circuitry loads the analog signal).
    // Doesn't rely on the Arduino core doing this for us implicitly.
    for (uint8_t pin : config_.pins.analog) {
        gpio_disable_pulls(pin);
        gpio_set_input_enabled(pin, false);
    }
}

void ArduinoInputSampler::configure(const ArduinoInputSamplerConfig& config) {
    config_ = config;
    if (config_.adcSamples == 0) {
        config_.adcSamples = 1;
    }
    if (config_.smoothingPercent > 100) {
        config_.smoothingPercent = 100;
    }
    if (config_.highPassPercent > 100) {
        config_.highPassPercent = 100;
    }
}

InputSnapshot ArduinoInputSampler::read() {
    InputSnapshot snapshot{};
    for (uint8_t i = 0; i < kMaxAnalogInputs; ++i) {
        const uint16_t raw = readAveragedAdc(config_.pins.analog[i]);
        snapshot.rawAnalog[i] = raw;
        snapshot.analog[i] = smoothAdc(i, raw);
        snapshot.threePositionFirstHigh[i] = digitalRead(config_.pins.threePosition[i * 2]) == HIGH;
        snapshot.threePositionSecondHigh[i] = digitalRead(config_.pins.threePosition[i * 2 + 1]) == HIGH;
    }
    filterReady_ = true;
    return snapshot;
}

uint16_t ArduinoInputSampler::readAveragedAdc(uint8_t pin) const {
    // Discard the first conversion after the ADC mux switches to this pin --
    // the sample-and-hold capacitor still carries residual charge from
    // whichever pin was read last (read() cycles through all 4 axes), which
    // otherwise leaks a bit of the previous axis's voltage into this one's
    // first sample on every single read cycle.
    (void)analogRead(pin);
    uint32_t sum = 0;
    for (uint8_t i = 0; i < config_.adcSamples; ++i) {
        sum += analogRead(pin);
    }
    return static_cast<uint16_t>((sum + (config_.adcSamples / 2u)) / config_.adcSamples);
}

uint16_t ArduinoInputSampler::smoothAdc(uint8_t index, uint16_t raw) {
    if (index >= kMaxAnalogInputs || !filterReady_) {
        if (index < kMaxAnalogInputs) {
            filteredAdc_[index] = raw;
            filteredAdcState_[index] = adcToFilterState(raw);
        }
        return raw;
    }
    const uint16_t previous = adcFromFilterState(filteredAdcState_[index]);
    filteredAdcState_[index] =
        lowPassAdcFilterStep(filteredAdcState_[index], raw, config_.smoothingPercent);
    filteredAdc_[index] = adcFromFilterState(filteredAdcState_[index]);
    if (config_.highPassPercent == 0) return filteredAdc_[index];

    const int32_t delta = static_cast<int32_t>(filteredAdc_[index]) - static_cast<int32_t>(previous);
    int32_t boosted = static_cast<int32_t>(filteredAdc_[index]) +
                      ((delta * static_cast<int32_t>(config_.highPassPercent)) / 100);
    if (boosted < 0) boosted = 0;
    if (boosted > kAdc12BitMax) boosted = kAdc12BitMax;
    return static_cast<uint16_t>(boosted);
}

} // namespace input
} // namespace rc_handset

#endif // !UNIT_TEST
