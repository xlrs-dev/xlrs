#pragma once

#include <stdint.h>

#include "rc_handset/input/InputPipeline.h"

namespace rc_handset {
namespace input {

struct ArduinoInputPins {
    uint8_t analog[kMaxAnalogInputs];
    uint8_t threePosition[kMaxThreePositionInputs * 2];
};

struct ArduinoInputSamplerConfig {
    ArduinoInputPins pins;
    uint8_t adcSamples;
    uint8_t smoothingPercent;
};

class ArduinoInputSampler {
public:
    ArduinoInputSampler();

    void begin(const ArduinoInputSamplerConfig& config);
    void configure(const ArduinoInputSamplerConfig& config);
    InputSnapshot read();

private:
    uint16_t readAveragedAdc(uint8_t pin) const;
    uint16_t smoothAdc(uint8_t index, uint16_t raw);

    ArduinoInputSamplerConfig config_;
    uint16_t filteredAdc_[kMaxAnalogInputs];
    bool filterReady_;
};

ArduinoInputPins defaultRp2350InputPins();
ArduinoInputSamplerConfig defaultArduinoInputSamplerConfig();

} // namespace input
} // namespace rc_handset
