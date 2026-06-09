#include "rc_handset/input/InputPipeline.h"

#include <string.h>

namespace rc_handset {
namespace input {

namespace {

uint16_t clampToCrsfCommandRange(uint16_t value) {
    if (value < lora_link::kCrsfRaw1000) return lora_link::kCrsfRaw1000;
    if (value > lora_link::kCrsfRaw2000) return lora_link::kCrsfRaw2000;
    return value;
}

uint16_t interpolate(uint16_t raw, uint16_t inMin, uint16_t inMax,
                     uint16_t outMin, uint16_t outMax) {
    if (inMax <= inMin) return outMin;
    if (raw <= inMin) return outMin;
    if (raw >= inMax) return outMax;
    const uint32_t spanIn = static_cast<uint32_t>(inMax - inMin);
    const uint32_t spanOut = static_cast<uint32_t>(outMax - outMin);
    const uint32_t scaled = (static_cast<uint32_t>(raw - inMin) * spanOut + (spanIn / 2u)) / spanIn;
    return static_cast<uint16_t>(outMin + scaled);
}

ChannelFunction convertChannelFunction(config::ChannelFunction function) {
    switch (function) {
    case config::ChannelFunction::Aileron: return ChannelFunction::Aileron;
    case config::ChannelFunction::Elevator: return ChannelFunction::Elevator;
    case config::ChannelFunction::Rudder: return ChannelFunction::Rudder;
    case config::ChannelFunction::Throttle: return ChannelFunction::Throttle;
    case config::ChannelFunction::Aux1: return ChannelFunction::Aux1;
    case config::ChannelFunction::Aux2: return ChannelFunction::Aux2;
    case config::ChannelFunction::Aux3: return ChannelFunction::Aux3;
    case config::ChannelFunction::Aux4: return ChannelFunction::Aux4;
    default: return ChannelFunction::None;
    }
}

ChannelCutoff channelCutoffFromConfig(const config::ChannelLimits& limits) {
    return ChannelCutoff{limits.cutoffMin, limits.cutoffMax};
}

} // namespace

InputPipelineConfig defaultInputPipelineConfig() {
    InputPipelineConfig config{};
    const AdcCalibration fullRange{0, 2048, kAdc12BitMax};
    const ChannelCutoff fullTravel{lora_link::kCrsfRaw1000, lora_link::kCrsfRaw2000};

    config.analog[0] = {ChannelFunction::Aileron, fullRange, 0, false, 0, fullTravel};
    config.analog[1] = {ChannelFunction::Elevator, fullRange, 0, false, 0, fullTravel};
    config.analog[2] = {ChannelFunction::Rudder, fullRange, 0, false, 0, fullTravel};
    config.analog[3] = {ChannelFunction::Throttle, fullRange, 0, false, 0, fullTravel};
    config.analogCount = kMaxAnalogInputs;

    config.threePosition[0] = {ChannelFunction::Aux1, 0, fullTravel};
    config.threePosition[1] = {ChannelFunction::Aux2, 0, fullTravel};
    config.threePosition[2] = {ChannelFunction::Aux3, 0, fullTravel};
    config.threePosition[3] = {ChannelFunction::Aux4, 0, fullTravel};
    config.threePositionCount = kMaxThreePositionInputs;

    config.spikeJumpThreshold = 160;
    config.spikeConfirmTolerance = 80;
    return config;
}

InputPipelineConfig inputPipelineConfigFromHandsetConfig(const config::RcHandsetConfig& handsetConfig) {
    InputPipelineConfig pipeline{};
    pipeline.analogCount = config::kRcHandsetAxisCount < kMaxAnalogInputs
                               ? config::kRcHandsetAxisCount
                               : kMaxAnalogInputs;
    for (size_t i = 0; i < pipeline.analogCount; ++i) {
        const ChannelFunction function = convertChannelFunction(handsetConfig.axes[i].function);
        const uint8_t channelIndex = channelIndexForFunction(function);
        const config::ChannelLimits& limits =
            channelIndex < config::kRcHandsetChannelCount ? handsetConfig.channels[channelIndex]
                                                          : handsetConfig.channels[0];
        pipeline.analog[i].function = function;
        pipeline.analog[i].calibration = {
            handsetConfig.axes[i].calibration.min,
            handsetConfig.axes[i].calibration.center,
            handsetConfig.axes[i].calibration.max,
        };
        pipeline.analog[i].deadzone = handsetConfig.axes[i].deadzone;
        pipeline.analog[i].invert = handsetConfig.axes[i].inverted;
        pipeline.analog[i].trim = limits.trim;
        pipeline.analog[i].cutoff = channelCutoffFromConfig(limits);
    }

    const ChannelFunction toggleFunctions[kMaxThreePositionInputs] = {
        ChannelFunction::Aux1,
        ChannelFunction::Aux2,
        ChannelFunction::Aux3,
        ChannelFunction::Aux4,
    };
    pipeline.threePositionCount = kMaxThreePositionInputs;
    for (size_t i = 0; i < pipeline.threePositionCount; ++i) {
        const ChannelFunction function = toggleFunctions[i];
        const uint8_t channelIndex = channelIndexForFunction(function);
        const config::ChannelLimits& limits = handsetConfig.channels[channelIndex];
        pipeline.threePosition[i].function = function;
        pipeline.threePosition[i].trim = limits.trim;
        pipeline.threePosition[i].cutoff = channelCutoffFromConfig(limits);
    }

    pipeline.spikeJumpThreshold = handsetConfig.filter.spikeJumpThreshold;
    pipeline.spikeConfirmTolerance = handsetConfig.filter.spikeConfirmTolerance;
    return pipeline;
}

InputPipelineState defaultInputPipelineState() {
    InputPipelineState state{};
    for (uint8_t i = 0; i < lora_link::kRcChannelCount; ++i) {
        state.channels[i] = lora_link::kCrsfRawMid;
    }
    return state;
}

uint8_t channelIndexForFunction(ChannelFunction function) {
    switch (function) {
    case ChannelFunction::Aileron: return 0;
    case ChannelFunction::Elevator: return 1;
    case ChannelFunction::Rudder: return 2;
    case ChannelFunction::Throttle: return 3;
    case ChannelFunction::Aux1: return 4;
    case ChannelFunction::Aux2: return 5;
    case ChannelFunction::Aux3: return 6;
    case ChannelFunction::Aux4: return 7;
    case ChannelFunction::Aux5: return 8;
    case ChannelFunction::Aux6: return 9;
    case ChannelFunction::Aux7: return 10;
    case ChannelFunction::Aux8: return 11;
    case ChannelFunction::Aux9: return 12;
    case ChannelFunction::Aux10: return 13;
    case ChannelFunction::Aux11: return 14;
    case ChannelFunction::Aux12: return 15;
    case ChannelFunction::None:
    default:
        return lora_link::kRcChannelCount;
    }
}

bool mapChannelFunction(ChannelFunction function, uint16_t value,
                        uint16_t channels[lora_link::kRcChannelCount]) {
    const uint8_t index = channelIndexForFunction(function);
    if (index >= lora_link::kRcChannelCount) return false;
    channels[index] = clampToCrsfCommandRange(value);
    return true;
}

uint16_t adcToCalibratedChannel(uint16_t raw, const AdcCalibration& calibration) {
    const uint16_t rawMin = calibration.rawMin;
    const uint16_t rawMid = calibration.rawMid;
    const uint16_t rawMax = calibration.rawMax;
    if (rawMid <= rawMin || rawMax <= rawMid) {
        return interpolate(raw, 0, kAdc12BitMax, lora_link::kCrsfRaw1000, lora_link::kCrsfRaw2000);
    }
    if (raw <= rawMid) {
        return interpolate(raw, rawMin, rawMid, lora_link::kCrsfRaw1000, lora_link::kCrsfRawMid);
    }
    return interpolate(raw, rawMid, rawMax, lora_link::kCrsfRawMid, lora_link::kCrsfRaw2000);
}

uint16_t applyDeadzone(uint16_t value, uint16_t deadzone) {
    value = clampToCrsfCommandRange(value);
    const uint16_t delta = value > lora_link::kCrsfRawMid ? value - lora_link::kCrsfRawMid
                                                          : lora_link::kCrsfRawMid - value;
    if (delta <= deadzone) return lora_link::kCrsfRawMid;
    return value;
}

uint16_t applyInvert(uint16_t value, bool invert) {
    value = clampToCrsfCommandRange(value);
    if (!invert) return value;
    return static_cast<uint16_t>(lora_link::kCrsfRaw1000 + lora_link::kCrsfRaw2000 - value);
}

uint16_t applyTrim(uint16_t value, int16_t trim) {
    const int32_t adjusted = static_cast<int32_t>(value) + trim;
    if (adjusted < lora_link::kCrsfRaw1000) return lora_link::kCrsfRaw1000;
    if (adjusted > lora_link::kCrsfRaw2000) return lora_link::kCrsfRaw2000;
    return static_cast<uint16_t>(adjusted);
}

uint16_t applyCutoff(uint16_t value, const ChannelCutoff& cutoff) {
    value = clampToCrsfCommandRange(value);
    uint16_t low = cutoff.low;
    uint16_t high = cutoff.high;
    if (low < lora_link::kCrsfRaw1000) low = lora_link::kCrsfRaw1000;
    if (high > lora_link::kCrsfRaw2000) high = lora_link::kCrsfRaw2000;
    if (high < low) {
        low = lora_link::kCrsfRaw1000;
        high = lora_link::kCrsfRaw2000;
    }
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

uint16_t processAnalogChannel(uint16_t raw, const AnalogInputConfig& config) {
    uint16_t value = adcToCalibratedChannel(raw, config.calibration);
    value = applyDeadzone(value, config.deadzone);
    value = applyInvert(value, config.invert);
    value = applyTrim(value, config.trim);
    value = applyCutoff(value, config.cutoff);
    return value;
}

uint16_t decodeDualPinThreePosition(bool firstHigh, bool secondHigh) {
    if (firstHigh && !secondHigh) return lora_link::kCrsfRaw1000;
    if (!firstHigh && secondHigh) return lora_link::kCrsfRaw2000;
    return lora_link::kCrsfRawMid;
}

uint16_t processThreePositionChannel(bool firstHigh, bool secondHigh,
                                     const ThreePositionInputConfig& config) {
    uint16_t value = decodeDualPinThreePosition(firstHigh, secondHigh);
    value = applyTrim(value, config.trim);
    value = applyCutoff(value, config.cutoff);
    return value;
}

void buildCandidateChannels(const InputPipelineConfig& config, const InputSnapshot& snapshot,
                            uint16_t candidate[lora_link::kRcChannelCount]) {
    for (uint8_t i = 0; i < lora_link::kRcChannelCount; ++i) {
        candidate[i] = lora_link::kCrsfRawMid;
    }

    const size_t analogCount = config.analogCount < kMaxAnalogInputs ? config.analogCount : kMaxAnalogInputs;
    for (size_t i = 0; i < analogCount; ++i) {
        mapChannelFunction(config.analog[i].function,
                           processAnalogChannel(snapshot.analog[i], config.analog[i]),
                           candidate);
    }

    const size_t threePositionCount = config.threePositionCount < kMaxThreePositionInputs
                                          ? config.threePositionCount
                                          : kMaxThreePositionInputs;
    for (size_t i = 0; i < threePositionCount; ++i) {
        mapChannelFunction(config.threePosition[i].function,
                           processThreePositionChannel(snapshot.threePositionFirstHigh[i],
                                                       snapshot.threePositionSecondHigh[i],
                                                       config.threePosition[i]),
                           candidate);
    }
}

ProcessResult processInputSnapshot(const InputPipelineConfig& config, const InputSnapshot& snapshot,
                                   InputPipelineState& state) {
    uint16_t candidate[lora_link::kRcChannelCount];
    buildCandidateChannels(config, snapshot, candidate);
    if (!lora_link::sanitizeRcChannels(state.channels, state.haveChannels, candidate)) {
        return ProcessResult::RejectedBySanitizer;
    }
    if (!lora_link::acceptRcChannelsWithSpikeGate(state.channels, state.haveChannels, candidate,
                                                  state.spikeGate, config.spikeJumpThreshold,
                                                  config.spikeConfirmTolerance)) {
        return ProcessResult::HeldBySpikeGate;
    }
    memcpy(state.channels, candidate, sizeof(state.channels));
    state.haveChannels = true;
    return ProcessResult::Accepted;
}

} // namespace input
} // namespace rc_handset
