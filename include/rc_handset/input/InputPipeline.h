#pragma once

#include <stddef.h>
#include <stdint.h>

#include "lora_link/protocol.h"
#include "rc_handset/config/config.h"

namespace rc_handset {
namespace input {

constexpr uint16_t kAdc12BitMax = 4095;
constexpr uint8_t kMaxAnalogInputs = 4;
constexpr uint8_t kMaxThreePositionInputs = 4;

enum class ChannelFunction : uint8_t {
    None = 0,
    Aileron,
    Elevator,
    Rudder,
    Throttle,
    Aux1,
    Aux2,
    Aux3,
    Aux4,
    Aux5,
    Aux6,
    Aux7,
    Aux8,
    Aux9,
    Aux10,
    Aux11,
    Aux12,
};

struct AdcCalibration {
    uint16_t rawMin;
    uint16_t rawMid;
    uint16_t rawMax;
};

struct ChannelCutoff {
    uint16_t low;
    uint16_t high;
};

struct AnalogInputConfig {
    ChannelFunction function;
    AdcCalibration calibration;
    uint16_t deadzone;
    bool invert;
    int16_t trim;
    ChannelCutoff cutoff;
};

struct ThreePositionInputConfig {
    ChannelFunction function;
    int16_t trim;
    ChannelCutoff cutoff;
};

struct InputPipelineConfig {
    AnalogInputConfig analog[kMaxAnalogInputs];
    size_t analogCount;
    ThreePositionInputConfig threePosition[kMaxThreePositionInputs];
    size_t threePositionCount;
    uint16_t spikeJumpThreshold;
    uint16_t spikeConfirmTolerance;
};

struct InputSnapshot {
    uint16_t analog[kMaxAnalogInputs];
    bool threePositionFirstHigh[kMaxThreePositionInputs];
    bool threePositionSecondHigh[kMaxThreePositionInputs];
};

struct InputPipelineState {
    uint16_t channels[lora_link::kRcChannelCount];
    bool haveChannels;
    lora_link::RcSpikeGate spikeGate;
};

enum class ProcessResult : uint8_t {
    Accepted,
    RejectedBySanitizer,
    HeldBySpikeGate,
};

InputPipelineConfig defaultInputPipelineConfig();
InputPipelineConfig inputPipelineConfigFromHandsetConfig(const config::RcHandsetConfig& handsetConfig);
InputPipelineState defaultInputPipelineState();

uint8_t channelIndexForFunction(ChannelFunction function);
bool mapChannelFunction(ChannelFunction function, uint16_t value,
                        uint16_t channels[lora_link::kRcChannelCount]);

uint16_t adcToCalibratedChannel(uint16_t raw, const AdcCalibration& calibration);
uint16_t applyDeadzone(uint16_t value, uint16_t deadzone);
uint16_t applyInvert(uint16_t value, bool invert);
uint16_t applyTrim(uint16_t value, int16_t trim);
uint16_t applyCutoff(uint16_t value, const ChannelCutoff& cutoff);
uint16_t processAnalogChannel(uint16_t raw, const AnalogInputConfig& config);

uint16_t decodeDualPinThreePosition(bool firstHigh, bool secondHigh);
uint16_t processThreePositionChannel(bool firstHigh, bool secondHigh,
                                     const ThreePositionInputConfig& config);

void buildCandidateChannels(const InputPipelineConfig& config, const InputSnapshot& snapshot,
                            uint16_t candidate[lora_link::kRcChannelCount]);
ProcessResult processInputSnapshot(const InputPipelineConfig& config, const InputSnapshot& snapshot,
                                   InputPipelineState& state);

} // namespace input
} // namespace rc_handset
