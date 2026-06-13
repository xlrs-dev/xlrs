#pragma once

#include <stddef.h>
#include <stdint.h>

namespace rc_handset {
namespace config {

constexpr uint8_t kRcHandsetAxisCount = 4;
constexpr uint8_t kRcHandsetChannelCount = 8;
constexpr uint16_t kRcHandsetAdcMax = 4095;
constexpr uint16_t kRcHandsetCrsfRawMin = 172;
constexpr uint16_t kRcHandsetCrsfRawMid = 992;
constexpr uint16_t kRcHandsetCrsfRawMax = 1811;
constexpr uint16_t kRcHandsetCalibrationMinSpan = 256;
constexpr size_t kRcHandsetConfigPayloadSize = 106;
constexpr size_t kRcHandsetConfigRecordSize = 4 + 1 + 2 + kRcHandsetConfigPayloadSize + 2;

enum class ChannelFunction : uint8_t {
    Aileron = 0,
    Elevator = 1,
    Rudder = 2,
    Throttle = 3,
    Aux1 = 4,
    Aux2 = 5,
    Aux3 = 6,
    Aux4 = 7,
};

struct AxisCalibration {
    uint16_t min;
    uint16_t center;
    uint16_t max;
};

struct AxisConfig {
    AxisCalibration calibration;
    bool inverted;
    uint16_t deadzone;
    ChannelFunction function;
};

struct ChannelLimits {
    int16_t trim;
    uint16_t cutoffMin;
    uint16_t cutoffMax;
};

struct FilterSettings {
    uint8_t adcSamples;
    uint8_t smoothingPercent;
    uint8_t highPassPercent;
    uint16_t spikeJumpThreshold;
    uint16_t spikeConfirmTolerance;
    uint16_t slewLimit;
};

struct DisplayOptions {
    uint8_t brightnessPercent;
    uint16_t timeoutSeconds;
    bool inverted;
};

struct PowerOptions {
    uint16_t autoOffSeconds;
    uint16_t lowBatteryMv;
    bool skipPowerOnSequence;
};

struct RcHandsetConfig {
    AxisConfig axes[kRcHandsetAxisCount];
    ChannelLimits channels[kRcHandsetChannelCount];
    FilterSettings filter;
    DisplayOptions display;
    PowerOptions power;
};

RcHandsetConfig defaultRcHandsetConfig();
bool validateRcHandsetConfig(const RcHandsetConfig& cfg);
bool makeAxisCalibrationFromEndpoints(uint16_t min, uint16_t max, AxisCalibration& out);

bool encodeRcHandsetConfigRecord(const RcHandsetConfig& cfg,
                                 uint8_t* out,
                                 size_t outLen,
                                 size_t& bytesWritten);
bool decodeRcHandsetConfigRecord(const uint8_t* data,
                                 size_t len,
                                 RcHandsetConfig& out);

} // namespace config
} // namespace rc_handset
