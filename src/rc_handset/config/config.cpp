#include "rc_handset/config/config.h"

#include <string.h>

namespace rc_handset {
namespace config {

namespace {

constexpr uint32_t kRecordMagic = 0x31484352u; // RCH1, little-endian in EEPROM bytes.
constexpr uint8_t kRecordVersion = 2;
constexpr uint8_t kLegacyRecordVersion = 1;
constexpr size_t kHeaderSize = 7;
constexpr size_t kLegacyPayloadSize = 105;
constexpr size_t kLegacyRecordSize = kHeaderSize + kLegacyPayloadSize + 2;
constexpr int16_t kMaxTrim = 500;

uint16_t crc16Ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

void writeU16(uint8_t*& cursor, uint16_t value) {
    *cursor++ = static_cast<uint8_t>(value);
    *cursor++ = static_cast<uint8_t>(value >> 8);
}

void writeI16(uint8_t*& cursor, int16_t value) {
    writeU16(cursor, static_cast<uint16_t>(value));
}

void writeU32(uint8_t*& cursor, uint32_t value) {
    *cursor++ = static_cast<uint8_t>(value);
    *cursor++ = static_cast<uint8_t>(value >> 8);
    *cursor++ = static_cast<uint8_t>(value >> 16);
    *cursor++ = static_cast<uint8_t>(value >> 24);
}

uint16_t readU16(const uint8_t*& cursor) {
    const uint16_t value = static_cast<uint16_t>(cursor[0]) |
                           static_cast<uint16_t>(static_cast<uint16_t>(cursor[1]) << 8);
    cursor += 2;
    return value;
}

int16_t readI16(const uint8_t*& cursor) {
    return static_cast<int16_t>(readU16(cursor));
}

uint32_t readU32(const uint8_t*& cursor) {
    const uint32_t value = static_cast<uint32_t>(cursor[0]) |
                           (static_cast<uint32_t>(cursor[1]) << 8) |
                           (static_cast<uint32_t>(cursor[2]) << 16) |
                           (static_cast<uint32_t>(cursor[3]) << 24);
    cursor += 4;
    return value;
}

bool validChannelFunction(ChannelFunction function) {
    return static_cast<uint8_t>(function) <= static_cast<uint8_t>(ChannelFunction::Aux4);
}

bool validAxisCalibration(const AxisCalibration& calibration) {
    return calibration.min < calibration.center &&
           calibration.center < calibration.max &&
           calibration.max <= kRcHandsetAdcMax;
}

bool readBool(const uint8_t*& cursor, bool& out) {
    const uint8_t value = *cursor++;
    if (value > 1) return false;
    out = value != 0;
    return true;
}

void encodePayload(const RcHandsetConfig& cfg, uint8_t* out) {
    uint8_t* cursor = out;
    for (uint8_t i = 0; i < kRcHandsetAxisCount; ++i) {
        writeU16(cursor, cfg.axes[i].calibration.min);
        writeU16(cursor, cfg.axes[i].calibration.center);
        writeU16(cursor, cfg.axes[i].calibration.max);
        *cursor++ = cfg.axes[i].inverted ? 1 : 0;
        writeU16(cursor, cfg.axes[i].deadzone);
        *cursor++ = static_cast<uint8_t>(cfg.axes[i].function);
    }
    for (uint8_t i = 0; i < kRcHandsetChannelCount; ++i) {
        writeI16(cursor, cfg.channels[i].trim);
        writeU16(cursor, cfg.channels[i].cutoffMin);
        writeU16(cursor, cfg.channels[i].cutoffMax);
    }
    *cursor++ = cfg.filter.adcSamples;
    *cursor++ = cfg.filter.smoothingPercent;
    *cursor++ = cfg.filter.highPassPercent;
    writeU16(cursor, cfg.filter.spikeJumpThreshold);
    writeU16(cursor, cfg.filter.spikeConfirmTolerance);
    writeU16(cursor, cfg.filter.slewLimit);
    *cursor++ = cfg.display.brightnessPercent;
    writeU16(cursor, cfg.display.timeoutSeconds);
    *cursor++ = cfg.display.inverted ? 1 : 0;
    writeU16(cursor, cfg.power.autoOffSeconds);
    writeU16(cursor, cfg.power.lowBatteryMv);
    *cursor++ = cfg.power.skipPowerOnSequence ? 1 : 0;
}

bool decodePayload(const uint8_t* data, size_t payloadLen, RcHandsetConfig& out) {
    const uint8_t* cursor = data;
    RcHandsetConfig cfg{};
    for (uint8_t i = 0; i < kRcHandsetAxisCount; ++i) {
        cfg.axes[i].calibration.min = readU16(cursor);
        cfg.axes[i].calibration.center = readU16(cursor);
        cfg.axes[i].calibration.max = readU16(cursor);
        if (!readBool(cursor, cfg.axes[i].inverted)) return false;
        cfg.axes[i].deadzone = readU16(cursor);
        cfg.axes[i].function = static_cast<ChannelFunction>(*cursor++);
    }
    for (uint8_t i = 0; i < kRcHandsetChannelCount; ++i) {
        cfg.channels[i].trim = readI16(cursor);
        cfg.channels[i].cutoffMin = readU16(cursor);
        cfg.channels[i].cutoffMax = readU16(cursor);
    }
    cfg.filter.adcSamples = *cursor++;
    cfg.filter.smoothingPercent = *cursor++;
    cfg.filter.highPassPercent = payloadLen >= kRcHandsetConfigPayloadSize ? *cursor++ : 0;
    cfg.filter.spikeJumpThreshold = readU16(cursor);
    cfg.filter.spikeConfirmTolerance = readU16(cursor);
    cfg.filter.slewLimit = readU16(cursor);
    cfg.display.brightnessPercent = *cursor++;
    cfg.display.timeoutSeconds = readU16(cursor);
    if (!readBool(cursor, cfg.display.inverted)) return false;
    cfg.power.autoOffSeconds = readU16(cursor);
    cfg.power.lowBatteryMv = readU16(cursor);
    if (!readBool(cursor, cfg.power.skipPowerOnSequence)) return false;
    if (!validateRcHandsetConfig(cfg)) return false;
    out = cfg;
    return true;
}

} // namespace

RcHandsetConfig defaultRcHandsetConfig() {
    RcHandsetConfig cfg{};
    const ChannelFunction defaultFunctions[kRcHandsetAxisCount] = {
        ChannelFunction::Aileron,
        ChannelFunction::Elevator,
        ChannelFunction::Rudder,
        ChannelFunction::Throttle,
    };
    for (uint8_t i = 0; i < kRcHandsetAxisCount; ++i) {
        cfg.axes[i].calibration.min = 200;
        cfg.axes[i].calibration.center = 2048;
        cfg.axes[i].calibration.max = 3895;
        cfg.axes[i].inverted = false;
        cfg.axes[i].deadzone = 16;
        cfg.axes[i].function = defaultFunctions[i];
    }
    for (uint8_t i = 0; i < kRcHandsetChannelCount; ++i) {
        cfg.channels[i].trim = 0;
        cfg.channels[i].cutoffMin = kRcHandsetCrsfRawMin;
        cfg.channels[i].cutoffMax = kRcHandsetCrsfRawMax;
    }
    cfg.filter.adcSamples = 8;
    cfg.filter.smoothingPercent = 25;
    cfg.filter.highPassPercent = 0;
    cfg.filter.spikeJumpThreshold = 160;
    cfg.filter.spikeConfirmTolerance = 80;
    cfg.filter.slewLimit = 64;
    cfg.display.brightnessPercent = 80;
    cfg.display.timeoutSeconds = 30;
    cfg.display.inverted = false;
    cfg.power.autoOffSeconds = 600;
    cfg.power.lowBatteryMv = 3300;
    cfg.power.skipPowerOnSequence = false;
    return cfg;
}

bool validateRcHandsetConfig(const RcHandsetConfig& cfg) {
    for (uint8_t i = 0; i < kRcHandsetAxisCount; ++i) {
        if (!validAxisCalibration(cfg.axes[i].calibration)) return false;
        if (cfg.axes[i].deadzone > 512) return false;
        if (!validChannelFunction(cfg.axes[i].function)) return false;
    }
    for (uint8_t i = 0; i < kRcHandsetChannelCount; ++i) {
        if (cfg.channels[i].trim < -kMaxTrim || cfg.channels[i].trim > kMaxTrim) return false;
        if (cfg.channels[i].cutoffMin < kRcHandsetCrsfRawMin) return false;
        if (cfg.channels[i].cutoffMax > kRcHandsetCrsfRawMax) return false;
        if (cfg.channels[i].cutoffMin >= cfg.channels[i].cutoffMax) return false;
    }
    if (cfg.filter.adcSamples < 1 || cfg.filter.adcSamples > 32) return false;
    if (cfg.filter.smoothingPercent > 100) return false;
    if (cfg.filter.highPassPercent > 100) return false;
    if (cfg.filter.spikeJumpThreshold == 0) return false;
    if (cfg.filter.spikeConfirmTolerance > cfg.filter.spikeJumpThreshold) return false;
    if (cfg.filter.slewLimit == 0 || cfg.filter.slewLimit > 512) return false;
    if (cfg.display.brightnessPercent > 100) return false;
    if (cfg.display.timeoutSeconds > 3600) return false;
    if (cfg.power.autoOffSeconds > 43200) return false;
    if (cfg.power.lowBatteryMv < 2500 || cfg.power.lowBatteryMv > 5000) return false;
    return true;
}

bool makeAxisCalibrationFromEndpoints(uint16_t min, uint16_t max, AxisCalibration& out) {
    if (max > kRcHandsetAdcMax || min >= max) return false;
    const uint16_t span = static_cast<uint16_t>(max - min);
    if (span < kRcHandsetCalibrationMinSpan) return false;
    AxisCalibration calibration{};
    calibration.min = min;
    calibration.center = static_cast<uint16_t>(min + (span / 2u));
    calibration.max = max;
    if (!validAxisCalibration(calibration)) return false;
    out = calibration;
    return true;
}

bool encodeRcHandsetConfigRecord(const RcHandsetConfig& cfg,
                                 uint8_t* out,
                                 size_t outLen,
                                 size_t& bytesWritten) {
    bytesWritten = 0;
    if (!out || outLen < kRcHandsetConfigRecordSize || !validateRcHandsetConfig(cfg)) return false;

    memset(out, 0, kRcHandsetConfigRecordSize);
    uint8_t* cursor = out;
    writeU32(cursor, kRecordMagic);
    *cursor++ = kRecordVersion;
    writeU16(cursor, static_cast<uint16_t>(kRcHandsetConfigPayloadSize));
    encodePayload(cfg, cursor);
    cursor += kRcHandsetConfigPayloadSize;
    const uint16_t crc = crc16Ccitt(out, kHeaderSize + kRcHandsetConfigPayloadSize);
    writeU16(cursor, crc);
    bytesWritten = kRcHandsetConfigRecordSize;
    return true;
}

bool decodeRcHandsetConfigRecord(const uint8_t* data,
                                 size_t len,
                                 RcHandsetConfig& out) {
    if (!data || len < kLegacyRecordSize) return false;
    const uint8_t* cursor = data;
    const uint32_t magic = readU32(cursor);
    const uint8_t version = *cursor++;
    const uint16_t payloadLen = readU16(cursor);
    const bool currentRecord = version == kRecordVersion && payloadLen == kRcHandsetConfigPayloadSize;
    const bool legacyRecord = version == kLegacyRecordVersion && payloadLen == kLegacyPayloadSize;
    if (magic != kRecordMagic || (!currentRecord && !legacyRecord)) {
        return false;
    }
    if (currentRecord && len < kRcHandsetConfigRecordSize) return false;
    if (legacyRecord && len < kLegacyRecordSize) return false;
    const uint16_t expectedCrc = static_cast<uint16_t>(data[kHeaderSize + payloadLen]) |
                                static_cast<uint16_t>(
                                    static_cast<uint16_t>(data[kHeaderSize + payloadLen + 1]) << 8);
    const uint16_t actualCrc = crc16Ccitt(data, kHeaderSize + payloadLen);
    if (expectedCrc != actualCrc) return false;
    return decodePayload(cursor, payloadLen, out);
}

} // namespace config
} // namespace rc_handset
