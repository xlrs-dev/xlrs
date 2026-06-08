#pragma once

#include <stddef.h>
#include <stdint.h>

namespace lora_link {

constexpr uint8_t kRcChannelCount = 16;
constexpr uint8_t kUidSize = 8;
constexpr uint8_t kOtaFrameSize = 33;
constexpr uint8_t kOtaPayloadSize = 22;
#ifndef CRSF_UART_BAUD
#define CRSF_UART_BAUD 400000
#endif
constexpr uint32_t kCrsfBaud = CRSF_UART_BAUD;
constexpr uint16_t kCrsfRawMin = 172;
constexpr uint16_t kCrsfRaw1000 = 191;
constexpr uint16_t kCrsfRawMid = 992;
constexpr uint16_t kCrsfRaw2000 = 1792;
constexpr uint16_t kCrsfRawMax = 1811;

#ifndef RF_FIXED_CHANNEL
#define RF_FIXED_CHANNEL 0
#endif
constexpr int8_t kFixedRfChannel = RF_FIXED_CHANNEL;

enum class OtaType : uint8_t {
    Rc = 1,
    Telemetry = 2,
};

enum class RateId : uint8_t {
    L250 = 0,
    L100 = 1,
};

struct RateConfig {
    const char* name;
    uint32_t intervalUs;
    uint8_t telemetryRatio;
    float bandwidthKHz;
    uint8_t spreadingFactor;
    uint8_t codingRate;
};

struct OtaFrame {
    OtaType type;
    uint16_t sequence;
    uint32_t uidCheck;
    uint8_t payloadLen;
    uint8_t payload[kOtaPayloadSize];
};

struct DeviceConfig {
    char bindingPhrase[33];
    RateId rate;
};

struct ConfigRecord {
    uint32_t magic;
    uint8_t version;
    uint8_t rate;
    char bindingPhrase[33];
    uint16_t crc;
};

extern const RateConfig kRates[2];

uint16_t crc16Ccitt(const uint8_t* data, size_t len);
uint8_t crsfCrc8(const uint8_t* data, size_t len);
void deriveUid(const char* phrase, uint8_t uid[kUidSize]);
uint32_t uidCheck(const uint8_t uid[kUidSize]);
uint16_t syncWordFromUid(const uint8_t uid[kUidSize]);

bool encodeOtaFrame(const OtaFrame& frame, uint8_t out[kOtaFrameSize]);
bool decodeOtaFrame(const uint8_t in[kOtaFrameSize], uint32_t expectedUidCheck, OtaFrame& out);

void packRcChannels11Bit(const uint16_t channels[kRcChannelCount], uint8_t out[22]);
void unpackRcChannels11Bit(const uint8_t in[22], uint16_t channels[kRcChannelCount]);
uint16_t clampCrsfRaw(uint16_t value);
bool sanitizeRcChannels(const uint16_t previous[kRcChannelCount], bool havePrevious,
                        uint16_t channels[kRcChannelCount]);
void slewLimitPrimaryRcChannels(const uint16_t previous[kRcChannelCount], bool havePrevious,
                                uint16_t channels[kRcChannelCount], uint16_t maxDelta);
size_t encodeCrsfRcFrame(const uint16_t channels[kRcChannelCount], uint8_t* out, size_t outLen);
bool parseCrsfRcFrame(uint8_t byte, uint16_t channels[kRcChannelCount]);
size_t encodeCrsfLinkStats(int16_t rssiDbm, int8_t snr, uint8_t linkQuality, uint8_t activeRate,
                           uint8_t* out, size_t outLen);

void configDefaults(DeviceConfig& cfg, const char* defaultPhrase);
ConfigRecord makeConfigRecord(const DeviceConfig& cfg);
bool readConfigRecord(const ConfigRecord& record, DeviceConfig& cfg);

uint8_t fhssChannelFor(const uint8_t uid[kUidSize], uint16_t hop);
float fhssFrequencyMHz(uint8_t channel);

} // namespace lora_link
