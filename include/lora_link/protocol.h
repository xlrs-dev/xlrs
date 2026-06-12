#pragma once

#include <stddef.h>
#include <stdint.h>

namespace lora_link {

constexpr uint8_t kRcChannelCount = 16;
constexpr uint8_t kUidSize = 8;
constexpr uint8_t kOtaFrameSize = 33;
constexpr uint8_t kOtaPayloadSize = 22;
constexpr uint8_t kBindingPhraseMaxLength = 32;
constexpr uint8_t kFhssChannelCount = 40;
#ifndef CRSF_UART_BAUD
#define CRSF_UART_BAUD 400000
#endif
constexpr uint32_t kCrsfBaud = CRSF_UART_BAUD;
constexpr uint16_t kCrsfRawMin = 172;
constexpr uint16_t kCrsfRaw1000 = 191;
constexpr uint16_t kCrsfRawMid = 992;
constexpr uint16_t kCrsfRaw2000 = 1792;
constexpr uint16_t kCrsfRawMax = 1811;
constexpr uint32_t kFailsafeTimeoutMs = 250;
constexpr uint8_t kTelemetryResponseListenSlots = 1;

#ifndef RF_FIXED_CHANNEL
#define RF_FIXED_CHANNEL -1
#endif
constexpr int8_t kFixedRfChannel = RF_FIXED_CHANNEL;

enum class OtaType : uint8_t {
    Rc = 1,
    Telemetry = 2,
    Sync = 3,
    Bind = 4,
};

enum class RateId : uint8_t {
    L250 = 0,
    L100 = 1,
};

struct RateConfig {
    const char* name;
    uint32_t intervalUs;
    uint8_t telemetryRatio;
    uint8_t fhssHopInterval;
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

struct OtaSyncPayload {
    uint16_t nonce;
    uint16_t fhssIndex;
    uint8_t rateIndex;
    uint8_t nextRateIndex;
    uint32_t switchSequence;
    uint8_t telemetryRatio;
    uint8_t fhssHopInterval;
    uint32_t txTick;
};

struct DeviceConfig {
    char bindingPhrase[33];
    uint8_t linkUid[kUidSize];
    RateId rate;
};

enum class BindingControlOp : uint8_t {
    Get = 1,
    Set = 2,
    Clear = 3,
    Verify = 4,
    Bind = 5,
};

enum class BindingResult : uint8_t {
    Ok = 0,
    InvalidCommand = 1,
    InvalidPhrase = 2,
    PersistFailed = 3,
};

struct BindingStatus {
    char phrase[33];
    uint8_t uid[kUidSize];
    uint32_t uidCheck;
    bool persisted;
    bool requiresReboot;
};

struct BindingControlRequest {
    BindingControlOp op;
    char phrase[33];
};

struct ConfigRecord {
    uint32_t magic;
    uint8_t version;
    uint8_t rate;
    char bindingPhrase[33];
    uint8_t reserved;
    uint16_t crc;
    uint8_t linkUid[kUidSize];
};

struct OtaBindPayload {
    uint8_t linkUid[kUidSize];
    uint32_t uidCheck;
};

struct RcSpikeGate {
    uint16_t pending[kRcChannelCount];
    bool havePending;
};

extern const RateConfig kRates[2];

uint16_t crc16Ccitt(const uint8_t* data, size_t len);
uint8_t crsfCrc8(const uint8_t* data, size_t len);
void deriveUid(const char* phrase, uint8_t uid[kUidSize]);
void derivePairUidFromHardware(const char* phrase, const uint8_t boardId[kUidSize], uint8_t uid[kUidSize]);
uint32_t uidCheck(const uint8_t uid[kUidSize]);
uint8_t syncWordFromUid(const uint8_t uid[kUidSize]);
bool validateBindingPhrase(const char* phrase);
void makeBindingStatus(const char* phrase, bool persisted, bool requiresReboot, BindingStatus& out);
void makeBindingStatus(const char* phrase, const uint8_t uid[kUidSize], bool persisted,
                       bool requiresReboot, BindingStatus& out);

bool encodeOtaFrame(const OtaFrame& frame, uint8_t out[kOtaFrameSize]);
bool decodeOtaFrame(const uint8_t in[kOtaFrameSize], uint32_t expectedUidCheck, OtaFrame& out);
bool encodeOtaSyncPayload(const OtaSyncPayload& payload, uint8_t out[kOtaPayloadSize], uint8_t& payloadLen);
bool decodeOtaSyncPayload(const OtaFrame& frame, OtaSyncPayload& out);
bool encodeOtaBindPayload(const OtaBindPayload& payload, uint8_t out[kOtaPayloadSize], uint8_t& payloadLen);
bool decodeOtaBindPayload(const OtaFrame& frame, OtaBindPayload& out);
uint8_t rateIndexForConfig(const RateConfig& rate);
bool isValidRateIndex(uint8_t rateIndex);

void packRcChannels11Bit(const uint16_t channels[kRcChannelCount], uint8_t out[22]);
void unpackRcChannels11Bit(const uint8_t in[22], uint16_t channels[kRcChannelCount]);
uint16_t clampCrsfRaw(uint16_t value);
bool sanitizeRcChannels(const uint16_t previous[kRcChannelCount], bool havePrevious,
                        uint16_t channels[kRcChannelCount]);
bool acceptRcChannelsWithSpikeGate(const uint16_t previous[kRcChannelCount], bool havePrevious,
                                   const uint16_t candidate[kRcChannelCount], RcSpikeGate& gate,
                                   uint16_t jumpThreshold, uint16_t confirmTolerance);
bool isLinkFresh(uint32_t nowMs, uint32_t lastFrameMs, uint32_t timeoutMs = kFailsafeTimeoutMs);
bool shouldRequestTelemetry(uint16_t sequence, uint8_t telemetryRatio);
uint32_t telemetryListenWindowMs(const RateConfig& rate, uint8_t listenSlots = kTelemetryResponseListenSlots);
uint16_t hopForSequence(uint16_t sequence, uint8_t telemetryRatio, uint8_t listenSlots = kTelemetryResponseListenSlots);
uint16_t nextHopAfterReceivedSequence(uint16_t sequence, uint8_t telemetryRatio,
                                      uint8_t listenSlots = kTelemetryResponseListenSlots);
size_t encodeCrsfRcFrame(const uint16_t channels[kRcChannelCount], uint8_t* out, size_t outLen);
bool parseCrsfRcFrame(uint8_t byte, uint16_t channels[kRcChannelCount]);
size_t encodeCrsfLinkStats(int16_t rssiDbm, int8_t snr, uint8_t linkQuality, uint8_t activeRate,
                           uint8_t* out, size_t outLen);
bool parseCrsfBindingRequestFrame(uint8_t byte, BindingControlRequest& out);
size_t encodeCrsfBindingRequestFrame(const BindingControlRequest& request, uint8_t* out, size_t outLen);
size_t encodeCrsfBindingResponseFrame(const BindingStatus& status, BindingResult result,
                                      BindingControlOp op, uint8_t* out, size_t outLen);
bool parseCrsfBindingResponseFrame(uint8_t byte, BindingStatus& status, BindingResult& result,
                                   BindingControlOp& op);

void configDefaults(DeviceConfig& cfg, const char* defaultPhrase);
ConfigRecord makeConfigRecord(const DeviceConfig& cfg);
bool readConfigRecord(const ConfigRecord& record, DeviceConfig& cfg);
bool readConfigRecord(const ConfigRecord& record, DeviceConfig& cfg, bool& migratedFromV1);

uint8_t fhssChannelFor(const uint8_t uid[kUidSize], uint16_t hop);
float fhssFrequencyMHz(uint8_t channel);

} // namespace lora_link
