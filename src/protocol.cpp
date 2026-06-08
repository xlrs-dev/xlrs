#include "lora_link/protocol.h"

#include <string.h>

namespace lora_link {

const RateConfig kRates[2] = {
    {"L250", 4000, 16, 812.5f, 6, 5},
    {"L100", 10000, 128, 812.5f, 7, 5},
};

static constexpr uint32_t kConfigMagic = 0x314C524Cu; // LRL1
static constexpr uint8_t kConfigVersion = 1;
static constexpr uint8_t kOtaMagic = 0xA7;
static constexpr uint8_t kOtaHeaderSize = 9;
static constexpr uint8_t kOtaCrcOffset = kOtaHeaderSize + kOtaPayloadSize;
static constexpr uint8_t kCrsfAddressFlightController = 0xC8;
static constexpr uint8_t kCrsfAddressRadioTransmitter = 0xEA;
static constexpr uint8_t kCrsfAddressCrsfTransmitter = 0xEE;
static constexpr uint8_t kCrsfFrameRcChannelsPacked = 0x16;
static constexpr uint8_t kCrsfFrameLinkStatistics = 0x14;
static constexpr uint16_t kSpikeJumpThreshold = 450;
static constexpr uint16_t kSpikeHighThreshold = kCrsfRaw2000 - 24;

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

uint8_t crsfCrc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5)
                               : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

void deriveUid(const char* phrase, uint8_t uid[kUidSize]) {
    uint64_t hash = 14695981039346656037ull;
    if (!phrase) phrase = "";
    while (*phrase) {
        hash ^= static_cast<uint8_t>(*phrase++);
        hash *= 1099511628211ull;
    }
    for (uint8_t i = 0; i < kUidSize; ++i) {
        uid[i] = static_cast<uint8_t>(hash >> (8 * i));
    }
}

uint32_t uidCheck(const uint8_t uid[kUidSize]) {
    return (static_cast<uint32_t>(uid[0]) << 24) |
           (static_cast<uint32_t>(uid[2]) << 16) |
           (static_cast<uint32_t>(uid[5]) << 8) |
           static_cast<uint32_t>(uid[7]);
}

uint16_t syncWordFromUid(const uint8_t uid[kUidSize]) {
    uint16_t word = static_cast<uint16_t>((uid[1] << 8) | uid[6]);
    return word == 0 ? 0x12AD : word;
}

bool encodeOtaFrame(const OtaFrame& frame, uint8_t out[kOtaFrameSize]) {
    if (frame.payloadLen > kOtaPayloadSize) return false;
    memset(out, 0, kOtaFrameSize);
    out[0] = kOtaMagic;
    out[1] = static_cast<uint8_t>(frame.type);
    out[2] = static_cast<uint8_t>(frame.sequence >> 8);
    out[3] = static_cast<uint8_t>(frame.sequence);
    out[4] = static_cast<uint8_t>(frame.uidCheck >> 24);
    out[5] = static_cast<uint8_t>(frame.uidCheck >> 16);
    out[6] = static_cast<uint8_t>(frame.uidCheck >> 8);
    out[7] = static_cast<uint8_t>(frame.uidCheck);
    out[8] = frame.payloadLen;
    memcpy(&out[9], frame.payload, frame.payloadLen);
    const uint16_t crc = crc16Ccitt(out, kOtaCrcOffset);
    out[kOtaCrcOffset] = static_cast<uint8_t>(crc >> 8);
    out[kOtaCrcOffset + 1] = static_cast<uint8_t>(crc);
    return true;
}

bool decodeOtaFrame(const uint8_t in[kOtaFrameSize], uint32_t expectedUidCheck, OtaFrame& out) {
    const uint16_t expectedCrc = static_cast<uint16_t>((in[kOtaCrcOffset] << 8) | in[kOtaCrcOffset + 1]);
    if (in[0] != kOtaMagic || crc16Ccitt(in, kOtaCrcOffset) != expectedCrc) return false;
    const uint32_t gotUid = (static_cast<uint32_t>(in[4]) << 24) |
                            (static_cast<uint32_t>(in[5]) << 16) |
                            (static_cast<uint32_t>(in[6]) << 8) |
                            static_cast<uint32_t>(in[7]);
    if (gotUid != expectedUidCheck || in[8] > kOtaPayloadSize) return false;
    out.type = static_cast<OtaType>(in[1]);
    if (out.type != OtaType::Rc && out.type != OtaType::Telemetry) return false;
    out.sequence = static_cast<uint16_t>((in[2] << 8) | in[3]);
    out.uidCheck = gotUid;
    out.payloadLen = in[8];
    memcpy(out.payload, &in[9], out.payloadLen);
    return true;
}

void packRcChannels11Bit(const uint16_t channels[kRcChannelCount], uint8_t out[22]) {
    memset(out, 0, 22);
    uint16_t bitIndex = 0;
    for (uint8_t ch = 0; ch < kRcChannelCount; ++ch) {
        uint16_t value = channels[ch] & 0x07FF;
        for (uint8_t bit = 0; bit < 11; ++bit) {
            if (value & (1u << bit)) out[bitIndex >> 3] |= static_cast<uint8_t>(1u << (bitIndex & 7));
            ++bitIndex;
        }
    }
}

void unpackRcChannels11Bit(const uint8_t in[22], uint16_t channels[kRcChannelCount]) {
    uint16_t bitIndex = 0;
    for (uint8_t ch = 0; ch < kRcChannelCount; ++ch) {
        uint16_t value = 0;
        for (uint8_t bit = 0; bit < 11; ++bit) {
            if (in[bitIndex >> 3] & (1u << (bitIndex & 7))) value |= static_cast<uint16_t>(1u << bit);
            ++bitIndex;
        }
        channels[ch] = value;
    }
}

uint16_t clampCrsfRaw(uint16_t value) {
    if (value < kCrsfRawMin) return kCrsfRawMin;
    if (value > kCrsfRawMax) return kCrsfRawMax;
    return value;
}

bool sanitizeRcChannels(const uint16_t previous[kRcChannelCount], bool havePrevious,
                        uint16_t channels[kRcChannelCount]) {
    uint8_t largeJumpsToHigh = 0;
    for (uint8_t i = 0; i < kRcChannelCount; ++i) {
        channels[i] = clampCrsfRaw(channels[i]);
        if (havePrevious && previous) {
            const uint16_t oldValue = clampCrsfRaw(previous[i]);
            const uint16_t delta = oldValue > channels[i] ? oldValue - channels[i] : channels[i] - oldValue;
            if (delta >= kSpikeJumpThreshold && channels[i] >= kSpikeHighThreshold) ++largeJumpsToHigh;
        }
    }
    return largeJumpsToHigh < 4;
}

static bool hasLargeRcJump(const uint16_t previous[kRcChannelCount],
                           const uint16_t candidate[kRcChannelCount],
                           uint16_t threshold) {
    for (uint8_t i = 0; i < kRcChannelCount; ++i) {
        const uint16_t oldValue = clampCrsfRaw(previous[i]);
        const uint16_t newValue = clampCrsfRaw(candidate[i]);
        const uint16_t delta = oldValue > newValue ? oldValue - newValue : newValue - oldValue;
        if (delta > threshold) return true;
    }
    return false;
}

static bool rcChannelsMatchWithin(const uint16_t a[kRcChannelCount],
                                  const uint16_t b[kRcChannelCount],
                                  uint16_t tolerance) {
    for (uint8_t i = 0; i < kRcChannelCount; ++i) {
        const uint16_t av = clampCrsfRaw(a[i]);
        const uint16_t bv = clampCrsfRaw(b[i]);
        const uint16_t delta = av > bv ? av - bv : bv - av;
        if (delta > tolerance) return false;
    }
    return true;
}

bool acceptRcChannelsWithSpikeGate(const uint16_t previous[kRcChannelCount], bool havePrevious,
                                   const uint16_t candidate[kRcChannelCount], RcSpikeGate& gate,
                                   uint16_t jumpThreshold, uint16_t confirmTolerance) {
    if (!havePrevious || !previous) {
        gate.havePending = false;
        return true;
    }
    if (!hasLargeRcJump(previous, candidate, jumpThreshold)) {
        gate.havePending = false;
        return true;
    }
    if (gate.havePending && rcChannelsMatchWithin(gate.pending, candidate, confirmTolerance)) {
        gate.havePending = false;
        return true;
    }
    memcpy(gate.pending, candidate, sizeof(gate.pending));
    gate.havePending = true;
    return false;
}

void slewLimitPrimaryRcChannels(const uint16_t previous[kRcChannelCount], bool havePrevious,
                                uint16_t channels[kRcChannelCount], uint16_t maxDelta) {
    if (!havePrevious || !previous) return;
    for (uint8_t i = 0; i < 4; ++i) {
        const uint16_t oldValue = clampCrsfRaw(previous[i]);
        channels[i] = clampCrsfRaw(channels[i]);
        if (channels[i] > oldValue && channels[i] - oldValue > maxDelta) {
            channels[i] = static_cast<uint16_t>(oldValue + maxDelta);
        } else if (oldValue > channels[i] && oldValue - channels[i] > maxDelta) {
            channels[i] = static_cast<uint16_t>(oldValue - maxDelta);
        }
    }
}

size_t encodeCrsfRcFrame(const uint16_t channels[kRcChannelCount], uint8_t* out, size_t outLen) {
    if (!out || outLen < 26) return 0;
    uint16_t sanitized[kRcChannelCount];
    memcpy(sanitized, channels, sizeof(sanitized));
    sanitizeRcChannels(nullptr, false, sanitized);
    out[0] = kCrsfAddressFlightController;
    out[1] = 24; // type + 22-byte payload + crc
    out[2] = kCrsfFrameRcChannelsPacked;
    packRcChannels11Bit(sanitized, &out[3]);
    out[25] = crsfCrc8(&out[2], 23);
    return 26;
}

bool parseCrsfRcFrame(uint8_t byte, uint16_t channels[kRcChannelCount]) {
    static uint8_t buf[64];
    static uint8_t pos = 0;
    if (pos == 0 && byte != kCrsfAddressFlightController &&
        byte != kCrsfAddressRadioTransmitter &&
        byte != kCrsfAddressCrsfTransmitter) {
        return false;
    }
    buf[pos++] = byte;
    if (pos == 2 && (buf[1] < 2 || buf[1] > 62)) {
        pos = 0;
        return false;
    }
    if (pos >= 2 && pos == static_cast<uint8_t>(buf[1] + 2)) {
        const uint8_t len = buf[1];
        const bool ok = buf[2] == kCrsfFrameRcChannelsPacked &&
                        len == 24 &&
                        crsfCrc8(&buf[2], len - 1) == buf[pos - 1];
        if (ok) {
            unpackRcChannels11Bit(&buf[3], channels);
        }
        pos = 0;
        return ok;
    }
    if (pos >= sizeof(buf)) pos = 0;
    return false;
}

size_t encodeCrsfLinkStats(int16_t rssiDbm, int8_t snr, uint8_t linkQuality, uint8_t activeRate,
                           uint8_t* out, size_t outLen) {
    if (!out || outLen < 14) return 0;
    out[0] = kCrsfAddressFlightController;
    out[1] = 12; // type + 10-byte payload + crc
    out[2] = kCrsfFrameLinkStatistics;
    out[3] = static_cast<uint8_t>(rssiDbm < 0 ? -rssiDbm : rssiDbm);
    out[4] = 0;
    out[5] = linkQuality;
    out[6] = static_cast<uint8_t>(snr);
    out[7] = 0;
    out[8] = 0;
    out[9] = activeRate;
    out[10] = 0;
    out[11] = 0;
    out[12] = 0;
    out[13] = crsfCrc8(&out[2], 11);
    return 14;
}

void configDefaults(DeviceConfig& cfg, const char* defaultPhrase) {
    memset(&cfg, 0, sizeof(cfg));
    if (!defaultPhrase || !defaultPhrase[0]) defaultPhrase = "default";
    strncpy(cfg.bindingPhrase, defaultPhrase, sizeof(cfg.bindingPhrase) - 1);
    cfg.rate = RateId::L100;
}

ConfigRecord makeConfigRecord(const DeviceConfig& cfg) {
    ConfigRecord record{};
    record.magic = kConfigMagic;
    record.version = kConfigVersion;
    record.rate = static_cast<uint8_t>(cfg.rate);
    strncpy(record.bindingPhrase, cfg.bindingPhrase, sizeof(record.bindingPhrase) - 1);
    record.crc = crc16Ccitt(reinterpret_cast<const uint8_t*>(&record), offsetof(ConfigRecord, crc));
    return record;
}

bool readConfigRecord(const ConfigRecord& record, DeviceConfig& cfg) {
    if (record.magic != kConfigMagic || record.version != kConfigVersion || record.rate > 1) return false;
    const uint16_t crc = crc16Ccitt(reinterpret_cast<const uint8_t*>(&record), offsetof(ConfigRecord, crc));
    if (crc != record.crc || record.bindingPhrase[0] == '\0') return false;
    memset(&cfg, 0, sizeof(cfg));
    strncpy(cfg.bindingPhrase, record.bindingPhrase, sizeof(cfg.bindingPhrase) - 1);
    cfg.rate = static_cast<RateId>(record.rate);
    return true;
}

uint8_t fhssChannelFor(const uint8_t uid[kUidSize], uint16_t hop) {
    uint16_t x = static_cast<uint16_t>((uid[0] << 8) | uid[3]);
    x ^= static_cast<uint16_t>(hop * 0x45D9u);
    x ^= static_cast<uint16_t>(x >> 7);
    return static_cast<uint8_t>(x % 40);
}

float fhssFrequencyMHz(uint8_t channel) {
    return 2404.0f + static_cast<float>(channel % 40) * 2.0f;
}

} // namespace lora_link
