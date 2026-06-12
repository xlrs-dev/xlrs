#include "lora_link/protocol.h"

#include <string.h>

namespace lora_link {

const RateConfig kRates[2] = {
    {"L250", 4000, 0, 4, 812.5f, 5, 5},
    {"L100", 40000, 0, 4, 812.5f, 6, 5},
};

static constexpr uint32_t kConfigMagic = 0x314C524Cu; // LRL1
static constexpr uint8_t kConfigVersion = 1;
static constexpr uint8_t kOtaMagic = 0xA7;
static constexpr uint8_t kOtaHeaderSize = 9;
static constexpr uint8_t kOtaCrcOffset = kOtaHeaderSize + kOtaPayloadSize;
static constexpr uint8_t kOtaSyncPayloadSize = 16;
static constexpr uint8_t kDownlinkTelemetryPayloadVersion = 1;
static constexpr uint8_t kDownlinkTelemetryPayloadSize = 12;
static constexpr uint8_t kCrsfAddressFlightController = 0xC8;
static constexpr uint8_t kCrsfAddressRadioTransmitter = 0xEA;
static constexpr uint8_t kCrsfAddressCrsfTransmitter = 0xEE;
static constexpr uint8_t kCrsfFrameRcChannelsPacked = 0x16;
static constexpr uint8_t kCrsfFrameLinkStatistics = 0x14;
static constexpr uint8_t kCrsfFrameXlrsBindingControl = 0x7D;
static constexpr uint8_t kBindingControlVersion = 1;
static constexpr uint8_t kBindingControlMagic[4] = {'X', 'L', 'R', 'S'};
static constexpr uint16_t kSpikeJumpThreshold = 450;
static constexpr uint16_t kSpikeHighThreshold = kCrsfRaw2000 - 24;
static constexpr uint8_t kConfigRecordCrcSize = 4 + 1 + 1 + 33 + 1;

static_assert(offsetof(ConfigRecord, crc) == kConfigRecordCrcSize,
              "ConfigRecord crc offset must match explicit serialized CRC bytes");

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
    uint32_t hash = 2166136261u;
    for (uint8_t i = 0; i < kUidSize; ++i) {
        hash ^= uid[i];
        hash *= 16777619u;
    }
    return hash;
}

uint8_t syncWordFromUid(const uint8_t uid[kUidSize]) {
    uint8_t word = 0xA7;
    for (uint8_t i = 0; i < kUidSize; ++i) {
        word = static_cast<uint8_t>((word << 3) | (word >> 5));
        word ^= static_cast<uint8_t>(uid[i] + 0x9Du + static_cast<uint8_t>(i * 0x13u));
    }
    word ^= static_cast<uint8_t>(uidCheck(uid) >> 24);
    if (word == 0x00 || word == 0x34) word ^= 0x5A;
    if (word == 0x00) word = 0x12;
    return word;
}

bool validateBindingPhrase(const char* phrase) {
    if (!phrase) return false;
    size_t len = 0;
    while (phrase[len]) {
        const uint8_t c = static_cast<uint8_t>(phrase[len]);
        if (c < 0x20 || c == 0x7F) return false;
        ++len;
        if (len > kBindingPhraseMaxLength) return false;
    }
    return len > 0;
}

void makeBindingStatus(const char* phrase, bool persisted, bool requiresReboot, BindingStatus& out) {
    memset(&out, 0, sizeof(out));
    if (phrase) strncpy(out.phrase, phrase, sizeof(out.phrase) - 1);
    deriveUid(out.phrase, out.uid);
    out.uidCheck = uidCheck(out.uid);
    out.persisted = persisted;
    out.requiresReboot = requiresReboot;
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
    if (out.type != OtaType::Rc && out.type != OtaType::Telemetry && out.type != OtaType::Sync) return false;
    out.sequence = static_cast<uint16_t>((in[2] << 8) | in[3]);
    out.uidCheck = gotUid;
    out.payloadLen = in[8];
    memcpy(out.payload, &in[9], out.payloadLen);
    return true;
}

static void writeBe16(uint8_t* out, uint16_t value) {
    out[0] = static_cast<uint8_t>(value >> 8);
    out[1] = static_cast<uint8_t>(value);
}

static void writeBe32(uint8_t* out, uint32_t value) {
    out[0] = static_cast<uint8_t>(value >> 24);
    out[1] = static_cast<uint8_t>(value >> 16);
    out[2] = static_cast<uint8_t>(value >> 8);
    out[3] = static_cast<uint8_t>(value);
}

static uint16_t readBe16(const uint8_t* in) {
    return static_cast<uint16_t>((static_cast<uint16_t>(in[0]) << 8) | in[1]);
}

static uint32_t readBe32(const uint8_t* in) {
    return (static_cast<uint32_t>(in[0]) << 24) |
           (static_cast<uint32_t>(in[1]) << 16) |
           (static_cast<uint32_t>(in[2]) << 8) |
           static_cast<uint32_t>(in[3]);
}

static uint8_t rssiMagnitudeByte(int16_t rssiDbm) {
    if (rssiDbm == 0) return 0;
    const int16_t magnitude = rssiDbm < 0 ? static_cast<int16_t>(-rssiDbm) : rssiDbm;
    return magnitude > 255 ? 255 : static_cast<uint8_t>(magnitude);
}

static uint8_t crsfSignedDbmByte(int16_t rssiDbm) {
    if (rssiDbm < -128) rssiDbm = -128;
    if (rssiDbm > 127) rssiDbm = 127;
    return static_cast<uint8_t>(static_cast<int8_t>(rssiDbm));
}

static uint8_t clampPercent(uint8_t value) {
    return value > 100 ? 100 : value;
}

bool encodeOtaSyncPayload(const OtaSyncPayload& payload, uint8_t out[kOtaPayloadSize], uint8_t& payloadLen) {
    if (!isValidRateIndex(payload.rateIndex) || !isValidRateIndex(payload.nextRateIndex) ||
        payload.fhssHopInterval == 0) {
        return false;
    }
    memset(out, 0, kOtaPayloadSize);
    writeBe16(&out[0], payload.nonce);
    writeBe16(&out[2], payload.fhssIndex);
    out[4] = payload.rateIndex;
    out[5] = payload.nextRateIndex;
    writeBe32(&out[6], payload.switchSequence);
    out[10] = payload.telemetryRatio;
    out[11] = payload.fhssHopInterval;
    writeBe32(&out[12], payload.txTick);
    payloadLen = kOtaSyncPayloadSize;
    return true;
}

bool decodeOtaSyncPayload(const OtaFrame& frame, OtaSyncPayload& out) {
    if (frame.type != OtaType::Sync || frame.payloadLen != kOtaSyncPayloadSize) return false;
    out.nonce = readBe16(&frame.payload[0]);
    out.fhssIndex = readBe16(&frame.payload[2]);
    out.rateIndex = frame.payload[4];
    out.nextRateIndex = frame.payload[5];
    out.switchSequence = readBe32(&frame.payload[6]);
    out.telemetryRatio = frame.payload[10];
    out.fhssHopInterval = frame.payload[11];
    out.txTick = readBe32(&frame.payload[12]);
    return isValidRateIndex(out.rateIndex) && isValidRateIndex(out.nextRateIndex) &&
           out.fhssHopInterval != 0;
}

uint8_t rateIndexForConfig(const RateConfig& rate) {
    for (uint8_t i = 0; i < 2; ++i) {
        if (rate.intervalUs == kRates[i].intervalUs &&
            rate.telemetryRatio == kRates[i].telemetryRatio &&
            rate.fhssHopInterval == kRates[i].fhssHopInterval &&
            rate.spreadingFactor == kRates[i].spreadingFactor &&
            rate.codingRate == kRates[i].codingRate) {
            return i;
        }
    }
    return 0xFF;
}

bool isValidRateIndex(uint8_t rateIndex) {
    return rateIndex < 2;
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
    uint16_t sanitized[kRcChannelCount];
    for (uint8_t i = 0; i < kRcChannelCount; ++i) {
        sanitized[i] = clampCrsfRaw(channels[i]);
        if (havePrevious && previous) {
            const uint16_t oldValue = clampCrsfRaw(previous[i]);
            const uint16_t delta = oldValue > sanitized[i] ? oldValue - sanitized[i] : sanitized[i] - oldValue;
            if (delta >= kSpikeJumpThreshold && sanitized[i] >= kSpikeHighThreshold) ++largeJumpsToHigh;
        }
    }
    if (largeJumpsToHigh >= 4) return false;
    memcpy(channels, sanitized, sizeof(sanitized));
    return true;
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

static bool rcChannelsContinuePendingMove(const uint16_t previous[kRcChannelCount],
                                          const uint16_t pending[kRcChannelCount],
                                          const uint16_t candidate[kRcChannelCount],
                                          uint16_t jumpThreshold,
                                          uint16_t tolerance) {
    bool sawLargePendingMove = false;
    for (uint8_t i = 0; i < kRcChannelCount; ++i) {
        const uint16_t oldValue = clampCrsfRaw(previous[i]);
        const uint16_t pendingValue = clampCrsfRaw(pending[i]);
        const uint16_t candidateValue = clampCrsfRaw(candidate[i]);
        const uint16_t pendingDelta = oldValue > pendingValue ? oldValue - pendingValue
                                                              : pendingValue - oldValue;
        const uint16_t pendingToCandidate = pendingValue > candidateValue ? pendingValue - candidateValue
                                                                          : candidateValue - pendingValue;
        if (pendingDelta <= jumpThreshold) {
            if (pendingToCandidate > tolerance) return false;
            continue;
        }

        sawLargePendingMove = true;
        if (pendingValue > oldValue) {
            if (candidateValue + tolerance < pendingValue) return false;
        } else if (pendingValue + tolerance < oldValue) {
            if (candidateValue > pendingValue + tolerance) return false;
        }
    }
    return sawLargePendingMove;
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
    if (gate.havePending &&
        rcChannelsContinuePendingMove(previous, gate.pending, candidate, jumpThreshold, confirmTolerance)) {
        gate.havePending = false;
        return true;
    }
    memcpy(gate.pending, candidate, sizeof(gate.pending));
    gate.havePending = true;
    return false;
}

bool isLinkFresh(uint32_t nowMs, uint32_t lastFrameMs, uint32_t timeoutMs) {
    return lastFrameMs != 0 && static_cast<uint32_t>(nowMs - lastFrameMs) < timeoutMs;
}

bool shouldRequestTelemetry(uint16_t sequence, uint8_t telemetryRatio) {
    return telemetryRatio != 0 && (sequence % telemetryRatio) == 0;
}

uint32_t telemetryListenWindowMs(const RateConfig& rate, uint8_t listenSlots) {
    return (static_cast<uint32_t>(listenSlots) * rate.intervalUs + 999u) / 1000u;
}

uint16_t hopForSequence(uint16_t sequence, uint8_t telemetryRatio, uint8_t listenSlots) {
    if (telemetryRatio == 0 || listenSlots == 0 || sequence == 0) return sequence;
    const uint16_t previousTelemetryRequests = static_cast<uint16_t>(((sequence - 1u) / telemetryRatio) + 1u);
    return static_cast<uint16_t>(sequence + previousTelemetryRequests * listenSlots);
}

uint16_t nextHopAfterReceivedSequence(uint16_t sequence, uint8_t telemetryRatio, uint8_t listenSlots) {
    return hopForSequence(static_cast<uint16_t>(sequence + 1), telemetryRatio, listenSlots);
}

bool encodeDownlinkTelemetryPayload(const DownlinkTelemetryPayload& payload,
                                    uint8_t out[kOtaPayloadSize], uint8_t& payloadLen) {
    if (!out) return false;
    memset(out, 0, kOtaPayloadSize);
    out[0] = kDownlinkTelemetryPayloadVersion;
    out[1] = clampPercent(payload.uplinkLinkQuality);
    out[2] = rssiMagnitudeByte(payload.uplinkRssiDbm);
    out[3] = static_cast<uint8_t>(payload.uplinkSnrDb);
    out[4] = payload.rfMode;
    out[5] = payload.uplinkTxPower;
    out[6] = payload.faultFlags;
    out[7] = payload.fhssIndex;
    out[8] = payload.expectedFhssIndex;
    out[9] = static_cast<uint8_t>(payload.phaseErrorBucket);
    out[10] = payload.lostConnectionCount;
    out[11] = payload.radioRejectCount;
    payloadLen = kDownlinkTelemetryPayloadSize;
    return true;
}

bool decodeDownlinkTelemetryPayload(const OtaFrame& frame, DownlinkTelemetryPayload& out) {
    if (frame.type != OtaType::Telemetry) return false;
    memset(&out, 0, sizeof(out));
    if (frame.payloadLen == 4) {
        out.uplinkLinkQuality = clampPercent(frame.payload[0]);
        out.uplinkRssiDbm = -static_cast<int16_t>(frame.payload[1]);
        out.uplinkSnrDb = static_cast<int8_t>(frame.payload[2]);
        out.rfMode = frame.payload[3];
        return true;
    }
    if (frame.payloadLen < kDownlinkTelemetryPayloadSize ||
        frame.payload[0] != kDownlinkTelemetryPayloadVersion) {
        return false;
    }
    out.uplinkLinkQuality = clampPercent(frame.payload[1]);
    out.uplinkRssiDbm = -static_cast<int16_t>(frame.payload[2]);
    out.uplinkSnrDb = static_cast<int8_t>(frame.payload[3]);
    out.rfMode = frame.payload[4];
    out.uplinkTxPower = frame.payload[5];
    out.faultFlags = frame.payload[6];
    out.fhssIndex = frame.payload[7];
    out.expectedFhssIndex = frame.payload[8];
    out.phaseErrorBucket = static_cast<int8_t>(frame.payload[9]);
    out.lostConnectionCount = frame.payload[10];
    out.radioRejectCount = frame.payload[11];
    return true;
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

size_t encodeCrsfLinkStats(const LinkStats& stats, uint8_t* out, size_t outLen) {
    if (!out || outLen < 14) return 0;
    out[0] = kCrsfAddressFlightController;
    out[1] = 12; // type + 10-byte payload + crc
    out[2] = kCrsfFrameLinkStatistics;
    out[3] = crsfSignedDbmByte(stats.uplinkRssiDbm);
    out[4] = crsfSignedDbmByte(stats.uplinkRssi2Dbm);
    out[5] = clampPercent(stats.uplinkLinkQuality);
    out[6] = static_cast<uint8_t>(stats.uplinkSnrDb);
    out[7] = stats.activeAntenna;
    out[8] = stats.rfMode;
    out[9] = stats.uplinkTxPower;
    out[10] = crsfSignedDbmByte(stats.downlinkRssiDbm);
    out[11] = clampPercent(stats.downlinkLinkQuality);
    out[12] = static_cast<uint8_t>(stats.downlinkSnrDb);
    out[13] = crsfCrc8(&out[2], 11);
    return 14;
}

size_t encodeCrsfLinkStats(int16_t rssiDbm, int8_t snr, uint8_t linkQuality, uint8_t activeRate,
                           uint8_t* out, size_t outLen) {
    LinkStats stats{};
    stats.uplinkRssiDbm = rssiDbm;
    stats.uplinkLinkQuality = linkQuality;
    stats.uplinkSnrDb = snr;
    stats.rfMode = activeRate;
    return encodeCrsfLinkStats(stats, out, outLen);
}

static bool crsfAddressCanCarryBindingControl(uint8_t address) {
    return address == kCrsfAddressRadioTransmitter || address == kCrsfAddressCrsfTransmitter;
}

static size_t boundedPhraseLen(const char* phrase) {
    size_t phraseLen = 0;
    while (phrase && phraseLen < kBindingPhraseMaxLength && phrase[phraseLen]) ++phraseLen;
    return phraseLen;
}

static bool bindingControlMagicMatches(const uint8_t* payload, size_t payloadLen) {
    return payloadLen >= sizeof(kBindingControlMagic) &&
           memcmp(payload, kBindingControlMagic, sizeof(kBindingControlMagic)) == 0;
}

bool parseCrsfBindingRequestFrame(uint8_t byte, BindingControlRequest& out) {
    static uint8_t buf[64];
    static uint8_t pos = 0;
    if (pos == 0 && !crsfAddressCanCarryBindingControl(byte)) {
        return false;
    }
    buf[pos++] = byte;
    if (pos == 2 && (buf[1] < 2 || buf[1] > 62)) {
        pos = 0;
        return false;
    }
    if (pos >= 2 && pos == static_cast<uint8_t>(buf[1] + 2)) {
        const uint8_t len = buf[1];
        const bool crcOk = crsfCrc8(&buf[2], len - 1) == buf[pos - 1];
        const uint8_t payloadLen = static_cast<uint8_t>(len - 2);
        const uint8_t* payload = &buf[3];
        bool parsed = false;
        if (crcOk && buf[2] == kCrsfFrameXlrsBindingControl &&
            payloadLen >= 7 && bindingControlMagicMatches(payload, payloadLen) &&
            payload[4] == kBindingControlVersion) {
            const uint8_t phraseLen = payload[6];
            if (phraseLen <= kBindingPhraseMaxLength &&
                static_cast<uint8_t>(7 + phraseLen) <= payloadLen) {
                memset(&out, 0, sizeof(out));
                out.op = static_cast<BindingControlOp>(payload[5]);
                memcpy(out.phrase, &payload[7], phraseLen);
                out.phrase[phraseLen] = '\0';
                parsed = true;
            }
        }
        pos = 0;
        return parsed;
    }
    if (pos >= sizeof(buf)) pos = 0;
    return false;
}

size_t encodeCrsfBindingRequestFrame(const BindingControlRequest& request, uint8_t* out, size_t outLen) {
    const size_t phraseLen = boundedPhraseLen(request.phrase);
    const size_t payloadLen = 4 + 1 + 1 + 1 + phraseLen;
    const size_t frameLen = payloadLen + 4; // address + length + type + payload + crc
    if (!out || outLen < frameLen || payloadLen > 60) return 0;

    out[0] = kCrsfAddressCrsfTransmitter;
    out[1] = static_cast<uint8_t>(payloadLen + 2);
    out[2] = kCrsfFrameXlrsBindingControl;
    size_t pos = 3;
    memcpy(&out[pos], kBindingControlMagic, sizeof(kBindingControlMagic));
    pos += sizeof(kBindingControlMagic);
    out[pos++] = kBindingControlVersion;
    out[pos++] = static_cast<uint8_t>(request.op);
    out[pos++] = static_cast<uint8_t>(phraseLen);
    memcpy(&out[pos], request.phrase, phraseLen);
    pos += phraseLen;
    out[pos++] = crsfCrc8(&out[2], out[1] - 1);
    return pos;
}

size_t encodeCrsfBindingResponseFrame(const BindingStatus& status, BindingResult result,
                                      BindingControlOp op, uint8_t* out, size_t outLen) {
    const size_t phraseLen = boundedPhraseLen(status.phrase);
    const size_t payloadLen = 4 + 1 + 1 + 1 + 1 + 1 + 4 + kUidSize + 1 + phraseLen;
    const size_t frameLen = payloadLen + 4; // address + length + type + payload + crc
    if (!out || outLen < frameLen || payloadLen > 60) return 0;

    out[0] = kCrsfAddressRadioTransmitter;
    out[1] = static_cast<uint8_t>(payloadLen + 2);
    out[2] = kCrsfFrameXlrsBindingControl;
    size_t pos = 3;
    memcpy(&out[pos], kBindingControlMagic, sizeof(kBindingControlMagic));
    pos += sizeof(kBindingControlMagic);
    out[pos++] = kBindingControlVersion;
    out[pos++] = static_cast<uint8_t>(op);
    out[pos++] = static_cast<uint8_t>(result);
    out[pos++] = status.persisted ? 1 : 0;
    out[pos++] = status.requiresReboot ? 1 : 0;
    out[pos++] = static_cast<uint8_t>(status.uidCheck >> 24);
    out[pos++] = static_cast<uint8_t>(status.uidCheck >> 16);
    out[pos++] = static_cast<uint8_t>(status.uidCheck >> 8);
    out[pos++] = static_cast<uint8_t>(status.uidCheck);
    memcpy(&out[pos], status.uid, kUidSize);
    pos += kUidSize;
    out[pos++] = static_cast<uint8_t>(phraseLen);
    memcpy(&out[pos], status.phrase, phraseLen);
    pos += phraseLen;
    out[pos++] = crsfCrc8(&out[2], out[1] - 1);
    return pos;
}

bool parseCrsfBindingResponseFrame(uint8_t byte, BindingStatus& status, BindingResult& result,
                                   BindingControlOp& op) {
    static uint8_t buf[64];
    static uint8_t pos = 0;
    if (pos == 0 && !crsfAddressCanCarryBindingControl(byte)) {
        return false;
    }
    buf[pos++] = byte;
    if (pos == 2 && (buf[1] < 2 || buf[1] > 62)) {
        pos = 0;
        return false;
    }
    if (pos >= 2 && pos == static_cast<uint8_t>(buf[1] + 2)) {
        const uint8_t len = buf[1];
        const bool crcOk = crsfCrc8(&buf[2], len - 1) == buf[pos - 1];
        const uint8_t payloadLen = static_cast<uint8_t>(len - 2);
        const uint8_t* payload = &buf[3];
        bool parsed = false;
        if (crcOk && buf[2] == kCrsfFrameXlrsBindingControl &&
            payloadLen >= 22 && bindingControlMagicMatches(payload, payloadLen) &&
            payload[4] == kBindingControlVersion) {
            const uint8_t phraseLen = payload[21];
            if (phraseLen <= kBindingPhraseMaxLength &&
                static_cast<uint8_t>(22 + phraseLen) <= payloadLen) {
                memset(&status, 0, sizeof(status));
                op = static_cast<BindingControlOp>(payload[5]);
                result = static_cast<BindingResult>(payload[6]);
                status.persisted = payload[7] != 0;
                status.requiresReboot = payload[8] != 0;
                status.uidCheck = (static_cast<uint32_t>(payload[9]) << 24) |
                                  (static_cast<uint32_t>(payload[10]) << 16) |
                                  (static_cast<uint32_t>(payload[11]) << 8) |
                                  static_cast<uint32_t>(payload[12]);
                memcpy(status.uid, &payload[13], kUidSize);
                memcpy(status.phrase, &payload[22], phraseLen);
                status.phrase[phraseLen] = '\0';
                parsed = true;
            }
        }
        pos = 0;
        return parsed;
    }
    if (pos >= sizeof(buf)) pos = 0;
    return false;
}

void configDefaults(DeviceConfig& cfg, const char* defaultPhrase) {
    memset(&cfg, 0, sizeof(cfg));
    if (!defaultPhrase || !defaultPhrase[0]) defaultPhrase = "default";
    strncpy(cfg.bindingPhrase, defaultPhrase, sizeof(cfg.bindingPhrase) - 1);
    cfg.rate = RateId::L100;
}

static void writeLe32(uint8_t* out, uint32_t value) {
    out[0] = static_cast<uint8_t>(value);
    out[1] = static_cast<uint8_t>(value >> 8);
    out[2] = static_cast<uint8_t>(value >> 16);
    out[3] = static_cast<uint8_t>(value >> 24);
}

static uint16_t configRecordCrc(const ConfigRecord& record) {
    uint8_t bytes[kConfigRecordCrcSize] = {};
    writeLe32(bytes, record.magic);
    bytes[4] = record.version;
    bytes[5] = record.rate;
    memcpy(&bytes[6], record.bindingPhrase, sizeof(record.bindingPhrase));
    bytes[39] = record.reserved;
    return crc16Ccitt(bytes, sizeof(bytes));
}

ConfigRecord makeConfigRecord(const DeviceConfig& cfg) {
    ConfigRecord record{};
    record.magic = kConfigMagic;
    record.version = kConfigVersion;
    record.rate = static_cast<uint8_t>(cfg.rate);
    strncpy(record.bindingPhrase, cfg.bindingPhrase, sizeof(record.bindingPhrase) - 1);
    record.reserved = 0;
    record.crc = configRecordCrc(record);
    return record;
}

bool readConfigRecord(const ConfigRecord& record, DeviceConfig& cfg) {
    if (record.magic != kConfigMagic || record.version != kConfigVersion || record.rate > 1) return false;
    const uint16_t crc = configRecordCrc(record);
    if (crc != record.crc || record.bindingPhrase[0] == '\0') return false;
    memset(&cfg, 0, sizeof(cfg));
    strncpy(cfg.bindingPhrase, record.bindingPhrase, sizeof(cfg.bindingPhrase) - 1);
    cfg.rate = static_cast<RateId>(record.rate);
    return true;
}

uint8_t fhssChannelFor(const uint8_t uid[kUidSize], uint16_t hop) {
    static constexpr uint8_t kCoprimeStrides[] = {
        1, 3, 7, 9, 11, 13, 17, 19, 21, 23, 27, 29, 31, 33, 37, 39,
    };
    uint8_t offset = 0;
    uint8_t selector = 0;
    for (uint8_t i = 0; i < kUidSize; ++i) {
        offset = static_cast<uint8_t>((offset + uid[i]) % kFhssChannelCount);
        selector ^= static_cast<uint8_t>((uid[i] << (i & 3u)) | (uid[i] >> (8u - (i & 3u))));
    }
    const uint8_t stride = kCoprimeStrides[selector & 0x0Fu];
    return static_cast<uint8_t>((offset + (hop % kFhssChannelCount) * stride) % kFhssChannelCount);
}

float fhssFrequencyMHz(uint8_t channel) {
    return 2404.0f + static_cast<float>(channel % kFhssChannelCount) * 2.0f;
}

} // namespace lora_link
