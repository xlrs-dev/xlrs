#include "lora_link/control/frame.h"

#include <string.h>

namespace lora_link {
namespace control {

static void copyPhrase(char* dst, const char* src) {
    memset(dst, 0, kMaxBindingPhraseLen + 1);
    if (!src || !src[0]) src = "default";
    strncpy(dst, src, kMaxBindingPhraseLen);
}

static void copyPhraseBytes(char* dst, const uint8_t* src, uint8_t len) {
    memset(dst, 0, kMaxBindingPhraseLen + 1);
    memcpy(dst, src, len);
}

static bool fillIdentityResponse(CommandId command, const char* phrase, Frame& response) {
    response.command = static_cast<uint8_t>(command);
    response.payloadLen = kBindingIdentityPayloadSize;
    return makeBindingIdentityPayload(phrase, response.payload);
}

size_t encodedFrameSize(uint8_t payloadLen) {
    return payloadLen <= kMaxPayloadSize ? kFrameHeaderSize + payloadLen + kFrameCrcSize : 0;
}

bool isKnownCommand(uint8_t command) {
    switch (static_cast<CommandId>(command)) {
    case CommandId::Hello:
    case CommandId::Status:
    case CommandId::BindingGet:
    case CommandId::BindingSet:
    case CommandId::BindingClear:
    case CommandId::BindingVerify:
        return true;
    }
    return false;
}

bool validateBindingPhrase(const uint8_t* phrase, size_t len) {
    if (!phrase || len < 1 || len > kMaxBindingPhraseLen) return false;
    for (size_t i = 0; i < len; ++i) {
        if (phrase[i] == 0) return false;
    }
    return true;
}

bool validateBindingPhrase(const char* phrase) {
    if (!phrase) return false;
    size_t len = 0;
    while (phrase[len] != '\0') {
        ++len;
        if (len > kMaxBindingPhraseLen) return false;
    }
    return len >= 1;
}

bool encodeFrame(uint8_t command, const uint8_t* payload, uint8_t payloadLen,
                 uint8_t* out, size_t outLen, size_t& written) {
    written = 0;
    const size_t frameLen = encodedFrameSize(payloadLen);
    if (!out || frameLen == 0 || outLen < frameLen) return false;
    if (payloadLen > 0 && !payload) return false;

    out[0] = kFrameMagic;
    out[1] = kFrameVersion;
    out[2] = command;
    out[3] = payloadLen;
    if (payloadLen > 0) memcpy(&out[kFrameHeaderSize], payload, payloadLen);

    const uint16_t crc = crc16Ccitt(out, kFrameHeaderSize + payloadLen);
    out[kFrameHeaderSize + payloadLen] = static_cast<uint8_t>(crc >> 8);
    out[kFrameHeaderSize + payloadLen + 1] = static_cast<uint8_t>(crc);
    written = frameLen;
    return true;
}

DecodeStatus decodeFrame(const uint8_t* data, size_t len, Frame& out) {
    memset(&out, 0, sizeof(out));
    if (!data || len < kFrameHeaderSize + kFrameCrcSize || len > kMaxFrameSize) {
        return DecodeStatus::BadLength;
    }
    if (data[0] != kFrameMagic) return DecodeStatus::BadMagic;
    if (data[1] != kFrameVersion) return DecodeStatus::BadVersion;

    const uint8_t payloadLen = data[3];
    if (payloadLen > kMaxPayloadSize || len != encodedFrameSize(payloadLen)) {
        return DecodeStatus::BadLength;
    }
    const uint16_t expectedCrc = static_cast<uint16_t>(
        (static_cast<uint16_t>(data[kFrameHeaderSize + payloadLen]) << 8) |
        data[kFrameHeaderSize + payloadLen + 1]);
    if (crc16Ccitt(data, kFrameHeaderSize + payloadLen) != expectedCrc) {
        return DecodeStatus::BadCrc;
    }

    out.command = data[2];
    out.payloadLen = payloadLen;
    if (payloadLen > 0) memcpy(out.payload, &data[kFrameHeaderSize], payloadLen);
    return DecodeStatus::Ok;
}

void initBindingControlState(BindingControlState& state, const char* bindingPhrase,
                             const char* defaultPhrase) {
    copyPhrase(state.bindingPhrase, bindingPhrase);
    copyPhrase(state.defaultPhrase, defaultPhrase);
    state.rebootRequired = false;
}

bool makeBindingIdentityPayload(const char* phrase, uint8_t out[kBindingIdentityPayloadSize]) {
    if (!out || !validateBindingPhrase(phrase)) return false;
    uint8_t uid[kUidSize];
    deriveUid(phrase, uid);
    memcpy(out, uid, kUidSize);
    const uint32_t check = uidCheck(uid);
    out[8] = static_cast<uint8_t>(check >> 24);
    out[9] = static_cast<uint8_t>(check >> 16);
    out[10] = static_cast<uint8_t>(check >> 8);
    out[11] = static_cast<uint8_t>(check);
    return true;
}

bool decodeBindingIdentityPayload(const uint8_t payload[kBindingIdentityPayloadSize],
                                  uint8_t uid[kUidSize], uint32_t& uidCheckOut) {
    if (!payload || !uid) return false;
    memcpy(uid, payload, kUidSize);
    uidCheckOut = (static_cast<uint32_t>(payload[8]) << 24) |
                  (static_cast<uint32_t>(payload[9]) << 16) |
                  (static_cast<uint32_t>(payload[10]) << 8) |
                  static_cast<uint32_t>(payload[11]);
    return uidCheck(uid) == uidCheckOut;
}

BindingControlResult handleBindingControlFrame(const uint8_t* data, size_t len,
                                               BindingControlState& state, Frame& response) {
    memset(&response, 0, sizeof(response));
    Frame request{};
    if (decodeFrame(data, len, request) != DecodeStatus::Ok) {
        return BindingControlResult::InvalidFrame;
    }
    if (!isKnownCommand(request.command)) return BindingControlResult::Ignored;

    switch (static_cast<CommandId>(request.command)) {
    case CommandId::Hello:
        response.command = request.command;
        response.payloadLen = 4;
        response.payload[0] = kFrameVersion;
        response.payload[1] = kMaxPayloadSize;
        response.payload[2] = kMaxBindingPhraseLen;
        response.payload[3] = state.rebootRequired ? 1 : 0;
        return BindingControlResult::ResponseReady;

    case CommandId::Status:
        return fillIdentityResponse(CommandId::Status, state.bindingPhrase, response)
                   ? BindingControlResult::ResponseReady
                   : BindingControlResult::InvalidPayload;

    case CommandId::BindingGet:
        response.command = request.command;
        response.payloadLen = static_cast<uint8_t>(strlen(state.bindingPhrase));
        memcpy(response.payload, state.bindingPhrase, response.payloadLen);
        return BindingControlResult::ResponseReady;

    case CommandId::BindingSet:
        if (!validateBindingPhrase(request.payload, request.payloadLen)) {
            return BindingControlResult::InvalidPayload;
        }
        copyPhraseBytes(state.bindingPhrase, request.payload, request.payloadLen);
        state.rebootRequired = true;
        return fillIdentityResponse(CommandId::BindingSet, state.bindingPhrase, response)
                   ? BindingControlResult::ResponseReady
                   : BindingControlResult::InvalidPayload;

    case CommandId::BindingClear:
        if (!validateBindingPhrase(state.defaultPhrase)) {
            return BindingControlResult::InvalidPayload;
        }
        copyPhrase(state.bindingPhrase, state.defaultPhrase);
        state.rebootRequired = true;
        return fillIdentityResponse(CommandId::BindingClear, state.bindingPhrase, response)
                   ? BindingControlResult::ResponseReady
                   : BindingControlResult::InvalidPayload;

    case CommandId::BindingVerify: {
        char phrase[kMaxBindingPhraseLen + 1];
        if (request.payloadLen == 0) {
            copyPhrase(phrase, state.bindingPhrase);
        } else {
            if (!validateBindingPhrase(request.payload, request.payloadLen)) {
                return BindingControlResult::InvalidPayload;
            }
            copyPhraseBytes(phrase, request.payload, request.payloadLen);
        }
        return fillIdentityResponse(CommandId::BindingVerify, phrase, response)
                   ? BindingControlResult::ResponseReady
                   : BindingControlResult::InvalidPayload;
    }
    }

    return BindingControlResult::Ignored;
}

} // namespace control
} // namespace lora_link
