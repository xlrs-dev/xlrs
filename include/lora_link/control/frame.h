#pragma once

#include <stddef.h>
#include <stdint.h>

#include "lora_link/protocol.h"

namespace lora_link {
namespace control {

constexpr uint8_t kFrameMagic = 0xBC;
constexpr uint8_t kFrameVersion = 1;
constexpr uint8_t kFrameHeaderSize = 4;
constexpr uint8_t kFrameCrcSize = 2;
constexpr uint8_t kMaxBindingPhraseLen = 32;
constexpr uint8_t kMaxPayloadSize = 40;
constexpr uint8_t kMaxFrameSize = kFrameHeaderSize + kMaxPayloadSize + kFrameCrcSize;
constexpr uint8_t kBindingIdentityPayloadSize = kUidSize + 4;

enum class CommandId : uint8_t {
    Hello = 0x01,
    Status = 0x02,
    BindingGet = 0x10,
    BindingSet = 0x11,
    BindingClear = 0x12,
    BindingVerify = 0x13,
};

enum class DecodeStatus : uint8_t {
    Ok,
    BadLength,
    BadMagic,
    BadVersion,
    BadCrc,
};

enum class BindingControlResult : uint8_t {
    ResponseReady,
    Ignored,
    InvalidFrame,
    InvalidPayload,
};

struct Frame {
    uint8_t command;
    uint8_t payloadLen;
    uint8_t payload[kMaxPayloadSize];
};

struct BindingControlState {
    char bindingPhrase[kMaxBindingPhraseLen + 1];
    char defaultPhrase[kMaxBindingPhraseLen + 1];
    bool rebootRequired;
};

size_t encodedFrameSize(uint8_t payloadLen);
bool isKnownCommand(uint8_t command);
bool validateBindingPhrase(const uint8_t* phrase, size_t len);
bool validateBindingPhrase(const char* phrase);

bool encodeFrame(uint8_t command, const uint8_t* payload, uint8_t payloadLen,
                 uint8_t* out, size_t outLen, size_t& written);
DecodeStatus decodeFrame(const uint8_t* data, size_t len, Frame& out);

void initBindingControlState(BindingControlState& state, const char* bindingPhrase,
                             const char* defaultPhrase);
bool makeBindingIdentityPayload(const char* phrase, uint8_t out[kBindingIdentityPayloadSize]);
bool decodeBindingIdentityPayload(const uint8_t payload[kBindingIdentityPayloadSize],
                                  uint8_t uid[kUidSize], uint32_t& uidCheckOut);
BindingControlResult handleBindingControlFrame(const uint8_t* data, size_t len,
                                               BindingControlState& state, Frame& response);

} // namespace control
} // namespace lora_link
