// Helpers for translating CRSF packed RC channels at application boundaries.
#pragma once

#include <stdint.h>
#include <string.h>

#include "crsf_protocol.h"
#include "ota/ChannelPack.h"

namespace xlrs {

static inline uint16_t clampRcUs(int32_t value) {
    if (value < 1000) return 1000;
    if (value > 2000) return 2000;
    return (uint16_t)value;
}

static inline uint16_t crsfChannelToRcUs(uint16_t value) {
    const int32_t span = CRSF_CHANNEL_VALUE_2000 - CRSF_CHANNEL_VALUE_1000;
    const int32_t mapped = 1000 + (((int32_t)value - CRSF_CHANNEL_VALUE_1000) * 1000 + span / 2) /
                                  span;
    return clampRcUs(mapped);
}

static inline uint16_t rcUsToCrsfChannel(uint16_t value) {
    const uint16_t clamped = clampRcUs(value);
    const int32_t span = CRSF_CHANNEL_VALUE_2000 - CRSF_CHANNEL_VALUE_1000;
    return (uint16_t)(CRSF_CHANNEL_VALUE_1000 +
                      (((int32_t)clamped - 1000) * span + 500) / 1000);
}

static inline void crsfChannelsToRcUs(const crsf_channels_t& input,
                                      uint16_t output[CRSF_NUM_CHANNELS]) {
    static_assert(sizeof(crsf_channels_t) == CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE,
                  "crsf_channels_t must match the CRSF packed channel payload size");
    uint16_t raw[CRSF_NUM_CHANNELS] = {};
    unpackChannels(reinterpret_cast<const uint8_t*>(&input), CRSF_NUM_CHANNELS, raw);

    for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        output[i] = crsfChannelToRcUs(raw[i]);
    }
}

static inline void rcUsToCrsfChannels(const uint16_t input[CRSF_NUM_CHANNELS],
                                      crsf_channels_t& output) {
    static_assert(sizeof(crsf_channels_t) == CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE,
                  "crsf_channels_t must match the CRSF packed channel payload size");
    uint16_t raw[CRSF_NUM_CHANNELS] = {};
    for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        raw[i] = rcUsToCrsfChannel(input[i]);
    }
    memset(&output, 0, sizeof(output));
    packChannels(raw, CRSF_NUM_CHANNELS, reinterpret_cast<uint8_t*>(&output));
}

} // namespace xlrs
