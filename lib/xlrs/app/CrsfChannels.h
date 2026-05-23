// Helpers for translating CRSF packed RC channels at application boundaries.
#pragma once

#include <stdint.h>

#include "crsf_protocol.h"

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
    const uint16_t raw[CRSF_NUM_CHANNELS] = {
        (uint16_t)input.ch0,  (uint16_t)input.ch1,  (uint16_t)input.ch2,
        (uint16_t)input.ch3,  (uint16_t)input.ch4,  (uint16_t)input.ch5,
        (uint16_t)input.ch6,  (uint16_t)input.ch7,  (uint16_t)input.ch8,
        (uint16_t)input.ch9,  (uint16_t)input.ch10, (uint16_t)input.ch11,
        (uint16_t)input.ch12, (uint16_t)input.ch13, (uint16_t)input.ch14,
        (uint16_t)input.ch15,
    };

    for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        output[i] = crsfChannelToRcUs(raw[i]);
    }
}

static inline void setCrsfChannelByIndex(crsf_channels_t& output, uint8_t index, uint16_t value) {
    switch (index) {
        case 0: output.ch0 = value; break;
        case 1: output.ch1 = value; break;
        case 2: output.ch2 = value; break;
        case 3: output.ch3 = value; break;
        case 4: output.ch4 = value; break;
        case 5: output.ch5 = value; break;
        case 6: output.ch6 = value; break;
        case 7: output.ch7 = value; break;
        case 8: output.ch8 = value; break;
        case 9: output.ch9 = value; break;
        case 10: output.ch10 = value; break;
        case 11: output.ch11 = value; break;
        case 12: output.ch12 = value; break;
        case 13: output.ch13 = value; break;
        case 14: output.ch14 = value; break;
        case 15: output.ch15 = value; break;
        default: break;
    }
}

static inline void rcUsToCrsfChannels(const uint16_t input[CRSF_NUM_CHANNELS],
                                      crsf_channels_t& output) {
    for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        setCrsfChannelByIndex(output, i, rcUsToCrsfChannel(input[i]));
    }
}

} // namespace xlrs
