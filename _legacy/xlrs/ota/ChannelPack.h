// ChannelPack — 11-bit channel (un)packing (CRSF-compatible bit order).
//
// Each channel is an 11-bit value (0..2047). N channels pack into ceil(N*11/8) bytes,
// LSB-first. Pure, header-only, host-testable. Scales to any channel count.
#pragma once
#include <stdint.h>

namespace xlrs {

static constexpr uint16_t CHANNEL_11BIT_MAX = 0x7FF; // 2047

// Bytes needed to pack `n` 11-bit channels.
inline constexpr uint8_t packedSize(uint8_t n) { return (uint8_t)((n * 11 + 7) / 8); }

// Pack `n` channels (each masked to 11 bits) into `out` (must hold packedSize(n) bytes).
inline void packChannels(const uint16_t* ch, uint8_t n, uint8_t* out) {
    uint32_t buf = 0;
    int bits = 0, o = 0;
    for (uint8_t i = 0; i < n; ++i) {
        buf |= (uint32_t)(ch[i] & CHANNEL_11BIT_MAX) << bits;
        bits += 11;
        while (bits >= 8) { out[o++] = (uint8_t)(buf & 0xFF); buf >>= 8; bits -= 8; }
    }
    if (bits > 0) out[o++] = (uint8_t)(buf & 0xFF);
}

// Unpack `n` channels from `in` into `ch`.
inline void unpackChannels(const uint8_t* in, uint8_t n, uint16_t* ch) {
    uint32_t buf = 0;
    int bits = 0, idx = 0;
    for (uint8_t i = 0; i < n; ++i) {
        while (bits < 11) { buf |= (uint32_t)in[idx++] << bits; bits += 8; }
        ch[i] = (uint16_t)(buf & CHANNEL_11BIT_MAX);
        buf >>= 11; bits -= 11;
    }
}

} // namespace xlrs
