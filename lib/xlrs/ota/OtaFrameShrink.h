#pragma once
#include <stdint.h>
#include "ota/OtaPacket.h"

namespace xlrs {

// Pack 5 primary channels scaled to 10 bits (0..1023) and a 6-bit sequence number
// into a compact 8-byte OTA frame (1 header byte + 7 packed bytes).
inline void packChannels8Byte(const uint16_t* ch, uint8_t seq, uint8_t* out) {
    // Write header byte: OtaType::Rc with compressed flag/indicator
    out[0] = otaMakeHeader(OtaType::Rc, 0x01); // 0x01 flag indicates compressed 8-byte layout

    // Scale 11-bit channels to 10-bit: ch10 = ch11 >> 1
    uint16_t c0 = (ch[0] >> 1) & 0x3FF;
    uint16_t c1 = (ch[1] >> 1) & 0x3FF;
    uint16_t c2 = (ch[2] >> 1) & 0x3FF;
    uint16_t c3 = (ch[3] >> 1) & 0x3FF;
    uint16_t c4 = (ch[4] >> 1) & 0x3FF;

    // Bit pack into out[1..7]
    out[1] = c0 & 0xFF;                                       // 8 bits of c0
    out[2] = ((c0 >> 8) & 0x03) | ((c1 & 0x3F) << 2);         // 2 bits of c0, 6 bits of c1
    out[3] = ((c1 >> 6) & 0x0F) | ((c2 & 0x0F) << 4);         // 4 bits of c1, 4 bits of c2
    out[4] = ((c2 >> 4) & 0x3F) | ((c3 & 0x03) << 6);         // 6 bits of c2, 2 bits of c3
    out[5] = (c3 >> 2) & 0xFF;                                // 8 bits of c3
    out[6] = c4 & 0xFF;                                       // 8 bits of c4
    out[7] = ((c4 >> 8) & 0x03) | ((seq & 0x3F) << 2);        // 2 bits of c4, 6 bits of seq
}

// Unpack 5 primary channels and extract the 6-bit sequence from the 8-byte frame,
// scaling back to 11-bit resolution (ch11 = ch10 << 1).
inline bool unpackChannels8Byte(const uint8_t* in, uint16_t* ch, uint8_t& outSeq) {
    if (otaVersion(in[0]) != OTA_VERSION) return false;
    if (otaType(in[0]) != OtaType::Rc) return false;
    if ((otaFlags(in[0]) & 0x01) == 0) return false; // Verify compression flag is set

    // Retrieve 10-bit scaled values
    uint16_t c0 = in[1] | ((in[2] & 0x03) << 8);
    uint16_t c1 = ((in[2] >> 2) & 0x3F) | ((in[3] & 0x0F) << 6);
    uint16_t c2 = ((in[3] >> 4) & 0x0F) | ((in[4] & 0x3F) << 4);
    uint16_t c3 = ((in[4] >> 6) & 0x03) | (in[5] << 2);
    uint16_t c4 = in[6] | ((in[7] & 0x03) << 8);

    outSeq = (in[7] >> 2) & 0x3F;

    // Scale up to 11-bit resolution
    ch[0] = c0 << 1;
    ch[1] = c1 << 1;
    ch[2] = c2 << 1;
    ch[3] = c3 << 1;
    ch[4] = c4 << 1;

    // Set fallback defaults for unused channels 5..7
    ch[5] = 1024;
    ch[6] = 1024;
    ch[7] = 1024;

    return true;
}

} // namespace xlrs
