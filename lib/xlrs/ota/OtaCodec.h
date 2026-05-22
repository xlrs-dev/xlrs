// OtaCodec — minimal OTA frame encode/decode (M3).
//
// RC frame: [header][packed 11-bit channels]. Header carries version + OtaType (§OtaPacket).
// The cipher seal/open wraps the payload at M8; M3 uses NullCipher (passthrough). Pure,
// header-only, host-testable. Sync/telemetry frame types are added at M4/M5.
#pragma once
#include <stdint.h>
#include "ota/OtaPacket.h"
#include "ota/ChannelPack.h"

namespace xlrs {

// Encode `n` channels into `out`; returns frame length (1 header + packed channels).
inline uint8_t otaEncodeRc(const uint16_t* ch, uint8_t n, uint8_t* out) {
    out[0] = otaMakeHeader(OtaType::Rc);
    packChannels(ch, n, out + 1);
    return (uint8_t)(1 + packedSize(n));
}

// Decode an RC frame of `n` channels. Returns false on wrong version/type or short length.
inline bool otaDecodeRc(const uint8_t* in, uint8_t len, uint16_t* ch, uint8_t n) {
    if (len < (uint8_t)(1 + packedSize(n)))   return false;
    if (otaVersion(in[0]) != OTA_VERSION)     return false;
    if (otaType(in[0]) != OtaType::Rc)        return false;
    unpackChannels(in + 1, n, ch);
    return true;
}

// Sync beacon: [header][fhssIndex][rateIndex][nextRateIndex][switchTick:4][tlmRatioDenom][uidCrc] = 10 bytes.
inline uint8_t otaEncodeSync(const SyncPayload& s, uint8_t* out) {
    out[0] = otaMakeHeader(OtaType::Sync);
    out[1] = s.fhssIndex;
    out[2] = s.rateIndex;
    out[3] = s.nextRateIndex;
    out[4] = (uint8_t)((s.switchTick >> 24) & 0xFF);
    out[5] = (uint8_t)((s.switchTick >> 16) & 0xFF);
    out[6] = (uint8_t)((s.switchTick >> 8) & 0xFF);
    out[7] = (uint8_t)(s.switchTick & 0xFF);
    out[8] = s.tlmRatioDenom;
    out[9] = s.uidCrc;
    return 10;
}

inline bool otaDecodeSync(const uint8_t* in, uint8_t len, SyncPayload& s) {
    if (len < 10)                         return false;
    if (otaVersion(in[0]) != OTA_VERSION) return false;
    if (otaType(in[0]) != OtaType::Sync)  return false;
    s.fhssIndex     = in[1];
    s.rateIndex     = in[2];
    s.nextRateIndex = in[3];
    s.switchTick    = ((uint32_t)in[4] << 24) |
                      ((uint32_t)in[5] << 16) |
                      ((uint32_t)in[6] << 8)  |
                      in[7];
    s.tlmRatioDenom = in[8];
    s.uidCrc        = in[9];
    return true;
}

// Telemetry downlink (RX→TX): [header][uplinkLQ][uplinkRSSI(-dBm magnitude)][uplinkSNR] = 4 bytes.
// Carries the RX's measurement of the UPLINK so the TX/handset can show "how well my craft
// hears me". RSSI is stored as the positive magnitude of the negative dBm.
inline uint8_t otaEncodeTlmDown(uint8_t upLq, int16_t upRssiDbm, int8_t upSnr, uint8_t* out) {
    out[0] = otaMakeHeader(OtaType::TlmDown);
    out[1] = upLq;
    out[2] = (uint8_t)(upRssiDbm <= 0 ? -upRssiDbm : 0);
    out[3] = (uint8_t)upSnr;
    return 4;
}

inline bool otaDecodeTlmDown(const uint8_t* in, uint8_t len,
                             uint8_t* upLq, int16_t* upRssiDbm, int8_t* upSnr) {
    if (len < 4)                            return false;
    if (otaVersion(in[0]) != OTA_VERSION)   return false;
    if (otaType(in[0]) != OtaType::TlmDown) return false;
    *upLq      = in[1];
    *upRssiDbm = -(int16_t)in[2];
    *upSnr     = (int8_t)in[3];
    return true;
}

} // namespace xlrs
