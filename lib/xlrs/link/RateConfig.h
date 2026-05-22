// RateConfig — packet rates as data.
//
// Each rate is one table row. Adding/removing a rate (or retuning latency vs. range) is a
// table edit; the link/PHY read these fields and never hard-code a rate. This is how the
// link scales across the latency↔range tradeoff that the legacy single-FLRC link lacked.
#pragma once
#include <stdint.h>
#include "phy/IRadioPhy.h"   // Modulation
#include "ota/OtaPacket.h"   // OTA8_LEN

namespace xlrs {

struct RateConfig {
    const char* name;
    uint16_t    intervalUs;       // packet period (TX timer)
    uint8_t     fhssHopInterval;  // hop every N packets
    uint8_t     tlmRatioDenom;    // telemetry slot ratio 1:N
    Modulation  modulation;
    float       bwKHz;            // LoRa bandwidth
    uint8_t     sf;               // LoRa spreading factor
    uint8_t     cr;               // coding rate
    uint16_t    flrcBitrateKbps;  // FLRC bitrate (0 for LoRa)
    uint8_t     payloadLen;
    uint16_t    airtime8Us;       // pre-calculated airtime for an 8-byte frame
    uint16_t    airtime16Us;      // pre-calculated airtime for a 16-byte frame
};

// Representative starting set — latency-first (FLRC) down to range-first (LoRa).
// Tune during M1/M6 against measured sensitivity and airtime.
inline constexpr RateConfig kRates[] = {
    // name     interval hop tlm  modulation        bw     sf cr  flrc  len         a8Us a16Us
    { "F1000",   1000,    4,  64,  Modulation::Flrc, 0.0f,  0, 2,  1300, OTA16_LEN,  80,  160 },
    { "F500",    2000,    4,  32,  Modulation::Flrc, 0.0f,  0, 2,  1300, OTA16_LEN,  80,  160 },
    { "D250",    4000,    4,  16,  Modulation::Flrc, 0.0f,  0, 2,   650, OTA16_LEN, 140,  280 },
    { "L150",    6666,    4,   8,  Modulation::Lora, 812.5f,6, 8,     0, OTA16_LEN, 250,  480 },
    { "L50",    20000,    4,   4,  Modulation::Lora, 812.5f,8, 8,     0, OTA16_LEN, 780, 1420 },
};
inline constexpr uint8_t kNumRates = sizeof(kRates) / sizeof(kRates[0]);

inline PhyConfig makePhyConfig(const RateConfig& rate, float freqMHz, int8_t powerDbm, uint16_t syncWord) {
    PhyConfig cfg{};
    cfg.freqMHz = freqMHz;
    cfg.modulation = rate.modulation;
    cfg.bwKHz = rate.bwKHz;
    cfg.sf = rate.sf;
    cfg.cr = rate.cr;
    cfg.flrcBitrateKbps = rate.flrcBitrateKbps;
    cfg.powerDbm = powerDbm;
    cfg.syncWord = syncWord;
    cfg.payloadLen = rate.payloadLen;
    return cfg;
}

} // namespace xlrs

