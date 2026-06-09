// ExpressLRS-compatible FHSS domain definitions (SX1280 / 2.4 GHz).
//
// Replaces the legacy linear-MHz kFhssChannels2g4[] table in channels_2g4.h.
// Channel centres match ExpressLRS FHSS.cpp (RADIO_SX128X ISM2G4 / CE_LBT):
//   freq_spread = (freq_stop - freq_start) * FREQ_SPREAD_SCALE / (freq_count - 1)
//   reg(ch)     = freq_start + (freq_spread * ch / FREQ_SPREAD_SCALE)
// where freq_start/stop are SX1280 register values (FREQ_HZ_TO_REG_VAL).
#pragma once
#include <stdint.h>

namespace xlrs {

// SX1280: FREQ_STEP = 52 MHz XTAL / 2^18 (ExpressLRS SX1280_Regs.h).
static constexpr uint32_t kSx1280FreqDiv     = 1u << 18;
static constexpr uint32_t kSx1280XtalHz      = 52000000u;
static constexpr uint32_t kFhssFreqSpreadScale = 256u;

struct FhssDomain {
    const char* name;
    uint32_t    freq_start;     // SX1280 RF frequency register value
    uint32_t    freq_stop;      // SX1280 RF frequency register value
    uint32_t    freq_count;
    uint32_t    freq_center_hz; // nominal band centre (Hz); ELRS metadata only
};

inline constexpr uint32_t fhssHzToRegVal(uint32_t freq_hz) {
    return (uint32_t)(((uint64_t)freq_hz * kSx1280FreqDiv) / kSx1280XtalHz);
}

// ELRS ISM2G4 domain: 2400.4–2479.4 MHz, 80 channels (same map for CE_LBT).
static constexpr FhssDomain kFhssDomainIsm2g4 = {
    "ISM2G4",
    fhssHzToRegVal(2400400000u),
    fhssHzToRegVal(2479400000u),
    80u,
    2440000000u,
};

static constexpr uint32_t kFhssIsm2g4FreqSpread =
    (kFhssDomainIsm2g4.freq_stop - kFhssDomainIsm2g4.freq_start) * kFhssFreqSpreadScale
    / (kFhssDomainIsm2g4.freq_count - 1u);

static constexpr uint8_t kFhssIsm2g4SyncChannel =
    (uint8_t)(kFhssDomainIsm2g4.freq_count / 2u); // ELRS: sync_channel = freq_count / 2

inline constexpr uint32_t fhssRegValForChannelIndex(uint8_t channelIndex,
                                                    const FhssDomain& domain = kFhssDomainIsm2g4) {
    const uint32_t spread =
        (domain.freq_stop - domain.freq_start) * kFhssFreqSpreadScale / (domain.freq_count - 1u);
    const uint8_t ch = (uint8_t)(channelIndex % domain.freq_count);
    return domain.freq_start + (spread * ch / kFhssFreqSpreadScale);
}

inline float fhssRegValToMHz(uint32_t reg) {
    return (float)reg * (float)kSx1280XtalHz / (float)kSx1280FreqDiv / 1000000.0f;
}

// Returns channel centre frequency in MHz for PHY (float MHz API).
inline float fhssFreqMHzForChannelIndex(uint8_t channelIndex,
                                        const FhssDomain& domain = kFhssDomainIsm2g4) {
    return fhssRegValToMHz(fhssRegValForChannelIndex(channelIndex, domain));
}

// Initial / sync channel frequency (ELRS FHSSgetInitialFreq, channel 40).
inline float fhssInitialSyncFreqMHz() {
    return fhssFreqMHzForChannelIndex(kFhssIsm2g4SyncChannel);
}

} // namespace xlrs
