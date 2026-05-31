// 2.4 GHz FHSS channel mapping (indices into the hop sequence).
//
// Uses the ExpressLRS ISM2G4 spread formula (see fhss_domain.h) instead of the legacy
// kFhssChannels2g4[] linear 1 MHz table. The UID-seeded Fhss sequence produces indices
// into [0, kNumFhssChannels2g4); fhssFreqMHzForChannelIndex() maps each index to MHz
// for the PHY layer.
#pragma once
#include "fhss/fhss_domain.h"
#include <stdint.h>

namespace xlrs {

enum class FhssRegion : uint8_t {
    ISM2G4,  // ELRS default (2400.4–2479.4 MHz, 80 channels)
    US_FCC = ISM2G4,
    EU_CE    // ELRS CE_LBT — same hop map; LBT/dwell policy differs (future)
};

static constexpr uint8_t kNumFhssChannels2g4 = (uint8_t)kFhssDomainIsm2g4.freq_count;

inline float fhssFreqForIndex(uint8_t idx, FhssRegion region = FhssRegion::ISM2G4) {
    (void)region; // Reserved for regulatory/LBT gating once regions diverge
    return fhssFreqMHzForChannelIndex(idx);
}

} // namespace xlrs
