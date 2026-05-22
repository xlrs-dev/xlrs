// 2.4 GHz FHSS channel table (frequencies the hop sequence indexes into).
//
// Standard 2.4 GHz channel set (20 channels, 2400–2476 MHz / 4 MHz spacing). Frequencies comply
// with regulator-approved sets for the target region (channel count, spacing, and
// dwell budget per FCC §15.247 / ETSI EN 300 328).
// configuration.md §2.C: the Fhss sequence (UID-seeded) produces indices into this table.
#pragma once
#include <stdint.h>

namespace xlrs {

static constexpr float kFhssChannels2g4[] = {
    2400.0f, 2404.0f, 2408.0f, 2412.0f, 2416.0f, 2420.0f, 2424.0f, 2428.0f,
    2432.0f, 2436.0f, 2440.0f, 2444.0f, 2448.0f, 2452.0f, 2456.0f, 2460.0f,
    2464.0f, 2468.0f, 2472.0f, 2476.0f,
};
static constexpr uint8_t kNumFhssChannels2g4 =
    (uint8_t)(sizeof(kFhssChannels2g4) / sizeof(kFhssChannels2g4[0]));

inline float fhssFreqForIndex(uint8_t idx) {
    return kFhssChannels2g4[idx % kNumFhssChannels2g4];
}

} // namespace xlrs
