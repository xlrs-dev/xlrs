// 2.4 GHz FHSS channel table (frequencies the hop sequence indexes into).
//
// ⚠ REPRESENTATIVE set for sim/bench (20 channels, 2402–2478 MHz / 4 MHz spacing). Kept inside
// the 2400–2483.5 MHz ISM band with margin for the modulation's occupied bandwidth (FLRC ~1.2 MHz
// at 1.3 Mbps), so no channel's signal spills past a band edge. This is NOT a certified table:
// the SHIPPING set MUST be the regulator-approved channels / spacing / dwell budget for the target
// region (FCC §15.247 / ETSI EN 300 328), validated under the RF compliance gate (architecture.md
// roadmap) before any above-bench-power transmission.
// configuration.md §2.C: the Fhss sequence (UID-seeded) produces indices into this table.
#pragma once
#include <stdint.h>

namespace xlrs {

static constexpr float kFhssChannels2g4[] = {
    2402.0f, 2406.0f, 2410.0f, 2414.0f, 2418.0f, 2422.0f, 2426.0f, 2430.0f,
    2434.0f, 2438.0f, 2442.0f, 2446.0f, 2450.0f, 2454.0f, 2458.0f, 2462.0f,
    2466.0f, 2470.0f, 2474.0f, 2478.0f,
};
static constexpr uint8_t kNumFhssChannels2g4 =
    (uint8_t)(sizeof(kFhssChannels2g4) / sizeof(kFhssChannels2g4[0]));

inline float fhssFreqForIndex(uint8_t idx) {
    return kFhssChannels2g4[idx % kNumFhssChannels2g4];
}

} // namespace xlrs
