// 2.4 GHz FHSS channel table (frequencies the hop sequence indexes into).
//
// ⚠ REPRESENTATIVE set for sim/bench (20 channels, 2402–2478 MHz / 4 MHz spacing). Kept inside
// the 2400–2483.5 MHz ISM band with margin for the modulation's occupied bandwidth (FLRC ~1.2 MHz
// at 1.3 Mbps), so no channel's signal spills past a band edge. This is NOT a certified table:
// the SHIPPING set MUST be the regulator-approved channels / spacing / dwell budget for the target
// region (FCC §15.247 / ETSI EN 300 328), validated under the RF compliance gate (docs/developer/architecture.md
// roadmap) before any above-bench-power transmission.
// docs/developer/configuration.md §2.C: the Fhss sequence (UID-seeded) produces indices into this table.
#pragma once
#include <stdint.h>

namespace xlrs {

enum class FhssRegion : uint8_t {
    US_FCC,   // 80 channels (2400..2479 MHz)
    EU_CE     // 80 channels with LBT and power limits
};

static constexpr float kFhssChannels2g4[] = {
    2400.0f, 2401.0f, 2402.0f, 2403.0f, 2404.0f, 2405.0f, 2406.0f, 2407.0f,
    2408.0f, 2409.0f, 2410.0f, 2411.0f, 2412.0f, 2413.0f, 2414.0f, 2415.0f,
    2416.0f, 2417.0f, 2418.0f, 2419.0f, 2420.0f, 2421.0f, 2422.0f, 2423.0f,
    2424.0f, 2425.0f, 2426.0f, 2427.0f, 2428.0f, 2429.0f, 2430.0f, 2431.0f,
    2432.0f, 2433.0f, 2434.0f, 2435.0f, 2436.0f, 2437.0f, 2438.0f, 2439.0f,
    2440.0f, 2441.0f, 2442.0f, 2443.0f, 2444.0f, 2445.0f, 2446.0f, 2447.0f,
    2448.0f, 2449.0f, 2450.0f, 2451.0f, 2452.0f, 2453.0f, 2454.0f, 2455.0f,
    2456.0f, 2457.0f, 2458.0f, 2459.0f, 2460.0f, 2461.0f, 2462.0f, 2463.0f,
    2464.0f, 2465.0f, 2466.0f, 2467.0f, 2468.0f, 2469.0f, 2470.0f, 2471.0f,
    2472.0f, 2473.0f, 2474.0f, 2475.0f, 2476.0f, 2477.0f, 2478.0f, 2479.0f
};

static constexpr uint8_t kNumFhssChannels2g4 =
    (uint8_t)(sizeof(kFhssChannels2g4) / sizeof(kFhssChannels2g4[0]));

inline float fhssFreqForIndex(uint8_t idx, FhssRegion region = FhssRegion::US_FCC) {
    (void)region; // Reserved for dynamic regulatory checks, LBT gating, or dwell budgets
    return kFhssChannels2g4[idx % kNumFhssChannels2g4];
}

} // namespace xlrs
