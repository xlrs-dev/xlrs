// ExpressLRS FHSS PRNG — byte-for-byte port of src/lib/FHSS/random.cpp + random.h.
//
// LCG: seed = (214013 * seed + 2531011) % 2147483648; rng() returns seed >> 16 (0..0x7FFF).
#pragma once
#include <stdint.h>

namespace xlrs {
namespace elrs {

static constexpr uint16_t RNG_MAX = 0x7FFF;

inline uint32_t& rngSeedState() {
    static uint32_t seed = 0;
    return seed;
}

// returns values between 0 and 0x7FFF
// NB rngN depends on this output range, so if we change the behaviour rngN will need updating
inline uint16_t rng() {
    const uint32_t m = 2147483648u;
    const uint32_t a = 214013u;
    const uint32_t c = 2531011u;
    uint32_t& seed = rngSeedState();
    seed = (a * seed + c) % m;
    return (uint16_t)(seed >> 16);
}

inline void rngSeed(const uint32_t newSeed) {
    rngSeedState() = newSeed;
}

// returns 0 <= x < max where max < 256
inline uint8_t rngN(const uint8_t max) {
    return (uint8_t)(rng() % max);
}

// 0..255 returned
inline uint8_t rng8Bit() {
    return (uint8_t)(rng() & 0xffu);
}

// 0..31 returned
inline uint8_t rng5Bit() {
    return (uint8_t)(rng() & 0x1Fu);
}

} // namespace elrs
} // namespace xlrs
