// Uid — Link UID derivation (binding identity).  See configuration.md §2.A / §2.C.
//
// The Link UID seeds the FHSS hop sequence and the radio sync word, so a bound TX and RX
// MUST compute the SAME value — therefore it is derived from the shared binding PHRASE,
// not a per-device hardware ID. Hash is FNV-1a (fast, non-cryptographic — this is for
// addressing/isolation, not secrecy; confidentiality is the M8 cipher's job).
#pragma once
#include <stdint.h>
#include "ota/OtaPacket.h"   // LINK_UID_SIZE

namespace xlrs {

// 64-bit FNV-1a over a NUL-terminated phrase.
inline uint64_t fnv1a64(const char* s) {
    uint64_t h = 0xcbf29ce484222325ULL;                 // FNV offset basis
    for (; s && *s; ++s) { h ^= (uint8_t)*s; h *= 0x100000001b3ULL; }  // ^ then * FNV prime
    return h;
}

// Derive the 8-byte Link UID from the binding phrase (big-endian split of the hash).
inline void linkUidFromPhrase(const char* phrase, uint8_t out[LINK_UID_SIZE]) {
    uint64_t h = fnv1a64(phrase);
    for (int i = 0; i < LINK_UID_SIZE; ++i) out[i] = (uint8_t)(h >> (56 - 8 * i));
}

// FHSS PRNG seed = the lower 32 bits of the Link UID (configuration.md §2.C).
inline uint32_t fhssSeedFromUid(const uint8_t uid[LINK_UID_SIZE]) {
    return ((uint32_t)uid[4] << 24) | ((uint32_t)uid[5] << 16) |
           ((uint32_t)uid[6] <<  8) |  (uint32_t)uid[7];
}

// Radio sync word from the Link UID (configuration.md §2.A). Provides PHY-level filtering so
// a mismatched-phrase link is rejected even on a shared frequency. Uses the LOW 16 bits:
// FNV-1a's high bits barely change for inputs differing only in the trailing byte (so the
// top bits collide for e.g. "CraftA"/"CraftB"), whereas the low bits avalanche.
inline uint16_t syncWordFromUid(const uint8_t uid[LINK_UID_SIZE]) {
    return (uint16_t)(((uint16_t)uid[6] << 8) | uid[7]);
}

} // namespace xlrs
