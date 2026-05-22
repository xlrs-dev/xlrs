// Fhss — UID-seeded frequency-hopping sequence.
//
// Deterministic: the same seed (derived from the bind UID) produces the same hop
// sequence on TX and RX, so they hop together with no per-packet channel signalling.
// Balanced: when seqLen is a multiple of numChannels, each channel is used equally.
// Pure, header-only, host-testable. Region channel tables live alongside (see
// channels_2g4.h, added with M4); this class is table-agnostic and works on indices.
#pragma once
#include <stdint.h>

namespace xlrs {

class Fhss {
public:
    static constexpr uint8_t MAX_SEQ = 64;

    // Build a hop sequence of `seqLen` channel indices in [0, numChannels) from `seed`.
    void generate(uint32_t seed, uint8_t numChannels, uint8_t seqLen) {
        _num = numChannels ? numChannels : 1;
        _len = seqLen > MAX_SEQ ? MAX_SEQ : seqLen;
        for (uint8_t i = 0; i < _len; ++i) _seq[i] = i % _num;     // balanced multiset
        uint32_t s = seed ? seed : 0xA5A5A5A5u;                    // seeded Fisher–Yates
        for (int i = _len - 1; i > 0; --i) {
            s = s * 1664525u + 1013904223u;                        // LCG (Numerical Recipes)
            uint8_t j = (uint8_t)(s % (uint32_t)(i + 1));
            uint8_t t = _seq[i]; _seq[i] = _seq[j]; _seq[j] = t;
        }
        _idx = 0;
    }

    uint8_t count() const       { return _len; }
    uint8_t numChannels() const { return _num; }
    uint8_t at(uint8_t i) const { return _seq[i % _len]; }

    // Iterate the live hop position.
    uint8_t next()              { uint8_t c = _seq[_idx]; _idx = (uint8_t)((_idx + 1) % _len); return c; }
    void    setIndex(uint8_t i) { _idx = (uint8_t)(i % _len); }
    uint8_t index() const       { return _idx; }

private:
    uint8_t _seq[MAX_SEQ] = {};
    uint8_t _len = 0, _num = 0, _idx = 0;
};

} // namespace xlrs
