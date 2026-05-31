// Fhss — UID-seeded frequency-hopping sequence (ExpressLRS FHSSrandomiseFHSSsequenceBuild).
//
// Deterministic: the same seed produces the same hop sequence on TX and RX.
// Pure, header-only, host-testable. RF channel centres live in fhss_domain.h; this class
// operates on channel indices only.
//
// Expected behavior (ISM2G4 / 2.4 GHz):
//   FHSS_SEQUENCE_LEN = 256; primaryBandCount = (256/80)*80 = 240; syncChannel = 40.
//   Indices i where i % 80 == 0 hold the sync channel (40); block-local shuffle excludes them.
//   Self-check: test/fhss_tests.cpp (golden prefix for seed 0x12345678).
#pragma once
#include "fhss/elrs_random.h"
#include "fhss/fhss_domain.h"
#include <stdint.h>

namespace xlrs {

static constexpr uint16_t FHSS_SEQUENCE_LEN = 256u;

// ELRS FHSSrandomiseFHSSsequenceBuild — sequence length is primaryBandCount, not FHSS_SEQUENCE_LEN.
inline void elrsFhssSequenceBuild(uint32_t seed,
                                  uint32_t freqCount,
                                  uint8_t syncChannel,
                                  uint16_t seqCount,
                                  uint8_t* inSequence) {
    elrs::rngSeed(seed);

    for (uint16_t i = 0; i < seqCount; ++i) {
        if (i % freqCount == 0u) {
            inSequence[i] = syncChannel;
        } else if (i % freqCount == syncChannel) {
            inSequence[i] = 0;
        } else {
            inSequence[i] = (uint8_t)(i % freqCount);
        }
    }

    for (uint16_t i = 0; i < seqCount; ++i) {
        if (i % freqCount != 0u) {
            const uint8_t offset = (uint8_t)((i / freqCount) * freqCount);
            const uint8_t rand = (uint8_t)(elrs::rngN((uint8_t)(freqCount - 1u)) + 1u);

            const uint8_t temp = inSequence[i];
            inSequence[i] = inSequence[offset + rand];
            inSequence[offset + rand] = temp;
        }
    }
}

class Fhss {
public:
    static constexpr uint16_t MAX_SEQ = FHSS_SEQUENCE_LEN;

    // Build the ELRS primary-band hop sequence for ISM2G4 (80 channels, 240 entries).
    void generate(uint32_t seed) {
        _num = (uint8_t)kFhssDomainIsm2g4.freq_count;
        _len = (uint16_t)((FHSS_SEQUENCE_LEN / _num) * _num);
        elrsFhssSequenceBuild(seed, _num, syncChannel(), _len, _seq);
        _idx = 0;
    }

    uint16_t count() const       { return _len; }
    uint8_t  numChannels() const { return _num; }
    uint8_t  at(uint16_t i) const { return _seq[i % _len]; }

    static constexpr uint8_t syncChannel() { return kFhssIsm2g4SyncChannel; }

    bool onSyncChannel(uint16_t seqIndex) const { return at(seqIndex) == syncChannel(); }

    uint8_t next() {
        const uint8_t c = _seq[_idx];
        _idx = (uint16_t)((_idx + 1u) % _len);
        return c;
    }
    void    setIndex(uint16_t i) { _idx = (uint16_t)(i % _len); }
    uint16_t index() const       { return _idx; }

private:
    uint8_t  _seq[MAX_SEQ] = {};
    uint16_t _len = 0;
    uint8_t  _num = 0;
    uint16_t _idx = 0;
};

} // namespace xlrs
