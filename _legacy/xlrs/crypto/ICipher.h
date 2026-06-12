// ICipher — pluggable security layer over the OTA payload.
//
// Defaults to NullCipher (no confidentiality), so the link ships without crypto and the
// crypto decision is deferred (docs/developer/architecture.md §3.2): slot in real authenticated encryption
// later by swapping the cipher — link and PHY unchanged. Candidates: ChaCha20-Poly1305
// (preferred — no hardware AES on RP2040/RP2350), AES-CCM, or a MAC-only construction
// (integrity without confidentiality, often the right call for an RC link). NOT AES-CTR
// alone — it provides no authenticity.
//
// seal()/open() operate in place on the payload. The 96-bit nonce is built per §3.10 and
// must never repeat under one key.
#pragma once
#include <stdint.h>
#include <stddef.h>

namespace xlrs {

// 96-bit nonce, byte-concatenated per docs/developer/architecture.md §3.10 (no bit-overlap aliasing):
//   bytes[0..3]   session_salt   (4 bytes)
//   bytes[4..9]   packet_counter (6 bytes / 48-bit — cannot wrap within a session)
//   bytes[10..11] fhss_index     (2 bytes)
struct Nonce96 {
    uint8_t bytes[12];

    static Nonce96 build(uint32_t sessionSalt, uint64_t packetCounter, uint16_t fhssIndex) {
        Nonce96 n{};
        n.bytes[0]  = (uint8_t)(sessionSalt   >> 24);
        n.bytes[1]  = (uint8_t)(sessionSalt   >> 16);
        n.bytes[2]  = (uint8_t)(sessionSalt   >>  8);
        n.bytes[3]  = (uint8_t)(sessionSalt        );
        n.bytes[4]  = (uint8_t)(packetCounter >> 40);
        n.bytes[5]  = (uint8_t)(packetCounter >> 32);
        n.bytes[6]  = (uint8_t)(packetCounter >> 24);
        n.bytes[7]  = (uint8_t)(packetCounter >> 16);
        n.bytes[8]  = (uint8_t)(packetCounter >>  8);
        n.bytes[9]  = (uint8_t)(packetCounter      );
        n.bytes[10] = (uint8_t)(fhssIndex     >>  8);
        n.bytes[11] = (uint8_t)(fhssIndex          );
        return n;
    }
};

class ICipher {
public:
    virtual ~ICipher() = default;

    // Encrypt/authenticate `buf` (len bytes) in place. May be a no-op.
    virtual void seal(uint8_t* buf, uint8_t len, const Nonce96& nonce) = 0;

    // Decrypt/verify in place. Returns false on auth failure — caller DROPS the packet and
    // must NOT advance/desync the nonce counter on failure (see §3.10).
    virtual bool open(uint8_t* buf, uint8_t len, const Nonce96& nonce) = 0;

    // Extra bytes this cipher appends (e.g. a truncated auth tag). 0 for none.
    virtual uint8_t overhead() const = 0;
};

// Default: plaintext passthrough. Integrity left to the PHY hardware CRC.
class NullCipher : public ICipher {
public:
    void seal(uint8_t*, uint8_t, const Nonce96&) override {}
    bool open(uint8_t*, uint8_t, const Nonce96&) override { return true; }
    uint8_t overhead() const override { return 0; }
};

} // namespace xlrs
