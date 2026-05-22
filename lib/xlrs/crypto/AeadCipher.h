// AeadCipher — ICipher backed by ChaCha20-Poly1305 with a TRUNCATED tag (M8).
//
// The Nonce96 (session_salt ‖ packet_counter ‖ fhss_index) is exactly the 12-byte ChaCha20
// IETF nonce. The Poly1305 tag is truncated to TAG_LEN bytes to fit the 8-byte OTA frame — a
// deliberate airtime ↔ forgery-resistance tradeoff (4 bytes ⇒ 2^-32 per-attempt forgery; pair
// with per-session salt + rekey, see architecture.md §3.2 / §3.10).
#pragma once
#include <stdint.h>
#include <string.h>
#include "crypto/ICipher.h"
#include "crypto/Chacha20Poly1305.h"

namespace xlrs {

class AeadCipher : public ICipher {
public:
    static constexpr uint8_t TAG_LEN = 4;

    void setKey(const uint8_t key[32]) { memcpy(_key, key, 32); }

    void seal(uint8_t* buf, uint8_t len, const Nonce96& n) override {
        uint8_t tag[16];
        cc20p1305::seal(_key, n.bytes, nullptr, 0, buf, len, tag);
        memcpy(buf + len, tag, TAG_LEN);           // append the truncated tag
    }

    bool open(uint8_t* buf, uint8_t len, const Nonce96& n) override {
        return cc20p1305::open(_key, n.bytes, nullptr, 0, buf, len, buf + len, TAG_LEN);
    }

    uint8_t overhead() const override { return TAG_LEN; }

private:
    uint8_t _key[32] = {0};
};

} // namespace xlrs
