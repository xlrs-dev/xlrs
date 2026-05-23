#include <unity.h>
#include <cstring>

#include "crypto/AeadCipher.h"
#include "crypto/Chacha20Poly1305.h"

using namespace xlrs;

static void test_chacha20poly1305_rfc8439() {
    uint8_t key[32];
    for (int i = 0; i < 32; ++i) key[i] = (uint8_t)(0x80 + i);
    uint8_t nonce[12] = {0x07, 0, 0, 0, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47};
    uint8_t aad[12] = {0x50, 0x51, 0x52, 0x53, 0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7};
    const char* pt = "Ladies and Gentlemen of the class of '99: If I could offer you only "
                     "one tip for the future, sunscreen would be it.";
    size_t len = strlen(pt);
    uint8_t buf[128];
    memcpy(buf, pt, len); // NOLINT(bugprone-not-null-terminated-result): AEAD test buffer is binary plaintext.
    uint8_t tag[16];
    cc20p1305::seal(key, nonce, aad, 12, buf, len, tag);
    static const uint8_t expect_tag[16] =
        {0x1a, 0xe1, 0x0b, 0x59, 0x4f, 0x09, 0xe2, 0x6a,
         0x7e, 0x90, 0x2e, 0xcb, 0xd0, 0x60, 0x06, 0x91};
    TEST_ASSERT_EQUAL_MEMORY(expect_tag, tag, 16);
}

static void test_aead_cipher() {
    uint8_t key[32];
    for (int i = 0; i < 32; ++i) key[i] = (uint8_t)i;
    AeadCipher c;
    c.setKey(key);
    Nonce96 n = Nonce96::build(0x11223344u, 5, 7);

    uint8_t plain[6] = {1, 2, 3, 4, 5, 6};
    uint8_t buf[16];
    memcpy(buf, plain, 6);
    c.seal(buf, 6, n);
    bool changed = false;
    for (int i = 0; i < 6; ++i) {
        if (buf[i] != plain[i]) changed = true;
    }
    TEST_ASSERT_TRUE(changed);

    AeadCipher rx;
    rx.setKey(key);
    uint8_t good[16];
    memcpy(good, buf, 10);
    TEST_ASSERT_TRUE(rx.open(good, 6, n));
    TEST_ASSERT_EQUAL_MEMORY(plain, good, 6);

    uint8_t bad[16];
    memcpy(bad, buf, 10);
    bad[0] ^= 0x01;
    AeadCipher rx2;
    rx2.setKey(key);
    TEST_ASSERT_FALSE(rx2.open(bad, 6, n));

    uint8_t wrongkey[32];
    for (int i = 0; i < 32; ++i) wrongkey[i] = (uint8_t)(i + 1);
    AeadCipher rx3;
    rx3.setKey(wrongkey);
    uint8_t buf3[16];
    memcpy(buf3, buf, 10);
    TEST_ASSERT_FALSE(rx3.open(buf3, 6, n));
}

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_chacha20poly1305_rfc8439);
    RUN_TEST(test_aead_cipher);
    return UNITY_END();
}
