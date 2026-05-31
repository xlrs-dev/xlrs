// ExpressLRS FHSS parity self-checks (host-native). Golden vectors from ELRS algorithm.
#include <unity.h>
#include "fhss/Fhss.h"
#include "link/Uid.h"

using namespace xlrs;

static void test_elrs_rng_known_values() {
    elrs::rngSeed(1u);
    TEST_ASSERT_EQUAL_UINT16(41u, elrs::rng());
    TEST_ASSERT_EQUAL_UINT16(18467u, elrs::rng());
    elrs::rngSeed(0x12345678u);
    TEST_ASSERT_EQUAL_UINT16(13289u, elrs::rng());
}

static void test_elrs_fhss_structure() {
    Fhss fhss;
    fhss.generate(0xABCDu);

    TEST_ASSERT_EQUAL_UINT16(240u, fhss.count());
    TEST_ASSERT_EQUAL_UINT8(80u, fhss.numChannels());
    TEST_ASSERT_EQUAL_UINT8(40u, Fhss::syncChannel());

    for (uint16_t i = 0; i < fhss.count(); ++i) {
        TEST_ASSERT_TRUE(fhss.at(i) < 80u);
        if (i % 80u == 0u) {
            TEST_ASSERT_TRUE(fhss.onSyncChannel(i));
            TEST_ASSERT_EQUAL_UINT8(40u, fhss.at(i));
        }
    }
}

static void test_elrs_fhss_deterministic() {
    Fhss a, b;
    a.generate(0xDEADBEEFu);
    b.generate(0xDEADBEEFu);
    for (uint16_t i = 0; i < a.count(); ++i) {
        TEST_ASSERT_EQUAL_UINT8(a.at(i), b.at(i));
    }
}

// Captured from ELRS FHSSrandomiseFHSSsequenceBuild(0x12345678, 80, 40, seq) — first 16 entries.
static void test_elrs_fhss_golden_prefix() {
    static const uint8_t kExpected[] = {
        40, 19, 55, 78, 11, 12, 22,  6,
         4, 24, 32, 64, 39, 47, 53, 20,
    };

    Fhss fhss;
    fhss.generate(0x12345678u);
    for (unsigned i = 0; i < sizeof(kExpected); ++i) {
        TEST_ASSERT_EQUAL_UINT8(kExpected[i], fhss.at((uint16_t)i));
    }
}

static void test_elrs_fhss_domain_freq() {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2440.399887f, fhssInitialSyncFreqMHz());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2400.399933f, fhssFreqMHzForChannelIndex(0));
}

static void test_elrs_fhss_uid_seed_shared() {
    uint8_t uidA[LINK_UID_SIZE], uidB[LINK_UID_SIZE], uidC[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uidA);
    linkUidFromPhrase("Kikobot-02", uidB);
    linkUidFromPhrase("OtherCraft", uidC);

    Fhss tx, rx, other;
    tx.generate(fhssSeedFromUid(uidA));
    rx.generate(fhssSeedFromUid(uidB));
    other.generate(fhssSeedFromUid(uidC));

    for (uint16_t i = 0; i < tx.count(); ++i) {
        TEST_ASSERT_EQUAL_UINT8(tx.at(i), rx.at(i));
    }

    bool differs = false;
    for (uint16_t i = 0; i < tx.count(); ++i) {
        if (other.at(i) != tx.at(i)) {
            differs = true;
            break;
        }
    }
    TEST_ASSERT_TRUE(differs);
}

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_elrs_rng_known_values);
    RUN_TEST(test_elrs_fhss_structure);
    RUN_TEST(test_elrs_fhss_deterministic);
    RUN_TEST(test_elrs_fhss_golden_prefix);
    RUN_TEST(test_elrs_fhss_domain_freq);
    RUN_TEST(test_elrs_fhss_uid_seed_shared);
    return UNITY_END();
}
