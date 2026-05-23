#include <unity.h>

#include "crypto/AeadCipher.h"
#include "link/Uid.h"
#include "SimEnvironment.h"

using namespace xlrs;
using namespace xlrs_test;

static void test_link_connect_flow_failsafe() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);

    uint16_t ch[4] = {100, 1000, 2000, 1024};
    env.tx.setChannels(ch, 4);

    for (uint32_t t = 1; t <= 20; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);
    TEST_ASSERT_TRUE(env.rx.outputActive());
    uint16_t out[4];
    TEST_ASSERT_EQUAL_UINT8(4, env.rx.getChannels(out, 4));
    for (int i = 0; i < 4; ++i) TEST_ASSERT_EQUAL_UINT16(ch[i], out[i]);
    TEST_ASSERT_EQUAL_UINT8(100, env.rx.stats().lqUp);

    for (uint32_t t = 21; t <= 40; ++t) simTick(env, t, false);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Failsafe);
    TEST_ASSERT_FALSE(env.rx.outputActive());
    TEST_ASSERT_TRUE(env.rx.stats().lqUp < 100);
}

static void test_link_mismatched_phrase_no_connect() {
    uint8_t uidA[LINK_UID_SIZE], uidB[LINK_UID_SIZE];
    linkUidFromPhrase("CraftA", uidA);
    linkUidFromPhrase("CraftB", uidB);

    SimEnvironment env;
    PhyConfig cfg{};
    cfg.freqMHz = 2420.0f;
    cfg.payloadLen = 8;
    env.txPhy.init(cfg);
    env.rxPhy.init(cfg);
    MockPhy::connect(env.txPhy, env.rxPhy);

    env.tx.begin(Role::Tx, uidA, 2);
    env.rx.begin(Role::Rx, uidB, 2);
    env.txSched.begin(&env.txPhy, &env.tx, 2);
    env.rxSched.begin(&env.rxPhy, &env.rx, 2);

    uint16_t ch[4] = {1, 2, 3, 4};
    env.tx.setChannels(ch, 4);
    for (uint32_t t = 1; t <= 30; ++t) simTick(env, t);

    TEST_ASSERT_TRUE(env.rx.state() != LinkState::Connected);
    TEST_ASSERT_FALSE(env.rx.outputActive());
}

static void test_link_bind_scan_accepts_offered_uid() {
    uint8_t txUid[LINK_UID_SIZE], rxUid[LINK_UID_SIZE];
    linkUidFromPhrase("CraftA", txUid);
    linkUidFromPhrase("CraftB", rxUid);

    SimEnvironment env;
    PhyConfig cfg{};
    cfg.freqMHz = 2420.0f;
    cfg.payloadLen = OTA16_LEN;
    env.txPhy.init(cfg);
    env.rxPhy.init(cfg);
    MockPhy::connect(env.txPhy, env.rxPhy);

    env.tx.begin(Role::Tx, txUid, 2);
    env.rx.begin(Role::Rx, rxUid, 2);
    env.txSched.begin(&env.txPhy, &env.tx, 2);
    env.rxSched.begin(&env.rxPhy, &env.rx, 2);

    env.tx.startBindTransmit(txUid);
    env.rx.startBindScan();
    env.txPhy.setSyncWord(env.tx.syncWord());
    env.rxPhy.setSyncWord(env.rx.syncWord());

    for (uint32_t t = 1; t <= 80; ++t) simTick(env, t);

    uint8_t receivedUid[LINK_UID_SIZE] = {};
    TEST_ASSERT_TRUE(env.rx.takeReceivedBindUid(receivedUid));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(txUid, receivedUid, LINK_UID_SIZE);
}

static void test_link_fhss_acquire_and_recover() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);

    uint16_t ch[4] = {200, 1500, 2047, 0};
    env.tx.setChannels(ch, 4);

    for (uint32_t t = 1; t <= 300; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);
    TEST_ASSERT_TRUE(env.rx.outputActive());
    uint16_t out[4];
    env.rx.getChannels(out, 4);
    for (int i = 0; i < 4; ++i) TEST_ASSERT_EQUAL_UINT16(ch[i], out[i]);
    TEST_ASSERT_TRUE(env.rx.stats().lqUp >= 90);

    for (uint32_t t = 301; t <= 340; ++t) simTick(env, t, false);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Failsafe);
    TEST_ASSERT_FALSE(env.rx.outputActive());

    for (uint32_t t = 341; t <= 650; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);
    TEST_ASSERT_TRUE(env.rx.outputActive());
}

static void test_link_telemetry_downlink() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);
    env.txPhy.setRssi(-72);

    uint16_t ch[4] = {300, 1500, 2047, 0};
    env.tx.setChannels(ch, 4);
    for (uint32_t t = 1; t <= 600; ++t) simTick(env, t);

    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);
    TEST_ASSERT_TRUE(env.rx.outputActive());
    uint16_t out[4];
    env.rx.getChannels(out, 4);
    for (int i = 0; i < 4; ++i) TEST_ASSERT_EQUAL_UINT16(ch[i], out[i]);

    TEST_ASSERT_TRUE(env.tx.stats().lqDown > 0);
    TEST_ASSERT_EQUAL_INT16(-72, env.tx.stats().rssiDbm);
    TEST_ASSERT_TRUE(env.tx.stats().lqUp >= 90);
}

static void test_link_rate_switch_and_power() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);
    env.txPhy.setRssi(-55);

    uint16_t ch[4] = {300, 1500, 2047, 0};
    env.tx.setChannels(ch, 4);

    for (uint32_t t = 1; t <= 800; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);
    TEST_ASSERT_EQUAL_UINT8(2, env.rx.stats().rateIndex);
    TEST_ASSERT_TRUE(env.txPhy.outputPowerDbm() < 10);

    env.tx.requestRate(4);
    for (uint32_t t = 801; t <= 1600; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);
    TEST_ASSERT_EQUAL_UINT8(4, env.rx.stats().rateIndex);
}

static void test_link_encrypted() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);

    uint8_t key[32];
    for (int i = 0; i < 32; ++i) key[i] = (uint8_t)(0x42 ^ i);
    AeadCipher txC, rxC;
    txC.setKey(key);
    rxC.setKey(key);

    env.tx.setCipher(&txC);
    env.rx.setCipher(&rxC);
    uint16_t ch[4] = {222, 1444, 1999, 55};
    env.tx.setChannels(ch, 4);

    for (uint32_t t = 1; t <= 300; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);
    uint16_t out[4];
    env.rx.getChannels(out, 4);
    for (int i = 0; i < 4; ++i) TEST_ASSERT_EQUAL_UINT16(ch[i], out[i]);
    TEST_ASSERT_TRUE(env.rx.stats().lqUp >= 90);
}

static void test_link_wrong_key() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);

    uint8_t k1[32], k2[32];
    for (int i = 0; i < 32; ++i) {
        k1[i] = (uint8_t)i;
        k2[i] = (uint8_t)(i + 1);
    }
    AeadCipher txC, rxC;
    txC.setKey(k1);
    rxC.setKey(k2);

    env.tx.setCipher(&txC);
    env.rx.setCipher(&rxC);
    uint16_t ch[4] = {200, 200, 200, 200};
    env.tx.setChannels(ch, 4);

    for (uint32_t t = 1; t <= 300; ++t) simTick(env, t);
    uint16_t out[4];
    env.rx.getChannels(out, 4);
    TEST_ASSERT_EQUAL_UINT16(1024, out[0]);
    TEST_ASSERT_TRUE(env.rx.stats().lqUp < 10);
}

static void test_link_long_soak() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);

    uint16_t ch[4] = {250, 1250, 1750, 800};
    env.tx.setChannels(ch, 4);

    for (uint32_t t = 1; t <= 300; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);

    for (uint32_t t = 301; t <= 8000; ++t) simTick(env, t);

    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);
    TEST_ASSERT_TRUE(env.rx.outputActive());
    TEST_ASSERT_TRUE(env.rx.stats().lqUp >= 95);
    TEST_ASSERT_EQUAL_UINT16(0, env.rx.stats().rxQueueDrops);
    uint16_t out[4];
    env.rx.getChannels(out, 4);
    for (int i = 0; i < 4; ++i) TEST_ASSERT_EQUAL_UINT16(ch[i], out[i]);
}

static void test_link_corrupt_rc_rejected() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);

    uint16_t ch[4] = {400, 1600, 2000, 100};
    env.tx.setChannels(ch, 4);
    for (uint32_t t = 1; t <= 200; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);
    TEST_ASSERT_TRUE(env.rx.stats().lqUp >= 90);

    uint16_t before[4];
    env.rx.getChannels(before, 4);
    for (int i = 0; i < 4; ++i) TEST_ASSERT_EQUAL_UINT16(ch[i], before[i]);

    env.rxPhy.corruptNextDeliveries(1000000);
    for (uint32_t t = 201; t <= 260; ++t) simTick(env, t);

    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Failsafe);
    TEST_ASSERT_FALSE(env.rx.outputActive());
    uint16_t during[4];
    env.rx.getChannels(during, 4);
    for (int i = 0; i < 4; ++i) TEST_ASSERT_EQUAL_UINT16(ch[i], during[i]);

    env.rxPhy.corruptNextDeliveries(0);
    for (uint32_t t = 261; t <= 700; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);
    TEST_ASSERT_TRUE(env.rx.stats().lqUp >= 90);
}

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_link_connect_flow_failsafe);
    RUN_TEST(test_link_mismatched_phrase_no_connect);
    RUN_TEST(test_link_bind_scan_accepts_offered_uid);
    RUN_TEST(test_link_fhss_acquire_and_recover);
    RUN_TEST(test_link_telemetry_downlink);
    RUN_TEST(test_link_rate_switch_and_power);
    RUN_TEST(test_link_encrypted);
    RUN_TEST(test_link_wrong_key);
    RUN_TEST(test_link_long_soak);
    RUN_TEST(test_link_corrupt_rc_rejected);
    return UNITY_END();
}
