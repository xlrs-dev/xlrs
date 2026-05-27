#include <unity.h>

#include "crypto/AeadCipher.h"
#include "link/Uid.h"
#include "SimEnvironment.h"

using namespace xlrs;
using namespace xlrs_test;

static void simTickWithPhaseOffset(SimEnvironment& env, uint32_t txTick, uint32_t rxTick,
                                   bool txOn = true) {
    g_txSched = &env.txSched;
    g_rxSched = &env.rxSched;

    env.txPhy.setOnRxDone(onTxRxDone);
    env.txPhy.setOnTxDone(onTxTxDone);
    env.rxPhy.setOnRxDone(onRxRxDone);
    env.rxPhy.setOnTxDone(onRxTxDone);

    Slot slot = env.tx.slotForTick(txTick);
    if (slot == Slot::Telemetry) {
        if (txOn) env.txSched.onTick(txTick);
        env.rxSched.onTick(rxTick);
    } else {
        env.rxSched.onTick(rxTick);
        if (txOn) env.txSched.onTick(txTick);
    }

    if (txOn) env.tx.service(txTick);
    env.rx.service(rxTick);
}

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

    for (uint32_t t = 21; t <= 120; ++t) simTick(env, t, false);
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

    for (uint32_t t = 1; t <= 80; ++t) simTick(env, t);

    uint8_t receivedUid[LINK_UID_SIZE] = {};
    TEST_ASSERT_TRUE(env.rx.takeReceivedBindUid(receivedUid));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(txUid, receivedUid, LINK_UID_SIZE);
}

static void test_link_bind_scan_accepts_offered_uid_with_tick_phase_offset() {
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

    const uint32_t txPhaseOffset = 137;
    for (uint32_t t = 1; t <= 80; ++t) {
        simTickWithPhaseOffset(env, t + txPhaseOffset, t);
    }

    uint8_t receivedUid[LINK_UID_SIZE] = {};
    TEST_ASSERT_TRUE(env.rx.takeReceivedBindUid(receivedUid));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(txUid, receivedUid, LINK_UID_SIZE);
}

static void test_link_bind_scan_rejects_padded_bind_frame() {
    uint8_t txUid[LINK_UID_SIZE], rxUid[LINK_UID_SIZE];
    linkUidFromPhrase("CraftA", txUid);
    linkUidFromPhrase("CraftB", rxUid);

    Link tx;
    Link rx;
    tx.begin(Role::Tx, txUid, 2);
    rx.begin(Role::Rx, rxUid, 2);
    tx.startBindTransmit(txUid);
    rx.startBindScan();

    uint8_t frame[OTA16_LEN + 1] = {};
    uint8_t frameLen = 0;
    const uint32_t tick = 1;
    TEST_ASSERT_TRUE(tx.getTxPayload(tick, tx.txPos(tick), frame, frameLen));
    TEST_ASSERT_TRUE(frameLen < sizeof(frame));
    frame[frameLen] = 0xA5;

    TEST_ASSERT_FALSE(rx.processRxPayload(tick, rx.rxPos(), frame, (uint8_t)(frameLen + 1), -50, 0));
    uint8_t receivedUid[LINK_UID_SIZE] = {};
    TEST_ASSERT_FALSE(rx.takeReceivedBindUid(receivedUid));
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

    for (uint32_t t = 301; t <= 450; ++t) simTick(env, t, false);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Failsafe);
    TEST_ASSERT_FALSE(env.rx.outputActive());

    for (uint32_t t = 451; t <= 650; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);
    TEST_ASSERT_TRUE(env.rx.outputActive());
}

static void test_link_telemetry_downlink() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);
    env.txPhy.setRssi(-72);  // TX->RX uplink RSSI measured by RX.
    env.rxPhy.setRssi(-64);  // RX->TX downlink RSSI measured by TX.
    env.rxPhy.setSnr(6);

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
    TEST_ASSERT_EQUAL_INT16(-64, env.tx.stats().downlinkRssiDbm);
    TEST_ASSERT_EQUAL_INT8(6, env.tx.stats().downlinkSnr);
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
    TEST_ASSERT_EQUAL_INT8(10, env.txPhy.outputPowerDbm());

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

    TEST_ASSERT_FALSE(env.tx.setCipher(&txC));
    TEST_ASSERT_TRUE(env.tx.setSessionSalt(0x12345678u));
    TEST_ASSERT_TRUE(env.rx.setSessionSalt(0x12345678u));
    TEST_ASSERT_TRUE(env.tx.setCipher(&txC));
    TEST_ASSERT_TRUE(env.rx.setCipher(&rxC));
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

    TEST_ASSERT_TRUE(env.tx.setSessionSalt(0x12345678u));
    TEST_ASSERT_TRUE(env.rx.setSessionSalt(0x12345678u));
    TEST_ASSERT_TRUE(env.tx.setCipher(&txC));
    TEST_ASSERT_TRUE(env.rx.setCipher(&rxC));
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
    for (uint32_t t = 201; t <= 350; ++t) simTick(env, t);

    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Failsafe);
    TEST_ASSERT_FALSE(env.rx.outputActive());
    uint16_t during[4];
    env.rx.getChannels(during, 4);
    for (int i = 0; i < 4; ++i) TEST_ASSERT_EQUAL_UINT16(ch[i], during[i]);

    env.rxPhy.corruptNextDeliveries(0);
    for (uint32_t t = 351; t <= 700; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);
    TEST_ASSERT_TRUE(env.rx.stats().lqUp >= 90);
}

static void test_link_tx_failsafe_grace_after_connect() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2); // D250: tlm 1:16

    uint16_t ch[4] = {512, 512, 512, 512};
    env.tx.setChannels(ch, 4);

    for (uint32_t t = 1; t <= 100; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.tx.state() == LinkState::Connected);

    // Block downlink delivery; 500 ticks (~31 telemetry slots) stays inside the grace window.
    env.rxPhy.corruptNextDeliveries(10000);
    for (uint32_t t = 101; t <= 600; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.tx.state() == LinkState::Connected);
}

static void test_link_tx_bench_mode_stays_connected_on_downlink() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2); // D250: tlm 1:16

    env.tx.setBenchTxMode(true);
    // No setChannels — simulates bench TX without a controller handset.

    for (uint32_t t = 1; t <= 200; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.tx.state() == LinkState::Connected);
    TEST_ASSERT_TRUE(env.tx.stats().lqDown > 0);

    // Brief downlink loss must not failsafe while LQ window still shows healthy telemetry.
    env.txPhy.corruptNextDeliveries(80);
    for (uint32_t t = 201; t <= 280; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.tx.stats().lqDown > 0);
    TEST_ASSERT_TRUE(env.tx.state() == LinkState::Connected);
}

static void test_link_tx_bench_mode_failsafe_when_downlink_lost() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);

    env.tx.setBenchTxMode(true);
    for (uint32_t t = 1; t <= 100; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.tx.state() == LinkState::Connected);

    env.txPhy.corruptNextDeliveries(100000);
    for (uint32_t t = 101; t <= 2500; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.tx.stats().lqDown == 0);
    TEST_ASSERT_TRUE(env.tx.state() == LinkState::Failsafe);
}

static void test_link_rx_lq_holdoff_does_not_mask_rc_loss() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);

    uint16_t ch[4] = {512, 512, 512, 512};
    env.tx.setChannels(ch, 4);

    for (uint32_t t = 1; t <= 200; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);
    TEST_ASSERT_TRUE(env.rx.stats().lqUp >= Link::FAILSAFE_LQ_HOLDOFF);

    // Burst longer than FAILSAFE_MISS while the LQ window still shows a healthy link.
    // Failsafe must follow RC freshness, not a slow-draining LQ diagnostic.
    for (uint32_t t = 201; t <= 220; ++t) simTick(env, t, false);
    TEST_ASSERT_TRUE(env.rx.stats().lqUp >= Link::FAILSAFE_LQ_HOLDOFF);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Failsafe);
}

static void test_link_rx_lq_holdoff_failsafe_on_total_loss() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);

    uint16_t ch[4] = {512, 512, 512, 512};
    env.tx.setChannels(ch, 4);

    for (uint32_t t = 1; t <= 200; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);

    for (uint32_t t = 201; t <= 400; ++t) simTick(env, t, false);
    TEST_ASSERT_TRUE(env.rx.stats().lqUp < Link::FAILSAFE_LQ_HOLDOFF);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Failsafe);
}

static void test_link_tx_failsafe_counts_telemetry_slots() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 0); // F1000: tlmRatioDenom = 64

    uint16_t ch[4] = {512, 512, 512, 512};
    env.tx.setChannels(ch, 4);

    for (uint32_t t = 1; t <= 100; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.tx.state() == LinkState::Connected);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);

    // TX failsafe must count missed telemetry slots, not raw ticks. With tlm 1:64 the
    // next downlink slot can be ~54 ms away; 30 ticks (~30 ms) must not trip failsafe.
    for (uint32_t t = 101; t <= 130; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.tx.state() == LinkState::Connected);
}

// OI-019: AEAD must not run under the default (zero) session salt. Enabling a cipher before a
// nonzero salt is set must be refused, setSessionSalt(0) must be rejected and disable the
// cipher, and only a nonzero salt unlocks encryption. Guards against shipping a fixed/zero
// nonce salt in production.
static void test_link_cipher_requires_nonzero_session_salt() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    Link link;
    link.begin(Role::Tx, uid, 2);

    uint8_t key[32];
    for (int i = 0; i < 32; ++i) key[i] = (uint8_t)i;
    AeadCipher cipher;
    cipher.setKey(key);

    // Default salt is zero -> enabling a cipher is refused.
    TEST_ASSERT_FALSE(link.setCipher(&cipher));
    // Explicit zero salt is rejected and leaves AEAD disabled.
    TEST_ASSERT_FALSE(link.setSessionSalt(0));
    TEST_ASSERT_FALSE(link.setCipher(&cipher));
    // A nonzero, negotiated salt unlocks the cipher.
    TEST_ASSERT_TRUE(link.setSessionSalt(0xA5A5A5A5u));
    TEST_ASSERT_TRUE(link.setCipher(&cipher));
}

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_link_connect_flow_failsafe);
    RUN_TEST(test_link_mismatched_phrase_no_connect);
    RUN_TEST(test_link_bind_scan_accepts_offered_uid);
    RUN_TEST(test_link_bind_scan_accepts_offered_uid_with_tick_phase_offset);
    RUN_TEST(test_link_bind_scan_rejects_padded_bind_frame);
    RUN_TEST(test_link_fhss_acquire_and_recover);
    RUN_TEST(test_link_telemetry_downlink);
    RUN_TEST(test_link_rate_switch_and_power);
    RUN_TEST(test_link_encrypted);
    RUN_TEST(test_link_wrong_key);
    RUN_TEST(test_link_long_soak);
    RUN_TEST(test_link_corrupt_rc_rejected);
    RUN_TEST(test_link_tx_failsafe_grace_after_connect);
    RUN_TEST(test_link_tx_bench_mode_stays_connected_on_downlink);
    RUN_TEST(test_link_tx_bench_mode_failsafe_when_downlink_lost);
    RUN_TEST(test_link_rx_lq_holdoff_does_not_mask_rc_loss);
    RUN_TEST(test_link_rx_lq_holdoff_failsafe_on_total_loss);
    RUN_TEST(test_link_tx_failsafe_counts_telemetry_slots);
    RUN_TEST(test_link_cipher_requires_nonzero_session_salt);
    return UNITY_END();
}
