#include <unity.h>

#include "link/RfScheduler.h"
#include "link/Uid.h"
#include "timing/HwTimer.h"
#include "timing/Pfd.h"
#include "SimEnvironment.h"

using namespace xlrs;
using namespace xlrs_test;

static void test_pfd_step_settles() {
    Pfd pfd;
    pfd.begin(4000);
    int32_t offset = 900;
    for (int i = 0; i < 1000; ++i) offset -= pfd.update(offset);
    TEST_ASSERT_TRUE(offset > -10 && offset < 10);
}

static void test_pfd_nulls_drift() {
    Pfd pfd;
    pfd.begin(4000);
    int32_t offset = 0;
    const int32_t drift = 3;
    for (int i = 0; i < 4000; ++i) {
        int32_t c = pfd.update(offset);
        offset += drift - c;
    }
    TEST_ASSERT_TRUE(offset > -5 && offset < 5);
}

static void test_scheduler_timing_pfd_lock() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);

    SimEnvironment env;
    env.setup(uid, 2);

    uint32_t txClock = 100000;
    uint32_t rxClock = 100000;
    setSimulatedTimeUs(rxClock);

    for (uint32_t t = 1; t <= 50; ++t) {
        txClock += 4000;
        rxClock += getSimulatedIntervalUs();
        env.txPhy.setClockUs(txClock);
        setSimulatedTimeUs(rxClock);
        simTick(env, t);
    }
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);

    uint32_t pfdUpdatesAtConnect = env.rxSched.pfdUpdateCount();
    int32_t phaseAtConnect = env.rxSched.pfdPhaseErrorUs();

    for (uint32_t t = 51; t <= 450; ++t) {
        txClock += 4012;
        rxClock += getSimulatedIntervalUs();
        env.txPhy.setClockUs(txClock);
        setSimulatedTimeUs(rxClock);
        simTick(env, t);
    }

    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);
    TEST_ASSERT_INT_WITHIN(2, 4012, getSimulatedIntervalUs());
    TEST_ASSERT_TRUE(env.rxSched.pfdUpdateCount() > pfdUpdatesAtConnect);
    TEST_ASSERT_INT_WITHIN(20, 0, env.rxSched.pfdPhaseErrorUs());
    TEST_ASSERT_INT_WITHIN(20, 0, phaseAtConnect);
}

static void test_scheduler_pfd_skipped_until_connected() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);

    SimEnvironment env;
    env.setup(uid, 2);

    uint32_t txClock = 100000;
    uint32_t rxClock = 100000;
    setSimulatedTimeUs(rxClock);
    bool sawConnected = false;

    for (uint32_t t = 1; t <= 50; ++t) {
        txClock += 4000;
        rxClock += getSimulatedIntervalUs();
        env.txPhy.setClockUs(txClock);
        setSimulatedTimeUs(rxClock);
        simTick(env, t);
        if (env.rx.state() == LinkState::Connected) {
            sawConnected = true;
        } else {
            TEST_ASSERT_EQUAL_UINT32(0, env.rxSched.pfdUpdateCount());
            TEST_ASSERT_EQUAL_UINT32(4000, getSimulatedIntervalUs());
        }
    }
    TEST_ASSERT_TRUE(sawConnected);
}

static void test_rc_decode_across_int32_tick_boundary() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);

    Link tx;
    Link rx;
    tx.begin(Role::Tx, uid, 2);
    rx.begin(Role::Rx, uid, 2);

    uint32_t syncTick = 0x7FFFFFF0u;
    while (tx.slotForTick(syncTick) != Slot::Sync) {
        ++syncTick;
    }
    uint8_t buf[OTA16_LEN] = {};
    uint8_t len = 0;
    const uint16_t syncPos = tx.txPos(syncTick);
    tx.onTick(syncTick);
    TEST_ASSERT_TRUE(tx.getTxPayload(syncTick, syncPos, buf, len));
    TEST_ASSERT_TRUE(rx.processRxPayload(syncTick, syncPos, buf, len, -45, 0));

    uint32_t uplinkTick = 0x80000000u;
    while (tx.slotForTick(uplinkTick) != Slot::Uplink) {
        ++uplinkTick;
    }
    uint16_t channels[4] = {333, 777, 1024, 1666};
    tx.setChannels(channels, 4);
    tx.onTick(uplinkTick);
    rx.onTick(uplinkTick);
    const uint16_t pos = tx.txPos(uplinkTick);
    TEST_ASSERT_TRUE(tx.getTxPayload(uplinkTick, pos, buf, len));
    TEST_ASSERT_TRUE(rx.processRxPayload(uplinkTick, pos, buf, len, -45, 0));

    uint16_t out[4] = {};
    rx.getChannels(out, 4);
    for (int i = 0; i < 4; ++i) TEST_ASSERT_EQUAL_UINT16(channels[i], out[i]);
}

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_pfd_step_settles);
    RUN_TEST(test_pfd_nulls_drift);
    RUN_TEST(test_scheduler_timing_pfd_lock);
    RUN_TEST(test_scheduler_pfd_skipped_until_connected);
    RUN_TEST(test_rc_decode_across_int32_tick_boundary);
    return UNITY_END();
}
