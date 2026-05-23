#include <unity.h>

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

    for (uint32_t t = 51; t <= 450; ++t) {
        txClock += 4012;
        rxClock += getSimulatedIntervalUs();
        env.txPhy.setClockUs(txClock);
        setSimulatedTimeUs(rxClock);
        simTick(env, t);
    }

    TEST_ASSERT_INT_WITHIN(2, 4012, getSimulatedIntervalUs());
}

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_pfd_step_settles);
    RUN_TEST(test_pfd_nulls_drift);
    RUN_TEST(test_scheduler_timing_pfd_lock);
    return UNITY_END();
}
