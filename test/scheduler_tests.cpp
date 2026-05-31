#include <unity.h>

#include "link/Uid.h"
#include "timing/HwTimer.h"
#include "SimEnvironment.h"

using namespace xlrs;
using namespace xlrs_test;

static void test_scheduler_phy_recovery() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);

    TEST_ASSERT_TRUE(env.rxPhy.healthy());
    env.rxPhy.forceFault();
    TEST_ASSERT_FALSE(env.rxPhy.healthy());

    env.rxSched.poll();

    TEST_ASSERT_TRUE(env.rxPhy.healthy());
    TEST_ASSERT_EQUAL_UINT16(1, env.rx.stats().phyRecoveries);
    TEST_ASSERT_EQUAL_UINT16(0, env.rx.stats().phyRecoveryFailures);
}

static void test_scheduler_poll_skips_stale_ticks() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    MockPhy phy;
    PhyConfig cfg{};
    cfg.freqMHz = 2420.0f;
    phy.init(cfg);
    Link link;
    link.begin(Role::Tx, uid, 2);
    RfScheduler sched;
    sched.begin(&phy, &link, 2);

    for (int i = 0; i < 5; ++i) fireSimTimerTick();
    TEST_ASSERT_EQUAL_UINT32(5, sched.tickEvents());
    TEST_ASSERT_EQUAL_UINT32(0, sched.processedTick());

    sched.poll();
    TEST_ASSERT_EQUAL_UINT32(5, sched.processedTick());
    TEST_ASSERT_EQUAL_UINT16(4, link.stats().missedDeadlines);

    sched.poll();
    TEST_ASSERT_EQUAL_UINT32(5, sched.processedTick());
}

static void test_scheduler_poll_records_large_backlog() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    MockPhy phy;
    PhyConfig cfg{};
    cfg.freqMHz = 2420.0f;
    phy.init(cfg);
    Link link;
    link.begin(Role::Tx, uid, 2);
    RfScheduler sched;
    sched.begin(&phy, &link, 2);

    const uint32_t backlog = 84;
    for (uint32_t i = 0; i < backlog; ++i) fireSimTimerTick();
    TEST_ASSERT_EQUAL_UINT32(backlog, sched.tickEvents());

    sched.poll();
    TEST_ASSERT_EQUAL_UINT32(backlog, sched.processedTick());
    TEST_ASSERT_TRUE(link.stats().missedDeadlines >= backlog - 1);

    sched.poll();
    TEST_ASSERT_EQUAL_UINT32(backlog, sched.processedTick());
}

static void test_scheduler_rx_backlog_preserves_locked_phase() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);

    for (uint32_t t = 1; t <= 300; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);
    TEST_ASSERT_TRUE(env.rx.isLocked());
    TEST_ASSERT_TRUE(env.rx.outputActive());

    const uint32_t backlog = 36;
    for (uint32_t i = 0; i < backlog; ++i) fireSimTimerTick();
    env.rxSched.poll();

    TEST_ASSERT_EQUAL_UINT32(300u + backlog, env.rxSched.processedTick());
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);
    TEST_ASSERT_TRUE(env.rx.isLocked());
    TEST_ASSERT_TRUE(env.rx.stats().missedDeadlines >= backlog - 1);
    TEST_ASSERT_EQUAL_UINT8(env.rx.txPos(env.rx.effectiveTxTick(env.rxSched.processedTick())),
                            env.rx.stats().fhssIndex);

    fireSimTimerTick();
    env.rxSched.poll();
    TEST_ASSERT_EQUAL_UINT32(301u + backlog, env.rxSched.processedTick());
    TEST_ASSERT_EQUAL_UINT8(env.rx.txPos(env.rx.effectiveTxTick(env.rxSched.processedTick())),
                            env.rx.stats().fhssIndex);

    simTick(env, 302u + backlog);
    TEST_ASSERT_TRUE(env.rx.outputActive());
}

static void test_scheduler_phy_recovery_failure() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);

    env.rxPhy.setRecoveryFailures(3);
    env.rxPhy.forceFault();
    TEST_ASSERT_FALSE(env.rxPhy.healthy());

    for (int i = 0; i < 6 && !env.rxPhy.healthy(); ++i) {
        setSimulatedTimeUs(i * RfScheduler::PHY_RECOVERY_BACKOFF_US);
        env.rxSched.poll();
    }

    TEST_ASSERT_TRUE(env.rxPhy.healthy());
    TEST_ASSERT_EQUAL_UINT16(3, env.rx.stats().phyRecoveryFailures);
    TEST_ASSERT_EQUAL_UINT16(1, env.rx.stats().phyRecoveries);
}

static void test_scheduler_phy_recovery_backoff() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);

    env.rxPhy.setRecoveryFailures(2);
    env.rxPhy.forceFault();
    setSimulatedTimeUs(0);

    env.rxSched.poll();
    env.rxSched.poll();

    TEST_ASSERT_FALSE(env.rxPhy.healthy());
    TEST_ASSERT_EQUAL_UINT16(1, env.rx.stats().phyRecoveryFailures);

    setSimulatedTimeUs(RfScheduler::PHY_RECOVERY_BACKOFF_US);
    env.rxSched.poll();

    TEST_ASSERT_FALSE(env.rxPhy.healthy());
    TEST_ASSERT_EQUAL_UINT16(2, env.rx.stats().phyRecoveryFailures);
}

static void test_scheduler_skips_ticks_while_phy_unhealthy() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);

    env.rxPhy.setRecoveryFailures(1);
    env.rxPhy.forceFault();
    for (int i = 0; i < 3; ++i) fireSimTimerTick();

    setSimulatedTimeUs(0);
    env.rxSched.poll();

    TEST_ASSERT_FALSE(env.rxPhy.healthy());
    TEST_ASSERT_EQUAL_UINT32(0, env.rxSched.processedTick());
    TEST_ASSERT_EQUAL_UINT16(1, env.rx.stats().phyRecoveryFailures);

    setSimulatedTimeUs(RfScheduler::PHY_RECOVERY_BACKOFF_US);
    env.rxSched.poll();

    TEST_ASSERT_TRUE(env.rxPhy.healthy());
    TEST_ASSERT_EQUAL_UINT32(3, env.rxSched.processedTick());
    TEST_ASSERT_EQUAL_UINT16(1, env.rx.stats().phyRecoveries);
}

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_scheduler_phy_recovery);
    RUN_TEST(test_scheduler_poll_skips_stale_ticks);
    RUN_TEST(test_scheduler_poll_records_large_backlog);
    RUN_TEST(test_scheduler_rx_backlog_preserves_locked_phase);
    RUN_TEST(test_scheduler_phy_recovery_failure);
    RUN_TEST(test_scheduler_phy_recovery_backoff);
    RUN_TEST(test_scheduler_skips_ticks_while_phy_unhealthy);
    return UNITY_END();
}
