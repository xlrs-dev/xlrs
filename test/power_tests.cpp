#include <unity.h>

#include "link/Link.h"
#include "link/DynamicPower.h"
#include "link/Uid.h"
#include "ota/OtaCodec.h"

using namespace xlrs;

static void test_dynamic_power() {
    DynamicPower p;
    p.begin(0, 30, 15);
    for (int i = 0; i < 30; ++i) p.update(50, -90);
    TEST_ASSERT_EQUAL_INT8(30, p.powerDbm());
    for (int i = 0; i < 30; ++i) p.update(100, -55);
    TEST_ASSERT_EQUAL_INT8(0, p.powerDbm());
    int8_t held = p.powerDbm();
    for (int i = 0; i < 30; ++i) p.update(100, -80);
    TEST_ASSERT_EQUAL_INT8(held, p.powerDbm());
}

static void test_unauthenticated_tlmdown_does_not_steer_dynamic_power() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);

    Link tx;
    tx.begin(Role::Tx, uid, 2, 30, true);
    TEST_ASSERT_EQUAL_INT8(30, tx.txPowerDbm());

    for (uint32_t tick = 1; tick <= 4; ++tick) {
        uint8_t frame[OTA16_LEN] = {};
        uint8_t len = otaEncodeTlmDown(100, -30, 10, frame);
        frame[len++] = 0;
        TEST_ASSERT_TRUE(tx.processRxPayload(tick, tx.txPos(tick), frame, len, -30, 10));
    }

    TEST_ASSERT_EQUAL_INT8(30, tx.txPowerDbm());
    TEST_ASSERT_EQUAL_UINT8(100, tx.stats().lqUp);
}

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_dynamic_power);
    RUN_TEST(test_unauthenticated_tlmdown_does_not_steer_dynamic_power);
    return UNITY_END();
}
