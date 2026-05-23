#include <unity.h>

#include "app/CrsfLinkStats.h"

using namespace xlrs;

static void test_crsf_link_statistics() {
    LinkStats s{};
    s.lqUp = 95;
    s.lqDown = 80;
    s.rssiDbm = -72;
    s.snr = 7;
    s.rateIndex = 2;
    uint8_t f[10];
    buildCrsfLinkStatistics(s, f);
    TEST_ASSERT_EQUAL_UINT8(72, f[0]);
    TEST_ASSERT_EQUAL_UINT8(95, f[2]);
    TEST_ASSERT_EQUAL_UINT8(7, f[3]);
    TEST_ASSERT_EQUAL_UINT8(2, f[5]);
    TEST_ASSERT_EQUAL_UINT8(80, f[8]);
}

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_crsf_link_statistics);
    return UNITY_END();
}
