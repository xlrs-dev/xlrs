#include <unity.h>

#include "link/DynamicPower.h"

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

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_dynamic_power);
    return UNITY_END();
}
