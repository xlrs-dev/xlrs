#include <unity.h>

#include "util/FlashBootCounter.h"

using namespace xlrs;

static void test_flash_boot_counter() {
    FlashBootCounter::resetSim();
    TEST_ASSERT_EQUAL_UINT32(0, FlashBootCounter::read());
    TEST_ASSERT_EQUAL_UINT32(1, FlashBootCounter::increment());
    TEST_ASSERT_EQUAL_UINT32(1, FlashBootCounter::read());
    TEST_ASSERT_EQUAL_UINT32(2, FlashBootCounter::increment());
    TEST_ASSERT_EQUAL_UINT32(2, FlashBootCounter::read());
}

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_flash_boot_counter);
    return UNITY_END();
}
