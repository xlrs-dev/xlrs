#include <unity.h>

#include "hal/FlashStore.h"
#include "link/BindingStore.h"
#include "link/Uid.h"
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

static void test_flash_store_power_cut_keeps_last_committed_binding() {
    xlrs::hal::FlashStore::resetSim();
    TEST_ASSERT_TRUE(xlrs::hal::FlashStore::begin());

    BindingStore store;
    TEST_ASSERT_TRUE(store.setBindingPhrase("CUSTOM-A"));
    uint8_t customA[LINK_UID_SIZE] = {};
    TEST_ASSERT_TRUE(store.getBindingUid(customA));

    xlrs::hal::FlashStore::simulatePowerCutOnNextCommit();
    TEST_ASSERT_TRUE(store.setBindingPhrase("CUSTOM-B"));

    TEST_ASSERT_TRUE(xlrs::hal::FlashStore::begin());
    BindingStore rebooted;
    TEST_ASSERT_TRUE(rebooted.begin());
    uint8_t afterReboot[LINK_UID_SIZE] = {};
    TEST_ASSERT_TRUE(rebooted.getBindingUid(afterReboot));

    TEST_ASSERT_EQUAL_MEMORY(customA, afterReboot, LINK_UID_SIZE);
}

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_flash_boot_counter);
    RUN_TEST(test_flash_store_power_cut_keeps_last_committed_binding);
    return UNITY_END();
}
