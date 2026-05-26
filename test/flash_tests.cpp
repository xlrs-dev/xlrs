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

static void test_flash_store_migrates_legacy_layout_without_erasing_on_cut() {
    uint8_t legacy[4096] = {};
    memset(legacy, 0xFF, sizeof(legacy));

    xlrs::hal::FlashStore::resetSim();
    TEST_ASSERT_TRUE(xlrs::hal::FlashStore::begin());
    BindingStore legacyWriter;
    TEST_ASSERT_TRUE(legacyWriter.setBindingPhrase("LEGACY-PHRASE"));
    for (size_t i = 0; i < sizeof(legacy); ++i) {
        legacy[i] = xlrs::hal::FlashStore::read(i);
    }
    uint8_t legacyUid[LINK_UID_SIZE] = {};
    TEST_ASSERT_TRUE(legacyWriter.getBindingUid(legacyUid));

    xlrs::hal::FlashStore::seedLegacySim(legacy, sizeof(legacy));
    TEST_ASSERT_TRUE(xlrs::hal::FlashStore::begin());
    BindingStore migrated;
    TEST_ASSERT_TRUE(migrated.begin());
    uint8_t migratedUid[LINK_UID_SIZE] = {};
    TEST_ASSERT_TRUE(migrated.getBindingUid(migratedUid));
    TEST_ASSERT_EQUAL_MEMORY(legacyUid, migratedUid, LINK_UID_SIZE);

    xlrs::hal::FlashStore::simulatePowerCutOnNextCommit();
    TEST_ASSERT_FALSE(xlrs::hal::FlashStore::commit());

    TEST_ASSERT_TRUE(xlrs::hal::FlashStore::begin());
    BindingStore afterCut;
    TEST_ASSERT_TRUE(afterCut.begin());
    uint8_t afterCutUid[LINK_UID_SIZE] = {};
    TEST_ASSERT_TRUE(afterCut.getBindingUid(afterCutUid));
    TEST_ASSERT_EQUAL_MEMORY(legacyUid, afterCutUid, LINK_UID_SIZE);
}

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_flash_boot_counter);
    RUN_TEST(test_flash_store_power_cut_keeps_last_committed_binding);
    RUN_TEST(test_flash_store_migrates_legacy_layout_without_erasing_on_cut);
    return UNITY_END();
}
