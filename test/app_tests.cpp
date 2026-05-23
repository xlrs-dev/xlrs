#include <unity.h>

#include "app/CrsfChannels.h"
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

static void test_crsf_channel_mapping() {
    TEST_ASSERT_EQUAL_UINT16(1000, crsfChannelToRcUs(CRSF_CHANNEL_VALUE_1000));
    TEST_ASSERT_EQUAL_UINT16(1500, crsfChannelToRcUs(CRSF_CHANNEL_VALUE_MID));
    TEST_ASSERT_EQUAL_UINT16(2000, crsfChannelToRcUs(CRSF_CHANNEL_VALUE_2000));

    TEST_ASSERT_EQUAL_UINT16(CRSF_CHANNEL_VALUE_1000, rcUsToCrsfChannel(1000));
    TEST_ASSERT_EQUAL_UINT16(CRSF_CHANNEL_VALUE_MID, rcUsToCrsfChannel(1500));
    TEST_ASSERT_EQUAL_UINT16(CRSF_CHANNEL_VALUE_2000, rcUsToCrsfChannel(2000));
}

static void test_crsf_packed_channels_to_rc_channels() {
    crsf_channels_t crsfChannels{};
    for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        setCrsfChannelByIndex(crsfChannels, i, rcUsToCrsfChannel((uint16_t)(1000 + i * 20)));
    }

    uint16_t rcChannels[CRSF_NUM_CHANNELS] = {};
    crsfChannelsToRcUs(crsfChannels, rcChannels);

    TEST_ASSERT_EQUAL_UINT16(1000, rcChannels[0]);
    TEST_ASSERT_EQUAL_UINT16(1020, rcChannels[1]);
    TEST_ASSERT_EQUAL_UINT16(1140, rcChannels[7]);
    TEST_ASSERT_EQUAL_UINT16(1300, rcChannels[15]);
}

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_crsf_link_statistics);
    RUN_TEST(test_crsf_channel_mapping);
    RUN_TEST(test_crsf_packed_channels_to_rc_channels);
    return UNITY_END();
}
