#include <unity.h>

#include "app/AppTelemetry.h"
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

static void test_crsf_frame_address_validation() {
    TEST_ASSERT_TRUE(isCrsfFrameAddress(CRSF_ADDRESS_BROADCAST));
    TEST_ASSERT_TRUE(isCrsfFrameAddress(CRSF_ADDRESS_FLIGHT_CONTROLLER));
    TEST_ASSERT_TRUE(isCrsfFrameAddress(CRSF_ADDRESS_RADIO_TRANSMITTER));
    TEST_ASSERT_TRUE(isCrsfFrameAddress(CRSF_ADDRESS_CRSF_RECEIVER));
    TEST_ASSERT_TRUE(isCrsfFrameAddress(CRSF_ADDRESS_CRSF_TRANSMITTER));
    TEST_ASSERT_TRUE(isCrsfFrameAddress(CRSF_ADDRESS_GPS));

    TEST_ASSERT_FALSE(isCrsfFrameAddress(0x41));
    TEST_ASSERT_FALSE(isCrsfFrameAddress(0xFF));
}

static void test_app_telemetry_messages() {
    RfConfigData cfg{};
    RfConfig::setDefaults(cfg);
    cfg.region = (uint8_t)RfRegion::EU;
    cfg.defaultRate = 3;
    cfg.maxPowerDbm = 10;
    cfg.failsafeMode = 1;
    cfg.dynamicPower = 0;

    AppTelemetryMessage message{};
    TEST_ASSERT_TRUE(makeRxConfigMessage(cfg, message));
    RfConfigData parsed{};
    RfConfig::setDefaults(parsed);
    TEST_ASSERT_TRUE(parseRxConfigMessage(message.data, message.len, parsed));
    TEST_ASSERT_EQUAL_UINT8(cfg.region, parsed.region);
    TEST_ASSERT_EQUAL_UINT8(cfg.defaultRate, parsed.defaultRate);
    TEST_ASSERT_EQUAL_INT8(cfg.maxPowerDbm, parsed.maxPowerDbm);
    TEST_ASSERT_EQUAL_UINT8(cfg.failsafeMode, parsed.failsafeMode);
    TEST_ASSERT_EQUAL_UINT8(cfg.dynamicPower, parsed.dynamicPower);

    const uint8_t uid[LINK_UID_SIZE] = {1,2,3,4,5,6,7,8};
    uint8_t parsedUid[LINK_UID_SIZE] = {};
    TEST_ASSERT_TRUE(makeBindUidMessage(uid, message));
    TEST_ASSERT_TRUE(parseBindUidMessage(message.data, message.len, parsedUid));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(uid, parsedUid, LINK_UID_SIZE);
}

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_crsf_link_statistics);
    RUN_TEST(test_crsf_channel_mapping);
    RUN_TEST(test_crsf_packed_channels_to_rc_channels);
    RUN_TEST(test_crsf_frame_address_validation);
    RUN_TEST(test_app_telemetry_messages);
    return UNITY_END();
}
