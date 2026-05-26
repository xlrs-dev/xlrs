#include <unity.h>

#include "app/AppTelemetry.h"
#include "app/CrsfChannels.h"
#include "app/CrsfLinkStats.h"
#include "app/LinkStatusLed.h"
#include "app/PersistencePolicy.h"

using namespace xlrs;

static void test_crsf_link_statistics() {
    LinkStats s{};
    s.lqUp = 95;
    s.lqDown = 80;
    s.rssiDbm = -72;
    s.snr = 7;
    s.downlinkRssiDbm = -68;
    s.downlinkSnr = 5;
    s.rateIndex = 2;
    uint8_t f[10];
    buildCrsfLinkStatistics(s, f);
    TEST_ASSERT_EQUAL_UINT8(72, f[0]);
    TEST_ASSERT_EQUAL_UINT8(95, f[2]);
    TEST_ASSERT_EQUAL_UINT8(7, f[3]);
    TEST_ASSERT_EQUAL_UINT8(2, f[5]);
    TEST_ASSERT_EQUAL_UINT8(68, f[7]);
    TEST_ASSERT_EQUAL_UINT8(80, f[8]);
    TEST_ASSERT_EQUAL_UINT8(5, f[9]);
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
    uint16_t input[CRSF_NUM_CHANNELS] = {};
    crsf_channels_t crsfChannels{};
    for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        input[i] = (uint16_t)(1000 + i * 20);
    }
    rcUsToCrsfChannels(input, crsfChannels);

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

    memset(parsedUid, 0, sizeof(parsedUid));
    TEST_ASSERT_TRUE(makeStartBindMessage(uid, message));
    TEST_ASSERT_TRUE(parseStartBindMessage(message.data, message.len, parsedUid));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(uid, parsedUid, LINK_UID_SIZE);
}

static void test_persistence_policy_blocks_active_link_states() {
    TEST_ASSERT_TRUE(app::persistenceAllowed(LinkState::Disconnected, false));
    TEST_ASSERT_TRUE(app::persistenceAllowed(LinkState::Connecting, false));
    TEST_ASSERT_TRUE(app::persistenceAllowed(LinkState::Binding, false));
    TEST_ASSERT_FALSE(app::persistenceAllowed(LinkState::Connected, false));
    TEST_ASSERT_FALSE(app::persistenceAllowed(LinkState::Failsafe, false));
    TEST_ASSERT_FALSE(app::persistenceAllowed(LinkState::Connecting, true));
}
// OI-007: the "bind packet received" LED pattern must actually blink when sampled at the
// 50 ms update cadence — not alias to solid-on (which would be indistinguishable from the
// fault and connected indications). Sample one full pattern period at 50 ms steps and require
// both an ON and an OFF sample.
static void test_link_status_led_bind_received_blinks_not_solid() {
    using namespace xlrs::app;
    LinkStatusLedFlags flags;
    flags.bindPacketReceived = true;
    bool sawOn = false;
    bool sawOff = false;
    for (uint32_t ms = 50; ms <= 400; ms += 50) {
        const bool on = linkStatusLedComputeOn(LinkState::Connecting, flags, ms);
        sawOn |= on;
        sawOff |= !on;
    }
    TEST_ASSERT_TRUE(sawOn);   // pattern actually lights the LED
    TEST_ASSERT_TRUE(sawOff);  // and actually blinks — not aliased to solid-on (OI-007)
}

// A hardware/config fault must read as steady solid-on at every sample, so it stays
// distinguishable from the (blinking) bind-received pattern above.
static void test_link_status_led_fault_is_solid_on() {
    using namespace xlrs::app;
    LinkStatusLedFlags flags;
    flags.hardwareError = true;
    for (uint32_t ms = 50; ms <= 400; ms += 50) {
        TEST_ASSERT_TRUE(linkStatusLedComputeOn(LinkState::Disconnected, flags, ms));
    }
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
    RUN_TEST(test_persistence_policy_blocks_active_link_states);
    RUN_TEST(test_link_status_led_bind_received_blinks_not_solid);
    RUN_TEST(test_link_status_led_fault_is_solid_on);
    return UNITY_END();
}
