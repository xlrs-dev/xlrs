#include <unity.h>

#include <string.h>

#include "lora_link/protocol.h"

using namespace lora_link;

static void test_uid_is_deterministic() {
    uint8_t a[kUidSize];
    uint8_t b[kUidSize];
    uint8_t c[kUidSize];
    deriveUid("bench-phrase", a);
    deriveUid("bench-phrase", b);
    deriveUid("other-phrase", c);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(a, b, kUidSize);
    TEST_ASSERT_FALSE(memcmp(a, c, kUidSize) == 0);
    TEST_ASSERT_NOT_EQUAL(syncWordFromUid(a), syncWordFromUid(c));
}

static void test_fhss_depends_on_uid() {
    uint8_t a[kUidSize];
    uint8_t b[kUidSize];
    deriveUid("alpha", a);
    deriveUid("bravo", b);
    bool differs = false;
    for (uint16_t hop = 0; hop < 32; ++hop) {
        TEST_ASSERT_LESS_THAN_UINT8(40, fhssChannelFor(a, hop));
        if (fhssChannelFor(a, hop) != fhssChannelFor(b, hop)) differs = true;
    }
    TEST_ASSERT_TRUE(differs);
}

static void test_ota_round_trip_and_rejects_corruption() {
    uint8_t uid[kUidSize];
    deriveUid("ota", uid);
    OtaFrame in{};
    in.type = OtaType::Rc;
    in.sequence = 42;
    in.uidCheck = uidCheck(uid);
    in.payloadLen = 3;
    in.payload[0] = 1;
    in.payload[1] = 2;
    in.payload[2] = 3;

    uint8_t bytes[kOtaFrameSize];
    TEST_ASSERT_TRUE(encodeOtaFrame(in, bytes));
    OtaFrame out{};
    TEST_ASSERT_TRUE(decodeOtaFrame(bytes, uidCheck(uid), out));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OtaType::Rc), static_cast<uint8_t>(out.type));
    TEST_ASSERT_EQUAL_UINT16(42, out.sequence);
    TEST_ASSERT_EQUAL_UINT8(3, out.payloadLen);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(in.payload, out.payload, 3);

    bytes[12] ^= 0x40;
    TEST_ASSERT_FALSE(decodeOtaFrame(bytes, uidCheck(uid), out));
}

static void test_ota_carries_full_crsf_payload_without_corrupting_aux12() {
    uint8_t uid[kUidSize];
    deriveUid("full-payload", uid);
    uint16_t channels[kRcChannelCount];
    for (uint8_t i = 0; i < kRcChannelCount; ++i) channels[i] = 992;
    channels[15] = 1234;

    OtaFrame in{};
    in.type = OtaType::Rc;
    in.sequence = 7;
    in.uidCheck = uidCheck(uid);
    in.payloadLen = kOtaPayloadSize;
    packRcChannels11Bit(channels, in.payload);

    uint8_t bytes[kOtaFrameSize];
    TEST_ASSERT_TRUE(encodeOtaFrame(in, bytes));

    OtaFrame out{};
    uint16_t decoded[kRcChannelCount];
    TEST_ASSERT_TRUE(decodeOtaFrame(bytes, uidCheck(uid), out));
    TEST_ASSERT_EQUAL_UINT8(kOtaPayloadSize, out.payloadLen);
    unpackRcChannels11Bit(out.payload, decoded);
    TEST_ASSERT_EQUAL_UINT16_ARRAY(channels, decoded, kRcChannelCount);
}

static void test_crsf_channel_round_trip() {
    uint16_t channels[kRcChannelCount];
    uint16_t decoded[kRcChannelCount];
    for (uint8_t i = 0; i < kRcChannelCount; ++i) channels[i] = static_cast<uint16_t>(kCrsfRawMin + i * 100);

    uint8_t packed[22];
    packRcChannels11Bit(channels, packed);
    unpackRcChannels11Bit(packed, decoded);
    TEST_ASSERT_EQUAL_UINT16_ARRAY(channels, decoded, kRcChannelCount);

    uint8_t frame[32];
    const size_t len = encodeCrsfRcFrame(channels, frame, sizeof(frame));
    TEST_ASSERT_EQUAL_UINT(26, len);
    bool parsed = false;
    for (size_t i = 0; i < len; ++i) parsed = parseCrsfRcFrame(frame[i], decoded) || parsed;
    TEST_ASSERT_TRUE(parsed);
    TEST_ASSERT_EQUAL_UINT16_ARRAY(channels, decoded, kRcChannelCount);
}

static void test_channel_sanitizer_clamps_without_rejecting_normal_frame() {
    uint16_t previous[kRcChannelCount];
    uint16_t channels[kRcChannelCount];
    for (uint8_t i = 0; i < kRcChannelCount; ++i) {
        previous[i] = kCrsfRawMid;
        channels[i] = kCrsfRawMid;
    }
    channels[0] = 10;
    channels[1] = 2047;
    channels[2] = 1200;

    TEST_ASSERT_TRUE(sanitizeRcChannels(previous, true, channels));
    TEST_ASSERT_EQUAL_UINT16(kCrsfRawMin, channels[0]);
    TEST_ASSERT_EQUAL_UINT16(kCrsfRawMax, channels[1]);
    TEST_ASSERT_EQUAL_UINT16(1200, channels[2]);
}

static void test_channel_sanitizer_rejects_simultaneous_high_endpoint_spike() {
    uint16_t previous[kRcChannelCount];
    uint16_t channels[kRcChannelCount];
    for (uint8_t i = 0; i < kRcChannelCount; ++i) {
        previous[i] = kCrsfRawMid;
        channels[i] = kCrsfRawMid;
    }
    channels[0] = kCrsfRaw2000;
    channels[1] = kCrsfRaw2000;
    channels[2] = kCrsfRaw2000;
    channels[3] = kCrsfRaw2000;

    TEST_ASSERT_FALSE(sanitizeRcChannels(previous, true, channels));
}

static void test_channel_sanitizer_accepts_single_fast_channel_move() {
    uint16_t previous[kRcChannelCount];
    uint16_t channels[kRcChannelCount];
    for (uint8_t i = 0; i < kRcChannelCount; ++i) {
        previous[i] = kCrsfRawMid;
        channels[i] = kCrsfRawMid;
    }
    channels[3] = kCrsfRaw2000;

    TEST_ASSERT_TRUE(sanitizeRcChannels(previous, true, channels));
}

static void test_spike_gate_holds_one_frame_channel_outlier() {
    uint16_t previous[kRcChannelCount];
    uint16_t spike[kRcChannelCount];
    uint16_t normal[kRcChannelCount];
    for (uint8_t i = 0; i < kRcChannelCount; ++i) {
        previous[i] = kCrsfRawMid;
        spike[i] = kCrsfRawMid;
        normal[i] = kCrsfRawMid;
    }
    spike[6] = kCrsfRaw2000;

    RcSpikeGate gate{};
    TEST_ASSERT_FALSE(acceptRcChannelsWithSpikeGate(previous, true, spike, gate, 160, 80));
    TEST_ASSERT_TRUE(gate.havePending);
    TEST_ASSERT_TRUE(acceptRcChannelsWithSpikeGate(previous, true, normal, gate, 160, 80));
    TEST_ASSERT_FALSE(gate.havePending);
}

static void test_spike_gate_accepts_confirmed_large_channel_step() {
    uint16_t previous[kRcChannelCount];
    uint16_t first[kRcChannelCount];
    uint16_t second[kRcChannelCount];
    for (uint8_t i = 0; i < kRcChannelCount; ++i) {
        previous[i] = kCrsfRawMid;
        first[i] = kCrsfRawMid;
        second[i] = kCrsfRawMid;
    }
    first[0] = kCrsfRaw2000;
    second[0] = static_cast<uint16_t>(kCrsfRaw2000 - 20);

    RcSpikeGate gate{};
    TEST_ASSERT_FALSE(acceptRcChannelsWithSpikeGate(previous, true, first, gate, 160, 80));
    TEST_ASSERT_TRUE(acceptRcChannelsWithSpikeGate(previous, true, second, gate, 160, 80));
    TEST_ASSERT_FALSE(gate.havePending);
}

static void test_primary_slew_limiter_caps_stick_jumps_only() {
    uint16_t previous[kRcChannelCount];
    uint16_t channels[kRcChannelCount];
    for (uint8_t i = 0; i < kRcChannelCount; ++i) {
        previous[i] = kCrsfRawMid;
        channels[i] = kCrsfRawMid;
    }
    channels[0] = kCrsfRaw2000;
    channels[1] = kCrsfRaw1000;
    channels[4] = kCrsfRaw2000;

    slewLimitPrimaryRcChannels(previous, true, channels, 64);

    TEST_ASSERT_EQUAL_UINT16(kCrsfRawMid + 64, channels[0]);
    TEST_ASSERT_EQUAL_UINT16(kCrsfRawMid - 64, channels[1]);
    TEST_ASSERT_EQUAL_UINT16(kCrsfRawMid, channels[2]);
    TEST_ASSERT_EQUAL_UINT16(kCrsfRaw2000, channels[4]);
}

static void test_radio_transmitter_address_frame_is_accepted_as_crsf_raw() {
    uint16_t channels[kRcChannelCount];
    for (uint8_t i = 0; i < kRcChannelCount; ++i) channels[i] = kCrsfRawMid;
    channels[0] = kCrsfRaw1000;
    channels[1] = kCrsfRaw2000;

    uint8_t frame[26];
    frame[0] = 0xEE;
    frame[1] = 24;
    frame[2] = 0x16;
    packRcChannels11Bit(channels, &frame[3]);
    frame[25] = crsfCrc8(&frame[2], 23);

    uint16_t decoded[kRcChannelCount] = {};
    bool parsed = false;
    for (size_t i = 0; i < sizeof(frame); ++i) parsed = parseCrsfRcFrame(frame[i], decoded) || parsed;
    TEST_ASSERT_TRUE(parsed);
    TEST_ASSERT_EQUAL_UINT16(kCrsfRaw1000, decoded[0]);
    TEST_ASSERT_EQUAL_UINT16(kCrsfRaw2000, decoded[1]);
    TEST_ASSERT_EQUAL_UINT16(kCrsfRawMid, decoded[2]);
}

static void test_config_record_validation() {
    DeviceConfig cfg{};
    configDefaults(cfg, "default-phrase");
    cfg.rate = RateId::L100;
    ConfigRecord record = makeConfigRecord(cfg);

    DeviceConfig out{};
    TEST_ASSERT_TRUE(readConfigRecord(record, out));
    TEST_ASSERT_EQUAL_STRING("default-phrase", out.bindingPhrase);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(RateId::L100), static_cast<uint8_t>(out.rate));

    record.bindingPhrase[0] ^= 0x01;
    TEST_ASSERT_FALSE(readConfigRecord(record, out));
}

static void test_failsafe_gate_threshold_model() {
    const uint32_t lastUplinkMs = 1000;
    TEST_ASSERT_TRUE((1200u - lastUplinkMs) < 250u);
    TEST_ASSERT_FALSE((1300u - lastUplinkMs) < 250u);
}

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_uid_is_deterministic);
    RUN_TEST(test_fhss_depends_on_uid);
    RUN_TEST(test_ota_round_trip_and_rejects_corruption);
    RUN_TEST(test_ota_carries_full_crsf_payload_without_corrupting_aux12);
    RUN_TEST(test_crsf_channel_round_trip);
    RUN_TEST(test_channel_sanitizer_clamps_without_rejecting_normal_frame);
    RUN_TEST(test_channel_sanitizer_rejects_simultaneous_high_endpoint_spike);
    RUN_TEST(test_channel_sanitizer_accepts_single_fast_channel_move);
    RUN_TEST(test_spike_gate_holds_one_frame_channel_outlier);
    RUN_TEST(test_spike_gate_accepts_confirmed_large_channel_step);
    RUN_TEST(test_primary_slew_limiter_caps_stick_jumps_only);
    RUN_TEST(test_radio_transmitter_address_frame_is_accepted_as_crsf_raw);
    RUN_TEST(test_config_record_validation);
    RUN_TEST(test_failsafe_gate_threshold_model);
    return UNITY_END();
}
