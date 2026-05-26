#include <unity.h>
#include <cstring>

#include "link/StubbornTelemetry.h"
#include "link/Uid.h"
#include "SimEnvironment.h"

using namespace xlrs;
using namespace xlrs_test;

static void test_stubborn_telemetry_reliability() {
    StubbornSender sender;
    StubbornReceiver receiver;

    const char* message = "Flight telemetry packet test!";
    size_t msgLen = strlen(message);
    sender.queuePayload((const uint8_t*)message, msgLen);

    uint8_t steps = 0;
    uint32_t seed = 0x5678;
    while (!sender.idle() && steps < 200) {
        steps++;
        TelemetryChunk chunk;
        if (sender.getNextChunk(chunk)) {
            seed = seed * 1664525u + 1013904223u;
            if ((seed % 100) < 35) continue;

            uint8_t ackSeq = 0;
            (void)receiver.processChunk(chunk, ackSeq);

            seed = seed * 1664525u + 1013904223u;
            if ((seed % 100) < 35) continue;

            sender.receiveAck(ackSeq);
        }
    }

    TEST_ASSERT_TRUE(sender.idle());
    TEST_ASSERT_TRUE(receiver.ready());

    uint8_t decBuf[128] = {0};
    size_t decLen = 0;
    TEST_ASSERT_TRUE(receiver.getPayload(decBuf, decLen));
    TEST_ASSERT_EQUAL_UINT32(msgLen, decLen);
    TEST_ASSERT_EQUAL_STRING_LEN(message, (char*)decBuf, msgLen);
}

static void test_stubborn_sender_resets_after_peer_reboot_ack() {
    StubbornSender sender;
    const char* payload = "fragmented telemetry payload that spans several chunks";
    TEST_ASSERT_TRUE(sender.queuePayload((const uint8_t*)payload, strlen(payload)));

    TelemetryChunk first{};
    TEST_ASSERT_TRUE(sender.getNextChunk(first));
    TEST_ASSERT_EQUAL_UINT8(0, first.seq);
    sender.receiveAck(1);

    TelemetryChunk second{};
    TEST_ASSERT_TRUE(sender.getNextChunk(second));
    TEST_ASSERT_EQUAL_UINT8(1, second.seq);

    sender.receiveAck(0);
    TEST_ASSERT_FALSE(sender.idle());
    sender.receiveAck(0);
    TEST_ASSERT_FALSE(sender.idle());
    sender.receiveAck(0);
    TEST_ASSERT_TRUE(sender.idle());
}

static void test_link_stubborn_telemetry_integrated() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);

    for (uint32_t t = 1; t <= 100; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);

    const char* tlmData = "MSP config ok";
    size_t tlmLen = strlen(tlmData);
    env.rx.queueTelemetry((const uint8_t*)tlmData, tlmLen);

    for (uint32_t t = 101; t <= 1200; ++t) simTick(env, t);

    uint8_t rcvBuf[128] = {0};
    size_t rcvLen = 0;
    TEST_ASSERT_TRUE(env.tx.getTelemetry(rcvBuf, rcvLen));
    TEST_ASSERT_EQUAL_UINT32(tlmLen, rcvLen);
    TEST_ASSERT_EQUAL_STRING_LEN(tlmData, (char*)rcvBuf, tlmLen);
}

static void test_msp_frames_count_toward_lq() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);

    for (uint32_t t = 1; t <= 120; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);

    const uint16_t baselineMisses = env.tx.stats().telemetryMisses;
    const int8_t baselinePower = env.tx.txPowerDbm();

    const char* downlink = "RX stubborn telemetry should not look like packet loss";
    env.rx.queueTelemetry((const uint8_t*)downlink, strlen(downlink));
    for (uint32_t t = 121; t <= 900; ++t) simTick(env, t);

    uint8_t downBuf[128] = {0};
    size_t downLen = 0;
    TEST_ASSERT_TRUE(env.tx.getTelemetry(downBuf, downLen));
    TEST_ASSERT_EQUAL_UINT16(baselineMisses, env.tx.stats().telemetryMisses);
    TEST_ASSERT_TRUE(env.tx.txPowerDbm() <= baselinePower);
    TEST_ASSERT_TRUE(env.tx.stats().lqDown >= 95);

    const char* uplink = "TX MSP burst should not reduce uplink LQ";
    env.tx.queueTelemetry((const uint8_t*)uplink, strlen(uplink));
    for (uint32_t t = 901; t <= 1700; ++t) simTick(env, t);

    uint8_t upBuf[128] = {0};
    size_t upLen = 0;
    TEST_ASSERT_TRUE(env.rx.getTelemetry(upBuf, upLen));
    TEST_ASSERT_TRUE(env.rx.stats().lqUp >= 95);
}

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_stubborn_telemetry_reliability);
    RUN_TEST(test_stubborn_sender_resets_after_peer_reboot_ack);
    RUN_TEST(test_link_stubborn_telemetry_integrated);
    RUN_TEST(test_msp_frames_count_toward_lq);
    return UNITY_END();
}
