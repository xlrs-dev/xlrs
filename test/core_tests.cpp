#include <unity.h>
#include <string.h>

#include "app/CrsfLinkStats.h"
#include "fhss/Fhss.h"
#include "link/Link.h"
#include "link/RfScheduler.h"
#include "link/Uid.h"
#include "ota/ChannelPack.h"
#include "ota/OtaFrameShrink.h"
#include "phy/MockPhy.h"
#include "util/Mailbox.h"
#include "util/RingBuffer.h"

using namespace xlrs;

static int s_rxDoneCount = 0;
static void onRxDoneCb() { ++s_rxDoneCount; }

static void test_link_uid_from_phrase() {
    uint8_t a[LINK_UID_SIZE], b[LINK_UID_SIZE], c[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", a);
    linkUidFromPhrase("Kikobot-02", b);
    linkUidFromPhrase("different", c);
    TEST_ASSERT_EQUAL_MEMORY(a, b, LINK_UID_SIZE);
    TEST_ASSERT_TRUE(memcmp(a, c, LINK_UID_SIZE) != 0);

    uint8_t e[LINK_UID_SIZE];
    linkUidFromPhrase("", e);
    const uint64_t basis = 0xcbf29ce484222325ULL;
    for (int i = 0; i < LINK_UID_SIZE; ++i) {
        TEST_ASSERT_EQUAL_UINT8((uint8_t)(basis >> (56 - 8 * i)), e[i]);
    }
}

static void test_uid_drives_shared_fhss() {
    uint8_t txU[LINK_UID_SIZE], rxU[LINK_UID_SIZE], other[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", txU);
    linkUidFromPhrase("Kikobot-02", rxU);
    linkUidFromPhrase("OtherCraft", other);

    Fhss tx, rx, o;
    tx.generate(fhssSeedFromUid(txU), 20, 40);
    rx.generate(fhssSeedFromUid(rxU), 20, 40);
    o.generate(fhssSeedFromUid(other), 20, 40);

    for (int i = 0; i < 40; ++i) TEST_ASSERT_EQUAL_UINT8(tx.at(i), rx.at(i));
    bool differs = false;
    for (int i = 0; i < 40; ++i) {
        if (o.at(i) != tx.at(i)) { differs = true; break; }
    }
    TEST_ASSERT_TRUE(differs);
}

static void test_mockphy_loopback() {
    MockPhy tx, rx;
    PhyConfig cfg{};
    cfg.freqMHz = 2420.0f;
    cfg.payloadLen = 8;
    TEST_ASSERT_TRUE(tx.init(cfg));
    TEST_ASSERT_TRUE(rx.init(cfg));
    MockPhy::connect(tx, rx);
    s_rxDoneCount = 0;
    rx.setOnRxDone(onRxDoneCb);

    rx.startRx(2420.0f);
    uint8_t payload[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    tx.setClockUs(12345);
    tx.startTx(2420.0f, payload, 8);

    TEST_ASSERT_EQUAL_INT(1, s_rxDoneCount);
    RxPacket pkt;
    TEST_ASSERT_TRUE(rx.readRx(pkt));
    TEST_ASSERT_EQUAL_UINT8(8, pkt.len);
    for (int i = 0; i < 8; ++i) TEST_ASSERT_EQUAL_UINT8(payload[i], pkt.data[i]);
    TEST_ASSERT_EQUAL_UINT32(12345, pkt.timestampUs);
    TEST_ASSERT_FALSE(rx.readRx(pkt));

    s_rxDoneCount = 0;
    rx.startRx(2420.0f);
    tx.startTx(2421.0f, payload, 8);
    TEST_ASSERT_EQUAL_INT(0, s_rxDoneCount);
    TEST_ASSERT_FALSE(rx.readRx(pkt));
}

static void test_channel_pack_roundtrip() {
    uint16_t in[8] = {0, 2047, 1024, 1, 2046, 512, 1500, 3};
    uint8_t buf[packedSize(8)] = {0};
    packChannels(in, 8, buf);
    uint16_t out[8] = {0};
    unpackChannels(buf, 8, out);
    for (int i = 0; i < 8; ++i) TEST_ASSERT_EQUAL_UINT16(in[i], out[i]);
    TEST_ASSERT_EQUAL_UINT8(11, packedSize(8));
}

static void test_fhss_deterministic_and_balanced() {
    Fhss a, b;
    a.generate(0x1234, 8, 40);
    b.generate(0x1234, 8, 40);
    for (int i = 0; i < 40; ++i) TEST_ASSERT_EQUAL_UINT8(a.at(i), b.at(i));

    int counts[8] = {0};
    for (int i = 0; i < 40; ++i) {
        TEST_ASSERT_TRUE(a.at(i) < 8);
        counts[a.at(i)]++;
    }
    for (int i = 0; i < 8; ++i) TEST_ASSERT_EQUAL_INT(5, counts[i]);

    Fhss c;
    c.generate(0x9999, 8, 40);
    bool differs = false;
    for (int i = 0; i < 40; ++i) {
        if (c.at(i) != a.at(i)) { differs = true; break; }
    }
    TEST_ASSERT_TRUE(differs);
}

static void test_fhss_80_channel_balance() {
    Fhss a;
    a.generate(0xABCD, 80, 80);
    int counts[80] = {0};
    for (int i = 0; i < 80; ++i) {
        TEST_ASSERT_TRUE(a.at(i) < 80);
        counts[a.at(i)]++;
    }
    for (int i = 0; i < 80; ++i) TEST_ASSERT_EQUAL_INT(1, counts[i]);
}

static void test_ota_8_byte_packing() {
    uint16_t origCh[8] = {2000, 1000, 1500, 400, 100, 1024, 1024, 1024};
    uint8_t buf[8] = {0};
    uint8_t seq = 42;
    packChannels8Byte(origCh, seq, buf);

    uint16_t decCh[8] = {0};
    uint8_t decSeq = 0;
    TEST_ASSERT_TRUE(unpackChannels8Byte(buf, decCh, decSeq));
    TEST_ASSERT_EQUAL_UINT8(seq, decSeq);
    for (int i = 0; i < 5; ++i) TEST_ASSERT_EQUAL_UINT16(origCh[i] & 0xFFFE, decCh[i]);
    for (int i = 5; i < 8; ++i) TEST_ASSERT_EQUAL_UINT16(1024, decCh[i]);
}

static void test_lq_tracker() {
    LqTracker<4> lq;
    for (int i = 0; i < 4; ++i) lq.update((i % 4) != 3);
    TEST_ASSERT_EQUAL_UINT8(75, lq.lq());
}

static void test_spsc_ring() {
    SpscRing<int, 4> r;
    TEST_ASSERT_TRUE(r.push(1));
    TEST_ASSERT_TRUE(r.push(2));
    TEST_ASSERT_TRUE(r.push(3));
    TEST_ASSERT_FALSE(r.push(4));
    int v;
    TEST_ASSERT_TRUE(r.pop(v)); TEST_ASSERT_EQUAL_INT(1, v);
    TEST_ASSERT_TRUE(r.pop(v)); TEST_ASSERT_EQUAL_INT(2, v);
    TEST_ASSERT_TRUE(r.push(4));
    TEST_ASSERT_TRUE(r.pop(v)); TEST_ASSERT_EQUAL_INT(3, v);
    TEST_ASSERT_TRUE(r.pop(v)); TEST_ASSERT_EQUAL_INT(4, v);
    TEST_ASSERT_FALSE(r.pop(v));
}

static void test_latest_value_freshest_wins() {
    LatestValue<int> mb;
    int v = -1;
    TEST_ASSERT_TRUE(mb.load(v)); TEST_ASSERT_EQUAL_INT(0, v);
    mb.store(5);
    mb.store(7);
    TEST_ASSERT_TRUE(mb.load(v)); TEST_ASSERT_EQUAL_INT(7, v);
    TEST_ASSERT_TRUE(mb.load(v)); TEST_ASSERT_EQUAL_INT(7, v);
}

static void test_latest_value_load_or_keep() {
    LatestValue<int> mb;
    int local = 42;
    TEST_ASSERT_TRUE(mb.loadOrKeep(local));
    TEST_ASSERT_EQUAL_INT(0, local);
    mb.store(7);
    TEST_ASSERT_TRUE(mb.loadOrKeep(local));
    TEST_ASSERT_EQUAL_INT(7, local);
}

// (test_slot_for_tick removed: the free slotForTick helper was deleted in favor of the single
//  authoritative Link::slotForTick. Slot precedence is covered behaviorally by the link sim tests.)

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_channel_pack_roundtrip);
    RUN_TEST(test_fhss_deterministic_and_balanced);
    RUN_TEST(test_fhss_80_channel_balance);
    RUN_TEST(test_ota_8_byte_packing);
    RUN_TEST(test_lq_tracker);
    RUN_TEST(test_spsc_ring);
    RUN_TEST(test_latest_value_freshest_wins);
    RUN_TEST(test_latest_value_load_or_keep);
    RUN_TEST(test_mockphy_loopback);
    RUN_TEST(test_link_uid_from_phrase);
    RUN_TEST(test_uid_drives_shared_fhss);
    return UNITY_END();
}
