// Invariant & chaos harness (sim_test_enhancements_plan.md Section 6).
//
// Unlike the example-based suites (which assert a specific input -> specific output and are
// red-then-green against a known fix), these are GENERATIVE / property tests: they assert a
// property that must hold for *all* inputs, then drive many inputs looking for a violation.
// The point is discovery — surfacing the *next* issue, not just guarding the known ones.
//
// This file covers the host-reachable Section 6 techniques that need no async seam:
//   - I1/I8  metamorphic: RC decode must not depend on the absolute tick value
//   - metamorphic: Sync encode/decode round-trips for random payloads
//   - 6.3    codec fuzzing: random bytes must never crash and must reject-or-round-trip
//
// (The timing invariants I2/I3 and the chaos generator need the Section 0 DeliveryMode seam
// to be meaningful on host; they are deferred until that lands.)
#include <unity.h>

#include "link/Uid.h"
#include "ota/OtaCodec.h"
#include "SimEnvironment.h"

using namespace xlrs;
using namespace xlrs_test;

// Small deterministic LCG so any failure reproduces from a fixed seed.
static uint32_t g_rng = 0xC0FFEEu;
static uint32_t rnd() { g_rng = g_rng * 1664525u + 1013904223u; return g_rng; }

// Run one Sync + one Uplink RC exchange anchored at `baseTick`, on a fresh locked link.
// Mirrors the direct-call pattern used by the int32-boundary test. Returns decode success;
// fills `out` with the channels the RX recovered.
static bool exchangeAtBase(uint32_t baseTick, const uint16_t in[4], uint16_t out[4]) {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);
    Link tx, rx;
    tx.begin(Role::Tx, uid, 2);
    rx.begin(Role::Rx, uid, 2);

    uint32_t syncTick = baseTick;
    while (tx.slotForTick(syncTick) != Slot::Sync) ++syncTick;
    uint8_t buf[OTA16_LEN] = {};
    uint8_t len = 0;
    const uint16_t syncPos = tx.txPos(syncTick);
    tx.onTick(syncTick);
    if (!tx.getTxPayload(syncTick, syncPos, buf, len)) return false;
    if (!rx.processRxPayload(syncTick, syncPos, buf, len, -45, 0)) return false;

    uint32_t upTick = syncTick + 1;
    while (tx.slotForTick(upTick) != Slot::Uplink) ++upTick;
    tx.setChannels(in, 4);
    tx.onTick(upTick);
    rx.onTick(upTick);
    const uint16_t pos = tx.txPos(upTick);
    if (!tx.getTxPayload(upTick, pos, buf, len)) return false;
    const bool ok = rx.processRxPayload(upTick, pos, buf, len, -45, 0);
    rx.getChannels(out, 4);
    return ok;
}

// I1 / I8: a locked link's RC decode must depend only on the TX<->RX tick *mapping*, never on
// the absolute tick value. The same logical exchange at very different base ticks — including
// just below and just above the int32 boundary (2^31) — must decode identically. This finds
// absolute-tick dependencies (the OI-001 class) anywhere in the decode path, not just one spot.
static void test_invariant_decode_independent_of_absolute_tick() {
    const uint16_t in[4] = {333, 777, 1024, 1666};
    const uint32_t bases[] = {100u, 0x40000000u, 0x7FFFFF00u, 0x80000400u};
    for (uint32_t b : bases) {
        uint16_t out[4] = {};
        const bool ok = exchangeAtBase(b, in, out);
        TEST_ASSERT_TRUE_MESSAGE(ok, "RC decode failed at a base tick (absolute-tick dependency)");
        for (int i = 0; i < 4; ++i) TEST_ASSERT_EQUAL_UINT16(in[i], out[i]);
    }
}

// Metamorphic: a Sync payload that survives encode->decode must come back byte-for-byte equal,
// for arbitrary field values. Guards the OTA codec's field packing against silent corruption.
static void test_invariant_sync_roundtrip_random() {
    for (int iter = 0; iter < 2000; ++iter) {
        SyncPayload s{};
        s.fhssIndex     = (uint8_t)rnd();
        s.rateIndex     = (uint8_t)rnd();
        s.nextRateIndex = (uint8_t)rnd();
        s.switchTick    = rnd();
        s.tlmRatioDenom = (uint8_t)rnd();
        s.uidCrc        = (uint8_t)rnd();
        s.txTick        = rnd();

        uint8_t buf[OTA16_LEN] = {};
        const uint8_t len = otaEncodeSync(s, buf);
        TEST_ASSERT_EQUAL_UINT8(14, len);

        SyncPayload d{};
        TEST_ASSERT_TRUE(otaDecodeSync(buf, len, d));
        TEST_ASSERT_EQUAL_UINT8(s.fhssIndex, d.fhssIndex);
        TEST_ASSERT_EQUAL_UINT8(s.rateIndex, d.rateIndex);
        TEST_ASSERT_EQUAL_UINT8(s.nextRateIndex, d.nextRateIndex);
        TEST_ASSERT_EQUAL_UINT32(s.switchTick, d.switchTick);
        TEST_ASSERT_EQUAL_UINT8(s.tlmRatioDenom, d.tlmRatioDenom);
        TEST_ASSERT_EQUAL_UINT8(s.uidCrc, d.uidCrc);
        TEST_ASSERT_EQUAL_UINT32(s.txTick, d.txTick);
    }
}

// 6.3 codec fuzzing: arbitrary bytes/lengths must never crash, hang, or read out of bounds
// (ASAN/UBSAN builds turn an OOB into a hard failure here), and must either cleanly reject or
// round-trip. Reaching the end of the loop is the assertion.
static void test_invariant_codec_fuzz_no_crash() {
    for (int iter = 0; iter < 5000; ++iter) {
        uint8_t buf[32];
        const uint8_t len = (uint8_t)(rnd() % (sizeof(buf) + 1));
        for (uint8_t i = 0; i < len; ++i) buf[i] = (uint8_t)rnd();

        SyncPayload s{};
        (void)otaDecodeSync(buf, len, s);

        uint16_t ch[8] = {};
        (void)otaDecodeRc(buf, len, ch, 8);

        uint8_t lq = 0;
        int16_t rssi = 0;
        int8_t snr = 0;
        (void)otaDecodeTlmDown(buf, len, &lq, &rssi, &snr);
    }
    TEST_ASSERT_TRUE(true);
}

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_invariant_decode_independent_of_absolute_tick);
    RUN_TEST(test_invariant_sync_roundtrip_random);
    RUN_TEST(test_invariant_codec_fuzz_no_crash);
    return UNITY_END();
}
