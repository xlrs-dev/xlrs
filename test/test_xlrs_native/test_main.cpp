// Native (host) unit tests for the XLRS pure-logic layers.
#include <unity.h>
#include "ota/ChannelPack.h"
#include "ota/OtaFrameShrink.h"
#include "fhss/Fhss.h"
#include "timing/Pfd.h"
#include "util/RingBuffer.h"
#include "util/Mailbox.h"
#include "link/Link.h"
#include "link/StubbornTelemetry.h"
#include "link/RfScheduler.h"
#include "link/Uid.h"
#include "phy/MockPhy.h"
#include "app/CrsfLinkStats.h"
#include "crypto/AeadCipher.h"
#include "crypto/Chacha20Poly1305.h"
#include "util/FlashBootCounter.h"
#include <string.h>

using namespace xlrs;

// ---- Link UID: FNV-1a, deterministic + phrase-sensitive + known vector ----
static void test_link_uid_from_phrase() {
    uint8_t a[LINK_UID_SIZE], b[LINK_UID_SIZE], c[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", a);
    linkUidFromPhrase("Kikobot-02", b);
    linkUidFromPhrase("different",  c);
    TEST_ASSERT_EQUAL_MEMORY(a, b, LINK_UID_SIZE);          // deterministic
    TEST_ASSERT_TRUE(memcmp(a, c, LINK_UID_SIZE) != 0);     // phrase-sensitive

    uint8_t e[LINK_UID_SIZE];
    linkUidFromPhrase("", e);                               // FNV-1a("") == offset basis
    const uint64_t basis = 0xcbf29ce484222325ULL;
    for (int i = 0; i < LINK_UID_SIZE; ++i)
        TEST_ASSERT_EQUAL_UINT8((uint8_t)(basis >> (56 - 8 * i)), e[i]);
}

// ---- The bind property: same phrase => same FHSS sequence (they meet); else isolation ----
static void test_uid_drives_shared_fhss() {
    uint8_t txU[LINK_UID_SIZE], rxU[LINK_UID_SIZE], other[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", txU);
    linkUidFromPhrase("Kikobot-02", rxU);
    linkUidFromPhrase("OtherCraft", other);

    Fhss tx, rx, o;
    tx.generate(fhssSeedFromUid(txU), 20, 40);
    rx.generate(fhssSeedFromUid(rxU), 20, 40);
    o.generate(fhssSeedFromUid(other), 20, 40);

    for (int i = 0; i < 40; ++i) TEST_ASSERT_EQUAL_UINT8(tx.at(i), rx.at(i)); // same phrase → meet
    bool differs = false;
    for (int i = 0; i < 40; ++i) if (o.at(i) != tx.at(i)) { differs = true; break; }
    TEST_ASSERT_TRUE(differs);                                                 // else isolated
}

// ---- MockPhy: TX delivers to a listening RX only on a matching frequency (M1) ----
static int s_rxDoneCount = 0;
static void onRxDoneCb() { ++s_rxDoneCount; }

static void test_mockphy_loopback() {
    MockPhy tx, rx;
    PhyConfig cfg{}; cfg.freqMHz = 2420.0f; cfg.payloadLen = 8;
    TEST_ASSERT_TRUE(tx.init(cfg));
    TEST_ASSERT_TRUE(rx.init(cfg));
    MockPhy::connect(tx, rx);
    s_rxDoneCount = 0;
    rx.setOnRxDone(onRxDoneCb);

    rx.startRx(2420.0f);
    uint8_t payload[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    tx.setClockUs(12345);
    tx.startTx(2420.0f, payload, 8);

    TEST_ASSERT_EQUAL_INT(1, s_rxDoneCount);            // RxDone callback fired
    RxPacket pkt;
    TEST_ASSERT_TRUE(rx.readRx(pkt));
    TEST_ASSERT_EQUAL_UINT8(8, pkt.len);
    for (int i = 0; i < 8; ++i) TEST_ASSERT_EQUAL_UINT8(payload[i], pkt.data[i]);
    TEST_ASSERT_EQUAL_UINT32(12345, pkt.timestampUs);   // arrival timestamp carried
    TEST_ASSERT_FALSE(rx.readRx(pkt));                  // drained after one read

    // Frequency mismatch => no delivery (models FHSS channel isolation / hop desync).
    s_rxDoneCount = 0;
    rx.startRx(2420.0f);
    tx.startTx(2421.0f, payload, 8);
    TEST_ASSERT_EQUAL_INT(0, s_rxDoneCount);
    TEST_ASSERT_FALSE(rx.readRx(pkt));
}

// ---- ChannelPack: 11-bit (un)pack round-trips exactly ----
static void test_channel_pack_roundtrip() {
    uint16_t in[8] = { 0, 2047, 1024, 1, 2046, 512, 1500, 3 };
    uint8_t buf[packedSize(8)] = {0};
    packChannels(in, 8, buf);
    uint16_t out[8] = {0};
    unpackChannels(buf, 8, out);
    for (int i = 0; i < 8; ++i) TEST_ASSERT_EQUAL_UINT16(in[i], out[i]);
    TEST_ASSERT_EQUAL_UINT8(11, packedSize(8));
}

// ---- FHSS: deterministic per seed, balanced, seed-sensitive ----
static void test_fhss_deterministic_and_balanced() {
    Fhss a, b;
    a.generate(0x1234, 8, 40);
    b.generate(0x1234, 8, 40);
    for (int i = 0; i < 40; ++i) TEST_ASSERT_EQUAL_UINT8(a.at(i), b.at(i)); // determinism

    int counts[8] = {0};
    for (int i = 0; i < 40; ++i) { TEST_ASSERT_TRUE(a.at(i) < 8); counts[a.at(i)]++; }
    for (int i = 0; i < 8; ++i) TEST_ASSERT_EQUAL_INT(5, counts[i]);        // balanced (40/8)

    Fhss c; c.generate(0x9999, 8, 40);
    bool differs = false;
    for (int i = 0; i < 40; ++i) if (c.at(i) != a.at(i)) { differs = true; break; }
    TEST_ASSERT_TRUE(differs);                                             // seed-sensitive
}

static void test_fhss_80_channel_balance() {
    Fhss a;
    a.generate(0xABCD, 80, 80);
    int counts[80] = {0};
    for (int i = 0; i < 80; ++i) {
        TEST_ASSERT_TRUE(a.at(i) < 80);
        counts[a.at(i)]++;
    }
    for (int i = 0; i < 80; ++i) {
        TEST_ASSERT_EQUAL_INT(1, counts[i]); // exactly 1 hit per channel for 80 hops over 80 channels
    }
}

static void test_ota_8_byte_packing() {
    uint16_t origCh[8] = { 2000, 1000, 1500, 400, 100, 1024, 1024, 1024 };
    uint8_t buf[8] = {0};
    uint8_t seq = 42;

    packChannels8Byte(origCh, seq, buf);

    uint16_t decCh[8] = {0};
    uint8_t decSeq = 0;
    bool status = unpackChannels8Byte(buf, decCh, decSeq);

    TEST_ASSERT_TRUE(status);
    TEST_ASSERT_EQUAL_UINT8(seq, decSeq);

    // Verify 10-bit resolution round-trip (loss of LSB bit 0)
    for (int i = 0; i < 5; ++i) {
        TEST_ASSERT_EQUAL_UINT16(origCh[i] & 0xFFFE, decCh[i]);
    }
    // Verify default fallback values for aux channels
    for (int i = 5; i < 8; ++i) {
        TEST_ASSERT_EQUAL_UINT16(1024, decCh[i]);
    }
}

static void test_stubborn_telemetry_reliability() {
    StubbornSender sender;
    StubbornReceiver receiver;

    const char* message = "Flight telemetry packet test!";
    size_t msgLen = strlen(message); // 29 bytes

    sender.queuePayload((const uint8_t*)message, msgLen);

    uint8_t steps = 0;
    // LCG pseudo-random generator to simulate packet drop deterministically
    uint32_t seed = 0x5678;

    while (!sender.idle() && steps < 200) {
        steps++;
        TelemetryChunk chunk;
        if (sender.getNextChunk(chunk)) {
            // Simulate 35% forward loss
            seed = seed * 1664525u + 1013904223u;
            if ((seed % 100) < 35) {
                // Drop forward packet
                continue;
            }

            uint8_t ackSeq = 0;
            bool accepted = receiver.processChunk(chunk, ackSeq);
            (void)accepted;

            // Simulate 35% reverse loss (ACK drop)
            seed = seed * 1664525u + 1013904223u;
            if ((seed % 100) < 35) {
                // Drop reverse ACK packet
                continue;
            }

            sender.receiveAck(ackSeq);
        }
    }

    TEST_ASSERT_TRUE(sender.idle());
    TEST_ASSERT_TRUE(receiver.ready());

    uint8_t decBuf[128] = {0};
    size_t decLen = 0;
    bool status = receiver.getPayload(decBuf, decLen);

    TEST_ASSERT_TRUE(status);
    TEST_ASSERT_EQUAL_UINT32(msgLen, decLen);
    TEST_ASSERT_EQUAL_STRING_LEN(message, (char*)decBuf, msgLen);
}

// ---- PFD: a one-time phase step settles back to ~0 ----
static void test_pfd_step_settles() {
    Pfd pfd; pfd.begin(4000);
    int32_t offset = 900;                       // step, no persistent drift
    for (int i = 0; i < 1000; ++i) offset -= pfd.update(offset);
    TEST_ASSERT_TRUE(offset > -10 && offset < 10);
}

// ---- PFD: PI loop nulls steady-state error under constant crystal drift ----
static void test_pfd_nulls_drift() {
    Pfd pfd; pfd.begin(4000);
    int32_t offset = 0;
    const int32_t drift = 3;                    // us added every interval
    for (int i = 0; i < 4000; ++i) {
        int32_t c = pfd.update(offset);
        offset += drift - c;                    // disturbance - correction
    }
    TEST_ASSERT_TRUE(offset > -5 && offset < 5);
}

// ---- LqTracker: 3-of-4 received => 75% ----
static void test_lq_tracker() {
    LqTracker<4> lq;
    for (int i = 0; i < 4; ++i) lq.update((i % 4) != 3);
    TEST_ASSERT_EQUAL_UINT8(75, lq.lq());
}

// ---- SPSC ring (events): FIFO order + full/empty ----
static void test_spsc_ring() {
    SpscRing<int, 4> r;                 // 3 usable slots
    TEST_ASSERT_TRUE(r.push(1));
    TEST_ASSERT_TRUE(r.push(2));
    TEST_ASSERT_TRUE(r.push(3));
    TEST_ASSERT_FALSE(r.push(4));       // full
    int v;
    TEST_ASSERT_TRUE(r.pop(v)); TEST_ASSERT_EQUAL_INT(1, v);
    TEST_ASSERT_TRUE(r.pop(v)); TEST_ASSERT_EQUAL_INT(2, v);
    TEST_ASSERT_TRUE(r.push(4));
    TEST_ASSERT_TRUE(r.pop(v)); TEST_ASSERT_EQUAL_INT(3, v);
    TEST_ASSERT_TRUE(r.pop(v)); TEST_ASSERT_EQUAL_INT(4, v);
    TEST_ASSERT_FALSE(r.pop(v));        // empty
}

// ---- Mailbox (channels/stats): freshest wins, no backlog ----
static void test_latest_value_freshest_wins() {
    LatestValue<int> mb;
    int v = -1;
    TEST_ASSERT_TRUE(mb.load(v)); TEST_ASSERT_EQUAL_INT(0, v); // default
    mb.store(5);
    mb.store(7);                                               // overwrites; no queue
    TEST_ASSERT_TRUE(mb.load(v)); TEST_ASSERT_EQUAL_INT(7, v);
    TEST_ASSERT_TRUE(mb.load(v)); TEST_ASSERT_EQUAL_INT(7, v); // load is non-destructive
}

// ---- Mailbox loadOrKeep: refresh on success (last-good fallback is the false branch) ----
static void test_latest_value_load_or_keep() {
    LatestValue<int> mb;
    int local = 42;
    TEST_ASSERT_TRUE(mb.loadOrKeep(local));     // fresh mailbox reads default
    TEST_ASSERT_EQUAL_INT(0, local);
    mb.store(7);
    TEST_ASSERT_TRUE(mb.loadOrKeep(local));     // refreshed to newest
    TEST_ASSERT_EQUAL_INT(7, local);
}

// ---- RfScheduler slot selection: precedence Sync > Telemetry > Uplink ----
static void test_slot_for_tick() {
    const uint8_t tlm = 4;      // telemetry 1:4
    const uint16_t sync = 8;    // sync every 8 ticks
    TEST_ASSERT_EQUAL(Slot::Sync,      slotForTick(0,  tlm, sync)); // 0%8==0 & 0%4==0 -> Sync wins
    TEST_ASSERT_EQUAL(Slot::Uplink,    slotForTick(1,  tlm, sync));
    TEST_ASSERT_EQUAL(Slot::Telemetry, slotForTick(4,  tlm, sync)); // 4%4==0, 4%8!=0
    TEST_ASSERT_EQUAL(Slot::Uplink,    slotForTick(5,  tlm, sync));
    TEST_ASSERT_EQUAL(Slot::Sync,      slotForTick(8,  tlm, sync));
    TEST_ASSERT_EQUAL(Slot::Telemetry, slotForTick(12, tlm, sync));
}

// ---- Option B Decoupled Sim Environment ----
struct SimEnvironment {
    MockPhy txPhy;
    MockPhy rxPhy;
    Link tx;
    Link rx;
    RfScheduler txSched;
    RfScheduler rxSched;

    void setup(const uint8_t uid[8], uint8_t rateIndex) {
        PhyConfig cfg{}; cfg.freqMHz = 2420.0f; cfg.payloadLen = 8;
        txPhy.init(cfg);
        rxPhy.init(cfg);
        MockPhy::connect(txPhy, rxPhy);

        tx.begin(Role::Tx, uid, rateIndex);
        rx.begin(Role::Rx, uid, rateIndex);

        txSched.begin(&txPhy, nullptr, &tx, rateIndex);
        rxSched.begin(&rxPhy, nullptr, &rx, rateIndex);
    }
};

static RfScheduler* g_txSched = nullptr;
static RfScheduler* g_rxSched = nullptr;

static void onTxRxDone() { if (g_txSched) g_txSched->onRxDone(); }
static void onTxTxDone() { if (g_txSched) g_txSched->onTxDone(); }
static void onRxRxDone() { if (g_rxSched) g_rxSched->onRxDone(); }
static void onRxTxDone() { if (g_rxSched) g_rxSched->onTxDone(); }

static void simTick(SimEnvironment& env, uint32_t t, bool txOn = true) {
    // Dynamically update the global active scheduler pointers for this simulation context.
    g_txSched = &env.txSched;
    g_rxSched = &env.rxSched;

    env.txPhy.setOnRxDone(onTxRxDone);
    env.txPhy.setOnTxDone(onTxTxDone);
    env.rxPhy.setOnRxDone(onRxRxDone);
    env.rxPhy.setOnTxDone(onRxTxDone);

    Slot s = slotForTick(t, env.tx.tlmRatioDenom(), env.tx.syncEveryNTicks());
    if (s == Slot::Telemetry) {
        // Downlink Telemetry: TX role is receiving (RX), RX role is transmitting (TX)
        if (txOn) env.txSched.onTick(t);
        env.rxSched.onTick(t);
    } else {
        // Uplink/Sync: TX role is transmitting (TX), RX role is receiving (RX)
        env.rxSched.onTick(t);
        if (txOn) env.txSched.onTick(t);
    }
    
    if (txOn) env.tx.service(t);
    env.rx.service(t);
}

// ---- M3: two-node sim — connect, channel flow, induced loss → NoPulses failsafe ----
static void test_link_connect_flow_failsafe() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);

    SimEnvironment env;
    env.setup(uid, 2);

    uint16_t ch[4] = {100, 1000, 2000, 1024};
    env.tx.setChannels(ch, 4);

    for (uint32_t t = 1; t <= 20; ++t) simTick(env, t);   // steady link
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);
    TEST_ASSERT_TRUE(env.rx.outputActive());
    uint16_t out[4];
    TEST_ASSERT_EQUAL_UINT8(4, env.rx.getChannels(out, 4));
    for (int i = 0; i < 4; ++i) TEST_ASSERT_EQUAL_UINT16(ch[i], out[i]);   // channels propagate
    TEST_ASSERT_EQUAL_UINT8(100, env.rx.stats().lqUp);

    for (uint32_t t = 21; t <= 40; ++t) simTick(env, t, /*txOn=*/false);  // TX silent → loss
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Failsafe);
    TEST_ASSERT_FALSE(env.rx.outputActive());          // NoPulses: CRSF stops → FC failsafe
    TEST_ASSERT_TRUE(env.rx.stats().lqUp < 100);       // LQ degraded
}

// ---- M3: mismatched binding phrase never connects (sync-word filtering) ----
static void test_link_mismatched_phrase_no_connect() {
    uint8_t uidA[LINK_UID_SIZE], uidB[LINK_UID_SIZE];
    linkUidFromPhrase("CraftA", uidA);
    linkUidFromPhrase("CraftB", uidB);

    SimEnvironment env;
    PhyConfig cfg{}; cfg.freqMHz = 2420.0f; cfg.payloadLen = 8;
    env.txPhy.init(cfg); env.rxPhy.init(cfg);
    MockPhy::connect(env.txPhy, env.rxPhy);

    env.tx.begin(Role::Tx, uidA, 2);
    env.rx.begin(Role::Rx, uidB, 2);

    env.txSched.begin(&env.txPhy, nullptr, &env.tx, 2);
    env.rxSched.begin(&env.rxPhy, nullptr, &env.rx, 2);

    uint16_t ch[4] = {1, 2, 3, 4}; env.tx.setChannels(ch, 4);
    for (uint32_t t = 1; t <= 30; ++t) simTick(env, t);

    TEST_ASSERT_TRUE(env.rx.state() != LinkState::Connected);   // PHY sync-word filter blocks it
    TEST_ASSERT_FALSE(env.rx.outputActive());
}

// ---- M4: FHSS — cold-start acquisition via Sync beacon, hop tracking, loss, re-acquire ----
static void test_link_fhss_acquire_and_recover() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);

    SimEnvironment env;
    env.setup(uid, 2);

    uint16_t ch[4] = {200, 1500, 2047, 0};
    env.tx.setChannels(ch, 4);

    // Acquire (RX dwells on the acq channel until it catches a Sync) + settle.
    for (uint32_t t = 1; t <= 300; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);   // hopped together after acquire
    TEST_ASSERT_TRUE(env.rx.outputActive());
    uint16_t out[4];
    env.rx.getChannels(out, 4);
    for (int i = 0; i < 4; ++i) TEST_ASSERT_EQUAL_UINT16(ch[i], out[i]);  // RC flows while hopping
    TEST_ASSERT_TRUE(env.rx.stats().lqUp >= 90);                // packets land nearly every slot

    // Loss: TX silent → RX loses lock → Failsafe (NoPulses).
    for (uint32_t t = 301; t <= 340; ++t) simTick(env, t, /*txOn=*/false);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Failsafe);
    TEST_ASSERT_FALSE(env.rx.outputActive());

    // Recover: TX returns → RX must RE-ACQUIRE via a Sync on the acq channel.
    for (uint32_t t = 341; t <= 650; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);
    TEST_ASSERT_TRUE(env.rx.outputActive());
}

// ---- M5: deterministic telemetry slot — downlink reaches TX, no uplink collision ----
static void test_link_telemetry_downlink() {
    uint8_t uid[LINK_UID_SIZE]; linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);
    env.txPhy.setRssi(-72);                       // RX hears the TX's uplink at -72 dBm

    uint16_t ch[4] = {300, 1500, 2047, 0}; env.tx.setChannels(ch, 4);

    for (uint32_t t = 1; t <= 600; ++t) simTick(env, t);

    // Uplink stayed healthy — telemetry slots did NOT collide with RC:
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);
    TEST_ASSERT_TRUE(env.rx.outputActive());
    uint16_t out[4]; env.rx.getChannels(out, 4);
    for (int i = 0; i < 4; ++i) TEST_ASSERT_EQUAL_UINT16(ch[i], out[i]);

    // Downlink reached the TX: it heard the RX telemetry and learned the RX-side uplink view.
    TEST_ASSERT_TRUE(env.tx.stats().lqDown > 0);
    TEST_ASSERT_EQUAL_INT16(-72, env.tx.stats().rssiDbm);   // RX-reported uplink RSSI
    TEST_ASSERT_TRUE(env.tx.stats().lqUp >= 90);            // RX-reported uplink LQ
}

// ---- M5/M9: scheduler detects PHY fault and recovers without waiting for traffic ----
static void test_scheduler_phy_recovery() {
    uint8_t uid[LINK_UID_SIZE]; linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);

    TEST_ASSERT_TRUE(env.rxPhy.healthy());
    env.rxPhy.forceFault();
    TEST_ASSERT_FALSE(env.rxPhy.healthy());

    env.rxSched.poll();

    TEST_ASSERT_TRUE(env.rxPhy.healthy());
    TEST_ASSERT_EQUAL_UINT16(1, env.rx.stats().phyRecoveries);
    TEST_ASSERT_EQUAL_UINT16(0, env.rx.stats().phyRecoveryFailures);
}

// ---- M5: CRSF LINK_STATISTICS packing (what the RX emits to the FC) ----
static void test_crsf_link_statistics() {
    LinkStats s{};
    s.lqUp = 95; s.lqDown = 80; s.rssiDbm = -72; s.snr = 7; s.rateIndex = 2;
    uint8_t f[10];
    buildCrsfLinkStatistics(s, f);
    TEST_ASSERT_EQUAL_UINT8(72, f[0]);   // uplink_RSSI_1 (-dBm magnitude)
    TEST_ASSERT_EQUAL_UINT8(95, f[2]);   // uplink_LQ
    TEST_ASSERT_EQUAL_UINT8(7,  f[3]);   // uplink_SNR
    TEST_ASSERT_EQUAL_UINT8(2,  f[5]);   // rf_mode (rateIndex)
    TEST_ASSERT_EQUAL_UINT8(80, f[8]);   // downlink_LQ
}

// ---- M6: dynamic power policy (LQ-first, RSSI-second, hysteresis) ----
static void test_dynamic_power() {
    DynamicPower p; p.begin(0, 30, 15);
    for (int i = 0; i < 30; ++i) p.update(50, -90);     // weak LQ → raise to max
    TEST_ASSERT_EQUAL_INT8(30, p.powerDbm());
    for (int i = 0; i < 30; ++i) p.update(100, -55);    // strong LQ + margin → lower to min
    TEST_ASSERT_EQUAL_INT8(0, p.powerDbm());
    int8_t held = p.powerDbm();
    for (int i = 0; i < 30; ++i) p.update(100, -80);    // high LQ but weak RSSI → deadband, hold
    TEST_ASSERT_EQUAL_INT8(held, p.powerDbm());
}

// ---- M6: TX-initiated rate switch propagates to RX; dynamic power reacts to a strong link ----
static void test_link_rate_switch_and_power() {
    uint8_t uid[LINK_UID_SIZE]; linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);
    env.txPhy.setRssi(-55);                       // strong uplink → power should step down

    uint16_t ch[4] = {300, 1500, 2047, 0}; env.tx.setChannels(ch, 4);

    for (uint32_t t = 1; t <= 800; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);
    TEST_ASSERT_EQUAL_UINT8(2, env.rx.stats().rateIndex);
    TEST_ASSERT_TRUE(env.txPhy.outputPowerDbm() < 10);        // dynamic power dropped under strong link

    env.tx.requestRate(4);                        // switch to L50 (long range)
    for (uint32_t t = 801; t <= 1600; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected); // survived the switch
    TEST_ASSERT_EQUAL_UINT8(4, env.rx.stats().rateIndex);     // RX adopted the new rate via Sync
}

static void test_link_stubborn_telemetry_integrated() {
    uint8_t uid[LINK_UID_SIZE]; linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2); // F1000

    // Connect link first
    for (uint32_t t = 1; t <= 100; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);

    // Queue stubborn telemetry payload on RX side (representing MSP/MAVLink telemetry packet)
    const char* tlmData = "MSP config ok";
    size_t tlmLen = strlen(tlmData);
    env.rx.queueTelemetry((const uint8_t*)tlmData, tlmLen);

    // Run simulator loop to allow packetization, transmission, and ACK propagation
    for (uint32_t t = 101; t <= 1200; ++t) {
        simTick(env, t);
    }

    // Verify TX side successfully received and reassembled the payload
    uint8_t rcvBuf[128] = {0};
    size_t rcvLen = 0;
    bool received = env.tx.getTelemetry(rcvBuf, rcvLen);

    TEST_ASSERT_TRUE(received);
    TEST_ASSERT_EQUAL_UINT32(tlmLen, rcvLen);
    TEST_ASSERT_EQUAL_STRING_LEN(tlmData, (char*)rcvBuf, tlmLen);
}

static void test_msp_frames_count_toward_lq() {
    uint8_t uid[LINK_UID_SIZE]; linkUidFromPhrase("Kikobot-02", uid);
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

// ---- M8: ChaCha20-Poly1305 against the RFC 8439 §2.8.2 known-answer vector ----
static void test_chacha20poly1305_rfc8439() {
    uint8_t key[32];   for (int i = 0; i < 32; ++i) key[i] = (uint8_t)(0x80 + i);
    uint8_t nonce[12] = {0x07,0,0,0, 0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47};
    uint8_t aad[12]   = {0x50,0x51,0x52,0x53,0xc0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7};
    const char* pt = "Ladies and Gentlemen of the class of '99: If I could offer you only "
                     "one tip for the future, sunscreen would be it.";
    size_t len = strlen(pt);                          // 114
    uint8_t buf[128]; memcpy(buf, pt, len);
    uint8_t tag[16];
    cc20p1305::seal(key, nonce, aad, 12, buf, len, tag);
    static const uint8_t expect_tag[16] =
        {0x1a,0xe1,0x0b,0x59,0x4f,0x09,0xe2,0x6a,0x7e,0x90,0x2e,0xcb,0xd0,0x60,0x06,0x91};
    TEST_ASSERT_EQUAL_MEMORY(expect_tag, tag, 16);    // exact RFC tag ⇒ chacha+poly+AEAD correct
}

// ---- M8: AeadCipher (truncated-tag) round-trip, tamper rejection, wrong-key rejection ----
static void test_aead_cipher() {
    uint8_t key[32]; for (int i = 0; i < 32; ++i) key[i] = (uint8_t)i;
    AeadCipher c; c.setKey(key);
    Nonce96 n = Nonce96::build(0x11223344u, 5, 7);

    uint8_t plain[6] = {1, 2, 3, 4, 5, 6};
    uint8_t buf[16];  memcpy(buf, plain, 6);
    c.seal(buf, 6, n);                                // buf[0..5]=ciphertext, buf[6..9]=tag
    bool changed = false; for (int i = 0; i < 6; ++i) if (buf[i] != plain[i]) changed = true;
    TEST_ASSERT_TRUE(changed);                        // actually encrypted

    AeadCipher rx; rx.setKey(key);
    uint8_t good[16]; memcpy(good, buf, 10);
    TEST_ASSERT_TRUE(rx.open(good, 6, n));            // round-trip
    TEST_ASSERT_EQUAL_MEMORY(plain, good, 6);

    uint8_t bad[16]; memcpy(bad, buf, 10); bad[0] ^= 0x01;
    AeadCipher rx2; rx2.setKey(key);
    TEST_ASSERT_FALSE(rx2.open(bad, 6, n));           // tamper rejected

    uint8_t wrongkey[32]; for (int i = 0; i < 32; ++i) wrongkey[i] = (uint8_t)(i + 1);
    AeadCipher rx3; rx3.setKey(wrongkey);
    uint8_t buf3[16]; memcpy(buf3, buf, 10);
    TEST_ASSERT_FALSE(rx3.open(buf3, 6, n));          // wrong key rejected
}

// ---- M8: encrypted link — AEAD-sealed RC round-trips end-to-end through the sim ----
static void test_link_encrypted() {
    uint8_t uid[LINK_UID_SIZE]; linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);

    uint8_t key[32]; for (int i = 0; i < 32; ++i) key[i] = (uint8_t)(0x42 ^ i);
    AeadCipher txC, rxC; txC.setKey(key); rxC.setKey(key);

    env.tx.setCipher(&txC); env.rx.setCipher(&rxC);
    uint16_t ch[4] = {222, 1444, 1999, 55}; env.tx.setChannels(ch, 4);

    for (uint32_t t = 1; t <= 300; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);    // sealed RC opens + decodes
    uint16_t out[4]; env.rx.getChannels(out, 4);
    for (int i = 0; i < 4; ++i) TEST_ASSERT_EQUAL_UINT16(ch[i], out[i]);
    TEST_ASSERT_TRUE(env.rx.stats().lqUp >= 90);
}

// ---- M8: wrong key → RC auth fails → channels never decode (control rejected) ----
static void test_link_wrong_key() {
    uint8_t uid[LINK_UID_SIZE]; linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);

    uint8_t k1[32], k2[32];
    for (int i = 0; i < 32; ++i) { k1[i] = (uint8_t)i; k2[i] = (uint8_t)(i + 1); }
    AeadCipher txC, rxC; txC.setKey(k1); rxC.setKey(k2);     // mismatched keys

    env.tx.setCipher(&txC); env.rx.setCipher(&rxC);
    uint16_t ch[4] = {200, 200, 200, 200}; env.tx.setChannels(ch, 4);

    for (uint32_t t = 1; t <= 300; ++t) simTick(env, t);
    uint16_t out[4]; env.rx.getChannels(out, 4);
    TEST_ASSERT_EQUAL_UINT16(1024, out[0]);                  // RC auth failed → still default
    TEST_ASSERT_TRUE(env.rx.stats().lqUp < 10);                  // no valid uplink decoded
}

// ---- M8: Flash wear-leveled boot counter test ----
static void test_flash_boot_counter() {
    FlashBootCounter::resetSim();
    TEST_ASSERT_EQUAL_UINT32(0, FlashBootCounter::read());
    TEST_ASSERT_EQUAL_UINT32(1, FlashBootCounter::increment());
    TEST_ASSERT_EQUAL_UINT32(1, FlashBootCounter::read());
    TEST_ASSERT_EQUAL_UINT32(2, FlashBootCounter::increment());
    TEST_ASSERT_EQUAL_UINT32(2, FlashBootCounter::read());
}

// ---- M2 Timing: scheduler PFD locks simulated timer cadence under drift ----
static void test_scheduler_timing_pfd_lock() {
    uint8_t uid[LINK_UID_SIZE];
    linkUidFromPhrase("Kikobot-02", uid);

    SimEnvironment env;
    env.setup(uid, 2); // D250: intervalUs = 4000

    uint32_t txClock = 100000;
    uint32_t rxClock = 100000;
    setSimulatedTimeUs(rxClock);

    // Run connection phase
    for (uint32_t t = 1; t <= 50; ++t) {
        txClock += 4000;
        rxClock += getSimulatedIntervalUs();
        
        env.txPhy.setClockUs(txClock);
        setSimulatedTimeUs(rxClock);

        simTick(env, t);
    }
    
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);

    // Introduce +12 us constant drift on the TX side
    for (uint32_t t = 51; t <= 450; ++t) {
        txClock += 4012; // 4000 us interval + 12 us drift per tick
        rxClock += getSimulatedIntervalUs();

        env.txPhy.setClockUs(txClock);
        setSimulatedTimeUs(rxClock);

        simTick(env, t);
    }

    // Verify PFD integral/proportional control locked phase and set interval to ~4012
    uint32_t finalInterval = getSimulatedIntervalUs();
    TEST_ASSERT_INT_WITHIN(2, 4012, finalInterval);
}

// ---- M9: poll() drains hardware-timer tick events into onTick()+service() (the on-hardware path) ----
static void test_scheduler_poll_drains_ticks() {
    uint8_t uid[LINK_UID_SIZE]; linkUidFromPhrase("Kikobot-02", uid);
    MockPhy phy; PhyConfig cfg{}; cfg.freqMHz = 2420.0f; phy.init(cfg);
    Link link; link.begin(Role::Tx, uid, 2);
    RfScheduler sched; sched.begin(&phy, nullptr, &link, 2);

    for (int i = 0; i < 5; ++i) fireSimTimerTick();   // 5 hardware timer ISRs (tiny: counter++ only)
    TEST_ASSERT_EQUAL_UINT32(5, sched.tickEvents());
    TEST_ASSERT_EQUAL_UINT32(0, sched.processedTick());  // not yet drained

    sched.poll();                                     // drains 5 events → 5× (onTick + service)
    TEST_ASSERT_EQUAL_UINT32(5, sched.processedTick());

    sched.poll();                                     // idempotent: nothing new to drain
    TEST_ASSERT_EQUAL_UINT32(5, sched.processedTick());
}

// ---- M9: a huge tick backlog (core-1 stall) fast-forwards instead of replaying stale slots ----
static void test_scheduler_poll_caps_backlog() {
    uint8_t uid[LINK_UID_SIZE]; linkUidFromPhrase("Kikobot-02", uid);
    MockPhy phy; PhyConfig cfg{}; cfg.freqMHz = 2420.0f; phy.init(cfg);
    Link link; link.begin(Role::Tx, uid, 2);
    RfScheduler sched; sched.begin(&phy, nullptr, &link, 2);

    const uint32_t backlog = RfScheduler::MAX_TICK_CATCHUP + 20;   // well past the cap
    for (uint32_t i = 0; i < backlog; ++i) fireSimTimerTick();
    TEST_ASSERT_EQUAL_UINT32(backlog, sched.tickEvents());

    sched.poll();                                          // does NOT replay all `backlog` slots
    TEST_ASSERT_EQUAL_UINT32(backlog, sched.processedTick());        // jumped straight to latest
    TEST_ASSERT_TRUE(link.stats().missedDeadlines >= backlog);      // overrun recorded for diag

    sched.poll();                                          // idempotent once caught up
    TEST_ASSERT_EQUAL_UINT32(backlog, sched.processedTick());
}

// ---- Soak: link stays locked and accurate over thousands of healthy slots ----
static void test_link_long_soak() {
    uint8_t uid[LINK_UID_SIZE]; linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);

    uint16_t ch[4] = {250, 1250, 1750, 800}; env.tx.setChannels(ch, 4);

    for (uint32_t t = 1; t <= 300; ++t) simTick(env, t);          // acquire
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);

    for (uint32_t t = 301; t <= 8000; ++t) simTick(env, t);       // long healthy soak

    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);     // never spuriously dropped
    TEST_ASSERT_TRUE(env.rx.outputActive());
    TEST_ASSERT_TRUE(env.rx.stats().lqUp >= 95);                  // sustained high LQ
    TEST_ASSERT_EQUAL_UINT16(0, env.rx.stats().rxQueueDrops);     // no overflow over the soak
    uint16_t out[4]; env.rx.getChannels(out, 4);
    for (int i = 0; i < 4; ++i) TEST_ASSERT_EQUAL_UINT16(ch[i], out[i]);   // channels still exact
}

// ---- Corrupt uplink RC frames are rejected (CRC/AEAD): no garbage out, LQ reflects loss ----
static void test_link_corrupt_rc_rejected() {
    uint8_t uid[LINK_UID_SIZE]; linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);

    uint16_t ch[4] = {400, 1600, 2000, 100}; env.tx.setChannels(ch, 4);
    for (uint32_t t = 1; t <= 200; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);
    TEST_ASSERT_TRUE(env.rx.stats().lqUp >= 90);

    uint16_t before[4]; env.rx.getChannels(before, 4);
    for (int i = 0; i < 4; ++i) TEST_ASSERT_EQUAL_UINT16(ch[i], before[i]);   // last known-good

    // Flip a byte on every uplink frame the RX hears for a stretch.
    env.rxPhy.corruptNextDeliveries(1000000);
    for (uint32_t t = 201; t <= 260; ++t) simTick(env, t);

    // Corrupt frames are treated as loss, never accepted: the link drops to failsafe and
    // stops emitting, and the RC channels are never overwritten with garbage.
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Failsafe);
    TEST_ASSERT_FALSE(env.rx.outputActive());
    uint16_t during[4]; env.rx.getChannels(during, 4);
    for (int i = 0; i < 4; ++i) TEST_ASSERT_EQUAL_UINT16(ch[i], during[i]);   // last good held, no garbage

    // Stop corrupting → the link re-acquires and LQ recovers.
    env.rxPhy.corruptNextDeliveries(0);
    for (uint32_t t = 261; t <= 700; ++t) simTick(env, t);
    TEST_ASSERT_TRUE(env.rx.state() == LinkState::Connected);
    TEST_ASSERT_TRUE(env.rx.stats().lqUp >= 90);
}

// ---- M9: PHY recovery that fails several times is counted, then eventually succeeds ----
static void test_scheduler_phy_recovery_failure() {
    uint8_t uid[LINK_UID_SIZE]; linkUidFromPhrase("Kikobot-02", uid);
    SimEnvironment env;
    env.setup(uid, 2);

    env.rxPhy.setRecoveryFailures(3);    // recover() fails 3× before it takes
    env.rxPhy.forceFault();
    TEST_ASSERT_FALSE(env.rxPhy.healthy());

    for (int i = 0; i < 6 && !env.rxPhy.healthy(); ++i) env.rxSched.poll();

    TEST_ASSERT_TRUE(env.rxPhy.healthy());                       // recovered after retries
    TEST_ASSERT_EQUAL_UINT16(3, env.rx.stats().phyRecoveryFailures);
    TEST_ASSERT_EQUAL_UINT16(1, env.rx.stats().phyRecoveries);
}

void setUp() {}
void tearDown() {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_channel_pack_roundtrip);
    RUN_TEST(test_fhss_deterministic_and_balanced);
    RUN_TEST(test_fhss_80_channel_balance);
    RUN_TEST(test_ota_8_byte_packing);
    RUN_TEST(test_stubborn_telemetry_reliability);
    RUN_TEST(test_link_stubborn_telemetry_integrated);
    RUN_TEST(test_msp_frames_count_toward_lq);
    RUN_TEST(test_pfd_step_settles);
    RUN_TEST(test_pfd_nulls_drift);
    RUN_TEST(test_lq_tracker);
    RUN_TEST(test_spsc_ring);
    RUN_TEST(test_latest_value_freshest_wins);
    RUN_TEST(test_latest_value_load_or_keep);
    RUN_TEST(test_slot_for_tick);
    RUN_TEST(test_mockphy_loopback);
    RUN_TEST(test_link_uid_from_phrase);
    RUN_TEST(test_uid_drives_shared_fhss);
    RUN_TEST(test_link_connect_flow_failsafe);
    RUN_TEST(test_link_mismatched_phrase_no_connect);
    RUN_TEST(test_link_fhss_acquire_and_recover);
    RUN_TEST(test_link_telemetry_downlink);
    RUN_TEST(test_scheduler_phy_recovery);
    RUN_TEST(test_crsf_link_statistics);
    RUN_TEST(test_dynamic_power);
    RUN_TEST(test_link_rate_switch_and_power);
    RUN_TEST(test_chacha20poly1305_rfc8439);
    RUN_TEST(test_aead_cipher);
    RUN_TEST(test_link_encrypted);
    RUN_TEST(test_link_wrong_key);
    RUN_TEST(test_flash_boot_counter);
    RUN_TEST(test_scheduler_timing_pfd_lock);
    RUN_TEST(test_scheduler_poll_drains_ticks);
    RUN_TEST(test_scheduler_poll_caps_backlog);
    RUN_TEST(test_link_long_soak);
    RUN_TEST(test_link_corrupt_rc_rejected);
    RUN_TEST(test_scheduler_phy_recovery_failure);
    return UNITY_END();
}
