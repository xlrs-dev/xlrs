#include <unity.h>

#include <string.h>

#include "lora_link/control/frame.h"
#include "lora_link/protocol.h"
#include "lora_link/rf/scheduler.h"
#include "rc_handset/input/InputPipeline.h"
#include "rc_handset/core1/Snapshot.h"
#include "rc_handset/config/config.h"
#include "rc_handset/migration/LegacyCalibrationMigration.h"
#include "rc_handset/telemetry/HandsetTelemetry.h"
#include "rc_handset/display/DisplayState.h"
#include "rc_handset/power/Battery.h"
#include "rc_handset/power/Bq2562x.h"
#include "rc_handset/power/PowerButton.h"

using namespace lora_link;
using namespace rc_handset::input;
namespace control = lora_link::control;
namespace link_rf = lora_link::rf;
namespace handset_config = rc_handset::config;
namespace handset_core1 = rc_handset::core1;
namespace handset_migration = rc_handset::migration;
namespace handset_telemetry = rc_handset::telemetry;
namespace handset_display = rc_handset::display;
namespace handset_power = rc_handset::power;

static void writeLe16(uint8_t* storage, size_t offset, uint16_t value) {
    storage[offset] = static_cast<uint8_t>(value & 0xFFu);
    storage[offset + 1] = static_cast<uint8_t>(value >> 8);
}

static void writeLegacyCalibration(uint8_t* storage,
                                   const uint16_t min[handset_config::kRcHandsetAxisCount],
                                   const uint16_t center[handset_config::kRcHandsetAxisCount],
                                   const uint16_t max[handset_config::kRcHandsetAxisCount]) {
    storage[handset_migration::kLegacyCalibrationMagicAddr] = handset_migration::kLegacyCalibrationMagicValue;
    size_t offset = handset_migration::kLegacyCalibrationDataAddr;
    for (uint8_t axis = 0; axis < handset_config::kRcHandsetAxisCount; ++axis) {
        writeLe16(storage, offset, min[axis]);
        offset += sizeof(uint16_t);
    }
    for (uint8_t axis = 0; axis < handset_config::kRcHandsetAxisCount; ++axis) {
        writeLe16(storage, offset, max[axis]);
        offset += sizeof(uint16_t);
    }
    for (uint8_t axis = 0; axis < handset_config::kRcHandsetAxisCount; ++axis) {
        writeLe16(storage, offset, center[axis]);
        offset += sizeof(uint16_t);
    }
}

void run_rc_usb_protocol_tests();
void run_power_tests();

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
    TEST_ASSERT_NOT_EQUAL_UINT8(0, syncWordFromUid(a));
}

static void test_sync_word_is_eight_bit_and_guards_effective_zero() {
    uint8_t uid[kUidSize] = {0x42, 0x99, 0x18, 0x23, 0x54, 0x65, 0x00, 0x76};
    TEST_ASSERT_NOT_EQUAL_UINT8(0, syncWordFromUid(uid));
    uid[1] ^= 0x5A;
    TEST_ASSERT_NOT_EQUAL_UINT8(0, syncWordFromUid(uid));
}

static void test_uid_check_uses_all_uid_bytes() {
    uint8_t uid[kUidSize];
    deriveUid("identity", uid);
    const uint32_t original = uidCheck(uid);
    for (uint8_t i = 0; i < kUidSize; ++i) {
        uint8_t changed[kUidSize];
        memcpy(changed, uid, sizeof(changed));
        changed[i] ^= 0x5Au;
        TEST_ASSERT_NOT_EQUAL_UINT32(original, uidCheck(changed));
    }
}

static void test_crc_known_answer_vectors() {
    const uint8_t value[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    TEST_ASSERT_EQUAL_UINT16(0x29B1, crc16Ccitt(value, sizeof(value)));
    TEST_ASSERT_EQUAL_UINT8(0xBC, crsfCrc8(value, sizeof(value)));
}

static void test_fhss_depends_on_uid_and_permutates_channels() {
    uint8_t a[kUidSize];
    uint8_t b[kUidSize];
    deriveUid("alpha", a);
    deriveUid("bravo", b);
    bool differs = false;
    bool seen[kFhssChannelCount] = {};
    uint8_t previous = 0xFF;
    for (uint16_t hop = 0; hop < kFhssChannelCount; ++hop) {
        const uint8_t channel = fhssChannelFor(a, hop);
        TEST_ASSERT_LESS_THAN_UINT8(kFhssChannelCount, channel);
        TEST_ASSERT_FALSE(seen[channel]);
        seen[channel] = true;
        if (hop > 0) TEST_ASSERT_NOT_EQUAL_UINT8(previous, channel);
        previous = channel;
        if (fhssChannelFor(a, hop) != fhssChannelFor(b, hop)) differs = true;
    }
    TEST_ASSERT_NOT_EQUAL_UINT8(previous, fhssChannelFor(a, kFhssChannelCount));
    TEST_ASSERT_TRUE(differs);
}

static void test_fhss_frequency_uses_named_channel_count() {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2404.0f, fhssFrequencyMHz(0));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2482.0f, fhssFrequencyMHz(kFhssChannelCount - 1));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2404.0f, fhssFrequencyMHz(kFhssChannelCount));
}

static void test_fhss_is_enabled_by_default() {
    TEST_ASSERT_LESS_THAN_INT8(0, kFixedRfChannel);
}

static void test_hop_alignment_tracks_received_sequence_for_reacquisition() {
    TEST_ASSERT_EQUAL_UINT16(0, hopForSequence(0, 16));
    TEST_ASSERT_EQUAL_UINT16(2, nextHopAfterReceivedSequence(0, 16));
    TEST_ASSERT_EQUAL_UINT16(16, hopForSequence(15, 16));
    TEST_ASSERT_EQUAL_UINT16(17, hopForSequence(16, 16));
    TEST_ASSERT_EQUAL_UINT16(19, nextHopAfterReceivedSequence(16, 16));
    TEST_ASSERT_EQUAL_UINT16(4123, hopForSequence(4123, 0));
    TEST_ASSERT_EQUAL_UINT16(4124, nextHopAfterReceivedSequence(4123, 0));
}

static void test_rf_scheduler_tx_advances_nonce_and_fhss_on_tx_done() {
    uint8_t uid[kUidSize];
    deriveUid("scheduler-tx", uid);
    link_rf::TxScheduler scheduler;
    scheduler.begin(uid, kRates[0]);
    for (uint8_t i = 0; i < kRates[0].fhssHopInterval; ++i) {
        TEST_ASSERT_TRUE(scheduler.onTimerTick());
        TEST_ASSERT_EQUAL_UINT16(i, scheduler.sequence());
        scheduler.onTxDone();
    }
    TEST_ASSERT_EQUAL_UINT16(kRates[0].fhssHopInterval, scheduler.sequence());
    TEST_ASSERT_EQUAL_UINT16(1, scheduler.fhss().index());
}

static void test_rf_scheduler_tx_consumes_telemetry_slot_without_rc_frame() {
    uint8_t uid[kUidSize];
    deriveUid("scheduler-tlm", uid);
    RateConfig telemetryRate = kRates[0];
    telemetryRate.telemetryRatio = 4;
    link_rf::TxScheduler scheduler;
    scheduler.begin(uid, telemetryRate);
    for (uint8_t i = 0; i < telemetryRate.telemetryRatio; ++i) {
        TEST_ASSERT_TRUE(scheduler.onTimerTick());
        scheduler.onTxDone();
    }
    TEST_ASSERT_TRUE(scheduler.shouldListenForTelemetry());
    TEST_ASSERT_FALSE(scheduler.onTimerTick());
    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(telemetryRate.telemetryRatio - 1), scheduler.sequence());
    TEST_ASSERT_TRUE(scheduler.onTimerTick());
    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(telemetryRate.telemetryRatio), scheduler.sequence());
}

static void test_rf_scheduler_tx_sync_scheduling_metadata() {
    uint8_t uid[kUidSize];
    deriveUid("scheduler-sync-tx", uid);
    link_rf::TxScheduler scheduler;
    scheduler.begin(uid, kRates[0]);

    TEST_ASSERT_TRUE(scheduler.onTimerTick());
    TEST_ASSERT_FALSE(scheduler.shouldSendSyncFrame());
    OtaSyncPayload sync = scheduler.syncPayload();
    TEST_ASSERT_EQUAL_UINT16(0, sync.nonce);
    TEST_ASSERT_EQUAL_UINT16(0, sync.fhssIndex);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(RateId::L250), sync.rateIndex);
    TEST_ASSERT_EQUAL_UINT8(kRates[0].telemetryRatio, sync.telemetryRatio);
    TEST_ASSERT_EQUAL_UINT8(kRates[0].fhssHopInterval, sync.fhssHopInterval);

    const uint16_t firstSyncSequence =
        static_cast<uint16_t>(link_rf::kSyncChannelFhssIndex * kRates[0].fhssHopInterval);
    while (scheduler.sequence() < firstSyncSequence) {
        scheduler.onTxDone();
        TEST_ASSERT_TRUE(scheduler.onTimerTick());
    }

    TEST_ASSERT_TRUE(scheduler.shouldSendSyncFrame());
    sync = scheduler.syncPayload();
    TEST_ASSERT_EQUAL_UINT16(firstSyncSequence, sync.nonce);
    TEST_ASSERT_EQUAL_UINT16(link_rf::kSyncChannelFhssIndex, sync.fhssIndex);
}

static void test_rf_scheduler_rx_relocks_fhss_from_received_sequence() {
    uint8_t uid[kUidSize];
    deriveUid("scheduler-rx", uid);
    link_rf::RxScheduler scheduler;
    scheduler.begin(uid, kRates[0]);
    OtaSyncPayload sync{};
    sync.nonce = 36;
    sync.fhssIndex = 5;
    sync.rateIndex = static_cast<uint8_t>(RateId::L250);
    sync.nextRateIndex = static_cast<uint8_t>(RateId::L250);
    sync.telemetryRatio = kRates[0].telemetryRatio;
    sync.fhssHopInterval = kRates[0].fhssHopInterval;
    TEST_ASSERT_TRUE(scheduler.onValidSyncFrame(sync, 10000));
    TEST_ASSERT_EQUAL_UINT16(37, scheduler.nextExpectedSequence());
    TEST_ASSERT_TRUE(scheduler.onValidRcFrame(37, 20000, 20));
    scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock,
                           scheduler.packetTockReferenceUs(20000) - 100u, true, 20);
    TEST_ASSERT_EQUAL_UINT16(38, scheduler.nextExpectedSequence());
    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(37 / kRates[0].fhssHopInterval),
                             scheduler.fhss().index());
    TEST_ASSERT_EQUAL_INT32(100, scheduler.stats().phaseErrorUs);
}

static void test_rf_scheduler_rx_timer_miss_advances_expected_slot() {
    uint8_t uid[kUidSize];
    deriveUid("scheduler-rx-miss", uid);
    link_rf::RxScheduler scheduler;
    scheduler.begin(uid, kRates[0]);
    OtaSyncPayload sync{};
    sync.nonce = 6;
    sync.fhssIndex = 1;
    sync.rateIndex = static_cast<uint8_t>(RateId::L250);
    sync.nextRateIndex = static_cast<uint8_t>(RateId::L250);
    sync.telemetryRatio = kRates[0].telemetryRatio;
    sync.fhssHopInterval = kRates[0].fhssHopInterval;
    TEST_ASSERT_TRUE(scheduler.onValidSyncFrame(sync, 10000));
    scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock,
                           scheduler.packetTockReferenceUs(10000), true, 10);
    scheduler.onValidRcFrame(7, 20000, 20);
    scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock,
                           scheduler.packetTockReferenceUs(20000), true, 20);
    const link_rf::RxTimerEventResult event =
        scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock, 30000, true, 30);
    TEST_ASSERT_EQUAL_UINT16(9, scheduler.nextExpectedSequence());
    TEST_ASSERT_EQUAL_UINT32(1, scheduler.stats().missedRcFrames);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(link_rf::RxTimerTickEvent::None),
                            static_cast<uint8_t>(event.event));
}

static void test_rf_scheduler_rx_connected_miss_requests_retune_on_hop_boundary() {
    uint8_t uid[kUidSize];
    deriveUid("scheduler-rx-boundary-miss", uid);
    link_rf::RxScheduler scheduler;
    scheduler.begin(uid, kRates[0]);
    OtaSyncPayload sync{};
    sync.nonce = static_cast<uint16_t>(kRates[0].fhssHopInterval - 2);
    sync.fhssIndex = link_rf::kSyncChannelFhssIndex;
    sync.rateIndex = static_cast<uint8_t>(RateId::L250);
    sync.nextRateIndex = static_cast<uint8_t>(RateId::L250);
    sync.telemetryRatio = kRates[0].telemetryRatio;
    sync.fhssHopInterval = kRates[0].fhssHopInterval;
    TEST_ASSERT_TRUE(scheduler.onValidSyncFrame(sync, 10000));
    scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock,
                           scheduler.packetTockReferenceUs(10000), true, 10);
    scheduler.onValidRcFrame(static_cast<uint16_t>(kRates[0].fhssHopInterval - 1), 20000, 20);
    const uint16_t previousHop = scheduler.fhss().index();

    const link_rf::RxTimerEventResult event =
        scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock,
                               scheduler.packetTockReferenceUs(20000), true, 20);

    TEST_ASSERT_EQUAL_UINT16(kRates[0].fhssHopInterval, scheduler.nextExpectedSequence());
    TEST_ASSERT_NOT_EQUAL_UINT16(previousHop, scheduler.fhss().index());
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(link_rf::RxTimerTickEvent::ReceiveFrequencyChanged),
                            static_cast<uint8_t>(event.event));
}

static void test_rf_scheduler_rx_acquires_timing_from_sync_frame() {
    uint8_t uid[kUidSize];
    deriveUid("scheduler-rx-sync", uid);
    link_rf::RxScheduler scheduler;
    scheduler.begin(uid, kRates[0]);

    TEST_ASSERT_FALSE(scheduler.onValidRcFrame(37, 10000, 0));
    TEST_ASSERT_EQUAL_UINT16(0, scheduler.nextExpectedSequence());
    OtaSyncPayload sync{};
    sync.nonce = 36;
    sync.fhssIndex = 5;
    sync.rateIndex = static_cast<uint8_t>(RateId::L250);
    sync.nextRateIndex = static_cast<uint8_t>(RateId::L250);
    sync.telemetryRatio = kRates[0].telemetryRatio;
    sync.fhssHopInterval = kRates[0].fhssHopInterval;

    TEST_ASSERT_TRUE(scheduler.onValidSyncFrame(sync, 10000));
    TEST_ASSERT_EQUAL_UINT16(37, scheduler.nextExpectedSequence());
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(link_rf::ConnectionState::Tentative),
                            static_cast<uint8_t>(scheduler.stats().connectionState));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(link_rf::RxTimerState::TimDisconnected),
                            static_cast<uint8_t>(scheduler.stats().rxTimerState));
    scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock,
                           scheduler.packetTockReferenceUs(10000), true, 10);
    TEST_ASSERT_TRUE(scheduler.onValidRcFrame(37, 20000, 20));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(link_rf::ConnectionState::Connected),
                            static_cast<uint8_t>(scheduler.stats().connectionState));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(link_rf::RxTimerState::TimTentative),
                            static_cast<uint8_t>(scheduler.stats().rxTimerState));
}

static void test_rf_scheduler_rx_acquires_directly_from_acquisition_rc_frame() {
    uint8_t uid[kUidSize];
    deriveUid("scheduler-rx-direct-rc", uid);
    link_rf::RxScheduler scheduler;
    scheduler.begin(uid, kRates[0]);

    TEST_ASSERT_TRUE(scheduler.onAcquisitionRcFrame(123, 10000, 10));
    TEST_ASSERT_EQUAL_UINT16(123, scheduler.nextExpectedSequence());
    TEST_ASSERT_EQUAL_UINT16(123 / kRates[0].fhssHopInterval, scheduler.fhss().index());
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(link_rf::ConnectionState::Connected),
                            static_cast<uint8_t>(scheduler.stats().connectionState));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(link_rf::RxTimerState::TimTentative),
                            static_cast<uint8_t>(scheduler.stats().rxTimerState));
    TEST_ASSERT_EQUAL_UINT32(1, scheduler.stats().rxDone);
    TEST_ASSERT_EQUAL_UINT32(0, scheduler.stats().rcRejectedNoSync);
}

static void test_rx_pfd_reports_external_minus_internal_offset() {
    link_rf::PhaseFrequencyDetector pfd;
    pfd.extEvent(1200);
    TEST_ASSERT_FALSE(pfd.hasResult());
    pfd.intEvent(1000);
    TEST_ASSERT_TRUE(pfd.hasResult());
    TEST_ASSERT_EQUAL_INT32(200, pfd.result());
    pfd.reset();
    TEST_ASSERT_FALSE(pfd.hasResult());
}

static void test_rf_scheduler_rx_pfd_lpf_offset_and_derivative() {
    uint8_t uid[kUidSize];
    deriveUid("scheduler-rx-pfd-lpf", uid);
    link_rf::RxScheduler scheduler;
    scheduler.begin(uid, kRates[0]);
    OtaSyncPayload sync{};
    sync.nonce = 36;
    sync.fhssIndex = 5;
    sync.rateIndex = static_cast<uint8_t>(RateId::L250);
    sync.nextRateIndex = static_cast<uint8_t>(RateId::L250);
    sync.telemetryRatio = kRates[0].telemetryRatio;
    sync.fhssHopInterval = kRates[0].fhssHopInterval;

    TEST_ASSERT_TRUE(scheduler.onValidSyncFrame(sync, 10000));
    uint32_t reference = scheduler.packetTockReferenceUs(10000);
    scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock, reference - 80u, true, 10);
    TEST_ASSERT_EQUAL_INT32(80, scheduler.stats().phaseErrorUs);
    TEST_ASSERT_EQUAL_INT32(20, scheduler.stats().offsetUs);
    TEST_ASSERT_EQUAL_INT32(5, scheduler.stats().offsetDxUs);

    TEST_ASSERT_TRUE(scheduler.onValidRcFrame(37, 20000, 20));
    reference = scheduler.packetTockReferenceUs(20000);
    scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock, reference - 48u, true, 20);
    TEST_ASSERT_EQUAL_INT32(48, scheduler.stats().phaseErrorUs);
    TEST_ASSERT_EQUAL_INT32(27, scheduler.stats().offsetUs);
    TEST_ASSERT_EQUAL_INT32(2, scheduler.stats().offsetDxUs);
}

static void test_rf_scheduler_rx_tentative_phase_shift_uses_raw_offset() {
    uint8_t uid[kUidSize];
    deriveUid("scheduler-rx-tentative-shift", uid);
    link_rf::RxScheduler scheduler;
    scheduler.begin(uid, kRates[0]);
    OtaSyncPayload sync{};
    sync.nonce = 10;
    sync.fhssIndex = 2;
    sync.rateIndex = static_cast<uint8_t>(RateId::L250);
    sync.nextRateIndex = static_cast<uint8_t>(RateId::L250);
    sync.telemetryRatio = kRates[0].telemetryRatio;
    sync.fhssHopInterval = kRates[0].fhssHopInterval;

    TEST_ASSERT_TRUE(scheduler.onValidSyncFrame(sync, 10000));
    const uint32_t reference = scheduler.packetTockReferenceUs(10000);
    const link_rf::RxTimerEventResult result =
        scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock, reference - 80u, true, 10);
    TEST_ASSERT_TRUE(result.hasPhaseShift);
    TEST_ASSERT_EQUAL_INT32(80, result.phaseShiftUs);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(link_rf::ConnectionState::Tentative),
                            static_cast<uint8_t>(scheduler.stats().connectionState));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(link_rf::RxTimerState::TimDisconnected),
                            static_cast<uint8_t>(scheduler.stats().rxTimerState));
}

static void test_rf_scheduler_rx_connected_phase_shift_uses_filtered_offset_quarter() {
    uint8_t uid[kUidSize];
    deriveUid("scheduler-rx-connected-shift", uid);
    link_rf::RxScheduler scheduler;
    scheduler.begin(uid, kRates[0]);
    OtaSyncPayload sync{};
    sync.nonce = 36;
    sync.fhssIndex = 5;
    sync.rateIndex = static_cast<uint8_t>(RateId::L250);
    sync.nextRateIndex = static_cast<uint8_t>(RateId::L250);
    sync.telemetryRatio = kRates[0].telemetryRatio;
    sync.fhssHopInterval = kRates[0].fhssHopInterval;

    TEST_ASSERT_TRUE(scheduler.onValidSyncFrame(sync, 10000));
    scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock,
                           scheduler.packetTockReferenceUs(10000), true, 10);
    TEST_ASSERT_TRUE(scheduler.onValidRcFrame(37, 20000, 20));
    scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock,
                           scheduler.packetTockReferenceUs(20000) - 40u, true, 20);
    TEST_ASSERT_TRUE(scheduler.onValidRcFrame(38, 30000, 30));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(link_rf::ConnectionState::Connected),
                            static_cast<uint8_t>(scheduler.stats().connectionState));

    const link_rf::RxTimerEventResult result =
        scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock,
                               scheduler.packetTockReferenceUs(30000) - 80u, true, 30);
    TEST_ASSERT_TRUE(result.hasPhaseShift);
    TEST_ASSERT_EQUAL_INT32(6, result.phaseShiftUs);
}

static void test_rf_scheduler_rx_promotes_with_stable_negative_offset_like_elrs() {
    uint8_t uid[kUidSize];
    deriveUid("scheduler-rx-negative-offset", uid);
    link_rf::RxScheduler scheduler;
    scheduler.begin(uid, kRates[0]);
    OtaSyncPayload sync{};
    sync.nonce = 36;
    sync.fhssIndex = 5;
    sync.rateIndex = static_cast<uint8_t>(RateId::L250);
    sync.nextRateIndex = static_cast<uint8_t>(RateId::L250);
    sync.telemetryRatio = kRates[0].telemetryRatio;
    sync.fhssHopInterval = kRates[0].fhssHopInterval;

    TEST_ASSERT_TRUE(scheduler.onValidSyncFrame(sync, 10000));
    for (uint8_t i = 0; i < 64; ++i) {
        const uint16_t sequence = static_cast<uint16_t>(37 + i);
        const uint32_t packetUs = 20000u + static_cast<uint32_t>(i) * kRates[0].intervalUs;
        const uint32_t nowMs = 20u + static_cast<uint32_t>(i) * 10u;
        scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock,
                               scheduler.packetTockReferenceUs(packetUs - kRates[0].intervalUs) + 900u,
                               true, nowMs);
        TEST_ASSERT_TRUE(scheduler.onValidRcFrame(sequence, packetUs, nowMs));
    }
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(link_rf::ConnectionState::Connected),
                            static_cast<uint8_t>(scheduler.stats().connectionState));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(link_rf::RxTimerState::TimTentative),
                            static_cast<uint8_t>(scheduler.stats().rxTimerState));
}

static void test_rf_scheduler_rx_locks_only_after_dwell_and_low_derivative() {
    uint8_t uid[kUidSize];
    deriveUid("scheduler-rx-lock-dwell", uid);
    link_rf::RxScheduler scheduler;
    scheduler.begin(uid, kRates[0]);
    OtaSyncPayload sync{};
    sync.nonce = 36;
    sync.fhssIndex = 5;
    sync.rateIndex = static_cast<uint8_t>(RateId::L250);
    sync.nextRateIndex = static_cast<uint8_t>(RateId::L250);
    sync.telemetryRatio = kRates[0].telemetryRatio;
    sync.fhssHopInterval = kRates[0].fhssHopInterval;

    TEST_ASSERT_TRUE(scheduler.onValidSyncFrame(sync, 10000));
    scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock,
                           scheduler.packetTockReferenceUs(10000), true, 10);
    TEST_ASSERT_TRUE(scheduler.onValidRcFrame(37, 20000, 20));
    scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock,
                           scheduler.packetTockReferenceUs(20000) - 40u, true, 20);
    TEST_ASSERT_TRUE(scheduler.onValidRcFrame(38, 30000, 30));
    scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock,
                           scheduler.packetTockReferenceUs(30000) - 80u, true, 30);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(link_rf::RxTimerState::TimTentative),
                            static_cast<uint8_t>(scheduler.stats().rxTimerState));

    TEST_ASSERT_TRUE(scheduler.onValidRcFrame(39, 40000, 1031));
    scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock,
                           scheduler.packetTockReferenceUs(40000) - 80u, true, 1031);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(link_rf::RxTimerState::TimLocked),
                            static_cast<uint8_t>(scheduler.stats().rxTimerState));
}

static void test_rf_scheduler_rx_locked_frequency_trim_runs_every_8_nonces() {
    uint8_t uid[kUidSize];
    deriveUid("scheduler-rx-trim", uid);
    link_rf::RxScheduler scheduler;
    scheduler.begin(uid, kRates[0]);
    OtaSyncPayload sync{};
    sync.nonce = 36;
    sync.fhssIndex = 5;
    sync.rateIndex = static_cast<uint8_t>(RateId::L250);
    sync.nextRateIndex = static_cast<uint8_t>(RateId::L250);
    sync.telemetryRatio = kRates[0].telemetryRatio;
    sync.fhssHopInterval = kRates[0].fhssHopInterval;

    TEST_ASSERT_TRUE(scheduler.onValidSyncFrame(sync, 10000));
    scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock,
                           scheduler.packetTockReferenceUs(10000), true, 10);
    TEST_ASSERT_TRUE(scheduler.onValidRcFrame(37, 20000, 20));
    scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock,
                           scheduler.packetTockReferenceUs(20000) - 40u, true, 20);
    TEST_ASSERT_TRUE(scheduler.onValidRcFrame(38, 30000, 30));
    scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock,
                           scheduler.packetTockReferenceUs(30000) - 80u, true, 30);
    TEST_ASSERT_TRUE(scheduler.onValidRcFrame(39, 40000, 1031));
    scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock,
                           scheduler.packetTockReferenceUs(40000) - 80u, true, 1031);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(link_rf::RxTimerState::TimLocked),
                            static_cast<uint8_t>(scheduler.stats().rxTimerState));

    TEST_ASSERT_TRUE(scheduler.onValidRcFrame(47, 50000, 1040));
    const link_rf::RxTimerEventResult result =
        scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock,
                               scheduler.packetTockReferenceUs(50000) - 80u, true, 1040);
    TEST_ASSERT_EQUAL_UINT16(48, scheduler.nextExpectedSequence());
    TEST_ASSERT_EQUAL_INT32(1, result.frequencyOffsetUs);
    TEST_ASSERT_EQUAL_INT32(1, scheduler.stats().frequencyOffsetUs);
}

static void test_rf_scheduler_rx_tick_tock_event_sequencing() {
    uint8_t uid[kUidSize];
    deriveUid("scheduler-rx-tick-tock", uid);
    link_rf::RxScheduler scheduler;
    scheduler.begin(uid, kRates[0]);
    OtaSyncPayload sync{};
    sync.nonce = 12;
    sync.fhssIndex = 3;
    sync.rateIndex = static_cast<uint8_t>(RateId::L250);
    sync.nextRateIndex = static_cast<uint8_t>(RateId::L250);
    sync.telemetryRatio = kRates[0].telemetryRatio;
    sync.fhssHopInterval = kRates[0].fhssHopInterval;

    TEST_ASSERT_TRUE(scheduler.onValidSyncFrame(sync, 10000));
    scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tick, 11000, true, 10);
    TEST_ASSERT_EQUAL_UINT16(13, scheduler.nextExpectedSequence());
    TEST_ASSERT_EQUAL_UINT32(0, scheduler.stats().missedRcFrames);

    scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock,
                           scheduler.packetTockReferenceUs(10000), true, 10);
    TEST_ASSERT_EQUAL_UINT16(14, scheduler.nextExpectedSequence());
    scheduler.onTimerEvent(link_rf::RxTimerHalfEvent::Tock, 20000, true, 20);
    TEST_ASSERT_EQUAL_UINT16(15, scheduler.nextExpectedSequence());
    TEST_ASSERT_EQUAL_UINT32(1, scheduler.stats().missedRcFrames);
}

static void test_rf_scheduler_rx_rejects_invalid_sync_metadata() {
    uint8_t uid[kUidSize];
    deriveUid("scheduler-rx-bad-sync", uid);
    link_rf::RxScheduler scheduler;
    scheduler.begin(uid, kRates[0]);

    OtaSyncPayload sync{};
    sync.nonce = 36;
    sync.fhssIndex = kFhssChannelCount;
    sync.rateIndex = static_cast<uint8_t>(RateId::L250);
    sync.nextRateIndex = static_cast<uint8_t>(RateId::L250);
    sync.telemetryRatio = kRates[0].telemetryRatio;
    sync.fhssHopInterval = kRates[0].fhssHopInterval;

    TEST_ASSERT_FALSE(scheduler.onValidSyncFrame(sync, 10000));
    TEST_ASSERT_EQUAL_UINT16(0, scheduler.nextExpectedSequence());
    TEST_ASSERT_EQUAL_UINT16(0, scheduler.fhss().index());
}

static void test_rf_scheduler_rx_disconnected_scan_stays_on_sync_channel() {
    uint8_t uid[kUidSize];
    deriveUid("scheduler-rx-scan", uid);
    link_rf::RxScheduler scheduler;
    scheduler.begin(uid, kRates[0]);
    scheduler.onTimerTick(false);
    TEST_ASSERT_EQUAL_UINT16(0, scheduler.fhss().index());
    scheduler.onTimerTick(false);
    TEST_ASSERT_EQUAL_UINT16(0, scheduler.fhss().index());
    scheduler.onTimerTick(false);
    TEST_ASSERT_EQUAL_UINT16(0, scheduler.fhss().index());
    TEST_ASSERT_EQUAL_UINT16(0, scheduler.nextExpectedSequence());
}

static void test_ota_sync_payload_round_trip_and_validation() {
    uint8_t uid[kUidSize];
    deriveUid("ota-sync", uid);
    OtaFrame in{};
    in.type = OtaType::Sync;
    in.sequence = 48;
    in.uidCheck = uidCheck(uid);
    OtaSyncPayload sync{};
    sync.nonce = in.sequence;
    sync.fhssIndex = 12;
    sync.rateIndex = static_cast<uint8_t>(RateId::L250);
    sync.nextRateIndex = static_cast<uint8_t>(RateId::L250);
    sync.switchSequence = 0;
    sync.telemetryRatio = kRates[0].telemetryRatio;
    sync.fhssHopInterval = kRates[0].fhssHopInterval;
    sync.txTick = 48;
    TEST_ASSERT_TRUE(encodeOtaSyncPayload(sync, in.payload, in.payloadLen));

    uint8_t bytes[kOtaFrameSize];
    TEST_ASSERT_TRUE(encodeOtaFrame(in, bytes));
    OtaFrame frame{};
    TEST_ASSERT_TRUE(decodeOtaFrame(bytes, uidCheck(uid), frame));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OtaType::Sync), static_cast<uint8_t>(frame.type));

    OtaSyncPayload out{};
    TEST_ASSERT_TRUE(decodeOtaSyncPayload(frame, out));
    TEST_ASSERT_EQUAL_UINT16(sync.nonce, out.nonce);
    TEST_ASSERT_EQUAL_UINT16(sync.fhssIndex, out.fhssIndex);
    TEST_ASSERT_EQUAL_UINT8(sync.rateIndex, out.rateIndex);
    TEST_ASSERT_EQUAL_UINT8(sync.fhssHopInterval, out.fhssHopInterval);

    frame.payload[4] = 0xFF;
    TEST_ASSERT_FALSE(decodeOtaSyncPayload(frame, out));
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

static void test_ota_decode_rejects_wrong_uid_length_and_type() {
    uint8_t uid[kUidSize];
    deriveUid("ota-reject", uid);
    OtaFrame in{};
    in.type = OtaType::Rc;
    in.sequence = 9;
    in.uidCheck = uidCheck(uid);
    in.payloadLen = 1;
    in.payload[0] = 0xAA;

    uint8_t bytes[kOtaFrameSize];
    OtaFrame out{};
    TEST_ASSERT_TRUE(encodeOtaFrame(in, bytes));

    uint8_t otherUid[kUidSize];
    deriveUid("other", otherUid);
    TEST_ASSERT_FALSE(decodeOtaFrame(bytes, uidCheck(otherUid), out));

    TEST_ASSERT_TRUE(encodeOtaFrame(in, bytes));
    bytes[8] = static_cast<uint8_t>(kOtaPayloadSize + 1);
    uint16_t crc = crc16Ccitt(bytes, 31);
    bytes[31] = static_cast<uint8_t>(crc >> 8);
    bytes[32] = static_cast<uint8_t>(crc);
    TEST_ASSERT_FALSE(decodeOtaFrame(bytes, uidCheck(uid), out));

    TEST_ASSERT_TRUE(encodeOtaFrame(in, bytes));
    bytes[1] = 0x7F;
    crc = crc16Ccitt(bytes, 31);
    bytes[31] = static_cast<uint8_t>(crc >> 8);
    bytes[32] = static_cast<uint8_t>(crc);
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
    TEST_ASSERT_EQUAL_UINT16(kCrsfRaw2000, channels[0]);
    TEST_ASSERT_EQUAL_UINT16(kCrsfRaw2000, channels[1]);
    TEST_ASSERT_EQUAL_UINT16(kCrsfRaw2000, channels[2]);
    TEST_ASSERT_EQUAL_UINT16(kCrsfRaw2000, channels[3]);
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

static void test_spike_gate_accepts_fast_continuous_ramp() {
    uint16_t previous[kRcChannelCount];
    uint16_t first[kRcChannelCount];
    uint16_t second[kRcChannelCount];
    for (uint8_t i = 0; i < kRcChannelCount; ++i) {
        previous[i] = kCrsfRawMid;
        first[i] = kCrsfRawMid;
        second[i] = kCrsfRawMid;
    }
    first[0] = static_cast<uint16_t>(kCrsfRawMid + 200);
    second[0] = static_cast<uint16_t>(kCrsfRawMid + 380);

    RcSpikeGate gate{};
    TEST_ASSERT_FALSE(acceptRcChannelsWithSpikeGate(previous, true, first, gate, 160, 80));
    TEST_ASSERT_TRUE(acceptRcChannelsWithSpikeGate(previous, true, second, gate, 160, 80));
    TEST_ASSERT_FALSE(gate.havePending);
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

    record = makeConfigRecord(cfg);
    record.reserved = 0xA5;
    TEST_ASSERT_FALSE(readConfigRecord(record, out));
}

static void test_binding_phrase_validation() {
    TEST_ASSERT_TRUE(validateBindingPhrase("bench phrase 01"));
    TEST_ASSERT_TRUE(validateBindingPhrase("12345678901234567890123456789012"));
    TEST_ASSERT_FALSE(validateBindingPhrase(""));
    TEST_ASSERT_FALSE(validateBindingPhrase(nullptr));
    TEST_ASSERT_FALSE(validateBindingPhrase("123456789012345678901234567890123"));
    TEST_ASSERT_FALSE(validateBindingPhrase("bad\nphrase"));
}

static void test_binding_status_derives_uid_check() {
    BindingStatus status{};
    makeBindingStatus("webui-bind", true, true, status);

    uint8_t uid[kUidSize];
    deriveUid("webui-bind", uid);
    TEST_ASSERT_EQUAL_STRING("webui-bind", status.phrase);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(uid, status.uid, kUidSize);
    TEST_ASSERT_EQUAL_UINT32(uidCheck(uid), status.uidCheck);
    TEST_ASSERT_TRUE(status.persisted);
    TEST_ASSERT_TRUE(status.requiresReboot);
}

static size_t makeBindingRequestFrame(BindingControlOp op, const char* phrase, uint8_t* out, size_t outLen) {
    const size_t phraseLen = phrase ? strlen(phrase) : 0;
    const size_t payloadLen = 7 + phraseLen;
    const size_t frameLen = payloadLen + 4;
    if (outLen < frameLen) return 0;
    out[0] = 0xEE;
    out[1] = static_cast<uint8_t>(payloadLen + 2);
    out[2] = 0x7D;
    out[3] = 'X';
    out[4] = 'L';
    out[5] = 'R';
    out[6] = 'S';
    out[7] = 1;
    out[8] = static_cast<uint8_t>(op);
    out[9] = static_cast<uint8_t>(phraseLen);
    if (phraseLen) memcpy(&out[10], phrase, phraseLen);
    out[frameLen - 1] = crsfCrc8(&out[2], out[1] - 1);
    return frameLen;
}

static void test_crsf_binding_request_and_response_frames() {
    uint8_t frame[64] = {};
    const size_t requestLen = makeBindingRequestFrame(BindingControlOp::Set, "rx-usb-webui",
                                                      frame, sizeof(frame));
    TEST_ASSERT_GREATER_THAN_UINT(0, requestLen);

    BindingControlRequest request{};
    bool parsed = false;
    for (size_t i = 0; i < requestLen; ++i) {
        parsed = parseCrsfBindingRequestFrame(frame[i], request) || parsed;
    }
    TEST_ASSERT_TRUE(parsed);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BindingControlOp::Set), static_cast<uint8_t>(request.op));
    TEST_ASSERT_EQUAL_STRING("rx-usb-webui", request.phrase);

    BindingStatus status{};
    makeBindingStatus(request.phrase, true, true, status);
    uint8_t response[64] = {};
    const size_t responseLen = encodeCrsfBindingResponseFrame(status, BindingResult::Ok, request.op,
                                                              response, sizeof(response));
    TEST_ASSERT_GREATER_THAN_UINT(0, responseLen);
    TEST_ASSERT_EQUAL_UINT8(0xEA, response[0]);
    TEST_ASSERT_EQUAL_UINT8(0x7D, response[2]);
    TEST_ASSERT_EQUAL_UINT8('X', response[3]);
    TEST_ASSERT_EQUAL_UINT8('L', response[4]);
    TEST_ASSERT_EQUAL_UINT8('R', response[5]);
    TEST_ASSERT_EQUAL_UINT8('S', response[6]);
    TEST_ASSERT_EQUAL_UINT8(1, response[7]);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BindingControlOp::Set), response[8]);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BindingResult::Ok), response[9]);
    TEST_ASSERT_EQUAL_UINT8(1, response[10]);
    TEST_ASSERT_EQUAL_UINT8(1, response[11]);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(status.uidCheck >> 24), response[12]);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(status.uidCheck >> 16), response[13]);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(status.uidCheck >> 8), response[14]);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(status.uidCheck), response[15]);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(status.uid, &response[16], kUidSize);
    TEST_ASSERT_EQUAL_UINT8(strlen(status.phrase), response[24]);
    TEST_ASSERT_EQUAL_UINT8(crsfCrc8(&response[2], response[1] - 1), response[responseLen - 1]);
}

static void test_crsf_binding_request_encoder_and_response_parser() {
    BindingControlRequest request{};
    request.op = BindingControlOp::Verify;
    strncpy(request.phrase, "Field Radio", sizeof(request.phrase) - 1);

    uint8_t requestFrame[64] = {};
    const size_t requestLen = encodeCrsfBindingRequestFrame(request, requestFrame, sizeof(requestFrame));
    TEST_ASSERT_GREATER_THAN_UINT(0, requestLen);

    BindingControlRequest parsedRequest{};
    bool parsed = false;
    for (size_t i = 0; i < requestLen; ++i) {
        parsed = parseCrsfBindingRequestFrame(requestFrame[i], parsedRequest) || parsed;
    }
    TEST_ASSERT_TRUE(parsed);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BindingControlOp::Verify),
                            static_cast<uint8_t>(parsedRequest.op));
    TEST_ASSERT_EQUAL_STRING("Field Radio", parsedRequest.phrase);

    BindingStatus status{};
    makeBindingStatus(request.phrase, true, false, status);
    uint8_t responseFrame[64] = {};
    const size_t responseLen = encodeCrsfBindingResponseFrame(status, BindingResult::Ok, request.op,
                                                              responseFrame, sizeof(responseFrame));
    TEST_ASSERT_GREATER_THAN_UINT(0, responseLen);

    BindingStatus parsedStatus{};
    BindingResult parsedResult = BindingResult::InvalidCommand;
    BindingControlOp parsedOp = BindingControlOp::Get;
    parsed = false;
    for (size_t i = 0; i < responseLen; ++i) {
        parsed = parseCrsfBindingResponseFrame(responseFrame[i], parsedStatus, parsedResult, parsedOp) || parsed;
    }
    TEST_ASSERT_TRUE(parsed);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BindingControlOp::Verify), static_cast<uint8_t>(parsedOp));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BindingResult::Ok), static_cast<uint8_t>(parsedResult));
    TEST_ASSERT_EQUAL_STRING(status.phrase, parsedStatus.phrase);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(status.uid, parsedStatus.uid, kUidSize);
    TEST_ASSERT_EQUAL_UINT32(status.uidCheck, parsedStatus.uidCheck);
    TEST_ASSERT_TRUE(parsedStatus.persisted);
    TEST_ASSERT_FALSE(parsedStatus.requiresReboot);
}

static void test_failsafe_freshness_helper_uses_named_timeout() {
    const uint32_t lastUplinkMs = 1000;
    TEST_ASSERT_EQUAL_UINT32(250, kFailsafeTimeoutMs);
    TEST_ASSERT_TRUE(isLinkFresh(1200u, lastUplinkMs));
    TEST_ASSERT_FALSE(isLinkFresh(1250u, lastUplinkMs));
    TEST_ASSERT_FALSE(isLinkFresh(1300u, lastUplinkMs));
    TEST_ASSERT_FALSE(isLinkFresh(1200u, 0));
    TEST_ASSERT_TRUE(isLinkFresh(20u, 0xFFFFFFF0u));
}

static void test_telemetry_listen_window_is_short_and_scheduled_by_ratio() {
    TEST_ASSERT_EQUAL_UINT8(1, kTelemetryResponseListenSlots);
    TEST_ASSERT_FALSE(shouldRequestTelemetry(16, kRates[0].telemetryRatio));
    TEST_ASSERT_TRUE(shouldRequestTelemetry(16, 16));
    TEST_ASSERT_FALSE(shouldRequestTelemetry(17, kRates[0].telemetryRatio));
    TEST_ASSERT_FALSE(shouldRequestTelemetry(16, 0));
    TEST_ASSERT_EQUAL_UINT32(4, telemetryListenWindowMs(kRates[0]));
    TEST_ASSERT_EQUAL_UINT32(40, telemetryListenWindowMs(kRates[1]));
}

static void test_crsf_link_stats_puts_rate_in_rf_mode_not_power() {
    uint8_t frame[16] = {};
    LinkStats stats{};
    stats.uplinkRssiDbm = -71;
    stats.uplinkRssi2Dbm = -73;
    stats.uplinkLinkQuality = 93;
    stats.uplinkSnrDb = 8;
    stats.activeAntenna = 1;
    stats.rfMode = static_cast<uint8_t>(RateId::L100);
    stats.uplinkTxPower = 8;
    stats.downlinkRssiDbm = -68;
    stats.downlinkLinkQuality = 87;
    stats.downlinkSnrDb = 6;
    const size_t len = encodeCrsfLinkStats(stats, frame, sizeof(frame));
    TEST_ASSERT_EQUAL_UINT(14, len);
    TEST_ASSERT_EQUAL_UINT8(0x14, frame[2]);
    TEST_ASSERT_EQUAL_INT8(-71, static_cast<int8_t>(frame[3]));
    TEST_ASSERT_EQUAL_INT8(-73, static_cast<int8_t>(frame[4]));
    TEST_ASSERT_EQUAL_UINT8(93, frame[5]);
    TEST_ASSERT_EQUAL_UINT8(8, frame[6]);
    TEST_ASSERT_EQUAL_UINT8(1, frame[7]);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(RateId::L100), frame[8]);
    TEST_ASSERT_EQUAL_UINT8(8, frame[9]);
    TEST_ASSERT_EQUAL_INT8(-68, static_cast<int8_t>(frame[10]));
    TEST_ASSERT_EQUAL_UINT8(87, frame[11]);
    TEST_ASSERT_EQUAL_UINT8(6, frame[12]);
    TEST_ASSERT_EQUAL_UINT8(crsfCrc8(&frame[2], 11), frame[13]);
}

static void test_downlink_telemetry_payload_round_trips_diagnostics() {
    DownlinkTelemetryPayload payload{};
    payload.uplinkLinkQuality = 97;
    payload.uplinkRssiDbm = -64;
    payload.uplinkSnrDb = 9;
    payload.rfMode = static_cast<uint8_t>(RateId::L250);
    payload.uplinkTxPower = 3;
    payload.faultFlags = kXlrsTelemetryFaultTimingDrift | kXlrsTelemetryFaultRadioRejects;
    payload.fhssIndex = 12;
    payload.expectedFhssIndex = 13;
    payload.phaseErrorBucket = -11;
    payload.lostConnectionCount = 2;
    payload.radioRejectCount = 7;

    OtaFrame frame{};
    frame.type = OtaType::Telemetry;
    TEST_ASSERT_TRUE(encodeDownlinkTelemetryPayload(payload, frame.payload, frame.payloadLen));
    TEST_ASSERT_EQUAL_UINT8(12, frame.payloadLen);

    DownlinkTelemetryPayload parsed{};
    TEST_ASSERT_TRUE(decodeDownlinkTelemetryPayload(frame, parsed));
    TEST_ASSERT_EQUAL_UINT8(97, parsed.uplinkLinkQuality);
    TEST_ASSERT_EQUAL_INT16(-64, parsed.uplinkRssiDbm);
    TEST_ASSERT_EQUAL_INT8(9, parsed.uplinkSnrDb);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(RateId::L250), parsed.rfMode);
    TEST_ASSERT_EQUAL_UINT8(3, parsed.uplinkTxPower);
    TEST_ASSERT_EQUAL_UINT8(payload.faultFlags, parsed.faultFlags);
    TEST_ASSERT_EQUAL_UINT8(12, parsed.fhssIndex);
    TEST_ASSERT_EQUAL_UINT8(13, parsed.expectedFhssIndex);
    TEST_ASSERT_EQUAL_INT8(-11, parsed.phaseErrorBucket);
    TEST_ASSERT_EQUAL_UINT8(2, parsed.lostConnectionCount);
    TEST_ASSERT_EQUAL_UINT8(7, parsed.radioRejectCount);
}

static void test_downlink_telemetry_decoder_accepts_legacy_four_byte_payload() {
    OtaFrame frame{};
    frame.type = OtaType::Telemetry;
    frame.payloadLen = 4;
    frame.payload[0] = 88;
    frame.payload[1] = 52;
    frame.payload[2] = static_cast<uint8_t>(-4);
    frame.payload[3] = static_cast<uint8_t>(RateId::L100);

    DownlinkTelemetryPayload parsed{};
    TEST_ASSERT_TRUE(decodeDownlinkTelemetryPayload(frame, parsed));
    TEST_ASSERT_EQUAL_UINT8(88, parsed.uplinkLinkQuality);
    TEST_ASSERT_EQUAL_INT16(-52, parsed.uplinkRssiDbm);
    TEST_ASSERT_EQUAL_INT8(-4, parsed.uplinkSnrDb);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(RateId::L100), parsed.rfMode);
    TEST_ASSERT_EQUAL_UINT8(kXlrsTelemetryFaultNone, parsed.faultFlags);
}

static void test_core1_config_snapshot_generation_and_ack() {
    handset_core1::SnapshotExchange exchange;
    handset_core1::ConfigSnapshot config = handset_core1::makeDefaultConfigSnapshot();
    config.crsfFrameIntervalUs = 5000;
    config.handsetConfig.axes[0].function = handset_config::ChannelFunction::Aux2;
    config.handsetConfig.channels[5].trim = 17;

    const uint32_t firstGeneration = exchange.publishConfigFromCore0(config);
    TEST_ASSERT_EQUAL_UINT32(1, firstGeneration);
    TEST_ASSERT_EQUAL_UINT32(0, exchange.ackedConfigGeneration());

    uint32_t mailboxSequence = 0;
    handset_core1::ConfigSnapshot loaded{};
    TEST_ASSERT_TRUE(exchange.loadConfigForCore1(loaded, mailboxSequence));
    TEST_ASSERT_EQUAL_UINT32(firstGeneration, loaded.generation);
    TEST_ASSERT_EQUAL_UINT32(5000, loaded.crsfFrameIntervalUs);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_config::ChannelFunction::Aux2),
                            static_cast<uint8_t>(loaded.handsetConfig.axes[0].function));
    TEST_ASSERT_EQUAL_INT16(17, loaded.handsetConfig.channels[5].trim);

    exchange.ackConfigFromCore1(loaded.generation);
    TEST_ASSERT_EQUAL_UINT32(firstGeneration, exchange.ackedConfigGeneration());
    TEST_ASSERT_FALSE(exchange.loadConfigForCore1(loaded, mailboxSequence));

    config.crsfFrameIntervalUs = 0;
    const uint32_t secondGeneration = exchange.publishConfigFromCore0(config);
    TEST_ASSERT_EQUAL_UINT32(2, secondGeneration);
    TEST_ASSERT_TRUE(exchange.loadConfigForCore1(loaded, mailboxSequence));
    TEST_ASSERT_EQUAL_UINT32(secondGeneration, loaded.generation);
    TEST_ASSERT_EQUAL_UINT32(handset_core1::kDefaultCrsfFrameIntervalUs, loaded.crsfFrameIntervalUs);
}

static void test_core1_live_state_snapshot_is_latest_value() {
    handset_core1::SnapshotExchange exchange;
    handset_core1::LiveStateSnapshot state = handset_core1::makeDefaultLiveStateSnapshot();
    state.generation = 1;
    state.appliedConfigGeneration = 7;
    state.framesSent = 11;
    state.channelGuardRejects = 2;
    state.channelSpikeHolds = 3;
    state.haveChannels = true;
    state.channels[0] = kCrsfRaw1000;
    state.channels[7] = kCrsfRaw2000;
    exchange.publishLiveStateFromCore1(state);

    state.generation = 2;
    state.framesSent = 12;
    state.channels[0] = kCrsfRawMid;
    exchange.publishLiveStateFromCore1(state);

    handset_core1::LiveStateSnapshot loaded{};
    TEST_ASSERT_TRUE(exchange.readLiveStateFromCore0(loaded));
    TEST_ASSERT_EQUAL_UINT32(2, loaded.generation);
    TEST_ASSERT_EQUAL_UINT32(7, loaded.appliedConfigGeneration);
    TEST_ASSERT_EQUAL_UINT32(12, loaded.framesSent);
    TEST_ASSERT_EQUAL_UINT32(2, loaded.channelGuardRejects);
    TEST_ASSERT_EQUAL_UINT32(3, loaded.channelSpikeHolds);
    TEST_ASSERT_TRUE(loaded.haveChannels);
    TEST_ASSERT_EQUAL_UINT16(kCrsfRawMid, loaded.channels[0]);
    TEST_ASSERT_EQUAL_UINT16(kCrsfRaw2000, loaded.channels[7]);
}

static void test_handset_adc_calibration_maps_raw_to_crsf_units() {
    const AdcCalibration calibration{100, 2100, 4100};
    TEST_ASSERT_EQUAL_UINT16(kCrsfRaw1000, adcToCalibratedChannel(0, calibration));
    TEST_ASSERT_EQUAL_UINT16(kCrsfRaw1000, adcToCalibratedChannel(100, calibration));
    TEST_ASSERT_EQUAL_UINT16(kCrsfRawMid, adcToCalibratedChannel(2100, calibration));
    TEST_ASSERT_EQUAL_UINT16(kCrsfRaw2000, adcToCalibratedChannel(4100, calibration));
    TEST_ASSERT_EQUAL_UINT16(kCrsfRaw2000, adcToCalibratedChannel(4200, calibration));
}

static void test_handset_deadzone_and_invert_use_crsf_raw_units() {
    TEST_ASSERT_EQUAL_UINT16(kCrsfRawMid, applyDeadzone(static_cast<uint16_t>(kCrsfRawMid + 8), 10));
    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(kCrsfRawMid + 12),
                             applyDeadzone(static_cast<uint16_t>(kCrsfRawMid + 12), 10));
    TEST_ASSERT_EQUAL_UINT16(kCrsfRaw2000, applyInvert(kCrsfRaw1000, true));
    TEST_ASSERT_EQUAL_UINT16(kCrsfRaw1000, applyInvert(kCrsfRaw2000, true));
    TEST_ASSERT_EQUAL_UINT16(1200, applyInvert(1200, false));
}

static void test_handset_channel_function_mapping_places_values_in_crsf_slots() {
    uint16_t channels[kRcChannelCount];
    for (uint8_t i = 0; i < kRcChannelCount; ++i) channels[i] = kCrsfRawMid;

    TEST_ASSERT_TRUE(mapChannelFunction(ChannelFunction::Throttle, 1234, channels));
    TEST_ASSERT_TRUE(mapChannelFunction(ChannelFunction::Aux4, kCrsfRaw2000, channels));
    TEST_ASSERT_FALSE(mapChannelFunction(ChannelFunction::None, 1500, channels));

    TEST_ASSERT_EQUAL_UINT16(1234, channels[3]);
    TEST_ASSERT_EQUAL_UINT16(kCrsfRaw2000, channels[7]);
    TEST_ASSERT_EQUAL_UINT16(kCrsfRawMid, channels[8]);
}

static void test_handset_trim_and_cutoff_are_applied_after_adc_mapping() {
    AnalogInputConfig config{};
    config.function = ChannelFunction::Aileron;
    config.calibration = {0, 2048, 4095};
    config.deadzone = 0;
    config.invert = false;
    config.trim = 60;
    config.cutoff = {static_cast<uint16_t>(kCrsfRawMid - 20), static_cast<uint16_t>(kCrsfRawMid + 40)};

    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(kCrsfRawMid + 40), processAnalogChannel(2048, config));
    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(kCrsfRawMid - 20), applyCutoff(kCrsfRaw1000, config.cutoff));
}

static void test_handset_dual_pin_three_position_toggle_decoding() {
    TEST_ASSERT_EQUAL_UINT16(kCrsfRaw1000, decodeDualPinThreePosition(true, false));
    TEST_ASSERT_EQUAL_UINT16(kCrsfRaw2000, decodeDualPinThreePosition(false, true));
    TEST_ASSERT_EQUAL_UINT16(kCrsfRawMid, decodeDualPinThreePosition(false, false));
    TEST_ASSERT_EQUAL_UINT16(kCrsfRawMid, decodeDualPinThreePosition(true, true));
}

static void test_handset_config_builds_input_pipeline_mapping_and_limits() {
    handset_config::RcHandsetConfig config = handset_config::defaultRcHandsetConfig();
    config.axes[0].function = handset_config::ChannelFunction::Aux2;
    config.axes[0].calibration = {300, 1900, 3700};
    config.axes[0].inverted = true;
    config.axes[0].deadzone = 22;
    config.channels[5].trim = -33;
    config.channels[5].cutoffMin = static_cast<uint16_t>(kCrsfRawMid - 200);
    config.channels[5].cutoffMax = static_cast<uint16_t>(kCrsfRawMid + 200);
    config.filter.spikeJumpThreshold = 123;
    config.filter.spikeConfirmTolerance = 45;

    const InputPipelineConfig pipeline = inputPipelineConfigFromHandsetConfig(config);

    TEST_ASSERT_EQUAL_UINT8(kMaxAnalogInputs, pipeline.analogCount);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ChannelFunction::Aux2),
                            static_cast<uint8_t>(pipeline.analog[0].function));
    TEST_ASSERT_EQUAL_UINT16(300, pipeline.analog[0].calibration.rawMin);
    TEST_ASSERT_EQUAL_UINT16(1900, pipeline.analog[0].calibration.rawMid);
    TEST_ASSERT_EQUAL_UINT16(3700, pipeline.analog[0].calibration.rawMax);
    TEST_ASSERT_TRUE(pipeline.analog[0].invert);
    TEST_ASSERT_EQUAL_UINT16(22, pipeline.analog[0].deadzone);
    TEST_ASSERT_EQUAL_INT16(-33, pipeline.analog[0].trim);
    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(kCrsfRawMid - 200), pipeline.analog[0].cutoff.low);
    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(kCrsfRawMid + 200), pipeline.analog[0].cutoff.high);
    TEST_ASSERT_EQUAL_UINT16(123, pipeline.spikeJumpThreshold);
    TEST_ASSERT_EQUAL_UINT16(45, pipeline.spikeConfirmTolerance);
}

static void test_handset_three_position_toggle_trim_and_cutoff() {
    ThreePositionInputConfig config{};
    config.function = ChannelFunction::Aux1;
    config.trim = 30;
    config.cutoff = {static_cast<uint16_t>(kCrsfRawMid - 100), static_cast<uint16_t>(kCrsfRawMid + 100)};

    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(kCrsfRawMid - 100),
                             processThreePositionChannel(true, false, config));
    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(kCrsfRawMid + 30),
                             processThreePositionChannel(false, false, config));
    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(kCrsfRawMid + 100),
                             processThreePositionChannel(false, true, config));
}

static void test_handset_pipeline_integrates_existing_spike_gate() {
    InputPipelineConfig config = defaultInputPipelineConfig();
    InputPipelineState state = defaultInputPipelineState();
    InputSnapshot snapshot{};
    for (uint8_t i = 0; i < kMaxAnalogInputs; ++i) snapshot.analog[i] = 2048;

    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ProcessResult::Accepted),
                            static_cast<uint8_t>(processInputSnapshot(config, snapshot, state)));
    TEST_ASSERT_TRUE(state.haveChannels);

    snapshot.analog[0] = 4095;
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ProcessResult::HeldBySpikeGate),
                            static_cast<uint8_t>(processInputSnapshot(config, snapshot, state)));
    TEST_ASSERT_TRUE(state.spikeGate.havePending);
    TEST_ASSERT_EQUAL_UINT16(kCrsfRawMid, state.channels[0]);

    snapshot.analog[0] = 4080;
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ProcessResult::Accepted),
                            static_cast<uint8_t>(processInputSnapshot(config, snapshot, state)));
    TEST_ASSERT_FALSE(state.spikeGate.havePending);
    TEST_ASSERT_UINT16_WITHIN(80, kCrsfRaw2000, state.channels[0]);
}

static void test_rc_handset_config_defaults_are_valid() {
    handset_config::RcHandsetConfig cfg = handset_config::defaultRcHandsetConfig();
    TEST_ASSERT_TRUE(handset_config::validateRcHandsetConfig(cfg));
    TEST_ASSERT_EQUAL_UINT16(200, cfg.axes[0].calibration.min);
    TEST_ASSERT_EQUAL_UINT16(2048, cfg.axes[0].calibration.center);
    TEST_ASSERT_EQUAL_UINT16(3895, cfg.axes[0].calibration.max);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_config::ChannelFunction::Aileron),
                            static_cast<uint8_t>(cfg.axes[0].function));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_config::ChannelFunction::Throttle),
                            static_cast<uint8_t>(cfg.axes[3].function));
    TEST_ASSERT_EQUAL_UINT8(8, cfg.filter.adcSamples);
    TEST_ASSERT_EQUAL_UINT8(80, cfg.display.brightnessPercent);
    TEST_ASSERT_EQUAL_UINT16(3300, cfg.power.lowBatteryMv);
}

static void test_rc_handset_config_validation_rejects_invalid_fields() {
    handset_config::RcHandsetConfig cfg = handset_config::defaultRcHandsetConfig();
    cfg.axes[0].calibration.center = cfg.axes[0].calibration.min;
    TEST_ASSERT_FALSE(handset_config::validateRcHandsetConfig(cfg));

    cfg = handset_config::defaultRcHandsetConfig();
    cfg.axes[1].deadzone = 600;
    TEST_ASSERT_FALSE(handset_config::validateRcHandsetConfig(cfg));

    cfg = handset_config::defaultRcHandsetConfig();
    cfg.channels[2].cutoffMin = cfg.channels[2].cutoffMax;
    TEST_ASSERT_FALSE(handset_config::validateRcHandsetConfig(cfg));

    cfg = handset_config::defaultRcHandsetConfig();
    cfg.filter.smoothingPercent = 101;
    TEST_ASSERT_FALSE(handset_config::validateRcHandsetConfig(cfg));
}

static void assert_rc_handset_config_equal(const handset_config::RcHandsetConfig& expected,
                                           const handset_config::RcHandsetConfig& actual) {
    for (uint8_t i = 0; i < handset_config::kRcHandsetAxisCount; ++i) {
        TEST_ASSERT_EQUAL_UINT16(expected.axes[i].calibration.min, actual.axes[i].calibration.min);
        TEST_ASSERT_EQUAL_UINT16(expected.axes[i].calibration.center, actual.axes[i].calibration.center);
        TEST_ASSERT_EQUAL_UINT16(expected.axes[i].calibration.max, actual.axes[i].calibration.max);
        TEST_ASSERT_EQUAL(expected.axes[i].inverted, actual.axes[i].inverted);
        TEST_ASSERT_EQUAL_UINT16(expected.axes[i].deadzone, actual.axes[i].deadzone);
        TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(expected.axes[i].function),
                                static_cast<uint8_t>(actual.axes[i].function));
    }
    for (uint8_t i = 0; i < handset_config::kRcHandsetChannelCount; ++i) {
        TEST_ASSERT_EQUAL_INT16(expected.channels[i].trim, actual.channels[i].trim);
        TEST_ASSERT_EQUAL_UINT16(expected.channels[i].cutoffMin, actual.channels[i].cutoffMin);
        TEST_ASSERT_EQUAL_UINT16(expected.channels[i].cutoffMax, actual.channels[i].cutoffMax);
    }
    TEST_ASSERT_EQUAL_UINT8(expected.filter.adcSamples, actual.filter.adcSamples);
    TEST_ASSERT_EQUAL_UINT8(expected.filter.smoothingPercent, actual.filter.smoothingPercent);
    TEST_ASSERT_EQUAL_UINT16(expected.filter.spikeJumpThreshold, actual.filter.spikeJumpThreshold);
    TEST_ASSERT_EQUAL_UINT16(expected.filter.spikeConfirmTolerance, actual.filter.spikeConfirmTolerance);
    TEST_ASSERT_EQUAL_UINT16(expected.filter.slewLimit, actual.filter.slewLimit);
    TEST_ASSERT_EQUAL_UINT8(expected.display.brightnessPercent, actual.display.brightnessPercent);
    TEST_ASSERT_EQUAL_UINT16(expected.display.timeoutSeconds, actual.display.timeoutSeconds);
    TEST_ASSERT_EQUAL(expected.display.inverted, actual.display.inverted);
    TEST_ASSERT_EQUAL_UINT16(expected.power.autoOffSeconds, actual.power.autoOffSeconds);
    TEST_ASSERT_EQUAL_UINT16(expected.power.lowBatteryMv, actual.power.lowBatteryMv);
    TEST_ASSERT_EQUAL(expected.power.skipPowerOnSequence, actual.power.skipPowerOnSequence);
}

static void test_rc_handset_config_record_round_trips() {
    handset_config::RcHandsetConfig cfg = handset_config::defaultRcHandsetConfig();
    cfg.axes[0].inverted = true;
    cfg.axes[1].deadzone = 24;
    cfg.axes[2].calibration.min = 300;
    cfg.channels[3].trim = -42;
    cfg.channels[4].cutoffMin = 250;
    cfg.channels[4].cutoffMax = 1700;
    cfg.filter.smoothingPercent = 40;
    cfg.display.inverted = true;
    cfg.power.skipPowerOnSequence = true;

    uint8_t record[handset_config::kRcHandsetConfigRecordSize];
    size_t written = 0;
    TEST_ASSERT_TRUE(handset_config::encodeRcHandsetConfigRecord(cfg, record, sizeof(record), written));
    TEST_ASSERT_EQUAL_UINT(handset_config::kRcHandsetConfigRecordSize, written);

    handset_config::RcHandsetConfig out{};
    TEST_ASSERT_TRUE(handset_config::decodeRcHandsetConfigRecord(record, written, out));
    assert_rc_handset_config_equal(cfg, out);
}

static void test_rc_handset_config_record_rejects_crc_corruption() {
    handset_config::RcHandsetConfig cfg = handset_config::defaultRcHandsetConfig();
    uint8_t record[handset_config::kRcHandsetConfigRecordSize];
    size_t written = 0;
    TEST_ASSERT_TRUE(handset_config::encodeRcHandsetConfigRecord(cfg, record, sizeof(record), written));

    record[12] ^= 0x20;
    handset_config::RcHandsetConfig out{};
    TEST_ASSERT_FALSE(handset_config::decodeRcHandsetConfigRecord(record, written, out));
}

static void test_legacy_calibration_migration_valid_record() {
    uint8_t eeprom[handset_migration::kLegacyCalibrationEepromSize] = {};
    const uint16_t min[handset_config::kRcHandsetAxisCount] = {110, 120, 130, 140};
    const uint16_t center[handset_config::kRcHandsetAxisCount] = {2010, 2020, 2030, 2040};
    const uint16_t max[handset_config::kRcHandsetAxisCount] = {3900, 3910, 3920, 3930};
    writeLegacyCalibration(eeprom, min, center, max);

    handset_config::RcHandsetConfig config = handset_config::defaultRcHandsetConfig();
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_migration::LegacyCalibrationMigrationResult::Migrated),
                            static_cast<uint8_t>(handset_migration::migrateLegacyCalibration(
                                eeprom, sizeof(eeprom), config)));
    for (uint8_t axis = 0; axis < handset_config::kRcHandsetAxisCount; ++axis) {
        TEST_ASSERT_EQUAL_UINT16(min[axis], config.axes[axis].calibration.min);
        TEST_ASSERT_EQUAL_UINT16(center[axis], config.axes[axis].calibration.center);
        TEST_ASSERT_EQUAL_UINT16(max[axis], config.axes[axis].calibration.max);
    }
    TEST_ASSERT_TRUE(handset_config::validateRcHandsetConfig(config));
}

static void test_legacy_calibration_migration_missing_magic() {
    uint8_t eeprom[handset_migration::kLegacyCalibrationEepromSize] = {};
    handset_config::RcHandsetConfig config = handset_config::defaultRcHandsetConfig();
    const handset_config::AxisCalibration before = config.axes[0].calibration;

    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_migration::LegacyCalibrationMigrationResult::MissingMagic),
                            static_cast<uint8_t>(handset_migration::migrateLegacyCalibration(
                                eeprom, sizeof(eeprom), config)));
    TEST_ASSERT_EQUAL_UINT16(before.min, config.axes[0].calibration.min);
    TEST_ASSERT_EQUAL_UINT16(before.center, config.axes[0].calibration.center);
    TEST_ASSERT_EQUAL_UINT16(before.max, config.axes[0].calibration.max);
}

static void test_legacy_calibration_migration_rejects_invalid_axis_ranges() {
    uint8_t eeprom[handset_migration::kLegacyCalibrationEepromSize] = {};
    const uint16_t min[handset_config::kRcHandsetAxisCount] = {100, 200, 300, 400};
    const uint16_t center[handset_config::kRcHandsetAxisCount] = {150, 190, 350, 450};
    const uint16_t max[handset_config::kRcHandsetAxisCount] = {200, 300, 400, 500};
    writeLegacyCalibration(eeprom, min, center, max);

    handset_config::RcHandsetConfig config = handset_config::defaultRcHandsetConfig();
    const handset_config::AxisCalibration before = config.axes[1].calibration;
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_migration::LegacyCalibrationMigrationResult::InvalidAxisRange),
                            static_cast<uint8_t>(handset_migration::migrateLegacyCalibration(
                                eeprom, sizeof(eeprom), config)));
    TEST_ASSERT_EQUAL_UINT16(before.min, config.axes[1].calibration.min);
    TEST_ASSERT_EQUAL_UINT16(before.center, config.axes[1].calibration.center);
    TEST_ASSERT_EQUAL_UINT16(before.max, config.axes[1].calibration.max);
}

static void test_legacy_calibration_migration_is_idempotent() {
    uint8_t eeprom[handset_migration::kLegacyCalibrationEepromSize] = {};
    const uint16_t min[handset_config::kRcHandsetAxisCount] = {100, 110, 120, 130};
    const uint16_t center[handset_config::kRcHandsetAxisCount] = {200, 210, 220, 230};
    const uint16_t max[handset_config::kRcHandsetAxisCount] = {300, 310, 320, 330};
    writeLegacyCalibration(eeprom, min, center, max);

    handset_config::RcHandsetConfig config = handset_config::defaultRcHandsetConfig();
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_migration::LegacyCalibrationMigrationResult::Migrated),
                            static_cast<uint8_t>(handset_migration::migrateLegacyCalibration(
                                eeprom, sizeof(eeprom), config)));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_migration::LegacyCalibrationMigrationResult::AlreadyMigrated),
                            static_cast<uint8_t>(handset_migration::migrateLegacyCalibration(
                                eeprom, sizeof(eeprom), config)));
    for (uint8_t axis = 0; axis < handset_config::kRcHandsetAxisCount; ++axis) {
        TEST_ASSERT_EQUAL_UINT16(min[axis], config.axes[axis].calibration.min);
        TEST_ASSERT_EQUAL_UINT16(center[axis], config.axes[axis].calibration.center);
        TEST_ASSERT_EQUAL_UINT16(max[axis], config.axes[axis].calibration.max);
    }
}

static void test_control_frame_round_trip() {
    const uint8_t payload[] = {0x01, 0x02, 0x03};
    uint8_t bytes[control::kMaxFrameSize];
    size_t len = 0;
    TEST_ASSERT_TRUE(control::encodeFrame(static_cast<uint8_t>(control::CommandId::Hello),
                                          payload, sizeof(payload), bytes, sizeof(bytes), len));
    TEST_ASSERT_EQUAL_UINT(control::encodedFrameSize(sizeof(payload)), len);

    control::Frame decoded{};
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(control::DecodeStatus::Ok),
                            static_cast<uint8_t>(control::decodeFrame(bytes, len, decoded)));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(control::CommandId::Hello), decoded.command);
    TEST_ASSERT_EQUAL_UINT8(sizeof(payload), decoded.payloadLen);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, decoded.payload, sizeof(payload));
}

static void test_control_frame_rejects_bad_crc() {
    const uint8_t payload[] = {'o', 'k'};
    uint8_t bytes[control::kMaxFrameSize];
    size_t len = 0;
    TEST_ASSERT_TRUE(control::encodeFrame(static_cast<uint8_t>(control::CommandId::Status),
                                          payload, sizeof(payload), bytes, sizeof(bytes), len));
    bytes[control::kFrameHeaderSize] ^= 0x40;

    control::Frame decoded{};
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(control::DecodeStatus::BadCrc),
                            static_cast<uint8_t>(control::decodeFrame(bytes, len, decoded)));
}

static void test_control_frame_unknown_command_is_ignored() {
    uint8_t bytes[control::kMaxFrameSize];
    size_t len = 0;
    TEST_ASSERT_TRUE(control::encodeFrame(0xE1, nullptr, 0, bytes, sizeof(bytes), len));

    control::Frame decoded{};
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(control::DecodeStatus::Ok),
                            static_cast<uint8_t>(control::decodeFrame(bytes, len, decoded)));
    TEST_ASSERT_FALSE(control::isKnownCommand(decoded.command));

    control::BindingControlState state{};
    control::initBindingControlState(state, "active", "default");
    control::Frame response{};
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(control::BindingControlResult::Ignored),
                            static_cast<uint8_t>(control::handleBindingControlFrame(bytes, len, state, response)));
    TEST_ASSERT_EQUAL_STRING("active", state.bindingPhrase);
}

static void test_control_binding_phrase_validation() {
    uint8_t phrase32[control::kMaxBindingPhraseLen];
    uint8_t phrase33[control::kMaxBindingPhraseLen + 1];
    memset(phrase32, 'a', sizeof(phrase32));
    memset(phrase33, 'b', sizeof(phrase33));

    TEST_ASSERT_FALSE(control::validateBindingPhrase(nullptr, 1));
    TEST_ASSERT_FALSE(control::validateBindingPhrase(phrase32, 0));
    TEST_ASSERT_TRUE(control::validateBindingPhrase(phrase32, sizeof(phrase32)));
    TEST_ASSERT_FALSE(control::validateBindingPhrase(phrase33, sizeof(phrase33)));
    phrase32[4] = 0;
    TEST_ASSERT_FALSE(control::validateBindingPhrase(phrase32, sizeof(phrase32)));
}

static void test_control_binding_set_get_clear_state_machine() {
    control::BindingControlState state{};
    control::initBindingControlState(state, "active", "factory");

    const uint8_t phrase[] = {'n', 'e', 'w', '-', 'b', 'i', 'n', 'd'};
    uint8_t bytes[control::kMaxFrameSize];
    size_t len = 0;
    TEST_ASSERT_TRUE(control::encodeFrame(static_cast<uint8_t>(control::CommandId::BindingSet),
                                          phrase, sizeof(phrase), bytes, sizeof(bytes), len));

    control::Frame response{};
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(control::BindingControlResult::ResponseReady),
                            static_cast<uint8_t>(control::handleBindingControlFrame(bytes, len, state, response)));
    TEST_ASSERT_EQUAL_STRING("new-bind", state.bindingPhrase);
    TEST_ASSERT_TRUE(state.rebootRequired);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(control::CommandId::BindingSet), response.command);

    TEST_ASSERT_TRUE(control::encodeFrame(static_cast<uint8_t>(control::CommandId::BindingClear),
                                          nullptr, 0, bytes, sizeof(bytes), len));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(control::BindingControlResult::ResponseReady),
                            static_cast<uint8_t>(control::handleBindingControlFrame(bytes, len, state, response)));
    TEST_ASSERT_EQUAL_STRING("factory", state.bindingPhrase);
}

static void test_control_status_and_verify_return_uid_payloads() {
    control::BindingControlState state{};
    control::initBindingControlState(state, "active", "factory");

    uint8_t bytes[control::kMaxFrameSize];
    size_t len = 0;
    TEST_ASSERT_TRUE(control::encodeFrame(static_cast<uint8_t>(control::CommandId::Status),
                                          nullptr, 0, bytes, sizeof(bytes), len));

    control::Frame response{};
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(control::BindingControlResult::ResponseReady),
                            static_cast<uint8_t>(control::handleBindingControlFrame(bytes, len, state, response)));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(control::CommandId::Status), response.command);
    TEST_ASSERT_EQUAL_UINT8(control::kBindingIdentityPayloadSize, response.payloadLen);

    uint8_t decodedUid[kUidSize];
    uint8_t expectedUid[kUidSize];
    uint32_t decodedUidCheck = 0;
    deriveUid("active", expectedUid);
    TEST_ASSERT_TRUE(control::decodeBindingIdentityPayload(response.payload, decodedUid, decodedUidCheck));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expectedUid, decodedUid, kUidSize);
    TEST_ASSERT_EQUAL_UINT32(uidCheck(expectedUid), decodedUidCheck);

    const uint8_t candidate[] = {'c', 'a', 'n', 'd', 'i', 'd', 'a', 't', 'e'};
    TEST_ASSERT_TRUE(control::encodeFrame(static_cast<uint8_t>(control::CommandId::BindingVerify),
                                          candidate, sizeof(candidate), bytes, sizeof(bytes), len));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(control::BindingControlResult::ResponseReady),
                            static_cast<uint8_t>(control::handleBindingControlFrame(bytes, len, state, response)));
    deriveUid("candidate", expectedUid);
    TEST_ASSERT_TRUE(control::decodeBindingIdentityPayload(response.payload, decodedUid, decodedUidCheck));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expectedUid, decodedUid, kUidSize);
    TEST_ASSERT_EQUAL_UINT32(uidCheck(expectedUid), decodedUidCheck);
    TEST_ASSERT_EQUAL_STRING("active", state.bindingPhrase);
}

static void test_handset_telemetry_marks_stale_after_timeouts() {
    handset_telemetry::TelemetryTimeouts timeouts{};
    timeouts.localChannelsMs = 50;
    timeouts.txStatusMs = 50;
    timeouts.linkStatsMs = 50;
    timeouts.rxBatteryMs = 50;
    handset_telemetry::HandsetTelemetry model(timeouts);

    uint16_t channels[handset_telemetry::kHandsetTelemetryChannelCount];
    for (uint8_t i = 0; i < handset_telemetry::kHandsetTelemetryChannelCount; ++i) {
        channels[i] = kCrsfRawMid;
    }
    model.updateLocalChannels(channels, 1000);
    model.updateTxStatus(true, true, 0x11223344u, true, 0x11223344u, RateId::L250, 1000);
    model.updateLoRaLink(87, -74, 9, 1000);
    model.updateRxBattery(7400, 82, 1000);

    model.refresh(1051);

    const handset_telemetry::HandsetTelemetryState& state = model.state();
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_telemetry::FieldStatus::Stale),
                            static_cast<uint8_t>(state.localChannelsStatus));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_telemetry::FieldStatus::Stale),
                            static_cast<uint8_t>(state.txStatus));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_telemetry::FieldStatus::Stale),
                            static_cast<uint8_t>(state.linkStatsStatus));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_telemetry::FieldStatus::Stale),
                            static_cast<uint8_t>(state.rxBatteryStatus));
}

static void test_handset_telemetry_updates_lora_link_quality() {
    handset_telemetry::HandsetTelemetry model;
    model.updateLoRaLink(145, -95, -6, 42);

    const handset_telemetry::HandsetTelemetryState& state = model.state();
    TEST_ASSERT_EQUAL_UINT8(100, state.linkQuality);
    TEST_ASSERT_EQUAL_INT16(-95, state.rssiDbm);
    TEST_ASSERT_EQUAL_INT8(-6, state.snrDb);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_telemetry::FieldStatus::Valid),
                            static_cast<uint8_t>(state.linkStatsStatus));
}

static void test_handset_telemetry_flags_binding_uid_mismatch() {
    handset_telemetry::HandsetTelemetry model;
    model.updateTxStatus(true, true, 0x01020304u, true, 0x01020305u, RateId::L100, 10);
    TEST_ASSERT_TRUE(model.state().faults.bindingUidMismatch);
    TEST_ASSERT_FALSE(model.state().faults.bindingUidMissing);

    model.updateTxStatus(true, true, 0x01020304u, true, 0x01020304u, RateId::L100, 20);
    TEST_ASSERT_FALSE(model.state().faults.bindingUidMismatch);
    TEST_ASSERT_FALSE(model.state().faults.bindingUidMissing);

    model.updateTxStatus(true, true, 0x01020304u, false, 0, RateId::L100, 30);
    TEST_ASSERT_FALSE(model.state().faults.bindingUidMismatch);
    TEST_ASSERT_TRUE(model.state().faults.bindingUidMissing);
}

static void test_handset_telemetry_formats_usb_and_oled_state() {
    handset_telemetry::HandsetTelemetry model;
    uint16_t channels[handset_telemetry::kHandsetTelemetryChannelCount];
    for (uint8_t i = 0; i < handset_telemetry::kHandsetTelemetryChannelCount; ++i) {
        channels[i] = static_cast<uint16_t>(kCrsfRaw1000 + i);
    }
    handset_telemetry::PacketCounters counters{};
    counters.uplinkTransmitAttempts = 45;
    counters.uplinkTransmitSuccesses = 42;
    counters.validOtaFrames = 12;
    counters.rejectedOtaFrames = 1;
    counters.downlinkTelemetryFrames = 9;
    counters.lostPackets = 3;

    model.updateLocalChannels(channels, 100);
    model.updateTxStatus(true, true, 0xAABBCCDDu, true, 0x11223344u, RateId::L250, 100);
    model.updateLoRaLink(87, -74, 9, 100);
    model.updatePacketCounters(counters, 100);
    model.updateRxBattery(7400, 82, 100);
    model.setConfigFault(true, 100);

    char usb[320];
    char oled[96];
    TEST_ASSERT_TRUE(handset_telemetry::formatUsbStateResponse(model.state(), usb, sizeof(usb)));
    TEST_ASSERT_TRUE(handset_telemetry::formatOledDisplay(model.state(), oled, sizeof(oled)));

    TEST_ASSERT_NOT_NULL(strstr(usb, "tx=present"));
    TEST_ASSERT_NOT_NULL(strstr(usb, "rate=L250"));
    TEST_ASSERT_NOT_NULL(strstr(usb, "uid=mismatch"));
    TEST_ASSERT_NOT_NULL(strstr(usb, "lq=87"));
    TEST_ASSERT_NOT_NULL(strstr(usb, "rx_batt=7400mV/82%:valid"));
    TEST_ASSERT_NOT_NULL(strstr(usb, "faults=config,binding_mismatch"));
    TEST_ASSERT_NOT_NULL(strstr(oled, "TX OK L250"));
    TEST_ASSERT_NOT_NULL(strstr(oled, "LQ 87 RSSI -74"));
    TEST_ASSERT_NOT_NULL(strstr(oled, "FAULT CONFIG"));
}

static void test_display_renderer_boot_progress_text() {
    handset_display::DisplayState state{};
    state.screen = handset_display::Screen::Boot;
    state.bootProgressPercent = 45;
    state.bootMessage = "Inputs init";

    handset_display::DisplayLayout layout{};
    handset_display::renderDisplayLayout(state, layout);

    TEST_ASSERT_EQUAL_UINT8(4, layout.lineCount);
    TEST_ASSERT_EQUAL_STRING("XLRS RC HANDSET", layout.lines[0]);
    TEST_ASSERT_NOT_NULL(strstr(layout.lines[2], "45%"));
    TEST_ASSERT_EQUAL_STRING("Inputs init", layout.lines[3]);
}

static void test_display_renderer_main_status_metrics_and_batteries() {
    handset_display::DisplayState state{};
    state.screen = handset_display::Screen::MainStatus;
    state.link.connected = true;
    state.link.metricsAvailable = true;
    state.link.rateName = "L250";
    state.link.linkQuality = 93;
    state.link.rssiDbm = -71;
    state.link.snrDb = 8;
    state.txBattery = {true, 4080, 86};
    state.rxBattery = {true, 7400, 77};

    handset_display::DisplayLayout layout{};
    handset_display::renderDisplayLayout(state, layout);

    TEST_ASSERT_NOT_NULL(strstr(layout.lines[0], "LINK"));
    TEST_ASSERT_NOT_NULL(strstr(layout.lines[1], "4.08V 86%"));
    TEST_ASSERT_NOT_NULL(strstr(layout.lines[2], "7.40V 77%"));
    TEST_ASSERT_NOT_NULL(strstr(layout.lines[3], "L250"));
    TEST_ASSERT_NOT_NULL(strstr(layout.lines[4], "LQ  93 RSSI  -71"));
    TEST_ASSERT_NOT_NULL(strstr(layout.lines[5], "SNR    8 dB"));
}

static void test_display_renderer_channel_monitor_and_toggles() {
    handset_display::DisplayState state{};
    state.screen = handset_display::Screen::ChannelMonitor;
    state.channels.channelsAvailable = true;
    state.channels.aileron = 1000;
    state.channels.elevator = 1500;
    state.channels.rudder = 2000;
    state.channels.throttle = 1234;
    state.channels.toggles[0] = 191;
    state.channels.toggles[1] = 992;
    state.channels.toggles[2] = 1792;
    state.channels.toggles[3] = 992;

    handset_display::DisplayLayout layout{};
    handset_display::renderDisplayLayout(state, layout);

    TEST_ASSERT_EQUAL_STRING("CHANNEL MONITOR", layout.lines[0]);
    TEST_ASSERT_NOT_NULL(strstr(layout.lines[1], "A 1000 E 1500"));
    TEST_ASSERT_NOT_NULL(strstr(layout.lines[2], "R 2000 T 1234"));
    TEST_ASSERT_EQUAL_STRING("Tog L M H M", layout.lines[3]);
}

static void test_display_renderer_config_and_binding_status() {
    handset_display::DisplayState config{};
    config.screen = handset_display::Screen::ConfigSummary;
    config.config.rateName = "CRSF";
    config.config.bindingPhrase = "bench";
    config.config.calibrationValid = true;
    config.config.framesSent = 123;
    config.config.guardRejects = 2;
    config.config.spikeHolds = 3;

    handset_display::DisplayLayout layout{};
    handset_display::renderDisplayLayout(config, layout);
    TEST_ASSERT_EQUAL_STRING("CONFIG SUMMARY", layout.lines[0]);
    TEST_ASSERT_EQUAL_STRING("Rate CRSF", layout.lines[1]);
    TEST_ASSERT_EQUAL_STRING("Bind bench", layout.lines[2]);
    TEST_ASSERT_EQUAL_STRING("Cal OK", layout.lines[3]);
    TEST_ASSERT_EQUAL_STRING("Frames 123", layout.lines[4]);
    TEST_ASSERT_EQUAL_STRING("Guard 2 Hold 3", layout.lines[5]);

    handset_display::DisplayState binding{};
    binding.screen = handset_display::Screen::BindingStatus;
    binding.binding.mismatch = true;
    binding.binding.message = "Receiver UID differs";
    handset_display::renderDisplayLayout(binding, layout);
    TEST_ASSERT_EQUAL_STRING("BINDING STATUS", layout.lines[0]);
    TEST_ASSERT_EQUAL_STRING("UID MISMATCH", layout.lines[1]);
    TEST_ASSERT_EQUAL_STRING("Receiver UID differs", layout.lines[2]);
}

static void test_power_battery_percent_clamps_and_interpolates() {
    TEST_ASSERT_EQUAL_UINT8(0, handset_power::batteryPercentFromVoltageMv(3200));
    TEST_ASSERT_EQUAL_UINT8(0, handset_power::batteryPercentFromVoltageMv(3300));
    TEST_ASSERT_EQUAL_UINT8(50, handset_power::batteryPercentFromVoltageMv(3750));
    TEST_ASSERT_EQUAL_UINT8(100, handset_power::batteryPercentFromVoltageMv(4200));
    TEST_ASSERT_EQUAL_UINT8(100, handset_power::batteryPercentFromVoltageMv(4300));
}

static void test_power_battery_percent_handles_invalid_range() {
    TEST_ASSERT_EQUAL_UINT8(0, handset_power::batteryPercentFromVoltageMv(3700, 4200, 3300));
    TEST_ASSERT_EQUAL_UINT8(0, handset_power::batteryPercentFromVoltageMv(3700, 3300, 3300));
}

static void test_power_bq2562x_vbat_decode_uses_datasheet_lsb() {
    const uint16_t shiftedAdc = 2000;
    const uint16_t rawRegister = static_cast<uint16_t>(shiftedAdc << 1);
    TEST_ASSERT_EQUAL_UINT16(3980, handset_power::Bq2562x::decodeBatteryVoltageMv(rawRegister));
}

static void test_power_hold_to_boot_satisfies_after_required_time() {
    handset_power::PowerButtonController power({true, 5000, 2500});
    power.resetBootGate(1000);

    handset_power::PowerButtonUpdate update = power.updateBootGate(true, 3000);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_power::BootGateState::Waiting),
                            static_cast<uint8_t>(update.bootGateState));
    TEST_ASSERT_EQUAL_UINT8(40, update.progressPercent);

    update = power.updateBootGate(true, 6000);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_power::PowerButtonEvent::BootHoldSatisfied),
                            static_cast<uint8_t>(update.event));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_power::BootGateState::Satisfied),
                            static_cast<uint8_t>(update.bootGateState));
    TEST_ASSERT_EQUAL_UINT8(100, update.progressPercent);
}

static void test_power_hold_to_boot_aborts_on_release() {
    handset_power::PowerButtonController power({true, 5000, 2500});
    power.resetBootGate(10);

    handset_power::PowerButtonUpdate update = power.updateBootGate(false, 20);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_power::PowerButtonEvent::BootHoldAborted),
                            static_cast<uint8_t>(update.event));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_power::BootGateState::Aborted),
                            static_cast<uint8_t>(update.bootGateState));
}

static void test_power_off_requires_release_after_long_press() {
    handset_power::PowerButtonController power({false, 5000, 2500});

    handset_power::PowerButtonUpdate update = power.updatePowerOff(true, 100);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_power::PowerButtonEvent::PowerOffStarted),
                            static_cast<uint8_t>(update.event));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_power::PowerOffState::Holding),
                            static_cast<uint8_t>(update.powerOffState));

    update = power.updatePowerOff(true, 2600);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_power::PowerButtonEvent::PowerOffReady),
                            static_cast<uint8_t>(update.event));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_power::PowerOffState::ReadyToRelease),
                            static_cast<uint8_t>(update.powerOffState));
    TEST_ASSERT_EQUAL_UINT8(100, update.progressPercent);

    update = power.updatePowerOff(false, 2700);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_power::PowerButtonEvent::PowerOffConfirmed),
                            static_cast<uint8_t>(update.event));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_power::PowerOffState::Idle),
                            static_cast<uint8_t>(update.powerOffState));
}

static void test_power_off_short_press_cancels() {
    handset_power::PowerButtonController power({false, 5000, 2500});
    (void)power.updatePowerOff(true, 100);

    handset_power::PowerButtonUpdate update = power.updatePowerOff(false, 1000);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_power::PowerButtonEvent::PowerOffCancelled),
                            static_cast<uint8_t>(update.event));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(handset_power::PowerOffState::Idle),
                            static_cast<uint8_t>(update.powerOffState));
}

void run_power_tests() {
    RUN_TEST(test_power_battery_percent_clamps_and_interpolates);
    RUN_TEST(test_power_battery_percent_handles_invalid_range);
    RUN_TEST(test_power_bq2562x_vbat_decode_uses_datasheet_lsb);
    RUN_TEST(test_power_hold_to_boot_satisfies_after_required_time);
    RUN_TEST(test_power_hold_to_boot_aborts_on_release);
    RUN_TEST(test_power_off_requires_release_after_long_press);
    RUN_TEST(test_power_off_short_press_cancels);
}

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_uid_is_deterministic);
    RUN_TEST(test_sync_word_is_eight_bit_and_guards_effective_zero);
    RUN_TEST(test_uid_check_uses_all_uid_bytes);
    RUN_TEST(test_crc_known_answer_vectors);
    RUN_TEST(test_fhss_depends_on_uid_and_permutates_channels);
    RUN_TEST(test_fhss_frequency_uses_named_channel_count);
    RUN_TEST(test_fhss_is_enabled_by_default);
    RUN_TEST(test_hop_alignment_tracks_received_sequence_for_reacquisition);
    RUN_TEST(test_rf_scheduler_tx_advances_nonce_and_fhss_on_tx_done);
    RUN_TEST(test_rf_scheduler_tx_consumes_telemetry_slot_without_rc_frame);
    RUN_TEST(test_rf_scheduler_tx_sync_scheduling_metadata);
    RUN_TEST(test_rf_scheduler_rx_relocks_fhss_from_received_sequence);
    RUN_TEST(test_rf_scheduler_rx_timer_miss_advances_expected_slot);
    RUN_TEST(test_rf_scheduler_rx_connected_miss_requests_retune_on_hop_boundary);
    RUN_TEST(test_rf_scheduler_rx_acquires_timing_from_sync_frame);
    RUN_TEST(test_rf_scheduler_rx_acquires_directly_from_acquisition_rc_frame);
    RUN_TEST(test_rx_pfd_reports_external_minus_internal_offset);
    RUN_TEST(test_rf_scheduler_rx_pfd_lpf_offset_and_derivative);
    RUN_TEST(test_rf_scheduler_rx_tentative_phase_shift_uses_raw_offset);
    RUN_TEST(test_rf_scheduler_rx_connected_phase_shift_uses_filtered_offset_quarter);
    RUN_TEST(test_rf_scheduler_rx_promotes_with_stable_negative_offset_like_elrs);
    RUN_TEST(test_rf_scheduler_rx_locks_only_after_dwell_and_low_derivative);
    RUN_TEST(test_rf_scheduler_rx_locked_frequency_trim_runs_every_8_nonces);
    RUN_TEST(test_rf_scheduler_rx_tick_tock_event_sequencing);
    RUN_TEST(test_rf_scheduler_rx_rejects_invalid_sync_metadata);
    RUN_TEST(test_rf_scheduler_rx_disconnected_scan_stays_on_sync_channel);
    RUN_TEST(test_ota_sync_payload_round_trip_and_validation);
    RUN_TEST(test_ota_round_trip_and_rejects_corruption);
    RUN_TEST(test_ota_decode_rejects_wrong_uid_length_and_type);
    RUN_TEST(test_ota_carries_full_crsf_payload_without_corrupting_aux12);
    RUN_TEST(test_crsf_channel_round_trip);
    RUN_TEST(test_channel_sanitizer_clamps_without_rejecting_normal_frame);
    RUN_TEST(test_channel_sanitizer_rejects_simultaneous_high_endpoint_spike);
    RUN_TEST(test_channel_sanitizer_accepts_single_fast_channel_move);
    RUN_TEST(test_spike_gate_holds_one_frame_channel_outlier);
    RUN_TEST(test_spike_gate_accepts_confirmed_large_channel_step);
    RUN_TEST(test_spike_gate_accepts_fast_continuous_ramp);
    RUN_TEST(test_radio_transmitter_address_frame_is_accepted_as_crsf_raw);
    RUN_TEST(test_config_record_validation);
    RUN_TEST(test_binding_phrase_validation);
    RUN_TEST(test_binding_status_derives_uid_check);
    RUN_TEST(test_crsf_binding_request_and_response_frames);
    RUN_TEST(test_crsf_binding_request_encoder_and_response_parser);
    RUN_TEST(test_failsafe_freshness_helper_uses_named_timeout);
    RUN_TEST(test_telemetry_listen_window_is_short_and_scheduled_by_ratio);
    RUN_TEST(test_crsf_link_stats_puts_rate_in_rf_mode_not_power);
    RUN_TEST(test_downlink_telemetry_payload_round_trips_diagnostics);
    RUN_TEST(test_downlink_telemetry_decoder_accepts_legacy_four_byte_payload);
    RUN_TEST(test_core1_config_snapshot_generation_and_ack);
    RUN_TEST(test_core1_live_state_snapshot_is_latest_value);
    RUN_TEST(test_handset_adc_calibration_maps_raw_to_crsf_units);
    RUN_TEST(test_handset_deadzone_and_invert_use_crsf_raw_units);
    RUN_TEST(test_handset_channel_function_mapping_places_values_in_crsf_slots);
    RUN_TEST(test_handset_trim_and_cutoff_are_applied_after_adc_mapping);
    RUN_TEST(test_handset_dual_pin_three_position_toggle_decoding);
    RUN_TEST(test_handset_config_builds_input_pipeline_mapping_and_limits);
    RUN_TEST(test_handset_three_position_toggle_trim_and_cutoff);
    RUN_TEST(test_handset_pipeline_integrates_existing_spike_gate);
    RUN_TEST(test_rc_handset_config_defaults_are_valid);
    RUN_TEST(test_rc_handset_config_validation_rejects_invalid_fields);
    RUN_TEST(test_rc_handset_config_record_round_trips);
    RUN_TEST(test_rc_handset_config_record_rejects_crc_corruption);
    RUN_TEST(test_legacy_calibration_migration_valid_record);
    RUN_TEST(test_legacy_calibration_migration_missing_magic);
    RUN_TEST(test_legacy_calibration_migration_rejects_invalid_axis_ranges);
    RUN_TEST(test_legacy_calibration_migration_is_idempotent);
    RUN_TEST(test_control_frame_round_trip);
    RUN_TEST(test_control_frame_rejects_bad_crc);
    RUN_TEST(test_control_frame_unknown_command_is_ignored);
    RUN_TEST(test_control_binding_phrase_validation);
    RUN_TEST(test_control_binding_set_get_clear_state_machine);
    RUN_TEST(test_control_status_and_verify_return_uid_payloads);
    RUN_TEST(test_handset_telemetry_marks_stale_after_timeouts);
    RUN_TEST(test_handset_telemetry_updates_lora_link_quality);
    RUN_TEST(test_handset_telemetry_flags_binding_uid_mismatch);
    RUN_TEST(test_handset_telemetry_formats_usb_and_oled_state);
    RUN_TEST(test_display_renderer_boot_progress_text);
    RUN_TEST(test_display_renderer_main_status_metrics_and_batteries);
    RUN_TEST(test_display_renderer_channel_monitor_and_toggles);
    RUN_TEST(test_display_renderer_config_and_binding_status);
    run_rc_usb_protocol_tests();
    run_power_tests();
    return UNITY_END();
}
