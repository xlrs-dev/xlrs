#include "lora_link/rf/scheduler.h"

#include <string.h>

namespace lora_link {
namespace rf {

static constexpr uint32_t kPacketToTockSlackUs = 200;
static constexpr int32_t kConnectedOffsetLimitUs = 100;
static constexpr int32_t kConnectedOffsetDxLimitUs = 10;
static constexpr int32_t kLockedOffsetDxLimitUs = 5;
static constexpr uint32_t kLockDwellMs = 1000;

static int32_t abs32(int32_t value) {
    return value < 0 ? -value : value;
}

static uint32_t estimatedOtaFrameToaUs(const RateConfig& rate) {
    const uint32_t bandwidthHz = static_cast<uint32_t>(rate.bandwidthKHz * 1000.0f);
    if (bandwidthHz == 0 || rate.spreadingFactor == 0) return 0;

    const uint32_t symbolUs = ((1u << rate.spreadingFactor) * 1000000u + bandwidthHz - 1u) / bandwidthHz;
    const uint32_t preambleUs = (65u * symbolUs + 3u) / 4u; // 12-symbol preamble + 4.25 symbols.
    const uint32_t payloadBits = 8u * kOtaFrameSize;
    const int32_t payloadNumerator = static_cast<int32_t>(payloadBits) -
                                     static_cast<int32_t>(4u * rate.spreadingFactor) + 44;
    const uint32_t payloadDenominator = 4u * rate.spreadingFactor;
    const uint32_t payloadBlocks = payloadNumerator <= 0
                                       ? 0
                                       : (static_cast<uint32_t>(payloadNumerator) + payloadDenominator - 1u) /
                                             payloadDenominator;
    const uint32_t codingRateSymbols = rate.codingRate ? rate.codingRate : 5;
    const uint32_t payloadSymbols = 8u + payloadBlocks * codingRateSymbols;
    return preambleUs + payloadSymbols * symbolUs;
}

static uint32_t packetToTockSlackUs(const RateConfig& rate) {
    const uint32_t toaUs = estimatedOtaFrameToaUs(rate);
    const uint32_t computedSlack = rate.intervalUs > 2u * toaUs ? rate.intervalUs - 2u * toaUs : 0;
    return computedSlack > kPacketToTockSlackUs ? computedSlack : kPacketToTockSlackUs;
}

void PhaseFrequencyDetector::extEvent(uint32_t atUs) {
    externalUs_ = atUs;
    haveExternal_ = true;
    if (haveInternal_) {
        result_ = static_cast<int32_t>(externalUs_ - internalUs_);
        hasResult_ = true;
    }
}

void PhaseFrequencyDetector::intEvent(uint32_t atUs) {
    internalUs_ = atUs;
    haveInternal_ = true;
    if (haveExternal_) {
        result_ = static_cast<int32_t>(externalUs_ - internalUs_);
        hasResult_ = true;
    }
}

void PhaseFrequencyDetector::reset() {
    haveExternal_ = false;
    haveInternal_ = false;
    hasResult_ = false;
    externalUs_ = 0;
    internalUs_ = 0;
    result_ = 0;
}

void FhssState::begin(const uint8_t uid[kUidSize], uint8_t hopInterval) {
    memcpy(uid_, uid, kUidSize);
    hopInterval_ = hopInterval ? hopInterval : 1;
    index_ = 0;
}

void FhssState::reset() {
    index_ = 0;
}

void FhssState::setCurrIndex(uint16_t index) {
    index_ = static_cast<uint16_t>(index % kFhssChannelCount);
}

void FhssState::setFromSequence(uint16_t sequence) {
    setCurrIndex(static_cast<uint16_t>(sequence / hopInterval_));
}

void FhssState::advanceAfterSequence(uint16_t sequence) {
    setFromSequence(static_cast<uint16_t>(sequence + 1));
}

void FhssState::advanceIndex() {
    setCurrIndex(static_cast<uint16_t>(index_ + 1));
}

uint8_t FhssState::channel() const {
    if (kFixedRfChannel >= 0) return static_cast<uint8_t>(kFixedRfChannel);
    return fhssChannelFor(uid_, index_);
}

float FhssState::frequencyMHz() const {
    return fhssFrequencyMHz(channel());
}

void TxScheduler::begin(const uint8_t uid[kUidSize], const RateConfig& rate) {
    rate_ = rate;
    stats_ = {};
    sequence_ = 0;
    telemetryListenPending_ = false;
    telemetryAdvancePending_ = false;
    fhss_.begin(uid, rate_.fhssHopInterval);
}

bool TxScheduler::onTimerTick() {
    ++stats_.timerTicks;
    stats_.nonce = sequence_;
    stats_.fhssIndex = fhss_.index();
    if (telemetryListenPending_) {
        telemetryListenPending_ = false;
        telemetryAdvancePending_ = true;
        stats_.nonce = sequence_;
        stats_.fhssIndex = fhss_.index();
        return false;
    }
    if (telemetryAdvancePending_) {
        telemetryAdvancePending_ = false;
        advanceTock();
    }
    stats_.nonce = sequence_;
    stats_.fhssIndex = fhss_.index();
    return true;
}

void TxScheduler::onTxDone() {
    ++stats_.txDone;
    if (shouldRequestTelemetry(static_cast<uint16_t>(sequence_ + 1), rate_.telemetryRatio)) {
        telemetryListenPending_ = true;
    } else {
        advanceTock();
    }
    stats_.nonce = sequence_;
    stats_.fhssIndex = fhss_.index();
}

void TxScheduler::onTelemetryDone() {
    telemetryListenPending_ = false;
}

bool TxScheduler::shouldSendSyncFrame() const {
    return fhss_.index() == kSyncChannelFhssIndex &&
           (sequence_ % kSyncFrameInterval) == 0;
}

OtaSyncPayload TxScheduler::syncPayload() const {
    OtaSyncPayload payload{};
    payload.nonce = sequence_;
    payload.fhssIndex = fhss_.index();
    payload.rateIndex = rateIndexForConfig(rate_);
    payload.nextRateIndex = payload.rateIndex;
    payload.switchSequence = 0;
    payload.telemetryRatio = rate_.telemetryRatio;
    payload.fhssHopInterval = rate_.fhssHopInterval;
    payload.txTick = sequence_;
    return payload;
}

void TxScheduler::advanceTock() {
    ++sequence_;
    if ((sequence_ % rate_.fhssHopInterval) == 0) {
        fhss_.advanceIndex();
    }
}

SchedulerStats TxScheduler::stats() const {
    SchedulerStats out = stats_;
    out.nonce = sequence_;
    out.fhssIndex = fhss_.index();
    out.connectionState = ConnectionState::Connected;
    return out;
}

void RxScheduler::begin(const uint8_t uid[kUidSize], const RateConfig& rate) {
    rate_ = rate;
    stats_ = {};
    nextExpectedSequence_ = 0;
    disconnectedScanTick_ = 0;
    haveSequence_ = false;
    receivedFrameForCurrentTock_ = false;
    consecutiveRcFrames_ = 0;
    connectedSinceMs_ = 0;
    timerState_ = RxTimerState::TimDisconnected;
    offsetUs_ = 0;
    offsetDxUs_ = 0;
    previousRawOffsetUs_ = 0;
    frequencyOffsetUs_ = 0;
    pfd_.reset();
    fhss_.begin(uid, rate_.fhssHopInterval);
}

RxTimerTickEvent RxScheduler::onTimerTick(bool linkFresh) {
    return onTimerEvent(RxTimerHalfEvent::Tock, 0, linkFresh, 0).event;
}

RxTimerEventResult RxScheduler::onTimerEvent(RxTimerHalfEvent half, uint32_t eventUs,
                                             bool linkFresh, uint32_t nowMs) {
    const uint8_t previousChannel = fhss_.channel();
    RxTimerEventResult result{};
    ++stats_.timerTicks;

    if (half == RxTimerHalfEvent::Tick) {
        stats_.nonce = nextExpectedSequence_;
        stats_.fhssIndex = fhss_.index();
        stats_.rxTimerState = timerState_;
        stats_.offsetUs = offsetUs_;
        stats_.offsetDxUs = offsetDxUs_;
        stats_.frequencyOffsetUs = frequencyOffsetUs_;
        return result;
    }

    pfd_.intEvent(eventUs);
    ++stats_.pfdInternalEvents;
    if (!linkFresh) {
        lostConnection();
    } else if (haveSequence_) {
        disconnectedScanTick_ = 0;
        if (!receivedFrameForCurrentTock_) {
            ++stats_.missedRcFrames;
            consecutiveRcFrames_ = 0;
        }
        advanceTock();
        receivedFrameForCurrentTock_ = false;
        result = updatePhaseLock(nowMs);
    }
    stats_.nonce = nextExpectedSequence_;
    stats_.fhssIndex = fhss_.index();
    stats_.rxTimerState = timerState_;
    stats_.offsetUs = offsetUs_;
    stats_.offsetDxUs = offsetDxUs_;
    stats_.frequencyOffsetUs = frequencyOffsetUs_;
    result.event = fhss_.channel() != previousChannel ? RxTimerTickEvent::ReceiveFrequencyChanged
                                                      : RxTimerTickEvent::None;
    return result;
}

bool RxScheduler::onValidSyncFrame(const OtaSyncPayload& sync, uint32_t beginProcessingUs) {
    const uint8_t rateIndex = rateIndexForConfig(rate_);
    if (!isValidRateIndex(sync.rateIndex) || sync.rateIndex != rateIndex ||
        !isValidRateIndex(sync.nextRateIndex) ||
        sync.telemetryRatio != rate_.telemetryRatio ||
        sync.fhssHopInterval != rate_.fhssHopInterval ||
        sync.fhssIndex >= kFhssChannelCount) {
        return false;
    }

    haveSequence_ = true;
    receivedFrameForCurrentTock_ = true;
    ++stats_.syncAccepted;
    consecutiveRcFrames_ = 0;
    disconnectedScanTick_ = 0;
    nextExpectedSequence_ = sync.nonce;
    fhss_.setCurrIndex(sync.fhssIndex);
    pfd_.extEvent(packetTockReferenceUs(beginProcessingUs));
    ++stats_.pfdExternalEvents;
    stats_.nonce = nextExpectedSequence_;
    stats_.fhssIndex = fhss_.index();
    stats_.connectionState = ConnectionState::Tentative;
    stats_.rxTimerState = timerState_;
    return true;
}

bool RxScheduler::onValidRcFrame(uint16_t sequence, uint32_t beginProcessingUs, uint32_t nowMs) {
    if (!haveSequence_) {
        ++stats_.rcRejectedNoSync;
        return false;
    }
    ++stats_.rxDone;
    if (sequence != nextExpectedSequence_) {
        const uint16_t missed = static_cast<uint16_t>(sequence - nextExpectedSequence_);
        if (missed < 1000) stats_.missedRcFrames += missed;
        nextExpectedSequence_ = sequence;
        fhss_.setFromSequence(sequence);
    }
    pfd_.extEvent(packetTockReferenceUs(beginProcessingUs));
    ++stats_.pfdExternalEvents;
    haveSequence_ = true;
    receivedFrameForCurrentTock_ = true;
    disconnectedScanTick_ = 0;
    ++consecutiveRcFrames_;
    if (consecutiveRcFrames_ > stats_.maxConsecutiveRcFrames) {
        stats_.maxConsecutiveRcFrames = consecutiveRcFrames_;
    }
    if (stats_.connectionState == ConnectionState::Tentative) {
        ++stats_.promotionChecks;
        const bool hasEnoughFrames = true;
        const bool offsetOk = offsetUs_ < kConnectedOffsetLimitUs;
        const bool dxOk = abs32(offsetDxUs_) <= kConnectedOffsetDxLimitUs;
        if (!hasEnoughFrames) ++stats_.promotionFrameBlocks;
        if (!offsetOk) ++stats_.promotionOffsetBlocks;
        if (!dxOk) ++stats_.promotionDxBlocks;
        if (hasEnoughFrames && offsetOk && dxOk) {
            stats_.connectionState = ConnectionState::Connected;
            timerState_ = RxTimerState::TimTentative;
            connectedSinceMs_ = nowMs;
            ++stats_.connectedPromotions;
        }
    }
    stats_.nonce = nextExpectedSequence_;
    stats_.fhssIndex = fhss_.index();
    stats_.rxTimerState = timerState_;
    return true;
}

void RxScheduler::onTelemetryDone() {
    // Telemetry consumes the current response slot; the next uplink remains keyed by sequence.
}

void RxScheduler::setConnected(bool connected) {
    if (connected) {
        stats_.connectionState = ConnectionState::Connected;
        if (timerState_ == RxTimerState::TimDisconnected) timerState_ = RxTimerState::TimTentative;
    } else {
        lostConnection();
    }
}

uint32_t RxScheduler::packetTockReferenceUs(uint32_t beginProcessingUs) const {
    return beginProcessingUs + packetToTockSlackUs(rate_);
}

void RxScheduler::advanceTock() {
    ++nextExpectedSequence_;
    if ((nextExpectedSequence_ % rate_.fhssHopInterval) == 0) {
        fhss_.advanceIndex();
    }
}

void RxScheduler::lostConnection() {
    ++stats_.lostConnections;
    if (stats_.connectionState == ConnectionState::Tentative ||
        stats_.connectionState == ConnectionState::Connected) {
        ++stats_.tentativeLostConnections;
    }
    stats_.connectionState = ConnectionState::Disconnected;
    timerState_ = RxTimerState::TimDisconnected;
    haveSequence_ = false;
    receivedFrameForCurrentTock_ = false;
    consecutiveRcFrames_ = 0;
    connectedSinceMs_ = 0;
    nextExpectedSequence_ = 0;
    disconnectedScanTick_ = 0;
    offsetUs_ = 0;
    offsetDxUs_ = 0;
    previousRawOffsetUs_ = 0;
    frequencyOffsetUs_ = 0;
    pfd_.reset();
    fhss_.reset();
}

RxTimerEventResult RxScheduler::updatePhaseLock(uint32_t nowMs) {
    RxTimerEventResult result{};
    if (stats_.connectionState == ConnectionState::Disconnected || !pfd_.hasResult()) return result;

    const int32_t rawOffsetUs = pfd_.result();
    ++stats_.pfdResults;
    offsetUs_ = (offsetUs_ * 3 + rawOffsetUs) / 4;
    offsetDxUs_ = (offsetDxUs_ * 15 + (rawOffsetUs - previousRawOffsetUs_)) / 16;
    previousRawOffsetUs_ = rawOffsetUs;
    stats_.phaseErrorUs = rawOffsetUs;

    if (timerState_ == RxTimerState::TimLocked && (nextExpectedSequence_ % 8u) == 0u) {
        if (offsetUs_ > 0) {
            ++frequencyOffsetUs_;
        } else if (offsetUs_ < 0) {
            --frequencyOffsetUs_;
        }
    }

    if (stats_.connectionState == ConnectionState::Connected &&
        timerState_ == RxTimerState::TimTentative &&
        connectedSinceMs_ != 0 &&
        static_cast<uint32_t>(nowMs - connectedSinceMs_) > kLockDwellMs &&
        abs32(offsetDxUs_) <= kLockedOffsetDxLimitUs) {
        timerState_ = RxTimerState::TimLocked;
        ++stats_.lockedPromotions;
    }

    result.hasPhaseShift = true;
    result.phaseShiftUs = stats_.connectionState == ConnectionState::Connected ? (offsetUs_ >> 2)
                                                                               : rawOffsetUs;
    result.frequencyOffsetUs = frequencyOffsetUs_;
    ++stats_.phaseShiftRequests;
    stats_.lastPhaseShiftUs = result.phaseShiftUs;
    pfd_.reset();
    return result;
}

SchedulerStats RxScheduler::stats() const {
    SchedulerStats out = stats_;
    out.nonce = nextExpectedSequence_;
    out.fhssIndex = fhss_.index();
    out.rxTimerState = timerState_;
    out.offsetUs = offsetUs_;
    out.offsetDxUs = offsetDxUs_;
    out.frequencyOffsetUs = frequencyOffsetUs_;
    return out;
}

} // namespace rf
} // namespace lora_link
