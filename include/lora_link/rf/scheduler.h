#pragma once

#include <stdint.h>

#include "lora_link/protocol.h"

namespace lora_link {
namespace rf {

constexpr uint16_t kSyncChannelFhssIndex = 0;
constexpr uint16_t kSyncFrameInterval = 32;

enum class ConnectionState : uint8_t {
    Disconnected = 0,
    Tentative = 1,
    Connected = 2,
};

enum class RxTimerState : uint8_t {
    TimDisconnected = 0,
    TimTentative = 1,
    TimLocked = 2,
};

enum class RxTimerHalfEvent : uint8_t {
    Tick = 0,
    Tock = 1,
};

enum class RxTimerTickEvent : uint8_t {
    None = 0,
    ReceiveFrequencyChanged = 1,
};

struct RxTimerEventResult {
    RxTimerTickEvent event = RxTimerTickEvent::None;
    bool hasPhaseShift = false;
    int32_t phaseShiftUs = 0;
    int32_t frequencyOffsetUs = 0;
};

struct SchedulerStats {
    uint32_t timerTicks = 0;
    uint32_t missedTimerTicks = 0;
    uint32_t txDone = 0;
    uint32_t rxDone = 0;
    uint32_t missedRcFrames = 0;
    uint32_t syncAccepted = 0;
    uint32_t rcRejectedNoSync = 0;
    uint32_t pfdExternalEvents = 0;
    uint32_t pfdInternalEvents = 0;
    uint32_t pfdResults = 0;
    uint32_t lostConnections = 0;
    uint32_t connectedPromotions = 0;
    uint32_t lockedPromotions = 0;
    uint32_t tentativeLostConnections = 0;
    uint32_t promotionChecks = 0;
    uint32_t promotionFrameBlocks = 0;
    uint32_t promotionOffsetBlocks = 0;
    uint32_t promotionDxBlocks = 0;
    uint32_t phaseShiftRequests = 0;
    uint16_t maxConsecutiveRcFrames = 0;
    int32_t lastPhaseShiftUs = 0;
    uint16_t nonce = 0;
    uint16_t fhssIndex = 0;
    int32_t phaseErrorUs = 0;
    ConnectionState connectionState = ConnectionState::Disconnected;
    RxTimerState rxTimerState = RxTimerState::TimDisconnected;
    int32_t offsetUs = 0;
    int32_t offsetDxUs = 0;
    int32_t frequencyOffsetUs = 0;
};

class PhaseFrequencyDetector {
public:
    void extEvent(uint32_t atUs);
    void intEvent(uint32_t atUs);
    bool hasResult() const { return hasResult_; }
    int32_t result() const { return result_; }
    void reset();

private:
    bool haveExternal_ = false;
    bool haveInternal_ = false;
    bool hasResult_ = false;
    uint32_t externalUs_ = 0;
    uint32_t internalUs_ = 0;
    int32_t result_ = 0;
};

class FhssState {
public:
    void begin(const uint8_t uid[kUidSize], uint8_t hopInterval);
    void reset();
    void setCurrIndex(uint16_t index);
    void setFromSequence(uint16_t sequence);
    void advanceAfterSequence(uint16_t sequence);
    void advanceIndex();

    uint16_t index() const { return index_; }
    uint16_t currIndex() const { return index_; }
    uint8_t channel() const;
    float frequencyMHz() const;

private:
    uint8_t uid_[kUidSize] = {};
    uint8_t hopInterval_ = 4;
    uint16_t index_ = 0;
};

class TxScheduler {
public:
    void begin(const uint8_t uid[kUidSize], const RateConfig& rate);
    bool onTimerTick();
    void onTxDone();
    void onTelemetryDone();

    uint16_t sequence() const { return sequence_; }
    uint16_t nonce() const { return sequence_; }
    bool shouldSendSyncFrame() const;
    OtaSyncPayload syncPayload() const;
    bool shouldListenForTelemetry() const { return telemetryListenPending_; }
    void consumeTelemetryListen() { telemetryListenPending_ = false; }
    const FhssState& fhss() const { return fhss_; }
    SchedulerStats stats() const;

private:
    FhssState fhss_{};
    RateConfig rate_{};
    SchedulerStats stats_{};
    uint16_t sequence_ = 0;
    bool telemetryListenPending_ = false;

    void advanceTock();
};

class RxScheduler {
public:
    void begin(const uint8_t uid[kUidSize], const RateConfig& rate);
    RxTimerTickEvent onTimerTick(bool linkFresh);
    RxTimerEventResult onTimerEvent(RxTimerHalfEvent half, uint32_t eventUs, bool linkFresh, uint32_t nowMs);
    bool onValidSyncFrame(const OtaSyncPayload& sync, uint32_t beginProcessingUs);
    bool onValidRcFrame(uint16_t sequence, uint32_t beginProcessingUs, uint32_t nowMs);
    void onTelemetryDone();
    void setConnected(bool connected);
    uint32_t packetTockReferenceUs(uint32_t beginProcessingUs) const;

    uint16_t nextExpectedSequence() const { return nextExpectedSequence_; }
    uint16_t nonce() const { return nextExpectedSequence_; }
    const FhssState& fhss() const { return fhss_; }
    SchedulerStats stats() const;

private:
    FhssState fhss_{};
    RateConfig rate_{};
    SchedulerStats stats_{};
    uint16_t nextExpectedSequence_ = 0;
    uint8_t disconnectedScanTick_ = 0;
    bool haveSequence_ = false;
    bool receivedFrameForCurrentTock_ = false;
    uint16_t consecutiveRcFrames_ = 0;
    uint32_t connectedSinceMs_ = 0;
    PhaseFrequencyDetector pfd_{};
    RxTimerState timerState_ = RxTimerState::TimDisconnected;
    int32_t offsetUs_ = 0;
    int32_t offsetDxUs_ = 0;
    int32_t previousRawOffsetUs_ = 0;
    int32_t frequencyOffsetUs_ = 0;

    void advanceTock();
    void lostConnection();
    RxTimerEventResult updatePhaseLock(uint32_t nowMs);
};

} // namespace rf
} // namespace lora_link
