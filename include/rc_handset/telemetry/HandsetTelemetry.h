#pragma once

#include <stddef.h>
#include <stdint.h>

#include "lora_link/protocol.h"

namespace rc_handset {
namespace telemetry {

constexpr uint8_t kHandsetTelemetryChannelCount = lora_link::kRcChannelCount;

enum class FieldStatus : uint8_t {
    Missing = 0,
    Valid = 1,
    Stale = 2,
};

struct TelemetryTimeouts {
    uint32_t localChannelsMs = 100;
    uint32_t txStatusMs = 500;
    uint32_t linkStatsMs = 500;
    uint32_t rxBatteryMs = 5000;
};

struct PacketCounters {
    uint32_t uplinkTransmitAttempts = 0;
    uint32_t uplinkTransmitSuccesses = 0;
    uint32_t validOtaFrames = 0;
    uint32_t rejectedOtaFrames = 0;
    uint32_t downlinkTelemetryFrames = 0;
    uint32_t lostPackets = 0;
};

struct RxBattery {
    uint16_t millivolts = 0;
    uint8_t percent = 0;
};

struct FaultFlags {
    bool configFault = false;
    bool bindingUidMissing = false;
    bool bindingUidMismatch = false;
};

struct HandsetTelemetryState {
    uint16_t localChannels[kHandsetTelemetryChannelCount] = {};
    FieldStatus localChannelsStatus = FieldStatus::Missing;
    uint32_t lastLocalChannelsMs = 0;

    bool txPresent = false;
    FieldStatus txStatus = FieldStatus::Missing;
    uint32_t lastTxStatusMs = 0;
    bool haveExpectedUidCheck = false;
    uint32_t expectedUidCheck = 0;
    bool haveObservedUidCheck = false;
    uint32_t observedUidCheck = 0;
    lora_link::RateId activeRate = lora_link::RateId::L100;

    FieldStatus linkStatsStatus = FieldStatus::Missing;
    uint32_t lastLinkStatsMs = 0;
    uint8_t linkQuality = 0;
    int16_t rssiDbm = 0;
    int8_t snrDb = 0;
    PacketCounters packets{};

    FieldStatus rxBatteryStatus = FieldStatus::Missing;
    uint32_t lastRxBatteryMs = 0;
    RxBattery rxBattery{};

    FaultFlags faults{};
    uint32_t updatedAtMs = 0;
};

class HandsetTelemetry {
public:
    explicit HandsetTelemetry(TelemetryTimeouts timeouts = TelemetryTimeouts{});

    void reset();
    void refresh(uint32_t nowMs);

    void updateLocalChannels(const uint16_t channels[kHandsetTelemetryChannelCount],
                             uint32_t nowMs);
    void updateTxStatus(bool present,
                        bool haveExpectedUidCheck,
                        uint32_t expectedUidCheck,
                        bool haveObservedUidCheck,
                        uint32_t observedUidCheck,
                        lora_link::RateId activeRate,
                        uint32_t nowMs);
    void updateLoRaLink(uint8_t linkQuality,
                        int16_t rssiDbm,
                        int8_t snrDb,
                        uint32_t nowMs);
    void updatePacketCounters(const PacketCounters& counters, uint32_t nowMs);
    void updateRxBattery(uint16_t millivolts, uint8_t percent, uint32_t nowMs);
    void setConfigFault(bool fault, uint32_t nowMs);

    const HandsetTelemetryState& state() const { return state_; }
    const TelemetryTimeouts& timeouts() const { return timeouts_; }

private:
    TelemetryTimeouts timeouts_{};
    HandsetTelemetryState state_{};

    void updateBindingFaults();
    FieldStatus statusFor(bool haveValue, uint32_t lastUpdateMs, uint32_t timeoutMs,
                          uint32_t nowMs) const;
};

const char* fieldStatusName(FieldStatus status);
const char* rateName(lora_link::RateId rate);

bool formatUsbStateResponse(const HandsetTelemetryState& state, char* out, size_t outLen);
bool formatOledDisplay(const HandsetTelemetryState& state, char* out, size_t outLen);

} // namespace telemetry
} // namespace rc_handset
