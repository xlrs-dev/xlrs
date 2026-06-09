#include "rc_handset/telemetry/HandsetTelemetry.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace rc_handset {
namespace telemetry {

namespace {

static bool appendf(char* out, size_t outLen, size_t& used, const char* fmt, ...) {
    if (!out || outLen == 0) return false;
    if (used >= outLen) return false;

    va_list args;
    va_start(args, fmt);
    const int written = vsnprintf(out + used, outLen - used, fmt, args);
    va_end(args);

    if (written < 0) return false;
    const size_t needed = static_cast<size_t>(written);
    const bool fits = needed < (outLen - used);
    used += fits ? needed : (outLen - used - 1);
    return fits;
}

static const char* presentName(bool present) {
    return present ? "present" : "missing";
}

static const char* uidStatusName(const HandsetTelemetryState& state) {
    if (state.faults.bindingUidMismatch) return "mismatch";
    if (state.faults.bindingUidMissing) return "missing";
    if (state.haveExpectedUidCheck && state.haveObservedUidCheck) return "match";
    return "unknown";
}

static bool appendFaultList(const HandsetTelemetryState& state, char* out, size_t outLen,
                            size_t& used) {
    bool ok = true;
    bool any = false;
    ok = appendf(out, outLen, used, " faults=");
    if (state.faults.configFault) {
        ok = appendf(out, outLen, used, "%sconfig", any ? "," : "") && ok;
        any = true;
    }
    if (state.faults.bindingUidMissing) {
        ok = appendf(out, outLen, used, "%sbinding_missing", any ? "," : "") && ok;
        any = true;
    }
    if (state.faults.bindingUidMismatch) {
        ok = appendf(out, outLen, used, "%sbinding_mismatch", any ? "," : "") && ok;
        any = true;
    }
    if (!any) ok = appendf(out, outLen, used, "none") && ok;
    return ok;
}

static const char* oledFaultName(const HandsetTelemetryState& state) {
    if (state.faults.configFault) return "CONFIG";
    if (state.faults.bindingUidMismatch) return "BIND MISMATCH";
    if (state.faults.bindingUidMissing) return "BIND MISSING";
    return "NONE";
}

} // namespace

HandsetTelemetry::HandsetTelemetry(TelemetryTimeouts timeouts) : timeouts_(timeouts) {
    reset();
}

void HandsetTelemetry::reset() {
    memset(&state_, 0, sizeof(state_));
    state_.localChannelsStatus = FieldStatus::Missing;
    state_.txStatus = FieldStatus::Missing;
    state_.linkStatsStatus = FieldStatus::Missing;
    state_.rxBatteryStatus = FieldStatus::Missing;
    state_.activeRate = lora_link::RateId::L100;
}

void HandsetTelemetry::refresh(uint32_t nowMs) {
    state_.localChannelsStatus = statusFor(state_.localChannelsStatus != FieldStatus::Missing,
                                           state_.lastLocalChannelsMs,
                                           timeouts_.localChannelsMs,
                                           nowMs);
    state_.txStatus = statusFor(state_.txStatus != FieldStatus::Missing,
                                state_.lastTxStatusMs,
                                timeouts_.txStatusMs,
                                nowMs);
    state_.linkStatsStatus = statusFor(state_.linkStatsStatus != FieldStatus::Missing,
                                       state_.lastLinkStatsMs,
                                       timeouts_.linkStatsMs,
                                       nowMs);
    state_.rxBatteryStatus = statusFor(state_.rxBatteryStatus != FieldStatus::Missing,
                                       state_.lastRxBatteryMs,
                                       timeouts_.rxBatteryMs,
                                       nowMs);
    state_.updatedAtMs = nowMs;
    updateBindingFaults();
}

void HandsetTelemetry::updateLocalChannels(const uint16_t channels[kHandsetTelemetryChannelCount],
                                           uint32_t nowMs) {
    if (!channels) return;
    memcpy(state_.localChannels, channels, sizeof(state_.localChannels));
    state_.localChannelsStatus = FieldStatus::Valid;
    state_.lastLocalChannelsMs = nowMs;
    state_.updatedAtMs = nowMs;
}

void HandsetTelemetry::updateTxStatus(bool present,
                                      bool haveExpectedUidCheck,
                                      uint32_t expectedUidCheck,
                                      bool haveObservedUidCheck,
                                      uint32_t observedUidCheck,
                                      lora_link::RateId activeRate,
                                      uint32_t nowMs) {
    state_.txPresent = present;
    state_.txStatus = FieldStatus::Valid;
    state_.lastTxStatusMs = nowMs;
    state_.haveExpectedUidCheck = haveExpectedUidCheck;
    state_.expectedUidCheck = expectedUidCheck;
    state_.haveObservedUidCheck = haveObservedUidCheck;
    state_.observedUidCheck = observedUidCheck;
    state_.activeRate = activeRate;
    state_.updatedAtMs = nowMs;
    updateBindingFaults();
}

void HandsetTelemetry::updateLoRaLink(uint8_t linkQuality,
                                      int16_t rssiDbm,
                                      int8_t snrDb,
                                      uint32_t nowMs) {
    if (linkQuality > 100) linkQuality = 100;
    state_.linkQuality = linkQuality;
    state_.rssiDbm = rssiDbm;
    state_.snrDb = snrDb;
    state_.linkStatsStatus = FieldStatus::Valid;
    state_.lastLinkStatsMs = nowMs;
    state_.updatedAtMs = nowMs;
}

void HandsetTelemetry::updatePacketCounters(const PacketCounters& counters, uint32_t nowMs) {
    state_.packets = counters;
    state_.updatedAtMs = nowMs;
}

void HandsetTelemetry::updateRxBattery(uint16_t millivolts, uint8_t percent, uint32_t nowMs) {
    state_.rxBattery.millivolts = millivolts;
    state_.rxBattery.percent = percent > 100 ? 100 : percent;
    state_.rxBatteryStatus = FieldStatus::Valid;
    state_.lastRxBatteryMs = nowMs;
    state_.updatedAtMs = nowMs;
}

void HandsetTelemetry::setConfigFault(bool fault, uint32_t nowMs) {
    state_.faults.configFault = fault;
    state_.updatedAtMs = nowMs;
}

void HandsetTelemetry::updateBindingFaults() {
    state_.faults.bindingUidMissing = false;
    state_.faults.bindingUidMismatch = false;
    if (!state_.txPresent || state_.txStatus == FieldStatus::Missing) return;

    if (!state_.haveExpectedUidCheck || !state_.haveObservedUidCheck) {
        state_.faults.bindingUidMissing = true;
        return;
    }

    state_.faults.bindingUidMismatch = state_.expectedUidCheck != state_.observedUidCheck;
}

FieldStatus HandsetTelemetry::statusFor(bool haveValue, uint32_t lastUpdateMs,
                                        uint32_t timeoutMs, uint32_t nowMs) const {
    if (!haveValue) return FieldStatus::Missing;
    if (timeoutMs == 0) return FieldStatus::Valid;
    return static_cast<uint32_t>(nowMs - lastUpdateMs) <= timeoutMs
               ? FieldStatus::Valid
               : FieldStatus::Stale;
}

const char* fieldStatusName(FieldStatus status) {
    switch (status) {
    case FieldStatus::Valid:
        return "valid";
    case FieldStatus::Stale:
        return "stale";
    case FieldStatus::Missing:
    default:
        return "missing";
    }
}

const char* rateName(lora_link::RateId rate) {
    switch (rate) {
    case lora_link::RateId::L250:
        return "L250";
    case lora_link::RateId::L100:
    default:
        return "L100";
    }
}

bool formatUsbStateResponse(const HandsetTelemetryState& state, char* out, size_t outLen) {
    if (!out || outLen == 0) return false;
    out[0] = '\0';

    size_t used = 0;
    bool ok = true;
    ok = appendf(out, outLen, used,
                 "state tx=%s tx_status=%s rate=%s uid=%s "
                 "link=%s lq=%u rssi=%d snr=%d "
                 "packets=tx:%lu/%lu,ota:%lu/%lu,downlink:%lu,lost:%lu ",
                 presentName(state.txPresent),
                 fieldStatusName(state.txStatus),
                 rateName(state.activeRate),
                 uidStatusName(state),
                 fieldStatusName(state.linkStatsStatus),
                 static_cast<unsigned>(state.linkQuality),
                 static_cast<int>(state.rssiDbm),
                 static_cast<int>(state.snrDb),
                 static_cast<unsigned long>(state.packets.uplinkTransmitSuccesses),
                 static_cast<unsigned long>(state.packets.uplinkTransmitAttempts),
                 static_cast<unsigned long>(state.packets.validOtaFrames),
                 static_cast<unsigned long>(state.packets.rejectedOtaFrames),
                 static_cast<unsigned long>(state.packets.downlinkTelemetryFrames),
                 static_cast<unsigned long>(state.packets.lostPackets)) && ok;

    if (state.rxBatteryStatus == FieldStatus::Missing) {
        ok = appendf(out, outLen, used, "rx_batt=na ") && ok;
    } else {
        ok = appendf(out, outLen, used, "rx_batt=%umV/%u%%:%s ",
                     static_cast<unsigned>(state.rxBattery.millivolts),
                     static_cast<unsigned>(state.rxBattery.percent),
                     fieldStatusName(state.rxBatteryStatus)) && ok;
    }

    ok = appendf(out, outLen, used,
                 "channels=%s ch=%u,%u,%u,%u",
                 fieldStatusName(state.localChannelsStatus),
                 static_cast<unsigned>(state.localChannels[0]),
                 static_cast<unsigned>(state.localChannels[1]),
                 static_cast<unsigned>(state.localChannels[2]),
                 static_cast<unsigned>(state.localChannels[3])) && ok;
    ok = appendFaultList(state, out, outLen, used) && ok;
    return ok;
}

bool formatOledDisplay(const HandsetTelemetryState& state, char* out, size_t outLen) {
    if (!out || outLen == 0) return false;
    out[0] = '\0';

    size_t used = 0;
    bool ok = true;
    ok = appendf(out, outLen, used, "TX %s %s\n",
                 state.txPresent && state.txStatus != FieldStatus::Stale ? "OK" : "NO",
                 rateName(state.activeRate)) && ok;
    ok = appendf(out, outLen, used, "LQ %u RSSI %d\n",
                 static_cast<unsigned>(state.linkQuality),
                 static_cast<int>(state.rssiDbm)) && ok;
    if (state.rxBatteryStatus == FieldStatus::Missing) {
        ok = appendf(out, outLen, used, "SNR %d RX --\n",
                     static_cast<int>(state.snrDb)) && ok;
    } else {
        ok = appendf(out, outLen, used, "SNR %d RX %u.%02uV %u%%\n",
                     static_cast<int>(state.snrDb),
                     static_cast<unsigned>(state.rxBattery.millivolts / 1000),
                     static_cast<unsigned>((state.rxBattery.millivolts % 1000) / 10),
                     static_cast<unsigned>(state.rxBattery.percent)) && ok;
    }
    ok = appendf(out, outLen, used, "FAULT %s", oledFaultName(state)) && ok;
    return ok;
}

} // namespace telemetry
} // namespace rc_handset
