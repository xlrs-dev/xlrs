#include "rc_handset/core1/Runtime.h"
#include "rc_handset/input/ArduinoInputSampler.h"
#include "rc_handset/input/InputPipeline.h"

#include <string.h>

#if !defined(UNIT_TEST)
#include <Arduino.h>
#if defined(RC_HANDSET_ENABLE_CORE1)
#include <pico/multicore.h>
#endif
#endif

namespace rc_handset {
namespace core1 {

namespace {

SnapshotExchange g_exchange;
bool g_core1Started = false;
std::atomic<bool> g_crsfOutputSafetyHold{false};
std::atomic<bool> g_releaseSafetyHoldWhenInputsSafe{false};

} // namespace

SnapshotExchange& exchange() {
    return g_exchange;
}

uint32_t publishConfig(const ConfigSnapshot& config) {
    return g_exchange.publishConfigFromCore0(config);
}

uint32_t publishDefaultConfig() {
    return publishConfig(makeDefaultConfigSnapshot());
}

uint32_t ackedConfigGeneration() {
    return g_exchange.ackedConfigGeneration();
}

bool readLiveState(LiveStateSnapshot& out) {
    return g_exchange.readLiveStateFromCore0(out);
}

void setCrsfOutputSafetyHold(bool enabled) {
    g_crsfOutputSafetyHold.store(enabled, std::memory_order_release);
    if (enabled) {
        g_releaseSafetyHoldWhenInputsSafe.store(false, std::memory_order_release);
    }
}

void releaseCrsfOutputSafetyHoldWhenInputsSafe() {
    g_releaseSafetyHoldWhenInputsSafe.store(true, std::memory_order_release);
}

bool crsfOutputSafetyHoldEnabled() {
    return g_crsfOutputSafetyHold.load(std::memory_order_acquire);
}

bool isCore1Started() {
    return g_core1Started;
}

#if !defined(UNIT_TEST)

using namespace lora_link;
using namespace rc_handset::input;

#ifndef CRSF_UART_TX_PIN
#define CRSF_UART_TX_PIN 8
#endif

#ifndef CRSF_UART_RX_PIN
#define CRSF_UART_RX_PIN 9
#endif

namespace {

InputPipelineConfig g_inputConfig = defaultInputPipelineConfig();
InputPipelineState g_inputState = defaultInputPipelineState();
ArduinoInputSampler g_inputSampler;
ArduinoInputSamplerConfig g_inputSamplerConfig = defaultArduinoInputSamplerConfig();
uint32_t g_framesSent = 0;
uint32_t g_channelGuardRejects = 0;
uint32_t g_channelSpikeHolds = 0;
uint32_t g_lastFrameUs = 0;
uint32_t g_liveStateGeneration = 0;
uint32_t g_configMailboxSequence = 0;
ConfigSnapshot g_config = makeDefaultConfigSnapshot();
uint8_t g_safetyResumeStableFrames = 0;

void fillSafeDisarmedChannels(uint16_t channels[kRcChannelCount]) {
    for (uint8_t i = 0; i < kRcChannelCount; ++i) channels[i] = kCrsfRawMid;
    channels[2] = kCrsfRaw1000;
    const uint8_t throttleIndex = channelIndexForFunction(ChannelFunction::Throttle);
    if (throttleIndex < kRcChannelCount) channels[throttleIndex] = kCrsfRaw1000;
    for (uint8_t i = 4; i < kRcChannelCount; ++i) channels[i] = kCrsfRaw1000;
}

bool channelsSafeForResume(const uint16_t channels[kRcChannelCount]) {
    static constexpr uint16_t kSafeLowThreshold = kCrsfRaw1000 + 80u;
    const uint8_t throttleIndex = channelIndexForFunction(ChannelFunction::Throttle);
    if (throttleIndex >= kRcChannelCount || channels[throttleIndex] > kSafeLowThreshold) {
        return false;
    }
    for (uint8_t i = 4; i < kRcChannelCount; ++i) {
        if (channels[i] > kSafeLowThreshold) return false;
    }
    return true;
}

void refreshConfig() {
    ConfigSnapshot next{};
    if (g_exchange.loadConfigForCore1(next, g_configMailboxSequence)) {
        if (next.crsfFrameIntervalUs == 0) {
            next.crsfFrameIntervalUs = kDefaultCrsfFrameIntervalUs;
        }
        g_config = next;
        g_inputConfig = inputPipelineConfigFromHandsetConfig(g_config.handsetConfig);
        g_inputSamplerConfig.adcSamples = g_config.handsetConfig.filter.adcSamples;
        g_inputSamplerConfig.smoothingPercent = g_config.handsetConfig.filter.smoothingPercent;
        g_inputSampler.configure(g_inputSamplerConfig);
        g_exchange.ackConfigFromCore1(g_config.generation);
    }
}

void updateChannels() {
    const InputSnapshot snapshot = g_inputSampler.read();
    const ProcessResult result = processInputSnapshot(g_inputConfig, snapshot, g_inputState);
    if (result == ProcessResult::RejectedBySanitizer) {
        ++g_channelGuardRejects;
        return;
    }
    if (result == ProcessResult::HeldBySpikeGate) {
        ++g_channelSpikeHolds;
    }
}

void serviceOutputSafetyHold() {
    if (!g_crsfOutputSafetyHold.load(std::memory_order_acquire)) {
        g_safetyResumeStableFrames = 0;
        return;
    }
    if (!g_releaseSafetyHoldWhenInputsSafe.load(std::memory_order_acquire) ||
        !g_inputState.haveChannels ||
        !channelsSafeForResume(g_inputState.channels)) {
        g_safetyResumeStableFrames = 0;
        return;
    }
    ++g_safetyResumeStableFrames;
    if (g_safetyResumeStableFrames >= 3u) {
        g_inputState.spikeGate.havePending = false;
        g_safetyResumeStableFrames = 0;
        g_releaseSafetyHoldWhenInputsSafe.store(false, std::memory_order_release);
        g_crsfOutputSafetyHold.store(false, std::memory_order_release);
    }
}

void sendCrsfChannels() {
    uint8_t frame[32];
    uint16_t channels[kRcChannelCount];
    const uint16_t* channelsToSend = g_inputState.channels;
    if (g_crsfOutputSafetyHold.load(std::memory_order_acquire)) {
        fillSafeDisarmedChannels(channels);
        channelsToSend = channels;
    }
    const size_t len = encodeCrsfRcFrame(channelsToSend, frame, sizeof(frame));
    if (len) {
        Serial2.write(frame, len);
        ++g_framesSent;
    }
}

void publishLiveState(uint32_t nowUs) {
    LiveStateSnapshot snapshot{};
    snapshot.generation = ++g_liveStateGeneration;
    snapshot.appliedConfigGeneration = g_config.generation;
    snapshot.framesSent = g_framesSent;
    snapshot.channelGuardRejects = g_channelGuardRejects;
    snapshot.channelSpikeHolds = g_channelSpikeHolds;
    snapshot.lastFrameUs = nowUs;
    snapshot.haveChannels = g_inputState.haveChannels;
    if (g_crsfOutputSafetyHold.load(std::memory_order_acquire)) {
        fillSafeDisarmedChannels(snapshot.channels);
        snapshot.haveChannels = true;
    } else {
        memcpy(snapshot.channels, g_inputState.channels, sizeof(snapshot.channels));
    }
    g_exchange.publishLiveStateFromCore1(snapshot);
}

void configureTimingCoreHardware() {
    Serial2.setTX(CRSF_UART_TX_PIN);
    Serial2.setRX(CRSF_UART_RX_PIN);
    Serial2.begin(kCrsfBaud);
    g_inputSampler.begin(g_inputSamplerConfig);
    refreshConfig();
    updateChannels();
    publishLiveState(micros());
}

void timingCoreLoopOnce() {
    refreshConfig();
    const uint32_t nowUs = micros();
    const uint32_t intervalUs = g_config.crsfFrameIntervalUs ? g_config.crsfFrameIntervalUs : kDefaultCrsfFrameIntervalUs;
    if (static_cast<uint32_t>(nowUs - g_lastFrameUs) >= intervalUs) {
        g_lastFrameUs = nowUs;
        updateChannels();
        serviceOutputSafetyHold();
        if (g_inputState.haveChannels || g_crsfOutputSafetyHold.load(std::memory_order_acquire)) {
            sendCrsfChannels();
        }
        publishLiveState(nowUs);
    }
}

#if defined(RC_HANDSET_ENABLE_CORE1)
void core1Main() {
    configureTimingCoreHardware();
    for (;;) {
        timingCoreLoopOnce();
    }
}
#endif

} // namespace

void begin() {
#if defined(RC_HANDSET_ENABLE_CORE1)
    multicore_launch_core1(core1Main);
    g_core1Started = true;
#else
    configureTimingCoreHardware();
    g_core1Started = false;
#endif
}

void loopOnceFallback() {
#if !defined(RC_HANDSET_ENABLE_CORE1)
    timingCoreLoopOnce();
#endif
}

#else

void begin() {
    g_core1Started = false;
}

void loopOnceFallback() {}

#endif // !UNIT_TEST

} // namespace core1
} // namespace rc_handset
