#pragma once

#include <stdint.h>
#include <atomic>

#include "lora_link/protocol.h"
#include "rc_handset/config/config.h"

#ifndef RC_CRSF_SIMPLETX_FRAME_US
#define RC_CRSF_SIMPLETX_FRAME_US 2000u
#endif

namespace rc_handset {
namespace core1 {

constexpr uint32_t kDefaultCrsfFrameIntervalUs = static_cast<uint32_t>(RC_CRSF_SIMPLETX_FRAME_US);

struct ConfigSnapshot {
    uint32_t generation;
    uint32_t crsfFrameIntervalUs;
    config::RcHandsetConfig handsetConfig;
};

struct LiveStateSnapshot {
    uint32_t generation;
    uint32_t appliedConfigGeneration;
    uint32_t framesSent;
    uint32_t channelGuardRejects;
    uint32_t channelSpikeHolds;
    uint32_t lastFrameUs;
    bool haveChannels;
    bool haveAdc;
    uint16_t rawAdc[config::kRcHandsetAxisCount];
    uint16_t filteredAdc[config::kRcHandsetAxisCount];
    uint16_t channels[lora_link::kRcChannelCount];
};

ConfigSnapshot makeDefaultConfigSnapshot();
LiveStateSnapshot makeDefaultLiveStateSnapshot();

class SnapshotExchange {
public:
    SnapshotExchange();

    uint32_t publishConfigFromCore0(ConfigSnapshot config);
    bool loadConfigForCore1(ConfigSnapshot& out, uint32_t& lastMailboxSequence) const;
    void ackConfigFromCore1(uint32_t generation);
    uint32_t ackedConfigGeneration() const;

    void publishLiveStateFromCore1(const LiveStateSnapshot& state);
    bool readLiveStateFromCore0(LiveStateSnapshot& out) const;

private:
    bool loadConfig(ConfigSnapshot& out, uint32_t& sequence) const;
    bool loadLiveState(LiveStateSnapshot& out) const;

    mutable std::atomic<uint32_t> _configSequence;
    ConfigSnapshot _config;
    mutable std::atomic<uint32_t> _liveStateSequence;
    LiveStateSnapshot _liveState;
    uint32_t _nextConfigGeneration;
    std::atomic<uint32_t> _ackedConfigGeneration;
};

} // namespace core1
} // namespace rc_handset
