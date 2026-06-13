#include "rc_handset/core1/Snapshot.h"

#include <string.h>

namespace rc_handset {
namespace core1 {

ConfigSnapshot makeDefaultConfigSnapshot() {
    ConfigSnapshot snapshot{};
    snapshot.generation = 0;
    snapshot.crsfFrameIntervalUs = kDefaultCrsfFrameIntervalUs;
    snapshot.handsetConfig = config::defaultRcHandsetConfig();
    return snapshot;
}

LiveStateSnapshot makeDefaultLiveStateSnapshot() {
    LiveStateSnapshot snapshot{};
    snapshot.appliedConfigGeneration = 0;
    snapshot.haveChannels = false;
    snapshot.haveAdc = false;
    for (uint8_t i = 0; i < config::kRcHandsetAxisCount; ++i) {
        snapshot.rawAdc[i] = 0;
        snapshot.filteredAdc[i] = 0;
    }
    for (uint8_t i = 0; i < lora_link::kRcChannelCount; ++i) {
        snapshot.channels[i] = lora_link::kCrsfRawMid;
    }
    return snapshot;
}

SnapshotExchange::SnapshotExchange()
    : _configSequence(0),
      _config(makeDefaultConfigSnapshot()),
      _liveStateSequence(0),
      _liveState(makeDefaultLiveStateSnapshot()),
      _nextConfigGeneration(0),
      _ackedConfigGeneration(0) {
}

uint32_t SnapshotExchange::publishConfigFromCore0(ConfigSnapshot config) {
    config.generation = ++_nextConfigGeneration;
    if (config.crsfFrameIntervalUs == 0) {
        config.crsfFrameIntervalUs = kDefaultCrsfFrameIntervalUs;
    }
    const uint32_t sequence = _configSequence.load(std::memory_order_relaxed);
    _configSequence.store(sequence + 1u, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    _config = config;
    std::atomic_thread_fence(std::memory_order_release);
    _configSequence.store(sequence + 2u, std::memory_order_relaxed);
    return config.generation;
}

bool SnapshotExchange::loadConfigForCore1(ConfigSnapshot& out, uint32_t& lastMailboxSequence) const {
    return loadConfig(out, lastMailboxSequence);
}

void SnapshotExchange::ackConfigFromCore1(uint32_t generation) {
    _ackedConfigGeneration.store(generation, std::memory_order_release);
}

uint32_t SnapshotExchange::ackedConfigGeneration() const {
    return _ackedConfigGeneration.load(std::memory_order_acquire);
}

void SnapshotExchange::publishLiveStateFromCore1(const LiveStateSnapshot& state) {
    const uint32_t sequence = _liveStateSequence.load(std::memory_order_relaxed);
    _liveStateSequence.store(sequence + 1u, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    _liveState = state;
    std::atomic_thread_fence(std::memory_order_release);
    _liveStateSequence.store(sequence + 2u, std::memory_order_relaxed);
}

bool SnapshotExchange::readLiveStateFromCore0(LiveStateSnapshot& out) const {
    return loadLiveState(out);
}

bool SnapshotExchange::loadConfig(ConfigSnapshot& out, uint32_t& sequence) const {
    for (uint8_t tries = 0; tries < 8; ++tries) {
        const uint32_t before = _configSequence.load(std::memory_order_relaxed);
        if (before & 1u) {
            continue;
        }
        if (before == sequence) {
            return false;
        }
        std::atomic_thread_fence(std::memory_order_acquire);
        const ConfigSnapshot snapshot = _config;
        std::atomic_thread_fence(std::memory_order_acquire);
        const uint32_t after = _configSequence.load(std::memory_order_relaxed);
        if (before == after) {
            out = snapshot;
            sequence = before;
            return true;
        }
    }
    return false;
}

bool SnapshotExchange::loadLiveState(LiveStateSnapshot& out) const {
    for (uint8_t tries = 0; tries < 8; ++tries) {
        const uint32_t before = _liveStateSequence.load(std::memory_order_relaxed);
        if (before & 1u) {
            continue;
        }
        std::atomic_thread_fence(std::memory_order_acquire);
        const LiveStateSnapshot snapshot = _liveState;
        std::atomic_thread_fence(std::memory_order_acquire);
        const uint32_t after = _liveStateSequence.load(std::memory_order_relaxed);
        if (before == after) {
            out = snapshot;
            return true;
        }
    }
    return false;
}

} // namespace core1
} // namespace rc_handset
