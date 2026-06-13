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
    snapshot.safetyHold = false;
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
    : _configLock(false),
      _config(makeDefaultConfigSnapshot()),
      _liveStateLock(false),
      _liveState(makeDefaultLiveStateSnapshot()),
      _nextConfigGeneration(0),
      _ackedConfigGeneration(0) {
}

uint32_t SnapshotExchange::publishConfigFromCore0(ConfigSnapshot config) {
    config.generation = ++_nextConfigGeneration;
    if (config.crsfFrameIntervalUs == 0) {
        config.crsfFrameIntervalUs = kDefaultCrsfFrameIntervalUs;
    }
    lock(_configLock);
    _config = config;
    unlock(_configLock);
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
    lock(_liveStateLock);
    _liveState = state;
    unlock(_liveStateLock);
}

bool SnapshotExchange::readLiveStateFromCore0(LiveStateSnapshot& out) const {
    return loadLiveState(out);
}

void SnapshotExchange::lock(std::atomic<bool>& flag) {
    bool expected = false;
    while (!flag.compare_exchange_weak(expected, true,
                                       std::memory_order_acquire,
                                       std::memory_order_relaxed)) {
        expected = false;
    }
}

void SnapshotExchange::unlock(std::atomic<bool>& flag) {
    flag.store(false, std::memory_order_release);
}

bool SnapshotExchange::loadConfig(ConfigSnapshot& out, uint32_t& generation) const {
    lock(_configLock);
    const ConfigSnapshot snapshot = _config;
    unlock(_configLock);
    if (snapshot.generation == generation) {
        return false;
    }
    out = snapshot;
    generation = snapshot.generation;
    return true;
}

bool SnapshotExchange::loadLiveState(LiveStateSnapshot& out) const {
    lock(_liveStateLock);
    out = _liveState;
    unlock(_liveStateLock);
    return true;
}

} // namespace core1
} // namespace rc_handset
