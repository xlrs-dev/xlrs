#pragma once

#include "rc_handset/core1/Snapshot.h"

namespace rc_handset {
namespace core1 {

SnapshotExchange& exchange();

uint32_t publishConfig(const ConfigSnapshot& config);
uint32_t publishDefaultConfig();
uint32_t ackedConfigGeneration();
bool readLiveState(LiveStateSnapshot& out);
void setCrsfOutputSafetyHold(bool enabled);
void releaseCrsfOutputSafetyHoldWhenInputsSafe();
bool crsfOutputSafetyHoldEnabled();

void begin();
void loopOnceFallback();
bool isCore1Started();

} // namespace core1
} // namespace rc_handset
