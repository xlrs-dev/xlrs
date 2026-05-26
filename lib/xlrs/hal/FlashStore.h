#pragma once

#include <stddef.h>
#include <stdint.h>

namespace xlrs::hal::FlashStore {

bool begin();
void setMulticoreSafetyEnabled(bool enabled);
uint8_t read(size_t offset);
void write(size_t offset, uint8_t value);
bool commit();

#if !defined(XLRS_PICO_SDK)
void resetSim();
void seedLegacySim(const uint8_t* data, size_t len);
void simulatePowerCutOnNextCommit();
#endif

} // namespace xlrs::hal::FlashStore
