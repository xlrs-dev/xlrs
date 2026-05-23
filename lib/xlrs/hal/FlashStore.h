#pragma once

#include <stddef.h>
#include <stdint.h>

namespace xlrs::hal::FlashStore {

bool begin();
uint8_t read(size_t offset);
void write(size_t offset, uint8_t value);
bool commit();

} // namespace xlrs::hal::FlashStore
