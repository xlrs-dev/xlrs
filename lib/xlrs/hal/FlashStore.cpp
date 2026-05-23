#include "hal/FlashStore.h"

#include <string.h>

#if defined(XLRS_PICO_SDK)
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <pico/stdlib.h>

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#endif

namespace {
constexpr size_t kStoreSize = FLASH_SECTOR_SIZE;
constexpr uint32_t kStoreOffset = PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE;
uint8_t s_cache[kStoreSize];
bool s_loaded = false;
bool s_dirty = false;
} // namespace

namespace xlrs::hal::FlashStore {

bool begin() {
    const uint8_t* flash = reinterpret_cast<const uint8_t*>(XIP_BASE + kStoreOffset);
    memcpy(s_cache, flash, kStoreSize);
    s_loaded = true;
    s_dirty = false;
    return true;
}

uint8_t read(size_t offset) {
    if (!s_loaded) begin();
    if (offset >= kStoreSize) return 0xFF;
    return s_cache[offset];
}

void write(size_t offset, uint8_t value) {
    if (!s_loaded) begin();
    if (offset >= kStoreSize) return;
    if (s_cache[offset] != value) {
        s_cache[offset] = value;
        s_dirty = true;
    }
}

bool commit() {
    if (!s_loaded) begin();
    if (!s_dirty) return true;

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(kStoreOffset, FLASH_SECTOR_SIZE);
    flash_range_program(kStoreOffset, s_cache, kStoreSize);
    restore_interrupts(ints);

    s_dirty = false;
    return true;
}

} // namespace xlrs::hal::FlashStore

#else
namespace {
uint8_t s_cache[4096] = {};
}

namespace xlrs::hal::FlashStore {
bool begin() { return true; }
uint8_t read(size_t offset) { return offset < sizeof(s_cache) ? s_cache[offset] : 0xFF; }
void write(size_t offset, uint8_t value) { if (offset < sizeof(s_cache)) s_cache[offset] = value; }
bool commit() { return true; }
} // namespace xlrs::hal::FlashStore
#endif
