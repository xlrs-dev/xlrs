// FlashBootCounter implementation.
#include "util/FlashBootCounter.h"
#include <string.h>

#if defined(ARDUINO_ARCH_RP2040) || defined(PICO_BOARD)
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <hardware/address_mapped.h>
#else
#ifndef FLASH_SECTOR_SIZE
#define FLASH_SECTOR_SIZE 4096
#endif
#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE 256
#endif
#endif

namespace xlrs {

// 1.5 MB mark, safely beyond typical app size (<250KB) and below EEPROM at the end of 2MB flash.
static constexpr uint32_t BOOT_COUNTER_FLASH_OFFSET = 1536 * 1024;
static constexpr uint32_t BOOT_MAGIC = 0xBAADF00D;

struct BootEntry {
    uint32_t counter;
    uint32_t magic;
};

static constexpr int ENTRIES_PER_SECTOR = FLASH_SECTOR_SIZE / sizeof(BootEntry); // 4096 / 8 = 512
static constexpr int ENTRIES_PER_PAGE = FLASH_PAGE_SIZE / sizeof(BootEntry);     // 256 / 8 = 32

#if !defined(ARDUINO_ARCH_RP2040) && !defined(PICO_BOARD)
// Host-native in-memory fallback.
static uint32_t s_simCounter = 0;

uint32_t FlashBootCounter::read() {
    return s_simCounter;
}

uint32_t FlashBootCounter::increment() {
    return ++s_simCounter;
}

void FlashBootCounter::resetSim() {
    s_simCounter = 0;
}
#else
// RP2040 Pico SDK concrete wear-leveled flash implementation.
uint32_t FlashBootCounter::read() {
    const BootEntry* entries = (const BootEntry*)(XIP_BASE + BOOT_COUNTER_FLASH_OFFSET);
    uint32_t maxCounter = 0;
    
    for (int i = 0; i < ENTRIES_PER_SECTOR; ++i) {
        if (entries[i].magic == BOOT_MAGIC) {
            if (entries[i].counter > maxCounter) {
                maxCounter = entries[i].counter;
            }
        }
    }
    return maxCounter;
}

uint32_t FlashBootCounter::increment() {
    const BootEntry* entries = (const BootEntry*)(XIP_BASE + BOOT_COUNTER_FLASH_OFFSET);
    int activeIndex = -1;
    uint32_t maxCounter = 0;
    
    // Find the latest valid entry.
    for (int i = 0; i < ENTRIES_PER_SECTOR; ++i) {
        if (entries[i].magic == BOOT_MAGIC) {
            if (entries[i].counter >= maxCounter) {
                maxCounter = entries[i].counter;
                activeIndex = i;
            }
        }
    }
    
    uint32_t nextCounter = maxCounter + 1;
    int nextIndex = activeIndex + 1;
    
    if (nextIndex >= ENTRIES_PER_SECTOR) {
        // Sector is full! Erase sector and write nextCounter to slot 0.
        uint8_t writeBuf[FLASH_SECTOR_SIZE];
        memset(writeBuf, 0xFF, FLASH_SECTOR_SIZE);
        
        BootEntry* newEntries = (BootEntry*)writeBuf;
        newEntries[0].counter = nextCounter;
        newEntries[0].magic = BOOT_MAGIC;
        
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(BOOT_COUNTER_FLASH_OFFSET, FLASH_SECTOR_SIZE);
        flash_range_program(BOOT_COUNTER_FLASH_OFFSET, writeBuf, FLASH_SECTOR_SIZE);
        restore_interrupts(ints);
    } else {
        // We write only the page containing the next slot to avoid erasing.
        uint32_t pageIndex = nextIndex / ENTRIES_PER_PAGE;
        uint32_t pageOffset = pageIndex * FLASH_PAGE_SIZE;
        
        uint8_t pageBuf[FLASH_PAGE_SIZE];
        // Read current page content.
        memcpy(pageBuf, (const uint8_t*)(XIP_BASE + BOOT_COUNTER_FLASH_OFFSET + pageOffset), FLASH_PAGE_SIZE);
        
        // Update the specific slot inside pageBuf.
        BootEntry* pageEntries = (BootEntry*)pageBuf;
        pageEntries[nextIndex % ENTRIES_PER_PAGE].counter = nextCounter;
        pageEntries[nextIndex % ENTRIES_PER_PAGE].magic = BOOT_MAGIC;
        
        uint32_t ints = save_and_disable_interrupts();
        flash_range_program(BOOT_COUNTER_FLASH_OFFSET + pageOffset, pageBuf, FLASH_PAGE_SIZE);
        restore_interrupts(ints);
    }
    
    return nextCounter;
}

void FlashBootCounter::resetSim() {
    // No-op on hardware.
}
#endif

} // namespace xlrs
