#include "hal/FlashStore.h"

#include <atomic>
#include <string.h>

#if defined(XLRS_PICO_SDK)
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <pico/flash.h>
#include <pico/stdlib.h>

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#endif
#endif

namespace {

constexpr uint32_t kSlotMagic = 0x58465331UL;  // "XFS1"
constexpr uint32_t kCommitMagic = 0x5846434DUL; // "XFCM"
constexpr uint32_t kErasedWord = 0xFFFFFFFFUL;

struct SlotHeader {
    uint32_t magic;
    uint32_t sequence;
    uint32_t payloadSize;
    uint32_t checksum;
};

struct SlotFooter {
    uint32_t commitMagic;
    uint32_t sequence;
};

#if defined(XLRS_PICO_SDK)
constexpr size_t kSlotSize = FLASH_SECTOR_SIZE;
constexpr size_t kStoreSize = kSlotSize - sizeof(SlotHeader) - sizeof(SlotFooter);
constexpr uint32_t kStoreOffset = PICO_FLASH_SIZE_BYTES - (2 * FLASH_SECTOR_SIZE);
#else
constexpr size_t kSlotSize = 4096;
constexpr size_t kStoreSize = kSlotSize - sizeof(SlotHeader) - sizeof(SlotFooter);
#endif

uint8_t s_cache[kStoreSize];
bool s_loaded = false;
bool s_dirty = false;
uint32_t s_sequence = 0;
uint8_t s_activeSlot = 0;

uint32_t checksum(const uint8_t* data, size_t len) {
    uint32_t crc = 0x811C9DC5UL;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        crc *= 0x01000193UL;
    }
    return crc;
}

void put32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

uint32_t get32(const uint8_t* p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

void buildSlotImage(uint8_t* out, const uint8_t* payload, uint32_t sequence) {
    memset(out, 0xFF, kSlotSize);
    put32(out + 0, kSlotMagic);
    put32(out + 4, sequence);
    put32(out + 8, (uint32_t)kStoreSize);
    put32(out + 12, checksum(payload, kStoreSize));
    memcpy(out + sizeof(SlotHeader), payload, kStoreSize);
    put32(out + kSlotSize - sizeof(SlotFooter), kCommitMagic);
    put32(out + kSlotSize - 4, sequence);
}

bool decodeSlot(const uint8_t* slot, uint8_t* payload, uint32_t& sequence) {
    if (get32(slot + 0) != kSlotMagic) return false;
    const uint32_t headerSeq = get32(slot + 4);
    if (get32(slot + 8) != kStoreSize) return false;
    const uint32_t expectedChecksum = get32(slot + 12);
    const uint8_t* data = slot + sizeof(SlotHeader);
    const uint8_t* footer = slot + kSlotSize - sizeof(SlotFooter);
    if (get32(footer) != kCommitMagic) return false;
    if (get32(footer + 4) != headerSeq) return false;
    if (checksum(data, kStoreSize) != expectedChecksum) return false;
    if (payload) memcpy(payload, data, kStoreSize);
    sequence = headerSeq;
    return true;
}

bool sequenceNewer(uint32_t a, uint32_t b) {
    return a != b && (uint32_t)(a - b) < 0x80000000UL;
}

#if defined(XLRS_PICO_SDK)
std::atomic<bool> s_multicoreSafetyEnabled{false};

const uint8_t* flashSlot(uint8_t slot) {
    return reinterpret_cast<const uint8_t*>(XIP_BASE + kStoreOffset + (slot * kSlotSize));
}

const uint8_t* legacyFlashStore() {
    return reinterpret_cast<const uint8_t*>(XIP_BASE + PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE);
}

void programSlotSector(uint8_t slot, const uint8_t* image) {
    const uint32_t offset = kStoreOffset + (slot * kSlotSize);
    flash_range_erase(offset, FLASH_SECTOR_SIZE);
    for (uint32_t off = 0; off < kSlotSize; off += FLASH_PAGE_SIZE) {
        flash_range_program(offset + off, image + off, FLASH_PAGE_SIZE);
    }
}

struct CommitContext {
    uint8_t slot;
    const uint8_t* image;
};

void programFlash(void* param) {
    auto* ctx = static_cast<CommitContext*>(param);
    programSlotSector(ctx->slot, ctx->image);
}
#else
uint8_t s_flash[2][kSlotSize];
bool s_flashInitialized = false;
bool s_cutNextCommit = false;

const uint8_t* flashSlot(uint8_t slot) {
    return s_flash[slot];
}

const uint8_t* legacyFlashStore() {
    return s_flash[1];
}

void ensureHostFlashInit() {
    if (!s_flashInitialized) {
        memset(s_flash, 0xFF, sizeof(s_flash));
        s_flashInitialized = true;
    }
}
#endif

bool looksErased(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        if (data[i] != 0xFF) return false;
    }
    return true;
}

} // namespace

namespace xlrs::hal::FlashStore {

bool begin() {
#if !defined(XLRS_PICO_SDK)
    ensureHostFlashInit();
#endif
    uint8_t bestSlot = 0;
    uint32_t bestSeq = 0;
    bool found = false;
    uint8_t candidate[kStoreSize];
    for (uint8_t slot = 0; slot < 2; ++slot) {
        uint32_t seq = 0;
        if (decodeSlot(flashSlot(slot), candidate, seq) &&
            (!found || sequenceNewer(seq, bestSeq))) {
            memcpy(s_cache, candidate, kStoreSize);
            bestSeq = seq;
            bestSlot = slot;
            found = true;
        }
    }
    if (!found) {
        const uint8_t* legacy = legacyFlashStore();
        if (!looksErased(legacy, kStoreSize)) {
            memcpy(s_cache, legacy, kStoreSize);
            s_dirty = true;
            bestSlot = 1;
        } else {
            memset(s_cache, 0xFF, sizeof(s_cache));
            s_dirty = false;
            bestSlot = 0;
        }
        bestSeq = 0;
    } else {
        s_dirty = false;
    }
    s_sequence = bestSeq;
    s_activeSlot = bestSlot;
    s_loaded = true;
    return true;
}

void setMulticoreSafetyEnabled(bool enabled) {
#if defined(XLRS_PICO_SDK)
    s_multicoreSafetyEnabled.store(enabled, std::memory_order_release);
#else
    (void)enabled;
#endif
}

uint8_t read(size_t offset) {
    if (!s_loaded) begin();
    if (offset >= kStoreSize) return 0xFF;
    return s_cache[offset];
}

size_t capacity() {
    return kStoreSize;
}

bool write(size_t offset, uint8_t value) {
    if (!s_loaded) begin();
    if (offset >= kStoreSize) return false;
    if (s_cache[offset] != value) {
        s_cache[offset] = value;
        s_dirty = true;
    }
    return true;
}

bool commit() {
    if (!s_loaded) begin();
    if (!s_dirty) return true;

    const uint8_t nextSlot = (uint8_t)(s_activeSlot ^ 1);
    const uint32_t nextSeq = (s_sequence == kErasedWord) ? 1 : s_sequence + 1;
    uint8_t image[kSlotSize];
    buildSlotImage(image, s_cache, nextSeq);

#if defined(XLRS_PICO_SDK)
    if (s_multicoreSafetyEnabled.load(std::memory_order_acquire)) {
        CommitContext ctx{ nextSlot, image };
        if (flash_safe_execute(programFlash, &ctx, 1000) != PICO_OK) {
            return false;
        }
    } else {
        uint32_t ints = save_and_disable_interrupts();
        programSlotSector(nextSlot, image);
        restore_interrupts(ints);
    }
#else
    ensureHostFlashInit();
    memset(s_flash[nextSlot], 0xFF, kSlotSize);
    if (s_cutNextCommit) {
        s_cutNextCommit = false;
        return false;
    }
    memcpy(s_flash[nextSlot], image, kSlotSize);
#endif

    uint8_t verify[kStoreSize];
    uint32_t verifiedSeq = 0;
    if (!decodeSlot(flashSlot(nextSlot), verify, verifiedSeq) ||
        verifiedSeq != nextSeq ||
        memcmp(verify, s_cache, kStoreSize) != 0) {
        return false;
    }

    s_sequence = nextSeq;
    s_activeSlot = nextSlot;
    s_dirty = false;
    return true;
}

#if !defined(XLRS_PICO_SDK)
void resetSim() {
    memset(s_flash, 0xFF, sizeof(s_flash));
    memset(s_cache, 0xFF, sizeof(s_cache));
    s_flashInitialized = true;
    s_cutNextCommit = false;
    s_loaded = false;
    s_dirty = false;
    s_sequence = 0;
    s_activeSlot = 0;
}

void seedLegacySim(const uint8_t* data, size_t len) {
    ensureHostFlashInit();
    memset(s_flash, 0xFF, sizeof(s_flash));
    if (data && len > 0) {
        if (len > kStoreSize) len = kStoreSize;
        memcpy(s_flash[1], data, len);
    }
    s_loaded = false;
    s_dirty = false;
    s_sequence = 0;
    s_activeSlot = 0;
}

void simulatePowerCutOnNextCommit() {
    s_cutNextCommit = true;
}
#endif

} // namespace xlrs::hal::FlashStore
