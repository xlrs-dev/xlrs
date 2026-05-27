#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "hal/FlashStore.h"
#include "link/Uid.h"

#ifndef DEFAULT_BINDING_PHRASE
#define DEFAULT_BINDING_PHRASE "FPV-DEFAULT-2024"
#endif

namespace xlrs {

static constexpr uint32_t BINDING_STORE_MAGIC = 0x58424E44UL; // "XBND"
static constexpr uint8_t  BINDING_STORE_VERSION = 1;
static constexpr int      BINDING_STORE_MAGIC_ADDR = 120;
static constexpr int      BINDING_STORE_VERSION_ADDR = 124;
static constexpr int      BINDING_STORE_UID_ADDR = 128;
static constexpr int      BINDING_STORE_CHECKSUM_ADDR = 136;

class BindingStore {
public:
    bool begin() {
        if (!valid()) {
            return setBindingPhrase(DEFAULT_BINDING_PHRASE);
        }
        loadUid(_uid);
        _loaded = true;
        return true;
    }

    bool setBindingPhrase(const char* phrase) {
        if (!phrase) return false;
        const size_t len = strlen(phrase);
        if (len == 0 || len > 32) return false;

        uint8_t uid[LINK_UID_SIZE];
        linkUidFromPhrase(phrase, uid);
        if (!persistUid(uid)) return false;
        memcpy(_uid, uid, LINK_UID_SIZE);
        _loaded = true;
        return true;
    }

    bool setBindingUid(const uint8_t uid[LINK_UID_SIZE]) {
        if (!uid) return false;
        bool any = false;
        for (uint8_t i = 0; i < LINK_UID_SIZE; ++i) {
            if (uid[i] != 0) {
                any = true;
                break;
            }
        }
        if (!any) return false;

        if (!persistUid(uid)) return false;
        memcpy(_uid, uid, LINK_UID_SIZE);
        _loaded = true;
        return true;
    }

    bool getBindingUid(uint8_t uid[LINK_UID_SIZE]) const {
        if (!uid || !_loaded) return false;
        memcpy(uid, _uid, LINK_UID_SIZE);
        return true;
    }

private:
    // Persist the candidate UID to flash. Both setBinding* paths call this,
    // so the magic/version/UID/checksum storage layout (a safety-sensitive flash schema) lives
    // in exactly one place.
    bool persistUid(const uint8_t uid[LINK_UID_SIZE]) {
        bool ok = true;
        ok = hal::FlashStore::write(BINDING_STORE_MAGIC_ADDR + 0, (BINDING_STORE_MAGIC >> 24) & 0xFF) && ok;
        ok = hal::FlashStore::write(BINDING_STORE_MAGIC_ADDR + 1, (BINDING_STORE_MAGIC >> 16) & 0xFF) && ok;
        ok = hal::FlashStore::write(BINDING_STORE_MAGIC_ADDR + 2, (BINDING_STORE_MAGIC >> 8) & 0xFF) && ok;
        ok = hal::FlashStore::write(BINDING_STORE_MAGIC_ADDR + 3, BINDING_STORE_MAGIC & 0xFF) && ok;
        ok = hal::FlashStore::write(BINDING_STORE_VERSION_ADDR, BINDING_STORE_VERSION) && ok;
        for (uint8_t i = 0; i < LINK_UID_SIZE; ++i) {
            ok = hal::FlashStore::write(BINDING_STORE_UID_ADDR + i, uid[i]) && ok;
        }
        ok = writeChecksum(checksum()) && ok;
        return ok && hal::FlashStore::commit();
    }

    bool valid() const {
        uint32_t magic = 0;
        magic |= (uint32_t)hal::FlashStore::read(BINDING_STORE_MAGIC_ADDR + 0) << 24;
        magic |= (uint32_t)hal::FlashStore::read(BINDING_STORE_MAGIC_ADDR + 1) << 16;
        magic |= (uint32_t)hal::FlashStore::read(BINDING_STORE_MAGIC_ADDR + 2) << 8;
        magic |= (uint32_t)hal::FlashStore::read(BINDING_STORE_MAGIC_ADDR + 3);
        if (magic != BINDING_STORE_MAGIC) return false;
        if (hal::FlashStore::read(BINDING_STORE_VERSION_ADDR) != BINDING_STORE_VERSION) return false;

        uint8_t uid[LINK_UID_SIZE];
        loadUid(uid);
        bool any = false;
        for (uint8_t i = 0; i < LINK_UID_SIZE; ++i) {
            if (uid[i] != 0) {
                any = true;
                break;
            }
        }
        if (!any) return false;
        return storedChecksum() == checksum();
    }

    void loadUid(uint8_t uid[LINK_UID_SIZE]) const {
        for (uint8_t i = 0; i < LINK_UID_SIZE; ++i) {
            uid[i] = hal::FlashStore::read(BINDING_STORE_UID_ADDR + i);
        }
    }

    uint32_t checksum() const {
        uint32_t crc = 0x811C9DC5UL;
        const int ranges[][2] = {
            {BINDING_STORE_MAGIC_ADDR, 5},
            {BINDING_STORE_UID_ADDR, LINK_UID_SIZE},
        };
        for (unsigned r = 0; r < sizeof(ranges) / sizeof(ranges[0]); ++r) {
            for (int i = 0; i < ranges[r][1]; ++i) {
                crc ^= hal::FlashStore::read(ranges[r][0] + i);
                crc *= 0x01000193UL;
            }
        }
        return crc;
    }

    uint32_t storedChecksum() const {
        uint32_t value = 0;
        value |= (uint32_t)hal::FlashStore::read(BINDING_STORE_CHECKSUM_ADDR + 0) << 24;
        value |= (uint32_t)hal::FlashStore::read(BINDING_STORE_CHECKSUM_ADDR + 1) << 16;
        value |= (uint32_t)hal::FlashStore::read(BINDING_STORE_CHECKSUM_ADDR + 2) << 8;
        value |= (uint32_t)hal::FlashStore::read(BINDING_STORE_CHECKSUM_ADDR + 3);
        return value;
    }

    bool writeChecksum(uint32_t value) {
        bool ok = true;
        ok = hal::FlashStore::write(BINDING_STORE_CHECKSUM_ADDR + 0, (value >> 24) & 0xFF) && ok;
        ok = hal::FlashStore::write(BINDING_STORE_CHECKSUM_ADDR + 1, (value >> 16) & 0xFF) && ok;
        ok = hal::FlashStore::write(BINDING_STORE_CHECKSUM_ADDR + 2, (value >> 8) & 0xFF) && ok;
        ok = hal::FlashStore::write(BINDING_STORE_CHECKSUM_ADDR + 3, value & 0xFF) && ok;
        return ok;
    }

    uint8_t _uid[LINK_UID_SIZE] = {};
    bool _loaded = false;
};

} // namespace xlrs
