#pragma once
// Persisted TX/RX role for the switchable TX/RX module firmware (TXRX_SWITCHABLE).
//
// Stored in its own EEPROM slot, well clear of ConfigRecord (offset 0, ~50 bytes),
// so the safety-sensitive config schema and offsets are untouched. The slot is
// self-validating (magic + inverted copy of the role); anything else reads as
// "unset" and falls back to the build's default role (RX unless built with
// -DTXRX_DEFAULT_ROLE_TX).

#include <stdint.h>
#include <string.h>

namespace lora_link {

enum class ModuleRole : uint8_t {
    Tx = 1,
    Rx = 2,
};

struct RoleRecord {
    uint32_t magic;
    uint8_t role;
    uint8_t roleInverse;
};

constexpr uint32_t kRoleRecordMagic = 0x454C4F52u;  // "ROLE"
constexpr uint16_t kRoleRecordOffset = 128;
constexpr uint16_t kEepromSize = 256;

inline bool parseModuleRole(const char* text, ModuleRole& out) {
    if (!text) return false;
    if (strcmp(text, "tx") == 0) { out = ModuleRole::Tx; return true; }
    if (strcmp(text, "rx") == 0) { out = ModuleRole::Rx; return true; }
    return false;
}

inline const char* moduleRoleName(ModuleRole role) {
    return role == ModuleRole::Rx ? "rx" : "tx";
}

inline RoleRecord makeRoleRecord(ModuleRole role) {
    RoleRecord record{};
    record.magic = kRoleRecordMagic;
    record.role = static_cast<uint8_t>(role);
    record.roleInverse = static_cast<uint8_t>(~record.role);
    return record;
}

// True and sets `out` only for a well-formed record.
inline bool readRoleRecord(const RoleRecord& record, ModuleRole& out) {
    if (record.magic != kRoleRecordMagic) return false;
    if (record.roleInverse != static_cast<uint8_t>(~record.role)) return false;
    if (record.role != static_cast<uint8_t>(ModuleRole::Tx) &&
        record.role != static_cast<uint8_t>(ModuleRole::Rx)) return false;
    out = static_cast<ModuleRole>(record.role);
    return true;
}

}  // namespace lora_link
