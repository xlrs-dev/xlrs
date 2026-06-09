#pragma once

#include <stddef.h>
#include <stdint.h>

#include "rc_handset/config/config.h"

namespace rc_handset {
namespace migration {

constexpr size_t kLegacyCalibrationEepromSize = 512; // Legacy EEPROM.begin() size.
constexpr size_t kLegacyCalibrationMagicAddr = 150;
constexpr size_t kLegacyCalibrationDataAddr = 152;
constexpr uint8_t kLegacyCalibrationMagicValue = 0xC5;
constexpr size_t kLegacyCalibrationByteSize = config::kRcHandsetAxisCount * 3u * sizeof(uint16_t);

struct LegacyCalibration {
    uint16_t min[config::kRcHandsetAxisCount]{};
    uint16_t center[config::kRcHandsetAxisCount]{};
    uint16_t max[config::kRcHandsetAxisCount]{};
};

struct CalibrationMigrationAdapter {
    config::AxisCalibration* axes[config::kRcHandsetAxisCount]{};
    bool* legacyMigrationComplete = nullptr;
};

enum class LegacyCalibrationMigrationResult : uint8_t {
    Migrated,
    AlreadyMigrated,
    MissingMagic,
    StorageTooSmall,
    InvalidAxisRange,
    InvalidTarget,
};

LegacyCalibrationMigrationResult readLegacyCalibration(const uint8_t* storage,
                                                       size_t storageLen,
                                                       LegacyCalibration& calibration);

LegacyCalibrationMigrationResult migrateLegacyCalibration(const uint8_t* storage,
                                                          size_t storageLen,
                                                          CalibrationMigrationAdapter target);

inline LegacyCalibrationMigrationResult migrateLegacyCalibration(const uint8_t* storage,
                                                                 size_t storageLen,
                                                                 config::RcHandsetConfig& target) {
    CalibrationMigrationAdapter adapter{};
    for (uint8_t axis = 0; axis < config::kRcHandsetAxisCount; ++axis) {
        adapter.axes[axis] = &target.axes[axis].calibration;
    }
    return migrateLegacyCalibration(storage, storageLen, adapter);
}

} // namespace migration
} // namespace rc_handset
