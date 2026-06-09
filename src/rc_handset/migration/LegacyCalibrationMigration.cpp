#include "rc_handset/migration/LegacyCalibrationMigration.h"

namespace rc_handset {
namespace migration {
namespace {

constexpr size_t kLegacyCalibrationEndAddr = kLegacyCalibrationDataAddr + kLegacyCalibrationByteSize;

uint16_t readLe16(const uint8_t* storage, size_t offset) {
    return static_cast<uint16_t>(storage[offset] | (static_cast<uint16_t>(storage[offset + 1]) << 8));
}

bool validAxisRange(uint16_t min, uint16_t center, uint16_t max) {
    return min < center && center < max && max <= config::kRcHandsetAdcMax;
}

bool hasRequiredTarget(const CalibrationMigrationAdapter& target) {
    for (uint8_t axis = 0; axis < config::kRcHandsetAxisCount; ++axis) {
        if (!target.axes[axis]) return false;
    }
    return true;
}

bool targetMatches(const CalibrationMigrationAdapter& target, const LegacyCalibration& calibration) {
    for (uint8_t axis = 0; axis < config::kRcHandsetAxisCount; ++axis) {
        if (target.axes[axis]->min != calibration.min[axis] ||
            target.axes[axis]->center != calibration.center[axis] ||
            target.axes[axis]->max != calibration.max[axis]) {
            return false;
        }
    }
    return true;
}

} // namespace

LegacyCalibrationMigrationResult readLegacyCalibration(const uint8_t* storage,
                                                       size_t storageLen,
                                                       LegacyCalibration& calibration) {
    if (!storage || storageLen < kLegacyCalibrationEndAddr) {
        return LegacyCalibrationMigrationResult::StorageTooSmall;
    }
    if (storage[kLegacyCalibrationMagicAddr] != kLegacyCalibrationMagicValue) {
        return LegacyCalibrationMigrationResult::MissingMagic;
    }

    LegacyCalibration parsed{};
    size_t offset = kLegacyCalibrationDataAddr;
    for (uint8_t axis = 0; axis < config::kRcHandsetAxisCount; ++axis) {
        parsed.min[axis] = readLe16(storage, offset);
        offset += sizeof(uint16_t);
    }
    for (uint8_t axis = 0; axis < config::kRcHandsetAxisCount; ++axis) {
        parsed.max[axis] = readLe16(storage, offset);
        offset += sizeof(uint16_t);
    }
    for (uint8_t axis = 0; axis < config::kRcHandsetAxisCount; ++axis) {
        parsed.center[axis] = readLe16(storage, offset);
        offset += sizeof(uint16_t);
    }

    for (uint8_t axis = 0; axis < config::kRcHandsetAxisCount; ++axis) {
        if (!validAxisRange(parsed.min[axis], parsed.center[axis], parsed.max[axis])) {
            return LegacyCalibrationMigrationResult::InvalidAxisRange;
        }
    }

    calibration = parsed;
    return LegacyCalibrationMigrationResult::Migrated;
}

LegacyCalibrationMigrationResult migrateLegacyCalibration(const uint8_t* storage,
                                                          size_t storageLen,
                                                          CalibrationMigrationAdapter target) {
    if (!hasRequiredTarget(target)) {
        return LegacyCalibrationMigrationResult::InvalidTarget;
    }
    if (target.legacyMigrationComplete && *target.legacyMigrationComplete) {
        return LegacyCalibrationMigrationResult::AlreadyMigrated;
    }

    LegacyCalibration parsed{};
    const LegacyCalibrationMigrationResult result = readLegacyCalibration(storage, storageLen, parsed);
    if (result != LegacyCalibrationMigrationResult::Migrated) {
        return result;
    }
    if (targetMatches(target, parsed)) {
        if (target.legacyMigrationComplete) {
            *target.legacyMigrationComplete = true;
        }
        return LegacyCalibrationMigrationResult::AlreadyMigrated;
    }

    for (uint8_t axis = 0; axis < config::kRcHandsetAxisCount; ++axis) {
        target.axes[axis]->min = parsed.min[axis];
        target.axes[axis]->center = parsed.center[axis];
        target.axes[axis]->max = parsed.max[axis];
    }
    if (target.legacyMigrationComplete) {
        *target.legacyMigrationComplete = true;
    }

    return LegacyCalibrationMigrationResult::Migrated;
}

} // namespace migration
} // namespace rc_handset
