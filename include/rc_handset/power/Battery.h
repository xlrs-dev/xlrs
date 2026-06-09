#pragma once

#include <stdint.h>

namespace rc_handset {
namespace power {

enum class ChargeStatus : uint8_t {
    Unknown = 0,
    NotCharging,
    ConstantCurrent,
    ConstantVoltage,
    TopOff,
};

enum class VbusStatus : uint8_t {
    Unknown = 0,
    NoInput,
    UsbSdp,
    UsbCdp,
    UsbDcp,
    UnknownAdapter,
    NonStandardAdapter,
    Hvdcp,
    OtgMode,
};

struct BatterySnapshot {
    bool chargerPresent = false;
    bool batteryVoltageValid = false;
    uint16_t batteryVoltageMv = 0;
    uint8_t batteryPercent = 0;
    ChargeStatus chargeStatus = ChargeStatus::Unknown;
    VbusStatus vbusStatus = VbusStatus::Unknown;
};

uint8_t batteryPercentFromVoltageMv(uint16_t voltageMv,
                                    uint16_t emptyMv = 3300,
                                    uint16_t fullMv = 4200);

const char* chargeStatusName(ChargeStatus status);
const char* vbusStatusName(VbusStatus status);

}  // namespace power
}  // namespace rc_handset
