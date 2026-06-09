#include "rc_handset/power/Battery.h"

namespace rc_handset {
namespace power {

uint8_t batteryPercentFromVoltageMv(uint16_t voltageMv, uint16_t emptyMv, uint16_t fullMv) {
    if (fullMv <= emptyMv) return 0;
    if (voltageMv <= emptyMv) return 0;
    if (voltageMv >= fullMv) return 100;
    const uint32_t span = static_cast<uint32_t>(fullMv - emptyMv);
    const uint32_t aboveEmpty = static_cast<uint32_t>(voltageMv - emptyMv);
    return static_cast<uint8_t>((aboveEmpty * 100u + (span / 2u)) / span);
}

const char* chargeStatusName(ChargeStatus status) {
    switch (status) {
    case ChargeStatus::Unknown:
        return "unknown";
    case ChargeStatus::NotCharging:
        return "not_charging";
    case ChargeStatus::ConstantCurrent:
        return "constant_current";
    case ChargeStatus::ConstantVoltage:
        return "constant_voltage";
    case ChargeStatus::TopOff:
        return "topoff";
    }
    return "unknown";
}

const char* vbusStatusName(VbusStatus status) {
    switch (status) {
    case VbusStatus::Unknown:
        return "unknown";
    case VbusStatus::NoInput:
        return "no_input";
    case VbusStatus::UsbSdp:
        return "usb_sdp";
    case VbusStatus::UsbCdp:
        return "usb_cdp";
    case VbusStatus::UsbDcp:
        return "usb_dcp";
    case VbusStatus::UnknownAdapter:
        return "unknown_adapter";
    case VbusStatus::NonStandardAdapter:
        return "nonstandard_adapter";
    case VbusStatus::Hvdcp:
        return "hvdcp";
    case VbusStatus::OtgMode:
        return "otg";
    }
    return "unknown";
}

}  // namespace power
}  // namespace rc_handset
