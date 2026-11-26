#include "rc_handset/usb/HidReportMapping.h"

namespace rc_handset {
namespace usb {

int16_t crsfChannelToJoystickAxis(uint16_t raw) {
    if (raw < kHidAxisCrsfMin) raw = kHidAxisCrsfMin;
    if (raw > kHidAxisCrsfMax) raw = kHidAxisCrsfMax;
    const int32_t delta = static_cast<int32_t>(raw) - static_cast<int32_t>(kHidAxisCrsfMid);
    const int32_t span = delta >= 0 ? (kHidAxisCrsfMax - kHidAxisCrsfMid) : (kHidAxisCrsfMid - kHidAxisCrsfMin);
    int32_t scaled = (delta * kHidJoystickAxisMax) / span;
    if (scaled > kHidJoystickAxisMax) scaled = kHidJoystickAxisMax;
    if (scaled < -kHidJoystickAxisMax) scaled = -kHidJoystickAxisMax;
    return static_cast<int16_t>(scaled);
}

} // namespace usb
} // namespace rc_handset
