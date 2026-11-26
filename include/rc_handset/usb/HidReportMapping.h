#pragma once

#include <stdint.h>

namespace rc_handset {
namespace usb {

constexpr uint16_t kHidAxisCrsfMin = 172;
constexpr uint16_t kHidAxisCrsfMid = 992;
constexpr uint16_t kHidAxisCrsfMax = 1811;
constexpr int16_t kHidJoystickAxisMax = 32767;

// Maps a CRSF raw channel value (172..1811, center 992) onto a signed 16-bit
// joystick axis (-32767..32767) for a full-resolution USB HID joystick report.
// Used for both the 4 sticks and the 4 three-position AUX switches (which
// report clean low/mid/high values in the same CRSF raw range) -- switches
// are sent as axes, not buttons, so a host can read all three positions
// distinctly (matching how EdgeTX/RadioMaster expose switches over USB).
int16_t crsfChannelToJoystickAxis(uint16_t raw);

} // namespace usb
} // namespace rc_handset
