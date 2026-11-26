#pragma once

#include <stdint.h>

#include "lora_link/protocol.h"

namespace rc_handset {
namespace usb {

// Registers the composite USB HID joystick interface. Must be called before
// Serial.begin() so the HID interface is present when USB enumerates.
void hidJoystickBegin();

// Sends a HID joystick report built from the current CRSF channel values.
// Rate-limited internally; safe to call every loop() iteration.
void hidJoystickService(const uint16_t channels[lora_link::kRcChannelCount], uint32_t nowMs);

} // namespace usb
} // namespace rc_handset
