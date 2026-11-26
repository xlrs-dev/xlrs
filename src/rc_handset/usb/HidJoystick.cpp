#ifndef UNIT_TEST

#include "rc_handset/usb/HidJoystick.h"

#include <Adafruit_TinyUSB.h>

#include "rc_handset/usb/HidReportMapping.h"

namespace rc_handset {
namespace usb {

namespace {

constexpr uint32_t kReportIntervalMs = 8;  // ~125 Hz, typical joystick poll rate

// Custom joystick report: 8 full-resolution 16-bit axes -- the 4 sticks
// (X/Y/Z/Rz) and the 4 three-position AUX switches (Rx/Ry/Slider/Dial). The
// switches are sent as axes rather than buttons so a host (ArduPilot, a
// joystick tester) can read all three positions distinctly -- two buttons per
// switch can only signal "low pressed" / "high pressed" / "neither", which
// looks like an on/off input rather than a true 3-way state. This matches how
// EdgeTX/RadioMaster expose switches over USB. TinyUSB's canned
// TUD_HID_REPORT_DESC_GAMEPAD() is also locked to 8-bit axes, too coarse for
// smooth stick tracking in a PC simulator.
uint8_t const desc_hid_report[] = {
    HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),
    HID_USAGE(HID_USAGE_DESKTOP_JOYSTICK),
    HID_COLLECTION(HID_COLLECTION_APPLICATION),
        HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),
        HID_USAGE(HID_USAGE_DESKTOP_X),
        HID_USAGE(HID_USAGE_DESKTOP_Y),
        HID_USAGE(HID_USAGE_DESKTOP_Z),
        HID_USAGE(HID_USAGE_DESKTOP_RZ),
        HID_USAGE(HID_USAGE_DESKTOP_RX),
        HID_USAGE(HID_USAGE_DESKTOP_RY),
        HID_USAGE(HID_USAGE_DESKTOP_SLIDER),
        HID_USAGE(HID_USAGE_DESKTOP_DIAL),
        HID_LOGICAL_MIN_N(0x8001, 2),
        HID_LOGICAL_MAX_N(0x7fff, 2),
        HID_REPORT_COUNT(8),
        HID_REPORT_SIZE(16),
        HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
    HID_COLLECTION_END
};

struct TU_ATTR_PACKED JoystickReport {
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t rz;
    int16_t rx;
    int16_t ry;
    int16_t slider;
    int16_t dial;
};

Adafruit_USBD_HID g_usbHid;
uint32_t g_lastSentMs = 0;

}  // namespace

void hidJoystickBegin() {
    // USB manufacturer/product strings are set at compile time via the
    // USB_MANUFACTURER/USB_PRODUCT build flags -- the RP2040 Arduino core
    // resets them to board defaults in TinyUSB_Device_Init(), which runs
    // before setup(), so a runtime setProductDescriptor() call here would
    // race the host's enumeration and isn't reliable.
    g_usbHid.setPollInterval(2);
    g_usbHid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
    g_usbHid.begin();
}

void hidJoystickService(const uint16_t channels[lora_link::kRcChannelCount], uint32_t nowMs) {
    if (!TinyUSBDevice.mounted() || !g_usbHid.ready()) return;
    if (nowMs - g_lastSentMs < kReportIntervalMs) return;
    g_lastSentMs = nowMs;

    JoystickReport report;
    report.x = crsfChannelToJoystickAxis(channels[0]);       // Aileron / Roll
    report.y = crsfChannelToJoystickAxis(channels[1]);       // Elevator / Pitch
    report.z = crsfChannelToJoystickAxis(channels[3]);       // Throttle
    report.rz = crsfChannelToJoystickAxis(channels[2]);      // Rudder / Yaw
    report.rx = crsfChannelToJoystickAxis(channels[4]);      // AUX1
    report.ry = crsfChannelToJoystickAxis(channels[5]);      // AUX2
    report.slider = crsfChannelToJoystickAxis(channels[6]);  // AUX3
    report.dial = crsfChannelToJoystickAxis(channels[7]);    // AUX4

    g_usbHid.sendReport(0, &report, sizeof(report));
}

}  // namespace usb
}  // namespace rc_handset

#endif  // UNIT_TEST
