#include "rc_handset/display/OledDisplay.h"

#ifndef UNIT_TEST
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Arduino.h>
#include <Wire.h>
#endif

namespace rc_handset {
namespace display {

namespace {

constexpr uint8_t kOledSdaPin = 4;
constexpr uint8_t kOledSclPin = 5;
constexpr uint8_t kOledI2cAddress = 0x3C;
constexpr int8_t kOledResetPin = -1;

#ifndef UNIT_TEST
Adafruit_SH1106G g_oled(128, 64, &Wire, kOledResetPin);
#endif

} // namespace

bool OledDisplay::begin() {
#ifdef UNIT_TEST
    available_ = false;
    return false;
#else
    Wire.setSDA(kOledSdaPin);
    Wire.setSCL(kOledSclPin);
    Wire.begin();

    available_ = g_oled.begin(kOledI2cAddress, true);
    if (!available_) return false;

    g_oled.clearDisplay();
    g_oled.setTextSize(1);
    g_oled.setTextColor(SH110X_WHITE);
    g_oled.setCursor(0, 0);
    g_oled.display();
    return true;
#endif
}

void OledDisplay::show(const DisplayState& state) {
    if (!available_) return;

#ifndef UNIT_TEST
    DisplayLayout layout;
    renderDisplayLayout(state, layout);

    g_oled.clearDisplay();
    g_oled.setTextSize(1);
    g_oled.setTextColor(SH110X_WHITE);
    for (uint8_t i = 0; i < layout.lineCount; ++i) {
        g_oled.setCursor(0, static_cast<int16_t>(i) * 8);
        g_oled.print(layout.lines[i]);
    }
    g_oled.display();
#else
    (void)state;
#endif
}

} // namespace display
} // namespace rc_handset
