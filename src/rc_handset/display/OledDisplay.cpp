#include "rc_handset/display/OledDisplay.h"

#include <stdio.h>

#ifndef UNIT_TEST
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Arduino.h>
#include <Fonts/FreeSans9pt7b.h>
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

void printCentered(int16_t centerX, int16_t baseline, const char* text) {
    int16_t x1, y1;
    uint16_t w, h;
    g_oled.getTextBounds(text, 0, baseline, &x1, &y1, &w, &h);
    g_oled.setCursor(centerX - static_cast<int16_t>(w) / 2, baseline);
    g_oled.print(text);
}

// A simple left-to-right filled progress bar (0..100%), for the boot/power
// hold countdown.
void drawProgressBar(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t percent) {
    g_oled.drawRect(x, y, w, h, SH110X_WHITE);
    const int16_t innerW = w - 2;
    const int16_t innerH = h - 2;
    const int16_t fillW = static_cast<int16_t>(
        (static_cast<int32_t>(innerW) * (percent > 100 ? 100 : percent) + 50) / 100);
    if (fillW > 0) g_oled.fillRect(x + 1, y + 1, fillW, innerH, SH110X_WHITE);
}

// Channel values are raw CRSF units (172..1811, 992 mid), matching the
// convention used throughout the rest of the firmware and the webui.
constexpr uint16_t kCrsfRawMin = 172;
constexpr uint16_t kCrsfRawMax = 1811;

uint8_t stickPercent(uint16_t raw) {
    const uint16_t clamped = raw < kCrsfRawMin ? kCrsfRawMin : (raw > kCrsfRawMax ? kCrsfRawMax : raw);
    return static_cast<uint8_t>((static_cast<uint32_t>(clamped - kCrsfRawMin) * 100u) /
                                (kCrsfRawMax - kCrsfRawMin));
}

const char* toggleShortLabel(uint16_t value) {
    if (value < 600) return "L";
    if (value > 1400) return "H";
    return "M";
}

// Small outline battery icon (body + nub) with an internal fill proportional
// to charge percent, top-left of the Battery screen.
void drawBatteryIcon(int16_t x, int16_t y, const BatteryState& battery) {
    g_oled.drawRect(x, y, 16, 8, SH110X_WHITE);
    g_oled.fillRect(static_cast<int16_t>(x + 16), static_cast<int16_t>(y + 2), 2, 4, SH110X_WHITE);
    if (!battery.available || battery.percent == 0) return;
    const uint8_t percent = battery.percent > 100 ? 100 : battery.percent;
    const int16_t fillW = static_cast<int16_t>((14 * percent) / 100);
    if (fillW > 0) g_oled.fillRect(static_cast<int16_t>(x + 1), static_cast<int16_t>(y + 1), fillW, 6, SH110X_WHITE);
}

// One row: "Ax1" label + a horizontal bar filled to the axis's current
// position (0..100% of raw CRSF travel).
void drawAxisBarRow(int16_t y, const char* label, uint16_t raw, bool available) {
    g_oled.setCursor(0, y);
    g_oled.print(label);

    constexpr int16_t barX = 30;
    constexpr int16_t barW = 98;
    g_oled.drawRect(barX, y, barW, 8, SH110X_WHITE);
    if (!available) return;
    const uint8_t percent = stickPercent(raw);
    const int16_t fillW = static_cast<int16_t>(((barW - 2) * percent) / 100);
    if (fillW > 0) g_oled.fillRect(static_cast<int16_t>(barX + 1), static_cast<int16_t>(y + 1), fillW, 6, SH110X_WHITE);
}

void showBatteryScreen(const DisplayState& state) {
    // Top-left: RC's own battery, icon + voltage/percent text beside it.
    drawBatteryIcon(0, 0, state.txBattery);
    char batteryText[16];
    if (!state.txBattery.available) {
        snprintf(batteryText, sizeof(batteryText), "--");
    } else {
        snprintf(batteryText, sizeof(batteryText), "%u.%02uV %u%%",
                 static_cast<unsigned>(state.txBattery.millivolts / 1000),
                 static_cast<unsigned>((state.txBattery.millivolts % 1000) / 10),
                 static_cast<unsigned>(state.txBattery.percent));
    }
    g_oled.setCursor(20, 1);
    g_oled.print(batteryText);

    // Four horizontal axis bars.
    const ChannelMonitorState& ch = state.channels;
    drawAxisBarRow(14, "Ax1", ch.aileron, ch.channelsAvailable);
    drawAxisBarRow(24, "Ax2", ch.elevator, ch.channelsAvailable);
    drawAxisBarRow(34, "Ax3", ch.rudder, ch.channelsAvailable);
    drawAxisBarRow(44, "Ax4", ch.throttle, ch.channelsAvailable);

    // Toggle states underneath.
    char toggles[24];
    snprintf(toggles, sizeof(toggles), "T1:%s T2:%s T3:%s T4:%s",
             toggleShortLabel(ch.toggles[0]), toggleShortLabel(ch.toggles[1]),
             toggleShortLabel(ch.toggles[2]), toggleShortLabel(ch.toggles[3]));
    g_oled.setCursor(0, 56);
    g_oled.print(toggles);
}

// Boot and power-off hold both use this: a title, a filled progress bar, and
// a status/prompt line underneath it (DisplayState.bootProgressPercent /
// bootMessage are reused for both -- Screen::Boot isn't boot-only).
void showBootScreen(const DisplayState& state) {
    g_oled.setFont(&FreeSans9pt7b);
    printCentered(64, 13, "DHANUSH RC");

    drawProgressBar(6, 20, 116, 16, state.bootProgressPercent);

    g_oled.setFont();  // back to the compact built-in font for the message
    if (state.bootMessage && state.bootMessage[0]) {
        int16_t x1, y1;
        uint16_t w, h;
        g_oled.getTextBounds(state.bootMessage, 0, 0, &x1, &y1, &w, &h);
        g_oled.setCursor(static_cast<int16_t>((128 - static_cast<int16_t>(w)) / 2), 50);
        g_oled.print(state.bootMessage);
    }
}
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
    g_oled.clearDisplay();
    g_oled.setTextSize(1);
    g_oled.setTextColor(SH110X_WHITE);

    if (state.screen == Screen::Boot) {
        showBootScreen(state);
    } else if (state.screen == Screen::BatteryStatus) {
        showBatteryScreen(state);
    } else {
        DisplayLayout layout;
        renderDisplayLayout(state, layout);
        for (uint8_t i = 0; i < layout.lineCount; ++i) {
            g_oled.setCursor(0, static_cast<int16_t>(i) * 8);
            g_oled.print(layout.lines[i]);
        }
    }
    g_oled.display();
#else
    (void)state;
#endif
}

void OledDisplay::sleep() {
    if (!available_) return;

#ifndef UNIT_TEST
    // Clear the framebuffer before the panel-level low-power command, so it
    // goes fully dark instead of holding whatever it last showed (e.g. a
    // power-off countdown bar) while off. The I2C bus and controller stay
    // addressable at SH110X_DISPLAYOFF (a few uA); a full re-`begin()` after
    // wake turns the panel back on, so no separate wake() call is needed.
    g_oled.clearDisplay();
    g_oled.display();
    g_oled.oled_command(SH110X_DISPLAYOFF);
#endif
}

} // namespace display
} // namespace rc_handset
