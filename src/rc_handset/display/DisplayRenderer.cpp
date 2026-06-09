#include "rc_handset/display/DisplayState.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace rc_handset {
namespace display {

namespace {

const char* safeText(const char* text, const char* fallback = "-") {
    return (text && text[0]) ? text : fallback;
}

void clearLayout(DisplayLayout& layout) {
    memset(&layout, 0, sizeof(layout));
}

void addLine(DisplayLayout& layout, const char* fmt, ...) {
    if (layout.lineCount >= kDisplayRows) return;
    va_list args;
    va_start(args, fmt);
    vsnprintf(layout.lines[layout.lineCount], sizeof(layout.lines[layout.lineCount]), fmt, args);
    va_end(args);
    ++layout.lineCount;
}

const char* linkLabel(const LinkState& link) {
    if (link.bindingMismatch) return "MISMATCH";
    if (link.bindingActive) return "BIND";
    return link.connected ? "LINK" : "NO LINK";
}

const char* batteryLabel(const BatteryState& battery, char* buffer, size_t len) {
    if (!battery.available) return "--";
    snprintf(buffer, len, "%u.%02uV %u%%",
             static_cast<unsigned>(battery.millivolts / 1000),
             static_cast<unsigned>((battery.millivolts % 1000) / 10),
             static_cast<unsigned>(battery.percent));
    return buffer;
}

const char* toggleLabel(uint16_t value) {
    if (value < 600) return "L";
    if (value > 1400) return "H";
    return "M";
}

void renderBoot(const DisplayState& state, DisplayLayout& layout) {
    addLine(layout, "XLRS RC HANDSET");
    addLine(layout, "SH1106 OLED");
    addLine(layout, "Boot %3u%%", static_cast<unsigned>(state.bootProgressPercent));
    addLine(layout, "%s", safeText(state.bootMessage, "Starting"));
}

void renderMainStatus(const DisplayState& state, DisplayLayout& layout) {
    char txBattery[16];
    char rxBattery[16];
    addLine(layout, "XLRS %s", linkLabel(state.link));
    addLine(layout, "TX %s", batteryLabel(state.txBattery, txBattery, sizeof(txBattery)));
    addLine(layout, "RX %s", batteryLabel(state.rxBattery, rxBattery, sizeof(rxBattery)));
    addLine(layout, "Rate %s", safeText(state.link.rateName, safeText(state.config.rateName)));
    if (state.link.metricsAvailable) {
        addLine(layout, "LQ %3u RSSI %4d",
                static_cast<unsigned>(state.link.linkQuality),
                static_cast<int>(state.link.rssiDbm));
        addLine(layout, "SNR %4d dB", static_cast<int>(state.link.snrDb));
    } else {
        addLine(layout, "LQ -- RSSI --");
        addLine(layout, "SNR -- dB");
    }
}

void renderChannelMonitor(const DisplayState& state, DisplayLayout& layout) {
    addLine(layout, "CHANNEL MONITOR");
    if (!state.channels.channelsAvailable) {
        addLine(layout, "A ---- E ----");
        addLine(layout, "R ---- T ----");
    } else {
        addLine(layout, "A %4u E %4u",
                static_cast<unsigned>(state.channels.aileron),
                static_cast<unsigned>(state.channels.elevator));
        addLine(layout, "R %4u T %4u",
                static_cast<unsigned>(state.channels.rudder),
                static_cast<unsigned>(state.channels.throttle));
    }
    addLine(layout, "Tog %s %s %s %s",
            toggleLabel(state.channels.toggles[0]),
            toggleLabel(state.channels.toggles[1]),
            toggleLabel(state.channels.toggles[2]),
            toggleLabel(state.channels.toggles[3]));
}

void renderConfigSummary(const DisplayState& state, DisplayLayout& layout) {
    addLine(layout, "CONFIG SUMMARY");
    addLine(layout, "Rate %s", safeText(state.config.rateName));
    addLine(layout, "Bind %s", safeText(state.config.bindingPhrase));
    addLine(layout, "Cal %s", state.config.calibrationValid ? "OK" : "MISSING");
    addLine(layout, "Frames %lu", static_cast<unsigned long>(state.config.framesSent));
    addLine(layout, "Guard %lu Hold %lu",
            static_cast<unsigned long>(state.config.guardRejects),
            static_cast<unsigned long>(state.config.spikeHolds));
}

void renderBindingStatus(const DisplayState& state, DisplayLayout& layout) {
    addLine(layout, "BINDING STATUS");
    if (state.binding.mismatch || state.link.bindingMismatch) {
        addLine(layout, "UID MISMATCH");
    } else if (state.binding.bindMode || state.link.bindingActive) {
        addLine(layout, "Bind active");
    } else if (state.link.connected) {
        addLine(layout, "Receiver matched");
    } else {
        addLine(layout, "Waiting receiver");
    }
    addLine(layout, "%s", safeText(state.binding.message, "No bind update"));
}

} // namespace

void renderDisplayLayout(const DisplayState& state, DisplayLayout& layout) {
    clearLayout(layout);
    switch (state.screen) {
        case Screen::Boot:
            renderBoot(state, layout);
            break;
        case Screen::MainStatus:
            renderMainStatus(state, layout);
            break;
        case Screen::ChannelMonitor:
            renderChannelMonitor(state, layout);
            break;
        case Screen::ConfigSummary:
            renderConfigSummary(state, layout);
            break;
        case Screen::BindingStatus:
            renderBindingStatus(state, layout);
            break;
    }
}

} // namespace display
} // namespace rc_handset
