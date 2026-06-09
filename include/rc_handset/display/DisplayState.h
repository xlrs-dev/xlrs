#pragma once

#include <stdint.h>

namespace rc_handset {
namespace display {

constexpr uint8_t kDisplayRows = 8;
constexpr uint8_t kDisplayColumns = 21;
constexpr uint8_t kMonitorToggleCount = 4;

enum class Screen : uint8_t {
    Boot,
    MainStatus,
    ChannelMonitor,
    ConfigSummary,
    BindingStatus,
};

struct BatteryState {
    bool available = false;
    uint16_t millivolts = 0;
    uint8_t percent = 0;
};

struct LinkState {
    bool connected = false;
    bool bindingActive = false;
    bool bindingMismatch = false;
    bool metricsAvailable = false;
    const char* rateName = nullptr;
    uint8_t linkQuality = 0;
    int16_t rssiDbm = 0;
    int8_t snrDb = 0;
};

struct ChannelMonitorState {
    bool channelsAvailable = false;
    uint16_t aileron = 0;
    uint16_t elevator = 0;
    uint16_t rudder = 0;
    uint16_t throttle = 0;
    uint16_t toggles[kMonitorToggleCount] = {};
};

struct ConfigSummaryState {
    const char* rateName = nullptr;
    const char* bindingPhrase = nullptr;
    bool calibrationValid = false;
    uint32_t framesSent = 0;
    uint32_t guardRejects = 0;
    uint32_t spikeHolds = 0;
};

struct BindingStatusState {
    bool bindMode = false;
    bool mismatch = false;
    const char* message = nullptr;
};

struct DisplayState {
    Screen screen = Screen::Boot;
    uint8_t bootProgressPercent = 0;
    const char* bootMessage = nullptr;
    BatteryState txBattery;
    BatteryState rxBattery;
    LinkState link;
    ChannelMonitorState channels;
    ConfigSummaryState config;
    BindingStatusState binding;
};

struct DisplayLayout {
    char lines[kDisplayRows][kDisplayColumns + 1] = {};
    uint8_t lineCount = 0;
};

void renderDisplayLayout(const DisplayState& state, DisplayLayout& layout);

} // namespace display
} // namespace rc_handset
