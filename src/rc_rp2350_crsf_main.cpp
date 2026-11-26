#ifndef UNIT_TEST

#include <Arduino.h>
#include <EEPROM.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <vector>

#include "lora_link/protocol.h"
#include "rc_power_config.h"
#include "rc_handset/config/config.h"
#include "rc_handset/core1/Runtime.h"
#include "rc_handset/display/OledDisplay.h"
#include "rc_handset/input/ArduinoInputSampler.h"
#include "rc_handset/power/Rp2350Power.h"
#include "rc_handset/telemetry/HandsetTelemetry.h"
#include "rc_handset/usb/protocol.h"

#ifdef RC_ENABLE_HID_JOYSTICK
// Hand-built HID joystick over Adafruit TinyUSB -- ported from a working,
// hardware-verified (Mission Planner + Betaflight) implementation. The
// framework's convenience Joystick library (Pico SDK USB stack) was tried
// first and failed to enumerate at all on this RP2350 board, even using
// the library's own unmodified example -- see git history for that
// attempt. This build (rc-rp2350-hid) switches to the Adafruit TinyUSB
// stack specifically for this HID descriptor; every other RC env stays on
// the default Pico SDK stack.
#include "rc_handset/usb/HidJoystick.h"
#endif

using namespace lora_link;
namespace handset_config = rc_handset::config;
using rc_handset::core1::LiveStateSnapshot;
namespace handset_power = rc_handset::power;
namespace handset_input = rc_handset::input;
namespace handset_telemetry = rc_handset::telemetry;
namespace handset_usb = rc_handset::usb;

#ifndef CRSF_UART_TX_PIN
#define CRSF_UART_TX_PIN 8
#endif

#ifndef CRSF_UART_RX_PIN
#define CRSF_UART_RX_PIN 9
#endif

#ifndef RC_CRSF_SIMPLETX_FRAME_US
#define RC_CRSF_SIMPLETX_FRAME_US 2000u
#endif

#ifndef DEFAULT_BINDING_PHRASE
#define DEFAULT_BINDING_PHRASE "default"
#endif

#ifndef RC_TX_BATTERY_ADC_FULL_SCALE_MV
#define RC_TX_BATTERY_ADC_FULL_SCALE_MV 3300u
#endif

#ifndef RC_TX_BATTERY_EMPTY_MV
#define RC_TX_BATTERY_EMPTY_MV 3300u
#endif

#ifndef RC_TX_BATTERY_FULL_MV
#define RC_TX_BATTERY_FULL_MV 4200u
#endif

static constexpr uint16_t kAdcMax = 4095;
static constexpr uint32_t kDisplayIntervalMs = 100;
static constexpr uint32_t kDisplayPageMs = 2500;
static constexpr uint32_t kLinkTelemetryFreshMs = 1500;
static constexpr uint8_t kCrsfAddressFlightController = 0xC8;
static constexpr uint8_t kCrsfAddressRadioTransmitter = 0xEA;
static constexpr uint8_t kCrsfAddressCrsfTransmitter = 0xEE;
static constexpr uint8_t kCrsfFrameLinkStatistics = 0x14;
static constexpr uint8_t kCrsfFrameBatterySensor = 0x08;

static uint32_t g_lastDisplayMs = 0;
static uint32_t g_lastStreamStateMs = 0;
static uint32_t g_lastTxBindingProbeMs = 0;
static uint32_t g_streamStateIntervalMs = 0;
static bool g_streamStateEnabled = false;
static rc_handset::core1::ConfigSnapshot g_activeConfig = rc_handset::core1::makeDefaultConfigSnapshot();
static rc_handset::core1::ConfigSnapshot g_pendingConfig = rc_handset::core1::makeDefaultConfigSnapshot();
static bool g_configFault = false;
static bool g_calibrating = false;
static uint16_t g_calMin[handset_config::kRcHandsetAxisCount] = {};
static uint16_t g_calCenter[handset_config::kRcHandsetAxisCount] = {};
static uint16_t g_calMax[handset_config::kRcHandsetAxisCount] = {};
static LiveStateSnapshot g_liveState = rc_handset::core1::makeDefaultLiveStateSnapshot();
static rc_handset::display::OledDisplay g_display;
static handset_telemetry::HandsetTelemetry g_handsetTelemetry(
    {100u, 1500u, kLinkTelemetryFreshMs, 5000u});
static BindingStatus g_txBindingStatus = {};
static bool g_haveTxBindingStatus = false;

static const char* bindingResultName(BindingResult result) {
    switch (result) {
        case BindingResult::Ok:
            return "ok";
        case BindingResult::InvalidCommand:
            return "invalid_command";
        case BindingResult::InvalidPhrase:
            return "invalid_phrase";
        case BindingResult::PersistFailed:
            return "persist_failed";
        default:
            return "unknown";
    }
}

static std::string uidHexString(const uint8_t uid[kUidSize]) {
    static constexpr char kHex[] = "0123456789ABCDEF";
    std::string out;
    out.reserve(kUidSize * 2);
    for (uint8_t i = 0; i < kUidSize; ++i) {
        out += kHex[(uid[i] >> 4) & 0x0F];
        out += kHex[uid[i] & 0x0F];
    }
    return out;
}

static std::string uidCheckHexString(uint32_t uidCheck) {
    char out[9];
    snprintf(out, sizeof(out), "%08lX", static_cast<unsigned long>(uidCheck));
    return std::string(out);
}

static uint16_t readAveragedAdc(uint8_t pin) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 8; ++i) sum += analogRead(pin);
    return static_cast<uint16_t>((sum + 4) / 8);
}

static uint16_t readAxisAdc(uint8_t axis) {
    const handset_input::ArduinoInputPins pins = handset_input::defaultRp2350InputPins();
    if (axis >= handset_config::kRcHandsetAxisCount) return 0;
    return readAveragedAdc(pins.analog[axis]);
}

static std::string joinU16(const uint16_t* values, uint8_t count) {
    std::string out;
    for (uint8_t i = 0; i < count; ++i) {
        if (i) out += ",";
        out += std::to_string(values[i]);
    }
    return out;
}

static std::string configCalMinString(const handset_config::RcHandsetConfig& cfg) {
    uint16_t values[handset_config::kRcHandsetAxisCount];
    for (uint8_t i = 0; i < handset_config::kRcHandsetAxisCount; ++i) values[i] = cfg.axes[i].calibration.min;
    return joinU16(values, handset_config::kRcHandsetAxisCount);
}

static std::string configCalCenterString(const handset_config::RcHandsetConfig& cfg) {
    uint16_t values[handset_config::kRcHandsetAxisCount];
    for (uint8_t i = 0; i < handset_config::kRcHandsetAxisCount; ++i) values[i] = cfg.axes[i].calibration.center;
    return joinU16(values, handset_config::kRcHandsetAxisCount);
}

static std::string configCalMaxString(const handset_config::RcHandsetConfig& cfg) {
    uint16_t values[handset_config::kRcHandsetAxisCount];
    for (uint8_t i = 0; i < handset_config::kRcHandsetAxisCount; ++i) values[i] = cfg.axes[i].calibration.max;
    return joinU16(values, handset_config::kRcHandsetAxisCount);
}

static std::string configAxisBoolString(const handset_config::RcHandsetConfig& cfg) {
    std::string out;
    for (uint8_t i = 0; i < handset_config::kRcHandsetAxisCount; ++i) {
        if (i) out += ",";
        out += cfg.axes[i].inverted ? "1" : "0";
    }
    return out;
}

static std::string configAxisDeadzoneString(const handset_config::RcHandsetConfig& cfg) {
    uint16_t values[handset_config::kRcHandsetAxisCount];
    for (uint8_t i = 0; i < handset_config::kRcHandsetAxisCount; ++i) values[i] = cfg.axes[i].deadzone;
    return joinU16(values, handset_config::kRcHandsetAxisCount);
}

static std::string configAxisFunctionString(const handset_config::RcHandsetConfig& cfg) {
    std::string out;
    for (uint8_t i = 0; i < handset_config::kRcHandsetAxisCount; ++i) {
        if (i) out += ",";
        out += std::to_string(static_cast<unsigned>(cfg.axes[i].function));
    }
    return out;
}

static std::string configChannelTrimString(const handset_config::RcHandsetConfig& cfg) {
    std::string out;
    for (uint8_t i = 0; i < handset_config::kRcHandsetChannelCount; ++i) {
        if (i) out += ",";
        out += std::to_string(cfg.channels[i].trim);
    }
    return out;
}

static std::string configChannelCutoffMinString(const handset_config::RcHandsetConfig& cfg) {
    uint16_t values[handset_config::kRcHandsetChannelCount];
    for (uint8_t i = 0; i < handset_config::kRcHandsetChannelCount; ++i) values[i] = cfg.channels[i].cutoffMin;
    return joinU16(values, handset_config::kRcHandsetChannelCount);
}

static std::string configChannelCutoffMaxString(const handset_config::RcHandsetConfig& cfg) {
    uint16_t values[handset_config::kRcHandsetChannelCount];
    for (uint8_t i = 0; i < handset_config::kRcHandsetChannelCount; ++i) values[i] = cfg.channels[i].cutoffMax;
    return joinU16(values, handset_config::kRcHandsetChannelCount);
}

static std::string configFilterString(const handset_config::RcHandsetConfig& cfg) {
    return std::to_string(cfg.filter.adcSamples) + "," +
           std::to_string(cfg.filter.smoothingPercent) + ",0," +
           std::to_string(cfg.filter.highPassPercent);
}

static std::vector<handset_usb::ResponseField> configResponseFields(const handset_config::RcHandsetConfig& cfg) {
    return {
        {"version", "1"},
        {"cal_min", configCalMinString(cfg)},
        {"cal_center", configCalCenterString(cfg)},
        {"cal_max", configCalMaxString(cfg)},
        {"invert", configAxisBoolString(cfg)},
        {"deadzone", configAxisDeadzoneString(cfg)},
        {"function", configAxisFunctionString(cfg)},
        {"trim", configChannelTrimString(cfg)},
        {"cutoff_min", configChannelCutoffMinString(cfg)},
        {"cutoff_max", configChannelCutoffMaxString(cfg)},
        {"filter", configFilterString(cfg)},
        {"filter.low_pass", std::to_string(cfg.filter.smoothingPercent)},
        {"filter.high_pass", std::to_string(cfg.filter.highPassPercent)},
    };
}

static bool parseInt(const std::string& value, int32_t& out) {
    if (value.empty()) return false;
    char* end = nullptr;
    const long parsed = strtol(value.c_str(), &end, 10);
    if (!end || *end != '\0') return false;
    out = static_cast<int32_t>(parsed);
    return true;
}

static bool parseIndexField(const std::string& field, const char* prefix, uint8_t limit, uint8_t& index) {
    const size_t prefixLen = strlen(prefix);
    if (field.compare(0, prefixLen, prefix) != 0) return false;
    int32_t parsed = 0;
    if (!parseInt(field.substr(prefixLen), parsed) || parsed < 0 || parsed >= limit) return false;
    index = static_cast<uint8_t>(parsed);
    return true;
}

static bool parseCsvInts(const std::string& value, int32_t* out, uint8_t count) {
    const char* cursor = value.c_str();
    char* end = nullptr;
    for (uint8_t i = 0; i < count; ++i) {
        out[i] = static_cast<int32_t>(strtol(cursor, &end, 10));
        if (end == cursor) return false;
        if (i < count - 1u) {
            if (*end != ',') return false;
            cursor = end + 1;
        } else if (*end != '\0') {
            return false;
        }
    }
    return true;
}

static bool applyConfigField(handset_config::RcHandsetConfig& cfg,
                             const std::string& field,
                             const std::string& value) {
    int32_t parsed = 0;
    uint8_t index = 0;

    if (field == "filter") {
        int32_t values[4] = {0, 0, 0, 0};
        if (!parseCsvInts(value, values, 4)) return false;
        cfg.filter.adcSamples = static_cast<uint8_t>(values[0]);
        cfg.filter.smoothingPercent = static_cast<uint8_t>(values[1]);
        cfg.filter.highPassPercent = static_cast<uint8_t>(values[3]);
        return handset_config::validateRcHandsetConfig(cfg);
    }

    if (parseIndexField(field, "cal.axis.", handset_config::kRcHandsetAxisCount, index)) {
        int32_t values[3] = {0, 0, 0};
        if (!parseCsvInts(value, values, 3)) return false;
        cfg.axes[index].calibration.min = static_cast<uint16_t>(values[0]);
        cfg.axes[index].calibration.center = static_cast<uint16_t>(values[1]);
        cfg.axes[index].calibration.max = static_cast<uint16_t>(values[2]);
        return handset_config::validateRcHandsetConfig(cfg);
    }

    if (!parseInt(value, parsed)) return false;

    if (parseIndexField(field, "cal.min.", handset_config::kRcHandsetAxisCount, index)) {
        cfg.axes[index].calibration.min = static_cast<uint16_t>(parsed);
    } else if (parseIndexField(field, "cal.center.", handset_config::kRcHandsetAxisCount, index)) {
        cfg.axes[index].calibration.center = static_cast<uint16_t>(parsed);
    } else if (parseIndexField(field, "cal.max.", handset_config::kRcHandsetAxisCount, index)) {
        cfg.axes[index].calibration.max = static_cast<uint16_t>(parsed);
    } else if (parseIndexField(field, "axis.invert.", handset_config::kRcHandsetAxisCount, index)) {
        cfg.axes[index].inverted = parsed != 0;
    } else if (parseIndexField(field, "axis.deadzone.", handset_config::kRcHandsetAxisCount, index)) {
        cfg.axes[index].deadzone = static_cast<uint16_t>(parsed);
    } else if (parseIndexField(field, "axis.function.", handset_config::kRcHandsetAxisCount, index)) {
        cfg.axes[index].function = static_cast<handset_config::ChannelFunction>(parsed);
    } else if (parseIndexField(field, "channel.trim.", handset_config::kRcHandsetChannelCount, index)) {
        cfg.channels[index].trim = static_cast<int16_t>(parsed);
    } else if (parseIndexField(field, "channel.cutoff_min.", handset_config::kRcHandsetChannelCount, index)) {
        cfg.channels[index].cutoffMin = static_cast<uint16_t>(parsed);
    } else if (parseIndexField(field, "channel.cutoff_max.", handset_config::kRcHandsetChannelCount, index)) {
        cfg.channels[index].cutoffMax = static_cast<uint16_t>(parsed);
    } else if (field == "filter.low_pass") {
        cfg.filter.smoothingPercent = static_cast<uint8_t>(parsed);
    } else if (field == "filter.high_pass") {
        cfg.filter.highPassPercent = static_cast<uint8_t>(parsed);
    } else {
        return false;
    }
    return handset_config::validateRcHandsetConfig(cfg);
}

static void publishConfigForLiveEdit();
static void serviceStateStream();

static bool saveRcConfig(const handset_config::RcHandsetConfig& config) {
    uint8_t record[handset_config::kRcHandsetConfigRecordSize];
    size_t written = 0;
    if (!handset_config::encodeRcHandsetConfigRecord(config, record, sizeof(record), written)) {
        return false;
    }
    for (size_t i = 0; i < written; ++i) EEPROM.write(i, record[i]);
    return EEPROM.commit();
}

static bool applyRcConfigForLiveEdit(const handset_config::RcHandsetConfig& config) {
    if (!handset_config::validateRcHandsetConfig(config)) return false;
    g_activeConfig.handsetConfig = config;
    publishConfigForLiveEdit();
    return true;
}

static bool commitRcConfig(const handset_config::RcHandsetConfig& config) {
    if (!applyRcConfigForLiveEdit(config)) return false;
    return saveRcConfig(config);
}

static void loadRcConfig() {
    EEPROM.begin(256);
    uint8_t record[handset_config::kRcHandsetConfigRecordSize];
    for (size_t i = 0; i < sizeof(record); ++i) record[i] = EEPROM.read(i);
    handset_config::RcHandsetConfig stored{};
    if (handset_config::decodeRcHandsetConfigRecord(record, sizeof(record), stored)) {
        g_activeConfig.handsetConfig = stored;
        g_configFault = false;
    } else {
        g_activeConfig.handsetConfig = handset_config::defaultRcHandsetConfig();
        g_configFault = false;
    }
    g_activeConfig.crsfFrameIntervalUs = rc_handset::core1::kDefaultCrsfFrameIntervalUs;
    g_pendingConfig = g_activeConfig;
}

static void publishActiveConfig() {
    g_activeConfig.generation = rc_handset::core1::publishConfig(g_activeConfig);
    g_pendingConfig = g_activeConfig;
}

static void publishConfigForLiveEdit() {
    rc_handset::core1::setCrsfOutputSafetyHold(false);
    publishActiveConfig();
}

static void refreshLiveState();

static void collectCurrentAdc(uint16_t out[handset_config::kRcHandsetAxisCount]) {
    refreshLiveState();
    if (g_liveState.haveAdc) {
        for (uint8_t axis = 0; axis < handset_config::kRcHandsetAxisCount; ++axis) {
            out[axis] = g_liveState.rawAdc[axis];
        }
        return;
    }
    for (uint8_t axis = 0; axis < handset_config::kRcHandsetAxisCount; ++axis) {
        out[axis] = readAxisAdc(axis);
    }
}

static std::string currentAdcString() {
    uint16_t values[handset_config::kRcHandsetAxisCount];
    for (uint8_t axis = 0; axis < handset_config::kRcHandsetAxisCount; ++axis) {
        values[axis] = g_liveState.haveAdc ? g_liveState.rawAdc[axis] : 0;
    }
    return joinU16(values, handset_config::kRcHandsetAxisCount);
}

static std::string currentFilteredAdcString() {
    uint16_t values[handset_config::kRcHandsetAxisCount];
    for (uint8_t axis = 0; axis < handset_config::kRcHandsetAxisCount; ++axis) {
        values[axis] = g_liveState.haveAdc ? g_liveState.filteredAdc[axis] : 0;
    }
    return joinU16(values, handset_config::kRcHandsetAxisCount);
}

static void refreshLiveState() {
    rc_handset::core1::readLiveState(g_liveState);
}

static bool isCrsfAddress(uint8_t value) {
    return value == kCrsfAddressFlightController ||
           value == kCrsfAddressRadioTransmitter ||
           value == kCrsfAddressCrsfTransmitter;
}

static bool parseCrsfTelemetryByte(uint8_t byte, uint8_t frame[64], uint8_t& frameLen) {
    static uint8_t pos = 0;
    if (pos == 0 && !isCrsfAddress(byte)) return false;
    frame[pos++] = byte;
    if (pos == 2 && (frame[1] < 2 || frame[1] > 62)) {
        pos = 0;
        return false;
    }
    if (pos >= 2 && pos == static_cast<uint8_t>(frame[1] + 2)) {
        frameLen = pos;
        pos = 0;
        return crsfCrc8(&frame[2], frame[1] - 1) == frame[frameLen - 1];
    }
    if (pos >= 64) pos = 0;
    return false;
}

static RateId rateIdFromCrsfRfMode(uint8_t rfMode) {
    if (rfMode == static_cast<uint8_t>(RateId::L250)) return RateId::L250;
    if (rfMode == static_cast<uint8_t>(RateId::L50)) return RateId::L50;
    return RateId::L100;
}

static uint32_t defaultBindingUidCheck() {
    uint8_t uid[kUidSize];
    deriveUid(DEFAULT_BINDING_PHRASE, uid);
    return uidCheck(uid);
}

static void cacheTxBindingStatus(const BindingStatus& status) {
    g_txBindingStatus = status;
    g_haveTxBindingStatus = true;
}

static const char* displayBindingPhrase() {
    return g_haveTxBindingStatus ? g_txBindingStatus.phrase : DEFAULT_BINDING_PHRASE;
}

static uint32_t expectedTxUidCheck() {
    return g_haveTxBindingStatus ? g_txBindingStatus.uidCheck : defaultBindingUidCheck();
}

static void publishTxBindingTelemetry(bool present, const BindingStatus* status, uint32_t nowMs) {
    if (status) cacheTxBindingStatus(*status);
    const handset_telemetry::HandsetTelemetryState& state = g_handsetTelemetry.state();
    const bool haveExpectedUid = g_haveTxBindingStatus || status != nullptr;
    const uint32_t expectedUid = status ? status->uidCheck : expectedTxUidCheck();
    const bool haveObservedUid = status != nullptr || state.haveObservedUidCheck;
    const uint32_t observedUid = status ? status->uidCheck : state.observedUidCheck;
    g_handsetTelemetry.updateTxStatus(present,
                                      haveExpectedUid,
                                      expectedUid,
                                      haveObservedUid,
                                      observedUid,
                                      state.activeRate,
                                      nowMs);
}

static uint8_t percentFromMillivolts(uint16_t millivolts) {
    if (millivolts <= RC_TX_BATTERY_EMPTY_MV) return 0;
    if (millivolts >= RC_TX_BATTERY_FULL_MV) return 100;
    return static_cast<uint8_t>(
        ((static_cast<uint32_t>(millivolts - RC_TX_BATTERY_EMPTY_MV) * 100u) +
         ((RC_TX_BATTERY_FULL_MV - RC_TX_BATTERY_EMPTY_MV) / 2u)) /
        (RC_TX_BATTERY_FULL_MV - RC_TX_BATTERY_EMPTY_MV));
}

static rc_handset::display::BatteryState readTxBattery() {
    rc_handset::display::BatteryState battery{};
    const handset_power::BatterySnapshot powerBattery = handset_power::rcPowerSnapshot();
    if (powerBattery.batteryVoltageValid) {
        battery.available = true;
        battery.millivolts = powerBattery.batteryVoltageMv;
        battery.percent = powerBattery.batteryPercent;
        return battery;
    }
#ifdef RC_TX_BATTERY_ADC_PIN
    const uint16_t raw = readAveragedAdc(RC_TX_BATTERY_ADC_PIN);
    battery.available = true;
    battery.millivolts = static_cast<uint16_t>(
        (static_cast<uint32_t>(raw) * RC_TX_BATTERY_ADC_FULL_SCALE_MV + (kAdcMax / 2u)) / kAdcMax);
    battery.percent = percentFromMillivolts(battery.millivolts);
#endif
    return battery;
}

static void handleCrsfTelemetryFrame(const uint8_t frame[64], uint8_t frameLen) {
    if (frameLen < 4) return;
    const uint8_t type = frame[2];
    if (type == kCrsfFrameLinkStatistics && frame[1] == 12) {
        const uint32_t nowMs = millis();
        const int16_t rssiDbm = -static_cast<int16_t>(frame[3]);
        const uint8_t linkQuality = frame[5];
        const int8_t snrDb = static_cast<int8_t>(frame[6]);
        const handset_telemetry::HandsetTelemetryState& state = g_handsetTelemetry.state();
        g_handsetTelemetry.updateTxStatus(true,
                                          g_haveTxBindingStatus || state.haveExpectedUidCheck,
                                          g_haveTxBindingStatus ? g_txBindingStatus.uidCheck : state.expectedUidCheck,
                                          state.haveObservedUidCheck,
                                          state.observedUidCheck,
                                          rateIdFromCrsfRfMode(frame[8]),
                                          nowMs);
        g_handsetTelemetry.updateLoRaLink(linkQuality, rssiDbm, snrDb, nowMs);
    } else if (type == kCrsfFrameBatterySensor && frame[1] >= 10) {
        const uint16_t decivolts = static_cast<uint16_t>((frame[3] << 8) | frame[4]);
        if (decivolts > 0) {
            g_handsetTelemetry.updateRxBattery(static_cast<uint16_t>(decivolts * 100u),
                                               frame[10],
                                               millis());
        }
    }
}

static void serviceCrsfTelemetry() {
    uint8_t frame[64];
    uint8_t frameLen = 0;
    while (Serial2.available()) {
        const uint8_t byte = static_cast<uint8_t>(Serial2.read());
        BindingStatus ignoredStatus{};
        BindingResult ignoredResult = BindingResult::InvalidCommand;
        BindingControlOp ignoredOp = BindingControlOp::Get;
        parseCrsfBindingResponseFrame(byte, ignoredStatus, ignoredResult, ignoredOp);
        if (parseCrsfTelemetryByte(byte, frame, frameLen)) {
            handleCrsfTelemetryFrame(frame, frameLen);
        }
    }
}

static bool proxyTxBindingRequest(const BindingControlRequest& request, BindingStatus& status,
                                  BindingResult& result, BindingControlOp& responseOp) {
    uint8_t frame[64] = {};
    const size_t frameLen = encodeCrsfBindingRequestFrame(request, frame, sizeof(frame));
    if (frameLen == 0) return false;
    Serial2.write(frame, frameLen);

    uint8_t telemetryFrame[64];
    uint8_t telemetryFrameLen = 0;
    const uint32_t startMs = millis();
    while (static_cast<uint32_t>(millis() - startMs) < 750u) {
        while (Serial2.available()) {
            const uint8_t byte = static_cast<uint8_t>(Serial2.read());
            if (parseCrsfBindingResponseFrame(byte, status, result, responseOp) && responseOp == request.op) {
                if (result == BindingResult::Ok) publishTxBindingTelemetry(true, &status, millis());
                return true;
            }
            if (parseCrsfTelemetryByte(byte, telemetryFrame, telemetryFrameLen)) {
                handleCrsfTelemetryFrame(telemetryFrame, telemetryFrameLen);
            }
        }
        handset_power::rcPowerService();
        rc_handset::core1::loopOnceFallback();
        // The WebUI's live state stream is otherwise only serviced from the
        // main loop -- without this, every binding probe (every 2s until the
        // first reply, then every 30s) stalls the stream for up to 750ms.
        serviceStateStream();
#ifdef RC_ENABLE_HID_JOYSTICK
        // Same reasoning: without this, the HID joystick report freezes for
        // the same 750ms window every retry -- which with no TX ever
        // attached (the sim/joystick-only use case) means every ~2s,
        // forever. That's the "values sometimes stuck" symptom.
        refreshLiveState();
        handset_usb::hidJoystickService(g_liveState.channels, millis());
#endif
        yield();
    }
    return false;
}

static void serviceTxBindingProbe() {
    const uint32_t nowMs = millis();
    const uint32_t intervalMs = g_haveTxBindingStatus ? 30000u : 2000u;
    if (g_lastTxBindingProbeMs != 0 &&
        static_cast<uint32_t>(nowMs - g_lastTxBindingProbeMs) < intervalMs) {
        return;
    }
    g_lastTxBindingProbeMs = nowMs;

    BindingControlRequest request{};
    request.op = BindingControlOp::Get;
    BindingStatus status{};
    BindingResult result = BindingResult::InvalidCommand;
    BindingControlOp responseOp = BindingControlOp::Get;
    if (!proxyTxBindingRequest(request, status, result, responseOp) || result != BindingResult::Ok) {
        publishTxBindingTelemetry(false, nullptr, nowMs);
    }
}

static void printBindingResponse(const handset_usb::ParsedCommand& command, const BindingStatus& status,
                                 BindingResult result, BindingControlOp op) {
    if (result != BindingResult::Ok) {
        Serial.print(handset_usb::formatErr(command, bindingResultName(result), "TX binding command failed").c_str());
        return;
    }
    Serial.print(handset_usb::formatOk(command, {
        {"target", "tx"},
        {"op", handset_usb::commandTypeName(command.type)},
        {"result", bindingResultName(result)},
        {"phrase", status.phrase},
        {"uid", uidHexString(status.uid)},
        {"uid_check", uidCheckHexString(status.uidCheck)},
        {"persisted", status.persisted ? "1" : "0"},
        {"requires_reboot", status.requiresReboot ? "1" : "0"},
    }).c_str());
}

static const handset_usb::Argument* commandArg(const handset_usb::ParsedCommand& command, const char* key) {
    for (const handset_usb::Argument& arg : command.arguments) {
        if (arg.key == key) return &arg;
    }
    return nullptr;
}

static std::vector<handset_usb::ResponseField> stateResponseFields() {
    refreshLiveState();
    const uint32_t nowMs = millis();
    if (g_liveState.haveChannels) {
        g_handsetTelemetry.updateLocalChannels(g_liveState.channels, nowMs);
    }
    handset_telemetry::PacketCounters counters{};
    counters.uplinkTransmitAttempts = g_liveState.framesSent;
    counters.uplinkTransmitSuccesses = g_liveState.framesSent;
    counters.rejectedOtaFrames = g_liveState.channelGuardRejects;
    counters.lostPackets = g_liveState.channelSpikeHolds;
    g_handsetTelemetry.updatePacketCounters(counters, nowMs);
    g_handsetTelemetry.setConfigFault(g_configFault, nowMs);
    g_handsetTelemetry.refresh(nowMs);
    const handset_telemetry::HandsetTelemetryState& telemetry = g_handsetTelemetry.state();

    uint16_t toggles[4] = {};
    for (uint8_t i = 0; i < 4; ++i) toggles[i] = g_liveState.channels[i + 4];
    const rc_handset::display::BatteryState txBattery = readTxBattery();
    std::vector<handset_usb::ResponseField> fields = {
        {"adc", currentAdcString()},
        {"adc_filtered", currentFilteredAdcString()},
        {"ch", joinU16(g_liveState.channels, lora_link::kRcChannelCount)},
        {"toggles", joinU16(toggles, 4)},
        {"have_channels", g_liveState.haveChannels ? "1" : "0"},
        {"safety_hold", g_liveState.safetyHold ? "1" : "0"},
        {"lq", std::to_string(telemetry.linkQuality)},
        {"rssi", std::to_string(telemetry.rssiDbm)},
        {"snr", std::to_string(telemetry.snrDb)},
        {"tx_present", telemetry.txStatus != handset_telemetry::FieldStatus::Missing ? "1" : "0"},
        {"tx_status", handset_telemetry::fieldStatusName(telemetry.txStatus)},
        {"link_status", handset_telemetry::fieldStatusName(telemetry.linkStatsStatus)},
        {"uid", telemetry.faults.bindingUidMismatch ? "mismatch" : (telemetry.faults.bindingUidMissing ? "missing" : "match")},
    };
    if (txBattery.available) {
        fields.push_back({"tx_batt_mv", std::to_string(txBattery.millivolts)});
    }
    if (telemetry.rxBatteryStatus != handset_telemetry::FieldStatus::Missing) {
        fields.push_back({"rx_batt_mv", std::to_string(telemetry.rxBattery.millivolts)});
    }
    if (g_haveTxBindingStatus) {
        fields.push_back({"tx_phrase", g_txBindingStatus.phrase});
        fields.push_back({"tx_uid", uidHexString(g_txBindingStatus.uid)});
        fields.push_back({"tx_uid_check", uidCheckHexString(g_txBindingStatus.uidCheck)});
        fields.push_back({"tx_requires_reboot", g_txBindingStatus.requiresReboot ? "1" : "0"});
    }
    return fields;
}

static bool handleRcV1Command(const char* line) {
    if (strncmp(line, "rc.v1", 5) != 0) return false;

    const handset_usb::ParseResult parsed = handset_usb::parseLine(line);
    if (!parsed.ok) {
        Serial.print(handset_usb::formatErr(false, 0, handset_usb::parseErrorName(parsed.error),
                                            parsed.errorMessage).c_str());
        return true;
    }

    const handset_usb::ParsedCommand& command = parsed.command;
    switch (command.type) {
        case handset_usb::CommandType::Hello:
            Serial.print(handset_usb::formatOk(command, {
                {"role", "rc_handset"},
                {"fw", "0.1"},
                {"caps", "state,stream_state,config,calibration,binding,tx_proxy"},
            }).c_str());
            return true;

        case handset_usb::CommandType::TxHello: {
            BindingControlRequest request{};
            request.op = BindingControlOp::Get;
            BindingStatus status{};
            BindingResult result = BindingResult::InvalidCommand;
            BindingControlOp responseOp = BindingControlOp::Get;
            if (!proxyTxBindingRequest(request, status, result, responseOp) || result != BindingResult::Ok) {
                publishTxBindingTelemetry(false, nullptr, millis());
                Serial.print(handset_usb::formatOk(command, {{"tx_present", "0"}}).c_str());
                return true;
            }
            Serial.print(handset_usb::formatOk(command, {
                {"tx_present", "1"},
                {"phrase", status.phrase},
                {"uid", uidHexString(status.uid)},
                {"uid_check", uidCheckHexString(status.uidCheck)},
                {"persisted", status.persisted ? "1" : "0"},
                {"requires_reboot", status.requiresReboot ? "1" : "0"},
            }).c_str());
            return true;
        }

        case handset_usb::CommandType::BindingGet:
        case handset_usb::CommandType::BindingSet:
        case handset_usb::CommandType::BindingClear:
        case handset_usb::CommandType::BindingVerify: {
            if (!command.target.empty() && command.target != "tx") {
                Serial.print(handset_usb::formatErr(command, "unsupported_target",
                                                    "RC USB can only proxy target=tx").c_str());
                return true;
            }

            BindingControlRequest request{};
            if (command.type == handset_usb::CommandType::BindingGet) request.op = BindingControlOp::Get;
            else if (command.type == handset_usb::CommandType::BindingSet) request.op = BindingControlOp::Set;
            else if (command.type == handset_usb::CommandType::BindingClear) request.op = BindingControlOp::Clear;
            else request.op = BindingControlOp::Verify;

            if ((request.op == BindingControlOp::Set || request.op == BindingControlOp::Verify)) {
                if (!validateBindingPhrase(command.value.c_str())) {
                    Serial.print(handset_usb::formatErr(command, "invalid_phrase",
                                                        "phrase must be 1..32 printable bytes").c_str());
                    return true;
                }
                strncpy(request.phrase, command.value.c_str(), sizeof(request.phrase) - 1);
            }

            BindingStatus status{};
            BindingResult result = BindingResult::InvalidCommand;
            BindingControlOp responseOp = BindingControlOp::Get;
            if (!proxyTxBindingRequest(request, status, result, responseOp)) {
                Serial.print(handset_usb::formatErr(command, "tx_timeout",
                                                    "no TX binding response on CRSF UART").c_str());
                return true;
            }
            printBindingResponse(command, status, result, responseOp);
            return true;
        }

        case handset_usb::CommandType::GetConfig:
            Serial.print(handset_usb::formatOk(command, configResponseFields(g_activeConfig.handsetConfig)).c_str());
            return true;

        case handset_usb::CommandType::SetConfig: {
            handset_config::RcHandsetConfig candidate = g_activeConfig.handsetConfig;
            if (!applyConfigField(candidate, command.field, command.value)) {
                Serial.print(handset_usb::formatErr(command, "invalid_config_field",
                                                    "field or value is invalid").c_str());
                return true;
            }
            if (!applyRcConfigForLiveEdit(candidate)) {
                Serial.print(handset_usb::formatErr(command, "apply_failed",
                                                    "validated config could not be applied").c_str());
                return true;
            }
            Serial.print(handset_usb::formatOk(command, {
                {"field", command.field},
                {"value", command.value},
                {"persisted", "0"},
                {"generation", std::to_string(g_activeConfig.generation)},
            }).c_str());
            return true;
        }

        case handset_usb::CommandType::Apply:
            Serial.print(handset_usb::formatOk(command, {
                {"generation", std::to_string(g_activeConfig.generation)},
                {"persisted", "0"},
            }).c_str());
            return true;

        case handset_usb::CommandType::Save:
            if (saveRcConfig(g_activeConfig.handsetConfig)) {
                Serial.print(handset_usb::formatOk(command, {{"persisted", "1"}}).c_str());
            } else {
                Serial.print(handset_usb::formatErr(command, "persist_failed",
                                                    "failed to encode or commit config").c_str());
            }
            return true;

        case handset_usb::CommandType::ResetDefaults:
            if (!command.target.empty() && command.target != "rc_config") {
                Serial.print(handset_usb::formatErr(command, "unsupported_target",
                                                    "reset_defaults supports target=rc_config").c_str());
                return true;
            }
            if (!commitRcConfig(handset_config::defaultRcHandsetConfig())) {
                Serial.print(handset_usb::formatErr(command, "persist_failed",
                                                    "default config could not be saved").c_str());
                return true;
            }
            Serial.print(handset_usb::formatOk(command, configResponseFields(g_activeConfig.handsetConfig)).c_str());
            return true;

        case handset_usb::CommandType::CalStart:
            collectCurrentAdc(g_calCenter);
            memcpy(g_calMin, g_calCenter, sizeof(g_calMin));
            memcpy(g_calMax, g_calCenter, sizeof(g_calMax));
            g_calibrating = true;
            rc_handset::core1::setCrsfOutputSafetyHold(true);
            Serial.print(handset_usb::formatOk(command, {
                {"adc", joinU16(g_calCenter, handset_config::kRcHandsetAxisCount)},
                {"safety_hold", "1"},
            }).c_str());
            return true;

        case handset_usb::CommandType::CalSample: {
            if (!g_calibrating) {
                Serial.print(handset_usb::formatErr(command, "not_calibrating",
                                                    "run cal_start before cal_sample").c_str());
                return true;
            }
            uint16_t adc[handset_config::kRcHandsetAxisCount];
            collectCurrentAdc(adc);
            for (uint8_t axis = 0; axis < handset_config::kRcHandsetAxisCount; ++axis) {
                if (adc[axis] < g_calMin[axis]) g_calMin[axis] = adc[axis];
                if (adc[axis] > g_calMax[axis]) g_calMax[axis] = adc[axis];
            }
            Serial.print(handset_usb::formatOk(command, {
                {"adc", joinU16(adc, handset_config::kRcHandsetAxisCount)},
                {"cal_min", joinU16(g_calMin, handset_config::kRcHandsetAxisCount)},
                {"cal_max", joinU16(g_calMax, handset_config::kRcHandsetAxisCount)},
            }).c_str());
            return true;
        }

        case handset_usb::CommandType::CalFinish: {
            if (!g_calibrating) {
                Serial.print(handset_usb::formatErr(command, "not_calibrating",
                                                    "run cal_start before cal_finish").c_str());
                return true;
            }
            bool calibrationValid = true;
            handset_config::RcHandsetConfig candidate = g_activeConfig.handsetConfig;
            for (uint8_t axis = 0; axis < handset_config::kRcHandsetAxisCount; ++axis) {
                handset_config::AxisCalibration calibration{};
                if (!handset_config::makeAxisCalibrationFromEndpoints(g_calMin[axis], g_calMax[axis], calibration)) {
                    calibrationValid = false;
                    break;
                }
                g_calCenter[axis] = calibration.center;
                candidate.axes[axis].calibration = calibration;
            }
            g_calibrating = false;
            if (!calibrationValid || !handset_config::validateRcHandsetConfig(candidate)) {
                rc_handset::core1::setCrsfOutputSafetyHold(false);
                Serial.print(handset_usb::formatErr(command, "invalid_calibration",
                                                    "sampled calibration failed validation").c_str());
                return true;
            }
            if (!commitRcConfig(candidate)) {
                Serial.print(handset_usb::formatErr(command, "persist_failed",
                                                    "calibration could not be saved").c_str());
                return true;
            }
            Serial.print(handset_usb::formatOk(command, configResponseFields(g_activeConfig.handsetConfig)).c_str());
            return true;
        }

        case handset_usb::CommandType::StreamState: {
            uint32_t interval = 100;
            bool enabled = true;
            if (const handset_usb::Argument* intervalArg = commandArg(command, "interval_ms")) {
                interval = static_cast<uint32_t>(strtoul(intervalArg->value.c_str(), nullptr, 10));
            }
            if (const handset_usb::Argument* enabledArg = commandArg(command, "enabled")) {
                enabled = enabledArg->value != "0" && enabledArg->value != "false";
            } else if (!command.value.empty()) {
                enabled = command.value != "0" && command.value != "off";
            }
            if (interval < 50) interval = 50;
            if (interval > 2000) interval = 2000;
            g_streamStateEnabled = enabled;
            g_streamStateIntervalMs = interval;
            g_lastStreamStateMs = 0;
            Serial.print(handset_usb::formatOk(command, {
                {"enabled", enabled ? "1" : "0"},
                {"interval_ms", std::to_string(interval)},
            }).c_str());
            return true;
        }

        case handset_usb::CommandType::State: {
            Serial.print(handset_usb::formatOk(command, stateResponseFields()).c_str());
            return true;
        }

        default:
            Serial.print(handset_usb::formatErr(command, "unsupported_command",
                                                "command is not wired in RC firmware").c_str());
            return true;
    }
}

static void printChannels() {
    for (uint8_t i = 0; i < kRcChannelCount; ++i) {
        if (i) Serial.print(',');
        Serial.print(g_liveState.channels[i]);
    }
    Serial.println();
}

static void serviceCli() {
    static char line[128];
    static uint8_t pos = 0;
    while (Serial.available()) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\r') continue;
        if (c == '\n') {
            line[pos] = '\0';
            if (handleRcV1Command(line)) {
                pos = 0;
                continue;
            }
            refreshLiveState();
            if (strcmp(line, "status") == 0) {
                Serial.printf("role=rc-rp2350 core1=%u baud=%lu frames=%lu cfg=%lu ack=%lu guard=%lu hold=%lu tx_phrase=%s tx_uid=%s ch=%u,%u,%u,%u\n",
                              rc_handset::core1::isCore1Started() ? 1u : 0u,
                              static_cast<unsigned long>(kCrsfBaud),
                              static_cast<unsigned long>(g_liveState.framesSent),
                              static_cast<unsigned long>(g_liveState.appliedConfigGeneration),
                              static_cast<unsigned long>(rc_handset::core1::ackedConfigGeneration()),
                              static_cast<unsigned long>(g_liveState.channelGuardRejects),
                              static_cast<unsigned long>(g_liveState.channelSpikeHolds),
                              g_haveTxBindingStatus ? g_txBindingStatus.phrase : "unknown",
                              g_haveTxBindingStatus ? uidCheckHexString(g_txBindingStatus.uidCheck).c_str() : "unknown",
                              g_liveState.channels[0], g_liveState.channels[1],
                              g_liveState.channels[2], g_liveState.channels[3]);
                handset_power::rcPowerPrintStatus();
            } else if (strcmp(line, "channels") == 0) {
                printChannels();
            } else if (strcmp(line, "power") == 0) {
                handset_power::rcPowerPrintStatus();
            } else if (strcmp(line, "reboot") == 0) {
                rp2040.reboot();
            } else if (line[0]) {
                Serial.println("commands: status | channels | power | reboot");
            }
            pos = 0;
        } else if (pos < sizeof(line) - 1) {
            line[pos++] = c;
        }
    }
}

#ifdef XLRS_ENABLE_WIRELESS_BIND
// Wireless "soft bind": hold PUSH1+PUSH2 together while on the Bind screen
// to ask TX to broadcast its UID -- see startWirelessBindFromRc() (defined
// after the buzzer helpers it uses) and docs/crsf/binding.md.
static bool g_wirelessBindActive = false;
static bool g_wirelessBindAccepted = false;
static bool g_wirelessBindTimedOut = false;
static uint32_t g_wirelessBindStartMs = 0;
static uint32_t g_wirelessBindLastPollMs = 0;

enum class BindHoldPhase : uint8_t { Idle, Holding, ReadyToRelease };
static BindHoldPhase g_bindHoldPhase = BindHoldPhase::Idle;
static uint32_t g_bindHoldStartMs = 0;

static void startWirelessBindFromRc();

// Mirrors PowerButtonController's hold shape (Idle -> Holding ->
// ReadyToRelease, confirm on release-after-threshold) rather than a new
// pattern -- see include/rc_handset/power/PowerButton.h.
static void serviceBindHoldGesture(uint32_t nowMs, bool bothPressed) {
    switch (g_bindHoldPhase) {
        case BindHoldPhase::Idle:
            if (bothPressed) {
                g_bindHoldPhase = BindHoldPhase::Holding;
                g_bindHoldStartMs = nowMs;
            }
            break;
        case BindHoldPhase::Holding:
            if (!bothPressed) {
                g_bindHoldPhase = BindHoldPhase::Idle;
            } else if (static_cast<uint32_t>(nowMs - g_bindHoldStartMs) >= RC_BIND_HOLD_MS) {
                g_bindHoldPhase = BindHoldPhase::ReadyToRelease;
            }
            break;
        case BindHoldPhase::ReadyToRelease:
            if (!bothPressed) {
                g_bindHoldPhase = BindHoldPhase::Idle;
                if (!g_wirelessBindActive) startWirelessBindFromRc();
            }
            break;
    }
}
#endif  // XLRS_ENABLE_WIRELESS_BIND

// PUSH1 (GPIO13) / PUSH2 (GPIO14): manual screen navigation, overriding the
// time-based auto-rotation until RC_NAV_MANUAL_TIMEOUT_MS passes with no
// press. Also the auto-rotation order itself (displayScreenForTime below).
static constexpr rc_handset::display::Screen kNavScreens[] = {
    rc_handset::display::Screen::MainStatus,
    rc_handset::display::Screen::ChannelMonitor,
    rc_handset::display::Screen::ConfigSummary,
    rc_handset::display::Screen::BindingStatus,
    rc_handset::display::Screen::BatteryStatus,
};
static constexpr uint8_t kNavScreenCount = sizeof(kNavScreens) / sizeof(kNavScreens[0]);

static rc_handset::display::Screen displayScreenForTime(uint32_t nowMs) {
    return kNavScreens[(nowMs / kDisplayPageMs) % kNavScreenCount];
}

struct NavButtonState {
    bool rawPressed = false;
    bool pressed = false;
    uint32_t lastChangeMs = 0;
};

static NavButtonState g_push1State;
static NavButtonState g_push2State;
static uint8_t g_manualScreenIndex = 0;
static bool g_manualNavActive = false;
static uint32_t g_manualNavExpiresAtMs = 0;

// Debounced falling edge -- fires once per physical press, not on release
// and not repeatedly while held.
static bool navButtonPressedEdge(NavButtonState& state, uint8_t pin, uint32_t nowMs) {
    const bool raw = digitalRead(pin) == LOW;
    if (raw != state.rawPressed) {
        state.rawPressed = raw;
        state.lastChangeMs = nowMs;
    }
    if (static_cast<uint32_t>(nowMs - state.lastChangeMs) < RC_NAV_BUTTON_DEBOUNCE_MS) return false;
    if (state.rawPressed == state.pressed) return false;
    state.pressed = state.rawPressed;
    return state.pressed;
}

static rc_handset::display::Screen currentDisplayScreen(uint32_t nowMs) {
    if (g_manualNavActive) return kNavScreens[g_manualScreenIndex];
    return displayScreenForTime(nowMs);
}

static void serviceScreenNavButtons(uint32_t nowMs) {
    // Debounce state must stay fresh even on the Bind screen, so both edge
    // detectors always run -- only their nav-index side effects are skipped
    // there, in favor of the combined-hold bind gesture.
    const bool push1Edge = navButtonPressedEdge(g_push1State, RC_PUSH1_PIN, nowMs);
    const bool push2Edge = navButtonPressedEdge(g_push2State, RC_PUSH2_PIN, nowMs);

#ifdef XLRS_ENABLE_WIRELESS_BIND
    if (currentDisplayScreen(nowMs) == rc_handset::display::Screen::BindingStatus) {
        serviceBindHoldGesture(nowMs, g_push1State.pressed && g_push2State.pressed);
        return;
    }
#endif  // XLRS_ENABLE_WIRELESS_BIND

    bool pressed = false;
    if (push1Edge) {
        g_manualScreenIndex = static_cast<uint8_t>((g_manualScreenIndex + 1) % kNavScreenCount);
        pressed = true;
    }
    if (push2Edge) {
        g_manualScreenIndex =
            static_cast<uint8_t>((g_manualScreenIndex + kNavScreenCount - 1) % kNavScreenCount);
        pressed = true;
    }
    if (pressed) {
        g_manualNavActive = true;
        g_manualNavExpiresAtMs = nowMs + RC_NAV_MANUAL_TIMEOUT_MS;
    } else if (g_manualNavActive && static_cast<int32_t>(nowMs - g_manualNavExpiresAtMs) >= 0) {
        g_manualNavActive = false;
    }
}

static void populateDisplayState(rc_handset::display::DisplayState& state,
                                 rc_handset::display::Screen screen) {
    state = rc_handset::display::DisplayState{};
    state.screen = screen;
    state.txBattery = readTxBattery();

    const uint32_t nowMs = millis();

    refreshLiveState();
    if (g_liveState.haveChannels) {
        g_handsetTelemetry.updateLocalChannels(g_liveState.channels, nowMs);
    }
    handset_telemetry::PacketCounters counters{};
    counters.uplinkTransmitAttempts = g_liveState.framesSent;
    counters.uplinkTransmitSuccesses = g_liveState.framesSent;
    counters.rejectedOtaFrames = g_liveState.channelGuardRejects;
    counters.lostPackets = g_liveState.channelSpikeHolds;
    g_handsetTelemetry.updatePacketCounters(counters, nowMs);
    g_handsetTelemetry.refresh(nowMs);
    const handset_telemetry::HandsetTelemetryState& telemetry = g_handsetTelemetry.state();

    state.rxBattery.available = telemetry.rxBatteryStatus == handset_telemetry::FieldStatus::Valid;
    state.rxBattery.millivolts = telemetry.rxBattery.millivolts;
    state.rxBattery.percent = telemetry.rxBattery.percent;
    // linkQuality > 0 is required too: TX sends a LINK_STATISTICS frame over
    // the wired link to RC on a regular cadence regardless of whether it has
    // an RF connection to RX -- with RX off, that frame still arrives fine,
    // just reporting LQ 0 -- so linkStatsStatus alone only proves "TX is
    // alive," not "RX is connected".
    state.link.connected = telemetry.txPresent &&
                           telemetry.txStatus == handset_telemetry::FieldStatus::Valid &&
                           telemetry.linkStatsStatus == handset_telemetry::FieldStatus::Valid &&
                           telemetry.linkQuality > 0;
    state.link.metricsAvailable = telemetry.linkStatsStatus == handset_telemetry::FieldStatus::Valid;
    state.link.bindingMismatch = telemetry.faults.bindingUidMismatch;
    state.link.rateName = handset_telemetry::rateName(telemetry.activeRate);
    state.link.linkQuality = telemetry.linkQuality;
    state.link.rssiDbm = telemetry.rssiDbm;
    state.link.snrDb = telemetry.snrDb;

    state.channels.channelsAvailable = g_liveState.haveChannels;
    state.channels.aileron = g_liveState.channels[0];
    state.channels.elevator = g_liveState.channels[1];
    state.channels.rudder = g_liveState.channels[2];
    state.channels.throttle = g_liveState.channels[3];
    for (uint8_t i = 0; i < rc_handset::display::kMonitorToggleCount; ++i) {
        state.channels.toggles[i] = g_liveState.channels[i + 4];
    }

    state.config.rateName = handset_telemetry::rateName(telemetry.activeRate);
    state.config.bindingPhrase = displayBindingPhrase();
    state.config.calibrationValid = g_liveState.haveChannels;
    state.config.framesSent = g_liveState.framesSent;
    state.config.guardRejects = g_liveState.channelGuardRejects;
    state.config.spikeHolds = g_liveState.channelSpikeHolds;

    state.binding.mismatch = telemetry.faults.bindingUidMismatch;
    if (telemetry.faults.configFault) {
        state.binding.message = "Config fault";
    } else if (telemetry.faults.bindingUidMismatch) {
        state.binding.message = "Receiver UID differs";
    } else if (telemetry.faults.bindingUidMissing) {
        state.binding.message = "TX UID unchecked";
    } else if (telemetry.linkStatsStatus == handset_telemetry::FieldStatus::Valid) {
        state.binding.message = "Telemetry OK";
    } else {
        state.binding.message = "No receiver telemetry";
    }

#ifdef XLRS_ENABLE_WIRELESS_BIND
    static char bindMsgBuf[32];
    state.binding.bindMode = g_wirelessBindActive;
    if (g_wirelessBindAccepted) {
        state.binding.message = "Binding successful";
    } else if (g_wirelessBindActive) {
        const uint32_t elapsedMs = millis() - g_wirelessBindStartMs;
        const uint32_t remainingS = elapsedMs >= RC_BIND_WINDOW_MS
                                        ? 0u
                                        : (RC_BIND_WINDOW_MS - elapsedMs) / 1000u;
        snprintf(bindMsgBuf, sizeof(bindMsgBuf), "Binding... %lus left", static_cast<unsigned long>(remainingS));
        state.binding.message = bindMsgBuf;
    } else if (g_wirelessBindTimedOut) {
        state.binding.message = "No response, try again";
    }
    if (screen == rc_handset::display::Screen::BindingStatus) {
        if (g_bindHoldPhase == BindHoldPhase::ReadyToRelease) {
            state.binding.hint = "Release to bind!";
        } else if (g_bindHoldPhase == BindHoldPhase::Holding) {
            state.binding.hint = "Keep holding...";
        } else if (!g_wirelessBindActive) {
            state.binding.hint = "Hold both to bind";
        }
    }
#endif  // XLRS_ENABLE_WIRELESS_BIND
}

static void serviceDisplay() {
    const uint32_t nowMs = millis();
    if (static_cast<uint32_t>(nowMs - g_lastDisplayMs) < kDisplayIntervalMs) return;
    g_lastDisplayMs = nowMs;

    rc_handset::display::DisplayState state;
    populateDisplayState(state, currentDisplayScreen(nowMs));
    g_display.show(state);
}

static void printStateStreamLine() {
    Serial.print("rc.v1 state");
    for (const handset_usb::ResponseField& field : stateResponseFields()) {
        Serial.print(' ');
        Serial.print(field.key.c_str());
        Serial.print('=');
        Serial.print(handset_usb::percentEncode(field.value).c_str());
    }
    Serial.print('\n');
}

static void serviceStateStream() {
    if (!g_streamStateEnabled || g_streamStateIntervalMs == 0) return;
    const uint32_t nowMs = millis();
    if (g_lastStreamStateMs != 0 &&
        static_cast<uint32_t>(nowMs - g_lastStreamStateMs) < g_streamStateIntervalMs) {
        return;
    }
    g_lastStreamStateMs = nowMs;
    printStateStreamLine();
}

// Blocking power-on/power-off confirmation beep (RC_BUZZER_PIN, see
// rc_power_config.h). Blocking is required at both call sites: power-off in
// particular must not return with the buzzer still sounding, since nothing
// services it once POWMAN sleep is entered right after.
static void buzzerBeepBlocking(uint16_t durationMs) {
    digitalWrite(RC_BUZZER_PIN, HIGH);
    delay(durationMs);
    digitalWrite(RC_BUZZER_PIN, LOW);
}

// Non-blocking buzzer, serviced from loop() (see buzzerServiceNonBlocking).
// Used for the RX-link connected/disconnected beeps, which must not stall
// CRSF/USB/power-button servicing the way a delay()-based beep would.
static uint32_t g_buzzerOffAtMs = 0;
static bool g_buzzerOn = false;

static void buzzerStartNonBlocking(uint16_t durationMs) {
    digitalWrite(RC_BUZZER_PIN, HIGH);
    g_buzzerOn = true;
    g_buzzerOffAtMs = millis() + durationMs;
}

static void buzzerServiceNonBlocking() {
    if (g_buzzerOn && static_cast<int32_t>(millis() - g_buzzerOffAtMs) >= 0) {
        digitalWrite(RC_BUZZER_PIN, LOW);
        g_buzzerOn = false;
    }
}

// Connected: one beep, then silence. Disconnected: a short chirp every
// RC_BUZZER_DISCONNECTED_INTERVAL_MS. The first disconnected chirp is
// deferred by one interval rather than firing immediately, so it doesn't
// collide with the power-on beep or fire the instant a link drops.
static bool g_buzzerWasRxLinkActive = false;
static bool g_buzzerDisconnectedChirpArmed = false;
static uint32_t g_buzzerNextDisconnectedChirpMs = 0;

static void serviceConnectionBuzzer(bool rxLinkActive, uint32_t nowMs) {
    if (rxLinkActive) {
        if (!g_buzzerWasRxLinkActive) {
            buzzerStartNonBlocking(RC_BUZZER_CONNECTED_BEEP_MS);
        }
        g_buzzerDisconnectedChirpArmed = false;
    } else if (!g_buzzerDisconnectedChirpArmed) {
        g_buzzerNextDisconnectedChirpMs = nowMs + RC_BUZZER_DISCONNECTED_INTERVAL_MS;
        g_buzzerDisconnectedChirpArmed = true;
    } else if (static_cast<int32_t>(nowMs - g_buzzerNextDisconnectedChirpMs) >= 0) {
        if (!g_buzzerOn) buzzerStartNonBlocking(RC_BUZZER_DISCONNECTED_CHIRP_MS);
        g_buzzerNextDisconnectedChirpMs = nowMs + RC_BUZZER_DISCONNECTED_INTERVAL_MS;
    }
    g_buzzerWasRxLinkActive = rxLinkActive;
}

#ifdef XLRS_ENABLE_WIRELESS_BIND
static void startWirelessBindFromRc() {
    BindingControlRequest request{};
    request.op = BindingControlOp::WirelessBindStart;
    BindingStatus status{};
    BindingResult result = BindingResult::InvalidCommand;
    BindingControlOp responseOp = BindingControlOp::WirelessBindStart;
    const bool ok = proxyTxBindingRequest(request, status, result, responseOp);

    g_wirelessBindStartMs = millis();
    g_wirelessBindLastPollMs = g_wirelessBindStartMs;
    g_wirelessBindAccepted = false;
    g_wirelessBindActive = ok && result == BindingResult::Ok && status.wirelessBindActive;
    g_wirelessBindTimedOut = !g_wirelessBindActive;
    buzzerStartNonBlocking(g_wirelessBindActive ? RC_BUZZER_BIND_START_MS : RC_BUZZER_BIND_FAIL_MS);
}

// Polls TX for wireless-bind progress via the existing Get path (TX folds
// wirelessBindActive/wirelessBindAccepted into every binding response while
// its broadcast is running -- see serviceTxCrsfInput in src/main.cpp) --
// no separate poll op needed.
static void serviceWirelessBindPoll() {
    if (!g_wirelessBindActive) return;
    const uint32_t nowMs = millis();
    if (static_cast<uint32_t>(nowMs - g_wirelessBindLastPollMs) < RC_BIND_POLL_INTERVAL_MS) return;
    g_wirelessBindLastPollMs = nowMs;

    BindingControlRequest request{};
    request.op = BindingControlOp::Get;
    BindingStatus status{};
    BindingResult result = BindingResult::InvalidCommand;
    BindingControlOp responseOp = BindingControlOp::Get;
    if (proxyTxBindingRequest(request, status, result, responseOp) && result == BindingResult::Ok) {
        if (status.wirelessBindAccepted) {
            g_wirelessBindActive = false;
            g_wirelessBindAccepted = true;
            buzzerStartNonBlocking(RC_BUZZER_BIND_SUCCESS_MS);
            return;
        }
        g_wirelessBindActive = status.wirelessBindActive;
    }
    if (g_wirelessBindActive && static_cast<uint32_t>(nowMs - g_wirelessBindStartMs) >= RC_BIND_WINDOW_MS) {
        g_wirelessBindActive = false;
        g_wirelessBindTimedOut = true;
        buzzerStartNonBlocking(RC_BUZZER_BIND_FAIL_MS);
    }
}
#endif  // XLRS_ENABLE_WIRELESS_BIND

static void showBootScreen(uint8_t progressPercent, const char* message) {
    rc_handset::display::DisplayState state;
    state.screen = rc_handset::display::Screen::Boot;
    state.bootProgressPercent = progressPercent;
    state.bootMessage = message;
    g_display.show(state);
}

// Fires on every poll of the hold-to-boot gate (see rcPowerBegin), so the
// bar animates live instead of sitting static for the whole hold.
static void showBootHoldProgress(uint8_t percent, bool readyToRelease) {
    if (readyToRelease) {
        showBootScreen(100, "Release to power on");
        return;
    }
    showBootScreen(percent, "Hold power button");
}

// Fires on every poll of the hold-to-power-off gate (see rcPowerService).
static void showPowerOffProgress(uint8_t percent, bool readyToRelease, bool blocked) {
    if (blocked) {
        showBootScreen(percent, "RX connected: cannot power off");
        return;
    }
    if (readyToRelease) {
        showBootScreen(100, "Release to power off");
        return;
    }
    showBootScreen(percent, "Hold to power off");
}

#ifdef XLRS_ENABLE_TX_SLEEP
// EXPERIMENTAL (rc-rp2350-pwrmgmt / tx_lora_pico_pwrmgmt only): tells TX to
// put its radio in standby to stop draining the handset's battery while RC
// is powered off. Fire-and-forget -- no response is expected or waited for,
// RC is about to sleep itself right after this. If Serial2 isn't up yet
// (e.g. an aborted boot hold, before CRSF init) this silently does nothing,
// which is fine: TX just stays awake, the safe direction to fail in. See
// BindingControlOp::Sleep in protocol.h for why there's no matching wake
// call here -- TX wakes itself once real CRSF resumes. Not yet proven
// through repeated real power cycles on hardware -- see AGENTS.md.
static void sendTxSleepCommand() {
    BindingControlRequest request{};
    request.op = BindingControlOp::Sleep;
    uint8_t frame[64];
    const size_t len = encodeCrsfBindingRequestFrame(request, frame, sizeof(frame));
    if (len) {
        Serial2.write(frame, len);
        Serial2.flush();
    }
}
#endif  // XLRS_ENABLE_TX_SLEEP

void setup() {
#ifdef RC_ENABLE_HID_JOYSTICK
    // Must run before Serial.begin() -- the HID interface has to be
    // registered before TinyUSB enumerates, or the composite descriptor
    // race loses to the host and enumeration breaks.
    handset_usb::hidJoystickBegin();
#endif
    Serial.begin(115200);
#ifndef USE_TINYUSB
    Serial.ignoreFlowControl(true);
#endif
    pinMode(RC_BUZZER_PIN, OUTPUT);
    digitalWrite(RC_BUZZER_PIN, LOW);
    pinMode(RC_PUSH1_PIN, INPUT_PULLUP);
    pinMode(RC_PUSH2_PIN, INPUT_PULLUP);
    g_display.begin();
    handset_power::rcPowerSetSleepHook([]() {
#ifdef XLRS_ENABLE_TX_SLEEP
        sendTxSleepCommand();
#endif
        buzzerBeepBlocking(RC_BUZZER_POWER_BEEP_MS);
        g_display.sleep();
    });
    handset_power::rcPowerBegin(showBootHoldProgress);
    buzzerBeepBlocking(RC_BUZZER_POWER_BEEP_MS);
    showBootScreen(25, "Core 0 init");
    loadRcConfig();
    publishActiveConfig();
    showBootScreen(50, "Core 1 launch");
    rc_handset::core1::begin();
    refreshLiveState();
    showBootScreen(80, "CRSF ready");
    Serial.println("RC RP2350 CRSF output");
    Serial.printf("Core1=%u baud=%lu interval_us=%lu\n",
                  rc_handset::core1::isCore1Started() ? 1u : 0u,
                  static_cast<unsigned long>(kCrsfBaud),
                  static_cast<unsigned long>(rc_handset::core1::kDefaultCrsfFrameIntervalUs));
    showBootScreen(100, "Ready");
}

void loop() {
    // Refreshed unconditionally (not just when a USB client polls state) so
    // linkStatsStatus staleness -- and therefore the power-off block below --
    // stays accurate even with no USB host attached.
    g_handsetTelemetry.refresh(millis());
    // TX sends link-stats telemetry continuously whether or not it has an RX,
    // reporting linkQuality=0 (not silence) when disconnected -- so freshness
    // alone ("a frame arrived recently") is true any time a TX is attached at
    // all. The actual RX-connected signal is a *nonzero* quality value.
    const handset_telemetry::HandsetTelemetryState& telemetryState = g_handsetTelemetry.state();
    const bool rxLinkActive = telemetryState.linkStatsStatus == handset_telemetry::FieldStatus::Valid &&
                              telemetryState.linkQuality > 0;
#if RC_BUZZER_CONNECTION_ENABLED
    serviceConnectionBuzzer(rxLinkActive, millis());
#endif
    buzzerServiceNonBlocking();
    serviceScreenNavButtons(millis());
#ifdef XLRS_ENABLE_WIRELESS_BIND
    serviceWirelessBindPoll();
#endif
#ifdef RC_ENABLE_HID_JOYSTICK
    refreshLiveState();
    handset_usb::hidJoystickService(g_liveState.channels, millis());
#endif
    handset_power::rcPowerService(showPowerOffProgress, rxLinkActive);
    rc_handset::core1::loopOnceFallback();
    serviceCrsfTelemetry();
    serviceCli();
    serviceTxBindingProbe();
    serviceStateStream();
    // The power-off countdown owns the display for the duration of the hold;
    // don't fight it with the normal screen-rotation refresh.
    if (!handset_power::rcPowerOffHoldActive()) {
        serviceDisplay();
    }
    yield();
}

#endif // UNIT_TEST
