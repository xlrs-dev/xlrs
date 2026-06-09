#ifndef UNIT_TEST

#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include <string>

#include "lora_link/protocol.h"
#include "rc_handset/core1/Runtime.h"
#include "rc_handset/display/OledDisplay.h"
#include "rc_handset/power/Rp2350Power.h"
#include "rc_handset/telemetry/HandsetTelemetry.h"
#include "rc_handset/usb/protocol.h"

using namespace lora_link;
using rc_handset::core1::LiveStateSnapshot;
namespace handset_power = rc_handset::power;
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
static LiveStateSnapshot g_liveState = rc_handset::core1::makeDefaultLiveStateSnapshot();
static rc_handset::display::OledDisplay g_display;
static handset_telemetry::HandsetTelemetry g_handsetTelemetry(
    {100u, 1500u, kLinkTelemetryFreshMs, 5000u});

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
    return rfMode == static_cast<uint8_t>(RateId::L250) ? RateId::L250 : RateId::L100;
}

static uint32_t defaultBindingUidCheck() {
    uint8_t uid[kUidSize];
    deriveUid(DEFAULT_BINDING_PHRASE, uid);
    return uidCheck(uid);
}

static void publishTxBindingTelemetry(bool present, const BindingStatus* status, uint32_t nowMs) {
    const handset_telemetry::HandsetTelemetryState& state = g_handsetTelemetry.state();
    g_handsetTelemetry.updateTxStatus(present,
                                      true,
                                      defaultBindingUidCheck(),
                                      status != nullptr,
                                      status ? status->uidCheck : state.observedUidCheck,
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
                                          true,
                                          defaultBindingUidCheck(),
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
        yield();
    }
    return false;
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
                {"caps", "state,config,binding,tx_proxy"},
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

        case handset_usb::CommandType::State: {
            refreshLiveState();
            if (g_liveState.haveChannels) {
                g_handsetTelemetry.updateLocalChannels(g_liveState.channels, millis());
            }
            handset_telemetry::PacketCounters counters{};
            counters.uplinkTransmitAttempts = g_liveState.framesSent;
            counters.uplinkTransmitSuccesses = g_liveState.framesSent;
            counters.rejectedOtaFrames = g_liveState.channelGuardRejects;
            counters.lostPackets = g_liveState.channelSpikeHolds;
            g_handsetTelemetry.updatePacketCounters(counters, millis());
            g_handsetTelemetry.refresh(millis());

            char telemetryState[320];
            handset_telemetry::formatUsbStateResponse(g_handsetTelemetry.state(),
                                                       telemetryState,
                                                       sizeof(telemetryState));
            Serial.print(handset_usb::formatOk(command, {{"state", telemetryState}}).c_str());
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
                Serial.printf("role=rc-rp2350 core1=%u baud=%lu frames=%lu cfg=%lu ack=%lu guard=%lu hold=%lu ch=%u,%u,%u,%u\n",
                              rc_handset::core1::isCore1Started() ? 1u : 0u,
                              static_cast<unsigned long>(kCrsfBaud),
                              static_cast<unsigned long>(g_liveState.framesSent),
                              static_cast<unsigned long>(g_liveState.appliedConfigGeneration),
                              static_cast<unsigned long>(rc_handset::core1::ackedConfigGeneration()),
                              static_cast<unsigned long>(g_liveState.channelGuardRejects),
                              static_cast<unsigned long>(g_liveState.channelSpikeHolds),
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

static rc_handset::display::Screen displayScreenForTime(uint32_t nowMs) {
    switch ((nowMs / kDisplayPageMs) % 4u) {
        case 0:
            return rc_handset::display::Screen::MainStatus;
        case 1:
            return rc_handset::display::Screen::ChannelMonitor;
        case 2:
            return rc_handset::display::Screen::ConfigSummary;
        default:
            return rc_handset::display::Screen::BindingStatus;
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
    state.config.bindingPhrase = DEFAULT_BINDING_PHRASE;
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
}

static void serviceDisplay() {
    const uint32_t nowMs = millis();
    if (static_cast<uint32_t>(nowMs - g_lastDisplayMs) < kDisplayIntervalMs) return;
    g_lastDisplayMs = nowMs;

    rc_handset::display::DisplayState state;
    populateDisplayState(state, displayScreenForTime(nowMs));
    g_display.show(state);
}

static void showBootScreen(uint8_t progressPercent, const char* message) {
    rc_handset::display::DisplayState state;
    state.screen = rc_handset::display::Screen::Boot;
    state.bootProgressPercent = progressPercent;
    state.bootMessage = message;
    g_display.show(state);
}

void setup() {
    Serial.begin(115200);
    Serial.ignoreFlowControl(true);
    g_display.begin();
    showBootScreen(15, "Power init");
    handset_power::rcPowerBegin();
    showBootScreen(25, "Core 0 init");
    rc_handset::core1::publishDefaultConfig();
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
    handset_power::rcPowerService();
    rc_handset::core1::loopOnceFallback();
    serviceCrsfTelemetry();
    serviceCli();
    serviceDisplay();
    yield();
}

#endif // UNIT_TEST
