// XLRS transmitter firmware, Pico SDK entry point.
#include <atomic>
#include <stdio.h>
#include <string.h>

#include <hardware/uart.h>
#include <pico/flash.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <hardware/watchdog.h>

#include "UARTProtocol.h"
#include "app/AppTelemetry.h"
#include "app/CrsfLinkStats.h"
#include "app/LinkStatusLed.h"
#include "crsfSerial.h"
#include "fhss/Fhss.h"
#include "hal/FlashStore.h"
#include "hal/SerialPort.h"
#include "hal/Time.h"
#include "link/BindingStore.h"
#include "link/Link.h"
#include "link/RfConfig.h"
#include "link/RfScheduler.h"
#include "link/Uid.h"
#include "phy/Sx1280NativePhy.h"
#include "util/Mailbox.h"
#include "util/RingBuffer.h"

#ifndef XLRS_TX_CONTROLLER_CRSF
#define XLRS_TX_CONTROLLER_CRSF 0
#endif

#ifndef UART_PROTOCOL_TX
#define UART_PROTOCOL_TX 8
#endif
#ifndef UART_PROTOCOL_RX
#define UART_PROTOCOL_RX 9
#endif

#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN 10
#endif

struct AppToRfData {
    uint16_t channels[8];
};

struct RfToAppData {
    xlrs::LinkState state;
    xlrs::LinkStats stats;
    bool hardwareError;   // RF core: radio init failed or PHY unhealthy
    bool bindTransmitActive;
    uint8_t bindSecondsRemaining;
    uint32_t downlinkQueueDrops;
};

xlrs::LatestValue<AppToRfData> g_appToRf;
xlrs::LatestValue<RfToAppData> g_rfToApp;
xlrs::SpscRing<xlrs::AppTelemetryMessage, 4> g_appTelemetryToRf;
xlrs::SpscRing<xlrs::AppTelemetryMessage, 4> g_rfTelemetryToApp;

// Core 1 (RF) bumps this every loop; core 0 watches it to gate the hardware watchdog. A stall
// in EITHER core stops the heartbeat-gated feed and lets the watchdog reboot the module.
static std::atomic<uint32_t> g_rfHeartbeat{0};
static constexpr uint32_t WATCHDOG_TIMEOUT_MS    = 1000; // HW reboots if not fed within this
static constexpr uint32_t RF_STALL_MS            = 500;  // stop feeding if RF core stalls this long
static constexpr uint32_t WATCHDOG_BOOT_GRACE_MS = 3000; // feed unconditionally until RF core is up
static constexpr uint32_t BIND_TRANSMIT_WINDOW_MS = 30000;

xlrs::Link g_link;
xlrs::RfScheduler g_scheduler;
xlrs::Sx1280NativePhy g_phy;
xlrs::BindingStore g_bindingStore;
xlrs::RfConfigData g_rfConfig;
xlrs::hal::SerialPort g_controlUart(uart1, UART_PROTOCOL_TX, UART_PROTOCOL_RX);
#if XLRS_TX_CONTROLLER_CRSF
CrsfSerial controllerCrsf(g_controlUart, CRSF_BAUDRATE);
struct CrsfControllerDebug {
    uint32_t rcFrames;
    uint32_t devicePings;
    uint32_t parameterReads;
    uint32_t parameterWrites;
    uint32_t telemetryFramesToController;
    uint32_t invalidDownlinkMessages;
    uint32_t lastRcMs;
    uint32_t lastDevicePingMs;
    uint32_t lastParameterReadMs;
    uint32_t lastParameterWriteMs;
};
static CrsfControllerDebug g_crsfDebug{};
#else
UARTProtocol uartProto(&g_controlUart);
#endif

static uint32_t lastTelemetrySent = 0;
static constexpr uint32_t TELEMETRY_INTERVAL = 200;
static uint32_t lastStatusSent = 0;
static constexpr uint32_t STATUS_INTERVAL = 1000;

#if XLRS_TX_CONTROLLER_CRSF
static constexpr uint8_t CRSF_PARAM_COUNT = 8; // Includes root folder at parameter 0.
static constexpr uint8_t CRSF_PARAM_ROOT = 0;
static constexpr uint8_t CRSF_PARAM_RATE = 1;
static constexpr uint8_t CRSF_PARAM_MAX_POWER = 2;
static constexpr uint8_t CRSF_PARAM_DYNAMIC_POWER = 3;
static constexpr uint8_t CRSF_PARAM_REGION = 4;
static constexpr uint8_t CRSF_PARAM_FAILSAFE = 5;
static constexpr uint8_t CRSF_PARAM_BIND = 6;
static constexpr uint8_t CRSF_PARAM_REBOOT = 7;

static bool crsfDestinationIsTxModule(uint8_t destination) {
    return destination == CRSF_ADDRESS_BROADCAST || destination == CRSF_ADDRESS_CRSF_TRANSMITTER;
}

static void appendByte(uint8_t* payload, uint8_t& len, uint8_t value) {
    if (len < CRSF_MAX_PAYLOAD_LEN) payload[len++] = value;
}

static void appendString(uint8_t* payload, uint8_t& len, const char* value) {
    while (*value && len < CRSF_MAX_PAYLOAD_LEN) payload[len++] = (uint8_t)*value++;
    appendByte(payload, len, 0);
}

static void appendBe32(uint8_t* payload, uint8_t& len, uint32_t value) {
    appendByte(payload, len, (uint8_t)(value >> 24));
    appendByte(payload, len, (uint8_t)(value >> 16));
    appendByte(payload, len, (uint8_t)(value >> 8));
    appendByte(payload, len, (uint8_t)value);
}

static void sendCrsfParameterEntry(uint8_t destination, const uint8_t* payload, uint8_t len) {
    controllerCrsf.queueExtendedPacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY,
                                       destination, CRSF_ADDRESS_CRSF_TRANSMITTER, payload, len);
}

static void sendCrsfParameterWriteAck(uint8_t destination, uint8_t parameterNumber,
                                      const uint8_t* value, uint8_t valueLen) {
    uint8_t payload[CRSF_MAX_PAYLOAD_LEN] = {};
    uint8_t len = 0;
    appendByte(payload, len, parameterNumber);
    for (uint8_t i = 0; i < valueLen && len < CRSF_MAX_PAYLOAD_LEN; ++i) {
        appendByte(payload, len, value[i]);
    }
    controllerCrsf.queueExtendedPacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_PARAMETER_WRITE,
                                       destination, CRSF_ADDRESS_CRSF_TRANSMITTER, payload, len);
}

static bool enqueueAppTelemetryToRf(const xlrs::AppTelemetryMessage& message, const char* label) {
    if (g_appTelemetryToRf.push(message)) return true;

    printf("[CRSF] ERROR: RF command queue full, dropped %s (drops=%lu)\n",
           label, (unsigned long)g_appTelemetryToRf.dropped());
    return false;
}

static bool saveRfConfigFromCrsf(const char* label) {
    xlrs::RfConfig::refreshChecksum(g_rfConfig);
    bool saved = xlrs::RfConfig::save(g_rfConfig);
    xlrs::AppTelemetryMessage message{};
    bool queued = xlrs::makeRxConfigMessage(g_rfConfig, message) &&
                  enqueueAppTelemetryToRf(message, label);
    if (!saved) {
        printf("[CRSF] ERROR: Failed to save RF config for %s\n", label);
    }
    return saved && queued;
}

static void buildRootParameter(uint8_t* payload, uint8_t& len) {
    appendByte(payload, len, CRSF_PARAM_ROOT);
    appendByte(payload, len, 0); // chunks remaining
    appendByte(payload, len, 0); // parent
    appendByte(payload, len, CRSF_PARAM_FOLDER);
    appendString(payload, len, "ROOT");
    appendByte(payload, len, CRSF_PARAM_RATE);
    appendByte(payload, len, CRSF_PARAM_MAX_POWER);
    appendByte(payload, len, CRSF_PARAM_DYNAMIC_POWER);
    appendByte(payload, len, CRSF_PARAM_REGION);
    appendByte(payload, len, CRSF_PARAM_FAILSAFE);
    appendByte(payload, len, CRSF_PARAM_BIND);
    appendByte(payload, len, CRSF_PARAM_REBOOT);
    appendByte(payload, len, 0xFF);
}

static void buildSelectionParameter(uint8_t* payload, uint8_t& len, uint8_t parameterNumber,
                                    const char* label, uint8_t value, uint8_t maxValue,
                                    const char* options) {
    appendByte(payload, len, parameterNumber);
    appendByte(payload, len, 0);
    appendByte(payload, len, CRSF_PARAM_ROOT);
    appendByte(payload, len, CRSF_PARAM_TEXT_SELECTION);
    appendString(payload, len, label);
    appendString(payload, len, options);
    appendByte(payload, len, value);
    appendByte(payload, len, 0);
    appendByte(payload, len, maxValue);
    appendByte(payload, len, value);
    appendString(payload, len, "");
}

static void buildInt8Parameter(uint8_t* payload, uint8_t& len, uint8_t parameterNumber,
                               const char* label, int8_t value, int8_t minValue,
                               int8_t maxValue, const char* units) {
    appendByte(payload, len, parameterNumber);
    appendByte(payload, len, 0);
    appendByte(payload, len, CRSF_PARAM_ROOT);
    appendByte(payload, len, CRSF_PARAM_INT8);
    appendString(payload, len, label);
    appendByte(payload, len, (uint8_t)value);
    appendByte(payload, len, (uint8_t)minValue);
    appendByte(payload, len, (uint8_t)maxValue);
    appendString(payload, len, units);
}

static void buildCommandParameter(uint8_t* payload, uint8_t& len, uint8_t parameterNumber,
                                  const char* label, const char* info,
                                  uint8_t commandStatus = CRSF_COMMAND_READY) {
    appendByte(payload, len, parameterNumber);
    appendByte(payload, len, 0);
    appendByte(payload, len, CRSF_PARAM_ROOT);
    appendByte(payload, len, CRSF_PARAM_COMMAND);
    appendString(payload, len, label);
    appendByte(payload, len, commandStatus);
    appendByte(payload, len, 10); // poll/display timeout in 100 ms units
    appendString(payload, len, info);
}

static void buildOutOfRangeParameter(uint8_t* payload, uint8_t& len, uint8_t parameterNumber) {
    appendByte(payload, len, parameterNumber);
    appendByte(payload, len, 0);
    appendByte(payload, len, 0);
    appendByte(payload, len, CRSF_PARAM_OUT_OF_RANGE);
    appendString(payload, len, "Out of range");
}

static void sendCrsfParameter(uint8_t destination, uint8_t parameterNumber) {
    uint8_t payload[CRSF_MAX_PAYLOAD_LEN] = {};
    uint8_t len = 0;
    switch (parameterNumber) {
        case CRSF_PARAM_ROOT:
            buildRootParameter(payload, len);
            break;
        case CRSF_PARAM_RATE:
            buildSelectionParameter(payload, len, parameterNumber, "Rate", g_rfConfig.defaultRate,
                                    xlrs::kNumRates - 1, "F1000;F500;D250;L150;L50");
            break;
        case CRSF_PARAM_MAX_POWER:
            buildInt8Parameter(payload, len, parameterNumber, "Max Power", g_rfConfig.maxPowerDbm,
                               -18, g_rfConfig.region == (uint8_t)xlrs::RfRegion::EU ? 10 : 13,
                               "dBm");
            break;
        case CRSF_PARAM_DYNAMIC_POWER:
            buildSelectionParameter(payload, len, parameterNumber, "Dynamic Power",
                                    g_rfConfig.dynamicPower, 1, "Off;On");
            break;
        case CRSF_PARAM_REGION:
            buildSelectionParameter(payload, len, parameterNumber, "Region", g_rfConfig.region,
                                    1, "US;EU");
            break;
        case CRSF_PARAM_FAILSAFE:
            buildSelectionParameter(payload, len, parameterNumber, "Failsafe",
                                    g_rfConfig.failsafeMode, 1, "No Pulses;Hold");
            break;
        case CRSF_PARAM_REBOOT:
            buildCommandParameter(payload, len, parameterNumber, "Reboot", "Apply saved config");
            break;
        case CRSF_PARAM_BIND:
            {
                RfToAppData rfData{};
                const bool bindActive = g_rfToApp.load(rfData) && rfData.bindTransmitActive;
                buildCommandParameter(payload, len, parameterNumber, "Bind RX", "Send TX identity",
                                      bindActive ? CRSF_COMMAND_PROGRESS : CRSF_COMMAND_READY);
            }
            break;
        default:
            buildOutOfRangeParameter(payload, len, parameterNumber);
            break;
    }
    sendCrsfParameterEntry(destination, payload, len);
}

void onCrsfDevicePing(uint8_t destination, uint8_t origin) {
    if (!crsfDestinationIsTxModule(destination)) return;
    g_crsfDebug.devicePings++;
    g_crsfDebug.lastDevicePingMs = xlrs::hal::nowMs();
    printf("[CRSF CTRL] DEVICE_PING origin=0x%02X dest=0x%02X count=%lu\n",
           origin, destination, (unsigned long)g_crsfDebug.devicePings);

    uint8_t payload[CRSF_MAX_PAYLOAD_LEN] = {};
    uint8_t len = 0;
    appendString(payload, len, "XLRS TX");
    appendBe32(payload, len, 0x584C5253UL); // "XLRS"
    appendBe32(payload, len, 1);            // hardware id
    appendBe32(payload, len, 1);            // firmware id
    appendByte(payload, len, CRSF_PARAM_COUNT);
    appendByte(payload, len, 1);            // parameter protocol version

    controllerCrsf.queueExtendedPacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_DEVICE_INFO,
                                       origin, CRSF_ADDRESS_CRSF_TRANSMITTER, payload, len);
}

void onCrsfParameterRead(uint8_t parameterNumber, uint8_t chunkNumber,
                         uint8_t destination, uint8_t origin) {
    if (!crsfDestinationIsTxModule(destination) || chunkNumber != 0) return;
    g_crsfDebug.parameterReads++;
    g_crsfDebug.lastParameterReadMs = xlrs::hal::nowMs();
    printf("[CRSF CTRL] PARAMETER_READ id=%u origin=0x%02X count=%lu\n",
           (unsigned)parameterNumber, origin, (unsigned long)g_crsfDebug.parameterReads);
    sendCrsfParameter(origin, parameterNumber);
}

void onCrsfParameterWrite(uint8_t parameterNumber, const uint8_t* value, uint8_t valueLen,
                          uint8_t destination, uint8_t origin) {
    if (!crsfDestinationIsTxModule(destination) || !value) return;
    g_crsfDebug.parameterWrites++;
    g_crsfDebug.lastParameterWriteMs = xlrs::hal::nowMs();
    printf("[CRSF CTRL] PARAMETER_WRITE id=%u len=%u origin=0x%02X count=%lu\n",
           (unsigned)parameterNumber, (unsigned)valueLen, origin,
           (unsigned long)g_crsfDebug.parameterWrites);

    switch (parameterNumber) {
        case CRSF_PARAM_RATE:
            if (valueLen >= 1 && value[0] < xlrs::kNumRates) {
                g_rfConfig.defaultRate = value[0];
                if (saveRfConfigFromCrsf("rate")) {
                    sendCrsfParameterWriteAck(origin, parameterNumber, &g_rfConfig.defaultRate, 1);
                } else {
                    sendCrsfParameter(origin, parameterNumber);
                }
            }
            break;
        case CRSF_PARAM_MAX_POWER:
            if (valueLen >= 1) {
                int8_t requested = (int8_t)value[0];
                if (requested >= -18 && requested <= 13) {
                    g_rfConfig.maxPowerDbm = requested;
                    if (saveRfConfigFromCrsf("max power")) {
                        sendCrsfParameterWriteAck(origin, parameterNumber,
                                                  (const uint8_t*)&g_rfConfig.maxPowerDbm, 1);
                    } else {
                        sendCrsfParameter(origin, parameterNumber);
                    }
                }
            }
            break;
        case CRSF_PARAM_DYNAMIC_POWER:
            if (valueLen >= 1 && value[0] <= 1) {
                g_rfConfig.dynamicPower = value[0];
                if (saveRfConfigFromCrsf("dynamic power")) {
                    sendCrsfParameterWriteAck(origin, parameterNumber, &g_rfConfig.dynamicPower, 1);
                } else {
                    sendCrsfParameter(origin, parameterNumber);
                }
            }
            break;
        case CRSF_PARAM_REGION:
            if (valueLen >= 1 && value[0] <= 1) {
                g_rfConfig.region = value[0];
                if (saveRfConfigFromCrsf("region")) {
                    sendCrsfParameterWriteAck(origin, parameterNumber, &g_rfConfig.region, 1);
                } else {
                    sendCrsfParameter(origin, parameterNumber);
                }
            }
            break;
        case CRSF_PARAM_FAILSAFE:
            if (valueLen >= 1 && value[0] <= 1) {
                g_rfConfig.failsafeMode = value[0];
                if (saveRfConfigFromCrsf("failsafe")) {
                    sendCrsfParameterWriteAck(origin, parameterNumber, &g_rfConfig.failsafeMode, 1);
                } else {
                    sendCrsfParameter(origin, parameterNumber);
                }
            }
            break;
        case CRSF_PARAM_REBOOT:
            if (valueLen >= 1 && (value[0] == CRSF_COMMAND_START || value[0] == CRSF_COMMAND_CONFIRM)) {
                xlrs::AppTelemetryMessage message{};
                if (xlrs::makeRebootMessage(message) &&
                    enqueueAppTelemetryToRf(message, "reboot")) {
                    sendCrsfParameter(origin, parameterNumber);
                    xlrs::hal::sleepMs(1000);
                    watchdog_reboot(0, 0, 0);
                } else {
                    sendCrsfParameter(origin, parameterNumber);
                }
            }
            break;
        case CRSF_PARAM_BIND:
            if (valueLen >= 1 && value[0] == CRSF_COMMAND_START) {
                uint8_t uid[xlrs::LINK_UID_SIZE] = {};
                xlrs::AppTelemetryMessage message{};
                if (g_bindingStore.getBindingUid(uid) &&
                    xlrs::makeStartBindMessage(uid, message) &&
                    enqueueAppTelemetryToRf(message, "bind")) {
                    sendCrsfParameter(origin, parameterNumber);
                } else {
                    printf("[CRSF] ERROR: Failed to start bind command\n");
                    sendCrsfParameter(origin, parameterNumber);
                }
            }
            break;
        default:
            sendCrsfParameter(origin, parameterNumber);
            break;
    }
}

void onCrsfChannelsReceived() {
    AppToRfData payload;
    for (int i = 0; i < 8; i++) {
        payload.channels[i] = (uint16_t)controllerCrsf.getChannel((unsigned int)i + 1);
    }
    g_appToRf.store(payload);
    g_crsfDebug.rcFrames++;
    g_crsfDebug.lastRcMs = xlrs::hal::nowMs();
}
#else
void onChannelsReceived(const ChannelData* data) {
    if (!data) return;
    AppToRfData payload;
    for (int i = 0; i < 8; i++) {
        payload.channels[i] = data->channels[i];
    }
    g_appToRf.store(payload);
}

void onCommandReceived(UARTMsgType cmd) {
    switch (cmd) {
        case UART_MSG_CMD_PAIR:
            printf("[UART] PAIR command received - XLRS link runs on bind phrase\n");
            uartProto.sendAck(UART_MSG_CMD_PAIR);
            break;
        case UART_MSG_CMD_BOND:
            printf("[UART] BOND command received\n");
            uartProto.sendAck(UART_MSG_CMD_BOND);
            break;
        case UART_MSG_CMD_RESTART:
            printf("[UART] RESTART command received\n");
            uartProto.sendAck(UART_MSG_CMD_RESTART);
            xlrs::hal::sleepMs(100);
            watchdog_reboot(0, 0, 0);
            break;
        case UART_MSG_CMD_STATUS_REQ:
            uartProto.sendAck(UART_MSG_CMD_STATUS_REQ);
            break;
        default:
            break;
    }
}

void onCommandPayloadReceived(UARTMsgType cmd, const uint8_t* payload, uint8_t length) {
    if (cmd != UART_MSG_CMD_SET_BIND_TX || !payload || length == 0 || length > 32) return;

    char phrase[33];
    memcpy(phrase, payload, length);
    phrase[length] = '\0';
    printf("[UART] SET_BIND_TX command received: %s\n", phrase);
    uartProto.sendAck(UART_MSG_CMD_SET_BIND_TX);
    xlrs::hal::sleepMs(10);

    if (g_bindingStore.setBindingPhrase(phrase)) {
        printf("[UART] Binding phrase persisted. Rebooting TX...\n");
        xlrs::hal::sleepMs(100);
        watchdog_reboot(0, 0, 0);
    } else {
        printf("[UART] ERROR: Failed to persist binding phrase\n");
    }
}
#endif

// Print boot identity diagnostics ON CORE 0 (stdio is core-0 only — see rf_core_main). Lets the
// bench operator confirm both ends derived the same UID/sync word before expecting a link.
static void printIdentity(const uint8_t uid[8]) {
    printf("[UID] Computed identity: 0x%02X%02X%02X%02X%02X%02X%02X%02X\n",
           uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7]);
    const uint8_t r = (g_rfConfig.defaultRate < xlrs::kNumRates) ? g_rfConfig.defaultRate : 0;
    printf("[PHY] Mode: %s, Sync Word: 0x%04X\n",
           xlrs::kRates[r].modulation == xlrs::Modulation::Lora ? "LoRa" : "FLRC",
           (unsigned)xlrs::syncWordFromUid(uid));
}

static void drainPhyDiagLogs() {
    xlrs::PhyDiagEvent event{};
    while (g_phy.popDiagEvent(event)) {
        printf("[PHY DIAG] t=%luus phase=%s status=%s op=0x%02X timeouts=%lu drops=%lu\n",
               (unsigned long)event.timestampUs,
               xlrs::Sx1280NativePhy::diagPhaseName(event.phase),
               xlrs::Sx1280NativePhy::diagStatusName(event.status),
               (unsigned)event.opcode,
               (unsigned long)event.spiTimeouts,
               (unsigned long)g_phy.diagDrops());
    }
}

// Feed the hardware watchdog only while the RF core's heartbeat keeps advancing. A bounded boot
// grace lets core 1 finish radio init before liveness is enforced; after that, a hung RF core
// (including the init-failure spin below) stops the feed and the watchdog reboots to retry.
static void serviceWatchdog() {
    static uint32_t lastBeat = 0, lastBeatMs = 0;
    static bool seenBeat = false;
    const uint32_t now = xlrs::hal::nowMs();
    const uint32_t beat = g_rfHeartbeat.load(std::memory_order_acquire);
    if (beat != lastBeat) { lastBeat = beat; lastBeatMs = now; seenBeat = true; }
    const bool inGrace = now < WATCHDOG_BOOT_GRACE_MS;
    const bool rfAlive = seenBeat && (now - lastBeatMs) < RF_STALL_MS;
    if (inGrace || rfAlive) watchdog_update();
}

static void updateLED(xlrs::LinkState state, bool hardwareError, bool bindTransmitActive) {
    xlrs::app::LinkStatusLedFlags flags{};
    flags.hardwareError = hardwareError;
    flags.bindTransmitActive = bindTransmitActive;
    xlrs::app::linkStatusLedUpdate(STATUS_LED_PIN, state, flags);
}

static void setup_app_core() {
    printf("=== XLRS Pico SDK Transmitter ===\n");

    xlrs::hal::FlashStore::begin();
    if (g_bindingStore.begin()) {
        printf("Binding identity store initialized.\n");
    } else {
        printf("ERROR: Binding identity store initialization failed.\n");
    }

    if (xlrs::RfConfig::load(g_rfConfig)) {
        printf("RF configuration loaded from flash.\n");
    } else {
        printf("No valid RF configuration found. Storing defaults...\n");
        xlrs::RfConfig::setDefaults(g_rfConfig);
        xlrs::RfConfig::save(g_rfConfig);
    }

    printf("RF Region: %s\n", g_rfConfig.region == 0 ? "US" : "EU");
    printf("Max Power Cap: %d dBm\n", (int)g_rfConfig.maxPowerDbm);

    uint8_t uid[8];
    if (g_bindingStore.getBindingUid(uid)) {
        printIdentity(uid);
    }

#if XLRS_TX_CONTROLLER_CRSF
    controllerCrsf.begin(CRSF_BAUDRATE);
    controllerCrsf.onPacketChannels = onCrsfChannelsReceived;
    controllerCrsf.onDevicePing = onCrsfDevicePing;
    controllerCrsf.onParameterRead = onCrsfParameterRead;
    controllerCrsf.onParameterWrite = onCrsfParameterWrite;
    printf("Controller CRSF initialized.\n");
#else
    uartProto.begin(UART_PROTOCOL_BAUDRATE);
    uartProto.setOnChannels(onChannelsReceived);
    uartProto.setOnCommand(onCommandReceived);
    uartProto.setOnCommandPayload(onCommandPayloadReceived);
    printf("Controller UART initialized.\n");
#endif

    printf("Status LED on GP%d (active-%s).\n", (int)STATUS_LED_PIN,
           XLRS_STATUS_LED_ACTIVE_LOW ? "low" : "high");
}

static void app_core_loop() {
#if XLRS_TX_CONTROLLER_CRSF
    controllerCrsf.loop();
    xlrs::AppTelemetryMessage downlinkMessage{};
    while (g_rfTelemetryToApp.pop(downlinkMessage)) {
        const uint8_t* crsfFrame = nullptr;
        uint8_t crsfFrameLen = 0;
        if (xlrs::parseCrsfFrameMessage(downlinkMessage.data, downlinkMessage.len,
                                        crsfFrame, crsfFrameLen)) {
            controllerCrsf.write(crsfFrame, crsfFrameLen);
            g_crsfDebug.telemetryFramesToController++;
        } else {
            g_crsfDebug.invalidDownlinkMessages++;
        }
    }
#else
    uartProto.loop();
#endif
    drainPhyDiagLogs();

    static RfToAppData rfData{};
    static bool haveRfData = false;
    RfToAppData latest{};
    if (g_rfToApp.load(latest)) {
        rfData = latest;
        haveRfData = true;
    }

    updateLED(haveRfData ? rfData.state : xlrs::LinkState::Disconnected,
              haveRfData && rfData.hardwareError,
              haveRfData && rfData.bindTransmitActive);

    if (haveRfData) {
        uint32_t now = xlrs::hal::nowMs();
        static bool lastBindTransmitActive = false;
        if (rfData.bindTransmitActive != lastBindTransmitActive) {
            lastBindTransmitActive = rfData.bindTransmitActive;
            if (rfData.bindTransmitActive) {
                printf("[TX BIND] OTA bind transmit window open for %u seconds.\n",
                       (unsigned)rfData.bindSecondsRemaining);
            } else {
                printf("[TX BIND] OTA bind transmit window closed.\n");
            }
        }

        if (now - lastTelemetrySent >= TELEMETRY_INTERVAL) {
            lastTelemetrySent = now;

#if XLRS_TX_CONTROLLER_CRSF
            uint8_t crsfLinkStatistics[10] = {};
            xlrs::buildCrsfLinkStatistics(rfData.stats, crsfLinkStatistics);
            controllerCrsf.queuePacket(CRSF_ADDRESS_RADIO_TRANSMITTER,
                                       CRSF_FRAMETYPE_LINK_STATISTICS,
                                       crsfLinkStatistics,
                                       sizeof(crsfLinkStatistics));
#else
            TelemetryData telem{};
            telem.rssi = rfData.stats.rssiDbm;
            telem.snr = rfData.stats.snr;
            telem.linkQuality = rfData.stats.lqDown;
            uartProto.sendTelemetry(&telem);
#endif
        }

        if (now - lastStatusSent >= STATUS_INTERVAL) {
            lastStatusSent = now;

#if !XLRS_TX_CONTROLLER_CRSF
            StatusData status{};
            switch (rfData.state) {
                case xlrs::LinkState::Disconnected: status.connectionState = 0; break;
                case xlrs::LinkState::Binding:      status.connectionState = 1; break;
                case xlrs::LinkState::Connecting:   status.connectionState = 2; break;
                case xlrs::LinkState::Connected:    status.connectionState = 3; break;
                case xlrs::LinkState::Failsafe:     status.connectionState = 4; break;
                default:                            status.connectionState = 0; break;
            }
            status.pairingState = rfData.bindTransmitActive ? 2 : 1;
            status.packetsLost = rfData.stats.missedDeadlines;
            uartProto.sendStatus(&status);
#endif

            // Core-0 diagnostic: surface PHY fault counters so a wedged radio is visible on the
            // bench console, not just an opaque healthy()=false (docs/troubleshooting/index.md §3).
            printf("[TX STATUS] State: %d LQdown: %u%% RSSI: %d dBm | PHY timeouts: %lu CRC: %lu Phase: %s/%s LastOp: 0x%02X LastOk: 0x%02X LastFailOp: 0x%02X",
                   (int)rfData.state, (unsigned)rfData.stats.lqDown, (int)rfData.stats.rssiDbm,
                   (unsigned long)g_phy.spiTimeouts(), (unsigned long)g_phy.crcErrors(),
                   xlrs::Sx1280NativePhy::diagPhaseName(g_phy.lastDiagPhase()),
                   xlrs::Sx1280NativePhy::diagStatusName(g_phy.lastDiagStatus()),
                   (unsigned)g_phy.lastStartedOpcode(),
                   (unsigned)g_phy.lastCompletedOpcode(),
                   (unsigned)g_phy.lastFailOpcode());
#if XLRS_TX_CONTROLLER_CRSF
            const uint32_t rcAge = g_crsfDebug.lastRcMs == 0 ? 0 : now - g_crsfDebug.lastRcMs;
            printf(" | CRSF rc:%lu age:%lums ping:%lu pr:%lu pw:%lu fc:%lu bad:%lu qdrop:%lu dldrop:%lu",
                   (unsigned long)g_crsfDebug.rcFrames,
                   (unsigned long)rcAge,
                   (unsigned long)g_crsfDebug.devicePings,
                   (unsigned long)g_crsfDebug.parameterReads,
                   (unsigned long)g_crsfDebug.parameterWrites,
                   (unsigned long)g_crsfDebug.telemetryFramesToController,
                   (unsigned long)g_crsfDebug.invalidDownlinkMessages,
                   (unsigned long)g_appTelemetryToRf.dropped(),
                   (unsigned long)rfData.downlinkQueueDrops);
#endif
            if (rfData.hardwareError) {
                printf(" [HW FAULT]");
            }
            if (rfData.bindTransmitActive) {
                printf(" [BIND TX %us]", (unsigned)rfData.bindSecondsRemaining);
            }
            printf("\n");
        }
    }
}

// RF core (core 1) does NOT touch stdio — printf over USB CDC is blocking and not multicore-safe,
// and this core runs the µs-critical schedule. Fatal init faults are published to core 0 via the
// mailbox (logged there); the spin afterwards stops the heartbeat so the watchdog reboots.
static void publishRfFault() {
    RfToAppData d{};
    d.hardwareError = true;
    g_rfToApp.store(d);
}

static void rf_core_main() {
    if (flash_safe_execute_core_init()) {
        xlrs::hal::FlashStore::setMulticoreSafetyEnabled(true);
    }

    xlrs::hal::FlashStore::begin();
    g_bindingStore.begin();
    if (!xlrs::RfConfig::load(g_rfConfig)) {
        xlrs::RfConfig::setDefaults(g_rfConfig);
    }

    uint8_t uid[8];
    if (!g_bindingStore.getBindingUid(uid)) {
        g_bindingStore.setBindingPhrase(DEFAULT_BINDING_PHRASE);
        g_bindingStore.getBindingUid(uid);
    }

    g_link.begin(xlrs::Role::Tx, uid, g_rfConfig.defaultRate, g_rfConfig.maxPowerDbm, g_rfConfig.dynamicPower == 1);
    g_link.setRegion(g_rfConfig.region == (uint8_t)xlrs::RfRegion::EU ? xlrs::FhssRegion::EU_CE : xlrs::FhssRegion::US_FCC);

    xlrs::PhyConfig phyCfg = xlrs::makePhyConfig(xlrs::kRates[g_rfConfig.defaultRate], 2400.0f, g_rfConfig.maxPowerDbm, xlrs::syncWordFromUid(uid));
    if (!g_phy.init(phyCfg) ||
        !g_scheduler.begin(&g_phy, &g_link, g_rfConfig.defaultRate)) {
        publishRfFault();
        while (true) xlrs::hal::sleepMs(100);  // heartbeat stops → watchdog reboots to retry
    }

    while (true) {
        static uint32_t bindTransmitUntilMs = 0;
        static bool telemetryPending = false;
        static xlrs::AppTelemetryMessage pendingTelemetryMessage{};
        xlrs::AppTelemetryMessage telemetryMessage{};
        uint8_t bindUid[xlrs::LINK_UID_SIZE] = {};
        // pendingTelemetryMessage only ever holds a message that failed to queue, and StartBind is
        // consumed directly below (never queued/stashed), so a pending message is never a StartBind.
        if (telemetryPending) {
            telemetryPending = !g_link.queueTelemetry(pendingTelemetryMessage.data,
                                                      pendingTelemetryMessage.len);
        }
        if (!telemetryPending && g_appTelemetryToRf.pop(telemetryMessage)) {
            if (xlrs::parseStartBindMessage(telemetryMessage.data, telemetryMessage.len, bindUid)) {
                g_link.startBindTransmit(bindUid);
                bindTransmitUntilMs = xlrs::hal::nowMs() + BIND_TRANSMIT_WINDOW_MS;
            } else {
                telemetryPending = !g_link.queueTelemetry(telemetryMessage.data, telemetryMessage.len);
                if (telemetryPending) {
                    pendingTelemetryMessage = telemetryMessage;
                }
            }
        }

        if (g_link.bindTransmitActive() && bindTransmitUntilMs != 0 &&
            (int32_t)(xlrs::hal::nowMs() - bindTransmitUntilMs) >= 0) {
            g_link.setLinkUid(uid);
            bindTransmitUntilMs = 0;
        }

        AppToRfData appData;
        if (g_appToRf.load(appData)) {
            g_link.setChannels(appData.channels, xlrs::Link::RC_CHANNELS);
        }

        g_scheduler.poll();

        static uint32_t lastServiceTick = 0;
        uint32_t currentTick = g_scheduler.processedTick();
        if (currentTick > lastServiceTick) {
            lastServiceTick = currentTick;
            RfToAppData rfData;
            rfData.state = g_link.state();
            rfData.stats = g_link.stats();
            rfData.hardwareError = !g_phy.healthy();
            rfData.bindTransmitActive = g_link.bindTransmitActive();
            rfData.downlinkQueueDrops = g_rfTelemetryToApp.dropped();
            if (rfData.bindTransmitActive && bindTransmitUntilMs != 0) {
                const int32_t remainingMs = (int32_t)(bindTransmitUntilMs - xlrs::hal::nowMs());
                rfData.bindSecondsRemaining = remainingMs > 0
                    ? (uint8_t)((remainingMs + 999) / 1000)
                    : 0;
            } else {
                rfData.bindSecondsRemaining = 0;
            }
            g_rfToApp.store(rfData);

            xlrs::pumpLinkTelemetryToApp(g_link, g_rfTelemetryToApp);
        }

        g_rfHeartbeat.fetch_add(1, std::memory_order_release);
        tight_loop_contents();
    }
}

int main() {
    stdio_init_all();
    flash_safe_execute_core_init();
    xlrs::app::linkStatusLedInit(STATUS_LED_PIN);

    xlrs::hal::sleepMs(1000);
    setup_app_core();
    multicore_launch_core1(rf_core_main);

    // Arm the hardware watchdog AFTER boot setup; serviceWatchdog() then feeds it only while the
    // RF core is alive, so a hang in either core reboots the module instead of bricking the link.
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);

    while (true) {
        app_core_loop();
        serviceWatchdog();
        tight_loop_contents();
    }
}
