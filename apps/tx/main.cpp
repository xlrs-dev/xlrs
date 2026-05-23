// XLRS transmitter firmware, Pico SDK entry point.
#include <stdio.h>
#include <string.h>

#include <hardware/uart.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <hardware/watchdog.h>

#include "UARTProtocol.h"
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

#ifndef UART_PROTOCOL_TX
#define UART_PROTOCOL_TX 8
#endif
#ifndef UART_PROTOCOL_RX
#define UART_PROTOCOL_RX 9
#endif

struct AppToRfData {
    uint16_t channels[8];
};

struct RfToAppData {
    xlrs::LinkState state;
    xlrs::LinkStats stats;
};

xlrs::LatestValue<AppToRfData> g_appToRf;
xlrs::LatestValue<RfToAppData> g_rfToApp;

xlrs::Link g_link;
xlrs::RfScheduler g_scheduler;
xlrs::Sx1280NativePhy g_phy;
xlrs::Fhss g_fhss;
xlrs::BindingStore g_bindingStore;
xlrs::RfConfigData g_rfConfig;
xlrs::hal::SerialPort g_controlUart(uart1, UART_PROTOCOL_TX, UART_PROTOCOL_RX);
UARTProtocol uartProto(&g_controlUart);

static uint32_t lastTelemetrySent = 0;
static constexpr uint32_t TELEMETRY_INTERVAL = 200;
static uint32_t lastStatusSent = 0;
static constexpr uint32_t STATUS_INTERVAL = 1000;

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

    uartProto.begin(UART_PROTOCOL_BAUDRATE);
    uartProto.setOnChannels(onChannelsReceived);
    uartProto.setOnCommand(onCommandReceived);
    uartProto.setOnCommandPayload(onCommandPayloadReceived);

    printf("Controller UART initialized.\n");
}

static void app_core_loop() {
    uartProto.loop();

    RfToAppData rfData;
    if (g_rfToApp.load(rfData)) {
        uint32_t now = xlrs::hal::nowMs();
        if (now - lastTelemetrySent >= TELEMETRY_INTERVAL) {
            lastTelemetrySent = now;

            TelemetryData telem{};
            telem.rssi = rfData.stats.rssiDbm;
            telem.snr = rfData.stats.snr;
            telem.linkQuality = rfData.stats.lqDown;
            uartProto.sendTelemetry(&telem);
        }

        if (now - lastStatusSent >= STATUS_INTERVAL) {
            lastStatusSent = now;

            StatusData status{};
            switch (rfData.state) {
                case xlrs::LinkState::Disconnected: status.connectionState = 0; break;
                case xlrs::LinkState::Binding:      status.connectionState = 1; break;
                case xlrs::LinkState::Connecting:   status.connectionState = 2; break;
                case xlrs::LinkState::Connected:    status.connectionState = 3; break;
                case xlrs::LinkState::Failsafe:     status.connectionState = 4; break;
                default:                            status.connectionState = 0; break;
            }
            status.pairingState = 1;
            status.packetsLost = rfData.stats.missedDeadlines;
            uartProto.sendStatus(&status);
        }
    }
}

static void rf_core_main() {
    xlrs::hal::FlashStore::begin();
    g_bindingStore.begin();
    if (!xlrs::RfConfig::load(g_rfConfig)) {
        xlrs::RfConfig::setDefaults(g_rfConfig);
    }

    uint8_t uid[8];
    if (g_bindingStore.getBindingUid(uid)) {
        printf("[Core 1] Binding UID loaded from flash.\n");
    } else {
        printf("[Core 1] No binding UID found. Creating from default phrase...\n");
        g_bindingStore.setBindingPhrase(DEFAULT_BINDING_PHRASE);
        g_bindingStore.getBindingUid(uid);
    }

    g_link.begin(xlrs::Role::Tx, uid, g_rfConfig.defaultRate, g_rfConfig.maxPowerDbm, g_rfConfig.dynamicPower == 1);
    g_link.setRegion(g_rfConfig.region == (uint8_t)xlrs::RfRegion::EU ? xlrs::FhssRegion::EU_CE : xlrs::FhssRegion::US_FCC);

    xlrs::PhyConfig phyCfg = xlrs::makePhyConfig(xlrs::kRates[g_rfConfig.defaultRate], 2400.0f, g_rfConfig.maxPowerDbm, xlrs::syncWordFromUid(uid));
    if (!g_phy.init(phyCfg)) {
        printf("[ERROR] Radio PHY initialization failed.\n");
        while (true) xlrs::hal::sleepMs(100);
    }
    if (!g_scheduler.begin(&g_phy, &g_fhss, &g_link, g_rfConfig.defaultRate)) {
        printf("[ERROR] Scheduler initialization failed.\n");
        while (true) xlrs::hal::sleepMs(100);
    }

    while (true) {
        AppToRfData appData;
        if (g_appToRf.load(appData)) {
            g_link.setChannels(appData.channels, 8);
        }

        g_scheduler.poll();

        static uint32_t lastServiceTick = 0;
        uint32_t currentTick = g_scheduler.processedTick();
        if (currentTick > lastServiceTick) {
            lastServiceTick = currentTick;
            RfToAppData rfData;
            rfData.state = g_link.state();
            rfData.stats = g_link.stats();
            g_rfToApp.store(rfData);
        }
        tight_loop_contents();
    }
}

int main() {
    stdio_init_all();
    xlrs::hal::sleepMs(1000);
    setup_app_core();
    multicore_launch_core1(rf_core_main);

    while (true) {
        app_core_loop();
        tight_loop_contents();
    }
}
