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
    bool hardwareError;   // RF core: radio init failed or PHY unhealthy
};

xlrs::LatestValue<AppToRfData> g_appToRf;
xlrs::LatestValue<RfToAppData> g_rfToApp;

// Core 1 (RF) bumps this every loop; core 0 watches it to gate the hardware watchdog. A stall
// in EITHER core stops the heartbeat-gated feed and lets the watchdog reboot the module.
static std::atomic<uint32_t> g_rfHeartbeat{0};
static constexpr uint32_t WATCHDOG_TIMEOUT_MS    = 1000; // HW reboots if not fed within this
static constexpr uint32_t RF_STALL_MS            = 500;  // stop feeding if RF core stalls this long
static constexpr uint32_t WATCHDOG_BOOT_GRACE_MS = 3000; // feed unconditionally until RF core is up

xlrs::Link g_link;
xlrs::RfScheduler g_scheduler;
xlrs::Sx1280NativePhy g_phy;
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

            // Core-0 diagnostic: surface PHY fault counters so a wedged radio is visible on the
            // bench console, not just an opaque healthy()=false (debugging.md §3).
            printf("[TX STATUS] State: %d LQdown: %u%% RSSI: %d dBm | PHY timeouts: %lu CRC: %lu%s\n",
                   (int)rfData.state, (unsigned)rfData.stats.lqDown, (int)rfData.stats.rssiDbm,
                   (unsigned long)g_phy.spiTimeouts(), (unsigned long)g_phy.crcErrors(),
                   rfData.hardwareError ? " [HW FAULT]" : "");
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
            g_rfToApp.store(rfData);
        }

        g_rfHeartbeat.fetch_add(1, std::memory_order_release);
        tight_loop_contents();
    }
}

int main() {
    stdio_init_all();
    flash_safe_execute_core_init();
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
