// XLRS receiver firmware, Pico SDK entry point.
#include <math.h>
#include <stdio.h>

#include <hardware/gpio.h>
#include <hardware/uart.h>
#include <pico/flash.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>

#include "app/CrsfLinkStats.h"
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

#ifndef CRSF_TX_PIN
#define CRSF_TX_PIN 8
#endif
#ifndef CRSF_RX_PIN
#define CRSF_RX_PIN 9
#endif

#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN 13
#endif

struct RfToAppData {
    xlrs::LinkState state;
    xlrs::LinkStats stats;
    uint16_t channels[8];
    bool outputActive;
    bool hardwareError;
};

xlrs::LatestValue<RfToAppData> g_rfToApp;

xlrs::Link g_link;
xlrs::RfScheduler g_scheduler;
xlrs::Sx1280NativePhy g_phy;
xlrs::Fhss g_fhss;
xlrs::BindingStore g_bindingStore;
xlrs::RfConfigData g_rfConfig;
xlrs::hal::SerialPort g_crsfUart(uart1, CRSF_TX_PIN, CRSF_RX_PIN);
CrsfSerial crsf(g_crsfUart, CRSF_BAUDRATE);

static uint32_t lastLedUpdate = 0;
static constexpr uint32_t LED_UPDATE_INTERVAL_MS = 50;
static uint32_t lastLinkStats = 0;
static constexpr uint32_t LINK_STATS_INTERVAL_MS = 500;
static uint16_t localChannels[8] = {1500, 1500, 1500, 1000, 1500, 1500, 1500, 1500};
static bool g_configFault = false;

static long mapRange(long value, long inMin, long inMax, long outMin, long outMax) {
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

static void updateLED(xlrs::LinkState state, bool outputActive, bool hardwareError) {
    if (xlrs::hal::nowMs() - lastLedUpdate < LED_UPDATE_INTERVAL_MS) return;
    lastLedUpdate = xlrs::hal::nowMs();

    static uint32_t msCounter = 0;
    msCounter += LED_UPDATE_INTERVAL_MS;

    bool on = false;
    if (g_configFault || hardwareError) {
        on = true;
    } else if (state == xlrs::LinkState::Binding) {
        on = (msCounter % 100) < 50;
    } else if (state == xlrs::LinkState::Connected && outputActive) {
        on = true;
    } else if (state == xlrs::LinkState::Connecting) {
        on = (msCounter % 500) < 250;
    } else {
        on = (msCounter % 1000) < 500;
    }
    gpio_put(STATUS_LED_PIN, on);
}

static void setup_app_core() {
    printf("=== XLRS Pico SDK Receiver ===\n");

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
        g_configFault = true;
        xlrs::RfConfig::setDefaults(g_rfConfig);
        xlrs::RfConfig::save(g_rfConfig);
    }

    printf("RF Region: %s\n", g_rfConfig.region == 0 ? "US" : "EU");
    printf("Failsafe Mode: %s\n", g_rfConfig.failsafeMode == 0 ? "NoPulses" : "Hold");

    crsf.begin(CRSF_BAUDRATE);

    gpio_init(STATUS_LED_PIN);
    gpio_set_dir(STATUS_LED_PIN, GPIO_OUT);
    gpio_put(STATUS_LED_PIN, false);

    printf("CRSF UART and status LED initialized.\n");
}

static void app_core_loop() {
    static uint32_t lastSeq = 0;
    static uint32_t lastCRSF = 0;
    RfToAppData rfData{};

    bool isNew = g_rfToApp.loadIfNew(rfData, lastSeq);
    if (!isNew) {
        g_rfToApp.load(rfData);
    }

    xlrs::LinkState state = rfData.state;
    bool outputActive = rfData.outputActive;
    uint32_t now = xlrs::hal::nowMs();
    bool shouldOutput = isNew || (now - lastCRSF >= 20);

    if (shouldOutput) {
        lastCRSF = now;

        if (state == xlrs::LinkState::Connected) {
            for (int i = 0; i < 8; i++) localChannels[i] = rfData.channels[i];
        } else if (state == xlrs::LinkState::Failsafe) {
            localChannels[0] = 1500;
            localChannels[1] = 1500;
            localChannels[2] = 1500;
            localChannels[3] = 1000;
            for (int i = 4; i < 8; i++) localChannels[i] = 1500;
        }

        if (outputActive) {
            crsf_channels_t crsfChannels{};
            crsfChannels.ch0 = mapRange(localChannels[0], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
            crsfChannels.ch1 = mapRange(localChannels[1], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
            crsfChannels.ch2 = mapRange(localChannels[2], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
            crsfChannels.ch3 = mapRange(localChannels[3], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
            crsfChannels.ch4 = mapRange(localChannels[4], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
            crsfChannels.ch5 = mapRange(localChannels[5], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
            crsfChannels.ch6 = mapRange(localChannels[6], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
            crsfChannels.ch7 = mapRange(localChannels[7], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);

            uint16_t center = CRSF_CHANNEL_VALUE_MID;
            crsfChannels.ch8 = center;
            crsfChannels.ch9 = center;
            crsfChannels.ch10 = center;
            crsfChannels.ch11 = center;
            crsfChannels.ch12 = center;
            crsfChannels.ch13 = center;
            crsfChannels.ch14 = center;
            crsfChannels.ch15 = center;

            crsf.queuePacket(CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_RC_CHANNELS_PACKED,
                             &crsfChannels, CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE);
        }
    }

    if (now - lastLinkStats >= LINK_STATS_INTERVAL_MS) {
        lastLinkStats = now;
        uint8_t stats[10] = {};
        xlrs::buildCrsfLinkStatistics(rfData.stats, stats);
        crsf.queuePacket(CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_LINK_STATISTICS,
                         stats, sizeof(stats));
        printf("[RX STATUS] State: %d LQ: %u%% RSSI: %d dBm\n",
               (int)state, (unsigned)rfData.stats.lqUp, (int)rfData.stats.rssiDbm);
    }

    updateLED(state, outputActive, rfData.hardwareError);
    crsf.loop();
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
    if (g_bindingStore.getBindingUid(uid)) {
        printf("[Core 1] Binding UID loaded from flash.\n");
    } else {
        printf("[Core 1] No binding UID found. Creating from default phrase...\n");
        g_bindingStore.setBindingPhrase(DEFAULT_BINDING_PHRASE);
        g_bindingStore.getBindingUid(uid);
    }

    g_link.begin(xlrs::Role::Rx, uid, g_rfConfig.defaultRate, g_rfConfig.maxPowerDbm, g_rfConfig.dynamicPower == 1);
    g_link.setRegion(g_rfConfig.region == (uint8_t)xlrs::RfRegion::EU ? xlrs::FhssRegion::EU_CE : xlrs::FhssRegion::US_FCC);
    g_link.setFailsafeMode(g_rfConfig.failsafeMode == 0 ? xlrs::FailsafeMode::NoPulses : xlrs::FailsafeMode::Hold);

    xlrs::PhyConfig phyCfg = xlrs::makePhyConfig(xlrs::kRates[g_rfConfig.defaultRate], 2400.0f, g_rfConfig.maxPowerDbm, xlrs::syncWordFromUid(uid));
    if (!g_phy.init(phyCfg)) {
        printf("[ERROR] Radio PHY initialization failed.\n");
        RfToAppData rfData{};
        rfData.hardwareError = true;
        g_rfToApp.store(rfData);
        while (true) xlrs::hal::sleepMs(100);
    }
    if (!g_scheduler.begin(&g_phy, &g_fhss, &g_link, g_rfConfig.defaultRate)) {
        printf("[ERROR] Scheduler initialization failed.\n");
        RfToAppData rfData{};
        rfData.hardwareError = true;
        g_rfToApp.store(rfData);
        while (true) xlrs::hal::sleepMs(100);
    }

    while (true) {
        g_scheduler.poll();

        static uint32_t lastServiceTick = 0;
        uint32_t currentTick = g_scheduler.processedTick();
        if (currentTick > lastServiceTick) {
            lastServiceTick = currentTick;

            RfToAppData rfData;
            rfData.state = g_link.state();
            rfData.stats = g_link.stats();
            g_link.getChannels(rfData.channels, 8);
            rfData.outputActive = g_link.outputActive();
            rfData.hardwareError = !g_phy.healthy();
            g_rfToApp.store(rfData);
        }
        tight_loop_contents();
    }
}

int main() {
    stdio_init_all();
    flash_safe_execute_core_init();
    xlrs::hal::sleepMs(1000);
    setup_app_core();
    multicore_launch_core1(rf_core_main);

    while (true) {
        app_core_loop();
        tight_loop_contents();
    }
}
