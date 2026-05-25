// XLRS receiver firmware, Pico SDK entry point.
#include <atomic>
#include <math.h>
#include <stdio.h>

#include <hardware/gpio.h>
#include <hardware/uart.h>
#include <hardware/watchdog.h>
#include <pico/flash.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>

#include "app/AppTelemetry.h"
#include "app/CrsfChannels.h"
#include "app/CrsfLinkStats.h"
#include "app/LinkStatusLed.h"
#include "app/LinkRuntimeDiag.h"
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

#ifndef CRSF_TX_PIN
#define CRSF_TX_PIN 8
#endif
#ifndef CRSF_RX_PIN
#define CRSF_RX_PIN 9
#endif

#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN 10
#endif

// RC plumbing constants (no bare magic numbers in the control path).
static constexpr uint8_t  RC_CHANNELS           = xlrs::Link::RC_CHANNELS; // channels carried OTA
static constexpr uint16_t RC_US_MIN             = 1000;   // RC pulse range (µs)
static constexpr uint16_t RC_US_MID             = 1500;
static constexpr uint16_t RC_US_MAX             = 2000;
static constexpr uint8_t  RC_THROTTLE_CH        = 3;      // throttle channel index
static constexpr uint16_t RC_THROTTLE_FAILSAFE  = RC_US_MIN; // throttle low on failsafe

struct RfToAppData {
    xlrs::LinkState state;
    xlrs::LinkStats stats;
    uint16_t channels[RC_CHANNELS];
    bool outputActive;
    bool hardwareError;
    bool bindScanOpen;
    bool bindPacketReceived;
    xlrs::app::LinkRuntimeDiag linkDiag;
    uint32_t rfToAppQueueDrops;
};

xlrs::LatestValue<RfToAppData> g_rfToApp;
xlrs::SpscRing<xlrs::AppTelemetryMessage, 4> g_appTelemetryToRf;
xlrs::SpscRing<xlrs::AppTelemetryMessage, 4> g_rfTelemetryToApp;

// Core 1 (RF) bumps this every loop; core 0 watches it to gate the hardware watchdog. A stall in
// EITHER core stops the heartbeat-gated feed and lets the watchdog reboot the module.
static std::atomic<uint32_t> g_rfHeartbeat{0};
static constexpr uint32_t WATCHDOG_TIMEOUT_MS    = 1000; // HW reboots if not fed within this
static constexpr uint32_t RF_STALL_MS            = 500;  // stop feeding if RF core stalls this long
static constexpr uint32_t WATCHDOG_BOOT_GRACE_MS = 3000; // feed unconditionally until RF core is up
static constexpr uint32_t BIND_SCAN_NORMAL_WINDOW_MS = 4000;
static constexpr uint32_t BIND_SCAN_WINDOW_MS = 1500;
// Defer bind-scan cycling so a bound pair can acquire before the RX retunes to bind mode.
static constexpr uint32_t BIND_SCAN_GRACE_MS = 30000;

xlrs::Link g_link;
xlrs::RfScheduler g_scheduler;
xlrs::Sx1280NativePhy g_phy;
xlrs::BindingStore g_bindingStore;
xlrs::RfConfigData g_rfConfig;
xlrs::hal::SerialPort g_crsfUart(uart1, CRSF_TX_PIN, CRSF_RX_PIN);
CrsfSerial crsf(g_crsfUart, CRSF_BAUDRATE);

static uint32_t lastLinkStats = 0;
static constexpr uint32_t LINK_STATS_INTERVAL_MS = 500;
static uint16_t localChannels[RC_CHANNELS] = {
    RC_US_MID, RC_US_MID, RC_US_MID, RC_THROTTLE_FAILSAFE,
    RC_US_MID, RC_US_MID, RC_US_MID, RC_US_MID};
static bool g_configFault = false;
static uint32_t g_crsfRcFramesOut = 0;
static uint32_t g_crsfLinkStatsOut = 0;
static uint32_t g_fcCrsfFramesSeen = 0;
static uint32_t g_fcCrsfFramesQueued = 0;
static uint32_t g_fcCrsfFramesDropped = 0;
static uint32_t g_lastFcCrsfFrameMs = 0;
static uint32_t g_lastCrsfRcOutMs = 0;

static void blinkStatusLedBlocking(uint8_t pulses, uint32_t onMs, uint32_t offMs) {
    xlrs::app::linkStatusLedBlinkBlocking(STATUS_LED_PIN, pulses, onMs, offMs);
}

static void onCrsfFrameFromFlightController(const uint8_t *frame, uint8_t frameLen) {
    if (!frame || frameLen < 3) return;
    const uint8_t frameType = frame[2];
    if (frameType == CRSF_FRAMETYPE_RC_CHANNELS_PACKED ||
        frameType == CRSF_FRAMETYPE_LINK_STATISTICS) {
        return;
    }

    g_fcCrsfFramesSeen++;
    g_lastFcCrsfFrameMs = xlrs::hal::nowMs();
    xlrs::AppTelemetryMessage message{};
    if (xlrs::makeCrsfFrameMessage(frame, frameLen, message) &&
        g_appTelemetryToRf.push(message)) {
        g_fcCrsfFramesQueued++;
    } else {
        g_fcCrsfFramesDropped++;
    }
}

static void applyRxAppTelemetry(const xlrs::AppTelemetryMessage& message) {
    xlrs::RfConfigData pendingConfig = g_rfConfig;
    if (xlrs::parseRxConfigMessage(message.data, message.len, pendingConfig)) {
        g_rfConfig = pendingConfig;
        xlrs::RfConfig::save(g_rfConfig);
        printf("[RX CONFIG] RF config updated from TX. Reboot both modules to apply.\n");
        return;
    }

    uint8_t uid[xlrs::LINK_UID_SIZE] = {};
    if (xlrs::parseBindUidMessage(message.data, message.len, uid)) {
        if (g_bindingStore.setBindingUid(uid)) {
            printf("[RX CONFIG] Binding identity updated from TX. Rebooting RX...\n");
            blinkStatusLedBlocking(5, 40, 40);
            xlrs::hal::sleepMs(100);
            watchdog_reboot(0, 0, 0);
        }
        return;
    }

    if (xlrs::appTelemetryHasPrefix(message.data, message.len, xlrs::AppTelemetryType::Reboot)) {
        printf("[RX CONFIG] Reboot command received from TX.\n");
        xlrs::hal::sleepMs(100);
        watchdog_reboot(0, 0, 0);
    }
}

static void updateLED(xlrs::LinkState state, bool outputActive, bool hardwareError,
                      bool bindScanOpen, bool bindPacketReceived) {
    xlrs::app::LinkStatusLedFlags flags{};
    flags.hardwareError = hardwareError;
    flags.configFault = g_configFault;
    flags.bindScanOpen = bindScanOpen;
    flags.bindPacketReceived = bindPacketReceived;
    flags.outputActive = outputActive;
    flags.requireOutputForConnected = true;
    xlrs::app::linkStatusLedUpdate(STATUS_LED_PIN, state, flags);
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
// (including the init-failure spin in rf_core_main) stops the feed and the watchdog reboots.
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

    uint8_t uid[8];
    if (g_bindingStore.getBindingUid(uid)) {
        printIdentity(uid);
    }

    crsf.begin(CRSF_BAUDRATE);
    crsf.onPacketRaw = onCrsfFrameFromFlightController;

    printf("CRSF UART and status LED on GP%d (active-%s).\n", (int)STATUS_LED_PIN,
           XLRS_STATUS_LED_ACTIVE_LOW ? "low" : "high");
}

static void app_core_loop() {
    static uint32_t lastSeq = 0;
    static uint32_t lastCRSF = 0;
    RfToAppData rfData{};

    bool isNew = g_rfToApp.loadIfNew(rfData, lastSeq);
    if (!isNew) {
        g_rfToApp.load(rfData);
    }
    drainPhyDiagLogs();

    xlrs::LinkState state = rfData.state;
    bool outputActive = rfData.outputActive;
    uint32_t now = xlrs::hal::nowMs();
    bool shouldOutput = isNew || (now - lastCRSF >= 20);

    if (shouldOutput) {
        lastCRSF = now;

        if (state == xlrs::LinkState::Connected) {
            for (int i = 0; i < RC_CHANNELS; i++) localChannels[i] = rfData.channels[i];
        } else if (state == xlrs::LinkState::Failsafe) {
            for (int i = 0; i < RC_CHANNELS; i++) localChannels[i] = RC_US_MID;
            localChannels[RC_THROTTLE_CH] = RC_THROTTLE_FAILSAFE;
        }

        if (outputActive) {
            crsf_channels_t crsfChannels{};
            uint16_t crsfRcUs[CRSF_NUM_CHANNELS] = {};
            for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; ++i) {
                crsfRcUs[i] = i < RC_CHANNELS ? localChannels[i] : RC_US_MID;
            }
            xlrs::rcUsToCrsfChannels(crsfRcUs, crsfChannels);

            crsf.queuePacket(CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_RC_CHANNELS_PACKED,
                             &crsfChannels, CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE);
            g_crsfRcFramesOut++;
            g_lastCrsfRcOutMs = now;
        }
    }

    if (now - lastLinkStats >= LINK_STATS_INTERVAL_MS) {
        lastLinkStats = now;
        uint8_t stats[10] = {};
        xlrs::buildCrsfLinkStatistics(rfData.stats, stats);
        crsf.queuePacket(CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_LINK_STATISTICS,
                         stats, sizeof(stats));
        g_crsfLinkStatsOut++;
        const uint32_t fcAge = g_lastFcCrsfFrameMs == 0 ? 0 : now - g_lastFcCrsfFrameMs;
        const uint32_t rcOutAge = g_lastCrsfRcOutMs == 0 ? 0 : now - g_lastCrsfRcOutMs;
        printf("[RX STATUS] State: %d LQ: %u%% RSSI: %d dBm | PHY timeouts: %lu CRC: %lu Phase: %s/%s LastOp: 0x%02X LastOk: 0x%02X LastFailOp: 0x%02X | lock:%u sync:%u tick:%lu fhss:%u exp:%u skew:%d pfd:%ldus adj:%ldus n:%lu tmr:%lu/%lu | out:%u crsf_rc:%lu age:%lums stats:%lu fc:%lu fcq:%lu fcdrop:%lu fcage:%lums qdrop:%lu%s\n",
               (int)state, (unsigned)rfData.stats.lqUp, (int)rfData.stats.rssiDbm,
               (unsigned long)g_phy.spiTimeouts(), (unsigned long)g_phy.crcErrors(),
               xlrs::Sx1280NativePhy::diagPhaseName(g_phy.lastDiagPhase()),
               xlrs::Sx1280NativePhy::diagStatusName(g_phy.lastDiagStatus()),
               (unsigned)g_phy.lastStartedOpcode(),
               (unsigned)g_phy.lastCompletedOpcode(),
               (unsigned)g_phy.lastFailOpcode(),
               rfData.linkDiag.fhssLocked ? 1u : 0u,
               rfData.linkDiag.syncSeen ? 1u : 0u,
               (unsigned long)rfData.linkDiag.schedulerTick,
               (unsigned)rfData.linkDiag.fhssIndex,
               (unsigned)rfData.linkDiag.fhssExpected,
               (int)rfData.linkDiag.syncFhssSkew,
               (long)rfData.linkDiag.pfdPhaseUs,
               (long)rfData.linkDiag.pfdAdjUs,
               (unsigned long)rfData.linkDiag.pfdUpdates,
               (unsigned long)rfData.linkDiag.timerIntervalUs,
               (unsigned long)rfData.linkDiag.nomIntervalUs,
               outputActive ? 1u : 0u,
               (unsigned long)g_crsfRcFramesOut,
               (unsigned long)rcOutAge,
               (unsigned long)g_crsfLinkStatsOut,
               (unsigned long)g_fcCrsfFramesSeen,
               (unsigned long)g_fcCrsfFramesQueued,
               (unsigned long)g_fcCrsfFramesDropped,
               (unsigned long)fcAge,
               (unsigned long)rfData.rfToAppQueueDrops,
               rfData.hardwareError ? " [HW FAULT]" :
               rfData.bindPacketReceived ? " [BIND RX]" :
               rfData.bindScanOpen ? " [BIND SCAN]" : "");
    }

    updateLED(state, outputActive, rfData.hardwareError,
              rfData.bindScanOpen, rfData.bindPacketReceived);
    crsf.loop();

    xlrs::AppTelemetryMessage telemetryMessage{};
    while (g_rfTelemetryToApp.pop(telemetryMessage)) {
        applyRxAppTelemetry(telemetryMessage);
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

    g_link.begin(xlrs::Role::Rx, uid, g_rfConfig.defaultRate, g_rfConfig.maxPowerDbm, g_rfConfig.dynamicPower == 1);
    g_link.setRegion(g_rfConfig.region == (uint8_t)xlrs::RfRegion::EU ? xlrs::FhssRegion::EU_CE : xlrs::FhssRegion::US_FCC);
    g_link.setFailsafeMode(g_rfConfig.failsafeMode == 0 ? xlrs::FailsafeMode::NoPulses : xlrs::FailsafeMode::Hold);

    xlrs::PhyConfig phyCfg = xlrs::makePhyConfig(xlrs::kRates[g_rfConfig.defaultRate], 2400.0f, g_rfConfig.maxPowerDbm, xlrs::syncWordFromUid(uid));
    if (!g_phy.init(phyCfg) ||
        !g_scheduler.begin(&g_phy, &g_link, g_rfConfig.defaultRate)) {
        publishRfFault();
        while (true) xlrs::hal::sleepMs(100);  // heartbeat stops → watchdog reboots to retry
    }

    while (true) {
        for (int pollBurst = 0; pollBurst < 4; ++pollBurst) {
            g_scheduler.poll();
        }

        static bool bindScanning = false;
        static uint32_t nextBindScanSwitchMs = xlrs::hal::nowMs() + BIND_SCAN_NORMAL_WINDOW_MS;
        static bool bindPacketReceived = false;
        static bool pendingBindMessageReady = false;
        static xlrs::AppTelemetryMessage pendingBindMessage{};
        static bool normalLinkSeen = false;
        static const uint32_t bootMs = xlrs::hal::nowMs();
        static bool telemetryPending = false;
        static xlrs::AppTelemetryMessage pendingTelemetryMessage{};
        xlrs::AppTelemetryMessage telemetryMessage{};
        if (telemetryPending) {
            telemetryPending = !g_link.queueTelemetry(pendingTelemetryMessage.data,
                                                      pendingTelemetryMessage.len);
        }
        if (!telemetryPending && g_appTelemetryToRf.pop(telemetryMessage)) {
            telemetryPending = !g_link.queueTelemetry(telemetryMessage.data, telemetryMessage.len);
            if (telemetryPending) {
                pendingTelemetryMessage = telemetryMessage;
            }
        }

        // Bind-scan windowing only matters before the first link of this power cycle. Once
        // connected, normalLinkSeen latches and the RX never re-enters bind scan, so the window
        // timer — and its per-iteration nowMs() (a 64-bit divide on the RF-core spin loop) — is
        // skipped in the steady-state hot path. processedTick() free-runs off the hardware timer
        // regardless of link state, so the scan windows still toggle while disconnected.
        const xlrs::LinkState linkState = g_link.state();
        if (linkState == xlrs::LinkState::Connected || normalLinkSeen || bindPacketReceived) {
            if (linkState == xlrs::LinkState::Connected) normalLinkSeen = true;
            if (bindScanning) {
                g_link.endBindScan(uid);
                bindScanning = false;
            }
        } else if ((int32_t)(xlrs::hal::nowMs() - bootMs) >= (int32_t)BIND_SCAN_GRACE_MS) {
            const uint32_t now = xlrs::hal::nowMs();
            if ((int32_t)(now - nextBindScanSwitchMs) >= 0) {
                if (bindScanning) {
                    g_link.endBindScan(uid);
                    bindScanning = false;
                    nextBindScanSwitchMs = now + BIND_SCAN_NORMAL_WINDOW_MS;
                } else {
                    g_link.startBindScan();
                    bindScanning = true;
                    nextBindScanSwitchMs = now + BIND_SCAN_WINDOW_MS;
                }
            }
        }

        static uint32_t lastServiceTick = 0;
        uint32_t currentTick = g_scheduler.processedTick();
        if (currentTick > lastServiceTick) {
            lastServiceTick = currentTick;

            uint8_t receivedBindUid[xlrs::LINK_UID_SIZE] = {};
            xlrs::AppTelemetryMessage bindMessage{};
            if (g_link.takeReceivedBindUid(receivedBindUid) &&
                xlrs::makeBindUidMessage(receivedBindUid, bindMessage)) {
                bindPacketReceived = true;
                bindScanning = false;
                g_link.setLinkUid(receivedBindUid);
                if (!g_rfTelemetryToApp.push(bindMessage)) {
                    pendingBindMessage = bindMessage;
                    pendingBindMessageReady = true;
                }
            }
            if (pendingBindMessageReady && g_rfTelemetryToApp.push(pendingBindMessage)) {
                pendingBindMessageReady = false;
            }

            RfToAppData rfData;
            rfData.state = g_link.state();
            rfData.stats = g_link.stats();
            g_link.getChannels(rfData.channels, RC_CHANNELS);
            rfData.outputActive = g_link.outputActive();
            rfData.hardwareError = !g_phy.healthy();
            rfData.bindScanOpen = bindScanning;
            rfData.bindPacketReceived = bindPacketReceived;
            xlrs::app::fillLinkRuntimeDiag(rfData.linkDiag, g_link, g_scheduler);
            rfData.rfToAppQueueDrops = g_rfTelemetryToApp.dropped();
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
