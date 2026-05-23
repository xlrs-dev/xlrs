// XLRS High-Performance Dual-Core Receiver main (M3).
#include <Arduino.h>
#include <FastLED.h>
#include "link/Link.h"
#include "link/RfScheduler.h"
#include "link/Uid.h"
#ifdef USE_NATIVE_PHY
#include "phy/Sx1280NativePhy.h"
#else
#include "phy/RadioLibPhy.h"
#endif
#include "fhss/Fhss.h"
#include "app/CrsfLinkStats.h"
#include "util/Mailbox.h"
#include "crsfSerial.h"
#include "crsf_protocol.h"
#include "link/BindingStore.h"
#include "link/RfConfig.h"
#include <EEPROM.h>

#ifndef CRSF_TX_PIN
#define CRSF_TX_PIN 8
#endif
#ifndef CRSF_RX_PIN
#define CRSF_RX_PIN 9
#endif

#define WS2812_PIN 13
#define NUM_LEDS 1

// Thread-safe cross-core mailbox
struct RfToAppData {
    xlrs::LinkState state;
    xlrs::LinkStats stats;
    uint16_t channels[8];
    bool outputActive;
    bool hardwareError;
};

xlrs::LatestValue<RfToAppData> g_rfToApp;

// Clean-slate XLRS link core components
xlrs::Link g_link;
xlrs::RfScheduler g_scheduler;
#ifdef USE_NATIVE_PHY
xlrs::Sx1280NativePhy g_phy;
#else
xlrs::RadioLibPhy g_phy;
#endif
xlrs::Fhss g_fhss;

CrsfSerial crsf(Serial2, CRSF_BAUDRATE);
CRGB leds[NUM_LEDS];

// Binding identity and RF Config globals
xlrs::BindingStore g_bindingStore;
xlrs::RfConfigData g_rfConfig;

unsigned long lastLedUpdate = 0;
static const unsigned long LED_UPDATE_INTERVAL_MS = 50;  // 20Hz update rate
static unsigned long lastLinkStats = 0;
static const unsigned long LINK_STATS_INTERVAL_MS = 500; // CRSF LINK_STATISTICS cadence

// Local channels array (fallback to center/disarm values)
uint16_t localChannels[8] = {1500, 1500, 1500, 1000, 1500, 1500, 1500, 1500}; // Ail, Ele, Rud center; Thr low; aux center

// Config fault warning state
bool g_configFault = false;

void updateLED(xlrs::LinkState state, bool outputActive, bool hardwareError) {
    if (millis() - lastLedUpdate < LED_UPDATE_INTERVAL_MS) return;
    lastLedUpdate = millis();
    
    static uint32_t msCounter = 0;
    msCounter += LED_UPDATE_INTERVAL_MS;

    if (g_configFault) {
        // Bad Config: solid Red
        leds[0] = CRGB(50, 0, 0);
    } else if (hardwareError) {
        // Radio Hardware Fault: fast double-blinking Red (rapid flash-flash, pause)
        uint16_t phase = (msCounter % 1000);
        if (phase < 100 || (phase >= 200 && phase < 300)) {
            leds[0] = CRGB(60, 0, 0);
        } else {
            leds[0] = CRGB::Black;
        }
    } else if (state == xlrs::LinkState::Binding) {
        // Binding/Pairing: fast blinking Blue (10Hz)
        if ((msCounter % 100) < 50) {
            leds[0] = CRGB(0, 0, 50);
        } else {
            leds[0] = CRGB::Black;
        }
    } else if (state == xlrs::LinkState::Connected && outputActive) {
        // Connected: solid Green
        leds[0] = CRGB(0, 40, 0);
    } else if (state == xlrs::LinkState::Connecting) {
        // Connecting: slow breathing Orange
        static float angle = 0;
        uint8_t val = (uint8_t)(15.0f + 15.0f * sin(angle));
        leds[0] = CRGB(val * 2, val, 0);
        angle += 0.05f; // slower breathing
    } else {
        // Disconnected or Failsafe: slow blinking Red (1Hz)
        if ((msCounter % 1000) < 500) {
            leds[0] = CRGB(40, 0, 0);
        } else {
            leds[0] = CRGB::Black;
        }
    }
    FastLED.show();
}

// ============================================================
// Core 0: High-Level Flight Controller Interface & LEDs
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== XLRS Dual-Core Receiver ===");
    
    // Initialize EEPROM and binding identity store
    EEPROM.begin(512);
    if (g_bindingStore.begin()) {
        Serial.println("Binding identity store initialized.");
    } else {
        Serial.println("ERROR: Binding identity store initialization failed!");
    }

    // Load or initialize RF Config
    if (xlrs::RfConfig::load(g_rfConfig)) {
        Serial.println("RF Configuration loaded from EEPROM.");
    } else {
        Serial.println("No valid RF Configuration found. Storing default values...");
        g_configFault = true; // Flag configuration fault warning
        xlrs::RfConfig::setDefaults(g_rfConfig);
        xlrs::RfConfig::save(g_rfConfig);
    }
    Serial.print("RF Region: ");
    Serial.println(g_rfConfig.region == 0 ? "US" : "EU");
    Serial.print("Failsafe Mode: ");
    Serial.println(g_rfConfig.failsafeMode == 0 ? "NoPulses" : "Hold");

    // Initialize CRSF Serial
    Serial2.setTX(CRSF_TX_PIN);
    Serial2.setRX(CRSF_RX_PIN);
    crsf.begin(CRSF_BAUDRATE);
    
    // Initialize WS2812 LED
    FastLED.addLeds<WS2812, WS2812_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(255);
    leds[0] = CRGB::Black;
    FastLED.show();
    
    Serial.println("CRSF and LED on Core 0 initialized.");
}

void loop() {
    static uint32_t lastSeq = 0;
    static unsigned long lastCRSF = 0;
    RfToAppData rfData{};
    
    // Check if there is a new packet from Core 1
    bool isNew = g_rfToApp.loadIfNew(rfData, lastSeq);
    if (!isNew) {
        g_rfToApp.load(rfData);
    }
    
    xlrs::LinkState state = rfData.state;
    bool outputActive = rfData.outputActive;
    
    unsigned long now = millis();
    
    // Output immediately upon new packet arrival for ultra-low latency, 
    // or at 50Hz (20ms interval) as fallback for failsafe keepalive/disconnected.
    bool shouldOutput = isNew || (now - lastCRSF >= 20);
    
    if (shouldOutput) {
        lastCRSF = now;
        
        // Update local channels if link is active
        if (state == xlrs::LinkState::Connected) {
            for (int i = 0; i < 8; i++) {
                localChannels[i] = rfData.channels[i];
            }
        } else if (state == xlrs::LinkState::Failsafe) {
            // Failsafe condition: set Thr to low, other channels to center
            localChannels[0] = 1500;
            localChannels[1] = 1500;
            localChannels[2] = 1500;
            localChannels[3] = 1000;
            for (int i = 4; i < 8; i++) {
                localChannels[i] = 1500;
            }
        }
        
        if (outputActive) {
            crsf_channels_t crsfChannels = {0};
            crsfChannels.ch0 = map(localChannels[0], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
            crsfChannels.ch1 = map(localChannels[1], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
            crsfChannels.ch2 = map(localChannels[2], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
            crsfChannels.ch3 = map(localChannels[3], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
            crsfChannels.ch4 = map(localChannels[4], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
            crsfChannels.ch5 = map(localChannels[5], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
            crsfChannels.ch6 = map(localChannels[6], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
            crsfChannels.ch7 = map(localChannels[7], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
            
            uint16_t center = CRSF_CHANNEL_VALUE_MID;
            crsfChannels.ch8 = center;
            crsfChannels.ch9 = center;
            crsfChannels.ch10 = center;
            crsfChannels.ch11 = center;
            crsfChannels.ch12 = center;
            crsfChannels.ch13 = center;
            crsfChannels.ch14 = center;
            crsfChannels.ch15 = center;
            
            crsf.queuePacket(
                CRSF_ADDRESS_FLIGHT_CONTROLLER,
                CRSF_FRAMETYPE_RC_CHANNELS_PACKED,
                &crsfChannels,
                CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE
            );
        }
    }

    if (now - lastLinkStats >= LINK_STATS_INTERVAL_MS) {
        lastLinkStats = now;

        xlrs::LinkStats stats = rfData.stats;
        if (state != xlrs::LinkState::Connected) {
            stats.lqUp = 0;
            stats.lqDown = 0;
        }

        uint8_t payload[CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE];
        xlrs::buildCrsfLinkStatistics(stats, payload);
        crsf.queuePacket(
            CRSF_ADDRESS_FLIGHT_CONTROLLER,
            CRSF_FRAMETYPE_LINK_STATISTICS,
            payload,
            sizeof(payload)
        );
    }

    // Log status periodically
    static unsigned long lastLog = 0;
    if (now - lastLog > 2000) {
        lastLog = now;
        Serial.print("[RX STATUS] State: ");
        Serial.print((int)state);
        Serial.print(" LQ: ");
        Serial.print(rfData.stats.lqUp);
        Serial.print("% RSSI: ");
        Serial.print(rfData.stats.rssiDbm);
        Serial.println("dBm");
    }
    
    // Update WS2812 status LED
    updateLED(state, outputActive, rfData.hardwareError);
    
    // Let CRSF run its background tasks
    crsf.loop();
}

// ============================================================
// Core 1: Low-Jitter Microsecond RF Scheduling Core
// ============================================================
void setup1() {
    // Core 1 local EEPROM & binding identity init to prevent startup races
    EEPROM.begin(512);
    g_bindingStore.begin();
    if (!xlrs::RfConfig::load(g_rfConfig)) {
        xlrs::RfConfig::setDefaults(g_rfConfig);
    }

    uint8_t uid[8];
    // Load UID from EEPROM, or generate default if not present
    if (g_bindingStore.getBindingUid(uid)) {
        Serial.println("[Core 1] Binding UID loaded from XLRS binding store.");
    } else {
        Serial.println("[Core 1] No binding UID found. Creating from default phrase...");
        g_bindingStore.setBindingPhrase(DEFAULT_BINDING_PHRASE);
        g_bindingStore.getBindingUid(uid);
    }
    
    g_link.begin(xlrs::Role::Rx, uid, g_rfConfig.defaultRate, g_rfConfig.maxPowerDbm, g_rfConfig.dynamicPower == 1);
    g_link.setRegion(g_rfConfig.region == (uint8_t)xlrs::RfRegion::EU ? xlrs::FhssRegion::EU_CE : xlrs::FhssRegion::US_FCC);
    
    // Configure receiver failsafe behavior
    g_link.setFailsafeMode(g_rfConfig.failsafeMode == 0 ? xlrs::FailsafeMode::NoPulses : xlrs::FailsafeMode::Hold);
    
    xlrs::PhyConfig phyCfg = xlrs::makePhyConfig(xlrs::kRates[g_rfConfig.defaultRate], 2400.0f, g_rfConfig.maxPowerDbm, xlrs::syncWordFromUid(uid));

    if (!g_phy.init(phyCfg)) {
        Serial.println("[ERROR] Radio PHY initialization failed!");
        RfToAppData rfData{};
        rfData.hardwareError = true;
        g_rfToApp.store(rfData);
        while (true) {
            delay(100);
        }
    }
    if (!g_scheduler.begin(&g_phy, &g_fhss, &g_link, g_rfConfig.defaultRate)) {
        Serial.println("[ERROR] Scheduler initialization failed!");
        RfToAppData rfData{};
        rfData.hardwareError = true;
        g_rfToApp.store(rfData);
        while (true) {
            delay(100);
        }
    }
}

void loop1() {
    // 1. Poll the RF Scheduler background events (processes ticks, rxDone, txDone)
    g_scheduler.poll();
    
    // 2. Service the link lifecycle logic & publish state
    static uint32_t lastServiceTick = 0;
    uint32_t currentTick = g_scheduler.processedTick();
    if (currentTick > lastServiceTick) {
        lastServiceTick = currentTick;
        
        // Publish link state, stats, channels, outputActive, and hardware health to Core 0
        RfToAppData rfData;
        rfData.state = g_link.state();
        rfData.stats = g_link.stats();
        g_link.getChannels(rfData.channels, 8);
        rfData.outputActive = g_link.outputActive();
        rfData.hardwareError = !g_phy.healthy();
        
        g_rfToApp.store(rfData);
    }
}
