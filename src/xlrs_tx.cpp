// XLRS High-Performance Dual-Core Transmitter main (M3).
#include <Arduino.h>
#include "link/Link.h"
#include "link/RfScheduler.h"
#include "link/Uid.h"
#ifdef USE_NATIVE_PHY
#include "phy/Sx1280NativePhy.h"
#else
#include "phy/RadioLibPhy.h"
#endif
#include "fhss/Fhss.h"
#include "util/Mailbox.h"
#include "UARTProtocol.h"
#include "Security.h"
#include "link/RfConfig.h"
#include <EEPROM.h>

#ifndef UART_PROTOCOL_TX
#define UART_PROTOCOL_TX 8
#endif
#ifndef UART_PROTOCOL_RX
#define UART_PROTOCOL_RX 9
#endif

// Thread-safe cross-core mailboxes
struct AppToRfData {
    uint16_t channels[8];
};

struct RfToAppData {
    xlrs::LinkState state;
    xlrs::LinkStats stats;
};

xlrs::LatestValue<AppToRfData> g_appToRf;
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

UARTProtocol uartProto(&Serial2);

// Security and RF Config globals
Security security;
xlrs::RfConfigData g_rfConfig;

// Telemetry/Status reporting timing
static unsigned long lastTelemetrySent = 0;
static const unsigned long TELEMETRY_INTERVAL = 200;  // 5Hz

static unsigned long lastStatusSent = 0;
static const unsigned long STATUS_INTERVAL = 1000;  // 1Hz

void onChannelsReceived(const ChannelData* data) {
    if (data) {
        AppToRfData payload;
        for (int i = 0; i < 8; i++) {
            payload.channels[i] = data->channels[i];
        }
        g_appToRf.store(payload);
    }
}

void onCommandReceived(UARTMsgType cmd) {
    switch (cmd) {
        case UART_MSG_CMD_PAIR:
            Serial.println("[UART] PAIR command received - XLRS clean link runs on bind phrase!");
            uartProto.sendAck(UART_MSG_CMD_PAIR);
            break;
        case UART_MSG_CMD_BOND:
            Serial.println("[UART] BOND command received");
            uartProto.sendAck(UART_MSG_CMD_BOND);
            break;
        case UART_MSG_CMD_RESTART:
            Serial.println("[UART] RESTART command received");
            uartProto.sendAck(UART_MSG_CMD_RESTART);
            delay(100);
            rp2040.restart();
            break;
        case UART_MSG_CMD_STATUS_REQ:
            uartProto.sendAck(UART_MSG_CMD_STATUS_REQ);
            break;
        default:
            break;
    }
}

void onCommandPayloadReceived(UARTMsgType cmd, const uint8_t* payload, uint8_t length) {
    if (cmd == UART_MSG_CMD_SET_BIND_TX && payload && length > 0 && length <= 32) {
        char phrase[33];
        memcpy(phrase, payload, length);
        phrase[length] = '\0';
        Serial.print("[UART] SET_BIND_TX command received: ");
        Serial.println(phrase);
        uartProto.sendAck(UART_MSG_CMD_SET_BIND_TX);
        delay(10);
        if (security.setBindingPhrase(phrase)) {
            Serial.println("[UART] Binding phrase persisted. Rebooting TX...");
            delay(100);
            rp2040.restart();
        } else {
            Serial.println("[UART] ERROR: Failed to persist binding phrase!");
        }
    }
}

// ============================================================
// Core 0: High-Level Handset UART Interface & Telemetry
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== XLRS Dual-Core Transmitter ===");
    
    // Initialize EEPROM and Security
    EEPROM.begin(512);
    if (security.begin()) {
        Serial.println("Security system initialized.");
    } else {
        Serial.println("ERROR: Security system initialization failed!");
    }
    
    // Load or initialize RF Config
    if (xlrs::RfConfig::load(g_rfConfig)) {
        Serial.println("RF Configuration loaded from EEPROM.");
    } else {
        Serial.println("No valid RF Configuration found. Storing default values...");
        xlrs::RfConfig::setDefaults(g_rfConfig);
        xlrs::RfConfig::save(g_rfConfig);
    }
    Serial.print("RF Region: ");
    Serial.println(g_rfConfig.region == 0 ? "US" : "EU");
    Serial.print("Max Power Cap: ");
    Serial.print(g_rfConfig.maxPowerDbm);
    Serial.println(" dBm");

    // Initialize UART Protocol
    Serial2.setTX(UART_PROTOCOL_TX);
    Serial2.setRX(UART_PROTOCOL_RX);
    uartProto.begin(UART_PROTOCOL_BAUDRATE);
    uartProto.setOnChannels(onChannelsReceived);
    uartProto.setOnCommand(onCommandReceived);
    uartProto.setOnCommandPayload(onCommandPayloadReceived);
    
    Serial.println("UART Protocol on Core 0 initialized.");
}

void loop() {
    uartProto.loop();
    
    // Fetch LinkStats from Core 1
    RfToAppData rfData;
    if (g_rfToApp.load(rfData)) {
        unsigned long now = millis();
        // Send telemetry to handset
        if (now - lastTelemetrySent >= TELEMETRY_INTERVAL) {
            lastTelemetrySent = now;
            
            TelemetryData telem{};
            telem.rssi = rfData.stats.rssiDbm;
            telem.snr = rfData.stats.snr;
            telem.linkQuality = rfData.stats.lqDown; // down link quality for TX view
            telem.rxBattMv = 0;
            telem.rxBattPct = 0;
            
            uartProto.sendTelemetry(&telem);
        }
        
        // Send status to handset
        if (now - lastStatusSent >= STATUS_INTERVAL) {
            lastStatusSent = now;
            
            StatusData status{};
            // Map xlrs::LinkState to handset state (0=Disconn, 1=Pairing, 2=Conn_ing, 3=Conn_ed, 4=Lost)
            uint8_t connState = 0;
            switch (rfData.state) {
                case xlrs::LinkState::Disconnected: connState = 0; break;
                case xlrs::LinkState::Binding:      connState = 1; break;
                case xlrs::LinkState::Connecting:   connState = 2; break;
                case xlrs::LinkState::Connected:    connState = 3; break;
                case xlrs::LinkState::Failsafe:     connState = 4; break;
                default:                            connState = 0; break;
            }
            status.connectionState = connState;
            status.pairingState = 1; // Always paired/ready
            status.packetsReceived = 0;
            status.packetsLost = rfData.stats.missedDeadlines;
            
            uartProto.sendStatus(&status);
        }
    }
}

// ============================================================
// Core 1: Low-Jitter Microsecond RF Scheduling Core
// ============================================================
void setup1() {
    // Core 1 local EEPROM & Security init to prevent startup races
    EEPROM.begin(512);
    security.begin();
    if (!xlrs::RfConfig::load(g_rfConfig)) {
        xlrs::RfConfig::setDefaults(g_rfConfig);
    }

    uint8_t uid[8];
    // Load UID from EEPROM, or generate default if not present
    if (security.getBindingUID(uid)) {
        Serial.println("[Core 1] Binding UID loaded from Security store.");
    } else {
        Serial.println("[Core 1] No binding UID found. Creating from default phrase...");
        security.generateBindingUID(DEFAULT_BINDING_PHRASE);
        security.getBindingUID(uid);
    }
    
    g_link.begin(xlrs::Role::Tx, uid, g_rfConfig.defaultRate, g_rfConfig.maxPowerDbm, g_rfConfig.dynamicPower == 1);
    
    xlrs::PhyConfig phyCfg = xlrs::makePhyConfig(xlrs::kRates[g_rfConfig.defaultRate], 2400.0f, g_rfConfig.maxPowerDbm, xlrs::syncWordFromUid(uid));

    if (!g_phy.init(phyCfg)) {
        Serial.println("[ERROR] Radio PHY initialization failed!");
        while (true) {
            delay(100);
        }
    }
    if (!g_scheduler.begin(&g_phy, &g_fhss, &g_link, g_rfConfig.defaultRate)) {
        Serial.println("[ERROR] Scheduler initialization failed!");
        while (true) {
            delay(100);
        }
    }
}

void loop1() {
    // 1. Process incoming channels from Core 0
    AppToRfData appData;
    if (g_appToRf.load(appData)) {
        g_link.setChannels(appData.channels, 8);
    }
    
    // 2. Poll the RF Scheduler background events (processes ticks, rxDone, txDone)
    g_scheduler.poll();
    
    // 3. Service the link lifecycle logic
    static uint32_t lastServiceTick = 0;
    uint32_t currentTick = g_scheduler.processedTick();
    if (currentTick > lastServiceTick) {
        lastServiceTick = currentTick;
        
        // Publish link state and stats to Core 0
        RfToAppData rfData;
        rfData.state = g_link.state();
        rfData.stats = g_link.stats();
        g_rfToApp.store(rfData);
    }
}
