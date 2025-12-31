// RX Side using SX128x (RadioLib) instead of BLE.
#include <Arduino.h>
#include <RadioLib.h>
#include <string.h>
#include <EEPROM.h>

#include "Protocol.h"
#include "Security.h"
#include "crsfSerial.h"
#include "crsf_protocol.h"
#include "SX128xLink.h"
#include "pairing.h"
#include "PacketHandler.h"

// CRSF Serial pins
#define CRSF_TX_PIN 8
#define CRSF_RX_PIN 9


// Link timeout
const unsigned long TIMEOUT_MS = 1000;
const unsigned long CONNECTION_TIMEOUT_MS = 2000;  // Connection lost detection

// Application state
CrsfSerial crsf(Serial2, CRSF_BAUDRATE);
Security security;
SX128xLink radio;

// Channel values received from TX
uint16_t channels[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

// Connection state machine
ConnectionState connectionState = STATE_DISCONNECTED;
unsigned long lastDataReceived = 0;
unsigned long lastSyncReceived = 0;
uint16_t lastSequence = 0;
uint16_t syncSequence = 0;

// Device IDs
uint8_t rxDeviceId[DEVICE_ID_SIZE];
uint8_t pairedTxDeviceId[DEVICE_ID_SIZE];
bool hasPairedTxId = false;

// Pairing mode state
bool pairingMode = false;
unsigned long pairingModeStartTime = 0;

// Connection quality metrics
int16_t lastRSSI = 0;
float lastSNR = 0.0f;
unsigned long connectionStartTime = 0;

// Battery telemetry state
float batteryVoltage = 0.0f;
float batteryCurrent = 0.0f;
uint32_t batteryCapacity = 0;
uint8_t batteryRemaining = 0;
bool batteryDataValid = false;
unsigned long lastBatteryUpdate = 0;
unsigned long lastBatteryTxTime = 0;

// Link statistics telemetry state
crsfLinkStatistics_t lastLinkStats = {0};
unsigned long lastLinkStatsLog = 0;

// Forward declarations
void updateConnectionState();
bool isConnectionReady();
bool isPaired();
void detectConnectionLoss();

// ============================================================
// Battery / link callbacks from CRSF
// ============================================================
void onBatteryTelemetry(float voltage, float current, uint32_t capacity, uint8_t remaining) {
    batteryVoltage = voltage;
    batteryCurrent = current;
    batteryCapacity = capacity;
    batteryRemaining = remaining;
    batteryDataValid = true;
    lastBatteryUpdate = millis();

    static unsigned long lastLog = 0;
    if (millis() - lastLog > 10000) {
        Serial.print("[BATTERY] V:");
        Serial.print(voltage, 1);
        Serial.print(" A:");
        Serial.print(current, 1);
        Serial.print(" Rem:");
        Serial.print(remaining);
        Serial.println("%");
        lastLog = millis();
    }
}

void onLinkStats(crsfLinkStatistics_t *ls) {
    lastLinkStats = *ls;
    unsigned long now = millis();
    if (now - lastLinkStatsLog > 2000) {
        Serial.print("[CRSF] LQ:");
        Serial.print(ls->uplink_Link_quality);
        Serial.print("% RSSI1:");
        Serial.print(ls->uplink_RSSI_1);
        Serial.print(" RSSI2:");
        Serial.print(ls->uplink_RSSI_2);
        Serial.print(" SNR:");
        Serial.print(ls->uplink_SNR);
        Serial.print(" DL_RSSI:");
        Serial.print(ls->downlink_RSSI);
        Serial.print(" DL_LQ:");
        Serial.print(ls->downlink_Link_quality);
        Serial.println("%");
        lastLinkStatsLog = now;
    }
}

// ============================================================
// Setup
// ============================================================
void setup() {
    Serial.begin(115200);
    unsigned long start = millis();
    while (!Serial && (millis() - start < 2000)) {
        delay(10);
    }
    delay(500);

    Serial.println("=== FPV Receiver (SX128x) ===");
    Serial.println("Initializing...");

    // Initialize security (loads device ID and pairing key if exists)
    if (security.begin()) {
        Serial.println("Security initialized");
        
        // Get this device's ID
        if (security.getDeviceId(rxDeviceId)) {
            Serial.print("RX Device ID: ");
            for (int i = 0; i < DEVICE_ID_SIZE; i++) {
                if (rxDeviceId[i] < 0x10) Serial.print("0");
                Serial.print(rxDeviceId[i], HEX);
            }
            Serial.println();
        }
        
        // Get binding UID
        uint8_t bindingUID[BINDING_UID_SIZE];
        if (security.getBindingUID(bindingUID)) {
            Serial.print("Binding UID: ");
            for (int i = 0; i < BINDING_UID_SIZE; i++) {
                if (bindingUID[i] < 0x10) Serial.print("0");
                Serial.print(bindingUID[i], HEX);
            }
            Serial.print(" (from phrase: ");
            Serial.print(DEFAULT_BINDING_PHRASE);
            Serial.println(")");
        } else {
            Serial.println("WARNING: No binding UID!");
        }
        
        // Check if we have a paired TX device ID
        if (security.getPairedDeviceId(pairedTxDeviceId)) {
            hasPairedTxId = true;
            Serial.print("Paired TX Device ID: ");
            for (int i = 0; i < DEVICE_ID_SIZE; i++) {
                if (pairedTxDeviceId[i] < 0x10) Serial.print("0");
                Serial.print(pairedTxDeviceId[i], HEX);
            }
            Serial.println();
        }
        
        // Check pairing status
        if (isPaired()) {
            Serial.println("Already paired - ready to connect");
            connectionState = STATE_CONNECTING;
        } else {
            Serial.println("Not paired - enter pairing mode to connect");
            Serial.println("Hold BOOTSEL for 5 seconds to enter pairing mode");
            connectionState = STATE_DISCONNECTED;
        }
    } else {
        Serial.println("Security initialization failed!");
        connectionState = STATE_DISCONNECTED;
    }

    // Initialize radio
    if (radio.begin()) {
        Serial.println("SX128x ready");
    } else {
        Serial.println("SX128x init failed");
    }

    // Initialize CRSF
    Serial.println("Initializing CRSF...");
    Serial2.setTX(CRSF_TX_PIN);
    Serial2.setRX(CRSF_RX_PIN);
    crsf.begin(CRSF_BAUDRATE);
    crsf.onPacketBattery = onBatteryTelemetry;
    crsf.onPacketLinkStatistics = onLinkStats;
    Serial.print("CRSF on Serial2: TX=GP");
    Serial.print(CRSF_TX_PIN);
    Serial.print(", RX=GP");
    Serial.println(CRSF_RX_PIN);

    Serial.println("=== RX Ready ===");
    Serial.println("Hold BOOTSEL for 5 seconds to enter pairing mode");
}

// ============================================================
// Loop
// ============================================================
void loop() {
    // Update connection state machine
    updateConnectionState();
    
    // Process pairing button
    Pairing::checkBootselButton(&pairingMode, &pairingModeStartTime);
    Pairing::updatePairingMode(&pairingMode, pairingModeStartTime);

    // Handle radio RX
    SX128xPacket pkt;
    if (radio.receive(pkt) && pkt.length >= 1) {
        lastRSSI = pkt.rssi;
        lastSNR = pkt.snr;
        
        uint8_t type = pkt.data[0];
        const uint8_t* payload = pkt.data + 1;
        size_t payload_len = pkt.length - 1;
        
        switch (type) {
            case MSG_CHANNELS:
                // Handle channel data
                PacketHandler::handleRadioRxRx(&radio, &security, pairedTxDeviceId, hasPairedTxId,
                                                channels, &lastSequence, &lastDataReceived,
                                                &lastRSSI, &lastSNR, connectionState);
                break;
            case MSG_PAIRING:
                // Handle pairing packet
                if (Pairing::handlePairingPacket(payload, payload_len, &security, &radio,
                                                  rxDeviceId, pairedTxDeviceId, &hasPairedTxId,
                                                  &pairingMode)) {
                    connectionState = STATE_CONNECTING;
                }
                break;
            case MSG_SYNC:
                // Handle sync packet
                if (PacketHandler::handleSyncPacket(payload, payload_len, &security,
                                                      pairedTxDeviceId, hasPairedTxId,
                                                      &syncSequence, &lastSyncReceived)) {
                    // Send sync ACK
                    PacketHandler::sendSyncAck(&radio, &security, rxDeviceId, syncSequence);
                }
                break;
            case MSG_BATTERY:
                // Battery telemetry from TX (not used on RX side)
                break;
            default:
                break;
        }
    }
    
    // Detect connection loss
    detectConnectionLoss();

    // Check for timeout (only if connected)
    if (connectionState == STATE_CONNECTED && lastDataReceived > 0 && (millis() - lastDataReceived > TIMEOUT_MS)) {
        static unsigned long lastTimeout = 0;
        if (millis() - lastTimeout > 5000) {
            Serial.println("*** DATA TIMEOUT ***");
            lastTimeout = millis();
        }
        for (int i = 0; i < 8; i++) {
            channels[i] = 1500;
        }
    }

    // Process CRSF (receives telemetry from FC on GP9)
    crsf.loop();

    // Send battery telemetry to TX over radio (rate-limited)
    PacketHandler::sendBatteryTelemetry(&radio, batteryVoltage, batteryCurrent,
                                         batteryRemaining, batteryDataValid,
                                         &lastBatteryTxTime);

    // Send CRSF at 50Hz
    static unsigned long lastCRSF = 0;
    if (millis() - lastCRSF >= 20) {
        lastCRSF = millis();

        crsf_channels_t crsfChannels = {0};
        crsfChannels.ch0 = map(channels[0], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);  // Aileron
        crsfChannels.ch1 = map(channels[1], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);  // Elevator
        crsfChannels.ch2 = map(channels[2], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);  // Rudder
        crsfChannels.ch3 = map(channels[3], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);  // Throttle
        crsfChannels.ch4 = map(channels[4], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);  // Aux1
        crsfChannels.ch5 = map(channels[5], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);  // Aux2
        crsfChannels.ch6 = map(channels[6], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);  // Aux3
        crsfChannels.ch7 = map(channels[7], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);  // Aux4

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


// ============================================================
// Connection state management
// ============================================================
bool isPaired() {
    return security.hasPairingKey() && hasPairedTxId;
}

bool isConnectionReady() {
    return isPaired() && connectionState == STATE_CONNECTED && 
           (millis() - lastDataReceived < CONNECTION_TIMEOUT_MS);
}

void updateConnectionState() {
    static unsigned long lastStateLog = 0;
    
    // State transitions
    switch (connectionState) {
        case STATE_DISCONNECTED:
            if (pairingMode) {
                connectionState = STATE_PAIRING;
                Serial.println("[STATE] DISCONNECTED -> PAIRING");
            } else if (isPaired()) {
                connectionState = STATE_CONNECTING;
                Serial.println("[STATE] DISCONNECTED -> CONNECTING");
            }
            break;
            
        case STATE_PAIRING:
            if (!pairingMode && isPaired()) {
                connectionState = STATE_CONNECTING;
                Serial.println("[STATE] PAIRING -> CONNECTING");
            }
            break;
            
        case STATE_CONNECTING:
            // Wait for sync handshake to complete
            if (lastSyncReceived > 0 && (millis() - lastSyncReceived < 1000)) {
                connectionState = STATE_CONNECTED;
                connectionStartTime = millis();
                Serial.println("[STATE] CONNECTING -> CONNECTED");
            }
            break;
            
        case STATE_CONNECTED:
            // Stay connected as long as we receive data
            break;
            
        case STATE_LOST:
            // Try to reconnect
            if (isPaired()) {
                connectionState = STATE_CONNECTING;
                Serial.println("[STATE] LOST -> CONNECTING (reconnecting)");
            } else {
                connectionState = STATE_DISCONNECTED;
                Serial.println("[STATE] LOST -> DISCONNECTED (no pairing)");
            }
            break;
    }
    
    // Log state periodically
    if (millis() - lastStateLog > 10000) {
        const char* stateNames[] = {"DISCONNECTED", "PAIRING", "CONNECTING", "CONNECTED", "LOST"};
        Serial.print("[STATE] Current: ");
        Serial.print(stateNames[connectionState]);
        if (connectionState == STATE_CONNECTED) {
            Serial.print(" (RSSI: ");
            Serial.print(lastRSSI);
            Serial.print(", SNR: ");
            Serial.print(lastSNR);
            Serial.print(")");
        }
        Serial.println();
        lastStateLog = millis();
    }
}

void detectConnectionLoss() {
    if (connectionState == STATE_CONNECTED) {
        // Check if we haven't received data or sync for too long
        unsigned long timeSinceData = lastDataReceived > 0 ? (millis() - lastDataReceived) : CONNECTION_TIMEOUT_MS;
        unsigned long timeSinceSync = lastSyncReceived > 0 ? (millis() - lastSyncReceived) : CONNECTION_TIMEOUT_MS;
        
        if (timeSinceData > CONNECTION_TIMEOUT_MS && timeSinceSync > CONNECTION_TIMEOUT_MS) {
            connectionState = STATE_LOST;
            Serial.println("[STATE] CONNECTED -> LOST (timeout)");
        }
    }
}



