// TX Side using SX128x (RadioLib) - radio comms only.
// CRSF input will be added later.
#include <Arduino.h>
#include <RadioLib.h>
#include "Protocol.h"
#include "Security.h"
#include "SX128xLink.h"
#include "pairing.h"
#include "PacketHandler.h"

// Timing
static unsigned long lastDataSent = 0;
static const unsigned long CONNECTION_TIMEOUT_MS = 2000;  // Connection lost detection

// Application state
Security security;
SX128xLink radio;

// Channel values - initialize to safe low values (1000)
uint16_t channels[8] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
uint16_t sequenceNumber = 0;

// Connection state machine
ConnectionState connectionState = STATE_DISCONNECTED;
unsigned long lastSyncSent = 0;
unsigned long lastSyncAckReceived = 0;
uint16_t syncSequence = 0;
unsigned long connectionStartTime = 0;

// Device IDs
uint8_t txDeviceId[DEVICE_ID_SIZE];
uint8_t pairedRxDeviceId[DEVICE_ID_SIZE];
bool hasPairedRxId = false;

// Pairing state
bool pairingMode = false;
unsigned long lastPairingSend = 0;

// Ramp pattern state
static uint16_t rampValue = 1000;
static bool rampDirection = true;  // true = up, false = down
static const uint16_t RAMP_MIN = 1000;
static const uint16_t RAMP_MAX = 2000;
static const uint16_t RAMP_STEP = 5;

// Telemetry from RX
static int16_t last_rssi = 0;
static float last_snr = 0;

// Forward declarations
static void updateConnectionState();
static bool isConnectionReady();
static bool isPaired();
static void detectConnectionLoss();
static void updateRampPattern();

// ============================================================
// Setup
// ============================================================
void setup() {
    Serial.begin(115200);
    unsigned long start = millis();
    while (!Serial && (millis() - start < 2000)) {
        delay(10);
    }
    delay(200);

    Serial.println("=== FPV Transmitter (SX128x) ===");
    Serial.println("Radio comms only - sending ramp pattern (1000-2000)");
    Serial.flush();

    // Initialize security (loads device ID and pairing key if exists)
    Serial.println("Initializing security...");
    Serial.flush();
    if (security.begin()) {
        Serial.println("Security initialized");
        
        // Get this device's ID
        if (security.getDeviceId(txDeviceId)) {
            Serial.print("TX Device ID: ");
            for (int i = 0; i < DEVICE_ID_SIZE; i++) {
                if (txDeviceId[i] < 0x10) Serial.print("0");
                Serial.print(txDeviceId[i], HEX);
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
            Serial.println("WARNING: No binding UID - pairing will fail!");
        }
        
        // Check if we have a paired RX device ID
        if (security.getPairedDeviceId(pairedRxDeviceId)) {
            hasPairedRxId = true;
            Serial.print("Paired RX Device ID: ");
            for (int i = 0; i < DEVICE_ID_SIZE; i++) {
                if (pairedRxDeviceId[i] < 0x10) Serial.print("0");
                Serial.print(pairedRxDeviceId[i], HEX);
            }
            Serial.println();
        }
        
        // Check pairing status
        if (isPaired()) {
            Serial.println("Already paired - ready to connect");
            connectionState = STATE_CONNECTING;
        } else {
            Serial.println("Not paired - entering pairing mode");
            Serial.println("Will send pairing packets with binding UID");
            pairingMode = true;
            connectionState = STATE_PAIRING;
        }
    } else {
        Serial.println("Security init failed");
        connectionState = STATE_DISCONNECTED;
    }
    Serial.flush();

    // Initialize radio
    Serial.println("Initializing radio...");
    Serial.flush();
    bool radio_ready = false;
    if (radio.begin()) {
        Serial.println("SX128x ready");
        radio_ready = true;
    } else {
        Serial.println("SX128x init failed");
    }
    Serial.flush();

    Serial.println("=== TX Ready ===");
    if (radio_ready) {
        Serial.println("Radio ready - will send channels (low values 1000) at 50Hz");
    } else {
        Serial.println("WARNING: Radio not ready - cannot send channels");
    }
    Serial.flush();
}

// ============================================================
// Loop
// ============================================================
void loop() {
    // Update connection state machine
    updateConnectionState();
    
    // Handle pairing if needed
    if (pairingMode) {
        Pairing::sendPairingKey(&radio, &security, txDeviceId, &pairingMode, &lastPairingSend);
    }
    
    // Send sync packets if connecting/connected
    if (connectionState == STATE_CONNECTING || connectionState == STATE_CONNECTED) {
        PacketHandler::sendSyncPacket(&radio, &security, txDeviceId, &syncSequence, &lastSyncSent);
    }
    
    // Update ramp pattern for testing
    if (connectionState == STATE_CONNECTED && isPaired()) {
        updateRampPattern();
    }
    
    // Send channel data over radio (only if connected)
    if (connectionState == STATE_CONNECTED && isPaired()) {
        PacketHandler::sendChannelData(&radio, &security, txDeviceId, channels, 
                                       &sequenceNumber, &lastDataSent, &last_rssi);
    }

    // Handle radio RX (telemetry, pairing ACK, sync ACK)
    SX128xPacket pkt;
    if (radio.receive(pkt) && pkt.length >= 1) {
        last_rssi = pkt.rssi;
        last_snr = pkt.snr;
        
        uint8_t type = pkt.data[0];
        const uint8_t* payload = pkt.data + 1;
        size_t payload_len = pkt.length - 1;
        
        switch (type) {
            case MSG_BATTERY:
                // Handle battery telemetry (delegated to PacketHandler logic)
                if (isPaired() && hasPairedRxId && payload_len == 5) {
                    uint16_t voltage_x10 = (payload[0] << 8) | payload[1];
                    uint16_t current_x10 = (payload[2] << 8) | payload[3];
                    float rx_battery_voltage = voltage_x10 / 10.0f;
                    float rx_battery_current = current_x10 / 10.0f;
                    uint8_t rx_battery_remaining = payload[4];
                    Serial.print("[TELEM] RX Batt ");
                    Serial.print(rx_battery_voltage, 1);
                    Serial.print("V ");
                    Serial.print(rx_battery_remaining);
                    Serial.println("%");
                }
                break;
            case MSG_PAIRING_ACK:
                if (pairingMode) {
                    if (Pairing::handlePairingAck(payload, payload_len, &security, 
                                                   pairedRxDeviceId, &hasPairedRxId, &pairingMode)) {
                        connectionState = STATE_CONNECTING;
                    }
                }
                break;
            case MSG_SYNC_ACK:
                if (isPaired() && hasPairedRxId) {
                    uint16_t syncSeq = 0;
                    uint32_t timestamp = 0;
                    uint8_t sourceDeviceId[DEVICE_ID_SIZE];
                    
                    if (Protocol::parseSyncAck(payload, payload_len, &syncSeq, &timestamp, 
                                                &security, sourceDeviceId)) {
                        if (memcmp(sourceDeviceId, pairedRxDeviceId, DEVICE_ID_SIZE) == 0) {
                            lastSyncAckReceived = millis();
                        }
                    }
                }
                break;
        }
    }
    
    // Detect connection loss
    detectConnectionLoss();
}

// ============================================================
// Ramp pattern for testing
// ============================================================
void updateRampPattern() {
    // Update ramp pattern
    if (rampDirection) {
        rampValue += RAMP_STEP;
        if (rampValue >= RAMP_MAX) {
            rampValue = RAMP_MAX;
            rampDirection = false;  // Start going down
        }
    } else {
        rampValue -= RAMP_STEP;
        if (rampValue <= RAMP_MIN) {
            rampValue = RAMP_MIN;
            rampDirection = true;  // Start going up
        }
    }

    // Apply ramp pattern to channel values for testing
    for (int i = 0; i < 8; i++) {
        // Each channel gets a different phase offset for visual variety
        int32_t offset = rampValue + (i * 50);  // 50 us offset per channel
        // Wrap around if exceeds max
        if (offset > RAMP_MAX) {
            offset = RAMP_MIN + (offset - RAMP_MAX);
        }
        channels[i] = (uint16_t)constrain(offset, RAMP_MIN, RAMP_MAX);
    }
}

// ============================================================
// Connection state management
// ============================================================
bool isPaired() {
    return security.hasPairingKey() && hasPairedRxId;
}

bool isConnectionReady() {
    return isPaired() && connectionState == STATE_CONNECTED && 
           (lastSyncAckReceived > 0 && (millis() - lastSyncAckReceived < CONNECTION_TIMEOUT_MS));
}

void updateConnectionState() {
    static unsigned long lastStateLog = 0;
    
    // State transitions
    switch (connectionState) {
        case STATE_DISCONNECTED:
            if (pairingMode) {
                connectionState = STATE_PAIRING;
                Serial.println("[STATE] DISCONNECTED -> PAIRING");
            }
            break;
            
        case STATE_PAIRING:
            if (!pairingMode && isPaired()) {
                connectionState = STATE_CONNECTING;
                Serial.println("[STATE] PAIRING -> CONNECTING");
            }
            break;
            
        case STATE_CONNECTING:
            // Wait for sync ACK to complete handshake
            if (lastSyncAckReceived > 0 && (millis() - lastSyncAckReceived < 1000)) {
                connectionState = STATE_CONNECTED;
                connectionStartTime = millis();
                Serial.println("[STATE] CONNECTING -> CONNECTED");
            }
            break;
            
        case STATE_CONNECTED:
            // Stay connected as long as we receive sync ACKs
            break;
            
        case STATE_LOST:
            // Try to reconnect
            if (isPaired()) {
                connectionState = STATE_CONNECTING;
                Serial.println("[STATE] LOST -> CONNECTING (reconnecting)");
            } else {
                connectionState = STATE_DISCONNECTED;
                pairingMode = true;
                Serial.println("[STATE] LOST -> DISCONNECTED (need pairing)");
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
            Serial.print(last_rssi);
            Serial.print(", SNR: ");
            Serial.print(last_snr);
            Serial.print(")");
        }
        Serial.println();
        lastStateLog = millis();
    }
}

void detectConnectionLoss() {
    if (connectionState == STATE_CONNECTED) {
        // Check if we haven't received sync ACK for too long
        unsigned long timeSinceSyncAck = lastSyncAckReceived > 0 ? (millis() - lastSyncAckReceived) : CONNECTION_TIMEOUT_MS;
        
        if (timeSinceSyncAck > CONNECTION_TIMEOUT_MS) {
            connectionState = STATE_LOST;
            Serial.println("[STATE] CONNECTED -> LOST (timeout)");
        }
    }
}

