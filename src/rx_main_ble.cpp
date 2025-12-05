// RX Side - BLE Peripheral using BTstackLib for GATT
// Uses BTstackLib for GATT server (which works), paired with low-level TX client

#include <Arduino.h>
#include <string.h>
#include <EEPROM.h>
#include "Protocol.h"
#include "Security.h"
#include "crsfSerial.h"
#include "crsf_protocol.h"

// Use BTstackLib for GATT server
#include <BTstackLib.h>

// CRSF Serial pins
#define CRSF_TX_PIN 8
#define CRSF_RX_PIN 9

// Custom GATT UUIDs (16-bit)
#define FPV_SERVICE_UUID16     0xFF00
#define FPV_TX_CHAR_UUID16     0xFF01  // Write characteristic
#define FPV_RX_CHAR_UUID16     0xFF02  // Notify characteristic (future)

// UUID objects for BTstackLib
static UUID fpvServiceUUID("0000FF00-0000-1000-8000-00805F9B34FB");
static UUID txCharUUID("0000FF01-0000-1000-8000-00805F9B34FB");
static UUID rxCharUUID("0000FF02-0000-1000-8000-00805F9B34FB");

// Application state
CrsfSerial crsf(Serial2, CRSF_BAUDRATE);
Security security;

// Channel values received from TX
// Default to minimum (1000) for failsafe behavior
uint16_t channels[8] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};

// Connection state
bool clientConnected = false;
unsigned long lastDataReceived = 0;
const unsigned long TIMEOUT_MS = 1000;

// Security state
uint16_t lastSequence = 0;

// BTstackLib GATT write callback
int gattWriteCallback(uint16_t characteristic_id, uint8_t *buffer, uint16_t buffer_size) {
    Serial.print("[GATT Write] char_id: ");
    Serial.print(characteristic_id);
    Serial.print(", size: ");
    Serial.println(buffer_size);
    
    // Process channel data
    if (buffer_size == FRAME_SIZE) {
        uint16_t sequence = 0;
        if (Protocol::decodeFrame(buffer, channels, &sequence, &security, &lastSequence)) {
            lastDataReceived = millis();
            
            static unsigned long lastPrint = 0;
            if (millis() - lastPrint > 1000) {
                Serial.print("Channels: ");
                for (int i = 0; i < 8; i++) {
                    Serial.print(channels[i]);
                    Serial.print(" ");
                }
                Serial.print(" Seq: ");
                Serial.println(sequence);
                lastPrint = millis();
            }
            return 0;
        } else {
            Serial.println("Invalid frame (security check failed)");
            return 1;
        }
    }
    
    // Log any other data received
    Serial.print("Data: ");
    for (int i = 0; i < min((int)buffer_size, 16); i++) {
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    return 0;
}

// BTstackLib device connected callback
void deviceConnectedCallback(BLEStatus status, BLEDevice *device) {
    (void)device;
    if (status == BLE_STATUS_OK) {
        clientConnected = true;
        Serial.println("[BLE] Client connected!");
    }
}

// BTstackLib device disconnected callback
void deviceDisconnectedCallback(BLEDevice *device) {
    (void)device;
    clientConnected = false;
    Serial.println("[BLE] Client disconnected!");
    
    // Failsafe - set all channels to minimum (1000)
    for (int i = 0; i < 8; i++) {
        channels[i] = 1000;
    }
}

void setup() {
    Serial.begin(115200);
    
    unsigned long start = millis();
    while (!Serial && (millis() - start < 2000)) {
        delay(10);
    }
    delay(500);
    
    Serial.println("=== FPV Receiver (BLE) ===");
    Serial.println("Initializing...");
    
    // Initialize security
    if (security.begin()) {
        Serial.println("Security initialized");
    } else {
        Serial.println("Security initialization failed!");
    }
    
    // Set BTstackLib callbacks BEFORE setup
    Serial.println("Setting up BTstackLib callbacks...");
    BTstack.setBLEDeviceConnectedCallback(deviceConnectedCallback);
    BTstack.setBLEDeviceDisconnectedCallback(deviceDisconnectedCallback);
    BTstack.setGATTCharacteristicWrite(gattWriteCallback);
    
    // Add GATT service and characteristics BEFORE setup
    Serial.println("Adding GATT service...");
    BTstack.addGATTService(&fpvServiceUUID);
    
    // Add TX characteristic (Write + Write Without Response)
    // Properties: 0x0C = Write Without Response (0x04) + Write (0x08)
    Serial.println("Adding TX characteristic (Write)...");
    uint16_t tx_char_id = BTstack.addGATTCharacteristicDynamic(&txCharUUID, ATT_PROPERTY_WRITE | ATT_PROPERTY_WRITE_WITHOUT_RESPONSE, 0);
    Serial.print("TX char ID: ");
    Serial.println(tx_char_id);
    
    // Add RX characteristic (Notify) for future use
    Serial.println("Adding RX characteristic (Notify)...");
    uint16_t rx_char_id = BTstack.addGATTCharacteristicDynamic(&rxCharUUID, ATT_PROPERTY_NOTIFY, 0);
    Serial.print("RX char ID: ");
    Serial.println(rx_char_id);
    
    // Initialize BTstack
    Serial.println("Calling BTstack.setup()...");
    BTstack.setup();
    Serial.println("BTstack initialized");
    
    // Start advertising
    Serial.println("Starting advertising...");
    BTstack.startAdvertising();
    
    // Initialize CRSF
    Serial.println("Initializing CRSF...");
    Serial2.setTX(CRSF_TX_PIN);
    Serial2.setRX(CRSF_RX_PIN);
    crsf.begin(CRSF_BAUDRATE);
    
    Serial.print("CRSF on Serial2: TX=GP");
    Serial.print(CRSF_TX_PIN);
    Serial.print(", RX=GP");
    Serial.println(CRSF_RX_PIN);
    
    Serial.println("=== RX Ready ===");
    Serial.println("Waiting for connections...");
}

void loop() {
    // Process BTstack
    BTstack.loop();
    
    // Heartbeat
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 5000) {
        lastHeartbeat = millis();
        Serial.print("RX alive, connected: ");
        Serial.print(clientConnected ? "Yes" : "No");
        Serial.print(", last data: ");
        if (lastDataReceived > 0) {
            Serial.print(millis() - lastDataReceived);
            Serial.println("ms ago");
        } else {
            Serial.println("never");
        }
    }
    
    // Check for timeout
    if (clientConnected && lastDataReceived > 0 && (millis() - lastDataReceived > TIMEOUT_MS)) {
        static unsigned long lastTimeout = 0;
        if (millis() - lastTimeout > 5000) {
            Serial.println("*** TIMEOUT ***");
            lastTimeout = millis();
        }
        for (int i = 0; i < 8; i++) {
            channels[i] = 1000;
        }
    }
    
    // Process CRSF
    crsf.loop();
    
    // Send CRSF at 50Hz
    static unsigned long lastCRSF = 0;
    if (millis() - lastCRSF >= 20) {
        lastCRSF = millis();
        
        crsf_channels_t crsfChannels = {0};
        // Swap A and T: ch0 (Aileron) gets Throttle, ch2 (Throttle) gets Aileron
        crsfChannels.ch0 = map(channels[2], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);  // Throttle -> Aileron
        crsfChannels.ch1 = map(channels[1], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);  // Elevator
        crsfChannels.ch2 = map(channels[0], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);  // Aileron -> Throttle
        crsfChannels.ch3 = map(channels[3], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);  // Rudder
        crsfChannels.ch4 = map(channels[4], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
        crsfChannels.ch5 = map(channels[5], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
        crsfChannels.ch6 = map(channels[6], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
        crsfChannels.ch7 = map(channels[7], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
        
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
    
    delay(1);
}
