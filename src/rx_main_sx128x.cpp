// RX Side using SX128x (RadioLib) instead of BLE.
#include <Arduino.h>
#include <RadioLib.h>
#include <string.h>
#include <EEPROM.h>
#include <FastLED.h>

#include "Protocol.h"
#include "Security.h"
#include "crsfSerial.h"
#include "crsf_protocol.h"
#include "crc8.h"
#include "SX128xLink.h"
#include "pairing.h"
#include "PacketHandler.h"
#include "RCConfigProtocol.h"

// CRSF Serial pins (override via build flags if your wiring differs)
#ifndef CRSF_TX_PIN
#define CRSF_TX_PIN 8
#endif
#ifndef CRSF_RX_PIN
#define CRSF_RX_PIN 9
#endif

// WS2812 LED pin
#define WS2812_PIN 13
#define NUM_LEDS 1

static_assert(sizeof(crsf_channels_t) == CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE,
              "crsf_channels_t must be 22 bytes (packed CRSF RC channels payload)");


// Link timeout
const unsigned long TIMEOUT_MS = 1000;
const unsigned long CONNECTION_TIMEOUT_MS = 2000;  // Connection lost detection
static const unsigned long CRSF_CHANNEL_INTERVAL_MS = 6;  // Match radio channel cadence (~167Hz)

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
unsigned long unpairedStartTime = 0;

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

// WS2812 LED state
CRGB leds[NUM_LEDS];
unsigned long lastLedUpdate = 0;
static const unsigned long LED_UPDATE_INTERVAL_MS = 50;  // 20Hz update rate

// Auto pairing timeout (seconds), set via build flag; clamped 0..120
#ifndef AUTO_PAIR_TIMEOUT_SEC
#define AUTO_PAIR_TIMEOUT_SEC 0
#endif
static const unsigned long AUTO_PAIR_TIMEOUT_MS = (AUTO_PAIR_TIMEOUT_SEC > 120 ? 120 : AUTO_PAIR_TIMEOUT_SEC) * 1000UL;

// Forward declarations
void updateConnectionState();
bool isConnectionReady();
bool isPaired();
void detectConnectionLoss();
void hsvToRgb(float h, float s, float v, uint8_t* r, uint8_t* g, uint8_t* b);
void updateLED();
static bool apply_binding_phrase_and_pair(const char* phrase);
static bool rx_proto_set_binding_phrase(const char* phrase, uint8_t len);
static bool rx_proto_set_binding_phrase_unsupported(const char* phrase, uint8_t len);
static bool rx_proto_enter_pairing_mode(void);

static RCConfigProtocol g_rx_proto;
static Crc8 g_crsfWireCrc(0xd5);

static uint16_t usToCrsf(uint16_t us) {
    return (uint16_t)constrain(
        map((long)constrain(us, (uint16_t)1000, (uint16_t)2000),
            1000, 2000,
            CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000),
        (long)CRSF_CHANNEL_VALUE_1000,
        (long)CRSF_CHANNEL_VALUE_2000);
}

static void packCrsfChannelsPayload(uint8_t payload[CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE],
                                    const uint16_t crsfChannels[16]) {
    payload[0]  = (uint8_t)(crsfChannels[0] & 0x07FF);
    payload[1]  = (uint8_t)(((crsfChannels[0] & 0x07FF) >> 8) | ((crsfChannels[1] & 0x07FF) << 3));
    payload[2]  = (uint8_t)(((crsfChannels[1] & 0x07FF) >> 5) | ((crsfChannels[2] & 0x07FF) << 6));
    payload[3]  = (uint8_t)((crsfChannels[2] & 0x07FF) >> 2);
    payload[4]  = (uint8_t)(((crsfChannels[2] & 0x07FF) >> 10) | ((crsfChannels[3] & 0x07FF) << 1));
    payload[5]  = (uint8_t)(((crsfChannels[3] & 0x07FF) >> 7) | ((crsfChannels[4] & 0x07FF) << 4));
    payload[6]  = (uint8_t)(((crsfChannels[4] & 0x07FF) >> 4) | ((crsfChannels[5] & 0x07FF) << 7));
    payload[7]  = (uint8_t)((crsfChannels[5] & 0x07FF) >> 1);
    payload[8]  = (uint8_t)(((crsfChannels[5] & 0x07FF) >> 9) | ((crsfChannels[6] & 0x07FF) << 2));
    payload[9]  = (uint8_t)(((crsfChannels[6] & 0x07FF) >> 6) | ((crsfChannels[7] & 0x07FF) << 5));
    payload[10] = (uint8_t)((crsfChannels[7] & 0x07FF) >> 3);
    payload[11] = (uint8_t)(crsfChannels[8] & 0x07FF);
    payload[12] = (uint8_t)(((crsfChannels[8] & 0x07FF) >> 8) | ((crsfChannels[9] & 0x07FF) << 3));
    payload[13] = (uint8_t)(((crsfChannels[9] & 0x07FF) >> 5) | ((crsfChannels[10] & 0x07FF) << 6));
    payload[14] = (uint8_t)((crsfChannels[10] & 0x07FF) >> 2);
    payload[15] = (uint8_t)(((crsfChannels[10] & 0x07FF) >> 10) | ((crsfChannels[11] & 0x07FF) << 1));
    payload[16] = (uint8_t)(((crsfChannels[11] & 0x07FF) >> 7) | ((crsfChannels[12] & 0x07FF) << 4));
    payload[17] = (uint8_t)(((crsfChannels[12] & 0x07FF) >> 4) | ((crsfChannels[13] & 0x07FF) << 7));
    payload[18] = (uint8_t)((crsfChannels[13] & 0x07FF) >> 1);
    payload[19] = (uint8_t)(((crsfChannels[13] & 0x07FF) >> 9) | ((crsfChannels[14] & 0x07FF) << 2));
    payload[20] = (uint8_t)(((crsfChannels[14] & 0x07FF) >> 6) | ((crsfChannels[15] & 0x07FF) << 5));
    payload[21] = (uint8_t)((crsfChannels[15] & 0x07FF) >> 3);
}

static void sendCrsfChannelsToFlightController(const uint16_t channelsUs[8]) {
    uint16_t crsfChannels[16];
    for (int i = 0; i < 8; i++) {
        crsfChannels[i] = usToCrsf(channelsUs[i]);
    }
    for (int i = 8; i < 16; i++) {
        crsfChannels[i] = CRSF_CHANNEL_VALUE_MID;
    }

    uint8_t frame[CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE + 4];
    frame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    frame[1] = CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE + 2;
    frame[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    packCrsfChannelsPayload(&frame[3], crsfChannels);
    frame[sizeof(frame) - 1] = g_crsfWireCrc.calc(&frame[2], CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE + 1);
    crsf.write(frame, sizeof(frame));
}

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
    Serial.flush();

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

    // Initialize WS2812 LED
    FastLED.addLeds<WS2812, WS2812_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(255);  // Full brightness
    leds[0] = CRGB::Black;  // Start with LED off
    FastLED.show();
    Serial.print("WS2812 LED initialized on GPIO");
    Serial.println(WS2812_PIN);

    Serial.println("=== RX Ready ===");
    Serial.println("Hold BOOTSEL for 5 seconds to enter pairing mode");
    g_rx_proto.setStream(&Serial);
    g_rx_proto.setDeviceInfo("RX-SX128x", "1.1");
    g_rx_proto.setBindingPhraseRx(rx_proto_set_binding_phrase);
    g_rx_proto.setBindingPhraseTx(rx_proto_set_binding_phrase_unsupported);
    g_rx_proto.setEnterPairingMode(rx_proto_enter_pairing_mode);
    Serial.flush();
}

// ============================================================
// Loop
// ============================================================
void loop() {
    g_rx_proto.poll();

    // Update connection state machine
    updateConnectionState();
    
    // Process pairing button (log presses when not paired)
    Pairing::checkBootselButton(&pairingMode, &pairingModeStartTime, hasPairedTxId);
    Pairing::updatePairingMode(&pairingMode, pairingModeStartTime);

    // Auto-enter pairing if not paired within timeout
    if (!hasPairedTxId && !pairingMode && AUTO_PAIR_TIMEOUT_MS > 0) {
        if (unpairedStartTime == 0) {
            unpairedStartTime = millis();
        }
        if (millis() - unpairedStartTime > AUTO_PAIR_TIMEOUT_MS) {
            Serial.println("[PAIR] Auto-entering pairing mode (timeout reached)");
            Pairing::enterPairingMode(&pairingMode, &pairingModeStartTime);
            unpairedStartTime = millis();
        }
    } else if (hasPairedTxId) {
        unpairedStartTime = millis();
    }

    // Periodically log BOOTSEL state when not paired (visibility/debug)
    static unsigned long lastBootselLog = 0;
    if (!hasPairedTxId && (millis() - lastBootselLog > 1000)) {
        bool pressed = Pairing::getBootselButton();
        Serial.print("[PAIR] BOOTSEL state: ");
        Serial.println(pressed ? "PRESSED" : "released");
        lastBootselLog = millis();
    }

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
                // Handle channel data (decode THIS packet's payload).
                if (hasPairedTxId && (connectionState == STATE_CONNECTED || connectionState == STATE_CONNECTING)) {
                    if (payload_len >= FRAME_SIZE) {
                        uint16_t sequence = 0;
                        int decodeFail = 0;
                        if (Protocol::decodeFrame(payload, channels, &sequence, &security, &lastSequence, pairedTxDeviceId, &decodeFail)) {
                            lastDataReceived = millis();

                            static unsigned long lastChanLog = 0;
                            if (millis() - lastChanLog > 1000) {
                                Serial.print("[RX] Channels: ");
                                for (int i = 0; i < 8; i++) {
                                    Serial.print(channels[i]);
                                    Serial.print(" ");
                                }
                                Serial.print(" Seq: ");
                                Serial.print(sequence);
                                Serial.print(" RSSI: ");
                                Serial.print(lastRSSI);
                                Serial.print(" SNR: ");
                                Serial.println(lastSNR);
                                lastChanLog = millis();
                            }
                        } else {
                            static unsigned long lastBadLog = 0;
                            if (millis() - lastBadLog > 2000) {
                                const char* reason = (decodeFail == 1) ? "device_id" : (decodeFail == 2) ? "hmac" : (decodeFail == 3) ? "decrypt" : (decodeFail == 4) ? "channel_range" : (decodeFail == 5) ? "replay" : "unknown";
                                Serial.print("[RX] Invalid channel frame: ");
                                Serial.println(reason);
                                lastBadLog = millis();
                            }
                        }
                    } else {
                        static unsigned long lastLenLog = 0;
                        if (millis() - lastLenLog > 2000) {
                            Serial.print("[RX] MSG_CHANNELS wrong payload len=");
                            Serial.println(payload_len);
                            lastLenLog = millis();
                        }
                    }
                }
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
                // Debug visibility: confirm we are actually seeing sync packets and their lengths.
                {
                    static unsigned long lastSyncSeenLog = 0;
                    if (millis() - lastSyncSeenLog > 2000) {
                        lastSyncSeenLog = millis();
                        Serial.print("[SYNC] RX saw MSG_SYNC pktLen=");
                        Serial.print(pkt.length);
                        Serial.print(" payloadLen=");
                        Serial.print(payload_len);
                        if (payload_len > 0) {
                            Serial.print(" payload0=0x");
                            Serial.print(payload[0], HEX);
                        }
                        Serial.println();
                    }
                }
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
            case MSG_BINDING_PHRASE_SET:
                if (payload_len >= 2) {
                    uint8_t phraseLen = payload[0];
                    if (phraseLen > 0 && phraseLen <= 32 && payload_len >= (size_t)(1 + phraseLen)) {
                        char phrase[33];
                        memcpy(phrase, payload + 1, phraseLen);
                        phrase[phraseLen] = '\0';
                        if (apply_binding_phrase_and_pair(phrase)) {
                            Serial.println("[RX] Binding phrase updated; pairing state reset");
                            uint8_t ackPacket[1] = {MSG_BINDING_PHRASE_ACK};
                            radio.send(ackPacket, sizeof(ackPacket));
                        } else {
                            Serial.println("[RX] Failed to set binding phrase");
                        }
                    }
                }
                break;
            case MSG_BINDING_PHRASE_ACK:
                // ACK is consumed on TX side.
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
        // for (int i = 0; i < 8; i++) {
        //     channels[i] = 1500;
        // }
    }

    // Process CRSF (receives telemetry from FC on GP9)
    crsf.loop();

    // Send battery telemetry to TX over radio (rate-limited)
    PacketHandler::sendBatteryTelemetry(&radio, batteryVoltage, batteryCurrent,
                                         batteryRemaining, batteryDataValid,
                                         &lastBatteryTxTime);

    // Send RC channels to the flight controller at the radio cadence.
    static unsigned long lastCRSF = 0;
    if (millis() - lastCRSF >= CRSF_CHANNEL_INTERVAL_MS) {
        lastCRSF = millis();
        if (isConnectionReady()) {
            sendCrsfChannelsToFlightController(channels);
        }

        // Debug: confirm we're actually writing CRSF frames out the UART.
        static uint32_t crsfTxCount = 0;
        static unsigned long lastCrsfLog = 0;
        if (isConnectionReady()) {
            crsfTxCount++;
        }
        if (millis() - lastCrsfLog > 2000) {
            lastCrsfLog = millis();
            Serial.print("[CRSF OUT] sent RC frame #");
            Serial.print(crsfTxCount);
            Serial.print(" ch0=");
            Serial.print(channels[0]);
            Serial.print(" ch1=");
            Serial.print(channels[1]);
            Serial.print(" ready=");
            Serial.print(isConnectionReady() ? "Y" : "N");
            Serial.print(" txPin=GP");
            Serial.print(CRSF_TX_PIN);
            Serial.print(" rxPin=GP");
            Serial.println(CRSF_RX_PIN);
        }
    }

    // Update LED color based on channels 7 & 8
    updateLED();

    // Flush logs periodically to ensure output before potential crash
    static unsigned long lastFlush = 0;
    if (millis() - lastFlush > 500) {
        Serial.flush();
        lastFlush = millis();
    }
}

static bool apply_binding_phrase_and_pair(const char* phrase) {
    if (!phrase || !security.setBindingPhrase(phrase)) return false;
    hasPairedTxId = false;
    connectionState = STATE_PAIRING;
    Pairing::enterPairingMode(&pairingMode, &pairingModeStartTime);
    unpairedStartTime = millis();
    return true;
}

static bool rx_proto_set_binding_phrase(const char* phrase, uint8_t len) {
    if (!phrase || len == 0 || len > 32) return false;
    return apply_binding_phrase_and_pair(phrase);
}

static bool rx_proto_set_binding_phrase_unsupported(const char* phrase, uint8_t len) {
    (void)phrase;
    (void)len;
    return false;
}

static bool rx_proto_enter_pairing_mode(void) {
    connectionState = STATE_PAIRING;
    Pairing::enterPairingMode(&pairingMode, &pairingModeStartTime);
    return true;
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
            // Consider link "connected" once we see either sync OR valid channel data recently.
            // This avoids stalling control traffic on sync timing.
            if ((lastSyncReceived > 0 && (millis() - lastSyncReceived < 1000)) ||
                (lastDataReceived > 0 && (millis() - lastDataReceived < 1000))) {
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

// ============================================================
// WS2812 LED Control
// ============================================================
// Convert HSV to RGB
// h: 0-360 (hue)
// s: 0-100 (saturation %)
// v: 0-100 (value/brightness %)
void hsvToRgb(float h, float s, float v, uint8_t* r, uint8_t* g, uint8_t* b) {
    // Normalize inputs
    h = fmod(h, 360.0f);
    if (h < 0) h += 360.0f;
    s = constrain(s, 0.0f, 100.0f) / 100.0f;
    v = constrain(v, 0.0f, 100.0f) / 100.0f;
    
    float c = v * s;
    float x = c * (1.0f - fabs(fmod(h / 60.0f, 2.0f) - 1.0f));
    float m = v - c;
    
    float r1, g1, b1;
    
    if (h < 60) {
        r1 = c; g1 = x; b1 = 0;
    } else if (h < 120) {
        r1 = x; g1 = c; b1 = 0;
    } else if (h < 180) {
        r1 = 0; g1 = c; b1 = x;
    } else if (h < 240) {
        r1 = 0; g1 = x; b1 = c;
    } else if (h < 300) {
        r1 = x; g1 = 0; b1 = c;
    } else {
        r1 = c; g1 = 0; b1 = x;
    }
    
    *r = (uint8_t)((r1 + m) * 255.0f);
    *g = (uint8_t)((g1 + m) * 255.0f);
    *b = (uint8_t)((b1 + m) * 255.0f);
}

// Update LED color based on channels 7 & 8
// Channel 7 (channels[6]): 1000-2000 → Hue 0-360°
// Channel 8 (channels[7]): 1000-1500 → Saturation 0-100%, 1500-2000 → Value 0-100%
void updateLED() {
    // Rate limit updates
    if (millis() - lastLedUpdate < LED_UPDATE_INTERVAL_MS) {
        return;
    }
    lastLedUpdate = millis();
    
    // Only update LED if connected and receiving data
    if (connectionState != STATE_CONNECTED && connectionState != STATE_CONNECTING) {
        // Disconnected: show dim red
        leds[0] = CRGB(20, 0, 0);
        FastLED.show();
        return;
    }
    
    // Map channel 7 (channels[6]) to Hue: 1000-2000 → 0-360°
    float hue = (float)map(channels[6], 1000, 2000, 0, 360);
    
    // Map channel 8 (channels[7]) to Saturation and Value
    // 1000-1500 → Saturation 0-100%
    // 1500-2000 → Value 0-100%
    float saturation, value;
    
    if (channels[7] <= 1500) {
        // Lower half: control saturation
        saturation = (float)map(channels[7], 1000, 1500, 0, 100);
        value = 100.0f;  // Full brightness
    } else {
        // Upper half: control value/brightness
        saturation = 100.0f;  // Full saturation
        value = (float)map(channels[7], 1500, 2000, 0, 100);
    }
    
    // Convert HSV to RGB
    uint8_t r, g, b;
    hsvToRgb(hue, saturation, value, &r, &g, &b);
    
    // Update LED
    leds[0] = CRGB(r, g, b);
    FastLED.show();
    
    // Debug log (rate limited)
    static unsigned long lastLedLog = 0;
    if (millis() - lastLedLog > 2000) {
        Serial.print("[LED] Ch7=");
        Serial.print(channels[6]);
        Serial.print(" Ch8=");
        Serial.print(channels[7]);
        Serial.print(" → HSV(");
        Serial.print(hue, 1);
        Serial.print(",");
        Serial.print(saturation, 1);
        Serial.print(",");
        Serial.print(value, 1);
        Serial.print(") → RGB(");
        Serial.print(r);
        Serial.print(",");
        Serial.print(g);
        Serial.print(",");
        Serial.print(b);
        Serial.println(")");
        lastLedLog = millis();
    }
}


