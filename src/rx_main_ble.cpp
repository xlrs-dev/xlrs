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
// Low-level BTstack (for notifications)
extern "C" {
#include <btstack.h>
}

// Low-level includes for BOOTSEL button
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"

// Pico W LED control (LED is on WiFi chip, not GPIO)
#include "pico/cyw43_arch.h"

// CRSF Serial pins
#define CRSF_TX_PIN 8
#define CRSF_RX_PIN 9

// LED control for Pico W
// The onboard LED on Pico W is connected to the CYW43 WiFi chip
// We use cyw43_arch_gpio_put() instead of digitalWrite()
#define PICO_W_LED_PIN CYW43_WL_GPIO_LED_PIN

// Custom GATT UUIDs (16-bit)
#define FPV_SERVICE_UUID16     0xFF00
#define FPV_TX_CHAR_UUID16     0xFF01  // Write characteristic (channel data)
#define FPV_RX_CHAR_UUID16     0xFF02  // Notify characteristic (future)
#define FPV_PAIR_CHAR_UUID16   0xFF03  // Pairing key characteristic (write)
#define FPV_TIME_CHAR_UUID16   0xFF04  // Time sync characteristic (write)

// UUID objects for BTstackLib
static UUID fpvServiceUUID("0000FF00-0000-1000-8000-00805F9B34FB");
static UUID txCharUUID("0000FF01-0000-1000-8000-00805F9B34FB");
static UUID rxCharUUID("0000FF02-0000-1000-8000-00805F9B34FB");
static UUID pairCharUUID("0000FF03-0000-1000-8000-00805F9B34FB");
static UUID timeCharUUID("0000FF04-0000-1000-8000-00805F9B34FB");

// Pairing mode configuration
#define PAIRING_MODE_TIMEOUT_MS 60000   // 60 seconds pairing window
#define BOOTSEL_HOLD_TIME_MS    5000    // 5 seconds to enter pairing mode
#define PAIRING_KEY_SIZE        16

// Pairing mode state
bool pairingMode = false;
unsigned long pairingModeStartTime = 0;
uint16_t pairCharId = 0;
uint16_t rxCharId = 0;  // RX characteristic for telemetry notifications
uint16_t timeCharId = 0; // Time sync characteristic for RX clock alignment
static hci_con_handle_t att_conn_handle = HCI_CON_HANDLE_INVALID;  // BLE connection handle for notifications

// Battery telemetry state
float batteryVoltage = 0.0f;
float batteryCurrent = 0.0f;
uint32_t batteryCapacity = 0;
uint8_t batteryRemaining = 0;
bool batteryDataValid = false;
unsigned long lastBatteryUpdate = 0;
unsigned long lastBatteryTxTime = 0;
const unsigned long BATTERY_TX_INTERVAL_MS = 10000;  // Send battery telemetry every 10 seconds

// Link statistics telemetry state
crsfLinkStatistics_t lastLinkStats = {0};
unsigned long lastLinkStatsLog = 0;

// Application state
CrsfSerial crsf(Serial2, CRSF_BAUDRATE);
Security security;

// Channel values received from TX
// Default to center (1500) for failsafe behavior
uint16_t channels[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

// Connection state
bool clientConnected = false;
unsigned long lastDataReceived = 0;      // Last fresh packet applied (ms, RX clock)
unsigned long lastFrameArrivalMs = 0;    // Last frame arrival (ms, RX clock)
uint32_t lastRxAlignedMs = 0;            // Last TX timestamp aligned to RX clock
uint32_t lastTxTimestampMs = 0;          // Raw TX timestamp from last frame
uint32_t lastPacketAgeMs = 0;            // Age of most recent frame at reception
int32_t txToRxOffsetMs = 0;              // RX clock - TX clock (ms)
bool timeSynced = false;
const uint32_t PACKET_STALE_THRESHOLD_MS = 150;
const uint32_t PACKET_FAILSAFE_THRESHOLD_MS = 600;
const uint16_t FAILSAFE_CENTER_US = CHANNEL_CENTER;
const uint16_t FAILSAFE_RAMP_STEP_US = 10;
unsigned long lastFailsafeLogMs = 0;
unsigned long lastStaleLogMs = 0;

// Security state
uint16_t lastSequence = 0;

// Helper to clamp signed age calculations to unsigned milliseconds
static uint32_t clampAgeToMs(int32_t ageMs) {
    return (ageMs < 0) ? 0u : static_cast<uint32_t>(ageMs);
}

// Gradually move roll/pitch/yaw back to center for failsafe
static void rampChannelsToFailsafe() {
    auto ramp = [](uint16_t& value, uint16_t target) {
        if (value < target) {
            uint16_t delta = target - value;
            value += (delta < FAILSAFE_RAMP_STEP_US) ? delta : FAILSAFE_RAMP_STEP_US;
        } else if (value > target) {
            uint16_t delta = value - target;
            value -= (delta < FAILSAFE_RAMP_STEP_US) ? delta : FAILSAFE_RAMP_STEP_US;
        }
    };
    
    ramp(channels[0], FAILSAFE_CENTER_US); // Roll
    ramp(channels[1], FAILSAFE_CENTER_US); // Pitch
    ramp(channels[2], FAILSAFE_CENTER_US); // Yaw
}

// ============================================================
// LED Control for Pico W
// ============================================================
// The Pico W LED is connected to the CYW43 WiFi chip, not a GPIO
// Must use cyw43_arch_gpio_put() instead of digitalWrite()
void setLED(bool state) {
    cyw43_arch_gpio_put(PICO_W_LED_PIN, state ? 1 : 0);
}

// ============================================================
// BOOTSEL Button Reading (RP2040-specific)
// ============================================================
// The BOOTSEL button is on the QSPI chip select pin, not a normal GPIO
// This function safely reads its state
bool __no_inline_not_in_flash_func(get_bootsel_button)() {
    const uint CS_PIN_INDEX = 1;
    
    // Must disable interrupts, as interrupt handlers may be in flash
    uint32_t flags = save_and_disable_interrupts();
    
    // Set chip select to Hi-Z
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                   GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                   IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);
    
    // Brief delay for pin to settle
    for (volatile int i = 0; i < 1000; ++i);
    
    // Read the pin state (low = pressed)
    bool button_state = !(sio_hw->gpio_hi_in & (1u << CS_PIN_INDEX));
    
    // Restore chip select to normal operation
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                   GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                   IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);
    
    restore_interrupts(flags);
    
    return button_state;
}

// ============================================================
// Pairing Mode Functions
// ============================================================
void enterPairingMode() {
    pairingMode = true;
    pairingModeStartTime = millis();
    
    Serial.println("*** PAIRING MODE ACTIVE ***");
    Serial.println("Waiting for TX to send pairing key...");
    Serial.print("Timeout in ");
    Serial.print(PAIRING_MODE_TIMEOUT_MS / 1000);
    Serial.println(" seconds");
    
    // Fast LED blink to indicate pairing mode
    setLED(true);
}

void exitPairingMode(bool success) {
    pairingMode = false;
    
    if (success) {
        Serial.println("*** PAIRING SUCCESSFUL ***");
        // Solid LED briefly then off
        setLED(true);
        delay(500);
        setLED(false);
    } else {
        Serial.println("*** PAIRING MODE TIMEOUT ***");
        setLED(false);
    }
}

void handlePairingKeyReceived(uint8_t* key, uint16_t keyLen) {
    if (keyLen != PAIRING_KEY_SIZE) {
        Serial.print("Invalid pairing key size: ");
        Serial.println(keyLen);
        return;
    }
    
    Serial.println("Received pairing key from TX:");
    Serial.print("  Key: ");
    for (int i = 0; i < PAIRING_KEY_SIZE; i++) {
        if (key[i] < 0x10) Serial.print("0");
        Serial.print(key[i], HEX);
    }
    Serial.println();
    
    // Store the new pairing key
    if (security.setPairingKey(key)) {
        Serial.println("Pairing key saved to EEPROM");
        exitPairingMode(true);
        
        // Reset sequence number for new pairing
        lastSequence = 0;
    } else {
        Serial.println("ERROR: Failed to save pairing key!");
    }
}

void checkBootselButton() {
    // Rate-limit button checking to every 50ms (doesn't need to be faster)
    // get_bootsel_button() disables interrupts briefly, so we minimize calls
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck < 50) return;
    lastCheck = millis();
    
    static unsigned long buttonPressStart = 0;
    static bool buttonWasPressed = false;
    static bool lastLedState = false;
    
    bool buttonPressed = get_bootsel_button();
    
    if (buttonPressed && !buttonWasPressed) {
        // Button just pressed
        buttonPressStart = millis();
        buttonWasPressed = true;
    } else if (buttonPressed && buttonWasPressed) {
        // Button held - check duration
        unsigned long holdTime = millis() - buttonPressStart;
        
        // Visual feedback while holding (only update LED when state changes)
        if (holdTime > 1000) {
            bool newLedState = (millis() / 200) % 2;
            if (newLedState != lastLedState) {
                setLED(newLedState);
                lastLedState = newLedState;
            }
        }
        
        if (holdTime >= BOOTSEL_HOLD_TIME_MS && !pairingMode) {
            enterPairingMode();
        }
    } else if (!buttonPressed && buttonWasPressed) {
        // Button released
        buttonWasPressed = false;
        if (!pairingMode && lastLedState) {
            setLED(false);
            lastLedState = false;
        }
    }
}

void updatePairingMode() {
    if (!pairingMode) return;
    
    // Check timeout
    if (millis() - pairingModeStartTime > PAIRING_MODE_TIMEOUT_MS) {
        exitPairingMode(false);
        return;
    }
    
    // Fast blink LED in pairing mode (only update when state changes)
    static bool lastPairingLedState = false;
    bool newLedState = (millis() / 100) % 2;
    if (newLedState != lastPairingLedState) {
        setLED(newLedState);
        lastPairingLedState = newLedState;
    }
}

// BTstackLib GATT write callback
int gattWriteCallback(uint16_t characteristic_id, uint8_t *buffer, uint16_t buffer_size) {
    Serial.print("[GATT Write] char_id: ");
    Serial.print(characteristic_id);
    Serial.print(", size: ");
    Serial.println(buffer_size);
    
    // Check if this is a pairing key write (characteristic ID for pairing)
    if (characteristic_id == pairCharId && buffer_size == PAIRING_KEY_SIZE) {
        Serial.println("Pairing key write received");
        
        if (pairingMode) {
            handlePairingKeyReceived(buffer, buffer_size);
            return 0;
        } else {
            Serial.println("WARNING: Pairing key rejected - not in pairing mode!");
            Serial.println("Hold BOOTSEL for 5 seconds to enter pairing mode");
            return 1;  // Reject if not in pairing mode
        }
    }
    
    // Handle time sync writes (TX sends its millis() as big-endian uint32)
    if (characteristic_id == timeCharId) {
        if (buffer_size != 4) {
            Serial.print("Unexpected time sync size: ");
            Serial.println(buffer_size);
            return 1;
        }
        
        uint32_t txTimestampMs =
            (static_cast<uint32_t>(buffer[0]) << 24) |
            (static_cast<uint32_t>(buffer[1]) << 16) |
            (static_cast<uint32_t>(buffer[2]) << 8) |
            static_cast<uint32_t>(buffer[3]);
        
        unsigned long now = millis();
        txToRxOffsetMs = static_cast<int32_t>(now) - static_cast<int32_t>(txTimestampMs);
        timeSynced = true;
        lastTxTimestampMs = txTimestampMs;
        lastRxAlignedMs = static_cast<uint32_t>(static_cast<int32_t>(txTimestampMs) + txToRxOffsetMs);
        lastPacketAgeMs = clampAgeToMs(static_cast<int32_t>(now) - static_cast<int32_t>(lastRxAlignedMs));
        lastFrameArrivalMs = now;
        
        Serial.print("Time sync received. Offset (rx-tx): ");
        Serial.print(txToRxOffsetMs);
        Serial.println(" ms");
        return 0;
    }
    
    // Process channel data
    if (buffer_size == FRAME_SIZE) {
        uint16_t sequence = 0;
        uint32_t txTimestampMs = 0;
        uint16_t decodedChannels[NUM_CHANNELS] = {0};
        if (Protocol::decodeFrame(buffer, decodedChannels, &sequence, &txTimestampMs, &security, &lastSequence)) {
            unsigned long now = millis();
            lastFrameArrivalMs = now;
            
            uint32_t alignedRxTimeMs = timeSynced
                ? static_cast<uint32_t>(static_cast<int32_t>(txTimestampMs) + txToRxOffsetMs)
                : now;
            uint32_t ageMs = clampAgeToMs(static_cast<int32_t>(now) - static_cast<int32_t>(alignedRxTimeMs));
            
            lastTxTimestampMs = txTimestampMs;
            lastRxAlignedMs = alignedRxTimeMs;
            lastPacketAgeMs = ageMs;
            
            if (ageMs <= PACKET_STALE_THRESHOLD_MS) {
                // Fresh packet - apply channels
                memcpy(channels, decodedChannels, sizeof(channels));
                lastDataReceived = now;
            } else {
                // Stale packet - hold last channels
                if (now - lastStaleLogMs > 1000) {
                    Serial.print("Stale packet ignored, age=");
                    Serial.print(ageMs);
                    Serial.println("ms (holding last targets)");
                    lastStaleLogMs = now;
                }
            }
            
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
        att_conn_handle = device ? device->getHandle() : HCI_CON_HANDLE_INVALID;
        Serial.println("[BLE] Client connected!");
    }
}

// BTstackLib device disconnected callback
void deviceDisconnectedCallback(BLEDevice *device) {
    (void)device;
    clientConnected = false;
    att_conn_handle = HCI_CON_HANDLE_INVALID;
    Serial.println("[BLE] Client disconnected!");
    
    // Reset timing/timesync state
    timeSynced = false;
    txToRxOffsetMs = 0;
    lastRxAlignedMs = 0;
    lastTxTimestampMs = 0;
    lastPacketAgeMs = 0;
    lastDataReceived = 0;
    lastFrameArrivalMs = 0;
    
    // Failsafe - set all channels to center (1500)
    for (int i = 0; i < 8; i++) {
        channels[i] = 1500;
    }
}

// ============================================================
// Battery Telemetry
// ============================================================
// Callback when battery telemetry is received from flight controller via CRSF
void onBatteryTelemetry(float voltage, float current, uint32_t capacity, uint8_t remaining) {
    batteryVoltage = voltage;
    batteryCurrent = current;
    batteryCapacity = capacity;
    batteryRemaining = remaining;
    batteryDataValid = true;
    lastBatteryUpdate = millis();
    
    // Debug log occasionally
    static unsigned long lastLog = 0;
    if (millis() - lastLog > 10000) {
        Serial.print("[BATTERY] V:");
        Serial.print(voltage, 1);
        Serial.print(" A:");
        Serial.print(current, 1);
        Serial.print(" mAh:");
        Serial.print(capacity);
        Serial.print(" %:");
        Serial.println(remaining);
        lastLog = millis();
    }
}

// Link statistics callback (optional telemetry insight)
void onLinkStats(crsfLinkStatistics_t *ls) {
    lastLinkStats = *ls;
    unsigned long now = millis();
    if (now - lastLinkStatsLog > 2000) { // log every 2s max
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
        Serial.print("%");
        Serial.println();
        lastLinkStatsLog = now;
    }
}

// Send battery telemetry over BLE to TX
// Format: [voltage_x10 (16-bit BE), current_x10 (16-bit BE), remaining (8-bit)]
// Total: 5 bytes (keeping it small for BLE efficiency)
void sendBatteryTelemetry() {
    if (!clientConnected || !batteryDataValid || rxCharId == 0) {
        return;
    }
    
    // Rate limit to every 10 seconds
    if (millis() - lastBatteryTxTime < BATTERY_TX_INTERVAL_MS) {
        return;
    }
    lastBatteryTxTime = millis();
    
    // Pack battery data: voltage*10 (16-bit), current*10 (16-bit), remaining (8-bit)
    uint8_t telemetryData[5];
    uint16_t voltage_x10 = (uint16_t)(batteryVoltage * 10);
    uint16_t current_x10 = (uint16_t)(batteryCurrent * 10);
    
    // Big-endian encoding
    telemetryData[0] = (voltage_x10 >> 8) & 0xFF;
    telemetryData[1] = voltage_x10 & 0xFF;
    telemetryData[2] = (current_x10 >> 8) & 0xFF;
    telemetryData[3] = current_x10 & 0xFF;
    telemetryData[4] = batteryRemaining;
    
    // Send notification via low-level BTstack (server notify)
    if (att_conn_handle == HCI_CON_HANDLE_INVALID) {
        return;
    }
    int result = att_server_notify(att_conn_handle, rxCharId, telemetryData, sizeof(telemetryData));
    
    if (result == 0) {
        Serial.print("[BLE TELEM] Sent battery: ");
        Serial.print(batteryVoltage, 1);
        Serial.print("V ");
        Serial.print(batteryRemaining);
        Serial.println("%");
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
    
    // Note: LED on Pico W is controlled via CYW43 WiFi chip
    // It will be initialized after BTstack.setup() below
    
    // Initialize security
    if (security.begin()) {
        Serial.println("Security initialized");
        
        // Display current pairing key (for debugging)
        uint8_t currentKey[PAIRING_KEY_SIZE];
        if (security.getPairingKey(currentKey)) {
            Serial.print("Current pairing key: ");
            for (int i = 0; i < PAIRING_KEY_SIZE; i++) {
                if (currentKey[i] < 0x10) Serial.print("0");
                Serial.print(currentKey[i], HEX);
            }
            Serial.println();
        }
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
    
    // Add TX characteristic (Write + Write Without Response) - for channel data
    // Properties: 0x0C = Write Without Response (0x04) + Write (0x08)
    Serial.println("Adding TX characteristic (Write)...");
    uint16_t tx_char_id = BTstack.addGATTCharacteristicDynamic(&txCharUUID, ATT_PROPERTY_WRITE | ATT_PROPERTY_WRITE_WITHOUT_RESPONSE, 0);
    Serial.print("TX char ID: ");
    Serial.println(tx_char_id);
    
    // Add RX characteristic (Notify) for battery telemetry to TX
    Serial.println("Adding RX characteristic (Notify) for telemetry...");
    rxCharId = BTstack.addGATTCharacteristicDynamic(&rxCharUUID, ATT_PROPERTY_NOTIFY | ATT_PROPERTY_READ, 0);
    Serial.print("RX char ID: ");
    Serial.println(rxCharId);
    
    // Add Pairing characteristic (Write) - for receiving pairing key from TX
    Serial.println("Adding Pairing characteristic (Write)...");
    pairCharId = BTstack.addGATTCharacteristicDynamic(&pairCharUUID, ATT_PROPERTY_WRITE, 0);
    Serial.print("Pair char ID: ");
    Serial.println(pairCharId);
    
    // Add Time Sync characteristic (Write) so TX can align clocks
    Serial.println("Adding Time Sync characteristic (Write)...");
    timeCharId = BTstack.addGATTCharacteristicDynamic(&timeCharUUID, ATT_PROPERTY_WRITE | ATT_PROPERTY_WRITE_WITHOUT_RESPONSE, 0);
    Serial.print("Time char ID: ");
    Serial.println(timeCharId);
    
    // Initialize BTstack
    Serial.println("Calling BTstack.setup()...");
    BTstack.setup();
    Serial.println("BTstack initialized");
    
    // Initialize LED (must be after BTstack.setup() on Pico W)
    // The CYW43 WiFi chip controls the LED
    // Quick test blink to confirm LED is working
    Serial.println("LED test (Pico W CYW43)...");
    for (int i = 0; i < 3; i++) {
        setLED(true);
        delay(100);
        setLED(false);
        delay(100);
    }
    Serial.println("LED initialized");
    
    // Set custom advertising data to include our FPV service UUID
    // This allows TX to find us during scanning
    Serial.println("Setting advertising data...");
    
    // Build advertisement data:
    // - Flags (0x01): LE General Discoverable, BR/EDR not supported
    // - Complete List of 16-bit Service UUIDs (0x03): Our FPV service 0xFF00
    // - Complete Local Name (0x09): "FPV-RX"
    const uint8_t adv_data[] = {
        // Flags
        0x02,  // Length
        0x01,  // Type: Flags
        0x06,  // Value: LE General Discoverable | BR/EDR Not Supported
        
        // Complete list of 16-bit Service UUIDs
        0x03,  // Length
        0x03,  // Type: Complete List of 16-bit Service UUIDs
        0x00, 0xFF,  // UUID: 0xFF00 (little-endian)
        
        // Complete Local Name
        0x07,  // Length
        0x09,  // Type: Complete Local Name
        'F', 'P', 'V', '-', 'R', 'X'
    };
    
    BTstack.setAdvData(sizeof(adv_data), adv_data);
    
    // Start advertising
    Serial.println("Starting advertising as 'FPV-RX'...");
    BTstack.startAdvertising();
    
    // Initialize CRSF
    Serial.println("Initializing CRSF...");
    Serial2.setTX(CRSF_TX_PIN);
    Serial2.setRX(CRSF_RX_PIN);
    crsf.begin(CRSF_BAUDRATE);
    
    // Register battery telemetry callback
    crsf.onPacketBattery = onBatteryTelemetry;
    // Register link statistics callback for logging
    crsf.onPacketLinkStatistics = onLinkStats;
    Serial.println("Battery telemetry callback registered");
    
    Serial.print("CRSF on Serial2: TX=GP");
    Serial.print(CRSF_TX_PIN);
    Serial.print(", RX=GP");
    Serial.println(CRSF_RX_PIN);
    
    Serial.println("=== RX Ready ===");
    Serial.println("Waiting for connections...");
    Serial.println("Hold BOOTSEL for 5 seconds to enter pairing mode");
    Serial.println("Battery telemetry will be sent to TX every 10 seconds");
}

void loop() {
    // Process BTstack
    BTstack.loop();
    
    // Check BOOTSEL button for pairing mode entry
    checkBootselButton();
    
    // Update pairing mode state (timeout, LED blink)
    updatePairingMode();
    
    unsigned long now = millis();
    
    // Heartbeat
    static unsigned long lastHeartbeat = 0;
    if (now - lastHeartbeat > 5000) {
        lastHeartbeat = now;
        Serial.print("RX alive, connected: ");
        Serial.print(clientConnected ? "Yes" : "No");
        if (pairingMode) {
            Serial.print(", PAIRING MODE (");
            Serial.print((PAIRING_MODE_TIMEOUT_MS - (millis() - pairingModeStartTime)) / 1000);
            Serial.print("s remaining)");
        }
        Serial.print(", last data: ");
        if (lastDataReceived > 0) {
            Serial.print(now - lastDataReceived);
            Serial.println("ms ago (fresh)");
        } else if (lastFrameArrivalMs > 0) {
            Serial.print(now - lastFrameArrivalMs);
            Serial.println("ms ago (stale)");
        } else {
            Serial.println("never");
        }
        Serial.print("Last packet age: ");
        Serial.print(lastPacketAgeMs);
        Serial.println("ms");
    }
    
    // Evaluate packet age for stale/failsafe handling
    uint32_t ageSinceTx = (lastRxAlignedMs != 0)
        ? clampAgeToMs(static_cast<int32_t>(now) - static_cast<int32_t>(lastRxAlignedMs))
        : 0;
    if (clientConnected && lastRxAlignedMs != 0) {
        if (ageSinceTx > PACKET_FAILSAFE_THRESHOLD_MS) {
            rampChannelsToFailsafe();
            if (now - lastFailsafeLogMs > 500) {
                Serial.print("Failsafe ramp active, age=");
                Serial.print(ageSinceTx);
                Serial.println("ms");
                lastFailsafeLogMs = now;
            }
        } else if (ageSinceTx > PACKET_STALE_THRESHOLD_MS && now - lastStaleLogMs > 1000) {
            Serial.print("Holding last targets, age=");
            Serial.print(ageSinceTx);
            Serial.println("ms");
            lastStaleLogMs = now;
        }
    }
    
    // Process CRSF (receives telemetry from FC on GP9)
    crsf.loop();
    
    // Send battery telemetry to TX over BLE (rate-limited internally to every 10 seconds)
    sendBatteryTelemetry();
    
    // Send CRSF at 50Hz
    static unsigned long lastCRSF = 0;
    if (millis() - lastCRSF >= 20) {
        lastCRSF = millis();
        
        crsf_channels_t crsfChannels = {0};
        // Direct pass-through: TX channels map directly to CRSF channels
        // TX: ADS1115 ch0→A, ch1→E, ch2→R, ch3→T
        // CRSF: ch0=Aileron, ch1=Elevator, ch2=Rudder, ch3=Throttle
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
    
    delay(1);
}
