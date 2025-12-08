// RX Side using SX128x (RadioLib) instead of BLE.
#include <Arduino.h>
#include <string.h>
#include <EEPROM.h>

#include "Protocol.h"
#include "Security.h"
#include "crsfSerial.h"
#include "crsf_protocol.h"
#include "SX128xLink.h"

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include "pico/cyw43_arch.h"

// CRSF Serial pins
#define CRSF_TX_PIN 8
#define CRSF_RX_PIN 9

// Pairing mode configuration
#define PAIRING_MODE_TIMEOUT_MS 60000   // 60 seconds pairing window
#define BOOTSEL_HOLD_TIME_MS    5000    // 5 seconds to enter pairing mode
#define PAIRING_KEY_SIZE        16

// Battery telemetry
const unsigned long BATTERY_TX_INTERVAL_MS = 10000;  // Send battery telemetry every 10 seconds

// Link timeout
const unsigned long TIMEOUT_MS = 1000;

// Radio message types
enum RadioMsgType : uint8_t {
    MSG_CHANNELS = 0x01,
    MSG_BATTERY  = 0x02,
    MSG_PAIRING  = 0x03,
};

// Application state
CrsfSerial crsf(Serial2, CRSF_BAUDRATE);
Security security;
SX128xLink radio;

// Channel values received from TX
uint16_t channels[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

// Connection state
unsigned long lastDataReceived = 0;
uint16_t lastSequence = 0;

// Pairing mode state
bool pairingMode = false;
unsigned long pairingModeStartTime = 0;

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
void setLED(bool state);
bool get_bootsel_button();
void enterPairingMode();
void exitPairingMode(bool success);
void checkBootselButton();
void updatePairingMode();
void sendBatteryTelemetry();
void handleRadioRx();

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

    if (cyw43_arch_init()) {
        Serial.println("Failed to init CYW43");
    } else {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    }

    // Initialize security
    if (security.begin()) {
        Serial.println("Security initialized");
    } else {
        Serial.println("Security initialization failed!");
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
    // Process pairing button
    checkBootselButton();
    updatePairingMode();

    // Handle radio RX
    handleRadioRx();

    // Check for timeout
    if (lastDataReceived > 0 && (millis() - lastDataReceived > TIMEOUT_MS)) {
        static unsigned long lastTimeout = 0;
        if (millis() - lastTimeout > 5000) {
            Serial.println("*** TIMEOUT ***");
            lastTimeout = millis();
        }
        for (int i = 0; i < 8; i++) {
            channels[i] = 1500;
        }
    }

    // Process CRSF (receives telemetry from FC on GP9)
    crsf.loop();

    // Send battery telemetry to TX over radio (rate-limited)
    sendBatteryTelemetry();

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
// Radio RX handler
// ============================================================
void handleRadioRx() {
    SX128xPacket pkt;
    if (!radio.receive(pkt)) {
        return;
    }
    if (pkt.length < 1) {
        return;
    }

    uint8_t type = pkt.data[0];
    const uint8_t* payload = pkt.data + 1;
    size_t payload_len = pkt.length - 1;

    switch (type) {
        case MSG_CHANNELS: {
            if (payload_len != FRAME_SIZE) {
                return;
            }
            uint16_t sequence = 0;
            if (Protocol::decodeFrame(payload, channels, &sequence, &security, &lastSequence)) {
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
                setLED(true);
            } else {
                Serial.println("Invalid frame (security check failed)");
            }
            break;
        }
        case MSG_PAIRING: {
            if (!pairingMode || payload_len != PAIRING_KEY_SIZE) {
                return;
            }
            if (security.setPairingKey(payload)) {
                Serial.println("Pairing key saved");
                exitPairingMode(true);
            } else {
                Serial.println("Failed to save pairing key");
            }
            break;
        }
        default:
            break;
    }
}

// ============================================================
// Battery telemetry back to TX
// ============================================================
void sendBatteryTelemetry() {
    if (!batteryDataValid) return;
    if (!radio.isReady()) return;
    if (millis() - lastBatteryTxTime < BATTERY_TX_INTERVAL_MS) return;
    lastBatteryTxTime = millis();

    uint8_t packet[1 + 5];
    uint16_t voltage_x10 = (uint16_t)(batteryVoltage * 10);
    uint16_t current_x10 = (uint16_t)(batteryCurrent * 10);
    packet[0] = MSG_BATTERY;
    packet[1] = (voltage_x10 >> 8) & 0xFF;
    packet[2] = voltage_x10 & 0xFF;
    packet[3] = (current_x10 >> 8) & 0xFF;
    packet[4] = current_x10 & 0xFF;
    packet[5] = batteryRemaining;

    if (radio.send(packet, sizeof(packet))) {
        Serial.print("[RADIO TELEM] Sent battery: ");
        Serial.print(batteryVoltage, 1);
        Serial.print("V ");
        Serial.print(batteryRemaining);
        Serial.println("%");
    }
}

// ============================================================
// LED control (Pico W)
// ============================================================
void setLED(bool state) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, state ? 1 : 0);
}

// ============================================================
// BOOTSEL Button Reading (RP2040-specific)
// ============================================================
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
// Pairing helpers
// ============================================================
void enterPairingMode() {
    pairingMode = true;
    pairingModeStartTime = millis();

    Serial.println("*** PAIRING MODE ACTIVE ***");
    Serial.println("Waiting for TX to send pairing key...");
    Serial.print("Timeout in ");
    Serial.print(PAIRING_MODE_TIMEOUT_MS / 1000);
    Serial.println(" seconds");

    setLED(true);
}

void exitPairingMode(bool success) {
    pairingMode = false;

    if (success) {
        Serial.println("*** PAIRING SUCCESSFUL ***");
        setLED(true);
        delay(500);
        setLED(false);
    } else {
        Serial.println("*** PAIRING MODE TIMEOUT ***");
        setLED(false);
    }
}

void checkBootselButton() {
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck < 50) return;
    lastCheck = millis();

    static unsigned long buttonPressStart = 0;
    static bool buttonWasPressed = false;

    bool buttonPressed = get_bootsel_button();

    if (buttonPressed && !buttonWasPressed) {
        buttonPressStart = millis();
        buttonWasPressed = true;
    } else if (buttonPressed && buttonWasPressed) {
        unsigned long holdTime = millis() - buttonPressStart;
        if (holdTime > BOOTSEL_HOLD_TIME_MS && !pairingMode) {
            enterPairingMode();
        }
    } else if (!buttonPressed && buttonWasPressed) {
        buttonWasPressed = false;
    }
}

void updatePairingMode() {
    if (!pairingMode) return;
    if (millis() - pairingModeStartTime > PAIRING_MODE_TIMEOUT_MS) {
        exitPairingMode(false);
        return;
    }

    // Fast blink LED in pairing mode
    bool newLedState = (millis() / 100) % 2;
    setLED(newLedState);
}


