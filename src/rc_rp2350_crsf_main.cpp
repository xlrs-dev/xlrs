#ifndef UNIT_TEST

#include <Arduino.h>
#include <string.h>

#include "lora_link/protocol.h"

using namespace lora_link;

#ifndef CRSF_UART_TX_PIN
#define CRSF_UART_TX_PIN 8
#endif

#ifndef CRSF_UART_RX_PIN
#define CRSF_UART_RX_PIN 9
#endif

#ifndef RC_CRSF_SIMPLETX_FRAME_US
#define RC_CRSF_SIMPLETX_FRAME_US 2000u
#endif

static constexpr uint8_t kAileronPin = 26;
static constexpr uint8_t kElevatorPin = 27;
static constexpr uint8_t kRudderPin = 28;
static constexpr uint8_t kThrottlePin = 29;
static constexpr uint8_t kTogglePins[8] = {1, 2, 3, 6, 7, 10, 11, 12};
static constexpr uint8_t kAdcPins[4] = {kAileronPin, kElevatorPin, kRudderPin, kThrottlePin};
static constexpr uint16_t kAdcMax = 4095;

static uint16_t g_channels[kRcChannelCount];
static uint16_t g_filteredAdc[4];
static uint32_t g_framesSent = 0;
static uint32_t g_channelGuardRejects = 0;
static uint32_t g_channelSpikeHolds = 0;
static uint32_t g_lastFrameUs = 0;
static bool g_filterReady = false;
static bool g_haveChannels = false;
static RcSpikeGate g_channelSpikeGate{};
static constexpr uint16_t kRcSpikeJumpThreshold = 160;
static constexpr uint16_t kRcSpikeConfirmTolerance = 80;

static uint16_t readAveragedAdc(uint8_t pin) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 8; ++i) sum += analogRead(pin);
    return static_cast<uint16_t>((sum + 4) / 8);
}

static void updateAdcFilters() {
    for (uint8_t i = 0; i < 4; ++i) {
        const uint16_t raw = readAveragedAdc(kAdcPins[i]);
        if (!g_filterReady) {
            g_filteredAdc[i] = raw;
        } else {
            g_filteredAdc[i] = static_cast<uint16_t>(((static_cast<uint32_t>(g_filteredAdc[i]) * 3u) + raw + 2u) / 4u);
        }
    }
    g_filterReady = true;
}

static uint16_t adcToCrsfRaw(uint16_t value, bool invert = false) {
    if (value > kAdcMax) value = kAdcMax;
    if (invert) value = kAdcMax - value;
    return static_cast<uint16_t>(
        kCrsfRaw1000 + ((static_cast<uint32_t>(value) * (kCrsfRaw2000 - kCrsfRaw1000) + (kAdcMax / 2)) / kAdcMax));
}

static uint16_t readThreePosition(uint8_t in1Pin, uint8_t in2Pin) {
    const bool in1High = digitalRead(in1Pin) == HIGH;
    const bool in2High = digitalRead(in2Pin) == HIGH;
    if (in1High && !in2High) return kCrsfRaw1000;
    if (!in1High && in2High) return kCrsfRaw2000;
    return kCrsfRawMid;
}

static void updateChannels() {
    updateAdcFilters();
    uint16_t candidate[kRcChannelCount];
    candidate[0] = adcToCrsfRaw(g_filteredAdc[0]);
    candidate[1] = adcToCrsfRaw(g_filteredAdc[1]);
    candidate[2] = adcToCrsfRaw(g_filteredAdc[2]);
    candidate[3] = adcToCrsfRaw(g_filteredAdc[3]);
    candidate[4] = readThreePosition(kTogglePins[0], kTogglePins[1]);
    candidate[5] = readThreePosition(kTogglePins[2], kTogglePins[3]);
    candidate[6] = readThreePosition(kTogglePins[4], kTogglePins[5]);
    candidate[7] = readThreePosition(kTogglePins[6], kTogglePins[7]);
    for (uint8_t i = 8; i < kRcChannelCount; ++i) candidate[i] = kCrsfRawMid;
    if (!sanitizeRcChannels(g_channels, g_haveChannels, candidate)) {
        ++g_channelGuardRejects;
        return;
    }
    if (!acceptRcChannelsWithSpikeGate(g_channels, g_haveChannels, candidate, g_channelSpikeGate,
                                       kRcSpikeJumpThreshold, kRcSpikeConfirmTolerance)) {
        ++g_channelSpikeHolds;
        return;
    }
    memcpy(g_channels, candidate, sizeof(g_channels));
    g_haveChannels = true;
}

static void sendCrsfChannels() {
    uint8_t frame[32];
    const size_t len = encodeCrsfRcFrame(g_channels, frame, sizeof(frame));
    if (len) {
        Serial2.write(frame, len);
        ++g_framesSent;
    }
}

static void printChannels() {
    for (uint8_t i = 0; i < kRcChannelCount; ++i) {
        if (i) Serial.print(',');
        Serial.print(g_channels[i]);
    }
    Serial.println();
}

static void serviceCli() {
    static char line[32];
    static uint8_t pos = 0;
    while (Serial.available()) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\r') continue;
        if (c == '\n') {
            line[pos] = '\0';
            if (strcmp(line, "status") == 0) {
                Serial.printf("role=rc-rp2350 baud=%lu frames=%lu guard=%lu hold=%lu ch=%u,%u,%u,%u\n",
                              static_cast<unsigned long>(kCrsfBaud),
                              static_cast<unsigned long>(g_framesSent),
                              static_cast<unsigned long>(g_channelGuardRejects),
                              static_cast<unsigned long>(g_channelSpikeHolds),
                              g_channels[0], g_channels[1], g_channels[2], g_channels[3]);
            } else if (strcmp(line, "channels") == 0) {
                printChannels();
            } else if (strcmp(line, "reboot") == 0) {
                rp2040.reboot();
            } else if (line[0]) {
                Serial.println("commands: status | channels | reboot");
            }
            pos = 0;
        } else if (pos < sizeof(line) - 1) {
            line[pos++] = c;
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial.ignoreFlowControl(true);
    Serial2.setTX(CRSF_UART_TX_PIN);
    Serial2.setRX(CRSF_UART_RX_PIN);
    Serial2.begin(kCrsfBaud);
    for (uint8_t pin : kTogglePins) pinMode(pin, INPUT_PULLUP);
    analogReadResolution(12);
    updateChannels();
    Serial.println("RC RP2350 CRSF output");
    Serial.printf("Serial2 TX=GP%u RX=GP%u baud=%lu\n",
                  static_cast<unsigned>(CRSF_UART_TX_PIN),
                  static_cast<unsigned>(CRSF_UART_RX_PIN),
                  static_cast<unsigned long>(kCrsfBaud));
}

void loop() {
    serviceCli();
    const uint32_t nowUs = micros();
    if (static_cast<uint32_t>(nowUs - g_lastFrameUs) >= RC_CRSF_SIMPLETX_FRAME_US) {
        g_lastFrameUs = nowUs;
        updateChannels();
        sendCrsfChannels();
    }
    yield();
}

#endif // UNIT_TEST
