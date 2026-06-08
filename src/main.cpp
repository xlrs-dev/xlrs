#ifndef UNIT_TEST

#include <Arduino.h>
#include <EEPROM.h>
#include <RadioLib.h>
#include <SPI.h>
#include <string.h>

#include "lora_link/protocol.h"

using namespace lora_link;

#ifndef DEFAULT_BINDING_PHRASE
#define DEFAULT_BINDING_PHRASE "default"
#endif

#if !defined(LORA_TX_ROLE) && !defined(LORA_RX_ROLE)
#error "Select LORA_TX_ROLE or LORA_RX_ROLE"
#endif

static DeviceConfig g_config;
static uint8_t g_uid[kUidSize];
static uint32_t g_uidCheck;
static uint16_t g_sequence = 0;
static uint16_t g_hop = 0;
static uint32_t g_lastRcInputMs = 0;
static uint32_t g_lastUplinkMs = 0;
static uint32_t g_lastLinkStatsMs = 0;
static bool g_configFault = false;
static bool g_rxWaiting = false;
static volatile bool g_dio1 = false;
static uint16_t g_channels[kRcChannelCount];
static int16_t g_lastRssi = 0;
static int8_t g_lastSnr = 0;
static uint8_t g_linkQuality = 0;
static bool g_radioReady = false;
static uint32_t g_lastRxHopAdvanceUs = 0;
static uint32_t g_validOtaFrames = 0;
static uint32_t g_rejectedOtaFrames = 0;
#if LORA_TX_ROLE
static uint32_t g_downlinkTelemetryFrames = 0;
static uint32_t g_uplinkTransmitAttempts = 0;
static uint32_t g_uplinkTransmitSuccesses = 0;
static uint32_t g_telemetryListenSlots = 0;
static uint32_t g_crsfInputFrames = 0;
static uint32_t g_crsfInputRejects = 0;
static uint32_t g_crsfInputBytes = 0;
static uint8_t g_crsfByteRing[64];
static uint8_t g_crsfByteRingPos = 0;
static uint8_t g_telemetryListenSlotsRemaining = 0;
#endif
#if LORA_RX_ROLE
static bool g_haveLastSequence = false;
static uint16_t g_lastSequence = 0;
static uint32_t g_uplinkDropCount = 0;
static uint16_t g_lqWindowFrames = 0;
static uint16_t g_lqWindowDrops = 0;
static uint32_t g_lqWindowStartMs = 0;
static uint32_t g_rxChannelGuardRejects = 0;
static uint32_t g_rxPrimaryJumpHolds = 0;
static uint16_t g_pendingPrimaryJump[kRcChannelCount];
static bool g_havePendingPrimaryJump = false;
#endif

SX1280 radio = new Module(SX128X_SPI_CS, SX128X_SPI_DIO1, SX128X_SPI_RST, SX128X_SPI_BUSY);

static const RateConfig& activeRate() {
    return kRates[static_cast<uint8_t>(g_config.rate)];
}

static float rfFrequencyForHop(uint16_t hop) {
    if (kFixedRfChannel >= 0) return fhssFrequencyMHz(static_cast<uint8_t>(kFixedRfChannel));
    return fhssFrequencyMHz(fhssChannelFor(g_uid, hop));
}

static void onDio1() {
    g_dio1 = true;
}

static void setDefaultChannels() {
    for (uint8_t i = 0; i < kRcChannelCount; ++i) g_channels[i] = 992;
    g_channels[2] = 172; // throttle low in CRSF 11-bit units
}

static void saveConfig() {
    ConfigRecord record = makeConfigRecord(g_config);
    EEPROM.put(0, record);
    EEPROM.commit();
}

static void loadConfig() {
    EEPROM.begin(256);
    ConfigRecord record{};
    EEPROM.get(0, record);
    if (!readConfigRecord(record, g_config)) {
        configDefaults(g_config, DEFAULT_BINDING_PHRASE);
        g_configFault = true;
        saveConfig();
    }
    deriveUid(g_config.bindingPhrase, g_uid);
    g_uidCheck = uidCheck(g_uid);
}

static bool configureRadio(float freqMHz) {
    SPI.setSCK(SX128X_SPI_SCK);
    SPI.setTX(SX128X_SPI_MOSI);
    SPI.setRX(SX128X_SPI_MISO);
    SPI.begin();
    pinMode(SX128X_RXEN, OUTPUT);
    pinMode(SX128X_TXEN, OUTPUT);
    digitalWrite(SX128X_RXEN, HIGH);
    digitalWrite(SX128X_TXEN, LOW);

    const RateConfig& rate = activeRate();
    const int16_t state = radio.begin(freqMHz, rate.bandwidthKHz, rate.spreadingFactor,
                                      rate.codingRate, syncWordFromUid(g_uid), 10, 12);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("radio init failed: %d\n", state);
        return false;
    }
    radio.setDio1Action(onDio1);
    radio.setCRC(2);
    radio.startReceive();
    g_rxWaiting = true;
    g_lastRxHopAdvanceUs = micros();
    return true;
}

static void startReceiveOnHop() {
    if (!g_radioReady) return;
    digitalWrite(SX128X_TXEN, LOW);
    digitalWrite(SX128X_RXEN, HIGH);
    radio.setFrequency(rfFrequencyForHop(g_hop));
    radio.startReceive();
    g_rxWaiting = true;
    g_dio1 = false;
    g_lastRxHopAdvanceUs = micros();
}

static bool transmitFrame(const OtaFrame& frame) {
    if (!g_radioReady) return false;
    uint8_t bytes[kOtaFrameSize];
    if (!encodeOtaFrame(frame, bytes)) return false;
    g_rxWaiting = false;
    digitalWrite(SX128X_RXEN, LOW);
    digitalWrite(SX128X_TXEN, HIGH);
    radio.setFrequency(rfFrequencyForHop(g_hop));
    const int16_t state = radio.transmit(bytes, sizeof(bytes));
    digitalWrite(SX128X_TXEN, LOW);
    digitalWrite(SX128X_RXEN, HIGH);
    radio.startReceive();
    g_rxWaiting = true;
    g_dio1 = false;
    return state == RADIOLIB_ERR_NONE;
}

static bool readOtaFrame(OtaFrame& out) {
    if (!g_radioReady) return false;
    if (!g_dio1) return false;
    g_dio1 = false;
    uint8_t bytes[kOtaFrameSize] = {};
    const int16_t state = radio.readData(bytes, sizeof(bytes));
    radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        ++g_rejectedOtaFrames;
        return false;
    }
    g_lastRssi = static_cast<int16_t>(radio.getRSSI());
    g_lastSnr = static_cast<int8_t>(radio.getSNR());
    if (!decodeOtaFrame(bytes, g_uidCheck, out)) {
        ++g_rejectedOtaFrames;
        return false;
    }
    ++g_validOtaFrames;
    return true;
}

static void printStatus() {
    Serial.printf("role=%s phrase=%s rate=%s uid=%02X%02X%02X%02X%02X%02X%02X%02X lq=%u rssi=%d snr=%d good=%lu bad=%lu",
#if LORA_TX_ROLE
                  "tx",
#else
                  "rx",
#endif
                  g_config.bindingPhrase, activeRate().name,
                  g_uid[0], g_uid[1], g_uid[2], g_uid[3], g_uid[4], g_uid[5], g_uid[6], g_uid[7],
                  g_linkQuality, g_lastRssi, g_lastSnr,
                  static_cast<unsigned long>(g_validOtaFrames),
                  static_cast<unsigned long>(g_rejectedOtaFrames));
#if LORA_TX_ROLE
    const uint32_t crsfAgeMs = g_lastRcInputMs ? millis() - g_lastRcInputMs : 0xFFFFFFFFu;
    Serial.printf(" tlm=%lu tx_ok=%lu tx_try=%lu listen=%lu crsf=%lu crsf_rej=%lu crsf_bytes=%lu crsf_age=%lu ch=%u,%u,%u,%u",
                  static_cast<unsigned long>(g_downlinkTelemetryFrames),
                  static_cast<unsigned long>(g_uplinkTransmitSuccesses),
                  static_cast<unsigned long>(g_uplinkTransmitAttempts),
                  static_cast<unsigned long>(g_telemetryListenSlots),
                  static_cast<unsigned long>(g_crsfInputFrames),
                  static_cast<unsigned long>(g_crsfInputRejects),
                  static_cast<unsigned long>(g_crsfInputBytes),
                  static_cast<unsigned long>(crsfAgeMs),
                  g_channels[0], g_channels[1], g_channels[2], g_channels[3]);
#else
    Serial.printf(" drops=%lu guard=%lu hold=%lu",
                  static_cast<unsigned long>(g_uplinkDropCount),
                  static_cast<unsigned long>(g_rxChannelGuardRejects),
                  static_cast<unsigned long>(g_rxPrimaryJumpHolds));
#endif
    Serial.printf(" config=%s\n", g_configFault ? "defaulted" : "ok");
}

static void handleCliLine(char* line) {
    while (*line == ' ') ++line;
    if (strcmp(line, "bind get") == 0) {
        Serial.println(g_config.bindingPhrase);
    } else if (strncmp(line, "bind set ", 9) == 0) {
        const char* phrase = line + 9;
        const size_t len = strlen(phrase);
        if (len < 1 || len > 32) {
            Serial.println("ERR phrase must be 1..32 bytes");
            return;
        }
        memset(g_config.bindingPhrase, 0, sizeof(g_config.bindingPhrase));
        strncpy(g_config.bindingPhrase, phrase, sizeof(g_config.bindingPhrase) - 1);
        saveConfig();
        Serial.println("OK reboot required");
    } else if (strcmp(line, "bind clear") == 0) {
        configDefaults(g_config, DEFAULT_BINDING_PHRASE);
        saveConfig();
        Serial.println("OK reboot required");
    } else if (strncmp(line, "rate ", 5) == 0) {
        if (strcmp(line + 5, "L250") == 0) g_config.rate = RateId::L250;
        else if (strcmp(line + 5, "L100") == 0) g_config.rate = RateId::L100;
        else {
            Serial.println("ERR rate must be L250 or L100");
            return;
        }
        saveConfig();
        Serial.println("OK reboot required");
    } else if (strcmp(line, "status") == 0) {
        printStatus();
    } else if (strcmp(line, "channels") == 0) {
        for (uint8_t i = 0; i < kRcChannelCount; ++i) {
            if (i) Serial.print(',');
            Serial.print(g_channels[i]);
        }
        Serial.println();
#if LORA_TX_ROLE
    } else if (strcmp(line, "crsf dump") == 0) {
        for (uint8_t i = 0; i < sizeof(g_crsfByteRing); ++i) {
            const uint8_t index = static_cast<uint8_t>((g_crsfByteRingPos + i) & 0x3F);
            if (i) Serial.print(' ');
            if (g_crsfByteRing[index] < 16) Serial.print('0');
            Serial.print(g_crsfByteRing[index], HEX);
        }
        Serial.println();
#endif
    } else if (strcmp(line, "reboot") == 0) {
        rp2040.reboot();
    } else if (*line) {
        Serial.println("commands: bind get | bind set <phrase> | bind clear | rate L250|L100 | status | channels | reboot");
    }
}

static void serviceCli() {
    static char line[80];
    static uint8_t pos = 0;
    while (Serial.available()) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\r') continue;
        if (c == '\n') {
            line[pos] = '\0';
            handleCliLine(line);
            pos = 0;
        } else if (pos < sizeof(line) - 1) {
            line[pos++] = c;
        }
    }
}

static void setupCrsfUart() {
    Serial2.setTX(CRSF_UART_TX_PIN);
    Serial2.setRX(CRSF_UART_RX_PIN);
    Serial2.begin(kCrsfBaud);
}

#if LORA_TX_ROLE
static void serviceTxCrsfInput() {
    while (Serial2.available()) {
        const uint8_t byte = static_cast<uint8_t>(Serial2.read());
        ++g_crsfInputBytes;
        g_crsfByteRing[g_crsfByteRingPos] = byte;
        g_crsfByteRingPos = static_cast<uint8_t>((g_crsfByteRingPos + 1) & 0x3F);
        uint16_t candidate[kRcChannelCount];
        if (parseCrsfRcFrame(byte, candidate)) {
            if (!sanitizeRcChannels(g_channels, g_lastRcInputMs != 0, candidate)) {
                ++g_crsfInputRejects;
                continue;
            }
            memcpy(g_channels, candidate, sizeof(g_channels));
            g_lastRcInputMs = millis();
            ++g_crsfInputFrames;
        }
    }
}

static void sendTelemetryToHandset() {
    uint8_t frame[16];
    const size_t len = encodeCrsfLinkStats(g_lastRssi, g_lastSnr, g_linkQuality,
                                           static_cast<uint8_t>(g_config.rate), frame, sizeof(frame));
    if (len) Serial2.write(frame, len);
}

static void txLoop() {
    serviceTxCrsfInput();
    OtaFrame rx{};
    if (readOtaFrame(rx) && rx.type == OtaType::Telemetry) {
        g_linkQuality = rx.payload[0];
        if (rx.payloadLen > 1) g_lastRssi = -static_cast<int16_t>(rx.payload[1]);
        if (rx.payloadLen > 2) g_lastSnr = static_cast<int8_t>(rx.payload[2]);
        ++g_downlinkTelemetryFrames;
        sendTelemetryToHandset();
    }

    static uint32_t lastPacketUs = 0;
    const uint32_t nowUs = micros();
    if (static_cast<uint32_t>(nowUs - lastPacketUs) < activeRate().intervalUs) return;
    lastPacketUs = nowUs;

    if (g_telemetryListenSlotsRemaining > 0) {
        --g_telemetryListenSlotsRemaining;
        ++g_telemetryListenSlots;
        return;
    }

    if (millis() - g_lastRcInputMs > 250) setDefaultChannels();
    OtaFrame frame{};
    frame.type = OtaType::Rc;
    const uint16_t txSequence = g_sequence++;
    frame.sequence = txSequence;
    frame.uidCheck = g_uidCheck;
    frame.payloadLen = kOtaPayloadSize;
    packRcChannels11Bit(g_channels, frame.payload);
    ++g_uplinkTransmitAttempts;
    if (transmitFrame(frame)) {
        ++g_uplinkTransmitSuccesses;
        if (activeRate().telemetryRatio && (txSequence % activeRate().telemetryRatio) == 0) {
            g_telemetryListenSlotsRemaining = 8;
        }
    }
    ++g_hop;
}
#endif

#if LORA_RX_ROLE
static void writeFcChannels() {
    uint8_t frame[32];
    const size_t len = encodeCrsfRcFrame(g_channels, frame, sizeof(frame));
    if (len) Serial2.write(frame, len);
}

static bool hasLargePrimaryJump(const uint16_t previous[kRcChannelCount],
                                const uint16_t candidate[kRcChannelCount],
                                uint16_t threshold) {
    for (uint8_t i = 0; i < 4; ++i) {
        const uint16_t oldValue = clampCrsfRaw(previous[i]);
        const uint16_t newValue = clampCrsfRaw(candidate[i]);
        const uint16_t delta = oldValue > newValue ? oldValue - newValue : newValue - oldValue;
        if (delta > threshold) return true;
    }
    return false;
}

static bool primaryChannelsMatch(const uint16_t a[kRcChannelCount],
                                 const uint16_t b[kRcChannelCount],
                                 uint16_t tolerance) {
    for (uint8_t i = 0; i < 4; ++i) {
        const uint16_t av = clampCrsfRaw(a[i]);
        const uint16_t bv = clampCrsfRaw(b[i]);
        const uint16_t delta = av > bv ? av - bv : bv - av;
        if (delta > tolerance) return false;
    }
    return true;
}

static bool acceptRxOutputCandidate(const uint16_t candidate[kRcChannelCount]) {
    if (!hasLargePrimaryJump(g_channels, candidate, 220)) {
        g_havePendingPrimaryJump = false;
        return true;
    }
    if (g_havePendingPrimaryJump && primaryChannelsMatch(g_pendingPrimaryJump, candidate, 80)) {
        g_havePendingPrimaryJump = false;
        return true;
    }
    memcpy(g_pendingPrimaryJump, candidate, sizeof(g_pendingPrimaryJump));
    g_havePendingPrimaryJump = true;
    ++g_rxPrimaryJumpHolds;
    return false;
}

static void writeFcLinkStats() {
    uint8_t frame[16];
    const uint8_t lq = (millis() - g_lastUplinkMs < 250) ? g_linkQuality : 0;
    const size_t len = encodeCrsfLinkStats(g_lastRssi, g_lastSnr, lq,
                                           static_cast<uint8_t>(g_config.rate), frame, sizeof(frame));
    if (len) Serial2.write(frame, len);
}

static void updateRxSequenceStats(uint16_t rxSequence) {
    if (g_haveLastSequence) {
        const uint16_t expected = static_cast<uint16_t>(g_lastSequence + 1);
        if (rxSequence != expected) {
            const uint16_t missedFrames = static_cast<uint16_t>(rxSequence - expected);
            if (missedFrames < 1000) {
                g_uplinkDropCount += missedFrames;
                g_lqWindowDrops += missedFrames;
            }
        }
    }
    g_haveLastSequence = true;
    g_lastSequence = rxSequence;
}

static void updateRxLinkQuality(bool acceptedFrame) {
    const uint32_t nowMs = millis();
    if (g_lqWindowStartMs == 0) g_lqWindowStartMs = nowMs;
    if (acceptedFrame) ++g_lqWindowFrames;

    const uint32_t elapsedMs = nowMs - g_lqWindowStartMs;
    if (elapsedMs >= 1000) {
        const uint32_t expectedFrames = static_cast<uint32_t>(g_lqWindowFrames) + g_lqWindowDrops;
        uint32_t percent = expectedFrames ? (static_cast<uint32_t>(g_lqWindowFrames) * 100u) / expectedFrames : 0;
        if (percent > 100) percent = 100;
        g_linkQuality = static_cast<uint8_t>(percent);
        g_lqWindowFrames = 0;
        g_lqWindowDrops = 0;
        g_lqWindowStartMs = nowMs;
    }
}

static void rxLoop() {
    OtaFrame frame{};
    if (readOtaFrame(frame) && frame.type == OtaType::Rc && frame.payloadLen == kOtaPayloadSize) {
        const uint16_t rxSequence = frame.sequence;
        uint16_t candidate[kRcChannelCount];
        g_hop = rxSequence;
        unpackRcChannels11Bit(frame.payload, candidate);
        g_lastUplinkMs = millis();
        updateRxSequenceStats(rxSequence);
        updateRxLinkQuality(true);
        if (sanitizeRcChannels(g_channels, true, candidate)) {
            if (acceptRxOutputCandidate(candidate)) {
                memcpy(g_channels, candidate, sizeof(g_channels));
                writeFcChannels();
            }
        } else {
            ++g_rxChannelGuardRejects;
        }
        if ((frame.sequence % activeRate().telemetryRatio) == 0) {
            OtaFrame tlm{};
            tlm.type = OtaType::Telemetry;
            tlm.sequence = g_sequence++;
            tlm.uidCheck = g_uidCheck;
            tlm.payloadLen = 4;
            tlm.payload[0] = g_linkQuality;
            tlm.payload[1] = static_cast<uint8_t>(g_lastRssi < 0 ? -g_lastRssi : g_lastRssi);
            tlm.payload[2] = static_cast<uint8_t>(g_lastSnr);
            tlm.payload[3] = static_cast<uint8_t>(g_config.rate);
            transmitFrame(tlm);
        }
        g_hop = static_cast<uint16_t>(rxSequence + 1);
        startReceiveOnHop();
    }

    updateRxLinkQuality(false);
    const bool acquiring = (millis() - g_lastUplinkMs) > 250;
    const uint32_t scanIntervalUs = (kFixedRfChannel >= 0)
                                        ? 100000u
                                        : activeRate().intervalUs * (acquiring ? 4u : 1u);
    const bool shouldScan = (kFixedRfChannel < 0) || acquiring;
    if (shouldScan && g_radioReady && static_cast<uint32_t>(micros() - g_lastRxHopAdvanceUs) >= scanIntervalUs) {
        ++g_hop;
        startReceiveOnHop();
    }

    if (millis() - g_lastUplinkMs > 250) g_linkQuality = 0;
    if (millis() - g_lastLinkStatsMs >= 1000) {
        g_lastLinkStatsMs = millis();
        writeFcLinkStats();
    }
}
#endif

void setup() {
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    Serial.begin(115200);
    Serial.ignoreFlowControl(true);
    for (uint8_t i = 0; i < 30; ++i) {
        serviceCli();
        delay(100);
    }
    setDefaultChannels();
    loadConfig();
#if LORA_USB_DIAG
    Serial.println();
    Serial.println("Clean LoRa Link USB diagnostic");
    printStatus();
    g_radioReady = false;
    return;
#endif
    setupCrsfUart();
#if LORA_CRSF_DIAG
    Serial.println();
    Serial.println("Clean LoRa Link CRSF diagnostic");
    printStatus();
    g_radioReady = false;
    return;
#endif
    Serial.println();
    Serial.println("Clean LoRa Link");
    printStatus();
    g_radioReady = configureRadio(rfFrequencyForHop(0));
    if (!g_radioReady) Serial.println("radio fault: CLI remains available");
}

void loop() {
    serviceCli();
#if LORA_TX_ROLE
    txLoop();
#else
    rxLoop();
#endif
    digitalWrite(STATUS_LED_PIN, millis() - g_lastUplinkMs < 250 ? HIGH : LOW);
    yield();
}

#endif // UNIT_TEST
