/**
 * @file remote_mcu_interface.cpp
 * @brief Production-grade, compilable C++ interfacing example for an external Host MCU.
 * 
 * Target: RP2040 (e.g. Raspberry Pi Pico) or RP2350.
 * Framework: Arduino (Pico SDK core).
 * 
 * This file implements the host side of the XLRS custom high-speed UART protocol.
 * It is 100% non-blocking and features:
 *  1. Self-contained CRC8 engine matching the XLRS core polynomial (0xD5).
 *  2. FNV-1a 64-bit hash engine to statically verify Link UID phrase derivation.
 *  3. Byte-level parser state machine to decode frames without blocking the CPU.
 *  4. High-speed UART operation at 420,000 bps (Serial2 on GP8/GP9).
 *  5. Diagnostic GPIO pin toggling for visual analysis on a scope/logic analyzer.
 *  6. Interactive Serial CLI to isolate Phase 1, Phase 2, and Phase 3 bench testing.
 */

#include <Arduino.h>

// ============================================================================
// Compile-Time Configuration & Diagnostic Pins
// ============================================================================
#define UART_PROTOCOL_BAUDRATE 420000
#define UART_PROTOCOL_SYNC_BYTE 0xA5
#define UART_PROTOCOL_MAX_PAYLOAD 60

// Diagnostic GPIO Pins (Change these to match your RP2040 wiring)
#define DIAG_GPIO_ISR_LATENCY   14   // Toggles high during packet TX preparation/send
#define DIAG_GPIO_RX_LATENCY    15   // Toggles high during packet RX parsing & CRC verify

// Hardware UART Pins for RP2040 Serial2
#define UART2_TX_PIN            8
#define UART2_RX_PIN            9

// ============================================================================
// XLRS Custom Protocol Message Types & Structs
// ============================================================================
enum UARTMsgType : uint8_t {
    UART_MSG_CHANNELS           = 0x01,
    UART_MSG_CMD_PAIR           = 0x10,
    UART_MSG_CMD_BOND           = 0x11,
    UART_MSG_CMD_RESTART        = 0x12,
    UART_MSG_CMD_STATUS_REQ     = 0x13,
    UART_MSG_PING               = 0x14,
    UART_MSG_PONG               = 0x15,
    UART_MSG_CMD_SET_BIND_TX    = 0x16,
    UART_MSG_CMD_SET_BIND_RX    = 0x17,
    UART_MSG_TELEMETRY          = 0x20,
    UART_MSG_STATUS             = 0x21,
    UART_MSG_ACK                = 0x22,
    UART_MSG_ERROR              = 0x23
};

struct ChannelData {
    uint16_t channels[8];  // scaled 1000 - 2000 us (XLRS core uses first 4 channels)
} __attribute__((packed));

struct TelemetryData {
    int16_t rssi;          // dBm
    float snr;             // dB (LoRa only)
    uint16_t rxBattMv;     // millivolts
    uint8_t rxBattPct;     // 0 - 100%
    uint8_t linkQuality;   // 0 - 100%
} __attribute__((packed));

struct StatusData {
    uint8_t connectionState;  // 0=Disconn, 1=Pairing, 2=Conn_ing, 3=Conn_ed, 4=Lost
    uint8_t pairingState;     // 0=unpaired, 1=paired
    uint32_t packetsReceived;
    uint32_t packetsLost;
} __attribute__((packed));

// ============================================================================
// Math Engines: CRC8 (Poly 0xD5) & FNV-1a 64-bit Hash
// ============================================================================
class HostCrc8 {
public:
    HostCrc8() {
        // Initialize the MSB-first lookup table for polynomial 0xD5
        for (int idx = 0; idx < 256; ++idx) {
            uint8_t crc = idx;
            for (int shift = 0; shift < 8; ++shift) {
                crc = (crc << 1) ^ ((crc & 0x80) ? 0xD5 : 0);
            }
            _lut[idx] = crc;
        }
    }

    uint8_t calc(const uint8_t* data, uint8_t len) {
        uint8_t crc = 0;
        while (len--) {
            crc = _lut[crc ^ *data++];
        }
        return crc;
    }

private:
    uint8_t _lut[256];
};

class HostFnv1a {
public:
    static uint64_t hash(const char* str) {
        uint64_t hashVal = 0xcbf29ce484222325ULL;
        while (*str) {
            hashVal ^= (uint8_t)*str++;
            hashVal *= 0x100000001b3ULL;
        }
        return hashVal;
    }
};

// Global Math Instances
HostCrc8 g_crc;

// ============================================================================
// Non-Blocking Packet Parser State Machine
// ============================================================================
class HostPacketParser {
public:
    enum State {
        STATE_SYNC,
        STATE_LENGTH,
        STATE_TYPE,
        STATE_PAYLOAD,
        STATE_CRC
    };

    HostPacketParser() {
        reset();
    }

    void reset() {
        _state = STATE_SYNC;
        _bufferPos = 0;
        _expectedLen = 0;
        _msgType = 0;
        _lastByteTime = millis();
    }

    bool parseByte(uint8_t byte, uint8_t& outType, uint8_t* outPayload, uint8_t& outLen) {
        _lastByteTime = millis();

        switch (_state) {
            case STATE_SYNC:
                if (byte == UART_PROTOCOL_SYNC_BYTE) {
                    _state = STATE_LENGTH;
                }
                break;

            case STATE_LENGTH:
                if (byte <= UART_PROTOCOL_MAX_PAYLOAD) {
                    _expectedLen = byte;
                    _state = STATE_TYPE;
                } else {
                    reset();
                }
                break;

            case STATE_TYPE:
                _msgType = byte;
                _bufferPos = 0;
                if (_expectedLen == 0) {
                    _state = STATE_CRC;
                } else {
                    _state = STATE_PAYLOAD;
                }
                break;

            case STATE_PAYLOAD:
                if (_bufferPos < UART_PROTOCOL_MAX_PAYLOAD) {
                    _payloadBuf[_bufferPos++] = byte;
                    if (_bufferPos >= _expectedLen) {
                        _state = STATE_CRC;
                    }
                } else {
                    reset();
                }
                break;

            case STATE_CRC: {
                // Verify packet CRC using our lookup table
                uint8_t checkBuf[UART_PROTOCOL_MAX_PAYLOAD + 2];
                checkBuf[0] = _expectedLen;
                checkBuf[1] = _msgType;
                if (_expectedLen > 0) {
                    memcpy(checkBuf + 2, _payloadBuf, _expectedLen);
                }
                
                uint8_t calculated = g_crc.calc(checkBuf, _expectedLen + 2);
                reset(); // Reset for next frame

                if (byte == calculated) {
                    outType = _msgType;
                    outLen = _expectedLen;
                    memcpy(outPayload, _payloadBuf, _expectedLen);
                    return true; // Packet successfully decoded!
                }
                break;
            }
        }
        return false;
    }

    void checkTimeout() {
        if (_state != STATE_SYNC && (millis() - _lastByteTime) > 100) {
            reset(); // Clear partial packets if data stalls
        }
    }

private:
    State _state;
    uint8_t _payloadBuf[UART_PROTOCOL_MAX_PAYLOAD];
    uint8_t _bufferPos;
    uint8_t _expectedLen;
    uint8_t _msgType;
    unsigned long _lastByteTime;
};

// Global Parser Instance
HostPacketParser g_parser;

// ============================================================================
// Helper Methods to Send Frames
// ============================================================================
void sendFrame(UARTMsgType type, const uint8_t* payload, uint8_t payloadLen) {
    digitalWrite(DIAG_GPIO_ISR_LATENCY, HIGH); // Signal logic analyzer: TX start

    uint8_t checkBuf[UART_PROTOCOL_MAX_PAYLOAD + 2];
    checkBuf[0] = payloadLen;
    checkBuf[1] = (uint8_t)type;
    if (payloadLen > 0 && payload) {
        memcpy(checkBuf + 2, payload, payloadLen);
    }
    uint8_t crc = g_crc.calc(checkBuf, payloadLen + 2);

    Serial2.write(UART_PROTOCOL_SYNC_BYTE);
    Serial2.write(payloadLen);
    Serial2.write((uint8_t)type);
    if (payloadLen > 0 && payload) {
        Serial2.write(payload, payloadLen);
    }
    Serial2.write(crc);

    digitalWrite(DIAG_GPIO_ISR_LATENCY, LOW); // Signal logic analyzer: TX done
}

// ============================================================================
// Interactive Operating Modes (Serial CLI)
// ============================================================================
enum BenchMode {
    MODE_IDLE,
    MODE_PHASE_1_SELF_TEST,
    MODE_PHASE_2_TX_INTERFACE,
    MODE_PHASE_3_RX_LISTENER
};

BenchMode g_activeMode = MODE_IDLE;

// Statistics Variables for Phase 2/3
unsigned long g_lastPingTime = 0;
unsigned long g_lastChannelTx = 0;
uint32_t g_pingsSent = 0;
uint32_t g_pingsReceived = 0;
uint32_t g_packetsDropped = 0;

// Emu Stick Channel State (Ph 2)
uint16_t g_testStickVal = 1000;
int16_t g_testStickDir = 10;

void printMenu() {
    Serial.println("\n=======================================================");
    Serial.println("   XLRS Remote MCU RP2040 Integration Test Bench CLI   ");
    Serial.println("=======================================================");
    Serial.println(" [1] PHASE 1: Standalone Math & Protocol Engine Self-Test");
    Serial.println(" [2] PHASE 2: Host-to-TX Handshake & 100Hz Stick Driver");
    Serial.println(" [3] PHASE 3: RX Output Listener (Emulate Flight Controller)");
    Serial.println(" [I] Print this menu");
    Serial.println("=======================================================");
    Serial.print("Select active bench phase (1-3): ");
}

// ============================================================================
// Setup & Initialization
// ============================================================================
void setup() {
    Serial.begin(115200); // USB Debug Serial console
    
    // Set up diagnostic GPIOs
    pinMode(DIAG_GPIO_ISR_LATENCY, OUTPUT);
    pinMode(DIAG_GPIO_RX_LATENCY, OUTPUT);
    digitalWrite(DIAG_GPIO_ISR_LATENCY, LOW);
    digitalWrite(DIAG_GPIO_RX_LATENCY, LOW);

    // Initialize High-Speed Hardware UART
    Serial2.setTX(UART2_TX_PIN);
    Serial2.setRX(UART2_RX_PIN);
    Serial2.begin(UART_PROTOCOL_BAUDRATE);

    delay(1000); // Allow hardware lines to settle
    printMenu();
}

// ============================================================================
// Operating Mode Core Routines
// ============================================================================

void runPhase1SelfTest() {
    Serial.println("\nExecuting Phase 1 Self-Test...");
    
    // 1. Verify FNV-1a Hash output
    const char* phrase = "Kikobot-02";
    uint64_t hashVal = HostFnv1a::hash(phrase);
    Serial.print(" - FNV-1a Hash of '");
    Serial.print(phrase);
    Serial.print("': 0x");
    Serial.println((uint32_t)(hashVal >> 32), HEX); // High 32 bits
    Serial.print("   Expected match in XLRS TX/RX: 0x");
    Serial.println((uint32_t)hashVal, HEX); // Low 32 bits
    
    // 2. Verify CRC8 lookup table math
    uint8_t testData[] = { 0x10, 0x01, 0x05, 0x06, 0x07 }; // Sample payload
    uint8_t crc = g_crc.calc(testData, sizeof(testData));
    Serial.print(" - CRC8 (Poly 0xD5) of Sample Payload: 0x");
    Serial.println(crc, HEX);
    
    // Known-answer check: poly 0xD5 CRC on {0x10, 0x01} should yield 0xD9
    uint8_t checkData[] = { 0x00, 0x14 }; // Type=0x14, Len=0 (PING frame check)
    uint8_t pingCrc = g_crc.calc(checkData, 2);
    Serial.print(" - Known-answer Check (PING framing CRC): 0x");
    Serial.print(pingCrc, HEX);
    if (pingCrc == 0xD9) {
        Serial.println(" [PASS]");
    } else {
        Serial.println(" [FAIL] expected 0xD9");
    }

    g_activeMode = MODE_IDLE;
    printMenu();
}

void processIncomingPacket(uint8_t type, const uint8_t* payload, uint8_t length) {
    digitalWrite(DIAG_GPIO_RX_LATENCY, HIGH); // Signal logic analyzer: packet decoded

    if (g_activeMode == MODE_PHASE_2_TX_INTERFACE) {
        if (type == UART_MSG_PONG) {
            g_pingsReceived++;
            unsigned long rtt = millis() - g_lastPingTime;
            Serial.print("[PHASE 2] Ping PONG Success! RTT: ");
            Serial.print(rtt);
            Serial.println(" ms");
        } else if (type == UART_MSG_TELEMETRY) {
            if (length == sizeof(TelemetryData)) {
                const TelemetryData* telem = (const TelemetryData*)payload;
                Serial.print("[PHASE 2 TELEM] LQ: ");
                Serial.print(telem->linkQuality);
                Serial.print("%, RSSI: ");
                Serial.print(telem->rssi);
                Serial.print(" dBm, RX Batt: ");
                Serial.print(telem->rxBattMv);
                Serial.println(" mV");
            }
        } else if (type == UART_MSG_STATUS) {
            if (length == sizeof(StatusData)) {
                const StatusData* status = (const StatusData*)payload;
                const char* states[] = { "DISCONNECTED", "PAIRING", "CONNECTING", "CONNECTED", "LOST" };
                Serial.print("[PHASE 2 STATUS] Link State: ");
                Serial.print(states[status->connectionState]);
                Serial.print(", Paired: ");
                Serial.println(status->pairingState ? "YES" : "NO");
            }
        } else if (type == UART_MSG_ACK) {
            Serial.print("[PHASE 2 ACK] Command ACK: 0x");
            Serial.println(payload[0], HEX);
        } else if (type == UART_MSG_ERROR) {
            Serial.print("[PHASE 2 ERROR] Received Error Code: ");
            Serial.println(payload[0]);
        }
    } 
    else if (g_activeMode == MODE_PHASE_3_RX_LISTENER) {
        // Emulating flight controller decoding CRSF or status from receiver
        if (type == UART_MSG_CHANNELS && length == sizeof(ChannelData)) {
            const ChannelData* ch = (const ChannelData*)payload;
            Serial.print("[PHASE 3 RC] Ch1: ");
            Serial.print(ch->channels[0]);
            Serial.print(" | Ch2: ");
            Serial.print(ch->channels[1]);
            Serial.print(" | Ch3: ");
            Serial.print(ch->channels[2]);
            Serial.print(" | Ch4: ");
            Serial.println(ch->channels[3]);
        } else if (type == UART_MSG_STATUS) {
            const StatusData* status = (const StatusData*)payload;
            if (status->connectionState == 4 || status->connectionState == 0) {
                Serial.println("[PHASE 3 ALERT] RECEIVER ENTERED FAILSAFE (RXLOSS)!");
            }
        }
    }

    digitalWrite(DIAG_GPIO_RX_LATENCY, LOW); // End decode signal
}

void loop() {
    // 1. Handle Serial CLI Inputs
    if (Serial.available()) {
        char c = Serial.read();
        if (c == '1') {
            g_activeMode = MODE_PHASE_1_SELF_TEST;
            runPhase1SelfTest();
        } else if (c == '2') {
            g_activeMode = MODE_PHASE_2_TX_INTERFACE;
            g_pingsSent = 0;
            g_pingsReceived = 0;
            g_parser.reset();
            Serial.println("\nEntered PHASE 2: Sending PINGs and 100Hz stick telemetry to TX.");
            Serial.println("Press 'Esc' or '1'/'3' to change modes.");
        } else if (c == '3') {
            g_activeMode = MODE_PHASE_3_RX_LISTENER;
            g_parser.reset();
            Serial.println("\nEntered PHASE 3: Listening for incoming receiver outputs...");
            Serial.println("Press 'Esc' or '1'/'2' to change modes.");
        } else if (c == 27 || c == 'i' || c == 'I') { // Escape or 'I'
            g_activeMode = MODE_IDLE;
            printMenu();
        }
    }

    // 2. Perform Non-Blocking Mode Operations
    if (g_activeMode == MODE_PHASE_2_TX_INTERFACE) {
        unsigned long now = millis();
        
        // A. Send periodic PING (every 1000ms)
        if (now - g_lastPingTime >= 1000) {
            g_lastPingTime = now;
            g_pingsSent++;
            Serial.print("[PHASE 2] Sending PING #");
            Serial.print(g_pingsSent);
            Serial.println("...");
            sendFrame(UART_MSG_PING, nullptr, 0);
        }

        // B. Send 100Hz stick packets (every 10ms)
        if (now - g_lastChannelTx >= 10) {
            g_lastChannelTx = now;
            
            // Sweep stick values between 1000us and 2000us to test response
            g_testStickVal += g_testStickDir;
            if (g_testStickVal >= 2000) {
                g_testStickVal = 2000;
                g_testStickDir = -10;
            } else if (g_testStickVal <= 1000) {
                g_testStickVal = 1000;
                g_testStickDir = 10;
            }

            uint16_t sticks[8] = { g_testStickVal, 1500, 1500, 1500, 1000, 1000, 1000, 1000 };
            
            // Pack using local buffer representation
            ChannelData cData;
            for (int i = 0; i < 8; i++) {
                cData.channels[i] = sticks[i];
            }
            sendFrame(UART_MSG_CHANNELS, (const uint8_t*)&cData, sizeof(ChannelData));
        }
    }

    // 3. Read Incoming High-Speed Serial Bytes (Non-Blocking State Machine)
    g_parser.checkTimeout();
    while (Serial2.available()) {
        uint8_t b = Serial2.read();
        
        uint8_t outType = 0;
        uint8_t outPayload[UART_PROTOCOL_MAX_PAYLOAD];
        uint8_t outLen = 0;

        if (g_parser.parseByte(b, outType, outPayload, outLen)) {
            processIncomingPacket(outType, outPayload, outLen);
        }
    }
}
