#include <Arduino.h>
#include <string.h>
#include <EEPROM.h>
#include "Protocol.h"
#include "Security.h"
#include "crsfSerial.h"
#include "crsf_protocol.h"

// Arduino-Pico uses SerialBT for SPP (Serial Port Profile)
// SerialBT is a global object declared in the core when Bluetooth is enabled
#include <SerialBT.h>
// Include BluetoothHCI to get MAC address and monitor events
#include <BluetoothHCI.h>
#include <btstack.h>

// CRSF Serial pins (connect to flight controller)
#define CRSF_TX_PIN 8   // GP8 - TX to flight controller
#define CRSF_RX_PIN 9   // GP9 - RX from flight controller (optional, not used for TX-only)

// Bluetooth configuration
// SerialBT is available globally in Arduino-Pico
const char* BT_DEVICE_NAME_PREFIX = "FPV_RX_";
char BT_DEVICE_NAME[32];
char BT_PIN[7] = "000000";  // 6 hex digits + null terminator

// EEPROM addresses for Bluetooth config
#define EEPROM_SIZE 512
#define BT_PIN_ADDR 200
#define BT_NAME_SUFFIX_ADDR 220
#define BT_NAME_SUFFIX_LEN 3  // e.g., "001"

BluetoothHCI rxBT;
btstack_packet_callback_registration_t rx_hci_callback;

CrsfSerial crsf(Serial2, CRSF_BAUDRATE);
Security security;

// Channel values received from TX
// Default to minimum (1000) for failsafe behavior
uint16_t channels[8] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};

// Connection state
bool clientConnected = false;
unsigned long lastDataReceived = 0;
const unsigned long TIMEOUT_MS = 1000;  // 1 second timeout

// Security state
uint16_t lastSequence = 0;

#ifdef TEST_MODE
// Test mode: random channel values
unsigned long lastRandomUpdate = 0;
const unsigned long RANDOM_UPDATE_INTERVAL = 2000;  // 2 seconds
#endif

bool isValidHex(const String& str);
static void rx_hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static void rx_hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);
    if (packet_type != HCI_EVENT_PACKET) return;
    uint8_t event = packet[0];
    switch (event) {
        case HCI_EVENT_CONNECTION_COMPLETE:
            Serial.print("[BT-HCI] Connection complete, status=");
            Serial.print(packet[2]);
            Serial.print(" handle=");
            Serial.println(little_endian_read_16(packet, 3));
            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            Serial.print("[BT-HCI] Disconnected, reason=");
            Serial.println(packet[2]);
            break;
        case HCI_EVENT_PIN_CODE_REQUEST:
            Serial.println("[BT-HCI] PIN code requested by remote device");
            break;
        case HCI_EVENT_AUTHENTICATION_COMPLETE:
            Serial.print("[BT-HCI] Authentication complete, status=");
            Serial.println(packet[2]);
            break;
        case RFCOMM_EVENT_CHANNEL_OPENED:
            Serial.print("[BT-HCI] RFCOMM channel opened, status=");
            Serial.println(packet[2]);
            break;
        default:
            break;
    }
}

bool isValidHex(const String& str) {
    if (str.length() != 6) return false;
    for (int i = 0; i < 6; i++) {
        char c = str.charAt(i);
        if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f'))) {
            return false;
        }
    }
    return true;
}

void loadBTPairingCode() {
    EEPROM.begin(EEPROM_SIZE);
    
    // Load PIN from EEPROM
    bool hasPIN = false;
    for (int i = 0; i < 6; i++) {
        BT_PIN[i] = EEPROM.read(BT_PIN_ADDR + i);
        if (BT_PIN[i] != 0 && BT_PIN[i] != 0xFF) hasPIN = true;
    }
    BT_PIN[6] = '\0';
    
    if (!hasPIN) {
        // Default PIN
        strcpy(BT_PIN, "000000");
        Serial.println("No PIN found in EEPROM, using default: 000000");
    } else {
        Serial.print("Loaded PIN from EEPROM: ");
        Serial.println(BT_PIN);
    }
    
    // Load device name suffix
    char suffix[4] = "001";
    bool hasSuffix = false;
    for (int i = 0; i < BT_NAME_SUFFIX_LEN; i++) {
        suffix[i] = EEPROM.read(BT_NAME_SUFFIX_ADDR + i);
        if (suffix[i] != 0 && suffix[i] != 0xFF) hasSuffix = true;
    }
    suffix[BT_NAME_SUFFIX_LEN] = '\0';
    
    if (!hasSuffix) {
        strcpy(suffix, "001");
    }
    
    // Generate device name
    snprintf(BT_DEVICE_NAME, sizeof(BT_DEVICE_NAME), "%s%s", BT_DEVICE_NAME_PREFIX, suffix);
}

void saveBTPairingCode() {
    for (int i = 0; i < 6; i++) {
        EEPROM.write(BT_PIN_ADDR + i, BT_PIN[i]);
    }
    EEPROM.commit();
    Serial.print("Saved PIN to EEPROM: ");
    Serial.println(BT_PIN);
}

void setBTPairingCode(const String& pin) {
    if (pin.length() == 6 && isValidHex(pin)) {
        // Make a copy since pin is const and toUpperCase() modifies the string
        String pinUpper = pin;
        pinUpper.toUpperCase();
        pinUpper.toCharArray(BT_PIN, 7);
        saveBTPairingCode();
        
        // Note: BluetoothSerial.setPin() may need to be called before begin()
        // For now, we'll need to restart Bluetooth to apply new PIN
        Serial.println("PIN updated. Restart Bluetooth to apply.");
    } else {
        Serial.println("ERROR: Invalid PIN format (must be 6 hex digits)");
    }
}

void handleConfigCommands() {
    // Handle USB Serial commands for configuration
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd.startsWith("SETPIN:")) {
            String pin = cmd.substring(7);
            pin.trim();
            setBTPairingCode(pin);
            Serial.println("OK");
        } else if (cmd.startsWith("GETPIN")) {
            Serial.print("PIN: ");
            Serial.println(BT_PIN);
        } else if (cmd.startsWith("SETNAME:")) {
            String suffix = cmd.substring(8);
            suffix.trim();
            if (suffix.length() == 3) {
                for (int i = 0; i < 3; i++) {
                    EEPROM.write(BT_NAME_SUFFIX_ADDR + i, suffix.charAt(i));
                }
                EEPROM.commit();
                Serial.print("Name suffix updated. Restart to apply: FPV_RX_");
                Serial.println(suffix);
            } else {
                Serial.println("ERROR: Name suffix must be 3 characters");
            }
        } else if (cmd.startsWith("GETNAME")) {
            Serial.print("Device Name: ");
            Serial.println(BT_DEVICE_NAME);
        } else if (cmd.startsWith("HELP")) {
            Serial.println("Commands:");
            Serial.println("  SETPIN:XXXXXX  - Set 6-digit hex PIN (0-F)");
            Serial.println("  GETPIN         - Get current PIN");
            Serial.println("  SETNAME:XXX    - Set device name suffix (e.g., 001)");
            Serial.println("  GETNAME        - Get device name");
            Serial.println("  HELP           - Show this help");
        } else {
            Serial.println("ERROR: Unknown command. Type HELP for commands.");
        }
    }
}

bool handleSerialBTCommands() {
    // Handle SerialBT commands for configuration (from Flutter app)
    // Returns true if a command was processed, false otherwise
    if (SerialBT.available() > 0) {
        int available = SerialBT.available();
        
        // Commands are text and shorter than FRAME_SIZE (22 bytes)
        // If we have less than FRAME_SIZE bytes, it might be a command
        if (available < FRAME_SIZE) {
            // Peek at first byte to see if it's ASCII text (command) or binary (frame)
            // Commands start with 'G', 'S', 'H' (GETPIN, SETPIN, HELP)
            // Binary frames start with channel data (usually non-printable or specific values)
            if (available > 0) {
                // Peek at first byte to check if it's a command character
                char firstByte = SerialBT.peek();
                
                // Commands start with ASCII letters: 'G', 'S', 'H'
                // Frame data typically starts with binary values (0x00-0xFF, but usually not printable ASCII)
                // Check if first byte is a command character
                if (firstByte == 'G' || firstByte == 'S' || firstByte == 'H') {
                    // Read the data as a string to check if it's a command
                    String cmd = "";
                    int bytesToRead = min(available, 50); // Read up to 50 bytes or what's available
                    
                    for (int i = 0; i < bytesToRead; i++) {
                        if (SerialBT.available() > 0) {
                            char c = SerialBT.read();
                            cmd += c;
                            if (c == '\n' || c == '\r') {
                                break;
                            }
                        } else {
                            break;
                        }
                    }
                    cmd.trim();
                    
                    // Check if it looks like a command (starts with known command prefixes)
                    if (cmd.length() > 0 && 
                        (cmd.startsWith("GETPIN") || cmd.startsWith("SETPIN:") || 
                         cmd.startsWith("GETNAME") || cmd.startsWith("SETNAME:") || 
                         cmd.startsWith("HELP"))) {
                        // Process command
                        if (cmd.startsWith("SETPIN:")) {
                            String pin = cmd.substring(7);
                            pin.trim();
                            setBTPairingCode(pin);
                            SerialBT.println("OK");
                            Serial.print("BT Command: SETPIN:");
                            Serial.println(pin);
                        } else if (cmd.startsWith("GETPIN")) {
                            SerialBT.print("PIN: ");
                            SerialBT.println(BT_PIN);
                            Serial.print("BT Command: GETPIN -> ");
                            Serial.println(BT_PIN);
                        } else if (cmd.startsWith("SETNAME:")) {
                            String suffix = cmd.substring(8);
                            suffix.trim();
                            if (suffix.length() == 3) {
                                for (int i = 0; i < 3; i++) {
                                    EEPROM.write(BT_NAME_SUFFIX_ADDR + i, suffix.charAt(i));
                                }
                                EEPROM.commit();
                                SerialBT.print("Name suffix updated. Restart to apply: FPV_RX_");
                                SerialBT.println(suffix);
                                Serial.print("BT Command: SETNAME:");
                                Serial.println(suffix);
                            } else {
                                SerialBT.println("ERROR: Name suffix must be 3 characters");
                            }
                        } else if (cmd.startsWith("GETNAME")) {
                            SerialBT.print("Device Name: ");
                            SerialBT.println(BT_DEVICE_NAME);
                            Serial.print("BT Command: GETNAME -> ");
                            Serial.println(BT_DEVICE_NAME);
                        } else if (cmd.startsWith("HELP")) {
                            SerialBT.println("Commands:");
                            SerialBT.println("  SETPIN:XXXXXX  - Set 6-digit hex PIN (0-F)");
                            SerialBT.println("  GETPIN         - Get current PIN");
                            SerialBT.println("  SETNAME:XXX    - Set device name suffix (e.g., 001)");
                            SerialBT.println("  GETNAME        - Get device name");
                            SerialBT.println("  HELP           - Show this help");
                        }
                        
                        // Flush any remaining data in buffer after command processing
                        // This prevents leftover data from being misinterpreted as a frame
                        while (SerialBT.available() > 0) {
                            SerialBT.read(); // Discard remaining bytes
                        }
                        
                        return true; // Command processed
                    }
                    // If not a recognized command, the data was consumed but that's okay
                    // It was likely incomplete or invalid data
                }
            }
        }
    }
    return false; // No command processed
}

void setup() {
    // Initialize USB Serial FIRST (for debugging/logs)
    Serial.begin(115200);
    
    // Wait for USB Serial to be ready
    unsigned long start = millis();
    while (!Serial && (millis() - start < 2000)) {
        delay(10);
    }
    delay(500);
    Serial.flush();
    
#ifdef TEST_MODE
    randomSeed(analogRead(A0) + millis());
#endif
    
    Serial.println("FPV Receiver (Bluetooth) starting...");
    Serial.flush();
    
    // Initialize security (load or generate pairing key)
    Serial.println("Initializing security...");
    if (security.begin()) {
        Serial.println("Security initialized (pairing key loaded/generated)");
    } else {
        Serial.println("Security initialization failed!");
    }
    Serial.flush();
    
    // Load Bluetooth configuration
    loadBTPairingCode();
    
    // Initialize Bluetooth Serial
    Serial.println("Initializing Bluetooth...");
    Serial.print("Device Name: ");
    Serial.println(BT_DEVICE_NAME);
    Serial.print("PIN: ");
    Serial.println(BT_PIN);
    Serial.flush();
    
    // Note: Arduino-Pico SerialBT API
    // SerialBT.begin() may support device name as first parameter
    // Try begin(deviceName) - some versions accept name directly
    // If that doesn't compile, try begin(baud, deviceName) or use default
    Serial.println("Calling SerialBT.begin(115200)...");
    Serial.flush();
    SerialBT.begin(115200);  // Try passing device name directly
    Serial.println("SerialBT.begin() completed");
    Serial.flush();
    
    // Initialize BluetoothHCI globally for MAC discovery and event logging
    Serial.println("Initializing BluetoothHCI for connection monitoring...");
    Serial.flush();
    rxBT.install();
    rxBT.begin();
    delay(500);  // Wait for HCI to initialize
    Serial.println("BluetoothHCI initialized");
    Serial.flush();
    
    rx_hci_callback.callback = &rx_hci_packet_handler;
    hci_add_event_handler(&rx_hci_callback);
    
    Serial.println("Bluetooth initialized");
    Serial.print("Device Name (configured): ");
    Serial.println(BT_DEVICE_NAME);
    Serial.println("Device should appear as the configured name when scanning");
    Serial.println("If it appears as 'PicoW Serial', the name parameter may not be supported");
    Serial.println("In that case, identify device by MAC address: 28:CD:C1:04:BD:89");
    Serial.println("Bluetooth is now discoverable and ready to accept connections");
    Serial.println("Waiting for incoming RFCOMM/SPP connections...");
    Serial.flush();
    
    Serial.flush();
    
    // Initialize CRSF serial output on Serial2
    Serial.println("Initializing CRSF Serial2...");
    Serial.flush();
    
    Serial2.setTX(CRSF_TX_PIN);
    Serial2.setRX(CRSF_RX_PIN);
    
    Serial.println("Serial2 pins set");
    Serial.flush();
    
    crsf.begin(CRSF_BAUDRATE);
    
    Serial.println("crsf.begin() completed");
    Serial.flush();
    
    Serial.print("CRSF initialized on Serial2 (hardware UART): TX=GP");
    Serial.print(CRSF_TX_PIN);
    Serial.print(", RX=GP");
    Serial.println(CRSF_RX_PIN);
    Serial.print("Baud rate: ");
    Serial.println(CRSF_BAUDRATE);
    Serial.println("Note: Serial (USB) and Serial2 (UART) are completely separate");
    Serial.flush();
    
    // Send a test CRSF packet
    Serial.println("Sending test CRSF packet...");
    crsf_channels_t testChannels = {0};
    testChannels.ch0 = CRSF_CHANNEL_VALUE_MID;
    testChannels.ch1 = CRSF_CHANNEL_VALUE_MID;
    testChannels.ch2 = CRSF_CHANNEL_VALUE_MID;
    testChannels.ch3 = CRSF_CHANNEL_VALUE_MID;
    testChannels.ch4 = CRSF_CHANNEL_VALUE_MID;
    testChannels.ch5 = CRSF_CHANNEL_VALUE_MID;
    testChannels.ch6 = CRSF_CHANNEL_VALUE_MID;
    testChannels.ch7 = CRSF_CHANNEL_VALUE_MID;
    testChannels.ch8 = CRSF_CHANNEL_VALUE_MID;
    testChannels.ch9 = CRSF_CHANNEL_VALUE_MID;
    testChannels.ch10 = CRSF_CHANNEL_VALUE_MID;
    testChannels.ch11 = CRSF_CHANNEL_VALUE_MID;
    testChannels.ch12 = CRSF_CHANNEL_VALUE_MID;
    testChannels.ch13 = CRSF_CHANNEL_VALUE_MID;
    testChannels.ch14 = CRSF_CHANNEL_VALUE_MID;
    testChannels.ch15 = CRSF_CHANNEL_VALUE_MID;
    
    crsf.queuePacket(
        CRSF_ADDRESS_FLIGHT_CONTROLLER,
        CRSF_FRAMETYPE_RC_CHANNELS_PACKED,
        &testChannels,
        CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE
    );
    Serial.println("Test CRSF packet sent!");
    Serial.flush();
    
    Serial.println("Channel mapping: AETR1234 (Aileron, Elevator, Throttle, Rudder, Aux1-4)");
    Serial.println("RX ready, waiting for TX connection...");
    Serial.flush();
    
#ifdef TEST_MODE
    Serial.println("TEST MODE: Generating random channel values every 2 seconds");
    Serial.flush();
#endif
}

void loop() {
    // Handle configuration commands from USB Serial
    handleConfigCommands();
    
    // Debug: Print heartbeat every 5 seconds
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 5000) {
        lastHeartbeat = millis();
        Serial.print("Loop running, millis=");
        Serial.print(millis());
        Serial.print(", BT Connected: ");
        Serial.print(clientConnected ? "Yes" : "No");
        Serial.print(", SerialBT.available(): ");
        Serial.print(SerialBT.available());
        Serial.print(", Last data received: ");
        if (lastDataReceived > 0) {
            Serial.print(millis() - lastDataReceived);
            Serial.println(" ms ago");
        } else {
            Serial.println("Never");
        }
        Serial.flush();
    }
    
#ifdef TEST_MODE
    // Test mode: Generate random channel values every 2 seconds
    if (millis() - lastRandomUpdate >= RANDOM_UPDATE_INTERVAL) {
        lastRandomUpdate = millis();
        
        for (int i = 0; i < 8; i++) {
            channels[i] = random(1000, 2001);
        }
        
        Serial.print("Random channels [AETR1234]: ");
        Serial.print("A="); Serial.print(channels[0]);
        Serial.print(" E="); Serial.print(channels[1]);
        Serial.print(" T="); Serial.print(channels[2]);
        Serial.print(" R="); Serial.print(channels[3]);
        Serial.print(" Aux1="); Serial.print(channels[4]);
        Serial.print(" Aux2="); Serial.print(channels[5]);
        Serial.print(" Aux3="); Serial.print(channels[6]);
        Serial.print(" Aux4="); Serial.println(channels[7]);
    }
#endif
    
    // Check for Bluetooth client connection
    // Since SerialBT doesn't have hasClient(), check by seeing if data is available
    // or by tracking connection state through data reception
    int availableBytes = SerialBT.available();
    
    // Log connection state changes
    static bool lastConnectionState = false;
    static unsigned long lastConnectionLog = 0;
    
    if (availableBytes > 0 || clientConnected) {
        if (!clientConnected && availableBytes > 0) {
            Serial.print("*** NEW BLUETOOTH CLIENT CONNECTED ***");
            Serial.print(" (available bytes: ");
            Serial.print(availableBytes);
            Serial.println(")");
            Serial.flush();
            clientConnected = true;
            lastConnectionState = true;
            lastConnectionLog = millis();
        }
        
        // Log periodic connection status
        if (clientConnected && millis() - lastConnectionLog > 5000) {
            Serial.print("BT Client connected, available bytes: ");
            Serial.println(availableBytes);
            Serial.flush();
            lastConnectionLog = millis();
        }
        
        // First, check for text commands (GETPIN, SETPIN, etc.)
        // Commands are shorter than FRAME_SIZE, so check before reading frames
        // Only process commands if we have less than FRAME_SIZE bytes
        if (SerialBT.available() < FRAME_SIZE) {
            if (handleSerialBTCommands()) {
                // Command was processed, skip frame processing
                return;
            }
        }
        
        // Then check for channel data frames (exactly FRAME_SIZE bytes)
        // Only read frame if we have exactly FRAME_SIZE bytes and first byte doesn't look like a command
        if (SerialBT.available() >= FRAME_SIZE) {
            // Peek at first byte to ensure it's not a command character
            // This prevents reading a frame if a command is being sent
            char firstByte = SerialBT.peek();
            
            // If first byte is a command character, don't read as frame
            // Wait for more data or command completion
            if (firstByte == 'G' || firstByte == 'S' || firstByte == 'H') {
                // Likely a command, let it accumulate
                return;
            }
            
            uint8_t frame[FRAME_SIZE];
            size_t bytesRead = SerialBT.readBytes(frame, FRAME_SIZE);
            
            if (bytesRead == FRAME_SIZE) {
                // Log frame reception occasionally
                static unsigned long lastFrameLog = 0;
                if (millis() - lastFrameLog > 2000) {
                    Serial.print("Received frame (");
                    Serial.print(bytesRead);
                    Serial.println(" bytes)");
                    Serial.flush();
                    lastFrameLog = millis();
                }
                
                // Decode frame with security validation
                uint16_t sequence = 0;
                if (Protocol::decodeFrame(frame, channels, &sequence, &security, &lastSequence)) {
#ifdef TEST_MODE
                    Serial.println("Real data received - test mode still active");
#endif
                    
                    lastDataReceived = millis();
                    
                    // Debug: print channel values occasionally
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
                } else {
                    Serial.println("Invalid frame received (security check failed or invalid data)");
                }
            }
        }
        
        // Check for timeout
        if (millis() - lastDataReceived > TIMEOUT_MS && lastDataReceived > 0) {
            Serial.print("*** CLIENT TIMEOUT *** (no data for ");
            Serial.print(millis() - lastDataReceived);
            Serial.println(" ms)");
            Serial.flush();
            // SerialBT doesn't have disconnect() - connection will timeout naturally
            clientConnected = false;
            lastConnectionState = false;
            // Set channels to failsafe (minimum position)
            for (int i = 0; i < 8; i++) {
                channels[i] = 1000;
            }
        }
    } else {
        if (clientConnected) {
            Serial.println("*** BLUETOOTH CLIENT DISCONNECTED ***");
            Serial.flush();
            clientConnected = false;
            lastConnectionState = false;
            // Set channels to failsafe (minimum position)
            for (int i = 0; i < 8; i++) {
                channels[i] = 1000;
            }
        }
    }
    
    // Log connection state changes
    if (lastConnectionState != clientConnected) {
        if (clientConnected) {
            Serial.println(">>> Connection state changed: CONNECTED");
        } else {
            Serial.println(">>> Connection state changed: DISCONNECTED");
        }
        Serial.flush();
        lastConnectionState = clientConnected;
    }
    
    // Process CRSF serial input (for receiving telemetry, etc.)
    crsf.loop();
    
    // Send CRSF channels continuously at ~50Hz (20ms intervals)
    static unsigned long lastCRSFUpdate = 0;
    const unsigned long CRSF_INTERVAL = 20;  // ~50Hz (20ms)
    
    if (millis() - lastCRSFUpdate >= CRSF_INTERVAL) {
        lastCRSFUpdate = millis();
        
        // Convert channels from 1000-2000us to CRSF 11-bit format
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
        
        // Fill remaining channels with center value
        uint16_t centerCRSF = CRSF_CHANNEL_VALUE_MID;
        crsfChannels.ch8 = centerCRSF;
        crsfChannels.ch9 = centerCRSF;
        crsfChannels.ch10 = centerCRSF;
        crsfChannels.ch11 = centerCRSF;
        crsfChannels.ch12 = centerCRSF;
        crsfChannels.ch13 = centerCRSF;
        crsfChannels.ch14 = centerCRSF;
        crsfChannels.ch15 = centerCRSF;
        
        // Send CRSF RC channels packet
        crsf.queuePacket(
            CRSF_ADDRESS_FLIGHT_CONTROLLER,
            CRSF_FRAMETYPE_RC_CHANNELS_PACKED,
            &crsfChannels,
            CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE
        );
        
        // Debug: Print every 50 packets (once per second)
        static uint16_t packetCount = 0;
        packetCount++;
        if (packetCount % 50 == 0) {
            Serial.print("Sent ");
            Serial.print(packetCount);
            Serial.println(" CRSF packets");
            Serial.flush();
        }
    }
}

