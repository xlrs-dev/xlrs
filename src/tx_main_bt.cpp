#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ADS1X15.h>
#include <ctype.h>
#include "Protocol.h"
#include "Security.h"
// Bluetooth includes
#include <BluetoothHCI.h>
#include <SerialBT.h>
#include <btstack.h>
#include <l2cap.h>
#include <gap.h>
#include <classic/sdp_client.h>
// SerialBT is server-only in Arduino-Pico
// For client mode, we need to use btstack SPP client API directly

// OLED Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define I2C_ADDRESS 0x3C

// ADS1115 Configuration
#define ADS1115_ADDRESS 0x48  // Default I2C address (can be 0x48-0x4B)

// Pin definitions
#define I2C_SDA 4
#define I2C_SCL 5

// ADS1115 channel assignments (0-3)
#define STICK1_X_CHANNEL 0
#define STICK1_Y_CHANNEL 1
#define STICK2_X_CHANNEL 2
#define STICK2_Y_CHANNEL 3

// Button pins (4 buttons)
#define BUTTON1_PIN 6   // GP6
#define BUTTON2_PIN 7   // GP7
#define BUTTON3_PIN 8   // GP8
#define BUTTON4_PIN 9   // GP9

// Menu system
enum MenuState {
    MENU_NORMAL_OPERATION,
    MENU_MAIN,
    MENU_PAIRING_SCAN,
    MENU_PAIRING_SELECT_DEVICE,
    MENU_PAIRING_ENTER_PIN,
    MENU_PAIRING_CONNECTING
};

MenuState currentMenuState = MENU_NORMAL_OPERATION;

// Bluetooth
// SerialBT is available globally in Arduino-Pico (declared in core)
BluetoothHCI btHCI;

// Forward declarations
void connectToDevice(const uint8_t* address, const char* pin);

// Device scanning
struct BTDevice {
    String name;
    uint8_t address[6];
    int rssi;
    bool isClassic;
};
BTDevice foundDevices[10];
int deviceCount = 0;
int selectedDeviceIndex = 0;
bool scanning = false;
unsigned long scanStartTime = 0;
const unsigned long SCAN_TIMEOUT = 10000;  // 10 seconds

// EEPROM addresses for default RX device
#define EEPROM_SIZE 512
#define DEFAULT_RX_MAGIC_ADDR 99   // 1 byte magic value to validate EEPROM data
#define DEFAULT_RX_MAC_ADDR 100   // 6 bytes for MAC address
#define DEFAULT_RX_PIN_ADDR 110   // 6 bytes for PIN (hex string)
#define DEFAULT_RX_MAGIC_VALUE 0xAA  // Magic byte to validate EEPROM data

// Default RX device (will be loaded from EEPROM or set to defaults)
uint8_t defaultRXMac[6] = {0x28, 0xCD, 0xC1, 0x04, 0xBD, 0x89};  // 28:cd:c1:04:bd:89
char defaultRXPin[7] = "A123FF";  // Default PIN

// PIN entry
char pairingPIN[7] = "000000";
int pinDigitIndex = 0;
char hexChars[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
int currentHexCharIndex = 0;

// Navigation state
struct NavState {
    float x, y;
    bool center;
    unsigned long lastChange;
};
NavState stick1Nav, stick2Nav;
const float NAV_THRESHOLD = 0.3;  // Dead zone
const unsigned long NAV_DEBOUNCE = 200;  // ms

// Display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_ADS1115 ads;  // ADS1115 ADC on I2C bus
Security security;

// Channel values (8 channels: 4 from sticks + 4 from buttons)
uint16_t channels[8] = {1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000};

// Connection state
bool connected = false;
unsigned long lastDataSent = 0;
int rssi = 0;
String connectionStatus = "Disconnected";

// Auto-connect state
bool autoConnectEnabled = false;
bool autoConnectInProgress = false;
unsigned long autoConnectStartTime = 0;
const unsigned long AUTO_CONNECT_TIMEOUT = 60000;  // 1 minute

// Connection state tracking (declared early for access in handleButton1)
uint16_t spp_client_channel_id = 0;  // RFCOMM channel ID for SPP client connection

// Security state
uint16_t sequenceNumber = 0;

// Timing
const unsigned long DATA_INTERVAL = 20;       // Send data every 20ms (50Hz)
const unsigned long DISPLAY_UPDATE_INTERVAL = 100;  // Update display every 100ms

// Forward declarations
void saveDefaultRXDevice();

bool loadDefaultRXDevice() {
    EEPROM.begin(EEPROM_SIZE);
    
    // Check magic byte to validate EEPROM data
    uint8_t magic = EEPROM.read(DEFAULT_RX_MAGIC_ADDR);
    
    if (magic != DEFAULT_RX_MAGIC_VALUE) {
        // Magic byte doesn't match - EEPROM data is invalid or uninitialized
        Serial.println("EEPROM magic byte invalid - resetting to defaults");
        Serial.print("Expected: 0x");
        Serial.print(DEFAULT_RX_MAGIC_VALUE, HEX);
        Serial.print(", Got: 0x");
        Serial.println(magic, HEX);
        
        // Save defaults with correct magic byte
        saveDefaultRXDevice();
        return true;  // Return true so we can auto-connect with defaults
    }
    
    // Magic byte is valid, load data from EEPROM
    Serial.println("EEPROM magic byte valid, loading stored device");
    
    // Load default MAC address from EEPROM
    for (int i = 0; i < 6; i++) {
        defaultRXMac[i] = EEPROM.read(DEFAULT_RX_MAC_ADDR + i);
    }
    
    Serial.print("Loaded default RX MAC from EEPROM: ");
    for (int i = 0; i < 6; i++) {
        if (i > 0) Serial.print(":");
        if (defaultRXMac[i] < 0x10) Serial.print("0");
        Serial.print(defaultRXMac[i], HEX);
    }
    Serial.println();
    
    // Load default PIN from EEPROM
    for (int i = 0; i < 6; i++) {
        defaultRXPin[i] = EEPROM.read(DEFAULT_RX_PIN_ADDR + i);
    }
    defaultRXPin[6] = '\0';
    
    Serial.print("Loaded default RX PIN from EEPROM: ");
    Serial.println(defaultRXPin);
    
    return true;  // Both MAC and PIN loaded, can auto-connect
}

void saveDefaultRXDevice() {
    // Write magic byte first to mark EEPROM as valid
    EEPROM.write(DEFAULT_RX_MAGIC_ADDR, DEFAULT_RX_MAGIC_VALUE);
    
    // Save default MAC address
    for (int i = 0; i < 6; i++) {
        EEPROM.write(DEFAULT_RX_MAC_ADDR + i, defaultRXMac[i]);
    }
    
    // Save default PIN
    for (int i = 0; i < 6; i++) {
        EEPROM.write(DEFAULT_RX_PIN_ADDR + i, defaultRXPin[i]);
    }
    
    EEPROM.commit();
    
    Serial.println("Saved default RX device to EEPROM:");
    Serial.print("  MAC: ");
    for (int i = 0; i < 6; i++) {
        if (i > 0) Serial.print(":");
        if (defaultRXMac[i] < 0x10) Serial.print("0");
        Serial.print(defaultRXMac[i], HEX);
    }
    Serial.println();
    Serial.print("  PIN: ");
    Serial.println(defaultRXPin);
    Serial.print("  Magic: 0x");
    Serial.println(DEFAULT_RX_MAGIC_VALUE, HEX);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // Initialize I2C for OLED and ADS1115
    Wire.setSDA(I2C_SDA);
    Wire.setSCL(I2C_SCL);
    Wire.begin();
    
    // Set I2C bus speed to 400kHz (fast mode) for faster ADC reads
    // This allows us to read 4 channels quickly within our 20ms update window
    Wire.setClock(400000);  // 400kHz fast mode
    Serial.println("I2C bus set to 400kHz");
    
    // Initialize OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, I2C_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;); // Don't proceed if OLED init fails
    }
    display.display();
    delay(2000);
    display.clearDisplay();
    
    // Initialize ADS1115
    if (!ads.begin(ADS1115_ADDRESS, &Wire)) {
        Serial.println(F("Failed to initialize ADS1115"));
        Serial.println(F("Check wiring and I2C address"));
        // Continue anyway - will read 0 values
    } else {
        Serial.println(F("ADS1115 initialized"));
        // Set gain to 1 (default, ±4.096V range)
        // This gives us full 16-bit resolution (0-32767)
        ads.setGain(GAIN_ONE);
        
        // Set data rate to 250 SPS (Samples Per Second)
        // We need to read 4 channels at 50Hz (20ms intervals)
        // Minimum required: 4 channels * 50Hz = 200 SPS
        // Using 250 SPS provides headroom and ensures fast reads
        ads.setDataRate(RATE_ADS1115_250SPS);
        Serial.println(F("ADS1115 configured: Gain=1, DataRate=250 SPS"));
    }
    
    // Load default RX device from EEPROM (or initialize with defaults)
    Serial.println("Loading default RX device...");
    bool hasStoredDevice = loadDefaultRXDevice();
    
    // Initialize security
    Serial.println("Initializing security...");
    if (security.begin()) {
        Serial.println("Security initialized");
    } else {
        Serial.println("Security initialization failed!");
    }
    
    // Initialize input pins
    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);
    pinMode(BUTTON3_PIN, INPUT_PULLUP);
    pinMode(BUTTON4_PIN, INPUT_PULLUP);
    
    // Initialize Bluetooth HCI for scanning
    Serial.println("Initializing Bluetooth HCI...");
    // install() initializes the Bluetooth stack (requires BLE components even for Classic)
    btHCI.install();
    btHCI.begin();  // begin() returns void, not bool
    delay(1000);  // Wait for HCI to initialize
    Serial.println("Bluetooth HCI initialized");
    
    // Initialize SDP client (needed before performing SDP queries)
    sdp_client_init();
    Serial.println("SDP client initialized");
    
    // Initialize Bluetooth Serial (client mode)
    // Note: Arduino-Pico SerialBT API - SerialBT.begin() initializes in server mode
    // For client mode, we may need to use different API or initialize on connect
    Serial.println("Initializing Bluetooth Serial...");
    // SerialBT is global - will be initialized when connecting
    Serial.println("Bluetooth Serial ready (will initialize on connect)");
    
    // Display startup message
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("FPV TX BT Starting...");
    display.display();
    
    Serial.println("FPV Transmitter (Bluetooth) initialized");
    
    // If we have a stored device, enable auto-connect
    if (hasStoredDevice) {
        Serial.println("Stored RX device found, starting auto-connect...");
        autoConnectEnabled = true;
        autoConnectInProgress = true;
        autoConnectStartTime = millis();
        currentMenuState = MENU_PAIRING_CONNECTING;
        connectionStatus = "Auto-connecting...";
        
        // Start connection attempt
        connectToDevice(defaultRXMac, defaultRXPin);
    } else {
        Serial.println("No stored RX device, entering normal operation");
        currentMenuState = MENU_NORMAL_OPERATION;
    }
    
    delay(500);
}

void readInputs() {
    // Read analog sticks from ADS1115 (16-bit ADC: 0-32767)
    int16_t stick1_x = ads.readADC_SingleEnded(STICK1_X_CHANNEL);
    int16_t stick1_y = ads.readADC_SingleEnded(STICK1_Y_CHANNEL);
    int16_t stick2_x = ads.readADC_SingleEnded(STICK2_X_CHANNEL);
    int16_t stick2_y = ads.readADC_SingleEnded(STICK2_Y_CHANNEL);
    
    // Normalize to -1.0 to 1.0 (center at 16384 for 16-bit)
    stick1Nav.x = (stick1_x - 16384) / 16384.0;
    stick1Nav.y = (stick1_y - 16384) / 16384.0;
    stick2Nav.x = (stick2_x - 16384) / 16384.0;
    stick2Nav.y = (stick2_y - 16384) / 16384.0;
    
    // Check if centered
    stick1Nav.center = (abs(stick1Nav.x) < NAV_THRESHOLD && abs(stick1Nav.y) < NAV_THRESHOLD);
    stick2Nav.center = (abs(stick2Nav.x) < NAV_THRESHOLD && abs(stick2Nav.y) < NAV_THRESHOLD);
    
    // Only update channels in normal operation mode
    if (currentMenuState == MENU_NORMAL_OPERATION) {
        // Map from actual ADC range to desired RC range
        // Calibrated based on actual ADC readings:
        // ADC range appears to be ~2917 (min) to ~23420 (max), center ~13199
        // This maps to: 1089 (min) -> 1000, 1403 (center) -> 1500, 1715 (max) -> 2000
        channels[0] = map(stick1_x, 2917, 23420, 1000, 2000);  // Stick 1 X
        channels[1] = map(stick1_y, 2917, 23420, 1000, 2000);  // Stick 1 Y
        channels[2] = map(stick2_x, 2917, 23420, 1000, 2000);  // Stick 2 X
        channels[3] = map(stick2_y, 2917, 23420, 1000, 2000);  // Stick 2 Y
        
        // Clamp to valid RC range
        channels[0] = constrain(channels[0], 1000, 2000);
        channels[1] = constrain(channels[1], 1000, 2000);
        channels[2] = constrain(channels[2], 1000, 2000);
        channels[3] = constrain(channels[3], 1000, 2000);
        
        // Read buttons (inverted because of pull-up)
        bool btn1 = digitalRead(BUTTON1_PIN) == LOW;
        bool btn2 = digitalRead(BUTTON2_PIN) == LOW;
        bool btn3 = digitalRead(BUTTON3_PIN) == LOW;
        bool btn4 = digitalRead(BUTTON4_PIN) == LOW;
        
        channels[4] = btn1 ? 2000 : 1000;  // Button 1
        channels[5] = btn2 ? 2000 : 1000;  // Button 2
        channels[6] = btn3 ? 2000 : 1000;  // Button 3
        channels[7] = btn4 ? 2000 : 1000;  // Button 4
        
        // Log button states when they change
        static bool lastBtn1 = false, lastBtn2 = false, lastBtn3 = false, lastBtn4 = false;
        static unsigned long lastButtonLog = 0;
        
        if (btn1 != lastBtn1 || btn2 != lastBtn2 || btn3 != lastBtn3 || btn4 != lastBtn4) {
            Serial.print("[BUTTONS] ");
            Serial.print("B1(GP" + String(BUTTON1_PIN) + "):" + String(btn1 ? "PRESSED" : "released") + " ");
            Serial.print("B2(GP" + String(BUTTON2_PIN) + "):" + String(btn2 ? "PRESSED" : "released") + " ");
            Serial.print("B3(GP" + String(BUTTON3_PIN) + "):" + String(btn3 ? "PRESSED" : "released") + " ");
            Serial.print("B4(GP" + String(BUTTON4_PIN) + "):" + String(btn4 ? "PRESSED" : "released"));
            Serial.print(" -> Ch4:" + String(channels[4]) + " Ch5:" + String(channels[5]) + " Ch6:" + String(channels[6]) + " Ch7:" + String(channels[7]));
            Serial.println();
            lastBtn1 = btn1;
            lastBtn2 = btn2;
            lastBtn3 = btn3;
            lastBtn4 = btn4;
            lastButtonLog = millis();
        } else if (millis() - lastButtonLog > 2000) {
            // Log current state every 2 seconds even if no change
            Serial.print("[BUTTONS] ");
            Serial.print("B1(GP" + String(BUTTON1_PIN) + "):" + String(btn1 ? "PRESSED" : "released") + " ");
            Serial.print("B2(GP" + String(BUTTON2_PIN) + "):" + String(btn2 ? "PRESSED" : "released") + " ");
            Serial.print("B3(GP" + String(BUTTON3_PIN) + "):" + String(btn3 ? "PRESSED" : "released") + " ");
            Serial.print("B4(GP" + String(BUTTON4_PIN) + "):" + String(btn4 ? "PRESSED" : "released"));
            Serial.println();
            lastButtonLog = millis();
        }
    }
}

void scanForDevices() {
    if (scanning) {
        // Check if scan is complete
        if (millis() - scanStartTime > SCAN_TIMEOUT) {
            scanning = false;
            Serial.print("Scan complete. Found ");
            Serial.print(deviceCount);
            Serial.println(" devices");
            
            if (deviceCount > 0) {
                currentMenuState = MENU_PAIRING_SELECT_DEVICE;
            } else {
                currentMenuState = MENU_MAIN;
                connectionStatus = "No devices found";
            }
            return;
        }
        
        // Continue scanning
        // Note: BluetoothHCI::scan() may need to be called differently
        // Check Arduino-Pico docs for exact API
        return;
    }
    
    // Start new scan
    Serial.println("Starting device scan...");
    scanning = true;
    scanStartTime = millis();
    deviceCount = 0;
    
    // Wait for HCI to be ready (hciRunning() is private, so wait a bit)
    delay(2000);  // Give HCI time to initialize
    
    // Scan for devices (Classic Bluetooth)
    // Mask: 0xFFFFFFFF to scan all device classes
    // Note: Check Arduino-Pico docs for exact scan() API signature
    std::vector<BTDeviceInfo> devices = btHCI.scan(0xFFFFFFFF, 5, false);  // 5 second scan, synchronous
    
    deviceCount = 0;
    for (auto& device : devices) {
        if (deviceCount >= 10) break;  // Limit to 10 devices
        
        String name = device.name();
        // Filter for FPV_RX devices
        if (name.startsWith("FPV_RX_")) {
            foundDevices[deviceCount].name = name;
            memcpy(foundDevices[deviceCount].address, device.address(), 6);
            foundDevices[deviceCount].rssi = device.rssi();
            foundDevices[deviceCount].isClassic = true;
            deviceCount++;
            
            Serial.print("Found: ");
            Serial.print(name);
            Serial.print(" RSSI: ");
            Serial.println(device.rssi());
        }
    }
    
    scanning = false;
    
    if (deviceCount > 0) {
        currentMenuState = MENU_PAIRING_SELECT_DEVICE;
        selectedDeviceIndex = 0;
    } else {
        currentMenuState = MENU_MAIN;
        connectionStatus = "No RX devices found";
    }
}

// Global menu index for main menu (shared between handleMainMenu and handleButton2)
static int mainMenuIndex = 0;

void handleMainMenu(float nav_x, float nav_y) {
    static unsigned long lastNav = 0;
    
    if (abs(nav_y) > NAV_THRESHOLD && (millis() - lastNav > NAV_DEBOUNCE)) {
        if (nav_y > 0) {
            mainMenuIndex = (mainMenuIndex + 1) % 3;  // Quick Connect, Pairing Mode, Exit
        } else {
            mainMenuIndex = (mainMenuIndex - 1 + 3) % 3;
        }
        lastNav = millis();
    }
}

void handleDeviceSelection(float nav_y) {
    static unsigned long lastNav = 0;
    
    if (abs(nav_y) > NAV_THRESHOLD && (millis() - lastNav > NAV_DEBOUNCE)) {
        if (nav_y > 0 && selectedDeviceIndex < deviceCount - 1) {
            selectedDeviceIndex++;
        } else if (nav_y < 0 && selectedDeviceIndex > 0) {
            selectedDeviceIndex--;
        }
        lastNav = millis();
    }
}

void handlePINEntry(float nav_x, float nav_y, float select_x, float select_y) {
    static unsigned long lastNav = 0;
    static unsigned long lastSelect = 0;
    
    // Stick 1: Navigate between digit positions (left/right)
    if (abs(nav_x) > NAV_THRESHOLD && (millis() - lastNav > NAV_DEBOUNCE)) {
        if (nav_x > 0 && pinDigitIndex < 5) {
            pinDigitIndex++;
        } else if (nav_x < 0 && pinDigitIndex > 0) {
            pinDigitIndex--;
        }
        lastNav = millis();
        
        // Update current hex char index to match current digit
        for (int i = 0; i < 16; i++) {
            if (hexChars[i] == pairingPIN[pinDigitIndex]) {
                currentHexCharIndex = i;
                break;
            }
        }
    }
    
    // Stick 2: Scroll through hex characters (up/down)
    if (abs(select_y) > NAV_THRESHOLD && (millis() - lastSelect > NAV_DEBOUNCE)) {
        if (select_y > 0) {
            currentHexCharIndex = (currentHexCharIndex + 1) % 16;
        } else {
            currentHexCharIndex = (currentHexCharIndex - 1 + 16) % 16;
        }
        pairingPIN[pinDigitIndex] = hexChars[currentHexCharIndex];
        lastSelect = millis();
    }
}

void handleButton1() {
    // When connected and transmitting, require ALL buttons to enter menu
    if (connected && currentMenuState == MENU_NORMAL_OPERATION) {
        // Check if all buttons are pressed
        bool btn1 = digitalRead(BUTTON1_PIN) == LOW;
        bool btn2 = digitalRead(BUTTON2_PIN) == LOW;
        bool btn3 = digitalRead(BUTTON3_PIN) == LOW;
        bool btn4 = digitalRead(BUTTON4_PIN) == LOW;
        bool all_pressed = btn1 && btn2 && btn3 && btn4;
        
        static unsigned long all_buttons_press_start = 0;
        static bool all_buttons_pressed = false;
        
        if (all_pressed && !all_buttons_pressed) {
            all_buttons_pressed = true;
            all_buttons_press_start = millis();
        } else if (!all_pressed && all_buttons_pressed) {
            all_buttons_pressed = false;
            unsigned long duration = millis() - all_buttons_press_start;
            
            // Require all buttons held for at least 500ms to prevent accidental activation
            if (duration >= 500) {
                Serial.println("All buttons pressed - entering menu");
                currentMenuState = MENU_MAIN;
            }
        }
        // Don't process individual buttons when connected and transmitting
        return;
    }
    
    // Back/Cancel / Long press (3 seconds) for pairing mode
    static unsigned long button1PressStart = 0;
    static bool button1Pressed = false;
    static unsigned long lastPress = 0;
    
    bool buttonState = (digitalRead(BUTTON1_PIN) == LOW);
    
    if (buttonState && !button1Pressed) {
        // Button just pressed
        button1Pressed = true;
        button1PressStart = millis();
    } else if (!buttonState && button1Pressed) {
        // Button released
        button1Pressed = false;
        unsigned long pressDuration = millis() - button1PressStart;
        
        if (pressDuration >= 3000) {
            // Long press (3 seconds) - enter pairing mode
            Serial.println("Long press detected - entering pairing mode");
            autoConnectEnabled = false;
            autoConnectInProgress = false;
            if (connected) {
                // Disconnect if connected
                if (spp_client_channel_id != 0) {
                    rfcomm_disconnect(spp_client_channel_id);
                    spp_client_channel_id = 0;
                }
                connected = false;
                connectionStatus = "Disconnected";
            }
            currentMenuState = MENU_MAIN;
            mainMenuIndex = 0;
            return;
        } else if (pressDuration >= 300) {
            // Short press - normal back/cancel behavior
            if (millis() - lastPress < 300) return;  // Debounce
            lastPress = millis();
            
            switch (currentMenuState) {
                case MENU_PAIRING_ENTER_PIN:
                    if (pinDigitIndex > 0) {
                        pinDigitIndex--;
                    } else {
                        currentMenuState = MENU_PAIRING_SELECT_DEVICE;
                    }
                    break;
                case MENU_PAIRING_SELECT_DEVICE:
                    currentMenuState = MENU_PAIRING_SCAN;
                    break;
                case MENU_PAIRING_SCAN:
                    scanning = false;
                    currentMenuState = MENU_MAIN;
                    break;
                case MENU_MAIN:
                    currentMenuState = MENU_NORMAL_OPERATION;
                    break;
                case MENU_PAIRING_CONNECTING:
                    // Cancel auto-connect
                    if (autoConnectInProgress) {
                        autoConnectEnabled = false;
                        autoConnectInProgress = false;
                        currentMenuState = MENU_MAIN;
                    }
                    break;
            }
        }
    }
}

void handleButton2() {
    // Confirm/Enter
    static unsigned long lastPress = 0;
    if (millis() - lastPress < 300) return;  // Debounce
    lastPress = millis();
    
    switch (currentMenuState) {
        case MENU_PAIRING_ENTER_PIN:
            if (pinDigitIndex < 5) {
                pinDigitIndex++;
            } else {
                // All digits entered, try to connect
                currentMenuState = MENU_PAIRING_CONNECTING;
                connectToDevice(foundDevices[selectedDeviceIndex].address, pairingPIN);
            }
            break;
        case MENU_PAIRING_SELECT_DEVICE:
            currentMenuState = MENU_PAIRING_ENTER_PIN;
            pinDigitIndex = 0;
            // Initialize PIN entry with default PIN
            strcpy(pairingPIN, defaultRXPin);
            // Find current hex char index for first digit
            for (int i = 0; i < 16; i++) {
                if (hexChars[i] == pairingPIN[0]) {
                    currentHexCharIndex = i;
                    break;
                }
            }
            break;
        case MENU_MAIN:
            if (mainMenuIndex == 0) {
                // Quick Connect (use default MAC and PIN)
                Serial.println("Quick Connect to default RX device");
                Serial.print("MAC: ");
                for (int i = 0; i < 6; i++) {
                    if (i > 0) Serial.print(":");
                    Serial.print(defaultRXMac[i], HEX);
                }
                Serial.print(" PIN: ");
                Serial.println(defaultRXPin);
                
                currentMenuState = MENU_PAIRING_CONNECTING;
                connectToDevice(defaultRXMac, defaultRXPin);
            } else if (mainMenuIndex == 1) {
                // Pairing Mode (scan for devices)
                currentMenuState = MENU_PAIRING_SCAN;
                scanForDevices();
            }
            // mainMenuIndex == 2 is Exit, handled by handleButton3
            break;
    }
}

void handleButton3() {
    // When connected and transmitting, require ALL buttons to enter menu
    // This function is only called when not in normal operation or when not connected
    // So we can safely handle menu toggle here
    
    // Menu toggle / Exit
    static unsigned long lastPress = 0;
    if (millis() - lastPress < 300) return;  // Debounce
    lastPress = millis();
    
    // Only allow menu toggle if not connected, or if already in menu
    if (!connected && currentMenuState == MENU_NORMAL_OPERATION) {
        currentMenuState = MENU_MAIN;
        mainMenuIndex = 0;  // Reset to first menu item
    } else if (currentMenuState == MENU_MAIN) {
        // Exit from main menu
        if (mainMenuIndex == 2) {
            currentMenuState = MENU_NORMAL_OPERATION;
        }
    }
}

// Connection state tracking
static unsigned long connectionStartTime = 0;
static bool connectionInProgress = false;
static const unsigned long CONNECT_TIMEOUT = 15000;  // 15 seconds
static bd_addr_t target_address = {0};
static hci_con_handle_t acl_connection_handle = 0;
static bool acl_connected = false;
static bool rfcomm_retry_pending = false;
static unsigned long rfcomm_retry_time = 0;
static int rfcomm_retry_count = 0;
const int MAX_RFCOMM_RETRIES = 3;
const unsigned long RFCOMM_RETRY_DELAY = 2000;  // 2 seconds between retries
static uint16_t l2cap_cid_for_acl = 0;  // Store L2CAP CID - don't close it until RFCOMM is ready
static const char* current_pin = nullptr;  // Store PIN for pairing
static uint8_t rfcomm_channel_to_try = 1;  // Start with channel 1, will try others if needed
static bool security_requested = false;
static bool awaiting_authentication = false;
static unsigned long auth_request_time = 0;
static bool sdp_query_active = false;
static uint8_t discovered_rfcomm_channel = 1;
static bool sdp_query_pending = false;
static unsigned long sdp_query_time = 0;
static bool startSdpQueryForSerialService();

// SPP client callback
static void spp_client_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    // Handle SPP client events
    if (packet_type != HCI_EVENT_PACKET) return;
    
    uint8_t event = packet[0];
    
    switch (event) {
        case BTSTACK_EVENT_STATE:
            if (packet[2] == HCI_STATE_WORKING) {
                Serial.println("BT HCI ready for connection");
            }
            break;
            
        case HCI_EVENT_PIN_CODE_REQUEST:
            // Remote device is requesting PIN for pairing
            {
                Serial.println("*** PIN CODE REQUEST RECEIVED ***");
                bd_addr_t pin_address;
                reverse_bd_addr(&packet[2], pin_address);
                
                Serial.print("PIN requested for device: ");
                for (int i = 0; i < 6; i++) {
                    if (i > 0) Serial.print(":");
                    if (pin_address[i] < 0x10) Serial.print("0");
                    Serial.print(pin_address[i], HEX);
                }
                Serial.println();
                
                if (current_pin != nullptr && strlen(current_pin) >= 4 && strlen(current_pin) <= 16) {
                    // Send ASCII PIN code (legacy pairing expects ASCII digits)
                    char pin_str[17] = {0};
                    size_t pin_len = min((size_t)16, strlen(current_pin));
                    for (size_t i = 0; i < pin_len; i++) {
                        pin_str[i] = toupper(current_pin[i]);
                    }
                    
                    gap_pin_code_response(pin_address, pin_str);
                    
                    Serial.print("PIN response sent: ");
                    Serial.println(pin_str);
                } else {
                    Serial.println("ERROR: No PIN available for pairing!");
                    connectionStatus = "No PIN";
                }
            }
            break;
            
        case HCI_EVENT_CONNECTION_COMPLETE:
            // ACL connection established, wait a bit then create SPP connection
            {
                hci_con_handle_t con_handle = little_endian_read_16(packet, 3);
                uint8_t status = packet[2];
                
                if (status != 0) {
                    Serial.print("ACL connection failed: ");
                    Serial.println(status);
                    connectionStatus = "ACL failed";
                    connectionInProgress = false;
                    acl_connected = false;
                    break;
                }
                
                Serial.print("ACL connected, handle: ");
                Serial.println(con_handle);
                acl_connection_handle = con_handle;
                acl_connected = true;
                rfcomm_retry_count = 0;
                rfcomm_channel_to_try = 1;  // Reset to channel 1 for new connection
                security_requested = false;
                awaiting_authentication = true;
                
                // Close L2CAP channel now that ACL is established
                // ACL connection is independent and will remain open
                // Keeping L2CAP open might interfere with RFCOMM creation
                if (l2cap_cid_for_acl != 0) {
                    Serial.println("Closing L2CAP channel (ACL is independent)...");
                    l2cap_disconnect(l2cap_cid_for_acl);
                    l2cap_cid_for_acl = 0;
                    delay(500);  // Brief delay after closing L2CAP
                }
                
                // Request security (pairing/bonding) via GAP
                int auth_status = hci_send_cmd(&hci_authentication_requested, acl_connection_handle);
                if (auth_status != ERROR_CODE_SUCCESS) {
                    Serial.print("hci_authentication_requested failed: ");
                    Serial.println(auth_status);
                    connectionStatus = "Auth req failed";
                } else {
                    Serial.println("Authentication requested, waiting for completion...");
                    connectionStatus = "Pairing...";
                    security_requested = true;
                    auth_request_time = millis();
                }
                
                // RFCOMM creation will be triggered after authentication completes
            }
            break;
            
        case HCI_EVENT_AUTHENTICATION_COMPLETE:
            // Pairing/authentication completed
            {
                uint8_t status = packet[2];
                hci_con_handle_t con_handle = little_endian_read_16(packet, 3);
                awaiting_authentication = false;
                
                Serial.print("*** AUTHENTICATION COMPLETE ***");
                Serial.print(", status: ");
                Serial.print(status, HEX);
                Serial.print(", handle: ");
                Serial.println(con_handle);
                
                if (status == ERROR_CODE_SUCCESS && acl_connected) {
                    Serial.println("Authentication SUCCESSFUL - scheduling SDP query for SPP service");
                    sdp_query_pending = true;
                    sdp_query_time = millis() + 500;  // wait 500ms for remote service to be ready
                    connectionStatus = "Auth OK";
                } else {
                    Serial.print("ERROR: Authentication FAILED with status: 0x");
                    Serial.println(status, HEX);
                    Serial.println("Common error codes: 0x05=Auth failure, 0x06=PIN/key missing");
                    connectionStatus = "Auth failed";
                    connectionInProgress = false;
                    acl_connected = false;
                    rfcomm_retry_pending = false;
                }
            }
            break;
            
        case SDP_EVENT_QUERY_RFCOMM_SERVICE:
            {
                discovered_rfcomm_channel = sdp_event_query_rfcomm_service_get_rfcomm_channel(packet);
                const char* service_name = sdp_event_query_rfcomm_service_get_name(packet);
                Serial.print("SDP found service '");
                Serial.print(service_name);
                Serial.print("' on RFCOMM channel ");
                Serial.println(discovered_rfcomm_channel);
            }
            break;
            
        case SDP_EVENT_QUERY_COMPLETE:
            {
                sdp_query_active = false;
                uint8_t status = sdp_event_query_complete_get_status(packet);
                Serial.print("SDP query complete, status: 0x");
                Serial.println(status, HEX);
                
                if (status != ERROR_CODE_SUCCESS) {
                    Serial.println("SDP query failed, falling back to channel 1");
                    discovered_rfcomm_channel = 1;
                } else if (discovered_rfcomm_channel == 0) {
                    Serial.println("SDP query completed but no RFCOMM channel found, falling back to channel 1");
                    discovered_rfcomm_channel = 1;
                }
                
                rfcomm_channel_to_try = discovered_rfcomm_channel;
                rfcomm_retry_count = 0;
                rfcomm_retry_pending = true;
                rfcomm_retry_time = millis() + 200;  // short delay before creating RFCOMM
                connectionStatus = "Connecting (RFCOMM)...";
            }
            break;
            
        case L2CAP_EVENT_CHANNEL_OPENED:
            // L2CAP channel opened (we used this just to establish ACL)
            {
                uint16_t local_cid = little_endian_read_16(packet, 4);
                uint8_t status = packet[2];
                Serial.print("L2CAP channel opened, CID: ");
                Serial.print(local_cid);
                Serial.print(", status: ");
                Serial.println(status);
                
                if (status == 0) {
                    // L2CAP channel opened successfully, ACL will be created
                    // Store CID so we can close it after ACL is confirmed
                    l2cap_cid_for_acl = local_cid;
                    Serial.println("L2CAP channel opened, waiting for ACL...");
                } else {
                    Serial.print("L2CAP channel open failed: ");
                    Serial.println(status);
                    connectionStatus = "L2CAP failed";
                    connectionInProgress = false;
                }
            }
            break;
            
        case L2CAP_EVENT_CHANNEL_CLOSED:
            {
                uint16_t local_cid = little_endian_read_16(packet, 2);
                Serial.print("L2CAP channel closed, CID: ");
                Serial.println(local_cid);
                
                // If this was our ACL-maintaining channel and RFCOMM isn't connected yet, that's a problem
                if (local_cid == l2cap_cid_for_acl && !connected) {
                    Serial.println("L2CAP channel closed before RFCOMM established!");
                    connectionStatus = "L2CAP closed";
                    connectionInProgress = false;
                    acl_connected = false;
                }
            }
            break;
            
        case RFCOMM_EVENT_INCOMING_CONNECTION:
            // This shouldn't happen in client mode, but handle it
            break;
            
        case RFCOMM_EVENT_CHANNEL_OPENED:
            {
                uint8_t status = packet[2];
                uint16_t rfcomm_cid = little_endian_read_16(packet, 4);
                
                if (status == 0) {  // Success
                    spp_client_channel_id = rfcomm_cid;
                    Serial.print("SPP client connected! RFCOMM CID: ");
                    Serial.println(rfcomm_cid);
                    
                    // L2CAP channel should already be closed, but make sure
                    if (l2cap_cid_for_acl != 0) {
                        Serial.println("Closing any remaining L2CAP channel");
                        l2cap_disconnect(l2cap_cid_for_acl);
                        l2cap_cid_for_acl = 0;
                    }
                    
                    connected = true;
                    connectionStatus = "Connected";
                    connectionInProgress = false;
                    rfcomm_retry_pending = false;
                    rfcomm_retry_count = 0;
                } else {
                    Serial.print("RFCOMM channel open failed, status: ");
                    Serial.println(status);
                    Serial.println("Error codes: 1=Unknown, 2=Timeout, 3=Closed");
                    
                    // Retry if we haven't exceeded max retries
                    if (rfcomm_retry_count < MAX_RFCOMM_RETRIES) {
                        rfcomm_retry_count++;
                        rfcomm_retry_pending = true;
                        rfcomm_retry_time = millis() + RFCOMM_RETRY_DELAY;
                        Serial.print("Will retry RFCOMM (attempt ");
                        Serial.print(rfcomm_retry_count + 1);
                        Serial.print(" of ");
                        Serial.print(MAX_RFCOMM_RETRIES);
                        Serial.println(")...");
                        connectionStatus = "RFCOMM retry...";
                    } else {
                        Serial.println("Max RFCOMM retries reached");
                        connectionStatus = "RFCOMM failed";
                        connectionInProgress = false;
                        // Keep L2CAP open for now in case user wants to retry manually
                    }
                }
            }
            break;
            
        case RFCOMM_EVENT_CHANNEL_CLOSED:
            Serial.println("SPP client disconnected");
            spp_client_channel_id = 0;
            connected = false;
            connectionStatus = "Disconnected";
            break;
            
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            {
                uint8_t reason = packet[2];
                Serial.print("BT disconnected, reason: ");
                Serial.print(reason);
                // Common disconnect reasons:
                // 0x08 = Connection timeout
                // 0x13 = Remote user terminated connection
                // 0x16 = Connection terminated by local host
                // 0x05 = Authentication failure
                // 0x15 = Connection terminated due to security reasons
                Serial.print(" (");
                switch(reason) {
                    case 0x05: Serial.print("Auth failure"); break;
                    case 0x08: Serial.print("Timeout"); break;
                    case 0x13: Serial.print("Remote terminated"); break;
                    case 0x15: Serial.print("Security failure"); break;
                    case 0x16: Serial.print("Local terminated"); break;
                    case 0x00: Serial.print("No error/Unknown"); break;
                    default: Serial.print("Other"); break;
                }
                Serial.print(")");
                
                // If we were trying to create RFCOMM, log that
                if (rfcomm_retry_pending || rfcomm_retry_count > 0) {
                    Serial.print(" - RFCOMM was being created on channel ");
                    Serial.println(rfcomm_channel_to_try);
                } else {
                    Serial.println();
                }
                
                // Close L2CAP if still open
                if (l2cap_cid_for_acl != 0) {
                    l2cap_disconnect(l2cap_cid_for_acl);
                    l2cap_cid_for_acl = 0;
                }
                
                spp_client_channel_id = 0;
                connected = false;
                acl_connected = false;
                connectionStatus = "Disconnected";
                rfcomm_retry_pending = false;
                rfcomm_retry_count = 0;
                rfcomm_channel_to_try = 1;
                current_pin = nullptr;
                security_requested = false;
                awaiting_authentication = false;
                auth_request_time = 0;
                sdp_query_active = false;
                discovered_rfcomm_channel = 1;
                sdp_query_pending = false;
                sdp_query_time = 0;
            }
            break;
    }
}

void connectToDevice(const uint8_t* address, const char* pin) {
    Serial.print("Connecting to device: ");
    Serial.print(foundDevices[selectedDeviceIndex].name);
    Serial.print(" with PIN: ");
    Serial.println(pin);
    
    connectionStatus = "Connecting...";
    connectionInProgress = true;
    connectionStartTime = millis();
    currentMenuState = MENU_PAIRING_CONNECTING;
    
    // Stop any existing SerialBT connection
    if (connected || SerialBT) {
        SerialBT.end();
        delay(100);
    }
    
    // Copy target address
    memcpy(target_address, address, 6);
    
    // Force fresh pairing by dropping any stored link key for this device
    gap_drop_link_key_for_bd_addr(target_address);
    
    // Store PIN for pairing
    current_pin = pin;
    
    // Reset connection state
    acl_connected = false;
    rfcomm_retry_pending = false;
    rfcomm_retry_count = 0;
    spp_client_channel_id = 0;
    security_requested = false;
    awaiting_authentication = false;
    auth_request_time = 0;
    sdp_query_active = false;
    discovered_rfcomm_channel = 1;
    sdp_query_pending = false;
    sdp_query_time = 0;
    
    // Close any existing L2CAP channel
    if (l2cap_cid_for_acl != 0) {
        l2cap_disconnect(l2cap_cid_for_acl);
        l2cap_cid_for_acl = 0;
    }
    
    // Ensure BluetoothHCI is running
    if (!btHCI.running()) {
        Serial.println("Starting BluetoothHCI...");
        btHCI.install();
        btHCI.begin();
        delay(1000);
    }
    
    // Register HCI event handler for connection events
    static btstack_packet_callback_registration_t hci_callback_registration;
    hci_callback_registration.callback = &spp_client_packet_handler;
    hci_add_event_handler(&hci_callback_registration);
    
    // Connect to SPP service on the target device
    Serial.print("Initiating ACL connection to: ");
    for (int i = 0; i < 6; i++) {
        if (i > 0) Serial.print(":");
        if (target_address[i] < 0x10) Serial.print("0");
        Serial.print(target_address[i], HEX);
    }
    Serial.println();
    
    // For Classic Bluetooth, create ACL connection first using L2CAP
    // L2CAP will automatically create ACL if needed
    // We'll create RFCOMM channel after ACL is established in the callback
    Serial.println("Creating ACL connection via L2CAP...");
    
    // Create L2CAP connection to establish ACL
    // Use SDP PSM (1) - we just need ACL, then we'll create RFCOMM
    // The L2CAP channel will be closed after we get ACL, then we create RFCOMM
    uint16_t l2cap_psm = 1;  // SDP PSM - just to establish ACL
    uint16_t l2cap_mtu = 23;
    uint16_t l2cap_cid = 0;
    
    uint8_t l2cap_status = l2cap_create_channel(spp_client_packet_handler, target_address, l2cap_psm, l2cap_mtu, &l2cap_cid);
    
    if (l2cap_status != 0) {
        Serial.print("L2CAP create failed: ");
        Serial.println(l2cap_status);
        Serial.println("Will retry...");
        connectionStatus = "L2CAP init failed";
    } else {
        Serial.println("L2CAP connection initiated, ACL will be created...");
        Serial.print("L2CAP CID: ");
        Serial.println(l2cap_cid);
    }
    
    // RFCOMM channel will be created in HCI_EVENT_CONNECTION_COMPLETE callback
    // after ACL is established
}

void tryCreateRFCOMMChannel() {
    if (!acl_connected) {
        Serial.println("Cannot create RFCOMM: ACL not connected");
        return;
    }
    
    // Verify ACL connection handle is valid
    if (acl_connection_handle == 0) {
        Serial.println("ERROR: ACL connection handle is invalid!");
        return;
    }
    
    // Try different RFCOMM channels if previous attempts failed
    // SerialBT typically uses channel 1, but some implementations use others
    uint8_t rfcomm_channel_nr = rfcomm_channel_to_try;
    uint16_t out_cid = 0;
    
    Serial.print("Attempting RFCOMM channel creation on channel ");
    Serial.print(rfcomm_channel_nr);
    Serial.print(" (attempt ");
    Serial.print(rfcomm_retry_count + 1);
    Serial.print(" of ");
    Serial.print(MAX_RFCOMM_RETRIES);
    Serial.print("), ACL handle: ");
    Serial.println(acl_connection_handle);
    
    // Ensure we're using the correct connection handle
    // rfcomm_create_channel should use the ACL connection handle implicitly via target_address
    uint8_t rfcomm_status = rfcomm_create_channel(spp_client_packet_handler, target_address, rfcomm_channel_nr, &out_cid);
    
    Serial.print("rfcomm_create_channel returned: ");
    Serial.println(rfcomm_status);
    
    if (rfcomm_status != 0) {
        Serial.print("RFCOMM create channel failed (error ");
        Serial.print(rfcomm_status);
        Serial.println(")");
        
        rfcomm_retry_count++;
        
        // Try next channel number if this one failed
        if (rfcomm_channel_nr < 5 && rfcomm_retry_count < MAX_RFCOMM_RETRIES) {
            rfcomm_channel_to_try++;
            Serial.print("Trying next channel: ");
            Serial.println(rfcomm_channel_to_try);
            rfcomm_retry_pending = true;
            rfcomm_retry_time = millis() + 1000;  // Shorter delay when trying different channels
            connectionStatus = "Trying channel...";
        } else if (rfcomm_retry_count < MAX_RFCOMM_RETRIES) {
            // Reset to channel 1 and retry
            rfcomm_channel_to_try = 1;
            rfcomm_retry_pending = true;
            rfcomm_retry_time = millis() + RFCOMM_RETRY_DELAY;
            Serial.print("Will retry channel 1 in ");
            Serial.print(RFCOMM_RETRY_DELAY / 1000);
            Serial.println(" seconds...");
            connectionStatus = "Retrying RFCOMM...";
        } else {
            Serial.println("Max retries reached, all channels failed");
            connectionStatus = "RFCOMM failed";
            connectionInProgress = false;
        }
    } else {
        Serial.print("RFCOMM channel creation initiated on channel ");
        Serial.print(rfcomm_channel_nr);
        Serial.print(", CID: ");
        Serial.println(out_cid);
        Serial.println("Waiting for RFCOMM_EVENT_CHANNEL_OPENED...");
        rfcomm_retry_pending = false;
        connectionStatus = "RFCOMM connecting...";
        // Don't reset retry count yet - wait for RFCOMM_EVENT_CHANNEL_OPENED
    }
}

static bool startSdpQueryForSerialService() {
    discovered_rfcomm_channel = 0;
    sdp_query_pending = false;
    sdp_query_time = 0;
    
    if (sdp_query_active) {
        Serial.println("SDP query already active, waiting for results...");
        return true;
    }
    
    uint8_t status = sdp_client_query_rfcomm_channel_and_name_for_service_class_uuid(
        &spp_client_packet_handler,
        target_address,
        BLUETOOTH_SERVICE_CLASS_SERIAL_PORT
    );
    
    if (status != ERROR_CODE_SUCCESS) {
        Serial.print("SDP query start failed: ");
        Serial.println(status);
        connectionStatus = "SDP start failed";
        // Fall back to default channel immediately
        discovered_rfcomm_channel = 1;
        rfcomm_channel_to_try = 1;
        rfcomm_retry_pending = true;
        rfcomm_retry_time = millis() + 500;
        return false;
    }
    
    Serial.println("SDP query started for Serial Port service...");
    connectionStatus = "SDP query...";
    sdp_query_active = true;
    return true;
}

void checkConnectionStatus() {
    if (!connectionInProgress) {
        return;
    }
    
    // Handle deferred SDP query
    if (sdp_query_pending && millis() >= sdp_query_time) {
        sdp_query_pending = false;
        if (!startSdpQueryForSerialService()) {
            Serial.println("SDP query start failed, attempting RFCOMM fallback...");
            rfcomm_retry_pending = true;
            rfcomm_retry_time = millis() + 500;
        }
    }
    
    // Handle RFCOMM retry if pending
    if (rfcomm_retry_pending && millis() >= rfcomm_retry_time) {
        rfcomm_retry_pending = false;
        tryCreateRFCOMMChannel();
    }
    
    // If we're still waiting on authentication for too long, fall back to creating RFCOMM
    if (awaiting_authentication && security_requested) {
        if (millis() - auth_request_time > 5000) {
            Serial.println("Authentication taking too long, attempting RFCOMM anyway...");
            awaiting_authentication = false;
            rfcomm_retry_pending = true;
            rfcomm_retry_time = millis();
            connectionStatus = "Auth timeout";
        }
    }
    
    // Check for timeout (use auto-connect timeout if auto-connecting)
    unsigned long timeout = autoConnectInProgress ? AUTO_CONNECT_TIMEOUT : CONNECT_TIMEOUT;
    unsigned long startTime = autoConnectInProgress ? autoConnectStartTime : connectionStartTime;
    
    if (millis() - startTime > timeout) {
        Serial.println("Connection timeout");
        connectionStatus = "Timeout";
        connected = false;
        connectionInProgress = false;
        acl_connected = false;
        rfcomm_retry_pending = false;
        sdp_query_active = false;
        awaiting_authentication = false;
        security_requested = false;
        sdp_query_pending = false;
        sdp_query_time = 0;
        
        if (spp_client_channel_id != 0) {
            rfcomm_disconnect(spp_client_channel_id);
            spp_client_channel_id = 0;
        }
        
        if (autoConnectInProgress) {
            // Auto-connect failed, go to main menu
            autoConnectInProgress = false;
            autoConnectEnabled = false;
            currentMenuState = MENU_MAIN;
            Serial.println("Auto-connect failed, entering pairing mode");
        } else {
            // Manual connection failed
            currentMenuState = MENU_PAIRING_ENTER_PIN;
        }
        return;
    }
    
    // Connection status is updated in spp_client_packet_handler callback
    // If connected flag is set, transition to normal operation
    if (connected && connectionInProgress) {
        connectionInProgress = false;
        autoConnectInProgress = false;
        autoConnectEnabled = false;
        acl_connected = false;
        rfcomm_retry_pending = false;
        currentMenuState = MENU_NORMAL_OPERATION;
        
        // Get RSSI if available (from found devices or default)
        if (deviceCount > 0 && selectedDeviceIndex < deviceCount) {
            rssi = foundDevices[selectedDeviceIndex].rssi;
        }
        
        Serial.println("Connection established!");
    }
}

void sendChannelData() {
    if (!connected || spp_client_channel_id == 0) {
        return;
    }
    
    if (millis() - lastDataSent < DATA_INTERVAL) {
        return;
    }
    
    lastDataSent = millis();
    
    // Increment sequence number
    sequenceNumber++;
    if (sequenceNumber == 0) sequenceNumber = 1;  // Skip 0
    
    // Encode and send channel data with security
    uint8_t frame[FRAME_SIZE];
    Protocol::encodeFrame(channels, sequenceNumber, &security, frame);
    
    // Send via RFCOMM SPP client
    int result = rfcomm_send(spp_client_channel_id, frame, FRAME_SIZE);
    
    if (result != 0) {
        Serial.print("Failed to send frame. Error: ");
        Serial.println(result);
        
        // Track failures
        static int sendFailures = 0;
        sendFailures++;
        
        if (sendFailures > 10) {
            // Too many failures, disconnect
            connected = false;
            connectionStatus = "Send failed";
            spp_client_channel_id = 0;
            Serial.println("Too many send failures, disconnecting");
        }
    } else {
        // Reset failure counter on successful send
        static int sendFailures = 0;
        sendFailures = 0;
        
        // Debug: print occasionally
        static unsigned long lastDebugPrint = 0;
        if (millis() - lastDebugPrint > 1000) {
            Serial.print("Sent frame, seq: ");
            Serial.println(sequenceNumber);
            lastDebugPrint = millis();
        }
    }
}

void updateDisplay() {
    static unsigned long lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate < DISPLAY_UPDATE_INTERVAL) {
        return;
    }
    lastDisplayUpdate = millis();
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    
    switch (currentMenuState) {
        case MENU_NORMAL_OPERATION:
            display.print("Status: ");
            display.println(connectionStatus);
            if (connected) {
                display.print("RSSI: ");
                display.print(rssi);
                display.println(" dBm");
            }
            display.println("Ch1: ");
            display.setCursor(40, 32);
            display.print(channels[0]);
            display.setCursor(80, 32);
            display.print(channels[1]);
            display.setCursor(0, 40);
            display.print("Ch2: ");
            display.setCursor(40, 40);
            display.print(channels[2]);
            display.setCursor(80, 40);
            display.print(channels[3]);
            // Connection indicator
            display.fillRect(120, 0, 8, 8, connected ? SSD1306_WHITE : SSD1306_BLACK);
            display.drawRect(120, 0, 8, 8, SSD1306_WHITE);
            break;
            
        case MENU_MAIN:
            display.println("=== MAIN MENU ===");
            display.print(mainMenuIndex == 0 ? ">" : " ");
            display.println("Quick Connect");
            display.print(mainMenuIndex == 1 ? ">" : " ");
            display.println("Pairing Mode");
            display.print(mainMenuIndex == 2 ? ">" : " ");
            display.println("Exit");
            display.setCursor(0, 56);
            display.println("Btn3:Exit Btn2:Select");
            break;
            
        case MENU_PAIRING_SCAN:
            display.println("Scanning...");
            display.print("Found: ");
            display.print(deviceCount);
            display.println(" devices");
            if (scanning) {
                display.print("Time: ");
                display.print((millis() - scanStartTime) / 1000);
                display.println("s");
            }
            display.setCursor(0, 56);
            display.println("Btn1:Cancel");
            break;
            
        case MENU_PAIRING_SELECT_DEVICE:
            display.println("Select Device:");
            for (int i = 0; i < min(deviceCount, 4); i++) {
                if (i == selectedDeviceIndex) {
                    display.print("> ");
                } else {
                    display.print("  ");
                }
                String name = foundDevices[i].name;
                if (name.length() > 18) {
                    name = name.substring(0, 15) + "...";
                }
                display.println(name);
            }
            display.setCursor(0, 56);
            display.println("Stick1:Nav Btn2:Select");
            break;
            
        case MENU_PAIRING_ENTER_PIN:
            display.println("Enter PIN:");
            display.print("PIN: ");
            for (int i = 0; i < 6; i++) {
                if (i == pinDigitIndex) {
                    display.print("[");
                    display.print(pairingPIN[i]);
                    display.print("]");
                } else {
                    display.print(" ");
                    display.print(pairingPIN[i]);
                    display.print(" ");
                }
            }
            display.setCursor(0, 32);
            display.print("Char: ");
            display.print(hexChars[currentHexCharIndex]);
            display.print(" (");
            display.print(currentHexCharIndex, HEX);
            display.println(")");
            display.setCursor(0, 48);
            display.println("S1:Pos S2:Char B2:Next");
            break;
            
        case MENU_PAIRING_CONNECTING:
            if (autoConnectInProgress) {
                display.println("Auto-connecting...");
                display.print("MAC: ");
                // Show first 3 bytes of MAC
                for (int i = 0; i < 3; i++) {
                    if (i > 0) display.print(":");
                    if (defaultRXMac[i] < 0x10) display.print("0");
                    display.print(defaultRXMac[i], HEX);
                }
                display.println("...");
                display.print("PIN: ");
                display.println(defaultRXPin);
                display.setCursor(0, 56);
                display.print("Time: ");
                display.print((millis() - autoConnectStartTime) / 1000);
                display.println("s");
            } else {
                display.println("Connecting...");
                display.print("Device: ");
                if (deviceCount > 0 && selectedDeviceIndex < deviceCount) {
                    String name = foundDevices[selectedDeviceIndex].name;
                    if (name.length() > 18) {
                        name = name.substring(0, 15) + "...";
                    }
                    display.println(name);
                } else {
                    display.println("Unknown");
                }
                display.print("PIN: ");
                display.println(pairingPIN);
            }
            break;
    }
    
    display.display();
}

void loop() {
    readInputs();
    
    // Handle button 1 (with long press detection)
    handleButton1();
    
    // Handle menu navigation
    if (currentMenuState != MENU_NORMAL_OPERATION) {
        // Handle other button presses
        if (digitalRead(BUTTON2_PIN) == LOW) {
            handleButton2();
        }
        if (digitalRead(BUTTON3_PIN) == LOW) {
            handleButton3();
        }
        
        // Handle menu-specific navigation
        switch (currentMenuState) {
            case MENU_MAIN:
                handleMainMenu(stick1Nav.x, stick1Nav.y);
                break;
            case MENU_PAIRING_SCAN:
                if (scanning) {
                    scanForDevices();  // Continue scanning
                }
                break;
            case MENU_PAIRING_SELECT_DEVICE:
                handleDeviceSelection(stick1Nav.y);
                break;
            case MENU_PAIRING_ENTER_PIN:
                handlePINEntry(stick1Nav.x, stick1Nav.y, stick2Nav.x, stick2Nav.y);
                break;
            case MENU_PAIRING_CONNECTING:
                // Check connection status while connecting
                checkConnectionStatus();
                break;
        }
        
        updateDisplay();
        delay(50);
        return;
    }
    
    // Normal operation
    if (connected) {
        // Check if SPP channel is still valid
        if (spp_client_channel_id == 0) {
            // Connection lost
            connected = false;
            connectionStatus = "Disconnected";
            Serial.println("Connection lost");
        } else {
            // Send channel data
            sendChannelData();
        }
    }
    updateDisplay();
    
    // Small delay to prevent watchdog issues
    delay(1);
}

