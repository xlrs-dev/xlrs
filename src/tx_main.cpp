#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ADS1X15.h>
#include "WiFiConfig.h"
#include "Protocol.h"
#include "Security.h"

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

// WiFi and network
WiFiClient client;
WiFiUdp udp;  // UDP for discovery
WiFiConfigManager configManager;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_ADS1115 ads;  // ADS1115 ADC on I2C bus
Security security;

// Channel values (8 channels: 4 from sticks + 4 from buttons)
uint16_t channels[8] = {1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000};

// Connection state
bool connected = false;
unsigned long lastConnectAttempt = 0;
unsigned long lastDataSent = 0;
int rssi = 0;
String connectionStatus = "Disconnected";
IPAddress rxIP;  // Discovered RX IP address
bool rxIPDiscovered = false;

// Security state
uint16_t sequenceNumber = 0;

// Timing
const unsigned long CONNECT_INTERVAL = 5000;  // Try to connect every 5 seconds
const unsigned long DATA_INTERVAL = 20;       // Send data every 20ms (50Hz)
const unsigned long DISPLAY_UPDATE_INTERVAL = 200;  // Update display every 200ms
const unsigned long DISCOVERY_TIMEOUT = 5000;  // Listen for 5 seconds

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
    
    // Initialize security (load or generate pairing key)
    Serial.println("Initializing security...");
    if (security.begin()) {
        Serial.println("Security initialized (pairing key loaded/generated)");
    } else {
        Serial.println("Security initialization failed!");
    }
    
    // Initialize WiFi config
    configManager.begin();
    WiFiConfig config;
    configManager.setDefaultConfig("FPV_RX_AP", "fpv12345678");  // Default SSID/password
    
    // Initialize input pins
    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);
    pinMode(BUTTON3_PIN, INPUT_PULLUP);
    pinMode(BUTTON4_PIN, INPUT_PULLUP);
    
    // Display startup message
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("FPV TX Starting...");
    display.display();
    
    Serial.println("FPV Transmitter initialized");
}

void readInputs() {
    // Read analog sticks from ADS1115 (16-bit ADC: 0-32767)
    // Map to 1000-2000 microseconds (standard RC range)
    int16_t stick1_x = ads.readADC_SingleEnded(STICK1_X_CHANNEL);
    int16_t stick1_y = ads.readADC_SingleEnded(STICK1_Y_CHANNEL);
    int16_t stick2_x = ads.readADC_SingleEnded(STICK2_X_CHANNEL);
    int16_t stick2_y = ads.readADC_SingleEnded(STICK2_Y_CHANNEL);
    
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

IPAddress discoverRX() {
    Serial.println("Discovering RX...");
    connectionStatus = "Discovering...";
    
    // Start UDP listener
    udp.begin(DISCOVERY_PORT);
    
    unsigned long startTime = millis();
    while (millis() - startTime < DISCOVERY_TIMEOUT) {
        int packetSize = udp.parsePacket();
        if (packetSize > 0) {
            char packetBuffer[64];
            int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
            if (len > 0) {
                packetBuffer[len] = '\0';
                
                IPAddress discoveredIP;
                if (Protocol::parseDiscoveryPacket(packetBuffer, discoveredIP)) {
                    Serial.print("RX discovered at: ");
                    Serial.println(discoveredIP);
                    udp.stop();
                    return discoveredIP;
                }
            }
        }
        delay(10);
    }
    
    udp.stop();
    Serial.println("RX discovery timeout");
    return IPAddress(0, 0, 0, 0);
}

void connectToRX() {
    if (connected && client.connected()) {
        return;  // Already connected
    }
    
    if (millis() - lastConnectAttempt < CONNECT_INTERVAL) {
        return;  // Too soon to retry
    }
    
    lastConnectAttempt = millis();
    
    WiFiConfig config;
    if (!configManager.loadConfig(config)) {
        connectionStatus = "Config Error";
        return;
    }
    
    Serial.print("Connecting to: ");
    Serial.println(config.ssid);
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(config.ssid, config.password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(100);
        attempts++;
        Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        
        // Discover RX if not already discovered
        if (!rxIPDiscovered) {
            IPAddress discoveredIP = discoverRX();
            if (discoveredIP != IPAddress(0, 0, 0, 0)) {
                rxIP = discoveredIP;
                rxIPDiscovered = true;
            } else {
                connectionStatus = "Discovery Failed";
                return;
            }
        }
        
        // Connect to RX server (default port 8888)
        if (client.connect(rxIP, 8888)) {
            connected = true;
            connectionStatus = "Connected";
            Serial.print("Connected to RX at ");
            Serial.println(rxIP);
            sequenceNumber = 0;  // Reset sequence on new connection
        } else {
            connectionStatus = "Server Failed";
            Serial.println("Failed to connect to RX server");
            rxIPDiscovered = false;  // Retry discovery next time
        }
    } else {
        connectionStatus = "WiFi Failed";
        Serial.println("\nWiFi connection failed");
        rxIPDiscovered = false;
    }
}

void sendChannelData() {
    if (!connected || !client.connected()) {
        connected = false;
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
    
    size_t bytesWritten = client.write(frame, FRAME_SIZE);
    if (bytesWritten != FRAME_SIZE) {
        Serial.println("Failed to send complete frame");
        connected = false;
        client.stop();
        rxIPDiscovered = false;  // Retry discovery on next connection
    }
}

void updateDisplay() {
    static unsigned long lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate < DISPLAY_UPDATE_INTERVAL) {
        return;
    }
    lastDisplayUpdate = millis();
    
    display.clearDisplay();
    display.setCursor(0, 0);
    
    // Connection status
    display.setTextSize(1);
    display.print("Status: ");
    display.println(connectionStatus);
    
    // Signal strength
    if (WiFi.status() == WL_CONNECTED) {
        rssi = WiFi.RSSI();
        display.print("RSSI: ");
        display.print(rssi);
        display.println(" dBm");
    } else {
        display.println("RSSI: --");
    }
    
    // Channel values (show first 4 channels)
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
    
    display.display();
}

void loop() {
    readInputs();
    connectToRX();
    sendChannelData();
    updateDisplay();
    
    // Small delay to prevent watchdog issues
    delay(1);
}

