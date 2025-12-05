#include <Arduino.h>
#include <WiFi.h>
#include "WiFiConfig.h"
#include "Protocol.h"
#include "PPMGenerator.h"

// PPM output pin (connect to flight controller)
#define PPM_PIN 0  // GP9

// PPM polarity: Set to true if your flight controller expects inverted PPM
// Most Betaflight controllers expect inverted PPM (true)
#define PPM_INVERTED false

// Network
WiFiServer server(8888);
WiFiConfigManager configManager;
PPMGenerator ppmGenerator(PPM_PIN, PPM_INVERTED);

// Channel values received from TX
uint16_t channels[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

// Connection state
bool clientConnected = false;
unsigned long lastDataReceived = 0;
const unsigned long TIMEOUT_MS = 1000;  // 1 second timeout

#ifdef TEST_MODE
// Test mode: random channel values
unsigned long lastRandomUpdate = 0;
const unsigned long RANDOM_UPDATE_INTERVAL = 2000;  // 2 seconds
#endif

void setup() {
    // Initialize USB Serial FIRST (for debugging/logs)
    Serial.begin(115200);
    
    // Wait for USB Serial to be ready (with timeout to avoid blocking forever)
    unsigned long start = millis();
    while (!Serial && (millis() - start < 2000)) {
        delay(10);
    }
    delay(500);
    Serial.flush();
    
#ifdef TEST_MODE
    // Initialize random seed
    randomSeed(analogRead(A0) + millis());
#endif
    
    Serial.println("FPV Receiver (PPM) starting...");
    Serial.flush();
    
    // Initialize WiFi config
    configManager.begin();
    WiFiConfig config;
    configManager.setDefaultConfig("FPV_RX_AP", "fpv12345678");
    
    // Load config
    if (!configManager.loadConfig(config)) {
        Serial.println("Failed to load WiFi config, using defaults");
        strcpy(config.ssid, "FPV_RX_AP");
        strcpy(config.password, "fpv12345678");
    }
    
    // Start WiFi AP
    WiFi.mode(WIFI_AP);
    bool apStarted = WiFi.softAP(config.ssid, config.password);
    
    if (!apStarted) {
        Serial.println("Failed to start AP");
        while (1) delay(1000);
    }
    
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    Serial.print("SSID: ");
    Serial.println(config.ssid);
    Serial.print("Password: ");
    Serial.println(config.password);
    
    // Start server
    server.begin();
    Serial.println("Server started on port 8888");
    Serial.flush();
    
    // Initialize PPM generator
    Serial.println("Initializing PPM generator...");
    Serial.flush();
    
    ppmGenerator.begin();
    Serial.print("PPM generator initialized on pin GP");
    Serial.print(PPM_PIN);
    Serial.print(" (");
    Serial.print(PPM_INVERTED ? "INVERTED" : "NORMAL");
    Serial.println(" polarity)");
    Serial.flush();
    
    // Start PPM generation
    Serial.println("Starting PPM generation...");
    Serial.flush();
    ppmGenerator.start();
    
    Serial.println("PPM generator started!");
    Serial.flush();
    
    Serial.println("Channel mapping: AETR1234 (Aileron, Elevator, Throttle, Rudder, Aux1-4)");
    Serial.println("Our channels 0-7 map to: Ch0=A, Ch1=E, Ch2=T, Ch3=R, Ch4-7=Aux1-4");
    
    Serial.println("RX ready, waiting for TX connection...");
    Serial.flush();
    
#ifdef TEST_MODE
    Serial.println("TEST MODE: Generating random channel values every 2 seconds");
    Serial.println("Channels will vary randomly between 1000-2000 microseconds");
    Serial.println("PPM signal is generated continuously at 50Hz using hardware timers");
    Serial.flush();
#endif
}

void loop() {
    // Debug: Print heartbeat every 5 seconds to confirm loop is running
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 5000) {
        lastHeartbeat = millis();
        Serial.print("Loop running, millis=");
        Serial.println(millis());
        Serial.flush();
    }
    
#ifdef TEST_MODE
    // Test mode: Generate random channel values every 2 seconds
    if (millis() - lastRandomUpdate >= RANDOM_UPDATE_INTERVAL) {
        lastRandomUpdate = millis();
        
        // Generate random values for all 8 channels (1000-2000 range)
        for (int i = 0; i < 8; i++) {
            channels[i] = random(1000, 2001);
        }
        
        // Update PPM generator
        ppmGenerator.setChannels(channels);
        
        // Print channel values with mapping
        Serial.print("Random channels [AETR1234]: ");
        Serial.print("A="); Serial.print(channels[0]);
        Serial.print(" E="); Serial.print(channels[1]);
        Serial.print(" T="); Serial.print(channels[2]);
        Serial.print(" R="); Serial.print(channels[3]);
        Serial.print(" Aux1="); Serial.print(channels[4]);
        Serial.print(" Aux2="); Serial.print(channels[5]);
        Serial.print(" Aux3="); Serial.print(channels[6]);
        Serial.print(" Aux4="); Serial.println(channels[7]);
        Serial.flush();
    }
#endif
    
    // Check for client connection
    WiFiClient client = server.available();
    
    if (client) {
        if (!clientConnected) {
            Serial.println("New client connected");
            Serial.flush();
            clientConnected = true;
        }
        
        // Read data from client
        if (client.available() >= FRAME_SIZE) {
            uint8_t frame[FRAME_SIZE];
            size_t bytesRead = client.readBytes(frame, FRAME_SIZE);
            
            if (bytesRead == FRAME_SIZE) {
                // Decode frame
                if (Protocol::decodeFrame(frame, channels)) {
#ifdef TEST_MODE
                    Serial.println("Real data received - test mode still active (compile-time flag)");
                    Serial.flush();
#endif
                    
                    // Update PPM generator
                    ppmGenerator.setChannels(channels);
                    lastDataReceived = millis();
                    
                    // Debug: print channel values occasionally
                    static unsigned long lastPrint = 0;
                    if (millis() - lastPrint > 1000) {
                        Serial.print("Channels: ");
                        for (int i = 0; i < 8; i++) {
                            Serial.print(channels[i]);
                            Serial.print(" ");
                        }
                        Serial.println();
                        Serial.flush();
                        lastPrint = millis();
                    }
                } else {
                    Serial.println("Invalid frame received");
                    Serial.flush();
                }
            }
        }
        
        // Check for timeout
        if (millis() - lastDataReceived > TIMEOUT_MS) {
            Serial.println("Client timeout, disconnecting");
            Serial.flush();
            client.stop();
            clientConnected = false;
            // Set channels to failsafe (center position)
            for (int i = 0; i < 8; i++) {
                channels[i] = 1500;
            }
            ppmGenerator.setChannels(channels);
        }
    } else {
        if (clientConnected) {
            Serial.println("Client disconnected");
            Serial.flush();
            clientConnected = false;
            // Set channels to failsafe
            for (int i = 0; i < 8; i++) {
                channels[i] = 1500;
            }
            ppmGenerator.setChannels(channels);
        }
    }
    
    // PPM signal is generated by hardware timer interrupts
    // No need to call update() - it runs automatically
    // Just update channels when they change (already done above)
}

