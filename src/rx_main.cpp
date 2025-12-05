#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <string.h>
#include "WiFiConfig.h"
#include "Protocol.h"
#include "Security.h"
#include "crsfSerial.h"
#include "crsf_protocol.h"

// CRSF Serial pins (connect to flight controller)
#define CRSF_TX_PIN 8   // GP8 - TX to flight controller
#define CRSF_RX_PIN 9   // GP9 - RX from flight controller (optional, not used for TX-only)

// Network
WiFiServer server(8888);  // TCP server for binary protocol
WiFiServer webServer(80);  // HTTP server for web UI
WiFiUDP udp;  // UDP for discovery broadcasts
WiFiConfigManager configManager;
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

// Discovery broadcast
unsigned long lastDiscoveryBroadcast = 0;
const unsigned long DISCOVERY_INTERVAL = 2000;  // Broadcast every 2 seconds

#ifdef TEST_MODE
// Test mode: random channel values
unsigned long lastRandomUpdate = 0;
const unsigned long RANDOM_UPDATE_INTERVAL = 2000;  // 2 seconds
#endif

void setup() {
    // Initialize USB Serial FIRST (for debugging/logs)
    // IMPORTANT: Serial = USB CDC (for logs/debugging)
    //            Serial2 = Hardware UART1 (for CRSF communication)
    //            These are COMPLETELY SEPARATE and do not interfere with each other
    Serial.begin(115200);
    
    // Wait for USB Serial to be ready (with timeout to avoid blocking forever)
    // On RP2040, USB Serial may not be available immediately
    unsigned long start = millis();
    while (!Serial && (millis() - start < 2000)) {
        delay(10);
    }
    delay(500);  // Additional delay to ensure USB CDC is fully ready
    
    // Flush any pending data
    Serial.flush();
    
#ifdef TEST_MODE
    // Initialize random seed (using uninitialized memory or timer as seed)
    randomSeed(analogRead(A0) + millis());
#endif
    
    Serial.println("FPV Receiver starting...");
    Serial.flush();  // Ensure message is sent before continuing
    
    // Initialize security (load or generate pairing key)
    Serial.println("Initializing security...");
    if (security.begin()) {
        Serial.println("Security initialized (pairing key loaded/generated)");
    } else {
        Serial.println("Security initialization failed!");
    }
    Serial.flush();
    
    // Initialize WiFi config
    configManager.begin();
    
    // Default WiFi network credentials (change these to your network)
    const char* defaultSSID = "golain";  // CHANGE THIS to your WiFi network name
    const char* defaultPassword = "12345678";  // CHANGE THIS to your WiFi password
    
    WiFiConfig config;
    
    // FORCE USE OF DEFAULTS - Ignore EEPROM for now due to corruption issues
    // TODO: Add EEPROM clear/reset functionality via web UI
    Serial.println("Using hardcoded WiFi credentials (EEPROM ignored)");
    strncpy(config.ssid, defaultSSID, 31);
    strncpy(config.password, defaultPassword, 31);
    config.ssid[31] = '\0';
    config.password[31] = '\0';
    
    // Optionally save to EEPROM for future use (uncomment if you want to save)
    // configManager.setDefaultConfig(defaultSSID, defaultPassword);
    
    // Print actual values being used (for debugging)
    Serial.print("Connecting to WiFi SSID: '");
    Serial.print(config.ssid);
    Serial.print("' (length: ");
    Serial.print(strlen(config.ssid));
    Serial.println(")");
    
    Serial.print("Password length: ");
    Serial.println(strlen(config.password));
    
    // Connect to existing WiFi network (Station mode)
    WiFi.mode(WIFI_STA);
    WiFi.begin(config.ssid, config.password);
    
    Serial.print("Connecting to WiFi");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(1000);
        Serial.print(".");
        attempts++;
    }
    Serial.println();
    
    if (WiFi.status() == WL_CONNECTED) {
        IPAddress IP = WiFi.localIP();
        Serial.println("WiFi connected!");
        Serial.print("IP address: ");
        Serial.println(IP);
        Serial.print("Gateway: ");
        Serial.println(WiFi.gatewayIP());
        Serial.print("Subnet: ");
        Serial.println(WiFi.subnetMask());
        Serial.print("RSSI: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
        Serial.print("Web UI: http://");
        Serial.println(IP);
    } else {
        Serial.println("WiFi connection failed!");
        Serial.println("Please check:");
        Serial.println("1. SSID and password are correct");
        Serial.println("2. Network is in range");
        Serial.println("3. Update defaultSSID/defaultPassword in code if needed");
        Serial.println("Continuing anyway - web server may not be accessible...");
    }
    
    // Start TCP server (for binary protocol)
    server.begin();
    Serial.println("TCP server started on port 8888");
    
    // Start UDP for discovery broadcasts
    if (WiFi.status() == WL_CONNECTED) {
        udp.begin(DISCOVERY_PORT);
        Serial.print("UDP discovery server started on port ");
        Serial.println(DISCOVERY_PORT);
    }
    
    // Start HTTP web server
    webServer.begin();
    Serial.println("Web server started on port 80");
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Web UI available at: http://");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("Web UI will be available once WiFi connects");
    }
    Serial.flush();
    
    // Initialize CRSF serial output on Serial2 (hardware UART, NOT USB)
    // IMPORTANT: Serial2 is completely separate from Serial (USB CDC)
    Serial.println("Initializing CRSF Serial2...");
    Serial.flush();
    
    // Initialize Serial2 with pins directly in begin() call
    // On Arduino-Pico (earlephilhower), Serial2.begin(baud, config, rxPin, txPin)
    Serial.print("Initializing Serial2: TX=GP");
    Serial.print(CRSF_TX_PIN);
    Serial.print(", RX=GP");
    Serial.print(CRSF_RX_PIN);
    Serial.print(", Baud=");
    Serial.println(CRSF_BAUDRATE);
    Serial.flush();
    
    // Initialize Serial2 with pins in the begin() call directly
    // This avoids the setTX/setRX issue and ensures pins are set correctly
    Serial.println("Calling Serial2.begin() with pins...");
    Serial.flush();
    
    // Arduino-Pico: Serial2.begin(baud, config, rxPin, txPin)
    Serial2.setTX(CRSF_TX_PIN);
    Serial2.setRX(CRSF_RX_PIN);
    
    Serial.println("Serial2 pins set");
    Serial.flush();    
    // Now initialize CRSF
    Serial.println("Calling crsf.begin() - WARNING: may reset Serial2 pins");
    Serial.flush();
    
    // Pass CRSF_BAUDRATE explicitly
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
    
    // Send a test CRSF packet immediately to verify it's working
    Serial.println("Sending test CRSF packet...");
    crsf_channels_t testChannels = {0};
    // Set all channels to center value
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
    Serial.println("Our channels 0-7 map to: Ch0=A, Ch1=E, Ch2=T, Ch3=R, Ch4-7=Aux1-4");
    
    Serial.println("RX ready, waiting for TX connection...");
    Serial.flush();
    
#ifdef TEST_MODE
    Serial.println("TEST MODE: Generating random channel values every 2 seconds");
    Serial.println("Channels will vary randomly between 1000-2000 microseconds");
    Serial.println("CRSF packets will be sent continuously at ~50Hz");
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
    
    // Broadcast discovery packet periodically
    if (WiFi.status() == WL_CONNECTED && millis() - lastDiscoveryBroadcast >= DISCOVERY_INTERVAL) {
        lastDiscoveryBroadcast = millis();
        
        IPAddress rxIP = WiFi.localIP();
        char discoveryPacket[64];
        Protocol::createDiscoveryPacket(rxIP, discoveryPacket, sizeof(discoveryPacket));
        
        // Broadcast to 255.255.255.255
        IPAddress broadcastIP(255, 255, 255, 255);
        udp.beginPacket(broadcastIP, DISCOVERY_PORT);
        udp.write((const uint8_t*)discoveryPacket, strlen(discoveryPacket));
        udp.endPacket();
        
        // Debug: log broadcast occasionally
        static unsigned long lastBroadcastLog = 0;
        if (millis() - lastBroadcastLog > 10000) {
            lastBroadcastLog = millis();
            Serial.print("Discovery broadcast: ");
            Serial.println(discoveryPacket);
        }
    }
    
#ifdef TEST_MODE
    // Test mode: Generate random channel values every 2 seconds
    if (millis() - lastRandomUpdate >= RANDOM_UPDATE_INTERVAL) {
        lastRandomUpdate = millis();
        
        // Generate random values for all 8 channels (1000-2000 range)
        for (int i = 0; i < 8; i++) {
            channels[i] = random(1000, 2001);  // random between 1000 and 2000 inclusive
        }
        
        // Send CRSF channels (will be sent in main loop)
        
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
    }
#endif
    
    // Handle web server requests (HTTP on port 80)
    WiFiClient webClient = webServer.available();
    if (webClient) {
        // Wait for request line
        unsigned long timeout = millis() + 1000;
        while (!webClient.available() && millis() < timeout) {
            delay(1);
        }
        
        if (!webClient.available()) {
            webClient.stop();
        } else {
        
        String request = webClient.readStringUntil('\r');
        webClient.read(); // consume \n
        
        // Read remaining headers (skip until blank line)
        while (webClient.available()) {
            String line = webClient.readStringUntil('\r');
            if (webClient.available()) {
                webClient.read(); // consume \n
            }
            if (line.length() == 0) break; // Blank line indicates end of headers
        }
        
        // Handle CORS preflight requests
        if (request.indexOf("OPTIONS") >= 0) {
            webClient.println("HTTP/1.1 200 OK");
            webClient.println("Access-Control-Allow-Origin: *");
            webClient.println("Access-Control-Allow-Methods: GET, POST, OPTIONS");
            webClient.println("Access-Control-Allow-Headers: Content-Type");
            webClient.println("Connection: close");
            webClient.println();
            webClient.stop();
        } else {
        
        if (request.indexOf("GET / ") >= 0 || request.indexOf("GET /index.html") >= 0) {
            // Serve the web UI HTML page
            String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
            html += "<title>FPV RC Control</title>";
            html += "<style>body{font-family:Arial;margin:20px;background:#1a1a1a;color:#fff;}";
            html += ".container{max-width:900px;margin:0 auto;}";
            html += "h1{color:#4CAF50;text-align:center;}";
            html += ".stick-container{background:#333;padding:20px;margin:15px;border-radius:8px;display:inline-block;width:45%;vertical-align:top;}";
            html += ".stick-container h2{margin-top:0;color:#4CAF50;}";
            html += ".joystick-wrapper{position:relative;width:200px;height:200px;margin:20px auto;background:#222;border-radius:50%;border:3px solid #555;}";
            html += ".joystick{position:absolute;width:40px;height:40px;background:#4CAF50;border-radius:50%;top:50%;left:50%;transform:translate(-50%,-50%);cursor:pointer;transition:none;}";
            html += ".joystick.active{background:#45a049;}";
            html += ".values{text-align:center;margin-top:10px;}";
            html += ".value{color:#4CAF50;font-weight:bold;display:inline-block;margin:5px 15px;}";
            html += "button{background:#4CAF50;color:white;border:none;padding:10px 20px;margin:5px;border-radius:4px;cursor:pointer;font-size:14px;}";
            html += "button:active{background:#45a049;}";
            html += ".status{background:#333;padding:10px;margin:10px 0;border-radius:4px;text-align:center;}";
            html += ".aux-container{background:#333;padding:20px;margin:15px;border-radius:8px;text-align:center;}";
            html += "@media(max-width:600px){.stick-container{width:100%;display:block;}}</style></head><body>";
            html += "<div class='container'><h1>FPV RC Control</h1>";
            html += "<div class='status' id='status'>Connecting...</div>";
            
            // Stick 1 (Aileron/Roll, Elevator/Pitch)
            html += "<div class='stick-container'><h2>Stick 1</h2>";
            html += "<div class='joystick-wrapper' id='js1'>";
            html += "<div class='joystick' id='handle1'></div>";
            html += "</div>";
            html += "<div class='values'>";
            html += "<div>Aileron [A]: <span class='value' id='ch0'>1500</span></div>";
            html += "<div>Elevator [E]: <span class='value' id='ch1'>1500</span></div>";
            html += "</div></div>";
            
            // Stick 2 (Throttle, Rudder/Yaw)
            html += "<div class='stick-container'><h2>Stick 2</h2>";
            html += "<div class='joystick-wrapper' id='js2'>";
            html += "<div class='joystick' id='handle2'></div>";
            html += "</div>";
            html += "<div class='values'>";
            html += "<div>Throttle [T]: <span class='value' id='ch2'>1500</span></div>";
            html += "<div>Rudder [R]: <span class='value' id='ch3'>1500</span></div>";
            html += "</div></div>";
            
            // Auxiliary channels (buttons)
            html += "<div class='aux-container'><h2>Auxiliary Channels</h2>";
            html += "<button id='aux1' onclick='toggleAux(4)'>Aux 1: OFF</button>";
            html += "<button id='aux2' onclick='toggleAux(5)'>Aux 2: OFF</button>";
            html += "<button id='aux3' onclick='toggleAux(6)'>Aux 3: OFF</button>";
            html += "<button id='aux4' onclick='toggleAux(7)'>Aux 4: OFF</button>";
            html += "</div>";
            
            // Auxiliary channels (buttons)
            html += "<div class='stick'><h2>Auxiliary Channels</h2>";
            html += "<button id='aux1' onclick='toggleAux(4)'>Aux 1: OFF</button>";
            html += "<button id='aux2' onclick='toggleAux(5)'>Aux 2: OFF</button>";
            html += "<button id='aux3' onclick='toggleAux(6)'>Aux 3: OFF</button>";
            html += "<button id='aux4' onclick='toggleAux(7)'>Aux 4: OFF</button>";
            html += "</div>";
            
            html += "<script>";
            html += "let channels = [1500,1500,1500,1500,1000,1000,1000,1000];";
            html += "let auxStates = [false,false,false,false];";
            html += "let activeJoystick = null;";
            html += "function setupJoystick(id, chX, chY) {";
            html += "  const wrapper = document.getElementById('js' + id);";
            html += "  const handle = document.getElementById('handle' + id);";
            html += "  const rect = wrapper.getBoundingClientRect();";
            html += "  const centerX = rect.left + rect.width / 2;";
            html += "  const centerY = rect.top + rect.height / 2;";
            html += "  const radius = rect.width / 2 - 20;";
            html += "  function moveJoystick(e) {";
            html += "    if (activeJoystick !== id) return;";
            html += "    const clientX = e.touches ? e.touches[0].clientX : e.clientX;";
            html += "    const clientY = e.touches ? e.touches[0].clientY : e.clientY;";
            html += "    let dx = clientX - centerX;";
            html += "    let dy = clientY - centerY;";
            html += "    const dist = Math.sqrt(dx*dx + dy*dy);";
            html += "    if (dist > radius) { dx = dx * radius / dist; dy = dy * radius / dist; }";
            html += "    handle.style.transform = 'translate(calc(-50% + ' + dx + 'px), calc(-50% + ' + dy + 'px))';";
            html += "    const xVal = Math.round(1500 + (dx / radius) * 500);";
            html += "    const yVal = Math.round(1500 - (dy / radius) * 500);";
            html += "    channels[chX] = Math.max(1000, Math.min(2000, xVal));";
            html += "    channels[chY] = Math.max(1000, Math.min(2000, yVal));";
            html += "    updateDisplay(chX); updateDisplay(chY); sendChannels();";
            html += "    e.preventDefault();";
            html += "  }";
            html += "  function stopJoystick() {";
            html += "    if (activeJoystick === id) {";
            html += "      activeJoystick = null;";
            html += "      handle.classList.remove('active');";
            html += "      handle.style.transform = 'translate(-50%, -50%)';";
            html += "      channels[chX] = 1500; channels[chY] = 1500;";
            html += "      updateDisplay(chX); updateDisplay(chY); sendChannels();";
            html += "    }";
            html += "  }";
            html += "  wrapper.addEventListener('mousedown', (e) => { activeJoystick = id; handle.classList.add('active'); moveJoystick(e); });";
            html += "  wrapper.addEventListener('touchstart', (e) => { activeJoystick = id; handle.classList.add('active'); moveJoystick(e); });";
            html += "  document.addEventListener('mousemove', moveJoystick);";
            html += "  document.addEventListener('touchmove', moveJoystick);";
            html += "  document.addEventListener('mouseup', stopJoystick);";
            html += "  document.addEventListener('touchend', stopJoystick);";
            html += "}";
            html += "setupJoystick(1, 0, 1);"; // Stick 1: Aileron (ch0), Elevator (ch1)
            html += "setupJoystick(2, 2, 3);"; // Stick 2: Throttle (ch2), Rudder (ch3)
            html += "function toggleAux(ch){auxStates[ch-4]=!auxStates[ch-4];channels[ch]=auxStates[ch-4]?2000:1000;document.getElementById('aux'+(ch-3)).textContent='Aux '+(ch-3)+': '+(auxStates[ch-4]?'ON':'OFF');sendChannels();}";
            html += "function updateDisplay(ch){document.getElementById('ch'+ch).textContent=channels[ch];}";
            html += "function sendChannels(){fetch('/channels',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ch:channels})}).then(r=>{document.getElementById('status').textContent='Connected - Last update: '+new Date().toLocaleTimeString();}).catch(e=>{document.getElementById('status').textContent='Error: '+e;});}";
            html += "setInterval(sendChannels,50);"; // Send every 50ms (~20Hz)
            html += "document.getElementById('status').textContent='Connected';";
            html += "</script></body></html>";
            
            webClient.println("HTTP/1.1 200 OK");
            webClient.println("Content-Type: text/html");
            webClient.println("Access-Control-Allow-Origin: *");
            webClient.println("Connection: close");
            webClient.println();
            webClient.print(html);
            webClient.stop();
            } else if (request.indexOf("POST /channels") >= 0) {
            // Handle channel data POST request
            // Read body (wait for it to arrive)
            String body = "";
            unsigned long startTime = millis();
            while (millis() - startTime < 500) { // 500ms timeout
                if (webClient.available()) {
                    body += (char)webClient.read();
                } else {
                    delay(1);
                }
                // Check if we've read enough (typical JSON body is ~50-100 bytes)
                if (body.length() > 0 && !webClient.available() && body.length() > 20) {
                    break; // Likely got the full body
                }
            }
            
            // Parse JSON (simple parsing)
            // Format: {"ch":[1500,1500,1500,1500,1000,1000,1000,1000]}
            int startIdx = body.indexOf("\"ch\":[");
            if (startIdx >= 0) {
                startIdx += 6; // Skip "ch":[
                String chData = body.substring(startIdx);
                chData = chData.substring(0, chData.indexOf(']'));
                
                // Parse comma-separated values
                int chIdx = 0;
                int lastComma = -1;
                for (int i = 0; i < chData.length() && chIdx < 8; i++) {
                    if (chData.charAt(i) == ',' || i == chData.length() - 1) {
                        String valStr = chData.substring(lastComma + 1, i == chData.length() - 1 ? i + 1 : i);
                        channels[chIdx] = valStr.toInt();
                        if (channels[chIdx] < 1000) channels[chIdx] = 1000;
                        if (channels[chIdx] > 2000) channels[chIdx] = 2000;
                        chIdx++;
                        lastComma = i;
                    }
                }
                
                lastDataReceived = millis();
                
                // Send response
                webClient.println("HTTP/1.1 200 OK");
                webClient.println("Content-Type: application/json");
                webClient.println("Access-Control-Allow-Origin: *");
                webClient.println("Connection: close");
                webClient.println();
                webClient.println("{\"status\":\"ok\"}");
            } else {
                webClient.println("HTTP/1.1 400 Bad Request");
                webClient.println("Access-Control-Allow-Origin: *");
                webClient.println("Connection: close");
                webClient.println();
            }
            webClient.stop();
            } else if (request.indexOf("GET /discover") >= 0) {
            // Discovery endpoint - return RX IP
            IPAddress rxIP = WiFi.localIP();
            webClient.println("HTTP/1.1 200 OK");
            webClient.println("Content-Type: application/json");
            webClient.println("Access-Control-Allow-Origin: *");
            webClient.println("Connection: close");
            webClient.println();
            webClient.print("{\"ip\":\"");
            webClient.print(rxIP);
            webClient.println("\"}");
            webClient.stop();
            } else if (request.indexOf("POST /channels-binary") >= 0) {
            // Binary channel data endpoint (for web client with security)
            // Read binary frame (22 bytes: 16 data + 2 seq + 4 HMAC)
            if (webClient.available() >= FRAME_SIZE) {
                uint8_t frame[FRAME_SIZE];
                size_t bytesRead = 0;
                while (bytesRead < FRAME_SIZE && webClient.available()) {
                    frame[bytesRead++] = webClient.read();
                }
                
                if (bytesRead == FRAME_SIZE) {
                    // Decode frame with security validation
                    uint16_t sequence = 0;
                    if (Protocol::decodeFrame(frame, channels, &sequence, &security, &lastSequence)) {
                        lastDataReceived = millis();
                        
                        // Send success response
                        webClient.println("HTTP/1.1 200 OK");
                        webClient.println("Content-Type: application/json");
                        webClient.println("Access-Control-Allow-Origin: *");
                        webClient.println("Connection: close");
                        webClient.println();
                        webClient.println("{\"status\":\"ok\",\"sequence\":");
                        webClient.print(sequence);
                        webClient.println("}");
                    } else {
                        // Security check failed
                        webClient.println("HTTP/1.1 403 Forbidden");
                        webClient.println("Content-Type: application/json");
                        webClient.println("Access-Control-Allow-Origin: *");
                        webClient.println("Connection: close");
                        webClient.println();
                        webClient.println("{\"status\":\"error\",\"message\":\"Security check failed\"}");
                    }
                } else {
                    webClient.println("HTTP/1.1 400 Bad Request");
                    webClient.println("Access-Control-Allow-Origin: *");
                    webClient.println("Connection: close");
                    webClient.println();
                }
            } else {
                webClient.println("HTTP/1.1 400 Bad Request");
                webClient.println("Access-Control-Allow-Origin: *");
                webClient.println("Connection: close");
                webClient.println();
            }
            webClient.stop();
            } else {
                webClient.println("HTTP/1.1 404 Not Found");
                webClient.println("Access-Control-Allow-Origin: *");
                webClient.println("Connection: close");
                webClient.println();
                webClient.stop();
            }
        }
    }
}
    
    // Check for TCP client connection (binary protocol on port 8888)
    WiFiClient client = server.available();
    
    if (client) {
        if (!clientConnected) {
            Serial.println("New client connected");
            clientConnected = true;
        }
        
        // Read data from client
        if (client.available() >= FRAME_SIZE) {
            uint8_t frame[FRAME_SIZE];
            size_t bytesRead = client.readBytes(frame, FRAME_SIZE);
            
            if (bytesRead == FRAME_SIZE) {
                // Decode frame with security validation
                uint16_t sequence = 0;
                if (Protocol::decodeFrame(frame, channels, &sequence, &security, &lastSequence)) {
#ifdef TEST_MODE
                    Serial.println("Real data received - test mode still active (compile-time flag)");
#endif
                    
                    // Channels will be sent in main loop
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
        if (millis() - lastDataReceived > TIMEOUT_MS) {
            Serial.println("Client timeout, disconnecting");
            client.stop();
            clientConnected = false;
            // Set channels to failsafe (minimum position)
            for (int i = 0; i < 8; i++) {
                channels[i] = 1000;
            }
        }
    } else {
        if (clientConnected) {
            Serial.println("Client disconnected");
            clientConnected = false;
            // Set channels to failsafe (minimum position)
            for (int i = 0; i < 8; i++) {
                channels[i] = 1000;
            }
        }
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
        
        // Pack channels into CRSF format
        // Convert from microseconds (1000-2000) to CRSF values (191-1792)
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
        
        // Debug: Print every 50 packets (once per second) to confirm sending
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

