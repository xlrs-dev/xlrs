// TX Side - BLE Central using low-level BTstack API
// This bypasses BTstackLib which has known issues

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ADS1X15.h>
#include "Protocol.h"
#include "Security.h"

// Use BTstackLib for initialization, but with our own low-level callbacks
#include <BTstackLib.h>

// Low-level BTstack includes for custom GATT handling
extern "C" {
#include <btstack.h>
#include <ble/gatt_client.h>
#include <ble/sm.h>
}

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

// Button pins
#define BUTTON1_PIN 6
#define BUTTON2_PIN 7
#define BUTTON3_PIN 8
#define BUTTON4_PIN 9

// Target service and characteristic UUIDs (16-bit short form)
#define FPV_SERVICE_UUID16     0xFF00
#define FPV_TX_CHAR_UUID16     0xFF01

// EEPROM addresses
#define EEPROM_SIZE 512
#define BOND_MAGIC_ADDR 99
#define BOND_MAC_ADDR 100
#define BOND_MAGIC_VALUE 0xAA

// Connection states
typedef enum {
    STATE_IDLE,
    STATE_SCANNING,
    STATE_CONNECTING,
    STATE_CONNECTED,
    STATE_DISCOVERING_SERVICES,
    STATE_DISCOVERING_CHARACTERISTICS,
    STATE_READY,
    STATE_ERROR
} ble_state_t;

// Menu states
typedef enum {
    MENU_NORMAL_OPERATION,
    MENU_MAIN,
    MENU_SCANNING,
    MENU_SELECT_DEVICE,
    MENU_CONNECTING
} menu_state_t;

// Discovered device info
typedef struct {
    bd_addr_t address;
    bd_addr_type_t address_type;
    int8_t rssi;
    char name[32];
    bool has_fpv_service;
} discovered_device_t;

// Application state
static ble_state_t ble_state = STATE_IDLE;
static menu_state_t menu_state = MENU_NORMAL_OPERATION;

// BTstack objects
static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_timer_source_t scan_timer;

// Connection state
static hci_con_handle_t connection_handle = HCI_CON_HANDLE_INVALID;
static bool connected = false;

// GATT client state
static gatt_client_service_t fpv_service;
static gatt_client_characteristic_t tx_characteristic;
static bool service_found = false;
static bool characteristic_found = false;
static uint16_t tx_char_value_handle = 0;

// Discovered devices
#define MAX_DEVICES 10
static discovered_device_t discovered_devices[MAX_DEVICES];
static int discovered_count = 0;
static int selected_device = 0;

// Bonded device
static bd_addr_t bonded_address;
static bd_addr_type_t bonded_address_type = BD_ADDR_TYPE_LE_PUBLIC;
static bool has_bonded_device = false;

// Display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_ADS1115 ads;  // ADS1115 ADC on I2C bus

// ADS1115 status flag
bool ads_ok = false;

// Security
Security security;
uint16_t sequenceNumber = 0;

// Channel values
uint16_t channels[8] = {1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000};

// Timing
unsigned long lastDataSent = 0;
const unsigned long DATA_INTERVAL = 20;  // 50Hz
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL = 100;

// Status string
char status_str[32] = "Idle";

// Forward declarations
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void gatt_client_callback(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void start_scan(void);
static void stop_scan(void);
static void connect_to_device(int index);
static void connect_to_bonded_device(void);
static void discover_services(void);
static void discover_characteristics(void);
static void send_channel_data(void);
static void load_bonded_device(void);
static void save_bonded_device(void);

// EEPROM functions
static void load_bonded_device(void) {
    EEPROM.begin(EEPROM_SIZE);
    
    if (EEPROM.read(BOND_MAGIC_ADDR) != BOND_MAGIC_VALUE) {
        has_bonded_device = false;
        Serial.println("No bonded device found");
        return;
    }
    
    for (int i = 0; i < 6; i++) {
        bonded_address[i] = EEPROM.read(BOND_MAC_ADDR + i);
    }
    
    has_bonded_device = true;
    Serial.print("Loaded bonded device: ");
    Serial.println(bd_addr_to_str(bonded_address));
}

static void save_bonded_device(void) {
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.write(BOND_MAGIC_ADDR, BOND_MAGIC_VALUE);
    
    for (int i = 0; i < 6; i++) {
        EEPROM.write(BOND_MAC_ADDR + i, bonded_address[i]);
    }
    
    EEPROM.commit();
    has_bonded_device = true;
    Serial.println("Bonded device saved");
}

// Scan timeout callback
static void scan_timeout_handler(btstack_timer_source_t *ts) {
    (void)ts;
    Serial.println("Scan timeout");
    stop_scan();
    
    if (discovered_count > 0) {
        menu_state = MENU_SELECT_DEVICE;
        strcpy(status_str, "Select device");
    } else {
        menu_state = MENU_MAIN;
        strcpy(status_str, "No devices");
    }
}

// Start BLE scanning
static void start_scan(void) {
    Serial.println("Starting BLE scan...");
    ble_state = STATE_SCANNING;
    strcpy(status_str, "Scanning...");
    
    // Reset discovered devices
    discovered_count = 0;
    selected_device = 0;
    
    // Set scan parameters
    gap_set_scan_parameters(1, 0x0030, 0x0030);  // Active scan, 30ms interval/window
    gap_start_scan();
    
    // Set scan timeout (10 seconds)
    btstack_run_loop_set_timer(&scan_timer, 10000);
    btstack_run_loop_set_timer_handler(&scan_timer, scan_timeout_handler);
    btstack_run_loop_add_timer(&scan_timer);
}

// Stop BLE scanning
static void stop_scan(void) {
    Serial.println("Stopping scan");
    gap_stop_scan();
    btstack_run_loop_remove_timer(&scan_timer);
    ble_state = STATE_IDLE;
}

// Connect to discovered device
static void connect_to_device(int index) {
    if (index >= discovered_count) return;
    
    Serial.print("Connecting to: ");
    Serial.println(bd_addr_to_str(discovered_devices[index].address));
    
    ble_state = STATE_CONNECTING;
    menu_state = MENU_CONNECTING;
    strcpy(status_str, "Connecting...");
    
    // Copy address for bonding
    memcpy(bonded_address, discovered_devices[index].address, 6);
    bonded_address_type = discovered_devices[index].address_type;
    
    // Connect
    gap_connect(discovered_devices[index].address, discovered_devices[index].address_type);
}

// Connect to bonded device
static void connect_to_bonded_device(void) {
    if (!has_bonded_device) return;
    
    Serial.print("Connecting to bonded device: ");
    Serial.println(bd_addr_to_str(bonded_address));
    
    ble_state = STATE_CONNECTING;
    menu_state = MENU_CONNECTING;
    strcpy(status_str, "Auto-connect...");
    
    gap_connect(bonded_address, bonded_address_type);
}

// Discover GATT services
static void discover_services(void) {
    Serial.println("Discovering services...");
    ble_state = STATE_DISCOVERING_SERVICES;
    strcpy(status_str, "Finding service");
    
    service_found = false;
    
    // Discover all services
    gatt_client_discover_primary_services(gatt_client_callback, connection_handle);
}

// Discover characteristics for FPV service
static void discover_characteristics(void) {
    Serial.println("Discovering characteristics...");
    ble_state = STATE_DISCOVERING_CHARACTERISTICS;
    strcpy(status_str, "Finding chars");
    
    characteristic_found = false;
    
    // Discover characteristics for our service
    gatt_client_discover_characteristics_for_service(gatt_client_callback, connection_handle, &fpv_service);
}

// Send channel data
static void send_channel_data(void) {
    if (!connected || !characteristic_found || tx_char_value_handle == 0) {
        return;
    }
    
    if (millis() - lastDataSent < DATA_INTERVAL) {
        return;
    }
    lastDataSent = millis();
    
    // Increment sequence
    sequenceNumber++;
    if (sequenceNumber == 0) sequenceNumber = 1;
    
    // Encode frame
    uint8_t frame[FRAME_SIZE];
    Protocol::encodeFrame(channels, sequenceNumber, &security, frame);
    
    // Send via GATT write without response
    uint8_t status = gatt_client_write_value_of_characteristic_without_response(
        connection_handle,
        tx_char_value_handle,
        FRAME_SIZE,
        frame
    );
    
    // Debug occasionally
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 1000) {
        Serial.print("Sent seq ");
        Serial.print(sequenceNumber);
        Serial.print(" to handle 0x");
        Serial.print(tx_char_value_handle, HEX);
        Serial.print(", status: ");
        Serial.println(status);
        lastDebug = millis();
    }
}

// GATT client callback
static void gatt_client_callback(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;
    
    if (packet_type != HCI_EVENT_PACKET) return;
    
    uint8_t event_type = hci_event_packet_get_type(packet);
    
    switch (event_type) {
        case GATT_EVENT_SERVICE_QUERY_RESULT: {
            gatt_client_service_t service;
            gatt_event_service_query_result_get_service(packet, &service);
            
            Serial.print("Service found: ");
            Serial.print(service.uuid16, HEX);
            Serial.print(" (handles 0x");
            Serial.print(service.start_group_handle, HEX);
            Serial.print(" - 0x");
            Serial.print(service.end_group_handle, HEX);
            Serial.println(")");
            
            // Check if this is our FPV service
            if (service.uuid16 == FPV_SERVICE_UUID16) {
                Serial.println("*** FPV Service found! ***");
                service_found = true;
                fpv_service = service;
            }
            break;
        }
        
        case GATT_EVENT_QUERY_COMPLETE: {
            uint8_t att_status = gatt_event_query_complete_get_att_status(packet);
            Serial.print("Query complete, status: ");
            Serial.println(att_status);
            
            if (ble_state == STATE_DISCOVERING_SERVICES) {
                if (service_found) {
                    // Service found, now discover characteristics
                    discover_characteristics();
                } else {
                    Serial.println("FPV service not found!");
                    strcpy(status_str, "No FPV service");
                    ble_state = STATE_ERROR;
                }
            } else if (ble_state == STATE_DISCOVERING_CHARACTERISTICS) {
                if (characteristic_found) {
                    Serial.println("*** Ready to send data! ***");
                    Serial.print("TX handle: 0x");
                    Serial.println(tx_char_value_handle, HEX);
                    
                    ble_state = STATE_READY;
                    menu_state = MENU_NORMAL_OPERATION;
                    connected = true;
                    strcpy(status_str, "Ready");
                    
                    // Save bonded device (this may block briefly)
                    Serial.println("Saving bonded device...");
                    save_bonded_device();
                    Serial.println("Bonded device saved, continuing...");
                    
                    // Force display update on next loop iteration
                    lastDisplayUpdate = 0;
                    Serial.println("Ready state set, display will update in loop()");
                } else {
                    Serial.println("TX characteristic not found!");
                    strcpy(status_str, "No TX char");
                    ble_state = STATE_ERROR;
                }
            }
            break;
        }
        
        case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT: {
            gatt_client_characteristic_t characteristic;
            gatt_event_characteristic_query_result_get_characteristic(packet, &characteristic);
            
            Serial.print("Characteristic found: 0x");
            Serial.print(characteristic.uuid16, HEX);
            Serial.print(", value handle: 0x");
            Serial.print(characteristic.value_handle, HEX);
            Serial.print(", properties: 0x");
            Serial.println(characteristic.properties, HEX);
            
            // Check if this is our TX characteristic
            if (characteristic.uuid16 == FPV_TX_CHAR_UUID16) {
                Serial.println("*** TX Characteristic found! ***");
                characteristic_found = true;
                tx_characteristic = characteristic;
                tx_char_value_handle = characteristic.value_handle;
            }
            break;
        }
        
        default:
            break;
    }
}

// Main HCI packet handler
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;
    
    if (packet_type != HCI_EVENT_PACKET) return;
    
    uint8_t event_type = hci_event_packet_get_type(packet);
    
    switch (event_type) {
        case BTSTACK_EVENT_STATE:
            // BTstack state changed - just log it
            Serial.print("[BLE] BTstack state: ");
            Serial.println(btstack_event_state_get_state(packet));
            break;
            
        case GAP_EVENT_ADVERTISING_REPORT: {
            bd_addr_t addr;
            gap_event_advertising_report_get_address(packet, addr);
            bd_addr_type_t addr_type = (bd_addr_type_t)gap_event_advertising_report_get_address_type(packet);
            int8_t rssi = gap_event_advertising_report_get_rssi(packet);
            uint8_t length = gap_event_advertising_report_get_data_length(packet);
            const uint8_t *data = gap_event_advertising_report_get_data(packet);
            
            // Check if this device advertises our FPV service
            bool has_fpv_service = false;
            char name[32] = "Unknown";
            
            // Parse advertisement data
            ad_context_t context;
            for (ad_iterator_init(&context, length, data); ad_iterator_has_more(&context); ad_iterator_next(&context)) {
                uint8_t data_type = ad_iterator_get_data_type(&context);
                uint8_t data_len = ad_iterator_get_data_len(&context);
                const uint8_t *ad_data = ad_iterator_get_data(&context);
                
                switch (data_type) {
                    case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS:
                    case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS:
                        for (int i = 0; i < data_len; i += 2) {
                            uint16_t uuid16 = little_endian_read_16(ad_data, i);
                            if (uuid16 == FPV_SERVICE_UUID16) {
                                has_fpv_service = true;
                            }
                        }
                        break;
                        
                    case BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME:
                    case BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME:
                        memcpy(name, ad_data, data_len < 31 ? data_len : 31);
                        name[data_len < 31 ? data_len : 31] = '\0';
                        break;
                }
            }
            
            // Auto-connect if this is our bonded device
            if (has_bonded_device && ble_state == STATE_SCANNING) {
                if (memcmp(addr, bonded_address, 6) == 0) {
                    Serial.println("Found bonded device, connecting...");
                    stop_scan();
                    gap_connect(addr, addr_type);
                    ble_state = STATE_CONNECTING;
                    strcpy(status_str, "Found bond");
                    return;
                }
            }
            
            // Only add devices that have our service
            if (has_fpv_service && discovered_count < MAX_DEVICES) {
                // Check if already discovered
                bool already_found = false;
                for (int i = 0; i < discovered_count; i++) {
                    if (memcmp(discovered_devices[i].address, addr, 6) == 0) {
                        discovered_devices[i].rssi = rssi;
                        already_found = true;
                        break;
                    }
                }
                
                if (!already_found) {
                    Serial.print("FPV Device found: ");
                    Serial.print(name);
                    Serial.print(" (");
                    Serial.print(bd_addr_to_str(addr));
                    Serial.print(") RSSI: ");
                    Serial.println(rssi);
                    
                    memcpy(discovered_devices[discovered_count].address, addr, 6);
                    discovered_devices[discovered_count].address_type = addr_type;
                    discovered_devices[discovered_count].rssi = rssi;
                    strncpy(discovered_devices[discovered_count].name, name, 31);
                    discovered_devices[discovered_count].has_fpv_service = true;
                    discovered_count++;
                }
            }
            break;
        }
        
        case HCI_EVENT_LE_META:
            switch (hci_event_le_meta_get_subevent_code(packet)) {
                case HCI_SUBEVENT_LE_CONNECTION_COMPLETE: {
                    uint8_t status = hci_subevent_le_connection_complete_get_status(packet);
                    if (status == 0) {
                        connection_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                        connected = true;
                        
                        Serial.print("[BLE] Connected, handle: 0x");
                        Serial.println(connection_handle, HEX);
                        strcpy(status_str, "Connected");
                        
                        // Start service discovery
                        discover_services();
                    } else {
                        Serial.print("[BLE] Connection failed: ");
                        Serial.println(status);
                        ble_state = STATE_ERROR;
                        strcpy(status_str, "Conn failed");
                    }
                    break;
                }
            }
            break;
            
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            Serial.println("[BLE] Disconnected");
            connection_handle = HCI_CON_HANDLE_INVALID;
            connected = false;
            characteristic_found = false;
            service_found = false;
            tx_char_value_handle = 0;
            ble_state = STATE_IDLE;
            strcpy(status_str, "Disconnected");
            
            // Try to reconnect if we have a bonded device
            if (has_bonded_device) {
                delay(1000);
                connect_to_bonded_device();
            }
            break;
            
        default:
            break;
    }
}

// Read input devices
void readInputs() {
    // Read analog sticks from ADS1115 (16-bit ADC: 0-32767)
    // Use default center values if ADS1115 isn't available
    int16_t stick1_x = 16384;  // Center value (32767/2)
    int16_t stick1_y = 16384;
    int16_t stick2_x = 16384;
    int16_t stick2_y = 16384;
    
    // Only read from ADS1115 if initialized and in normal operation
    if (ads_ok && menu_state == MENU_NORMAL_OPERATION) {
        // Read from ADS1115 - Adafruit library handles I2C communication
        // These reads are fast (~1ms each) and shouldn't block
        // If ADS1115 isn't responding, reads will return last value or 0
        stick1_x = ads.readADC_SingleEnded(STICK1_X_CHANNEL);
        stick1_y = ads.readADC_SingleEnded(STICK1_Y_CHANNEL);
        stick2_x = ads.readADC_SingleEnded(STICK2_X_CHANNEL);
        stick2_y = ads.readADC_SingleEnded(STICK2_Y_CHANNEL);
    }
    
    // Map to channel values (1000-2000)
    // Calibrated based on actual ADC readings:
    // ADC range appears to be ~2917 (min) to ~23420 (max), center ~13199
    // This maps to: 1089 (min) -> 1000, 1403 (center) -> 1500, 1715 (max) -> 2000
    if (menu_state == MENU_NORMAL_OPERATION) {
        // Map from actual ADC range to desired RC range
        // Using measured values: min ADC ~2917, center ~13199, max ~23420
        channels[0] = map(stick1_x, 2917, 23420, 1000, 2000);
        channels[1] = map(stick1_y, 2917, 23420, 1000, 2000);
        channels[2] = map(stick2_x, 2917, 23420, 1000, 2000);
        channels[3] = map(stick2_y, 2917, 23420, 1000, 2000);
        
        // Clamp to valid RC range
        channels[0] = constrain(channels[0], 1000, 2000);
        channels[1] = constrain(channels[1], 1000, 2000);
        channels[2] = constrain(channels[2], 1000, 2000);
        channels[3] = constrain(channels[3], 1000, 2000);
        
        // Read buttons
        bool btn1 = digitalRead(BUTTON1_PIN) == LOW;
        bool btn2 = digitalRead(BUTTON2_PIN) == LOW;
        bool btn3 = digitalRead(BUTTON3_PIN) == LOW;
        bool btn4 = digitalRead(BUTTON4_PIN) == LOW;
        
        channels[4] = btn1 ? 2000 : 1000;
        channels[5] = btn2 ? 2000 : 1000;
        channels[6] = btn3 ? 2000 : 1000;
        channels[7] = btn4 ? 2000 : 1000;
        
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

// Update display
void updateDisplay() {
    if (millis() - lastDisplayUpdate < DISPLAY_INTERVAL) return;
    lastDisplayUpdate = millis();
    
    // Clear display buffer
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    
    switch (menu_state) {
        case MENU_NORMAL_OPERATION:
            display.print("Status: ");
            display.println(status_str);
            if (connected && characteristic_found) {
                display.print("Handle: 0x");
                display.println(tx_char_value_handle, HEX);
            }
            display.print("Ch: ");
            display.print(channels[0]);
            display.print(" ");
            display.print(channels[1]);
            display.print(" ");
            display.print(channels[2]);
            display.print(" ");
            display.println(channels[3]);
            // Connection indicator
            display.fillRect(120, 0, 8, 8, connected ? SSD1306_WHITE : SSD1306_BLACK);
            display.drawRect(120, 0, 8, 8, SSD1306_WHITE);
            break;
            
        case MENU_MAIN:
            display.println("=== MENU ===");
            display.println("> Scan");
            display.println("  Exit");
            display.println("");
            display.println("Btn2: Select");
            display.println("Btn1: Back (hold=pair)");
            break;
            
        case MENU_SCANNING:
            display.println("Scanning...");
            display.print("Found: ");
            display.println(discovered_count);
            break;
            
        case MENU_SELECT_DEVICE:
            display.println("Select Device:");
            for (int i = 0; i < min(discovered_count, 4); i++) {
                display.print(i == selected_device ? "> " : "  ");
                display.println(discovered_devices[i].name);
            }
            break;
            
        case MENU_CONNECTING:
            display.println("Connecting...");
            display.println(status_str);
            break;
    }
    
    // Update physical display
    display.display();
    
    // Verify display was updated (check if display buffer was written)
    // Note: This is a best-effort check - display.display() doesn't return error
}

// Handle buttons
void handleButtons() {
    static unsigned long btn1_press_start = 0;
    static bool btn1_pressed = false;
    static unsigned long last_btn2 = 0;
    static unsigned long last_btn3 = 0;
    static unsigned long all_buttons_press_start = 0;
    static bool all_buttons_pressed = false;
    
    bool btn1 = digitalRead(BUTTON1_PIN) == LOW;
    bool btn2 = digitalRead(BUTTON2_PIN) == LOW;
    bool btn3 = digitalRead(BUTTON3_PIN) == LOW;
    bool btn4 = digitalRead(BUTTON4_PIN) == LOW;
    
    // Check if all buttons are pressed (for entering menu when connected)
    bool all_pressed = btn1 && btn2 && btn3 && btn4;
    
    // When connected and transmitting, require ALL buttons to enter menu
    if (connected && menu_state == MENU_NORMAL_OPERATION) {
        if (all_pressed && !all_buttons_pressed) {
            all_buttons_pressed = true;
            all_buttons_press_start = millis();
        } else if (!all_pressed && all_buttons_pressed) {
            all_buttons_pressed = false;
            unsigned long duration = millis() - all_buttons_press_start;
            
            // Require all buttons held for at least 500ms to prevent accidental activation
            if (duration >= 500) {
                Serial.println("All buttons pressed - entering menu");
                menu_state = MENU_MAIN;
            }
        }
        // Don't process individual buttons when connected and transmitting
        return;
    }
    
    // Normal button handling when not connected or already in menu
    // Button 1: Long press (3s) = enter pairing mode
    if (btn1 && !btn1_pressed) {
        btn1_pressed = true;
        btn1_press_start = millis();
    } else if (!btn1 && btn1_pressed) {
        btn1_pressed = false;
        unsigned long duration = millis() - btn1_press_start;
        
        if (duration >= 3000) {
            // Long press - enter pairing/scan mode
            Serial.println("Long press - entering scan mode");
            if (connected) {
                gap_disconnect(connection_handle);
            }
            menu_state = MENU_SCANNING;
            start_scan();
        } else if (duration >= 200) {
            // Short press - back
            if (menu_state == MENU_SELECT_DEVICE) {
                menu_state = MENU_MAIN;
            } else if (menu_state == MENU_MAIN) {
                menu_state = MENU_NORMAL_OPERATION;
            }
        }
    }
    
    // Button 2: Select/Confirm
    if (btn2 && millis() - last_btn2 > 300) {
        last_btn2 = millis();
        
        if (menu_state == MENU_SELECT_DEVICE && discovered_count > 0) {
            connect_to_device(selected_device);
        } else if (menu_state == MENU_MAIN) {
            menu_state = MENU_SCANNING;
            start_scan();
        }
    }
    
    // Button 3: Navigate / Menu
    if (btn3 && millis() - last_btn3 > 300) {
        last_btn3 = millis();
        
        if (menu_state == MENU_NORMAL_OPERATION) {
            menu_state = MENU_MAIN;
        } else if (menu_state == MENU_SELECT_DEVICE) {
            selected_device = (selected_device + 1) % discovered_count;
        }
    }
}

void setup() {
    Serial.begin(115200);
    
    unsigned long start = millis();
    while (!Serial && (millis() - start < 2000)) {
        delay(10);
    }
    delay(500);
    
    Serial.println("=== FPV Transmitter (BLE - Low Level API) ===");
    
    // Initialize I2C for OLED and ADS1115
    Wire.setSDA(I2C_SDA);
    Wire.setSCL(I2C_SCL);
    Wire.begin();
    
    // Start with standard I2C speed (100kHz) for compatibility
    // We'll increase it after both devices are initialized
    Wire.setClock(100000);  // 100kHz standard mode
    Serial.println("I2C bus initialized at 100kHz");
    
    // Scan I2C bus to see what devices are present
    Serial.println("Scanning I2C bus...");
    byte error, address;
    int nDevices = 0;
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);
            Serial.println(" !");
            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0) {
        Serial.println("No I2C devices found!");
    } else {
        Serial.print("Found ");
        Serial.print(nDevices);
        Serial.println(" device(s)");
    }
    
    // Initialize OLED first (at standard speed)
    if (!display.begin(SSD1306_SWITCHCAPVCC, I2C_ADDRESS)) {
        Serial.println("OLED init failed!");
        while(1);
    }
    Serial.print("OLED initialized at address 0x");
    Serial.println(I2C_ADDRESS, HEX);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("FPV TX Starting...");
    display.display();
    
    // Small delay to ensure I2C bus is ready
    delay(100);
    
    // Initialize ADS1115
    Serial.print("Attempting to initialize ADS1115 at address 0x");
    Serial.println(ADS1115_ADDRESS, HEX);
    
    if (!ads.begin(ADS1115_ADDRESS, &Wire)) {
        Serial.println("Failed to initialize ADS1115");
        Serial.println("Trying alternative addresses...");
        
        // Try alternative addresses (0x49, 0x4A, 0x4B)
        bool found = false;
        uint8_t alt_addresses[] = {0x49, 0x4A, 0x4B};
        for (int i = 0; i < 3; i++) {
            Serial.print("Trying address 0x");
            Serial.println(alt_addresses[i], HEX);
            if (ads.begin(alt_addresses[i], &Wire)) {
                Serial.print("ADS1115 found at address 0x");
                Serial.println(alt_addresses[i], HEX);
                found = true;
                break;
            }
        }
        
        if (!found) {
            Serial.println("ADS1115 not found at any address");
            Serial.println("Check wiring: SDA=GP4, SCL=GP5, VCC=3.3V, GND=GND");
            Serial.println("ADDR pin: GND=0x48, VDD=0x49, SDA=0x4A, SCL=0x4B");
            Serial.println("Will use default center values for sticks");
            ads_ok = false;
        } else {
            ads_ok = true;
        }
    } else {
        Serial.print("ADS1115 initialized successfully at address 0x");
        Serial.println(ADS1115_ADDRESS, HEX);
        ads_ok = true;
    }
    
    if (ads_ok) {
        // Set gain to 1 (default, ±4.096V range)
        // This gives us full 16-bit resolution (0-32767)
        ads.setGain(GAIN_ONE);
        
        // Set data rate to 250 SPS (Samples Per Second)
        // We need to read 4 channels at 50Hz (20ms intervals)
        // Minimum required: 4 channels * 50Hz = 200 SPS
        // Using 250 SPS provides headroom and ensures fast reads
        // Each channel read takes ~4ms at 250 SPS, all 4 channels ~16ms (well within 20ms budget)
        ads.setDataRate(RATE_ADS1115_250SPS);
        Serial.println("ADS1115 configured: Gain=1, DataRate=250 SPS");
        
        // Now increase I2C speed to 400kHz for faster reads
        // Both devices should be able to handle this after initialization
        Wire.setClock(400000);  // 400kHz fast mode
        Serial.println("I2C bus speed increased to 400kHz");
    }
    
    // Initialize buttons
    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);
    pinMode(BUTTON3_PIN, INPUT_PULLUP);
    pinMode(BUTTON4_PIN, INPUT_PULLUP);
    
    // Load bonded device
    load_bonded_device();
    
    // Initialize security
    if (security.begin()) {
        Serial.println("Security initialized");
    }
    
    // Register HCI event handler BEFORE BTstack.setup()
    Serial.println("Registering event handlers...");
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    
    // Initialize GATT client
    gatt_client_init();
    
    // Use BTstackLib to initialize the Bluetooth stack
    Serial.println("Calling BTstack.setup()...");
    BTstack.setup();
    
    Serial.println("BTstack initialized");
    
    Serial.println("=== TX Ready ===");
    
    // If we have a bonded device, start scanning for it
    if (has_bonded_device) {
        Serial.println("Starting scan for bonded device...");
        strcpy(status_str, "Scanning...");
        start_scan();
    } else {
        strcpy(status_str, "No bond");
        menu_state = MENU_MAIN;
    }
}

void loop() {
    // Process BTstack events using BTstackLib
    BTstack.loop();
    
    // Read inputs
    readInputs();
    
    // Handle buttons
    handleButtons();
    
    // Send data if ready
    if (ble_state == STATE_READY && menu_state == MENU_NORMAL_OPERATION) {
        send_channel_data();
    }
    
    // Update display (always update, even if state changed)
    updateDisplay();
    
    // Process BTstack events using BTstackLib
    BTstack.loop();
    
    // Read inputs
    readInputs();
    
    // Handle buttons
    handleButtons();
    
    // Send data if ready
    if (ble_state == STATE_READY && menu_state == MENU_NORMAL_OPERATION) {
        send_channel_data();
    }
    
    // Update display (always update, even if state changed)
    updateDisplay();
    
    // Heartbeat
    static unsigned long lastHeart = 0;
    if (millis() - lastHeart > 5000) {
        lastHeart = millis();
        Serial.print("TX alive, state: ");
        Serial.print(ble_state);
        Serial.print(", connected: ");
        Serial.print(connected);
        Serial.print(", char_found: ");
        Serial.print(characteristic_found);
        Serial.print(", handle: 0x");
        Serial.println(tx_char_value_handle, HEX);
    }
    
    delay(1);
}
