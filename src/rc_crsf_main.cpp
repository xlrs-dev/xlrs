// RC-CRSF: Remote control that outputs CRSF over serial to TX side.
// Handles display, battery, analog inputs, UI - everything except radio comms.
#include <Arduino.h>
#include <Wire.h>
#if defined(CRSF_CORE1_CHANNELS)
#include "pico/mutex.h"
#include "hardware/sync.h"
#endif
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#if !defined(INTERNAL_ADC)
#include <Adafruit_ADS1X15.h>
#endif
#include <math.h>

#include "UARTProtocol.h"
#include "CRSFSerialConnector.h"
#include "bq2562x.h"
#include "RCConfig.h"
#include "RCConfigProtocol.h"

static_assert(sizeof(rc_config_data_t) == 132, "rc_config_data_t size must match tools/rc-webui CONFIG_PAYLOAD_SIZE");
#include "debug_ndjson.h"

// OLED Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define I2C_ADDRESS 0x3C
// Display layout (text size 1: ~6x8 per char, 8px row height). Strict: nothing past 127px to avoid wrap.
#define ROW_H 8
#define COL_0 0
#define COL_RIGHT 64
#define COL_LINK COL_RIGHT
#define LINK_INDICATOR_X 118
#define LINK_INDICATOR_Y 0
#define LINK_INDICATOR_W 8
#define LINK_INDICATOR_H 8
#define BATT_BAR_X 88
#define BATT_BAR_W 4
#define BATT_BAR_H 5
#define BATT_BAR_GAP 1
#define TX_BATT_LOW_PCT 10
#define MAX_CHARS_PER_ROW 21

// ADS1115 Configuration
#define ADS1115_ADDRESS 0x48  // Default I2C address (can be 0x48-0x4B)

// Pin definitions
// I2C - ADC
#define I2C_SDA 4
#define I2C_SCL 5

// CRSF Serial pins (outputting CRSF to TX side)
#define CRSF_TX_PIN 8
#define CRSF_RX_PIN 9

// ADS1115 channel assignments (0-3) → CRSF mapping: A,E,R,T
// ADS ch0 → channels[0] → CRSF ch0 = Aileron  (A)
// ADS ch1 → channels[1] → CRSF ch1 = Elevator (E)
// ADS ch2 → channels[2] → CRSF ch2 = Rudder   (R)
// ADS ch3 → channels[3] → CRSF ch3 = Throttle (T)
#define AILERON_CHANNEL  0  // Stick1 X
#define ELEVATOR_CHANNEL 1  // Stick1 Y
#define RUDDER_CHANNEL   2  // Stick2 X
#define THROTTLE_CHANNEL 3  // Stick2 Y

#if defined(INTERNAL_ADC)
// RP2350 internal ADC pins (ADC0-3) for sticks
#define INTERNAL_ADC_AILERON_PIN  26   // ADC0 -> Stick1 X
#define INTERNAL_ADC_ELEVATOR_PIN 27   // ADC1 -> Stick1 Y
#define INTERNAL_ADC_RUDDER_PIN   28   // ADC2 -> Stick2 X
#define INTERNAL_ADC_THROTTLE_PIN 29   // ADC3 -> Stick2 Y
#define INTERNAL_ADC_BITS  12
#define INTERNAL_ADC_MAX   ((1 << INTERNAL_ADC_BITS) - 1)  // 4095
#define INTERNAL_ADC_SCALE_16BIT  (32767)  // scale 12-bit to 16-bit range for same calibration
#if defined(RC_STICK_ADC_12BIT)
// Keep hardware counts 0–4095 (no scaling to 32767)
static inline int16_t internal_adc_raw_to_stick(uint32_t raw) {
    return (int16_t)constrain(raw, 0u, (uint32_t)INTERNAL_ADC_MAX);
}
#else
static inline int16_t internal_adc_raw_to_stick(uint32_t raw) {
    return (int16_t)((raw * (uint32_t)INTERNAL_ADC_SCALE_16BIT) / (uint32_t)INTERNAL_ADC_MAX);
}
#endif

// Internal ADC: oversampling + 1st-order IIR low-pass + optional 2-tap temporal blend (see cfg->stick_low_pass).
static uint32_t internal_adc_oversample_pin_n(uint8_t pin, uint8_t os_log2) {
    if (os_log2 > 6) os_log2 = 6;
    const uint32_t n = 1u << os_log2;
    uint32_t sum = 0;
    for (uint32_t k = 0; k < n; k++) {
        sum += analogRead(pin);
    }
    return (sum + (n >> 1)) >> os_log2;
}

static int16_t s_internal_stick_lpf_z[4];
static uint8_t s_internal_stick_lpf_init_mask;
static int16_t s_frame_blend_prev[4];
static uint8_t s_frame_blend_init_mask;

static void internal_adc_reset_stick_filter(void) {
    s_internal_stick_lpf_init_mask = 0;
    s_frame_blend_init_mask = 0;
}

// Populates axis_out[4] in the same units as internal_adc_raw_to_stick (0..4095 or 0..32767).
static void internal_read_sticks_filtered(int16_t axis_out[4], const rc_config_data_t* cfg) {
    uint8_t level = 2;
    if (cfg) {
        level = cfg->stick_low_pass;
        if (level > 3) level = 2;
    }
    uint8_t os_log2 = 3;
    uint8_t lpf_shift = 2;
    bool blend = true;
    switch (level) {
        case 0:
            os_log2 = 3;
            lpf_shift = 0;
            blend = false;
            break;
        case 1:
            os_log2 = 3;
            lpf_shift = 2;
            blend = true;
            break;
        case 2:
            os_log2 = 4;
            lpf_shift = 3;
            blend = true;
            break;
        case 3:
        default:
            os_log2 = 4;
            lpf_shift = 4;
            blend = true;
            break;
    }

    const uint8_t pins[4] = {
        INTERNAL_ADC_AILERON_PIN,
        INTERNAL_ADC_ELEVATOR_PIN,
        INTERNAL_ADC_RUDDER_PIN,
        INTERNAL_ADC_THROTTLE_PIN,
    };
    if (!blend) {
        s_frame_blend_init_mask = 0;
    }

    for (int i = 0; i < 4; i++) {
        uint32_t avg = internal_adc_oversample_pin_n(pins[i], os_log2);
        int16_t x = internal_adc_raw_to_stick(avg);
        if (lpf_shift == 0) {
            axis_out[i] = x;
            s_internal_stick_lpf_init_mask &= (uint8_t)~(1u << i);
        } else if ((s_internal_stick_lpf_init_mask & (1u << i)) == 0) {
            s_internal_stick_lpf_z[i] = x;
            s_internal_stick_lpf_init_mask |= (1u << i);
            axis_out[i] = x;
        } else {
            int32_t z = (int32_t)s_internal_stick_lpf_z[i];
            z += (((int32_t)x - z) >> lpf_shift);
            s_internal_stick_lpf_z[i] = (int16_t)z;
            axis_out[i] = (int16_t)z;
        }
    }
    if (blend) {
        for (int i = 0; i < 4; i++) {
            int16_t y = axis_out[i];
            if ((s_frame_blend_init_mask & (1u << i)) == 0) {
                s_frame_blend_prev[i] = y;
                s_frame_blend_init_mask |= (1u << i);
            } else {
                int32_t b = ((int32_t)y + (int32_t)s_frame_blend_prev[i] + 1) >> 1;
                s_frame_blend_prev[i] = y;
                axis_out[i] = (int16_t)b;
            }
        }
    }
#if defined(RC_STICK_ADC_12BIT)
    const int32_t vmax = (int32_t)INTERNAL_ADC_MAX;
#else
    const int32_t vmax = 32767;
#endif
    for (int i = 0; i < 4; i++) {
        axis_out[i] = (int16_t)constrain((int32_t)axis_out[i], 0, vmax);
    }
}
#endif

// Toggle switches (dual-pin reading)
// Logic: IN1 high = 1000, IN2 high = 2000, both low = 1500
// T1: GPIO1 (T1IN1), GPIO2 (T1IN2)
// T2: GPIO3 (T2IN1), GPIO6 (T2IN2)
// T3: GPIO7 (T3IN1), GPIO10 (T3IN2)
// T4: GPIO11 (T4IN1), GPIO12 (T4IN2)
#define TOGGLE_SWITCH1_IN1_PIN 1   // T1IN1
#define TOGGLE_SWITCH1_IN2_PIN 2   // T1IN2
#define TOGGLE_SWITCH2_IN1_PIN 3   // T2IN1
#define TOGGLE_SWITCH2_IN2_PIN 6   // T2IN2
#define TOGGLE_SWITCH3_IN1_PIN 7   // T3IN1
#define TOGGLE_SWITCH3_IN2_PIN 10  // T3IN2
#define TOGGLE_SWITCH4_IN1_PIN 11  // T4IN1
#define TOGGLE_SWITCH4_IN2_PIN 12  // T4IN2

// Menu navigation buttons (active high)
#define ENTER_BUTTON_PIN 13  // CN3 [push button 1]
#define BACK_BUTTON_PIN 14   // CN4 [push button 2]

// SPI Display pins (reserved for future use if needed)
#define SPI_SCK_PIN 18
#define SPI_TX_PIN 19
#define SPI_CSN_PIN 21

// Power button (GPIO 20)
#define POWER_BUTTON_PIN 19

// Buzzer (for boot jingle)
#define BUZZER_PIN 24

// Battery voltage thresholds
#define TX_BATT_EMPTY_V         3.2f
#define TX_BATT_FULL_V          4.2f

// EEPROM addresses
#define EEPROM_SIZE 512
#define CAL_MAGIC_ADDR 150
#define CAL_DATA_ADDR 152
#define CAL_MAGIC_VALUE 0xC5

// Timing
static unsigned long lastChannelSend = 0;
static const unsigned long CHANNEL_INTERVAL = 4;  // 250Hz (higher than 200Hz for safety margin)
#if defined(DEBUG_LINK_STATS)
static uint32_t s_channelSendsThisSec = 0;
#if defined(CRSF_CORE1_CHANNELS)
static volatile uint32_t s_core1_channelSendsThisSec = 0;
#endif
static unsigned long s_lastDiagPrintMs = 0;
static unsigned long s_loopStartMs = 0;
static unsigned long s_maxLoopMs = 0;
#endif
static unsigned long lastDisplayUpdate = 0;
static const unsigned long DISPLAY_INTERVAL = 200; // ms
static unsigned long lastPingSent = 0;
static const unsigned long PING_INTERVAL = 1000;  // 1Hz ping for device check
static unsigned long lastPongReceived = 0;
static const unsigned long PONG_TIMEOUT_MS = 2000;  // TX considered disconnected if no pong for 2s

// Application state
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#if !defined(INTERNAL_ADC)
Adafruit_ADS1115 ads;  // ADS1115 ADC on I2C bus
#endif
UARTProtocol uartProto(&Serial2);
CRSFSerialConnector crsfSerial(Serial2, CRSFSerialConnector::CRSF_BAUDRATE);

// TX type: detected at boot or overridden in menu
enum TxType : uint8_t { TX_TYPE_UNKNOWN = 0, TX_TYPE_CUSTOM = 1, TX_TYPE_ELRS = 2 };
enum TxModeOverride : uint8_t { TX_MODE_AUTO = 0, TX_MODE_CUSTOM = 1, TX_MODE_ELRS = 2 };
static TxType tx_type = TX_TYPE_UNKNOWN;
static TxModeOverride tx_mode_override = TX_MODE_AUTO;
static char elrs_device_name[24] = {0};
static bool detecting_tx = false;
static bool elrs_device_info_received = false;
static unsigned long lastElrsFrameReceived = 0;
// ELRS role: 0=unknown, 1=TX (correct), 2=RX (wrong mode - do not use for channels)
enum ElrsRole : uint8_t { ELRS_ROLE_UNKNOWN = 0, ELRS_ROLE_TX = 1, ELRS_ROLE_RX = 2 };
static ElrsRole elrs_role = ELRS_ROLE_UNKNOWN;
static const unsigned long ELRS_FRAME_TIMEOUT_MS = 2000;

// ELRS CRSF parameter discovery (binding phrase + bind command)
#define ELRS_PARAM_FIELD_NONE 0xFF
#define ELRS_PHRASE_MAX 32
static uint8_t elrs_phrase_field_id = ELRS_PARAM_FIELD_NONE;
static uint8_t elrs_bind_cmd_field_id = ELRS_PARAM_FIELD_NONE;
static uint8_t elrs_wifi_cmd_field_id = ELRS_PARAM_FIELD_NONE;
static char elrs_binding_phrase[ELRS_PHRASE_MAX + 1] = {0};
static uint8_t elrs_param_chunk_next = 0;
static uint8_t elrs_param_chunk_index = 0;
static bool elrs_param_read_in_progress = false;
static const unsigned int ELRS_PARAM_MAX_CHUNKS = 48;
static unsigned long elrs_last_param_entry_ms = 0;  // timestamp of last received parameter entry (for retry)

// Telemetry data from TX
static TelemetryData telemetry = {0};
static StatusData status = {0};
static bool telemetryValid = false;
static bool statusValid = false;
static unsigned long lastTelemetryUpdate = 0;
static const unsigned long TELEMETRY_TIMEOUT_MS = 2000;

// Button state for command handling
static bool enterButtonPressed = false;
static bool backButtonPressed = false;
static bool powerButtonPressed = false;
static bool enterButtonIdleLevel = HIGH;
static bool backButtonIdleLevel = HIGH;
static unsigned long lastEnterPress = 0;
static unsigned long lastBackPress = 0;
static unsigned long lastPowerPress = 0;
static const unsigned long BUTTON_DEBOUNCE_MS = 200;
static const unsigned long LONG_PRESS_MS = 600;
// Menu nav uses calibrated channel values (1000-2000, center=1500).
// Deadzone: stick must leave the center band (1500 ± NAV_DEAD) to unlock.
// Trigger: stick must be past 1500 ± NAV_TRIG for one step to fire.
static const int16_t MENU_NAV_CENTER_DEADZONE = 80;   // ±80us around 1500
static const int16_t MENU_NAV_TRIGGER_THRESHOLD = 200; // ±200us from 1500
static const unsigned long MENU_NAV_DEBOUNCE_MS = 120;
static unsigned long lastMenuDebugMs = 0;
static uint8_t menuDebugCount = 0;
static unsigned long lastMenuNavMs = 0;
static bool menuNavLatched = false;

// Menu state: 0 = main screen, 1 = top-level menu, 2+ = submenu
enum MenuScreen : uint8_t { MENU_MAIN = 0, MENU_TOP = 1, MENU_TX_MODE = 2, MENU_BINDING = 3 };
static MenuScreen menu_screen = MENU_MAIN;
static uint8_t menu_cursor = 0;  // index in current menu
static const uint8_t MENU_BINDING_ITEMS = 5; // View phrase, Set phrase, Initiate pairing, Start WiFi, Back
static unsigned long back_press_start = 0;
static bool back_long_press_handled = false;

// Device status flags
bool ads_ok = false;
bool display_ok = false;
bool tx_connected = false;  // TX board connection status

// Channel values (mapped to CRSF range 1000-2000)
uint16_t channels[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
#if defined(CRSF_CORE1_CHANNELS)
bool core1_separate_stack = true;  // 8KB stack for Core 1
static rc_config_data_t s_core1_config;  // Core 1's working snapshot; read exclusively by Core 1
// F1/F3: Core 0 must not write s_core1_config directly while Core 1 reads it field-by-field
// (torn read). Instead Core 0 stages into s_core1_config_staging and raises s_core1_config_dirty;
// Core 1 consumes it (copies into s_core1_config + resets its own filters) at a frame boundary.
static rc_config_data_t s_core1_config_staging;  // Core 0 writes; handed off via dirty flag
static volatile bool s_core1_config_dirty = false;  // Core 0 sets, Core 1 clears
static volatile uint16_t s_core1_channels[8];  // Core 1 writes; Core 0 reads for display
static volatile bool s_core1_send_elrs_channels = false;  // Set in setup() after TX detection
static volatile bool s_multicore_ready = false;  // Set by setup() after init
#if defined(INTERNAL_ADC)
// Latest filtered stick ADC (same pipeline as CRSF); Core 0 uses for raw_adc / calibration UI
static volatile int16_t s_core1_stick_filtered[4];
#endif
#endif

// Raw input snapshots (for logging/calibration)
int16_t raw_adc[4] = {0};
bool raw_toggles[4] = {false};

// Calibration data for sticks (legacy; kept in sync with g_rc_config until readInputs uses config)
typedef struct {
    uint16_t min[4];
    uint16_t max[4];
    uint16_t center[4];
} calib_data_t;

static calib_data_t calib = {
    {RC_CALIB_DEFAULT_MIN, RC_CALIB_DEFAULT_MIN, RC_CALIB_DEFAULT_MIN, RC_CALIB_DEFAULT_MIN},
    {RC_CALIB_DEFAULT_MAX, RC_CALIB_DEFAULT_MAX, RC_CALIB_DEFAULT_MAX, RC_CALIB_DEFAULT_MAX},
    {RC_CALIB_DEFAULT_CENTER, RC_CALIB_DEFAULT_CENTER, RC_CALIB_DEFAULT_CENTER, RC_CALIB_DEFAULT_CENTER}
};
static bool calib_loaded = false;

// Runtime config (USB WebUI): loaded from EEPROM or defaults; used by readInputs when wired
static rc_config_data_t g_rc_config;
static rc_config_data_t g_rc_config_draft;
static RCConfigProtocol g_rc_proto;
static unsigned long lastStateStreamMs = 0;
static volatile bool proxy_mode_enabled = false;
static const unsigned long STATE_STREAM_INTERVAL_MS = 50;

// Calibration via WebUI: sampling state
static bool cal_sampling = false;
static int16_t cal_axis_min[4];
static int16_t cal_axis_max[4];

// High-pass filter state (1st-order, per-axis) for ADC drift reduction
static float hpf_prev_in[4] = {0};
static float hpf_prev_out[4] = {0};
static bool hpf_initialized = false;
#if defined(INTERNAL_ADC) && defined(RC_STICK_ADC_12BIT)
#define RC_ADC_CENTER 2048
#define RC_STICK_VALUE_MAX 4095
#else
#define RC_ADC_CENTER 16384
#define RC_STICK_VALUE_MAX 32767
#endif
#define HPF_TAU_S     0.05f
#define HPF_DT_S      0.02f
#define HPF_ALPHA     (HPF_TAU_S / (HPF_TAU_S + HPF_DT_S))

// TX local battery (BQ2562X)
static float tx_batt_voltage = 0.0f;
static uint8_t tx_batt_percent = 0;
static uint8_t tx_batt_bars = 0;
static bool tx_batt_low = false;
static bool tx_batt_valid = false;
static unsigned long last_tx_batt_read = 0;
static const unsigned long TX_BATT_READ_INTERVAL = 5000; // ms
static uint16_t tx_batt_raw_last = 0;
// Charging status
static enum bq2562x_charge_status tx_charge_status = BQ2562X_CHG_STAT_NOT_CHARGING;
static enum bq2562x_vbus_status tx_vbus_status = BQ2562X_VBUS_STAT_NO_INPUT;
static bool tx_charge_status_valid = false;

// Forward declarations
static void load_calibration(void);
#if !defined(INTERNAL_ADC)
static bool init_ads1115(void);
#else
static bool init_internal_adc(void);
#endif
static void readInputs();
static void updateDisplay();
static void sendChannels();
static void receiveUARTMessages();
static void handleButtons();
static uint8_t voltage_to_percent(float v);
static uint8_t percent_to_bars(uint8_t pct);
static void update_tx_battery(void);
static const char* get_charge_status_string(enum bq2562x_charge_status status);
static const char* get_vbus_status_string(enum bq2562x_vbus_status status);
static void play_boot_jingle(void);
static void scan_i2c_bus(void);
static bool init_display(void);
static void drawBatteryBar(int x, int y, uint8_t percent, bool low_blink);
static bool is_menu_button_pressed(uint8_t pin, bool idleLevel);
static void drawLinkIndicator(int x, int y, bool connected);
static void resync_display_viewport(void);

// UART Protocol callbacks
static void onTelemetryReceived(const TelemetryData* data);
static void onStatusReceived(const StatusData* data);
static void onAckReceived(UARTMsgType ackedCmd);
static void onErrorReceived(uint8_t errorCode);
static void onPongReceived();

// Device check functions
static bool check_tx_connection(void);
static void ping_tx_device(void);
static void run_tx_detection(void);
static void send_crsf_device_info(void);
static void onCrsfExtendedFrame(uint8_t sourceAddr, uint8_t type);
static void onCrsfDeviceInfo(const uint8_t *serial4, const char *name, uint8_t sourceAddr);
static void onCrsfParameterEntry(uint8_t fieldId, uint8_t paramType, uint8_t chunksRemaining, const char *label, const uint8_t *value, uint8_t valueLen);
static TxType get_effective_tx_type(void);
static void elrs_request_param_chunk(uint8_t fieldId, uint8_t chunkIndex);
static bool elrs_discover_params(bool needPhrase, bool needBind, unsigned long timeoutMs);
static bool elrs_write_param_string(uint8_t fieldId, const char *str);
static bool elrs_write_param_byte(uint8_t fieldId, uint8_t val);

// USB config protocol callbacks
static const rc_config_data_t* rc_proto_get_config(void);
static void rc_proto_set_draft(const rc_config_data_t* draft);
static void rc_proto_apply(void);
static bool rc_proto_save(void);
static void rc_proto_cal_start(void);
static void rc_proto_cal_sample(void);
static void rc_proto_cal_finish(void);
static void rc_proto_get_state(int16_t* adc4, uint16_t* ch8, uint8_t* toggles4);
static bool rc_proto_set_binding_phrase_rx(const char* phrase, uint8_t len);
static bool rc_proto_set_binding_phrase_tx(const char* phrase, uint8_t len);
static void rc_proto_get_link_status(uint8_t* txConnected, uint8_t* txPaired, uint8_t* txState);
static void rc_proto_get_link_status_ex(uint8_t* buf, uint8_t bufLen);
static void rc_proto_re_detect_tx(void);
static bool rc_proto_get_elrs_binding_phrase(char* phrase, uint8_t maxLen);
static bool rc_proto_enter_pairing_mode(void);
static bool rc_proto_enter_wifi_mode(void);
static void rc_proto_enter_usb_uart_proxy(void);
static void rc_proto_exit_usb_uart_proxy(void);

// ============================================================
// Setup
// ============================================================
void setup() {
    Serial.begin(115200);
    unsigned long start = millis();
    while (!Serial && (millis() - start < 2000)) {
        delay(10);
    }
    delay(200);

#if defined(CRSF_CORE1_CHANNELS)
    crsfSerial.initTxMutex();
#endif

    Serial.println("=== RC-CRSF Controller ===");
    Serial.println("Display, battery, analog inputs -> CRSF serial output");

    // Initialize I2C for OLED and ADS1115
    Wire.setSDA(I2C_SDA);
    Wire.setSCL(I2C_SCL);
    Wire.begin();
    
    // Start with standard I2C speed (100kHz) for compatibility
    Wire.setClock(100000);  // 100kHz standard mode
    Serial.println("I2C bus initialized at 100kHz");
    Serial.print("I2C pins: SDA=GP");
    Serial.print(I2C_SDA);
    Serial.print(", SCL=GP");
    Serial.println(I2C_SCL);

    // Scan I2C bus to see what devices are present
    scan_i2c_bus();

    // Initialize display with error checking
    display_ok = init_display();
    if (!display_ok) {
        Serial.println("ERROR: Display initialization failed!");
        Serial.println("Check wiring: SDA=GP4, SCL=GP5, VCC=3.3V, GND=GND");
    }

    // Initialize ADC (internal RP2350 or I2C ADS1115)
#if defined(INTERNAL_ADC)
    ads_ok = init_internal_adc();
#else
    ads_ok = init_ads1115();
#endif

    // Initialize BQ2562X driver
    if (bq2562x_init() == 0) {
        Serial.println("BQ2562X driver initialized");
    } else {
        Serial.println("BQ2562X driver init failed");
    }

    // Initialize buttons and toggles
    pinMode(ENTER_BUTTON_PIN, INPUT_PULLUP);
    pinMode(BACK_BUTTON_PIN, INPUT_PULLUP);
    pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);  // Active LOW button, defaults HIGH
    delay(2);
    enterButtonIdleLevel = digitalRead(ENTER_BUTTON_PIN);
    backButtonIdleLevel = digitalRead(BACK_BUTTON_PIN);
    Serial.print("[RC] PUSH1 idle level: ");
    Serial.println(enterButtonIdleLevel == HIGH ? "HIGH (pressed=LOW)" : "LOW (pressed=HIGH)");
    Serial.print("[RC] PUSH2 idle level: ");
    Serial.println(backButtonIdleLevel == HIGH ? "HIGH (pressed=LOW)" : "LOW (pressed=HIGH)");
    Serial.print("[RC] Power button initialized on GPIO ");
    Serial.print(POWER_BUTTON_PIN);
    Serial.print(" (INPUT_PULLUP, active LOW)");
    Serial.print(" - Current state: ");
    Serial.println(digitalRead(POWER_BUTTON_PIN) == HIGH ? "HIGH (not pressed)" : "LOW (pressed)");
    // Toggle switches - both IN1 and IN2 pins configured as INPUT_PULLUP
    pinMode(TOGGLE_SWITCH1_IN1_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SWITCH1_IN2_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SWITCH2_IN1_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SWITCH2_IN2_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SWITCH3_IN1_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SWITCH3_IN2_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SWITCH4_IN1_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SWITCH4_IN2_PIN, INPUT_PULLUP);

    // Initialize buzzer
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // Load stick calibration (if present)
    load_calibration();

    // Load runtime config for WebUI; migrate legacy calib if no config yet
    if (!rc_config_load(&g_rc_config)) {
        rc_config_set_defaults(&g_rc_config);
        if (calib_loaded) {
            for (int i = 0; i < 4; i++) {
                g_rc_config.calib_min[i] = calib.min[i];
                g_rc_config.calib_max[i] = calib.max[i];
                g_rc_config.calib_center[i] = calib.center[i];
            }
            rc_config_save(&g_rc_config);
        }
    }
    memcpy(&g_rc_config_draft, &g_rc_config, sizeof(g_rc_config));

    g_rc_proto.setStream(&Serial);
    g_rc_proto.setDeviceInfo("RC-CRSF", "1.3");
    g_rc_proto.setConfigGetter(rc_proto_get_config);
    g_rc_proto.setConfigDraftSetter(rc_proto_set_draft);
    g_rc_proto.setApplyCallback(rc_proto_apply);
    g_rc_proto.setSaveCallback(rc_proto_save);
    g_rc_proto.setCalibrationStart(rc_proto_cal_start);
    g_rc_proto.setCalibrationSample(rc_proto_cal_sample);
    g_rc_proto.setCalibrationFinish(rc_proto_cal_finish);
    g_rc_proto.setStateGetter(rc_proto_get_state);
    g_rc_proto.setBindingPhraseRx(rc_proto_set_binding_phrase_rx);
    g_rc_proto.setBindingPhraseTx(rc_proto_set_binding_phrase_tx);
    g_rc_proto.setLinkStatusGetter(rc_proto_get_link_status);
    g_rc_proto.setLinkStatusGetterEx(rc_proto_get_link_status_ex);
    g_rc_proto.setReDetectTx(rc_proto_re_detect_tx);
    g_rc_proto.setGetElrsBindingPhrase(rc_proto_get_elrs_binding_phrase);
    g_rc_proto.setEnterPairingMode(rc_proto_enter_pairing_mode);
    g_rc_proto.setEnterUsbUartProxy(rc_proto_enter_usb_uart_proxy);

    // Initialize UART protocol (outputting to TX side)
    Serial.println("Initializing UART protocol...");
    Serial2.setTX(CRSF_TX_PIN);
    Serial2.setRX(CRSF_RX_PIN);
    uartProto.begin(UART_PROTOCOL_BAUDRATE);
    uartProto.setOnTelemetry(onTelemetryReceived);
    uartProto.setOnStatus(onStatusReceived);
    uartProto.setOnAck(onAckReceived);
    uartProto.setOnError(onErrorReceived);
    uartProto.setOnPong(onPongReceived);
    crsfSerial.onExtendedFrame = onCrsfExtendedFrame;
    crsfSerial.onParameterEntry = onCrsfParameterEntry;
    Serial.print("UART Protocol on Serial2: TX=GP");
    Serial.print(CRSF_TX_PIN);
    Serial.print(", RX=GP");
    Serial.println(CRSF_RX_PIN);
    
    // TX type detection at boot (Custom PING/PONG vs ELRS DEVICE_PING/DEVICE_INFO)
    delay(100);  // UART stabilize
    run_tx_detection();
    lastPingSent = millis();
    check_tx_connection();

#if defined(CRSF_CORE1_CHANNELS)
    memcpy(&s_core1_config, &g_rc_config, sizeof(s_core1_config));
    s_core1_send_elrs_channels = (tx_type == TX_TYPE_ELRS && elrs_role == ELRS_ROLE_TX);
#if defined(INTERNAL_ADC)
    for (int i = 0; i < 4; i++) {
        s_core1_stick_filtered[i] = RC_ADC_CENTER;
    }
#endif
    s_multicore_ready = true;
#endif

    if (display_ok) {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("RC-CRSF ready");
        display.display();
    }

    // Play boot jingle
    play_boot_jingle();
    
    // Keep I2C at 100kHz for display stability.
    // Some OLED modules show page-wrap artifacts at 400kHz on longer/noisy wires.
    Wire.setClock(100000);
    Serial.println("I2C bus speed kept at 100kHz for stable OLED updates");
}

// ============================================================
// Loop
// ============================================================
void loop() {
#if defined(DEBUG_LINK_STATS)
    s_loopStartMs = millis();
#endif
    if (proxy_mode_enabled) {
        // F2: exit the bridge when the USB host closes the port (CDC DTR deasserted), so the RC
        // resumes normal operation without a power cycle. There is no in-band escape because the
        // protocol poll is bypassed while bridging. NOTE: relies on the USB-CDC operator bool()
        // tracking DTR on this core — verify on hardware. If it never deasserts, behaviour is
        // unchanged from before (reboot to exit); a spurious deassert just drops the bridge.
        if (!Serial) {
            rc_proto_exit_usb_uart_proxy();
            return;
        }
        // Raw USB <-> UART passthrough: browser talks directly to TX module (ELRS/EdgeTX)
        while (Serial.available()) {
            Serial2.write(Serial.read());
        }
        while (Serial2.available()) {
            Serial.write(Serial2.read());
        }
        yield();
        return;
    }

    readInputs();
    sendChannels();
    receiveUARTMessages();
    g_rc_proto.poll();
    if (g_rc_proto.isStreamingState() && (millis() - lastStateStreamMs >= STATE_STREAM_INTERVAL_MS)) {
        lastStateStreamMs = millis();
        uint8_t toggles4[4];
        for (int i = 0; i < 4; i++) toggles4[i] = raw_toggles[i] ? 1 : 0;
        g_rc_proto.sendStateFrame(raw_adc, channels, toggles4);
    }
    ping_tx_device();  // Periodic device check
    check_tx_connection();  // Update connection status
    handleButtons();
    update_tx_battery();
    updateDisplay();
#if defined(DEBUG_LINK_STATS)
    {
        unsigned long loopMs = millis() - s_loopStartMs;
        if (loopMs > s_maxLoopMs) s_maxLoopMs = loopMs;
        if (millis() - s_lastDiagPrintMs >= 1000) {
            s_lastDiagPrintMs = millis();
            uint32_t linkStats = crsfSerial.getLinkStatsPacketCount();
            crsfSerial.resetLinkStatsPacketCount();
#if defined(CRSF_CORE1_CHANNELS)
            uint32_t chCount = s_core1_channelSendsThisSec;
            s_core1_channelSendsThisSec = 0;
#else
            uint32_t chCount = s_channelSendsThisSec;
            s_channelSendsThisSec = 0;
#endif
            Serial.printf("[LINK_DIAG] ch/s=%lu telem/s=%lu loopMaxMs=%lu link=%s\n",
                (unsigned long)chCount,
                (unsigned long)linkStats,
                (unsigned long)s_maxLoopMs,
                crsfSerial.isLinkUp() ? "UP" : "DOWN");
#if !defined(CRSF_CORE1_CHANNELS)
            s_channelSendsThisSec = 0;
#endif
            s_maxLoopMs = 0;
        }
    }
#endif
}

#if defined(CRSF_CORE1_CHANNELS) && defined(INTERNAL_ADC)
// Core 1: ADC + switches -> channels -> Serial2. Fully independent; no Core 0 dependency.
void setup1() {
    while (!s_multicore_ready) {
        delay(1);
    }
}

void loop1() {
    // F2: while Core 0 is bridging USB<->Serial2, Core 1 must not write Serial2 (interleaved
    // bytes corrupt the ELRS/EdgeTX passthrough). Skip the whole frame, including the UART send.
    if (proxy_mode_enabled) return;

    static unsigned long lastSend = 0;
    if (millis() - lastSend < 4) return;  // 250 Hz
    lastSend = millis();

    // F1/F3: pick up a newly-staged config at this safe point between frames. Copying
    // the whole struct here (rather than letting Core 0 write s_core1_config live) makes
    // the snapshot atomic from Core 1's view and lets Core 1 reset its own stick filter.
    if (s_core1_config_dirty) {
        memcpy(&s_core1_config, &s_core1_config_staging, sizeof(s_core1_config));
        __dmb();
        s_core1_config_dirty = false;
        internal_adc_reset_stick_filter();  // filter masks are owned by this core
    }

    const rc_config_data_t* cfg = &s_core1_config;
    int16_t axis_adc[4] = {RC_ADC_CENTER, RC_ADC_CENTER, RC_ADC_CENTER, RC_ADC_CENTER};

    internal_read_sticks_filtered(axis_adc, cfg);
    for (int i = 0; i < 4; i++) {
        s_core1_stick_filtered[i] = axis_adc[i];
    }

    auto applyDeadzone = [](int16_t val, uint16_t vmin, uint16_t vmax, float frac) -> int16_t {
        if (vmax <= vmin + 10 || frac <= 0.0f) return val;
        float center = (vmin + vmax) * 0.5f;
        float halfRange = (vmax - vmin) * 0.5f;
        float dz = halfRange * frac;
        float delta = val - center;
        if (fabsf(delta) < dz) return (int16_t)center;
        float sign = (delta > 0) ? 1.0f : -1.0f;
        float adjusted = sign * ((fabsf(delta) - dz) / (halfRange - dz)) * halfRange;
        return (int16_t)(center + adjusted);
    };
    auto mapToUs = [](int16_t val, uint16_t vmin, uint16_t vmax, bool inv) -> uint16_t {
        if (vmax <= vmin + 10) return RC_CHANNEL_MID;
        int low = inv ? 2000 : 1000;
        int high = inv ? 1000 : 2000;
        return (uint16_t)constrain(map(val, vmin, vmax, low, high), RC_CHANNEL_MIN, RC_CHANNEL_MAX);
    };
    uint16_t axis_us[4];
    for (int i = 0; i < 4; i++) {
        int16_t dz = applyDeadzone(axis_adc[i], cfg->calib_min[i], cfg->calib_max[i], cfg->deadzone[i]);
        axis_us[i] = mapToUs(dz, cfg->calib_min[i], cfg->calib_max[i], cfg->invert[i] != 0);
    }

    uint16_t localCh[8];
    for (int c = 0; c < 4; c++) localCh[c] = RC_CHANNEL_MID;
    for (int a = 0; a < 4; a++) {
        uint8_t fc = cfg->channel_function[a];
        if (fc < 4) localCh[fc] = axis_us[a];
    }

    auto readToggle = [](uint8_t in1_pin, uint8_t in2_pin) -> uint16_t {
        bool in1_high = digitalRead(in1_pin) == HIGH;
        bool in2_high = digitalRead(in2_pin) == HIGH;
        if (in1_high && !in2_high) return 1000;
        if (!in1_high && in2_high) return 2000;
        return 1500;
    };
    localCh[4] = readToggle(TOGGLE_SWITCH1_IN1_PIN, TOGGLE_SWITCH1_IN2_PIN);
    localCh[5] = readToggle(TOGGLE_SWITCH2_IN1_PIN, TOGGLE_SWITCH2_IN2_PIN);
    localCh[6] = readToggle(TOGGLE_SWITCH3_IN1_PIN, TOGGLE_SWITCH3_IN2_PIN);
    localCh[7] = readToggle(TOGGLE_SWITCH4_IN1_PIN, TOGGLE_SWITCH4_IN2_PIN);
    for (int c = 0; c < 8; c++) {
        int32_t v = (int32_t)localCh[c] + (int32_t)cfg->channel_trim[c];
        localCh[c] = (uint16_t)constrain(v, (int32_t)cfg->cutoff_min[c], (int32_t)cfg->cutoff_max[c]);
    }

    for (int i = 0; i < 8; i++) s_core1_channels[i] = localCh[i];

    crsfSerial.core1_drainAndSendChannels(localCh, s_core1_send_elrs_channels);
#if defined(DEBUG_LINK_STATS)
    if (s_core1_send_elrs_channels) s_core1_channelSendsThisSec++;
#endif
}
#endif

// ============================================================
// Input handling (runtime config: g_rc_config)
// ============================================================
void readInputs() {
#if defined(CRSF_CORE1_CHANNELS) && defined(INTERNAL_ADC)
    // Core 1 owns ADC+switches->channels. Core 0 mirrors filtered sticks (no second ADC pipeline).
    raw_adc[0] = s_core1_stick_filtered[0];
    raw_adc[1] = s_core1_stick_filtered[1];
    raw_adc[2] = s_core1_stick_filtered[2];
    raw_adc[3] = s_core1_stick_filtered[3];
    for (int i = 0; i < 8; i++) channels[i] = s_core1_channels[i];
    raw_toggles[0] = digitalRead(TOGGLE_SWITCH1_IN1_PIN) == HIGH;
    raw_toggles[1] = digitalRead(TOGGLE_SWITCH2_IN1_PIN) == HIGH;
    raw_toggles[2] = digitalRead(TOGGLE_SWITCH3_IN1_PIN) == HIGH;
    raw_toggles[3] = digitalRead(TOGGLE_SWITCH4_IN1_PIN) == HIGH;
    return;
#endif

    const rc_config_data_t* cfg = &g_rc_config;
    int16_t axis_adc[4] = {RC_ADC_CENTER, RC_ADC_CENTER, RC_ADC_CENTER, RC_ADC_CENTER};

    if (ads_ok) {
#if defined(INTERNAL_ADC)
        internal_read_sticks_filtered(axis_adc, &g_rc_config);
#else
        axis_adc[0] = ads.readADC_SingleEnded(AILERON_CHANNEL);
        axis_adc[1] = ads.readADC_SingleEnded(ELEVATOR_CHANNEL);
        axis_adc[2] = ads.readADC_SingleEnded(RUDDER_CHANNEL);
        axis_adc[3] = ads.readADC_SingleEnded(THROTTLE_CHANNEL);
#endif
    }

    // Optional high-pass on sticks (slow drift about center). Separate from cfg stick_low_pass (Core1 ADC path).
    if (cfg->high_pass_filter) {
        if (!hpf_initialized) {
            for (int i = 0; i < 4; i++) {
                float c = (float)(axis_adc[i] - RC_ADC_CENTER);
                hpf_prev_in[i] = c;
                hpf_prev_out[i] = c;
            }
            hpf_initialized = true;
        }
        for (int i = 0; i < 4; i++) {
            float centered = (float)(axis_adc[i] - RC_ADC_CENTER);
            float out = HPF_ALPHA * (hpf_prev_out[i] + centered - hpf_prev_in[i]);
            hpf_prev_in[i] = centered;
            hpf_prev_out[i] = out;
            int32_t v = RC_ADC_CENTER + (int32_t)out;
            axis_adc[i] = (int16_t)constrain(v, 0, RC_STICK_VALUE_MAX);
        }
    } else {
        hpf_initialized = false;
    }

    for (int i = 0; i < 4; i++) raw_adc[i] = axis_adc[i];

    auto applyDeadzone = [](int16_t val, uint16_t vmin, uint16_t vmax, float frac) -> int16_t {
        if (vmax <= vmin + 10 || frac <= 0.0f) return val;
        float center = (vmin + vmax) * 0.5f;
        float halfRange = (vmax - vmin) * 0.5f;
        float dz = halfRange * frac;
        float delta = val - center;
        if (fabsf(delta) < dz) return (int16_t)center;
        float sign = (delta > 0) ? 1.0f : -1.0f;
        float adjusted = sign * ((fabsf(delta) - dz) / (halfRange - dz)) * halfRange;
        return (int16_t)(center + adjusted);
    };
    auto mapToUs = [](int16_t val, uint16_t vmin, uint16_t vmax, bool inv) -> uint16_t {
        if (vmax <= vmin + 10) return RC_CHANNEL_MID;
        int low = inv ? 2000 : 1000;
        int high = inv ? 1000 : 2000;
        return (uint16_t)constrain(map(val, vmin, vmax, low, high), RC_CHANNEL_MIN, RC_CHANNEL_MAX);
    };
    uint16_t axis_us[4];
    for (int i = 0; i < 4; i++) {
        int16_t dz = applyDeadzone(axis_adc[i], cfg->calib_min[i], cfg->calib_max[i], cfg->deadzone[i]);
        axis_us[i] = mapToUs(dz, cfg->calib_min[i], cfg->calib_max[i], cfg->invert[i] != 0);
    }

    uint16_t newCh[8];
    for (int c = 0; c < 4; c++) newCh[c] = RC_CHANNEL_MID;
    for (int a = 0; a < 4; a++) {
        uint8_t fc = cfg->channel_function[a];
        if (fc < 4) newCh[fc] = axis_us[a];
    }

    auto readToggle = [](uint8_t in1_pin, uint8_t in2_pin) -> uint16_t {
        bool in1_high = digitalRead(in1_pin) == HIGH;
        bool in2_high = digitalRead(in2_pin) == HIGH;
        if (in1_high && !in2_high) return 1000;
        if (!in1_high && in2_high) return 2000;
        return 1500;
    };
    newCh[4] = readToggle(TOGGLE_SWITCH1_IN1_PIN, TOGGLE_SWITCH1_IN2_PIN);
    newCh[5] = readToggle(TOGGLE_SWITCH2_IN1_PIN, TOGGLE_SWITCH2_IN2_PIN);
    newCh[6] = readToggle(TOGGLE_SWITCH3_IN1_PIN, TOGGLE_SWITCH3_IN2_PIN);
    newCh[7] = readToggle(TOGGLE_SWITCH4_IN1_PIN, TOGGLE_SWITCH4_IN2_PIN);
    for (int c = 0; c < 8; c++) {
        int32_t v = (int32_t)newCh[c] + (int32_t)cfg->channel_trim[c];
        newCh[c] = (uint16_t)constrain(v, (int32_t)cfg->cutoff_min[c], (int32_t)cfg->cutoff_max[c]);
    }

    memcpy(channels, newCh, sizeof(channels));

    raw_toggles[0] = digitalRead(TOGGLE_SWITCH1_IN1_PIN) == HIGH;
    raw_toggles[1] = digitalRead(TOGGLE_SWITCH2_IN1_PIN) == HIGH;
    raw_toggles[2] = digitalRead(TOGGLE_SWITCH3_IN1_PIN) == HIGH;
    raw_toggles[3] = digitalRead(TOGGLE_SWITCH4_IN1_PIN) == HIGH;
}

// ============================================================
// UART Protocol - Channel Output
// ============================================================
void sendChannels() {
#if defined(CRSF_CORE1_CHANNELS)
    // ELRS channel sending is done on Core 1 at 250 Hz
    if (get_effective_tx_type() == TX_TYPE_ELRS && elrs_role == ELRS_ROLE_TX)
        return;
#endif
    if (millis() - lastChannelSend < CHANNEL_INTERVAL) return;
    lastChannelSend = millis();

    TxType eff = get_effective_tx_type();
    if (eff == TX_TYPE_ELRS && elrs_role == ELRS_ROLE_TX) {
        // Unreachable when CRSF_CORE1_CHANNELS (handled above)
        return;
    } else {
        uartProto.sendChannels(channels);
#if defined(DEBUG_LINK_STATS)
        s_channelSendsThisSec++;
#endif
    }
}

// ============================================================
// UART Protocol - Message Reception
// ============================================================
void receiveUARTMessages() {
    TxType eff = get_effective_tx_type();
    if (eff == TX_TYPE_ELRS) {
        crsfSerial.loop();
        // Optionally drive telemetry from CRSF link/battery when ELRS
        if (crsfSerial.isLinkUp()) {
            telemetryValid = true;
            lastTelemetryUpdate = millis();
            telemetry.rssi = crsfSerial.getLinkStatistics()->uplink_RSSI_1;
            telemetry.linkQuality = crsfSerial.getLinkStatistics()->uplink_Link_quality;
            telemetry.rxBattMv = (uint16_t)(crsfSerial.getBatteryVoltage() * 1000);
            telemetry.rxBattPct = crsfSerial.getBatteryRemaining();
        }
    } else {
        uartProto.loop();
    }
    if (telemetryValid && (millis() - lastTelemetryUpdate) > TELEMETRY_TIMEOUT_MS)
        telemetryValid = false;
}

// ============================================================
// UART Protocol - Callbacks
// ============================================================
void onTelemetryReceived(const TelemetryData* data) {
    if (data) {
        memcpy(&telemetry, data, sizeof(TelemetryData));
        telemetryValid = true;
        lastTelemetryUpdate = millis();
    }
}

void onStatusReceived(const StatusData* data) {
    if (data) {
        memcpy(&status, data, sizeof(StatusData));
        statusValid = true;
    }
}

void onAckReceived(UARTMsgType ackedCmd) {
    Serial.print("[UART] ACK received for command: 0x");
    Serial.println((uint8_t)ackedCmd, HEX);
}

void onErrorReceived(uint8_t errorCode) {
    Serial.print("[UART] Error received: 0x");
    Serial.println(errorCode, HEX);
}

void onPongReceived() {
    lastPongReceived = millis();
    if (!tx_connected) {
        tx_connected = true;
        Serial.println("[TX] Device connected!");
    }
}

// ============================================================
// Button Handling for Commands and Menu
// ============================================================
void handleButtons() {
    bool enterPressed = is_menu_button_pressed(ENTER_BUTTON_PIN, enterButtonIdleLevel);
    bool backPressed = is_menu_button_pressed(BACK_BUTTON_PIN, backButtonIdleLevel);
    bool powerPressed = digitalRead(POWER_BUTTON_PIN) == LOW;  // Active LOW button
    auto currentMenuItems = []() -> uint8_t {
        if (menu_screen == MENU_TOP)
            return (get_effective_tx_type() == TX_TYPE_ELRS && elrs_role == ELRS_ROLE_TX) ? 4 : 3;
        if (menu_screen == MENU_TX_MODE)
            return 4;
        if (menu_screen == MENU_BINDING)
            return MENU_BINDING_ITEMS;
        return 0;
    };
    auto moveMenuCursor = [&](int8_t dir) {
        uint8_t itemCount = currentMenuItems();
        if (itemCount <= 1)
            return;
        if (dir > 0)
            menu_cursor = (menu_cursor + 1) % itemCount;
        else
            menu_cursor = (menu_cursor + itemCount - 1) % itemCount;
    };

    // Use calibrated channel values (1000-2000, center=1500) so that the
    // physical rest position of each stick is always treated as neutral,
    // regardless of raw ADC offset or internal-ADC voltage.
    int16_t menuNav = 0;
    if (menu_screen != MENU_MAIN) {
        // channels[ELEVATOR_CHANNEL] and channels[RUDDER_CHANNEL] are already
        // deadzone- and calibration-corrected.  Re-center them around 0.
        const int16_t elevDev = (int16_t)channels[ELEVATOR_CHANNEL] - 1500;
        const int16_t ruddDev = (int16_t)channels[RUDDER_CHANNEL]   - 1500;
        if (abs(ruddDev) > MENU_NAV_CENTER_DEADZONE)
            menuNav = ruddDev;
        else if (abs(elevDev) > MENU_NAV_CENTER_DEADZONE)
            menuNav = elevDev;
    }
    if (abs(menuNav) <= MENU_NAV_CENTER_DEADZONE) {
        menuNavLatched = false;
    } else if (!menuNavLatched &&
               abs(menuNav) >= MENU_NAV_TRIGGER_THRESHOLD &&
               (millis() - lastMenuNavMs) > MENU_NAV_DEBOUNCE_MS) {
        moveMenuCursor(menuNav > 0 ? 1 : -1);
        lastMenuNavMs = millis();
        menuNavLatched = true;
    }

    // #region agent log
    if (menu_screen != MENU_MAIN && menuDebugCount < 20 && millis() - lastMenuDebugMs > 250) {
        char dataJson[256];
        snprintf(
            dataJson,
            sizeof(dataJson),
            "{\"menuScreen\":%u,\"menuCursor\":%u,\"enterPressed\":%u,\"backPressed\":%u,\"rawAdc\":[%d,%d,%d,%d],\"channels\":[%u,%u,%u,%u]}",
            (unsigned)menu_screen,
            (unsigned)menu_cursor,
            enterPressed ? 1u : 0u,
            backPressed ? 1u : 0u,
            raw_adc[0], raw_adc[1], raw_adc[2], raw_adc[3],
            (unsigned)channels[0], (unsigned)channels[1], (unsigned)channels[2], (unsigned)channels[3]);
        debug_log_ndjson("rc_crsf_main.cpp:698", "menu_state", "repro1", "H4", dataJson);
        lastMenuDebugMs = millis();
        menuDebugCount++;
    }
    // #endregion
    
    // BACK: long-press = menu / back, short-press = STATUS_REQ when on main
    if (backPressed) {
        if (back_press_start == 0) back_press_start = millis();
        if (!back_long_press_handled && (millis() - back_press_start) >= LONG_PRESS_MS) {
            back_long_press_handled = true;
            if (menu_screen == MENU_MAIN) {
                menu_screen = MENU_TOP;
                menu_cursor = 0;
                Serial.println("[RC] Menu open");
            } else if (menu_screen == MENU_TOP) {
                menu_screen = MENU_MAIN;
                Serial.println("[RC] Menu close");
            } else if (menu_screen == MENU_BINDING) {
                menu_screen = MENU_TOP;
                menu_cursor = 0;
            } else {
                menu_screen = MENU_TOP;
                menu_cursor = 0;
            }
        }
    } else {
        if (back_press_start > 0 && !back_long_press_handled && (millis() - back_press_start) >= BUTTON_DEBOUNCE_MS) {
            if (menu_screen == MENU_MAIN) {
                uartProto.sendCommand(UART_MSG_CMD_STATUS_REQ);
                Serial.println("[RC] Sending STATUS_REQ command");
            } else if (menu_screen == MENU_TOP) {
                uint8_t n = (get_effective_tx_type() == TX_TYPE_ELRS && elrs_role == ELRS_ROLE_TX) ? 4 : 3;
                menu_cursor = (menu_cursor + 1) % n;
            } else if (menu_screen == MENU_TX_MODE) {
                menu_cursor = (menu_cursor + 1) % 4;
            } else if (menu_screen == MENU_BINDING) {
                menu_cursor = (menu_cursor + 1) % MENU_BINDING_ITEMS;
            }
        }
        back_press_start = 0;
        back_long_press_handled = false;
        backButtonPressed = false;
    }
    if (backPressed) backButtonPressed = true;

    // ENTER: in menu = select, on main = PAIR
    if (enterPressed && !enterButtonPressed && (millis() - lastEnterPress) > BUTTON_DEBOUNCE_MS) {
        enterButtonPressed = true;
        lastEnterPress = millis();
        if (menu_screen == MENU_MAIN) {
            uartProto.sendCommand(UART_MSG_CMD_PAIR);
            Serial.println("[RC] Sending PAIR command");
        } else if (menu_screen == MENU_TOP) {
            if (menu_cursor == 0) { menu_screen = MENU_TX_MODE; menu_cursor = (uint8_t)tx_mode_override; }
            else if (menu_cursor == 1) { run_tx_detection(); check_tx_connection(); menu_screen = MENU_MAIN; }
            else if (menu_cursor == 2 && get_effective_tx_type() == TX_TYPE_ELRS && elrs_role == ELRS_ROLE_TX) { menu_screen = MENU_BINDING; menu_cursor = 0; }
            else menu_screen = MENU_MAIN;  // Back (cursor 2 when not ELRS, or cursor 3 when ELRS)
        } else if (menu_screen == MENU_BINDING) {
            if (menu_cursor == 0) {
                if (!elrs_discover_params(true, false, 1200))
                    Serial.println("[RC] Failed to read ELRS binding phrase");
            } else if (menu_cursor == 1) {
                if (elrs_phrase_field_id == ELRS_PARAM_FIELD_NONE) {
                    if (!elrs_discover_params(true, false, 1200))
                        Serial.println("[RC] Failed to discover ELRS binding phrase parameter");
                }
                if (elrs_phrase_field_id != ELRS_PARAM_FIELD_NONE) {
                    static const char gen[] = "ABCDEFGHJKLMNPQRSTUVWXYZ23456789";
                    char buf[ELRS_PHRASE_MAX + 1];
                    buf[0] = 'Q'; buf[1] = 'P'; buf[2] = '-';
                    for (int i = 0; i < 4; i++) buf[3 + i] = gen[random(0, (int)(sizeof(gen)-1))];
                    buf[7] = '-';
                    for (int i = 0; i < 4; i++) buf[8 + i] = gen[random(0, (int)(sizeof(gen)-1))];
                    buf[12] = '-';
                    for (int i = 0; i < 4; i++) buf[13 + i] = gen[random(0, (int)(sizeof(gen)-1))];
                    buf[16] = '\0';
                    elrs_write_param_string(elrs_phrase_field_id, buf);
                    memcpy(elrs_binding_phrase, buf, sizeof(buf));
                } else {
                    Serial.println("[RC] ELRS phrase parameter unavailable");
                }
            } else if (menu_cursor == 2) {
                if (!rc_proto_enter_pairing_mode())
                    Serial.println("[RC] Failed to enter ELRS pairing mode");
            } else if (menu_cursor == 3) {
                if (!rc_proto_enter_wifi_mode())
                    Serial.println("[RC] Failed to enter ELRS WiFi mode");
            } else {
                menu_screen = MENU_TOP;
                menu_cursor = 0;
            }
        } else if (menu_screen == MENU_TX_MODE) {
            if (menu_cursor <= 2) {
                tx_mode_override = (TxModeOverride)menu_cursor;
                if (tx_mode_override == TX_MODE_ELRS) crsfSerial.begin(CRSFSerialConnector::CRSF_BAUDRATE);
            }
            menu_screen = MENU_TOP;
            menu_cursor = 0;
        }
    } else if (!enterPressed) {
        enterButtonPressed = false;
    }
    
    // POWER button - Shutdown mode (active LOW)
    // Debug: log button state changes
    static bool lastPowerState = false;
    static unsigned long lastPowerDebugLog = 0;
    if (powerPressed != lastPowerState) {
        Serial.print("[RC] Power button state changed: ");
        Serial.print(lastPowerState ? "PRESSED (LOW)" : "RELEASED (HIGH)");
        Serial.print(" -> ");
        Serial.println(powerPressed ? "PRESSED (LOW)" : "RELEASED (HIGH)");
        lastPowerState = powerPressed;
    }
    // Periodic debug log every 5 seconds
    if (millis() - lastPowerDebugLog > 5000) {
        Serial.print("[RC] Power button (GPIO ");
        Serial.print(POWER_BUTTON_PIN);
        Serial.print(") state: ");
        Serial.println(powerPressed ? "PRESSED (LOW)" : "RELEASED (HIGH)");
        lastPowerDebugLog = millis();
    }
    
    if (powerPressed && !powerButtonPressed && (millis() - lastPowerPress) > BUTTON_DEBOUNCE_MS) {
        powerButtonPressed = true;
        lastPowerPress = millis();
        Serial.println("[RC] Power button pressed - entering shutdown mode");
        int ret = bq2562x_enter_shutdown_mode();
        if (ret == 0) {
            Serial.println("[RC] Shutdown mode command sent successfully");
        } else if (ret == -3) {
            Serial.println("[RC] Cannot shutdown: VBUS source is present");
        } else {
            Serial.print("[RC] Failed to enter shutdown mode: ");
            Serial.println(ret);
        }
    } else if (!powerPressed) {
        powerButtonPressed = false;
    }
}

static bool is_menu_button_pressed(uint8_t pin, bool idleLevel) {
    return digitalRead(pin) != idleLevel;
}

// ============================================================
// Display
// ============================================================
static void drawBatteryBar(int x, int y, uint8_t percent, bool low_blink) {
    // 4 bars, each BATT_BAR_W wide with BATT_BAR_GAP between
    uint8_t bars = (percent * 4 + 99) / 100;
    if (bars > 4) bars = 4;
    bool blink = low_blink && ((millis() / 500) % 2 == 0);
    for (uint8_t i = 0; i < 4; i++) {
        bool filled = (i < bars);
        if (low_blink && i == bars - 1 && bars > 0) filled = !blink;
        int bx = x + i * (BATT_BAR_W + BATT_BAR_GAP);
        display.fillRect(bx, y, BATT_BAR_W, BATT_BAR_H, filled ? SH110X_WHITE : SH110X_BLACK);
        display.drawRect(bx, y, BATT_BAR_W, BATT_BAR_H, SH110X_WHITE);
    }
}

static void drawLinkIndicator(int x, int y, bool connected) {
    display.fillRect(x, y, LINK_INDICATOR_W, LINK_INDICATOR_H, connected ? SH110X_WHITE : SH110X_BLACK);
    display.drawRect(x, y, LINK_INDICATOR_W, LINK_INDICATOR_H, SH110X_WHITE);
}

static void resync_display_viewport(void) {
    // Re-assert viewport/scroll registers in case I2C noise corrupts OLED state.
    // Symptom this fixes: bottom page appears at top (vertical wrap/look shifted).
    display.oled_command(0x2E); // Deactivate scroll
    display.oled_command(0x40); // Start line = 0
    display.oled_command(0xD3); // Set display offset
    display.oled_command(0x00); // Offset = 0
}

void updateDisplay() {
    if (millis() - lastDisplayUpdate < DISPLAY_INTERVAL) return;
    lastDisplayUpdate = millis();

    if (!display_ok) return;

    resync_display_viewport();

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setTextWrap(false);

    if (menu_screen != MENU_MAIN) {
        int y = 0;
        if (menu_screen == MENU_TOP) {
            bool elrs = (get_effective_tx_type() == TX_TYPE_ELRS && elrs_role == ELRS_ROLE_TX);
            const char* items4[] = {"TX Mode", "Re-detect TX", "Binding/ELRS", "Back"};
            const char* items3[] = {"TX Mode", "Re-detect TX", "Back"};
            int n = elrs ? 4 : 3;
            for (int i = 0; i < n; i++) {
                display.setCursor(COL_0, y);
                if (i == (int)menu_cursor) display.print(">");
                display.println(elrs ? items4[i] : items3[i]);
                y += ROW_H;
            }
        } else if (menu_screen == MENU_TX_MODE) {
            const char* items[] = {"Auto", "Custom", "ELRS", "Back"};
            for (int i = 0; i < 4; i++) {
                display.setCursor(COL_0, y);
                if (i == (int)menu_cursor) display.print(">");
                display.println(items[i]);
                y += ROW_H;
            }
        } else if (menu_screen == MENU_BINDING) {
            const char* items[] = {"View phrase", "Set phrase", "Initiate pair", "Start WiFi", "Back"};
            for (int i = 0; i < MENU_BINDING_ITEMS; i++) {
                display.setCursor(COL_0, y);
                if (i == (int)menu_cursor) display.print(">");
                display.println(items[i]);
                y += ROW_H;
            }
            if (elrs_binding_phrase[0]) {
                display.setCursor(COL_0, y);
                display.print(elrs_binding_phrase);
            }
        }
        display.display();
        return;
    }

    const char* linkStateNames[] = {"DISC", "PAIR", "CONN", "OK", "LOST"};
    int y = 0;

    // Row 0: TX type (or Detecting) + link indicator
    display.setCursor(COL_0, y);
    if (detecting_tx) {
        display.print("Detecting TX...");
    } else {
        TxType eff = get_effective_tx_type();
        if (eff == TX_TYPE_CUSTOM) display.print("TX: Custom");
        else if (eff == TX_TYPE_ELRS) {
            if (elrs_role == ELRS_ROLE_RX) {
                display.print("Wrong: ELRS RX");
            } else {
                display.print("TX: ELRS");
                if (elrs_device_name[0]) {
                    display.print(" ");
                    for (int i = 0; i < 10 && elrs_device_name[i]; i++) display.print(elrs_device_name[i]);
                }
            }
        } else display.print("TX: --");
    }
    // Link indicator: for ELRS, show RX connection (link stats); for Custom, show TX presence
    bool linkOk = (get_effective_tx_type() == TX_TYPE_ELRS && elrs_role == ELRS_ROLE_TX)
        ? crsfSerial.isLinkUp()
        : tx_connected;
    drawLinkIndicator(LINK_INDICATOR_X, LINK_INDICATOR_Y, linkOk);
    y += ROW_H;

    // Row 1: TX Batt  4.20V  100% [||||]
    display.setCursor(COL_0, y);
    display.print("TX ");
    if (tx_batt_valid) {
        display.print(tx_batt_voltage, 2);
        display.print("V ");
        display.print(tx_batt_percent);
        display.print("%");
        drawBatteryBar(BATT_BAR_X, y, tx_batt_percent, tx_batt_low);
    } else {
        display.print("--V --%");
    }
    y += ROW_H;

    // Row 2: RX Batt  4.2V  100%  Chg:xx V:xx
    display.setCursor(COL_0, y);
    display.print("RX ");
    if (telemetryValid) {
        display.print(telemetry.rxBattMv / 1000.0f, 1);
        display.print("V ");
        display.print(telemetry.rxBattPct);
        display.print("%");
    } else {
        display.print("--  --%");
    }
    if (tx_charge_status_valid) {
        display.setCursor(COL_LINK, y);
        display.print(get_charge_status_string(tx_charge_status));
        display.print(" ");
        const char* vbus = get_vbus_status_string(tx_vbus_status);
        for (int i = 0; i < 3 && vbus[i]; i++) display.print(vbus[i]);
    }
    y += ROW_H;

    // Row 3: Link: OK   RSSI:-50 LQ:98
    display.setCursor(COL_0, y);
    display.print("Link:");
    if (get_effective_tx_type() == TX_TYPE_ELRS && elrs_role == ELRS_ROLE_TX) {
        display.print(crsfSerial.isLinkUp() ? "OK" : "---");
    } else if (tx_connected) {
        if (statusValid && status.connectionState < 5) {
            display.print(linkStateNames[status.connectionState]);
        } else {
            display.print("OK");
        }
    } else {
        display.print("---");
    }
    display.setCursor(COL_LINK, y);
    if (telemetryValid) {
        display.print("R:");
        display.print(telemetry.rssi);
        display.print(" L:");
        display.print(telemetry.linkQuality);
    } else {
        display.print("R:-- L:--");
    }
    y += ROW_H;

    // Row 4-5: A   E   R   T  and values
    display.setCursor(COL_0, y);
    display.print("A    E    R    T");
    y += ROW_H;
    display.setCursor(COL_0, y);
    display.print(channels[AILERON_CHANNEL]);
    display.print(" ");
    display.print(channels[ELEVATOR_CHANNEL]);
    display.print(" ");
    display.print(channels[RUDDER_CHANNEL]);
    display.print(" ");
    display.print(channels[THROTTLE_CHANNEL]);

    display.display();
}

// ============================================================
// TX Device Connection Check
// ============================================================
void ping_tx_device(void) {
    if (get_effective_tx_type() != TX_TYPE_CUSTOM) return;  // Only ping for Custom TX
    if (millis() - lastPingSent < PING_INTERVAL) return;
    lastPingSent = millis();
    uartProto.sendPing();
}

bool check_tx_connection(void) {
    TxType eff = get_effective_tx_type();
    if (eff == TX_TYPE_ELRS) {
        // ELRS: consider connected if we received DEVICE_INFO (or later: link stats)
        bool wasConnected = tx_connected;
        tx_connected = elrs_device_info_received ||
                       crsfSerial.isLinkUp() ||
                       (lastElrsFrameReceived > 0 && (millis() - lastElrsFrameReceived) < ELRS_FRAME_TIMEOUT_MS);
        if (wasConnected != tx_connected && tx_connected)
            Serial.println("[TX] ELRS module connected");
        return tx_connected;
    }
    // Custom: pong-based
    unsigned long timeSincePong = (lastPongReceived > 0) ? (millis() - lastPongReceived) : PONG_TIMEOUT_MS + 1;
    bool wasConnected = tx_connected;
    tx_connected = (timeSincePong < PONG_TIMEOUT_MS);
    if (wasConnected != tx_connected) {
        if (tx_connected) Serial.println("[TX] Device connected");
        else Serial.println("[TX] Device disconnected or not responding");
    }
    static bool initialCheckDone = false;
    if (!initialCheckDone) {
        initialCheckDone = true;
        if (tx_connected) Serial.println("[TX] Initial check: Device connected");
        else Serial.println("[TX] Initial check: Device not responding");
    }
    return tx_connected;
}

static TxType get_effective_tx_type(void) {
    if (tx_mode_override == TX_MODE_CUSTOM) return TX_TYPE_CUSTOM;
    if (tx_mode_override == TX_MODE_ELRS) return TX_TYPE_ELRS;
    return tx_type;
}

static void onCrsfExtendedFrame(uint8_t sourceAddr, uint8_t type) {
    if (sourceAddr != CRSF_ADDRESS_CRSF_TRANSMITTER && sourceAddr != CRSF_ADDRESS_CRSF_RECEIVER)
        return;

    lastElrsFrameReceived = millis();

    ElrsRole newRole = (sourceAddr == CRSF_ADDRESS_CRSF_TRANSMITTER) ? ELRS_ROLE_TX : ELRS_ROLE_RX;
    bool firstSeen = (tx_type != TX_TYPE_ELRS) || (elrs_role != newRole);
    tx_type = TX_TYPE_ELRS;
    elrs_role = newRole;

    // ELRS TX sends periodic DEVICE_PING; responding with DEVICE_INFO registers us as
    // a Lua client so the TX will answer PARAMETER_READ requests.
    if (type == CRSF_FRAMETYPE_DEVICE_PING && newRole == ELRS_ROLE_TX)
        send_crsf_device_info();

    if (!firstSeen || type == CRSF_FRAMETYPE_DEVICE_INFO)
        return;

    if (newRole == ELRS_ROLE_TX) {
        Serial.print("[TX] Detected ELRS transmitter via CRSF frame 0x");
        Serial.println(type, HEX);
    } else {
        Serial.print("[TX] Wrong mode: ELRS RX via CRSF frame 0x");
        Serial.println(type, HEX);
    }
}

static void onCrsfDeviceInfo(const uint8_t *serial4, const char *name, uint8_t sourceAddr) {
    // Accept DEVICE_INFO from the CRSF TX module regardless of serial number.
    // ELRS 3.x uses a real MCU UID, not the string "ELRS", so don't check serial4.
    (void)serial4;
    if (sourceAddr != CRSF_ADDRESS_CRSF_TRANSMITTER) return;
    lastElrsFrameReceived = millis();
    tx_type = TX_TYPE_ELRS;
    elrs_device_info_received = true;
    if (sourceAddr == CRSF_ADDRESS_CRSF_TRANSMITTER)
        elrs_role = ELRS_ROLE_TX;
    else if (sourceAddr == CRSF_ADDRESS_CRSF_RECEIVER)
        elrs_role = ELRS_ROLE_RX;
    else
        elrs_role = ELRS_ROLE_UNKNOWN;
    memset(elrs_device_name, 0, sizeof(elrs_device_name));
    if (name) {
        size_t n = strlen(name);
        if (n >= sizeof(elrs_device_name)) n = sizeof(elrs_device_name) - 1;
        memcpy(elrs_device_name, name, n);
    }
    if (elrs_role == ELRS_ROLE_TX) {
        Serial.println("[TX] Detected ELRS transmitter");
        if (elrs_device_name[0]) { Serial.print("[TX] Device name: "); Serial.println(elrs_device_name); }
    } else if (elrs_role == ELRS_ROLE_RX) {
        Serial.println("[TX] Wrong mode: ELRS RX (need TX module)");
    }
}

// Announce ourselves to the ELRS TX module as a Lua/config client.
// ELRS TX requires a DEVICE_INFO handshake before it will respond to PARAMETER_READ.
static void send_crsf_device_info(void) {
    static const char devName[] = "RC-CRSF";
    uint8_t payload[32];
    uint8_t offset = 0;
    memcpy(payload, devName, sizeof(devName));   // includes null terminator
    offset += sizeof(devName);
    uint32_t serial = 0x52435246UL;              // "RCRF" as LE identifier
    memcpy(payload + offset, &serial, 4); offset += 4;
    uint32_t ver = 0x00010000UL;
    memcpy(payload + offset, &ver, 4); offset += 4;   // hwVer
    memcpy(payload + offset, &ver, 4); offset += 4;   // swVer
    payload[offset++] = 0;   // fieldCnt (we have no params to expose)
    payload[offset++] = 0;   // paramVer
    crsfSerial.queueExtendedPacket(
        CRSF_ADDRESS_CRSF_TRANSMITTER,
        CRSF_ADDRESS_RADIO_TRANSMITTER,
        CRSF_FRAMETYPE_DEVICE_INFO,
        payload, offset);
}

static void elrs_request_param_chunk(uint8_t fieldId, uint8_t chunkIndex) {
    Serial.printf("[ELRS PARAM_READ] field=%u chunk=%u\n", fieldId, chunkIndex);
    uint8_t req[2] = { fieldId, chunkIndex };
    crsfSerial.queueExtendedPacket(CRSF_ADDRESS_CRSF_TRANSMITTER, CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_FRAMETYPE_PARAMETER_READ, req, 2);
}

static bool elrs_discover_params(bool needPhrase, bool needBind, unsigned long timeoutMs) {
    if ((!needPhrase || elrs_phrase_field_id != ELRS_PARAM_FIELD_NONE) &&
        (!needBind || elrs_bind_cmd_field_id != ELRS_PARAM_FIELD_NONE)) {
        return true;
    }

    // Enable verbose UART logging for the entire discovery window so we can see
    // every raw byte in both directions and every parsed/rejected frame.
    crsfSerial.setVerboseMode(true);

    // Send DEVICE_PING to (re-)establish the Lua session on the ELRS TX.
    // ELRS TX firmware only responds to PARAMETER_READ when its luaConnected flag is set,
    // which is triggered by receiving a DEVICE_PING from the radio — not by DEVICE_INFO.
    // The session from boot detection expires after ~10-15 s, so we must refresh it here.
    Serial.println("[ELRS] Sending DEVICE_PING to start Lua session...");
    crsfSerial.queueExtendedPacket(
        CRSF_ADDRESS_CRSF_TRANSMITTER,
        CRSF_ADDRESS_RADIO_TRANSMITTER,
        CRSF_FRAMETYPE_DEVICE_PING,
        nullptr, 0);
    // Process incoming bytes for ~120 ms while the TX opens its Lua session and
    // optionally sends DEVICE_INFO back.
    {
        unsigned long pingEnd = millis() + 120;
        while (millis() < pingEnd) {
            receiveUARTMessages();
            sendChannels();
            delay(2);
        }
    }

    elrs_param_chunk_next = 1;
    elrs_param_chunk_index = 0;
    elrs_param_read_in_progress = true;
    elrs_last_param_entry_ms = millis();
    Serial.printf("[ELRS] Starting param discovery (needPhrase=%u needBind=%u timeout=%lums)\n",
                  needPhrase, needBind, timeoutMs);
    elrs_request_param_chunk(elrs_param_chunk_next, elrs_param_chunk_index);

    unsigned long deadline = millis() + timeoutMs;
    while (millis() < deadline) {
        receiveUARTMessages();
        sendChannels();  // keep channel packets flowing so the TX link stays healthy
        if ((!needPhrase || elrs_phrase_field_id != ELRS_PARAM_FIELD_NONE) &&
            (!needBind || elrs_bind_cmd_field_id != ELRS_PARAM_FIELD_NONE)) {
            elrs_param_read_in_progress = false;
            crsfSerial.setVerboseMode(false);
            return true;
        }
        if (!elrs_param_read_in_progress)
            break;
        // Retry the current request if we haven't received a response in 250 ms.
        if (millis() - elrs_last_param_entry_ms > 250) {
            Serial.printf("[ELRS] Retry param request field=%u chunk=%u\n",
                          elrs_param_chunk_next, elrs_param_chunk_index);
            elrs_last_param_entry_ms = millis();
            elrs_request_param_chunk(elrs_param_chunk_next, elrs_param_chunk_index);
        }
        delay(2);
    }

    elrs_param_read_in_progress = false;
    crsfSerial.setVerboseMode(false);
    return (!needPhrase || elrs_phrase_field_id != ELRS_PARAM_FIELD_NONE) &&
           (!needBind || elrs_bind_cmd_field_id != ELRS_PARAM_FIELD_NONE);
}

// CRSF type: 0x0A = STRING, 0x0D = COMMAND
static void onCrsfParameterEntry(uint8_t fieldId, uint8_t paramType, uint8_t chunksRemaining, const char *label, const uint8_t *value, uint8_t valueLen) {
    if (!label) return;
    elrs_last_param_entry_ms = millis();
    Serial.printf("[ELRS PARAM] field=%u type=%u chunks=%u label=\"%s\"\n", fieldId, paramType, chunksRemaining, label);
    // Match "Binding Phrase" / "binding phrase" (ExpressLRS)
    if (paramType == 0x0A) {
        if (strstr(label, "inding") && strstr(label, "hrase")) {
            elrs_phrase_field_id = fieldId;
            memset(elrs_binding_phrase, 0, sizeof(elrs_binding_phrase));
            if (value && valueLen > 0) {
                uint8_t n = valueLen;
                if (value[0] == 0 && valueLen > 1) { n = valueLen - 1; value = &value[1]; }  // optional max_len byte
                if (n >= sizeof(elrs_binding_phrase)) n = sizeof(elrs_binding_phrase) - 1;
                memcpy(elrs_binding_phrase, value, n);
            }
        }
    } else if (paramType == 0x0D) {
        if (strstr(label, "ind") && !strstr(label, "hrase")) {
            elrs_bind_cmd_field_id = fieldId;
        }
        // Match "Enable WiFi Update", "WiFi Connectivity", etc.  Take the first WiFi COMMAND found.
        if (elrs_wifi_cmd_field_id == ELRS_PARAM_FIELD_NONE &&
            (strstr(label, "WiFi") || strstr(label, "wifi") || strstr(label, "Wifi"))) {
            elrs_wifi_cmd_field_id = fieldId;
            Serial.printf("[ELRS PARAM] WiFi cmd field=%u\n", fieldId);
        }
    }

    // Continue scanning until all three fields are cached so the WiFi option works
    // even when discovery was originally triggered for phrase/bind only.
    if (elrs_phrase_field_id != ELRS_PARAM_FIELD_NONE &&
        elrs_bind_cmd_field_id != ELRS_PARAM_FIELD_NONE &&
        elrs_wifi_cmd_field_id != ELRS_PARAM_FIELD_NONE) {
        elrs_param_read_in_progress = false;
        return;
    }

    if (elrs_param_read_in_progress && chunksRemaining > 0) {
        elrs_param_chunk_index++;
        elrs_request_param_chunk(fieldId, elrs_param_chunk_index);
    } else if (elrs_param_read_in_progress && fieldId < ELRS_PARAM_MAX_CHUNKS) {
        elrs_param_chunk_next = fieldId + 1;
        elrs_param_chunk_index = 0;
        elrs_request_param_chunk(elrs_param_chunk_next, 0);
    } else {
        elrs_param_read_in_progress = false;
    }
}

static bool elrs_write_param_string(uint8_t fieldId, const char *str) {
    size_t len = str ? strlen(str) : 0;
    if (len > ELRS_PHRASE_MAX) len = ELRS_PHRASE_MAX;
    if (len + 2 > CRSFSerialConnector::CRSF_MAX_PAYLOAD_LEN) return false;
    uint8_t buf[CRSFSerialConnector::CRSF_MAX_PAYLOAD_LEN];
    buf[0] = fieldId;
    memcpy(&buf[1], str, len);
    buf[1 + len] = '\0';
    crsfSerial.queueExtendedPacket(CRSF_ADDRESS_CRSF_TRANSMITTER, CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_FRAMETYPE_PARAMETER_WRITE, buf, (uint8_t)(1 + len + 1));
    return true;
}

static bool elrs_write_param_byte(uint8_t fieldId, uint8_t val) {
    uint8_t buf[2] = { fieldId, val };
    crsfSerial.queueExtendedPacket(CRSF_ADDRESS_CRSF_TRANSMITTER, CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_FRAMETYPE_PARAMETER_WRITE, buf, 2);
    return true;
}

static void run_tx_detection(void) {
    detecting_tx = true;
    tx_type = TX_TYPE_UNKNOWN;
    elrs_device_info_received = false;
    elrs_role = ELRS_ROLE_UNKNOWN;
    elrs_phrase_field_id = ELRS_PARAM_FIELD_NONE;
    elrs_bind_cmd_field_id = ELRS_PARAM_FIELD_NONE;
    elrs_wifi_cmd_field_id = ELRS_PARAM_FIELD_NONE;
    elrs_binding_phrase[0] = '\0';
    lastElrsFrameReceived = 0;
    memset(elrs_device_name, 0, sizeof(elrs_device_name));
    lastPongReceived = 0;
    Serial.println("[TX] Detecting TX type...");
    
    // Phase 1: Custom PING
    uartProto.sendPing();
    lastPingSent = millis();
    const unsigned long detectTimeout = 400;
    unsigned long deadline = millis() + detectTimeout;
    while (millis() < deadline) {
        uartProto.loop();
        if (lastPongReceived != 0) {
            tx_type = TX_TYPE_CUSTOM;
            Serial.println("[TX] Detected: Custom TX (PONG)");
            detecting_tx = false;
#if defined(CRSF_CORE1_CHANNELS)
            s_core1_send_elrs_channels = false;
#endif
            return;
        }
        delay(2);
    }
    
    // Phase 2: CRSF DEVICE_PING to 0xEE
    while (Serial2.available()) Serial2.read();
    crsfSerial.begin(CRSFSerialConnector::CRSF_BAUDRATE);
    crsfSerial.onDeviceInfo = onCrsfDeviceInfo;
    crsfSerial.queueExtendedPacket(CRSF_ADDRESS_CRSF_TRANSMITTER, CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_FRAMETYPE_DEVICE_PING, nullptr, 0);
    deadline = millis() + detectTimeout;
    while (millis() < deadline) {
        crsfSerial.loop();
        if (tx_type == TX_TYPE_ELRS) {
            if (elrs_device_info_received)
                Serial.println("[TX] Detected: ELRS TX (DEVICE_INFO)");
            else if (elrs_role == ELRS_ROLE_TX)
                Serial.println("[TX] Detected: ELRS TX (CRSF traffic)");
            else if (elrs_role == ELRS_ROLE_RX)
                Serial.println("[TX] Detected: ELRS RX (CRSF traffic)");
            else
                Serial.println("[TX] Detected: ELRS device (CRSF traffic)");
            // Register ourselves so the TX will accept PARAMETER_READ requests.
            if (elrs_role == ELRS_ROLE_TX)
                send_crsf_device_info();
            detecting_tx = false;
#if defined(CRSF_CORE1_CHANNELS)
            s_core1_send_elrs_channels = (elrs_role == ELRS_ROLE_TX);
#endif
            return;
        }
        delay(2);
    }
    
    Serial.println("[TX] Detection: No TX responded (Unknown)");
    detecting_tx = false;
#if defined(CRSF_CORE1_CHANNELS)
    s_core1_send_elrs_channels = false;
#endif
}

// ============================================================
// USB config protocol callbacks
// ============================================================
// Push g_rc_config to legacy calib, Core 1 snapshot, and reset filters so trim/calibration take effect without reboot.
static void rc_sync_runtime_from_g_rc_config(void) {
    for (int i = 0; i < 4; i++) {
        calib.min[i] = g_rc_config.calib_min[i];
        calib.max[i] = g_rc_config.calib_max[i];
        calib.center[i] = g_rc_config.calib_center[i];
    }
#if defined(CRSF_CORE1_CHANNELS)
    // F1/F3: hand the new config to Core 1 via a staging buffer + dirty flag. Core 1 copies
    // it and resets its own stick filter at a frame boundary. Do NOT write s_core1_config or
    // the filter masks from here — both are owned by Core 1 (cross-core race otherwise).
    memcpy(&s_core1_config_staging, &g_rc_config, sizeof(s_core1_config_staging));
    __dmb();
    s_core1_config_dirty = true;
#elif defined(INTERNAL_ADC)
    internal_adc_reset_stick_filter();
#endif
    hpf_initialized = false;
}

static const rc_config_data_t* rc_proto_get_config(void) {
    return &g_rc_config;
}
static void rc_proto_set_draft(const rc_config_data_t* draft) {
    if (draft) memcpy(&g_rc_config_draft, draft, sizeof(g_rc_config_draft));
}
static void rc_proto_apply(void) {
    memcpy(&g_rc_config, &g_rc_config_draft, sizeof(g_rc_config));
    rc_config_validate(&g_rc_config);
    memcpy(&g_rc_config_draft, &g_rc_config, sizeof(g_rc_config_draft));
    rc_sync_runtime_from_g_rc_config();
}
static bool rc_proto_save(void) {
    memcpy(&g_rc_config, &g_rc_config_draft, sizeof(g_rc_config));
    rc_config_validate(&g_rc_config);
    memcpy(&g_rc_config_draft, &g_rc_config, sizeof(g_rc_config_draft));
    rc_sync_runtime_from_g_rc_config();
    return rc_config_save(&g_rc_config);
}
static void rc_proto_cal_start(void) {
    cal_sampling = true;
    for (int i = 0; i < 4; i++) {
        cal_axis_min[i] = raw_adc[i];
        cal_axis_max[i] = raw_adc[i];
    }
}
static void rc_proto_cal_sample(void) {
    if (!cal_sampling) return;
    for (int i = 0; i < 4; i++) {
        if (raw_adc[i] < cal_axis_min[i]) cal_axis_min[i] = raw_adc[i];
        if (raw_adc[i] > cal_axis_max[i]) cal_axis_max[i] = raw_adc[i];
    }
}
static void rc_proto_cal_finish(void) {
    if (!cal_sampling) return;
    cal_sampling = false;
    for (int i = 0; i < 4; i++) {
        g_rc_config.calib_min[i] = (uint16_t)cal_axis_min[i];
        g_rc_config.calib_max[i] = (uint16_t)cal_axis_max[i];
        g_rc_config.calib_center[i] = (uint16_t)((cal_axis_min[i] + cal_axis_max[i]) / 2);
    }
    rc_config_validate(&g_rc_config);
    rc_sync_runtime_from_g_rc_config();
    rc_config_save(&g_rc_config);
    memcpy(&g_rc_config_draft, &g_rc_config, sizeof(g_rc_config_draft));
}

static void rc_proto_get_state(int16_t* adc4, uint16_t* ch8, uint8_t* toggles4) {
    if (adc4) for (int i = 0; i < 4; i++) adc4[i] = raw_adc[i];
    if (ch8) for (int i = 0; i < 8; i++) ch8[i] = channels[i];
    if (toggles4) for (int i = 0; i < 4; i++) toggles4[i] = raw_toggles[i] ? 1 : 0;
}

static bool rc_proto_set_binding_phrase_rx(const char* phrase, uint8_t len) {
    if (!tx_connected || !phrase || len == 0 || len > 32) return false;
    bool sent = uartProto.sendCommandWithPayload(UART_MSG_CMD_SET_BIND_RX, (const uint8_t*)phrase, len);
    if (sent) {
        Serial.println("[RC] Forwarded RX binding phrase update command to TX");
    } else {
        Serial.println("[RC] Failed to forward RX binding phrase command to TX");
    }
    return sent;
}

static bool rc_proto_set_binding_phrase_tx(const char* phrase, uint8_t len) {
    if (!phrase || len == 0 || len > 32) return false;
    if (get_effective_tx_type() == TX_TYPE_ELRS && elrs_role == ELRS_ROLE_TX) {
        if (elrs_phrase_field_id == ELRS_PARAM_FIELD_NONE && !elrs_discover_params(true, false, 1200))
            return false;
        if (elrs_phrase_field_id == ELRS_PARAM_FIELD_NONE) return false;
        char buf[ELRS_PHRASE_MAX + 1];
        if (len >= sizeof(buf)) len = sizeof(buf) - 1;
        memcpy(buf, phrase, len);
        buf[len] = '\0';
        bool sent = elrs_write_param_string(elrs_phrase_field_id, buf);
        if (sent) {
            memcpy(elrs_binding_phrase, buf, len + 1);
            Serial.println("[RC] Set ELRS binding phrase via CRSF");
        }
        return sent;
    }
    if (!tx_connected) return false;
    bool sent = uartProto.sendCommandWithPayload(UART_MSG_CMD_SET_BIND_TX, (const uint8_t*)phrase, len);
    if (sent) Serial.println("[RC] Forwarded TX binding phrase to custom TX");
    return sent;
}

static void rc_proto_get_link_status(uint8_t* txConnected, uint8_t* txPaired, uint8_t* txState) {
    if (txConnected) *txConnected = tx_connected ? 1 : 0;
    if (txPaired) *txPaired = statusValid ? status.pairingState : 0;
    if (txState) *txState = statusValid ? status.connectionState : 0xFF;
}

static void rc_proto_get_link_status_ex(uint8_t* buf, uint8_t bufLen) {
    if (!buf || bufLen < 4) return;
    buf[0] = tx_connected ? 1 : 0;
    buf[1] = statusValid ? status.pairingState : 0;
    buf[2] = statusValid ? status.connectionState : 0xFF;
    TxType eff = get_effective_tx_type();
    buf[3] = (uint8_t)eff;
    if (bufLen >= 6 && eff == TX_TYPE_ELRS) {
        buf[4] = (uint8_t)elrs_role;  // 1=TX, 2=RX
        if (elrs_device_name[0]) {
            size_t n = strlen(elrs_device_name);
            if (n > 20) n = 20;
            buf[5] = (uint8_t)n;
            if (bufLen >= (size_t)(6 + n)) memcpy(&buf[6], elrs_device_name, n);
        } else {
            buf[5] = 0;
        }
    }
}

static void rc_proto_re_detect_tx(void) {
    run_tx_detection();
    check_tx_connection();
}

static bool rc_proto_get_elrs_binding_phrase(char* phrase, uint8_t maxLen) {
    if (!phrase || maxLen == 0 || get_effective_tx_type() != TX_TYPE_ELRS || elrs_role != ELRS_ROLE_TX) return false;
    if (elrs_binding_phrase[0]) {
        size_t n = strlen(elrs_binding_phrase);
        if (n >= maxLen) n = maxLen - 1;
        memcpy(phrase, elrs_binding_phrase, n);
        phrase[n] = '\0';
        return true;
    }
    if (!elrs_discover_params(true, false, 1200))
        return false;
    if (!elrs_binding_phrase[0]) return false;
    size_t n = strlen(elrs_binding_phrase);
    if (n >= maxLen) n = maxLen - 1;
    memcpy(phrase, elrs_binding_phrase, n);
    phrase[n] = '\0';
    return true;
}

static bool rc_proto_enter_pairing_mode(void) {
    if (get_effective_tx_type() == TX_TYPE_ELRS && elrs_role == ELRS_ROLE_TX) {
        if (elrs_bind_cmd_field_id == ELRS_PARAM_FIELD_NONE && !elrs_discover_params(false, true, 1200))
            return false;
        if (elrs_bind_cmd_field_id == ELRS_PARAM_FIELD_NONE) return false;
        bool sent = elrs_write_param_byte(elrs_bind_cmd_field_id, 0x01);  // CLICK = start bind
        if (sent) Serial.println("[RC] ELRS bind command sent (CRSF param)");
        return sent;
    }
    if (!tx_connected) return false;
    bool sent = uartProto.sendCommand(UART_MSG_CMD_PAIR);
    if (sent) Serial.println("[RC] Forwarded PAIR command to TX");
    return sent;
}

static bool rc_proto_enter_wifi_mode(void) {
    if (get_effective_tx_type() != TX_TYPE_ELRS || elrs_role != ELRS_ROLE_TX) return false;
    if (elrs_wifi_cmd_field_id == ELRS_PARAM_FIELD_NONE) {
        // Scan for all params; WiFi typically appears before phrase/bind so
        // the combined scan will cache it even if phrase/bind are the named goals.
        elrs_discover_params(true, true, 2000);
    }
    if (elrs_wifi_cmd_field_id == ELRS_PARAM_FIELD_NONE) {
        Serial.println("[RC] ELRS WiFi command param not found");
        return false;
    }
    bool sent = elrs_write_param_byte(elrs_wifi_cmd_field_id, 0x01);  // CLICK = enable WiFi
    if (sent) Serial.println("[RC] ELRS WiFi mode command sent");
    return sent;
}

static void rc_proto_enter_usb_uart_proxy(void) {
    // F2: raise the flag first so Core 1 stops writing Serial2, then wait long enough for it to
    // observe the flag (loop1 runs at <=4 ms) and finish any in-flight frame before we re-init
    // the UART. Otherwise Serial2.begin() races a concurrent Core 1 write.
    proxy_mode_enabled = true;
#if defined(CRSF_CORE1_CHANNELS) && defined(INTERNAL_ADC)
    delay(10);
#endif
    Serial2.begin(CRSFSerialConnector::CRSF_BAUDRATE);
    while (Serial.available()) Serial.read();
    while (Serial2.available()) Serial2.read();
    Serial.flush();
    Serial2.flush();
}

static void rc_proto_exit_usb_uart_proxy(void) {
    // Leave bridge mode and drain both directions so leftover passthrough bytes don't leak into
    // normal CRSF parsing on Serial2. Core 1 resumes channel sends once the flag clears.
    while (Serial.available()) Serial.read();
    while (Serial2.available()) Serial2.read();
    proxy_mode_enabled = false;
}

// ============================================================
// Helpers
// ============================================================
static void load_calibration(void) {
    EEPROM.begin(EEPROM_SIZE);
    if (EEPROM.read(CAL_MAGIC_ADDR) != CAL_MAGIC_VALUE) {
        Serial.println("Calibration not found, using defaults");
        calib_loaded = false;
        return;
    }
    EEPROM.get(CAL_DATA_ADDR, calib);
    for (int i = 0; i < 4; i++) {
        if (calib.min[i] >= calib.max[i]) {
            calib.min[i] = RC_CALIB_DEFAULT_MIN;
            calib.max[i] = RC_CALIB_DEFAULT_MAX;
        }
        if (calib.center[i] == 0 || calib.center[i] >= calib.max[i]) {
            calib.center[i] = RC_CALIB_DEFAULT_CENTER;
        }
    }
    calib_loaded = true;
    Serial.println("Calibration loaded");
}

// ==========================================
// I2C Bus Scanning
// ==========================================
static void scan_i2c_bus(void) {
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
            
            // Identify common devices
            if (address == 0x3C || address == 0x3D) {
                Serial.print(" (OLED/Display - SH1106/SSD1306)");
            } else if (address >= 0x48 && address <= 0x4B) {
                Serial.print(" (ADS1115 ADC)");
            } else if (address == 0x6A) {
                Serial.print(" (BQ25620 Charger)");
            }
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
        Serial.println("Check wiring: SDA=GP4, SCL=GP5, VCC=3.3V, GND=GND");
    } else {
        Serial.print("Found ");
        Serial.print(nDevices);
        Serial.println(" device(s)");
    }
}

// ==========================================
// Display Initialization
// ==========================================
static bool init_display(void) {
    Serial.print("Attempting to initialize display at address 0x");
    Serial.println(I2C_ADDRESS, HEX);
    
    // Wait a bit for display to power up
    delay(250);
    
    // Try default address first
    if (!display.begin(I2C_ADDRESS, false)) {  // false = no splash screen
        Serial.println("Failed to initialize display at 0x3C");
        Serial.println("Trying alternative address 0x3D...");
        
        // SH1106 can sometimes be at 0x3D
        if (!display.begin(0x3D, false)) {
            Serial.println("Display not found at 0x3C or 0x3D");
            Serial.println("Check wiring: SDA=GP4, SCL=GP5, VCC=3.3V, GND=GND");
            return false;
        } else {
            Serial.println("Display found at address 0x3D");
        }
    } else {
        Serial.print("Display initialized successfully at address 0x");
        Serial.println(I2C_ADDRESS, HEX);
    }
    
    // Configure display
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setTextWrap(false);
    display.setCursor(0, 0);
    display.println("RC-CRSF init...");
    display.display();
    
    return true;
}

#if !defined(INTERNAL_ADC)
static bool init_ads1115(void) {
    Serial.print("Attempting to initialize ADS1115 at address 0x");
    Serial.println(ADS1115_ADDRESS, HEX);

    if (!ads.begin(ADS1115_ADDRESS, &Wire)) {
        Serial.println("Failed to initialize ADS1115");
        Serial.println("Trying alternative addresses...");

        uint8_t alt_addresses[] = {0x49, 0x4A, 0x4B};
        bool found = false;
        for (uint8_t addr : alt_addresses) {
            Serial.print("Trying 0x");
            Serial.println(addr, HEX);
            if (ads.begin(addr, &Wire)) {
                Serial.print("ADS1115 found at address 0x");
                Serial.println(addr, HEX);
                found = true;
                break;
            }
        }

        if (!found) {
            Serial.println("ADS1115 not found at any address");
            Serial.println("Check wiring: SDA=GP4, SCL=GP5, VCC=3.3V, GND=GND");
            Serial.println("ADDR pin: GND=0x48, VDD=0x49, SDA=0x4A, SCL=0x4B");
            return false;
        }
    } else {
        Serial.print("ADS1115 initialized successfully at address 0x");
        Serial.println(ADS1115_ADDRESS, HEX);
    }

    // Configure ADS1115
    ads.setGain(GAIN_ONE); // ±4.096V range
    ads.setDataRate(RATE_ADS1115_250SPS);
    Serial.println("ADS1115 configured: Gain=1, DataRate=250 SPS");

    // Test read to verify ADC is working
    int16_t test = ads.readADC_SingleEnded(0);
    Serial.print("ADS1115 test read CH0: ");
    Serial.println(test);

    return true;
}
#else
static bool init_internal_adc(void) {
    Serial.println("Using RP2350 internal ADC on GPIO26,27,28,29 (ADC0-3)");
    internal_adc_reset_stick_filter();
    // No explicit init needed; analogRead() initializes ADC on first use.
    // Prime the ADC with a read to avoid first-sample skew
    (void)analogRead(INTERNAL_ADC_AILERON_PIN);
    (void)analogRead(INTERNAL_ADC_ELEVATOR_PIN);
    (void)analogRead(INTERNAL_ADC_RUDDER_PIN);
    (void)analogRead(INTERNAL_ADC_THROTTLE_PIN);
    int16_t test = internal_adc_raw_to_stick(analogRead(INTERNAL_ADC_AILERON_PIN));
    Serial.print("Internal ADC test read GP26 (Aileron): ");
    Serial.println(test);
    return true;
}
#endif

static uint8_t voltage_to_percent(float v) {
    if (v <= TX_BATT_EMPTY_V) return 0;
    if (v >= TX_BATT_FULL_V) return 100;
    return (uint8_t)constrain(((v - TX_BATT_EMPTY_V) / (TX_BATT_FULL_V - TX_BATT_EMPTY_V)) * 100.0f, 0.0f, 100.0f);
}

static uint8_t percent_to_bars(uint8_t pct) {
    if (pct >= 90) return 5;
    if (pct >= 70) return 4;
    if (pct >= 50) return 3;
    if (pct >= 30) return 2;
    if (pct >= 10) return 1;
    return 0;
}

// ==========================================
// BQ2562X Battery & Charging Status (TX battery)
// ==========================================
static const char* get_charge_status_string(enum bq2562x_charge_status status) {
    switch (status) {
        case BQ2562X_CHG_STAT_NOT_CHARGING:
            return "NChg";
        case BQ2562X_CHG_STAT_CC_MODE:
            return "CC";
        case BQ2562X_CHG_STAT_CV_MODE:
            return "CV";
        case BQ2562X_CHG_STAT_TOPOFF_ACTIVE:
            return "Top";
        default:
            return "---";
    }
}

static const char* get_vbus_status_string(enum bq2562x_vbus_status status) {
    switch (status) {
        case BQ2562X_VBUS_STAT_NO_INPUT:
            return "NoVBUS";
        case BQ2562X_VBUS_STAT_USB_SDP:
            return "SDP";
        case BQ2562X_VBUS_STAT_USB_CDP:
            return "CDP";
        case BQ2562X_VBUS_STAT_USB_DCP:
            return "DCP";
        case BQ2562X_VBUS_STAT_UNKNOWN_ADAPTER:
            return "Unk";
        case BQ2562X_VBUS_STAT_NON_STANDARD_ADAPTER:
            return "NonStd";
        case BQ2562X_VBUS_STAT_HVDCP:
            return "HVDCP";
        case BQ2562X_VBUS_STAT_OTG_MODE:
            return "OTG";
        default:
            return "---";
    }
}

static void update_tx_battery(void) {
    if (millis() - last_tx_batt_read < TX_BATT_READ_INTERVAL) return;
    last_tx_batt_read = millis();

    // Read battery voltage using one-shot ADC conversion
    uint16_t voltage_mv = 0;
    int ret = bq2562x_get_battery_voltage_oneshot(&voltage_mv);
    
    if (ret == 0 && voltage_mv > 0) {
        tx_batt_voltage = voltage_mv / 1000.0f;  // Convert mV to volts
        tx_batt_percent = voltage_to_percent(tx_batt_voltage);
        tx_batt_bars = percent_to_bars(tx_batt_percent);
        tx_batt_low = (tx_batt_percent <= TX_BATT_LOW_PCT);
        tx_batt_valid = true;
        tx_batt_raw_last = voltage_mv;
        
        Serial.print("[BQ2562X] Voltage: ");
        Serial.print(tx_batt_voltage, 3);
        Serial.print("V (");
        Serial.print(voltage_mv);
        Serial.print("mV) Percent: ");
        Serial.print(tx_batt_percent);
        Serial.println("%");
    } else {
        tx_batt_valid = false;
        static unsigned long lastFailLog = 0;
        if (millis() - lastFailLog > 10000) {
            Serial.print("[BQ2562X] Voltage read failed: ");
            Serial.println(ret);
            lastFailLog = millis();
        }
    }
    
    // Read charging status
    enum bq2562x_charge_status chg_stat;
    ret = bq2562x_get_charge_status(&chg_stat);
    if (ret == 0) {
        tx_charge_status = chg_stat;
        tx_charge_status_valid = true;
    } else {
        tx_charge_status_valid = false;
        chg_stat = tx_charge_status;  // Use last known status for comparison
    }
    
    // Read VBUS status
    enum bq2562x_vbus_status vbus_stat;
    ret = bq2562x_get_vbus_status(&vbus_stat);
    if (ret == 0) {
        tx_vbus_status = vbus_stat;
    } else {
        vbus_stat = tx_vbus_status;  // Use last known status for comparison
    }
    
    // Log status changes
    static enum bq2562x_charge_status last_chg_stat = BQ2562X_CHG_STAT_NOT_CHARGING;
    static enum bq2562x_vbus_status last_vbus_stat = BQ2562X_VBUS_STAT_NO_INPUT;
    if (chg_stat != last_chg_stat || vbus_stat != last_vbus_stat) {
        Serial.print("[BQ2562X] Status: ");
        Serial.print(get_charge_status_string(chg_stat));
        Serial.print(" VBUS: ");
        Serial.println(get_vbus_status_string(vbus_stat));
        last_chg_stat = chg_stat;
        last_vbus_stat = vbus_stat;
    }
}

// ==========================================
// Buzzer - Boot Jingle
// ==========================================
static void play_boot_jingle(void) {
    // buzz the led thrice 
    for (int i = 0; i < 3; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(500);
        digitalWrite(BUZZER_PIN, LOW);
        delay(500);
    }
    Serial.println("Boot jingle played");
}
