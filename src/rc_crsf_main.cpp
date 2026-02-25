// RC-CRSF: Remote control that outputs CRSF over serial to TX side.
// Handles display, battery, analog inputs, UI - everything except radio comms.
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#if !defined(INTERNAL_ADC)
#include <Adafruit_ADS1X15.h>
#endif
#include <math.h>

#include "UARTProtocol.h"
#include "bq2562x.h"
#include "RCConfig.h"
#include "RCConfigProtocol.h"

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
static unsigned long lastEnterPress = 0;
static unsigned long lastBackPress = 0;
static unsigned long lastPowerPress = 0;
static const unsigned long BUTTON_DEBOUNCE_MS = 200;

// Device status flags
bool ads_ok = false;
bool display_ok = false;
bool tx_connected = false;  // TX board connection status

// Channel values (mapped to CRSF range 1000-2000)
uint16_t channels[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

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
    {2917, 2917, 2917, 2917},
    {23420, 23420, 23420, 23420},
    {13199, 13199, 13199, 13199}
};
static bool calib_loaded = false;

// Runtime config (USB WebUI): loaded from EEPROM or defaults; used by readInputs when wired
static rc_config_data_t g_rc_config;
static rc_config_data_t g_rc_config_draft;
static RCConfigProtocol g_rc_proto;
static unsigned long lastStateStreamMs = 0;
static const unsigned long STATE_STREAM_INTERVAL_MS = 50;

// Calibration via WebUI: sampling state
static bool cal_sampling = false;
static int16_t cal_axis_min[4];
static int16_t cal_axis_max[4];

// High-pass filter state (1st-order, per-axis) for ADC drift reduction
static float hpf_prev_in[4] = {0};
static float hpf_prev_out[4] = {0};
static bool hpf_initialized = false;
#define RC_ADC_CENTER 16384
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
static bool rc_proto_enter_pairing_mode(void);

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
    pinMode(ENTER_BUTTON_PIN, INPUT_PULLDOWN);
    pinMode(BACK_BUTTON_PIN, INPUT_PULLDOWN);
    pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);  // Active LOW button, defaults HIGH
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
    g_rc_proto.setDeviceInfo("RC-CRSF", "1.1");
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
    g_rc_proto.setEnterPairingMode(rc_proto_enter_pairing_mode);

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
    Serial.print("UART Protocol on Serial2: TX=GP");
    Serial.print(CRSF_TX_PIN);
    Serial.print(", RX=GP");
    Serial.println(CRSF_RX_PIN);
    
    // Check TX connection - send initial ping after a short delay
    Serial.println("Checking TX board connection...");
    delay(100);  // Give UART time to stabilize
    lastPingSent = 0;  // Reset to allow immediate ping
    uartProto.sendPing();  // Send initial ping immediately
    lastPingSent = millis();
    Serial.println("[TX] Initial ping sent");
    check_tx_connection();  // Check initial status

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
}

// ============================================================
// Input handling (runtime config: g_rc_config)
// ============================================================
void readInputs() {
    const rc_config_data_t* cfg = &g_rc_config;
    int16_t axis_adc[4] = {16384, 16384, 16384, 16384};

    if (ads_ok) {
#if defined(INTERNAL_ADC)
        uint32_t a0 = analogRead(INTERNAL_ADC_AILERON_PIN);
        uint32_t a1 = analogRead(INTERNAL_ADC_ELEVATOR_PIN);
        uint32_t a2 = analogRead(INTERNAL_ADC_RUDDER_PIN);
        uint32_t a3 = analogRead(INTERNAL_ADC_THROTTLE_PIN);
        axis_adc[0] = (int16_t)((a0 * INTERNAL_ADC_SCALE_16BIT) / INTERNAL_ADC_MAX);
        axis_adc[1] = (int16_t)((a1 * INTERNAL_ADC_SCALE_16BIT) / INTERNAL_ADC_MAX);
        axis_adc[2] = (int16_t)((a2 * INTERNAL_ADC_SCALE_16BIT) / INTERNAL_ADC_MAX);
        axis_adc[3] = (int16_t)((a3 * INTERNAL_ADC_SCALE_16BIT) / INTERNAL_ADC_MAX);
#else
        axis_adc[0] = ads.readADC_SingleEnded(AILERON_CHANNEL);
        axis_adc[1] = ads.readADC_SingleEnded(ELEVATOR_CHANNEL);
        axis_adc[2] = ads.readADC_SingleEnded(RUDDER_CHANNEL);
        axis_adc[3] = ads.readADC_SingleEnded(THROTTLE_CHANNEL);
#endif
    }

    // Optional high-pass filter on ADC (reduces DC drift; center-referenced)
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
            axis_adc[i] = (int16_t)constrain(v, 0, 32767);
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
    auto applyCurve = [](uint16_t us, float rate, float expo) -> uint16_t {
        float x = (int(us) - 1500) / 500.0f;
        float x_expo = (1.0f - expo) * x + expo * x * x * x;
        float scaled = x_expo * rate;
        return (uint16_t)constrain((int)(1500 + scaled * 500), RC_CHANNEL_MIN, RC_CHANNEL_MAX);
    };

    uint16_t axis_us[4];
    for (int i = 0; i < 4; i++) {
        int16_t dz = applyDeadzone(axis_adc[i], cfg->calib_min[i], cfg->calib_max[i], cfg->deadzone[i]);
        axis_us[i] = mapToUs(dz, cfg->calib_min[i], cfg->calib_max[i], cfg->invert[i] != 0);
        axis_us[i] = applyCurve(axis_us[i], cfg->rate[i], cfg->expo[i]);
    }

    for (int c = 0; c < 4; c++) channels[c] = RC_CHANNEL_MID;
    for (int a = 0; a < 4; a++) {
        uint8_t fc = cfg->channel_function[a];
        if (fc < 4) channels[fc] = axis_us[a];
    }
    for (int c = 0; c < 4; c++) {
        channels[c] = (uint16_t)constrain(channels[c], cfg->cutoff_min[c], cfg->cutoff_max[c]);
    }

    // Read toggle switches (dual-pin reading)
    // Logic: IN1 high = 1000, IN2 high = 2000, both low = 1500
    auto readToggle = [](uint8_t in1_pin, uint8_t in2_pin) -> uint16_t {
        bool in1_high = digitalRead(in1_pin) == HIGH;
        bool in2_high = digitalRead(in2_pin) == HIGH;
        
        if (in1_high && !in2_high) {
            return 1000;  // IN1 high, IN2 low
        } else if (!in1_high && in2_high) {
            return 2000;  // IN1 low, IN2 high
        } else {
            return 1500;  // Both low (or both high - treat as center)
        }
    };
    
    channels[4] = readToggle(TOGGLE_SWITCH1_IN1_PIN, TOGGLE_SWITCH1_IN2_PIN);
    channels[5] = readToggle(TOGGLE_SWITCH2_IN1_PIN, TOGGLE_SWITCH2_IN2_PIN);
    channels[6] = readToggle(TOGGLE_SWITCH3_IN1_PIN, TOGGLE_SWITCH3_IN2_PIN);
    channels[7] = readToggle(TOGGLE_SWITCH4_IN1_PIN, TOGGLE_SWITCH4_IN2_PIN);
    for (int c = 4; c < 8; c++) {
        channels[c] = (uint16_t)constrain(channels[c], cfg->cutoff_min[c], cfg->cutoff_max[c]);
    }

    // Store raw toggle states for logging (using IN1 state as primary indicator)
    raw_toggles[0] = digitalRead(TOGGLE_SWITCH1_IN1_PIN) == HIGH;
    raw_toggles[1] = digitalRead(TOGGLE_SWITCH2_IN1_PIN) == HIGH;
    raw_toggles[2] = digitalRead(TOGGLE_SWITCH3_IN1_PIN) == HIGH;
    raw_toggles[3] = digitalRead(TOGGLE_SWITCH4_IN1_PIN) == HIGH;
}

// ============================================================
// UART Protocol - Channel Output
// ============================================================
void sendChannels() {
    if (millis() - lastChannelSend < CHANNEL_INTERVAL) return;
    lastChannelSend = millis();

    uartProto.sendChannels(channels);
}

// ============================================================
// UART Protocol - Message Reception
// ============================================================
void receiveUARTMessages() {
    uartProto.loop();
    
    // Check telemetry timeout
    if (telemetryValid && (millis() - lastTelemetryUpdate) > TELEMETRY_TIMEOUT_MS) {
        telemetryValid = false;
    }
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
// Button Handling for Commands
// ============================================================
void handleButtons() {
    bool enterPressed = digitalRead(ENTER_BUTTON_PIN) == HIGH;
    bool backPressed = digitalRead(BACK_BUTTON_PIN) == HIGH;
    bool powerPressed = digitalRead(POWER_BUTTON_PIN) == LOW;  // Active LOW button
    
    // ENTER button - PAIR command
    if (enterPressed && !enterButtonPressed && (millis() - lastEnterPress) > BUTTON_DEBOUNCE_MS) {
        enterButtonPressed = true;
        lastEnterPress = millis();
        uartProto.sendCommand(UART_MSG_CMD_PAIR);
        Serial.println("[RC] Sending PAIR command");
    } else if (!enterPressed) {
        enterButtonPressed = false;
    }
    
    // BACK button - STATUS_REQ command
    if (backPressed && !backButtonPressed && (millis() - lastBackPress) > BUTTON_DEBOUNCE_MS) {
        backButtonPressed = true;
        lastBackPress = millis();
        uartProto.sendCommand(UART_MSG_CMD_STATUS_REQ);
        Serial.println("[RC] Sending STATUS_REQ command");
    } else if (!backPressed) {
        backButtonPressed = false;
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

    const char* linkStateNames[] = {"DISC", "PAIR", "CONN", "OK", "LOST"};
    int y = 0;

    // Row 0: Title + link indicator (top-right)
    display.setCursor(COL_0, y);
    display.print("RC-CRSF");
    drawLinkIndicator(LINK_INDICATOR_X, LINK_INDICATOR_Y, tx_connected);
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
    if (tx_connected) {
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
    if (millis() - lastPingSent < PING_INTERVAL) return;
    lastPingSent = millis();
    
    static unsigned long lastPingLog = 0;
    if (millis() - lastPingLog > 5000) {  // Log every 5 seconds to avoid spam
        Serial.print("[TX] Sending ping (TX connected: ");
        Serial.print(tx_connected ? "YES" : "NO");
        Serial.println(")...");
        lastPingLog = millis();
    }
    
    uartProto.sendPing();
}

bool check_tx_connection(void) {
    // Check if we've received a pong recently
    unsigned long timeSincePong = (lastPongReceived > 0) ? (millis() - lastPongReceived) : PONG_TIMEOUT_MS + 1;
    
    bool wasConnected = tx_connected;
    tx_connected = (timeSincePong < PONG_TIMEOUT_MS);
    
    if (wasConnected != tx_connected) {
        if (tx_connected) {
            Serial.println("[TX] Device connected");
        } else {
            Serial.println("[TX] Device disconnected or not responding");
            Serial.print("[TX] Last pong received: ");
            if (lastPongReceived > 0) {
                Serial.print(timeSincePong);
                Serial.println("ms ago");
            } else {
                Serial.println("never");
            }
        }
    }
    
    // Log initial status check
    static bool initialCheckDone = false;
    if (!initialCheckDone) {
        initialCheckDone = true;
        if (tx_connected) {
            Serial.println("[TX] Initial check: Device connected");
        } else {
            Serial.println("[TX] Initial check: Device not responding (waiting for pong...)");
        }
    }
    
    return tx_connected;
}

// ============================================================
// USB config protocol callbacks
// ============================================================
static const rc_config_data_t* rc_proto_get_config(void) {
    return &g_rc_config;
}
static void rc_proto_set_draft(const rc_config_data_t* draft) {
    if (draft) memcpy(&g_rc_config_draft, draft, sizeof(g_rc_config_draft));
}
static void rc_proto_apply(void) {
    memcpy(&g_rc_config, &g_rc_config_draft, sizeof(g_rc_config));
    for (int i = 0; i < 4; i++) {
        calib.min[i] = g_rc_config.calib_min[i];
        calib.max[i] = g_rc_config.calib_max[i];
        calib.center[i] = g_rc_config.calib_center[i];
    }
}
static bool rc_proto_save(void) {
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
        calib.min[i] = g_rc_config.calib_min[i];
        calib.max[i] = g_rc_config.calib_max[i];
        calib.center[i] = g_rc_config.calib_center[i];
    }
    rc_config_save(&g_rc_config);
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
    if (!tx_connected || !phrase || len == 0 || len > 32) return false;
    bool sent = uartProto.sendCommandWithPayload(UART_MSG_CMD_SET_BIND_TX, (const uint8_t*)phrase, len);
    if (sent) {
        Serial.println("[RC] Forwarded TX binding phrase update command to TX");
    } else {
        Serial.println("[RC] Failed to forward TX binding phrase command to TX");
    }
    return sent;
}

static void rc_proto_get_link_status(uint8_t* txConnected, uint8_t* txPaired, uint8_t* txState) {
    if (txConnected) *txConnected = tx_connected ? 1 : 0;
    if (txPaired) *txPaired = statusValid ? status.pairingState : 0;
    if (txState) *txState = statusValid ? status.connectionState : 0xFF;
}

static bool rc_proto_enter_pairing_mode(void) {
    if (!tx_connected) return false;
    bool sent = uartProto.sendCommand(UART_MSG_CMD_PAIR);
    if (sent) {
        Serial.println("[RC] Forwarded PAIR command to TX");
    } else {
        Serial.println("[RC] Failed to forward PAIR command to TX");
    }
    return sent;
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
            calib.min[i] = 2917;
            calib.max[i] = 23420;
        }
        if (calib.center[i] == 0 || calib.center[i] >= calib.max[i]) {
            calib.center[i] = 13199;
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
    // No explicit init needed; analogRead() initializes ADC on first use.
    // Prime the ADC with a read to avoid first-sample skew
    (void)analogRead(INTERNAL_ADC_AILERON_PIN);
    (void)analogRead(INTERNAL_ADC_ELEVATOR_PIN);
    (void)analogRead(INTERNAL_ADC_RUDDER_PIN);
    (void)analogRead(INTERNAL_ADC_THROTTLE_PIN);
    int16_t test = (int16_t)((analogRead(INTERNAL_ADC_AILERON_PIN) * INTERNAL_ADC_SCALE_16BIT) / INTERNAL_ADC_MAX);
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
