// RC-CRSF: Remote control that outputs CRSF over serial to TX side.
// Handles display, battery, analog inputs, UI - everything except radio comms.
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>

#include "crsfSerial.h"
#include "crsf_protocol.h"

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

// Menu navigation buttons (active high)
#define ENTER_BUTTON_PIN 18
#define BACK_BUTTON_PIN 13

// Toggle switches (active low)
#define TOGGLE_SWITCH1_PIN 28
#define TOGGLE_SWITCH2_PIN 21
#define TOGGLE_SWITCH3_PIN 3
#define TOGGLE_SWITCH4_PIN 9

// BQ25620 charger / fuel (ADC) over I2C
#define BQ25620_I2C_ADDR       0x6A  // Scan showed device at 0x6A
#define BQ25620_REG_ADC_CONTROL 0x26
#define BQ25620_REG_ADC_FUNC_DIS0 0x27
#define BQ25620_REG_VBAT_ADC    0x30
#define BQ25620_ADC_ENABLE_BIT  0x80
#define BQ25620_ADC_STEP_V      0.0010f   // Empirically closer for VBAT (was 1.99mV)
#define TX_BATT_EMPTY_V         3.2f
#define TX_BATT_FULL_V          4.2f

// EEPROM addresses
#define EEPROM_SIZE 512
#define CAL_MAGIC_ADDR 150
#define CAL_DATA_ADDR 152
#define CAL_MAGIC_VALUE 0xC5

// Timing
static unsigned long lastCrsfSend = 0;
static const unsigned long CRSF_INTERVAL = 20;  // 50Hz
static unsigned long lastDisplayUpdate = 0;
static const unsigned long DISPLAY_INTERVAL = 200; // ms

// Application state
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_ADS1115 ads;  // ADS1115 ADC on I2C bus
CrsfSerial crsf(Serial2, CRSF_BAUDRATE);

// ADS1115 status flag
bool ads_ok = false;

// Channel values (mapped to CRSF range 1000-2000)
uint16_t channels[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

// Raw input snapshots (for logging/calibration)
int16_t raw_adc[4] = {0};
bool raw_toggles[4] = {false};

// Calibration data for sticks
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

// TX local battery (BQ25620)
static float tx_batt_voltage = 0.0f;
static uint8_t tx_batt_percent = 0;
static uint8_t tx_batt_bars = 0;
static bool tx_batt_valid = false;
static unsigned long last_tx_batt_read = 0;
static const unsigned long TX_BATT_READ_INTERVAL = 5000; // ms
static uint16_t tx_batt_raw_last = 0;

// Forward declarations
static void load_calibration(void);
static bool init_ads1115(void);
static void readInputs();
static void updateDisplay();
static void sendCrsfChannels();
static uint8_t voltage_to_percent(float v);
static uint8_t percent_to_bars(uint8_t pct);
static bool bq25620_begin(void);
static bool bq25620_read_voltage(float &voltage_out, uint16_t &raw_out);
static void update_tx_battery(void);

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

    // Initialize display
    display.begin(I2C_ADDRESS, true);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println("RC-CRSF init...");
    display.display();

    // Initialize ADS1115
    ads_ok = init_ads1115();

    // Initialize BQ25620 ADC for TX battery monitoring
    if (bq25620_begin()) {
        Serial.println("BQ25620 ADC enabled");
    } else {
        Serial.println("BQ25620 init failed");
    }

    // Initialize buttons and toggles
    pinMode(ENTER_BUTTON_PIN, INPUT_PULLDOWN);
    pinMode(BACK_BUTTON_PIN, INPUT_PULLDOWN);
    pinMode(TOGGLE_SWITCH1_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SWITCH2_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SWITCH3_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SWITCH4_PIN, INPUT_PULLUP);

    // Load stick calibration (if present)
    load_calibration();

    // Initialize CRSF serial (outputting to TX side)
    Serial.println("Initializing CRSF serial...");
    Serial2.setTX(CRSF_TX_PIN);
    Serial2.setRX(CRSF_RX_PIN);
    crsf.begin(CRSF_BAUDRATE);
    Serial.print("CRSF on Serial2: TX=GP");
    Serial.print(CRSF_TX_PIN);
    Serial.print(", RX=GP");
    Serial.println(CRSF_RX_PIN);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("RC-CRSF ready");
    display.display();
}

// ============================================================
// Loop
// ============================================================
void loop() {
    readInputs();
    sendCrsfChannels();
    update_tx_battery();
    updateDisplay();
}

// ============================================================
// Input handling
// ============================================================
void readInputs() {
    // Read analog sticks from ADS1115 (16-bit ADC: 0-32767)
    int16_t stick1_x = 16384;  // Center value (32767/2)
    int16_t stick1_y = 16384;
    int16_t stick2_x = 16384;
    int16_t stick2_y = 16384;

    if (ads_ok) {
        stick1_x = ads.readADC_SingleEnded(AILERON_CHANNEL);   // Aileron
        stick1_y = ads.readADC_SingleEnded(ELEVATOR_CHANNEL);  // Elevator
        stick2_x = ads.readADC_SingleEnded(RUDDER_CHANNEL);    // Rudder
        stick2_y = ads.readADC_SingleEnded(THROTTLE_CHANNEL);  // Throttle
    }

    raw_adc[0] = stick1_x;
    raw_adc[1] = stick1_y;
    raw_adc[2] = stick2_x;
    raw_adc[3] = stick2_y;

    // Apply deadzone on ADC (5%) before mapping
    auto applyDeadzone = [](int16_t val, uint16_t vmin, uint16_t vmax, float frac) -> int16_t {
        if (vmax <= vmin + 10) return val;
        float center = (vmin + vmax) * 0.5f;
        float halfRange = (vmax - vmin) * 0.5f;
        float dz = halfRange * frac;
        float delta = val - center;
        if (fabsf(delta) < dz) return (int16_t)center;
        float sign = (delta > 0) ? 1.0f : -1.0f;
        float adjusted = sign * ((fabsf(delta) - dz) / (halfRange - dz)) * halfRange;
        return (int16_t)(center + adjusted);
    };
    const float DEADZONE = 0.05f; // 5%
    stick1_x = applyDeadzone(stick1_x, calib.min[0], calib.max[0], DEADZONE);
    stick1_y = applyDeadzone(stick1_y, calib.min[1], calib.max[1], DEADZONE);
    stick2_x = applyDeadzone(stick2_x, calib.min[2], calib.max[2], DEADZONE);
    // Leave throttle without deadzone to preserve full resolution

    // Map to channel values (1000-2000)
    auto mapCal = [](int16_t val, uint16_t vmin, uint16_t vmax) -> uint16_t {
        if (vmax <= vmin + 10) return 1500; // avoid div0
        return (uint16_t)constrain(map(val, vmin, vmax, 1000, 2000), 1000, 2000);
    };
    auto mapCalInverted = [](int16_t val, uint16_t vmin, uint16_t vmax) -> uint16_t {
        if (vmax <= vmin + 10) return 1500; // avoid div0
        return (uint16_t)constrain(map(val, vmin, vmax, 2000, 1000), 1000, 2000);
    };
    channels[0] = mapCalInverted(stick1_x, calib.min[0], calib.max[0]);  // Aileron (invert)
    channels[1] = mapCal(stick1_y, calib.min[1], calib.max[1]);          // Elevator
    channels[2] = mapCal(stick2_x, calib.min[2], calib.max[2]);          // Rudder
    channels[3] = mapCal(stick2_y, calib.min[3], calib.max[3]);          // Throttle

    // Apply EdgeTX-style dual-rate/expo
    auto applyCurve = [](uint16_t us, float rate, float expo) -> uint16_t {
        float x = (int(us) - 1500) / 500.0f;                  // -1..1
        float x_expo = (1.0f - expo) * x + expo * x * x * x;  // soften center
        float scaled = x_expo * rate;                         // cap max throw
        int out = int(1500 + scaled * 500);
        return (uint16_t)constrain(out, 1000, 2000);
    };
    const float RATE = 0.7f;
    const float EXPO = 0.3f;
    channels[0] = applyCurve(channels[0], RATE, EXPO);
    channels[1] = applyCurve(channels[1], RATE, EXPO);
    channels[3] = applyCurve(channels[3], RATE, EXPO);
    channels[0] = (uint16_t)constrain(channels[0], 1300, 1700);
    channels[1] = (uint16_t)constrain(channels[1], 1300, 1700);

    // Read toggle switches (active low - LOW = switch engaged)
    bool sw1 = digitalRead(TOGGLE_SWITCH1_PIN) == LOW;
    bool sw2 = digitalRead(TOGGLE_SWITCH2_PIN) == LOW;
    bool sw3 = digitalRead(TOGGLE_SWITCH3_PIN) == LOW;
    bool sw4 = digitalRead(TOGGLE_SWITCH4_PIN) == LOW;
    raw_toggles[0] = sw1;
    raw_toggles[1] = sw2;
    raw_toggles[2] = sw3;
    raw_toggles[3] = sw4;

    channels[4] = sw1 ? 2000 : 1000;
    channels[5] = sw2 ? 2000 : 1000;
    channels[6] = sw3 ? 2000 : 1000;
    channels[7] = sw4 ? 2000 : 1000;
}

// ============================================================
// CRSF Output
// ============================================================
void sendCrsfChannels() {
    if (millis() - lastCrsfSend < CRSF_INTERVAL) return;
    lastCrsfSend = millis();

    // Map channels from 1000-2000 us to CRSF 11-bit values (191-1792)
    auto mapToCrsf = [](uint16_t us) -> uint16_t {
        return map(us, 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
    };

    crsf_channels_t crsfChannels = {0};
    crsfChannels.ch0 = mapToCrsf(channels[0]);
    crsfChannels.ch1 = mapToCrsf(channels[1]);
    crsfChannels.ch2 = mapToCrsf(channels[2]);
    crsfChannels.ch3 = mapToCrsf(channels[3]);
    crsfChannels.ch4 = mapToCrsf(channels[4]);
    crsfChannels.ch5 = mapToCrsf(channels[5]);
    crsfChannels.ch6 = mapToCrsf(channels[6]);
    crsfChannels.ch7 = mapToCrsf(channels[7]);

    uint16_t center = CRSF_CHANNEL_VALUE_MID;
    crsfChannels.ch8 = center;
    crsfChannels.ch9 = center;
    crsfChannels.ch10 = center;
    crsfChannels.ch11 = center;
    crsfChannels.ch12 = center;
    crsfChannels.ch13 = center;
    crsfChannels.ch14 = center;
    crsfChannels.ch15 = center;

    crsf.queuePacket(
        CRSF_ADDRESS_FLIGHT_CONTROLLER,
        CRSF_FRAMETYPE_RC_CHANNELS_PACKED,
        &crsfChannels,
        CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE
    );
}

// ============================================================
// Display
// ============================================================
void updateDisplay() {
    if (millis() - lastDisplayUpdate < DISPLAY_INTERVAL) return;
    lastDisplayUpdate = millis();

    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("RC-CRSF  ");
    display.print(tx_batt_percent);
    display.println("%");

    display.print("Ch0/1:");
    display.print(channels[0]);
    display.print("/");
    display.println(channels[1]);
    display.print("Ch2/3:");
    display.print(channels[2]);
    display.print("/");
    display.println(channels[3]);

    display.print("Aux:");
    display.print(channels[4] > 1500 ? "1" : "0");
    display.print(channels[5] > 1500 ? "1" : "0");
    display.print(channels[6] > 1500 ? "1" : "0");
    display.print(channels[7] > 1500 ? "1" : "0");

    display.display();
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
// BQ25620 Helpers (TX battery)
// ==========================================
static bool bq25620_read8(uint8_t reg, uint8_t &val) {
    Wire.beginTransmission(BQ25620_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(BQ25620_I2C_ADDR, (uint8_t)1) != 1) return false;
    val = Wire.read();
    return true;
}

static bool bq25620_write8(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(BQ25620_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool bq25620_begin(void) {
    uint8_t adc_ctrl = 0;
    if (!bq25620_read8(BQ25620_REG_ADC_CONTROL, adc_ctrl)) {
        return false;
    }
    // Enable ADC_EN (bit7)
    adc_ctrl |= BQ25620_ADC_ENABLE_BIT;
    if (!bq25620_write8(BQ25620_REG_ADC_CONTROL, adc_ctrl)) {
        return false;
    }
    // Touch function-disable register to confirm comms (keep default)
    uint8_t dummy;
    bq25620_read8(BQ25620_REG_ADC_FUNC_DIS0, dummy);
    return true;
}

static bool bq25620_read_voltage(float &voltage_out, uint16_t &raw_out) {
    // VBAT ADC is 12-bit in 2 bytes, MSB first
    Wire.beginTransmission(BQ25620_I2C_ADDR);
    Wire.write(BQ25620_REG_VBAT_ADC);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(BQ25620_I2C_ADDR, (uint8_t)2) != 2) return false;
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    uint16_t raw = ((uint16_t)msb << 4) | (lsb >> 4);
    if (raw == 0 || raw == 0x0FFF) {
        return false;
    }
    raw_out = raw;
    voltage_out = raw * BQ25620_ADC_STEP_V; // volts
    return true;
}

static void update_tx_battery(void) {
    if (millis() - last_tx_batt_read < TX_BATT_READ_INTERVAL) return;
    last_tx_batt_read = millis();

    float v = 0.0f;
    uint16_t raw = 0;
    if (bq25620_read_voltage(v, raw)) {
        tx_batt_voltage = v;
        tx_batt_percent = voltage_to_percent(tx_batt_voltage);
        tx_batt_bars = percent_to_bars(tx_batt_percent);
        tx_batt_valid = true;
        tx_batt_raw_last = raw;
        Serial.print("[BQ25620] raw:");
        Serial.print(raw);
        Serial.print(" V:");
        Serial.print(tx_batt_voltage, 3);
        Serial.print(" Pct:");
        Serial.print(tx_batt_percent);
        Serial.println("%");
    } else {
        static unsigned long lastFailLog = 0;
        if (millis() - lastFailLog > 10000) {
            Serial.println("BQ25620 read failed");
            lastFailLog = millis();
        }
    }
}
