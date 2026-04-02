// Minimal SX1280 initialization debug test
#include <Arduino.h>
#include <RadioLib.h>
#include "SX128xLink.h"

// Global variables
Module* module = nullptr;
SX1280* radio = nullptr;
bool radioInitialized = false;
unsigned long lastTestTime = 0;
const unsigned long TEST_INTERVAL_MS = 2000; // Run test every 2 seconds

void printErrorDescription(int16_t state) {
    Serial.print("Error code: ");
    Serial.print(state);
    Serial.print(" - ");
    switch(state) {
        case RADIOLIB_ERR_NONE:
            Serial.println("NONE (success)");
            break;
        case RADIOLIB_ERR_WRONG_MODEM:
            Serial.println("WRONG_MODEM");
            break;
        case RADIOLIB_ERR_INVALID_FREQUENCY:
            Serial.println("INVALID_FREQUENCY");
            break;
        case RADIOLIB_ERR_INVALID_BIT_RATE:
            Serial.println("INVALID_BIT_RATE");
            break;
        case RADIOLIB_ERR_INVALID_OUTPUT_POWER:
            Serial.println("INVALID_OUTPUT_POWER");
            break;
        case RADIOLIB_ERR_PACKET_TOO_LONG:
            Serial.println("PACKET_TOO_LONG");
            break;
        case RADIOLIB_ERR_SPI_WRITE_FAILED:
            Serial.println("SPI_WRITE_FAILED");
            break;
        case RADIOLIB_ERR_SPI_CMD_TIMEOUT:
            Serial.println("SPI_CMD_TIMEOUT");
            break;
        case RADIOLIB_ERR_CHIP_NOT_FOUND:
            Serial.println("CHIP_NOT_FOUND (chip not responding)");
            break;
        default:
            Serial.print("UNKNOWN (check RadioLib docs)");
            break;
    }
}

void performResetSequence() {
    Serial.println("Performing hardware reset sequence...");
    
    // Set RST as output
    pinMode(SX128X_SPI_RST, OUTPUT);
    
    // Pull RST low
    digitalWrite(SX128X_SPI_RST, LOW);
    delay(10);
    
    // Pull RST high
    digitalWrite(SX128X_SPI_RST, HIGH);
    delay(10);
    
    // Wait for BUSY to go low (chip ready)
    Serial.print("Waiting for BUSY to go LOW (chip ready)...");
    unsigned long start = millis();
    while (digitalRead(SX128X_SPI_BUSY) == HIGH && (millis() - start) < 1000) {
        delay(1);
    }
    
    if (digitalRead(SX128X_SPI_BUSY) == LOW) {
        Serial.println(" OK (chip ready)");
    } else {
        Serial.println(" TIMEOUT (chip may not be responding)");
    }
    
    // Set RST back to input (module will control it)
    pinMode(SX128X_SPI_RST, INPUT);
}

bool testSPITransaction() {
    Serial.println("Testing manual SPI transaction (no RadioLib)...");
    
    // Check initial BUSY state
    Serial.print("  Initial BUSY: ");
    Serial.println(digitalRead(SX128X_SPI_BUSY) ? "HIGH" : "LOW");
    
    // If BUSY is HIGH, the chip might be stuck - try a quick reset pulse
    if (digitalRead(SX128X_SPI_BUSY) == HIGH) {
        Serial.println("  BUSY is HIGH - performing quick reset...");
        pinMode(SX128X_SPI_RST, OUTPUT);
        digitalWrite(SX128X_SPI_RST, LOW);
        delay(1);
        digitalWrite(SX128X_SPI_RST, HIGH);
        pinMode(SX128X_SPI_RST, INPUT);
        delay(10);
        
        Serial.print("  BUSY after reset: ");
        Serial.println(digitalRead(SX128X_SPI_BUSY) ? "HIGH" : "LOW");
    }
    
    // Configure CS pin as output (we'll control it manually)
    pinMode(SX128X_SPI_CS, OUTPUT);
    digitalWrite(SX128X_SPI_CS, HIGH); // CS inactive (high)
    delay(10); // Let it settle
    
    // Wait for BUSY to be LOW (chip ready)
    Serial.print("  Step 1: Waiting for BUSY LOW...");
    unsigned long start = millis();
    int busyChecks = 0;
    while (digitalRead(SX128X_SPI_BUSY) == HIGH && (millis() - start) < 500) {
        delay(1);
        busyChecks++;
        if (busyChecks % 100 == 0) {
            Serial.print(".");
        }
    }
    
    if (digitalRead(SX128X_SPI_BUSY) == HIGH) {
        Serial.println(" TIMEOUT");
        Serial.print("  BUSY remained HIGH for ");
        Serial.print(millis() - start);
        Serial.println("ms");
        Serial.println("  Chip BUSY pin stuck HIGH - chip may be in error state");
        return false;
    }
    Serial.print(" OK (waited ");
    Serial.print(millis() - start);
    Serial.println("ms)");
    
    // Test 1: GET_STATUS command (0xC0)
    // This is a simple command that should return status
    Serial.println("  Step 2: Sending GET_STATUS command (0xC0)...");
    
    uint8_t cmd = 0xC0; // GET_STATUS
    uint8_t response = 0;
    
    // Wait for BUSY LOW before transaction
    start = millis();
    while (digitalRead(SX128X_SPI_BUSY) == HIGH && (millis() - start) < 100) {
        delay(1);
    }
    
    if (digitalRead(SX128X_SPI_BUSY) == HIGH) {
        Serial.println("  ERROR: BUSY HIGH before transaction");
        return false;
    }
    
    // Pull CS LOW to start transaction
    digitalWrite(SX128X_SPI_CS, LOW);
    delayMicroseconds(5); // Small delay for CS setup
    
    // Perform SPI transaction
    // Send command, then dummy byte while reading response
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    SPI.transfer(cmd);
    response = SPI.transfer(0x00); // Read status during dummy byte
    SPI.endTransaction();
    
    // Pull CS HIGH to end transaction
    digitalWrite(SX128X_SPI_CS, HIGH);
    delayMicroseconds(5);
    
    Serial.print("  Response: 0x");
    Serial.println(response, HEX);
    
    // Wait for BUSY to go LOW (command processing)
    Serial.print("  Step 3: Waiting for command completion...");
    start = millis();
    while (digitalRead(SX128X_SPI_BUSY) == HIGH && (millis() - start) < 500) {
        delay(1);
    }
    
    if (digitalRead(SX128X_SPI_BUSY) == HIGH) {
        Serial.println(" TIMEOUT");
        Serial.println("  Chip did not complete GET_STATUS command");
        return false;
    }
    Serial.println(" OK");
    
    // Test 2: READ_REGISTER command (0x19) - read chip version
    Serial.println("  Step 4: Reading chip version register (0x0153)...");
    
    uint8_t readCmd = 0x19; // READ_REGISTER (0x19 for SX1280, 0x1D is SX126x)
    uint16_t regAddr = 0x0153; // Firmware version register
    uint8_t regValue = 0;
    
    // Wait for BUSY LOW
    start = millis();
    while (digitalRead(SX128X_SPI_BUSY) == HIGH && (millis() - start) < 100) {
        delay(1);
    }
    
    if (digitalRead(SX128X_SPI_BUSY) == HIGH) {
        Serial.println("  ERROR: BUSY HIGH before register read");
        return false;
    }
    
    // Pull CS LOW
    digitalWrite(SX128X_SPI_CS, LOW);
    delayMicroseconds(5);
    
    // Send READ_REGISTER command + address + dummy byte to read
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    SPI.transfer(readCmd);
    SPI.transfer((regAddr >> 8) & 0xFF); // Address MSB
    SPI.transfer(regAddr & 0xFF);         // Address LSB
    SPI.transfer(0x00);                   // Dummy byte (status)
    regValue = SPI.transfer(0x00);        // Read data
    SPI.endTransaction();
    
    // Pull CS HIGH
    digitalWrite(SX128X_SPI_CS, HIGH);
    delayMicroseconds(5);
    
    Serial.print("  Register value: 0x");
    Serial.println(regValue, HEX);
    
    // Wait for BUSY to go LOW
    Serial.print("  Step 5: Waiting for register read completion...");
    start = millis();
    while (digitalRead(SX128X_SPI_BUSY) == HIGH && (millis() - start) < 500) {
        delay(1);
    }
    
    if (digitalRead(SX128X_SPI_BUSY) == HIGH) {
        Serial.println(" TIMEOUT");
        Serial.println("  Chip did not complete register read");
        return false;
    }
    Serial.println(" OK");
    
    // Evaluate results
    Serial.println("\n  Results:");
    Serial.print("    GET_STATUS response: 0x");
    Serial.println(response, HEX);
    Serial.print("    Chip version (0x0153): 0x");
    Serial.println(regValue, HEX);
    
    // Check final BUSY state
    Serial.print("    Final BUSY state: ");
    Serial.println(digitalRead(SX128X_SPI_BUSY) ? "HIGH" : "LOW");
    
    // Analyze responses
    bool statusOk = (response != 0x00 && response != 0xFF);
    bool regOk = (regValue != 0x00 && regValue != 0xFF);
    
    if (!statusOk) {
        Serial.println("  WARNING: GET_STATUS returned 0x00/0xFF");
        Serial.println("    This typically means:");
        Serial.println("    - Chip not responding on MISO");
        Serial.println("    - SPI communication issue");
        Serial.println("    - Chip in wrong state");
    }
    
    if (!regOk) {
        Serial.println("  WARNING: Register read returned 0x00/0xFF");
        Serial.println("    Chip may not be responding correctly");
    }
    
    if (statusOk && regOk) {
        Serial.println("  SUCCESS: Manual SPI communication working!");
        Serial.print("    Chip version: 0x");
        Serial.println(regValue, HEX);
        return true;
    } else if (digitalRead(SX128X_SPI_BUSY) == LOW) {
        Serial.println("  PARTIAL: BUSY working, but responses invalid");
        Serial.println("    SPI may be partially working (check wiring)");
        return true; // BUSY working is a good sign
    } else {
        Serial.println("  FAILED: Both responses invalid and BUSY stuck");
        return false;
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n=== SX1280 Debug Test ===");
    Serial.println("Pin mappings:");
    Serial.print("  SPI_SCK:  GPIO "); Serial.println(SX128X_SPI_SCK);
    Serial.print("  SPI_MOSI: GPIO "); Serial.println(SX128X_SPI_MOSI);
    Serial.print("  SPI_MISO: GPIO "); Serial.println(SX128X_SPI_MISO);
    Serial.print("  SPI_CS:   GPIO "); Serial.println(SX128X_SPI_CS);
    Serial.print("  BUSY:     GPIO "); Serial.println(SX128X_SPI_BUSY);
    Serial.print("  DIO1:     GPIO "); Serial.println(SX128X_SPI_DIO1);
    Serial.print("  RST:      GPIO "); Serial.println(SX128X_SPI_RST);
    Serial.print("  RXEN:     GPIO "); Serial.println(SX128X_RXEN);
    Serial.print("  TXEN:     GPIO "); Serial.println(SX128X_TXEN);
    Serial.println();
    
    // Initialize SPI
    Serial.println("Initializing SPI...");
    SPI.setSCK(SX128X_SPI_SCK);
    SPI.setTX(SX128X_SPI_MOSI);
    SPI.setRX(SX128X_SPI_MISO);
    SPI.setCS(SX128X_SPI_CS);
    SPI.begin();
    Serial.println("SPI initialized");
    Serial.println();
    
    // Create Module
    Serial.println("Creating Module...");
    module = new Module(SX128X_SPI_CS, SX128X_SPI_DIO1, SX128X_SPI_RST, SX128X_SPI_BUSY);
    if (!module) {
        Serial.println("ERROR: Failed to create Module");
        return;
    }
    Serial.println("Module created");
    
    // Create SX1280
    Serial.println("Creating SX1280...");
    radio = new SX1280(module);
    if (!radio) {
        Serial.println("ERROR: Failed to create SX1280");
        return;
    }
    Serial.println("SX1280 created");
    
    // Configure GPIO pins for reading (set as inputs)
    pinMode(SX128X_SPI_BUSY, INPUT);
    pinMode(SX128X_SPI_DIO1, INPUT);
    pinMode(SX128X_SPI_RST, INPUT);
    pinMode(SX128X_SPI_CS, INPUT);
    pinMode(SX128X_RXEN, INPUT);
    pinMode(SX128X_TXEN, INPUT);
    
    // Check initial BUSY state before reset
    Serial.print("Initial BUSY state: ");
    Serial.println(digitalRead(SX128X_SPI_BUSY) ? "HIGH" : "LOW");
    
    // Perform hardware reset
    performResetSequence();
    
    // Check BUSY immediately after reset
    Serial.print("BUSY after reset: ");
    Serial.println(digitalRead(SX128X_SPI_BUSY) ? "HIGH" : "LOW");
    delay(100); // Wait a bit
    Serial.print("BUSY after 100ms delay: ");
    Serial.println(digitalRead(SX128X_SPI_BUSY) ? "HIGH" : "LOW");
    
    // Test basic SPI transaction
    Serial.println();
    bool spiOk = testSPITransaction();
    Serial.println();
    
    if (!spiOk) {
        Serial.println("WARNING: Basic SPI test failed - chip may not be responding");
        Serial.println("  Check wiring, power, and chip connections");
    }
    
    // Set RF switch pins
    Serial.println("Setting RF switch pins...");
    radio->setRfSwitchPins(SX128X_RXEN, SX128X_TXEN);
    Serial.println("RF switch pins set");
    
    Serial.println("Configuration:");
    Serial.print("  Frequency: "); Serial.print(SX128X_FREQ_MHZ); Serial.println(" MHz");
    Serial.print("  Bitrate: "); Serial.print(SX128X_FLRC_BR_KBPS); Serial.println(" kbps");
    Serial.print("  Output power: "); Serial.print(SX128X_OUTPUT_POWER_DBM); Serial.println(" dBm");
    Serial.println();
    Serial.println("Starting continuous test loop...\n");
}

void loop() {
    unsigned long now = millis();
    
    // Run test every TEST_INTERVAL_MS
    if (now - lastTestTime >= TEST_INTERVAL_MS) {
        lastTestTime = now;
        
        Serial.println("----------------------------------------");
        Serial.print("Test at ");
        Serial.print(now / 1000.0, 1);
        Serial.println(" seconds");
        Serial.println("----------------------------------------");
        
        if (!radio) {
            Serial.println("ERROR: Radio not created");
            Serial.println();
            return;
        }
        
        // Perform reset before each attempt
        performResetSequence();
        
        // Give chip time to stabilize after reset
        delay(50);
        
        // Monitor BUSY state changes
        Serial.print("BUSY immediately after reset: ");
        Serial.println(digitalRead(SX128X_SPI_BUSY) ? "HIGH" : "LOW");
        delay(10);
        Serial.print("BUSY after 10ms: ");
        Serial.println(digitalRead(SX128X_SPI_BUSY) ? "HIGH" : "LOW");
        delay(40);
        Serial.print("BUSY after 50ms total: ");
        Serial.println(digitalRead(SX128X_SPI_BUSY) ? "HIGH" : "LOW");
        
        // Test SPI transaction
        Serial.println("Testing SPI communication...");
        bool spiOk = testSPITransaction();
        if (!spiOk) {
            Serial.println("WARNING: SPI test failed - skipping beginFLRC()");
        }
        Serial.println();
        
        // Try to initialize/reinitialize
        Serial.println("Attempting beginFLRC()...");
        int16_t state = radio->beginFLRC(
            SX128X_FREQ_MHZ,
            SX128X_FLRC_BR_KBPS,
            SX128X_FLRC_CR,
            SX128X_OUTPUT_POWER_DBM,
            SX128X_FLRC_PREAMBLE_BITS,
            SX128X_FLRC_SHAPING
        );
        
        if (state != RADIOLIB_ERR_NONE) {
            Serial.print("FAILED: beginFLRC() returned error - ");
            printErrorDescription(state);
            radioInitialized = false;
            
            // Additional diagnostics for -10 error
            if (state == -10) {
                Serial.println("\nDiagnostics for error -10:");
                Serial.println("  This typically indicates the chip is not responding.");
                Serial.println("  Possible causes:");
                Serial.println("    - Wiring issue (check SPI connections)");
                Serial.println("    - Power issue (check 3.3V supply)");
                Serial.println("    - CS pin not working");
                Serial.println("    - BUSY pin stuck");
                Serial.println("    - Wrong chip or damaged chip");
            }
        } else {
            Serial.println("SUCCESS: beginFLRC() completed");
            radioInitialized = true;
            
            // Read status
            Serial.println("\nRadio Status:");
            int16_t rssi = radio->getRSSI();
            Serial.print("  RSSI: "); Serial.print(rssi); Serial.println(" dBm");
            
            float snr = radio->getSNR();
            Serial.print("  SNR: "); Serial.print(snr); Serial.println(" dB");
        }
        
        // Check GPIO states
        Serial.println("\nGPIO States:");
        Serial.print("  BUSY: "); Serial.print(digitalRead(SX128X_SPI_BUSY) ? "HIGH" : "LOW");
        Serial.println(" (should be LOW when ready, HIGH when busy)");
        Serial.print("  DIO1: "); Serial.print(digitalRead(SX128X_SPI_DIO1) ? "HIGH" : "LOW");
        Serial.println(" (interrupt pin)");
        Serial.print("  RST:  "); Serial.print(digitalRead(SX128X_SPI_RST) ? "HIGH" : "LOW");
        Serial.println(" (should be HIGH for normal operation)");
        Serial.print("  CS:   "); Serial.print(digitalRead(SX128X_SPI_CS) ? "HIGH" : "LOW");
        Serial.println(" (should be HIGH when idle, LOW during SPI transaction)");
        Serial.print("  RXEN: "); Serial.println(digitalRead(SX128X_RXEN) ? "HIGH" : "LOW");
        Serial.print("  TXEN: "); Serial.println(digitalRead(SX128X_TXEN) ? "HIGH" : "LOW");
        
        // Check SPI pins
        Serial.println("\nSPI Pin States:");
        Serial.print("  SCK:  "); Serial.println(digitalRead(SX128X_SPI_SCK) ? "HIGH" : "LOW");
        Serial.print("  MOSI: "); Serial.println(digitalRead(SX128X_SPI_MOSI) ? "HIGH" : "LOW");
        Serial.print("  MISO: "); Serial.println(digitalRead(SX128X_SPI_MISO) ? "HIGH" : "LOW");
        
        Serial.println();
    }
    
    // Blink LED to show we're alive
    static unsigned long lastBlink = 0;
    static bool ledState = false;
    
    if (now - lastBlink > 500) {
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
        lastBlink = now;
    }
}

