#ifndef UNIT_TEST

#include "rc_handset/power/Rp2350Power.h"

#include <Arduino.h>
#include <Wire.h>

#include "rc_power_config.h"
#include "rc_handset/power/Bq2562x.h"
#include "rc_handset/power/PowerButton.h"

#if defined(PICO_RP2350) && PICO_RP2350 && __has_include(<hardware/powman.h>)
#define RC_HANDSET_HAS_POWERMAN 1
#include <hardware/gpio.h>
#include <hardware/powman.h>
#include <pico/error.h>
#endif

namespace rc_handset {
namespace power {

namespace {

class WireBq2562xBus : public Bq2562xBus {
public:
    bool probe(uint8_t address) override {
        Wire.beginTransmission(address);
        return Wire.endTransmission() == 0;
    }

    bool read8(uint8_t address, uint8_t reg, uint8_t& value) override {
        Wire.beginTransmission(address);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) return false;
        if (Wire.requestFrom(address, static_cast<uint8_t>(1)) != 1) return false;
        value = Wire.read();
        return true;
    }

    bool write8(uint8_t address, uint8_t reg, uint8_t value) override {
        Wire.beginTransmission(address);
        Wire.write(reg);
        Wire.write(value);
        return Wire.endTransmission() == 0;
    }

    bool read16(uint8_t address, uint8_t reg, uint16_t& value) override {
        Wire.beginTransmission(address);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) return false;
        if (Wire.requestFrom(address, static_cast<uint8_t>(2)) != 2) return false;
        const uint8_t lsb = Wire.read();
        const uint8_t msb = Wire.read();
        value = static_cast<uint16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
        return true;
    }

    bool write16(uint8_t address, uint8_t reg, uint16_t value) override {
        Wire.beginTransmission(address);
        Wire.write(reg);
        Wire.write(static_cast<uint8_t>(value & 0xFFu));
        Wire.write(static_cast<uint8_t>(value >> 8));
        return Wire.endTransmission() == 0;
    }

    void delayMs(uint32_t ms) override {
        delay(ms);
    }
};

WireBq2562xBus g_bqBus;
Bq2562x g_bq(g_bqBus);
BatterySnapshot g_snapshot{};
bool g_chargerReady = false;
PowerButtonController g_powerButton({
    RC_SKIP_POWER_ON_SEQUENCE == 0,
    RC_POWER_ON_HOLD_MS,
    RC_POWER_OFF_HOLD_MS,
});
uint32_t g_nextBatteryPollMs = 0;
uint8_t g_lastPowerOffProgressBucket = 255;

bool powerButtonPressed() {
    return digitalRead(RC_POWER_BUTTON_PIN) == LOW;
}

void refreshBatterySnapshot(bool logErrors) {
    if (!g_chargerReady) {
        g_snapshot = {};
        return;
    }
    const Bq2562xResult result = g_bq.readSnapshot(g_snapshot);
    if (result != Bq2562xResult::Ok && logErrors) {
        Serial.printf("[BQ] battery/status read failed: %d\n", static_cast<int>(result));
    }
}

void printBatterySnapshotPrefix(const char* prefix) {
    Serial.print(prefix);
    Serial.print(" charger=");
    Serial.print(g_snapshot.chargerPresent ? "present" : "missing");
    Serial.print(" batt=");
    if (g_snapshot.batteryVoltageValid) {
        Serial.print(g_snapshot.batteryVoltageMv);
        Serial.print("mV ");
        Serial.print(g_snapshot.batteryPercent);
        Serial.print('%');
    } else {
        Serial.print("unknown");
    }
    Serial.print(" charge=");
    Serial.print(chargeStatusName(g_snapshot.chargeStatus));
    Serial.print(" vbus=");
    Serial.println(vbusStatusName(g_snapshot.vbusStatus));
}

#if RC_HANDSET_HAS_POWERMAN
[[noreturn]] void enterDeepSleep(uint32_t wakeGpio) {
    Serial.println("[PWR] Entering RP2350 deep sleep (POWMAN)");
    Serial.flush();

    gpio_set_input_enabled(wakeGpio, true);
    gpio_set_pulls(wakeGpio, true, false);
    const uint32_t releaseStartMs = millis();
    while (digitalRead(wakeGpio) == LOW && (millis() - releaseStartMs) < 5000u) {
        delay(1);
    }
    delay(30);

    powman_disable_all_wakeups();
    powman_enable_gpio_wakeup(0, wakeGpio, true, false);

    const powman_power_state sleepState = 0;
    powman_power_state wakeState = POWMAN_POWER_STATE_NONE;
    wakeState = powman_power_state_with_domain_on(wakeState, POWMAN_POWER_DOMAIN_SRAM_BANK1);
    wakeState = powman_power_state_with_domain_on(wakeState, POWMAN_POWER_DOMAIN_SRAM_BANK0);
    wakeState = powman_power_state_with_domain_on(wakeState, POWMAN_POWER_DOMAIN_XIP_CACHE);
    wakeState = powman_power_state_with_domain_on(wakeState, POWMAN_POWER_DOMAIN_SWITCHED_CORE);
    if (!powman_configure_wakeup_state(sleepState, wakeState)) {
        Serial.println("[PWR] powman_configure_wakeup_state failed");
        Serial.flush();
    }
    powman_set_debug_power_request_ignored(true);
    const int result = powman_set_power_state(sleepState);
    if (result != PICO_OK) {
        Serial.printf("[PWR] powman_set_power_state failed: %d\n", result);
        Serial.flush();
    }
    for (;;) {
        __asm volatile("wfi");
    }
}
#else
[[noreturn]] void enterDeepSleep(uint32_t) {
    Serial.println("[PWR] Deep sleep requested without POWMAN support; halting");
    Serial.flush();
    for (;;) delay(1000);
}
#endif

[[noreturn]] void powerOff() {
    uint8_t before = 0;
    uint8_t after = 0;
    const bool readBefore = g_chargerReady &&
                            g_bq.readChargerControl3(before) == Bq2562xResult::Ok;
    const Bq2562xResult shipResult = g_chargerReady
                                         ? g_bq.requestShipModeForPowerOff()
                                         : Bq2562xResult::Io;
    const bool readAfter = g_chargerReady &&
                           g_bq.readChargerControl3(after) == Bq2562xResult::Ok;

    Serial.printf("[PWR] Ship-mode request result=%d charger=%s",
                  static_cast<int>(shipResult),
                  g_chargerReady ? "present" : "missing");
    if (readBefore || readAfter) {
        Serial.printf(" reg18=0x%02X->0x%02X", readBefore ? before : 0, readAfter ? after : 0);
    }
    Serial.println();
    Serial.flush();
    enterDeepSleep(RC_POWER_BUTTON_PIN);
}

}  // namespace

void rcPowerBegin() {
    pinMode(RC_POWER_BUTTON_PIN, INPUT_PULLUP);
    delay(2);

    g_powerButton.resetBootGate(millis());
    if (RC_SKIP_POWER_ON_SEQUENCE == 0) {
        Serial.printf("[PWR] Hold GP%u low for %lums to boot\n",
                      static_cast<unsigned>(RC_POWER_BUTTON_PIN),
                      static_cast<unsigned long>(RC_POWER_ON_HOLD_MS));
        uint8_t lastBucket = 255;
        for (;;) {
            const PowerButtonUpdate update = g_powerButton.updateBootGate(powerButtonPressed(), millis());
            const uint8_t bucket = static_cast<uint8_t>(update.progressPercent / 10u);
            if (bucket != lastBucket) {
                lastBucket = bucket;
                Serial.printf("[PWR] boot_hold=%u%%\n", update.progressPercent);
            }
            if (update.event == PowerButtonEvent::BootHoldSatisfied) {
                Serial.println("[PWR] Boot hold complete");
                break;
            }
            if (update.event == PowerButtonEvent::BootHoldAborted) {
                Serial.println("[PWR] Boot hold released; sleeping");
                Serial.flush();
                enterDeepSleep(RC_POWER_BUTTON_PIN);
            }
            delay(20);
        }
    }

    Wire.setSDA(RC_POWER_I2C_SDA_PIN);
    Wire.setSCL(RC_POWER_I2C_SCL_PIN);
    Wire.begin();
    Wire.setClock(RC_POWER_I2C_HZ);

    const Bq2562xResult result = g_bq.begin();
    g_chargerReady = result == Bq2562xResult::Ok;
    if (g_chargerReady) {
        Serial.println("[BQ] BQ2562X charger initialized");
        refreshBatterySnapshot(true);
        printBatterySnapshotPrefix("[BQ]");
    } else {
        g_snapshot = {};
        Serial.printf("[BQ] BQ2562X missing or not ready (%d); continuing without charger\n",
                      static_cast<int>(result));
    }
    g_nextBatteryPollMs = millis() + RC_POWER_BATTERY_POLL_MS;
}

void rcPowerService() {
    const uint32_t nowMs = millis();
    const PowerButtonUpdate update = g_powerButton.updatePowerOff(powerButtonPressed(), nowMs);
    if (update.powerUiActive) {
        const uint8_t bucket = static_cast<uint8_t>(update.progressPercent / 10u);
        if (bucket != g_lastPowerOffProgressBucket ||
            update.event == PowerButtonEvent::PowerOffReady) {
            g_lastPowerOffProgressBucket = bucket;
            Serial.printf("[PWR] power_off_hold=%u%% state=%s\n",
                          update.progressPercent,
                          update.powerOffState == PowerOffState::ReadyToRelease
                              ? "release_to_confirm"
                              : "holding");
        }
    } else if (g_lastPowerOffProgressBucket != 255) {
        g_lastPowerOffProgressBucket = 255;
    }
    if (update.event == PowerButtonEvent::PowerOffCancelled) {
        Serial.println("[PWR] Power-off cancelled");
    } else if (update.event == PowerButtonEvent::PowerOffConfirmed) {
        Serial.println("[PWR] Power-off confirmed");
        powerOff();
    }

    if (g_chargerReady && static_cast<int32_t>(nowMs - g_nextBatteryPollMs) >= 0) {
        refreshBatterySnapshot(false);
        g_nextBatteryPollMs = nowMs + RC_POWER_BATTERY_POLL_MS;
    }
}

void rcPowerPrintStatus() {
    printBatterySnapshotPrefix("[PWR]");
}

BatterySnapshot rcPowerSnapshot() {
    return g_snapshot;
}

}  // namespace power
}  // namespace rc_handset

#endif  // UNIT_TEST
