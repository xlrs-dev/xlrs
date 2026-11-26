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
#include <hardware/watchdog.h>
#include <pico/error.h>
#include <pico/multicore.h>
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
// Smooths the BQ25895's single-shot ADC reading across polls -- one-shot
// conversions on real hardware have real quantization jitter (+-20mV LSB
// steps), which without smoothing shows up as the displayed voltage
// hopping around reading to reading even with the battery perfectly
// steady.
int32_t g_smoothedBatteryMv = 0;
bool g_haveSmoothedBatteryMv = false;
PowerButtonController g_powerButton({
    RC_SKIP_POWER_ON_SEQUENCE == 0,
    RC_POWER_ON_HOLD_MS,
    RC_POWER_OFF_HOLD_MS,
});
uint32_t g_nextBatteryPollMs = 0;
// Non-blocking battery-voltage ADC state machine driven from rcPowerService()
// -- see pollBatterySnapshotNonBlocking() below for why this exists instead
// of just calling refreshBatterySnapshot() every poll tick.
enum class BatteryAdcPhase : uint8_t { Idle, Converting };
BatteryAdcPhase g_batteryAdcPhase = BatteryAdcPhase::Idle;
uint32_t g_nextBatteryTouchMs = 0;  // throttles the whole poll function, not just the ADC
uint32_t g_adcConversionStartMs = 0;
constexpr uint32_t kBatteryAdcPollIntervalMs = 50u;
constexpr uint32_t kBatteryAdcGiveUpMs = 2000u;  // generous vs. the ~1.5s datasheet worst case
uint8_t g_lastPowerOffProgressBucket = 255;
void (*g_sleepHook)() = nullptr;
bool g_powerOffUiActive = false;

bool powerButtonPressed() {
    return digitalRead(RC_POWER_BUTTON_PIN) == LOW;
}

void refreshBatterySnapshot(bool logErrors) {
    if (!g_chargerReady) {
        g_snapshot = {};
        g_haveSmoothedBatteryMv = false;
        return;
    }
    BatterySnapshot fresh{};
    const Bq2562xResult result = g_bq.readSnapshot(fresh);
    if (result != Bq2562xResult::Ok && logErrors) {
        Serial.printf("[BQ] battery/status read failed: %d\n", static_cast<int>(result));
    }

    if (fresh.batteryVoltageValid) {
        if (!g_haveSmoothedBatteryMv) {
            g_smoothedBatteryMv = fresh.batteryVoltageMv;
            g_haveSmoothedBatteryMv = true;
        } else {
            // Light EMA, 1/8 weight per new sample -- converges over ~10-15s,
            // which is plenty responsive for something that drains over
            // hours, in exchange for a visually stable reading.
            g_smoothedBatteryMv += (static_cast<int32_t>(fresh.batteryVoltageMv) - g_smoothedBatteryMv) / 8;
        }
        fresh.batteryVoltageMv = static_cast<uint16_t>(g_smoothedBatteryMv);
        fresh.batteryPercent = batteryPercentFromVoltageMv(fresh.batteryVoltageMv);
    } else {
        g_haveSmoothedBatteryMv = false;
    }
    g_snapshot = fresh;
}

// Non-blocking replacement for the periodic refreshBatterySnapshot() call.
// The BQ25895's one-shot battery-voltage ADC conversion takes up to ~1.5s to
// complete on real hardware (worse observed while actively charging --
// that's the "screen goes laggy on USB power, fine on battery alone"
// symptom this fixes). refreshBatterySnapshot() used to busy-wait for it
// inline via Bq2562x::readBatteryVoltageMv(), which blocked this whole
// core0 loop -- display rendering, button reads, CRSF telemetry -- for that
// entire stretch, once a second. This spreads the wait across
// rcPowerService() ticks instead: kick the conversion off, and poll for
// completion on later ticks with cheap non-blocking register reads.
//
// rcPowerService() (and so this) is called every loop() iteration, far more
// often than any of this needs touching I2C at all -- the whole function
// body (including the charge/VBUS status reads, which don't touch the ADC)
// is throttled to run at most once every kBatteryAdcPollIntervalMs via
// g_nextBatteryTouchMs, so it costs about the same I2C traffic the old
// blocking poll loop did, just spread out non-blockingly instead of stalled
// on inline delay() calls.
void pollBatterySnapshotNonBlocking(uint32_t nowMs) {
    if (!g_chargerReady) {
        g_snapshot = {};
        g_haveSmoothedBatteryMv = false;
        return;
    }
    if (static_cast<int32_t>(nowMs - g_nextBatteryTouchMs) < 0) return;
    g_nextBatteryTouchMs = nowMs + kBatteryAdcPollIntervalMs;

    g_snapshot.chargerPresent = true;
    Bq2562xChargeStatus charge = Bq2562xChargeStatus::NotCharging;
    if (g_bq.readChargeStatus(charge) == Bq2562xResult::Ok) {
        g_snapshot.chargeStatus = Bq2562x::toBatteryChargeStatus(charge);
    }
    Bq2562xVbusStatus vbus = Bq2562xVbusStatus::NoInput;
    if (g_bq.readVbusStatus(vbus) == Bq2562xResult::Ok) {
        g_snapshot.vbusStatus = Bq2562x::toBatteryVbusStatus(vbus);
    }

    if (g_batteryAdcPhase == BatteryAdcPhase::Idle) {
        if (static_cast<int32_t>(nowMs - g_nextBatteryPollMs) < 0) return;
        if (g_bq.startBatteryVoltageConversion() == Bq2562xResult::Ok) {
            g_batteryAdcPhase = BatteryAdcPhase::Converting;
            g_adcConversionStartMs = nowMs;
        } else {
            // Couldn't even trigger it (I2C hiccup) -- try again next tick
            // rather than getting stuck.
            g_nextBatteryPollMs = nowMs + RC_POWER_BATTERY_POLL_MS;
        }
        return;
    }

    // Converting phase: the top-of-function throttle above already caps how
    // often this runs, so no separate gate is needed here.
    if (static_cast<uint32_t>(nowMs - g_adcConversionStartMs) >= kBatteryAdcGiveUpMs) {
        Serial.println("[BQ] battery ADC conversion never completed; retrying");
        g_batteryAdcPhase = BatteryAdcPhase::Idle;
        g_nextBatteryPollMs = nowMs + RC_POWER_BATTERY_POLL_MS;
        return;
    }

    bool done = false;
    if (g_bq.isBatteryVoltageConversionDone(done) != Bq2562xResult::Ok || !done) return;

    uint16_t rawMv = 0;
    if (g_bq.readBatteryVoltageResultMv(rawMv) == Bq2562xResult::Ok) {
        if (!g_haveSmoothedBatteryMv) {
            g_smoothedBatteryMv = rawMv;
            g_haveSmoothedBatteryMv = true;
        } else {
            // Same light EMA as refreshBatterySnapshot() -- see its comment.
            g_smoothedBatteryMv += (static_cast<int32_t>(rawMv) - g_smoothedBatteryMv) / 8;
        }
        g_snapshot.batteryVoltageValid = true;
        g_snapshot.batteryVoltageMv = static_cast<uint16_t>(g_smoothedBatteryMv);
        g_snapshot.batteryPercent = batteryPercentFromVoltageMv(g_snapshot.batteryVoltageMv);
    }
    g_batteryAdcPhase = BatteryAdcPhase::Idle;
    g_nextBatteryPollMs = nowMs + RC_POWER_BATTERY_POLL_MS;
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
    if (g_sleepHook) g_sleepHook();

#if defined(RC_HANDSET_ENABLE_CORE1)
    // Core1 (rc_handset::core1::begin() -> core1Main()) is still spinning in
    // its timing loop here. Halt it before powering down shared domains, or
    // POWMAN sleep entry/wake is unreliable — core1 keeps touching shared
    // state while core0 tries to sleep. Matches the known-working reference
    // firmware, which resets core1 immediately before the same POWMAN calls.
    multicore_reset_core1();
#endif

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
    // Reaching this point means sleep either failed to engage or the SoC woke
    // and resumed here instead of restarting from the reset vector. Either
    // way, don't leave the app hung in a bare wfi loop with everything
    // (Serial, OLED, CRSF) uninitialized: force a clean reboot through the
    // normal boot path so wake reliably re-runs setup().
    Serial.println("[PWR] Sleep did not hold; forcing reboot");
    Serial.flush();
    watchdog_reboot(0, 0, 1);
    for (;;) {
        __asm volatile("wfi");
    }
}
#else
[[noreturn]] void enterDeepSleep(uint32_t) {
    Serial.println("[PWR] Deep sleep requested without POWMAN support; halting");
    Serial.flush();
    if (g_sleepHook) g_sleepHook();
    for (;;) delay(1000);
}
#endif

[[noreturn]] void powerOff() {
    // Do NOT request BQ2562X ship mode here: requestShipModeForPowerOff() opens
    // BATFET immediately (no delay, forced even with VBUS present), cutting the
    // system rail the RP2350 runs on before POWMAN's GPIO wake is armed below.
    // With no confirmed hardware path from the power button into the charger's
    // own ship-mode wake circuit, that leaves the board with no way to power
    // back on. POWMAN deep sleep alone still keeps the RP2350 in a low-power
    // state that reliably wakes on RC_POWER_BUTTON_PIN.
    Serial.printf("[PWR] Power off requested charger=%s\n",
                  g_chargerReady ? "present" : "missing");
    Serial.flush();
    enterDeepSleep(RC_POWER_BUTTON_PIN);
}

}  // namespace

void rcPowerSetSleepHook(void (*hook)()) {
    g_sleepHook = hook;
}

void rcPowerBegin(void (*onBootHoldProgress)(uint8_t percent, bool readyToRelease)) {
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
            const bool readyToRelease = update.bootGateState == BootGateState::ReadyToRelease;
            if (onBootHoldProgress) onBootHoldProgress(update.progressPercent, readyToRelease);
            const uint8_t bucket = static_cast<uint8_t>(update.progressPercent / 10u);
            if (bucket != lastBucket) {
                lastBucket = bucket;
                Serial.printf("[PWR] boot_hold=%u%% state=%s\n", update.progressPercent,
                              readyToRelease ? "release_to_confirm" : "holding");
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

void rcPowerService(void (*onPowerOffProgress)(uint8_t percent, bool readyToRelease, bool blocked),
                    bool rxLinkActive) {
    const uint32_t nowMs = millis();
    const PowerButtonUpdate update = g_powerButton.updatePowerOff(powerButtonPressed(), nowMs, rxLinkActive);
    g_powerOffUiActive = update.powerUiActive;
    const bool blocked = update.event == PowerButtonEvent::PowerOffBlocked;
    if (update.powerUiActive) {
        const bool readyToRelease = update.powerOffState == PowerOffState::ReadyToRelease;
        if (onPowerOffProgress) onPowerOffProgress(update.progressPercent, readyToRelease, blocked);
        const uint8_t bucket = static_cast<uint8_t>(update.progressPercent / 10u);
        if (bucket != g_lastPowerOffProgressBucket ||
            update.event == PowerButtonEvent::PowerOffReady || blocked) {
            g_lastPowerOffProgressBucket = bucket;
            Serial.printf("[PWR] power_off_hold=%u%% state=%s\n",
                          update.progressPercent,
                          blocked ? "blocked_rx_link_active"
                                  : (readyToRelease ? "release_to_confirm" : "holding"));
        }
    } else if (g_lastPowerOffProgressBucket != 255) {
        g_lastPowerOffProgressBucket = 255;
    }
    if (update.event == PowerButtonEvent::PowerOffCancelled) {
        Serial.println("[PWR] Power-off cancelled");
    } else if (update.event == PowerButtonEvent::PowerOffBlocked) {
        Serial.println("[PWR] Power-off blocked: RX link is active");
    } else if (update.event == PowerButtonEvent::PowerOffConfirmed) {
        // rxLinkActive is re-checked inside updatePowerOff() itself, so reaching
        // PowerOffConfirmed already guarantees it was false -- this call is safe.
        Serial.println("[PWR] Power-off confirmed");
        powerOff();
    }

    pollBatterySnapshotNonBlocking(nowMs);
}

bool rcPowerOffHoldActive() {
    return g_powerOffUiActive;
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
