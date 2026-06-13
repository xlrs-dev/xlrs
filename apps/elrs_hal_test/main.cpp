#include "ArduinoCompat.h"
#include "SPIExCompat.h"
#include "logging.h"
#include "phy/IRadioPhy.h"
#include "phy/Sx1280NativePhy.h"
#include "timing/HwTimer.h"
#include "xlrs_rp2040.h"
#include "fhss/fhss_domain.h"

#include <pico/stdlib.h>
#include <stdio.h>
#include <stdlib.h>

namespace {

volatile uint32_t g_dio1Count = 0;
volatile uint32_t g_timerTicks = 0;
volatile uint32_t g_lastTickUs = 0;
volatile int32_t g_jitterMinUs = 999999;
volatile int32_t g_jitterMaxUs = -999999;
volatile int64_t g_jitterSumUs = 0;

void onDio1() {
    g_dio1Count++;
}

void onTimerTick() {
    const uint32_t now = (uint32_t)time_us_64();
    if (g_lastTickUs != 0) {
        const int32_t delta = (int32_t)(now - g_lastTickUs);
        if (delta < g_jitterMinUs) {
            g_jitterMinUs = delta;
        }
        if (delta > g_jitterMaxUs) {
            g_jitterMaxUs = delta;
        }
        g_jitterSumUs += delta;
    }
    g_lastTickUs = now;
    g_timerTicks++;
}

void report(const char* test, bool pass, const char* detail) {
    printf("[ELRS-HAL] %s %s", pass ? "PASS" : "FAIL", test);
    if (detail && detail[0]) {
        printf(" — %s", detail);
    }
    printf("\n");
}

bool testGpioSwitch() {
    pinMode(GPIO_PIN_TX_ENABLE, OUTPUT);
    pinMode(GPIO_PIN_RX_ENABLE, OUTPUT);
    digitalWrite(GPIO_PIN_TX_ENABLE, HIGH);
    digitalWrite(GPIO_PIN_RX_ENABLE, LOW);
    sleep_ms(1);
    const bool txHigh = digitalRead(GPIO_PIN_TX_ENABLE) == HIGH;
    const bool rxLow = digitalRead(GPIO_PIN_RX_ENABLE) == LOW;
    digitalWrite(GPIO_PIN_TX_ENABLE, LOW);
    digitalWrite(GPIO_PIN_RX_ENABLE, HIGH);
    sleep_ms(1);
    const bool txLow = digitalRead(GPIO_PIN_TX_ENABLE) == LOW;
    const bool rxHigh = digitalRead(GPIO_PIN_RX_ENABLE) == HIGH;
    const bool ok = txHigh && rxLow && txLow && rxHigh;
    report("rf_switch", ok, ok ? "TX/RX enable pins toggle" : "pin readback mismatch");
    digitalWrite(GPIO_PIN_RX_ENABLE, HIGH);
    digitalWrite(GPIO_PIN_TX_ENABLE, LOW);
    return ok;
}

bool testSpiEx() {
    SPIEx.begin(GPIO_PIN_SCK, GPIO_PIN_MISO, GPIO_PIN_MOSI, GPIO_PIN_NSS);
    SPIEx.setFrequency(8000000);
    uint8_t buf[4] = {0x00, 0x00, 0x00, 0x00};
    SPIEx.read(1, buf, sizeof(buf));
    report("spi_ex", true, "SPIEx transfer completed");
    return true;
}

bool testRadioInit(xlrs::Sx1280NativePhy& phy) {
    xlrs::PhyConfig cfg{};
    cfg.freqMHz = xlrs::fhssInitialSyncFreqMHz();
    cfg.modulation = xlrs::Modulation::Lora;
    cfg.bwKHz = 800.0f;
    cfg.sf = 8;
    cfg.cr = 4;
    cfg.powerDbm = 13;
    cfg.syncWord = 0x1424;
    cfg.payloadLen = 8;

    const bool ok = phy.init(cfg);
    report("radio_init", ok, ok ? "SX1280 init LoRa" : "init returned false");
    return ok;
}

bool testRadioRxArm(xlrs::Sx1280NativePhy& phy) {
    phy.setOnRxDone(onDio1);
    phy.startRx(xlrs::fhssInitialSyncFreqMHz());
    sleep_ms(500);
    const bool ok = phy.healthy();
    char detail[64];
    snprintf(detail, sizeof(detail), "healthy=%d dio1=%lu", ok ? 1 : 0,
             (unsigned long)g_dio1Count);
    report("radio_rx_arm", ok, detail);
    return ok;
}

bool testTimerJitter() {
    xlrs::HwTimer* timer = xlrs::createHwTimer();
    if (!timer || !timer->begin(4000, onTimerTick)) {
        report("timer", false, "HwTimer begin failed");
        delete timer;
        return false;
    }
    sleep_ms(4100);
    timer->stop();
    delete timer;

    const uint32_t ticks = g_timerTicks;
    const bool ok = ticks >= 900 && g_jitterMinUs > 3500 && g_jitterMaxUs < 4500;
    char detail[96];
    snprintf(detail, sizeof(detail), "ticks=%lu jitter min=%ld max=%ld avg=%ld",
             (unsigned long)ticks, (long)g_jitterMinUs, (long)g_jitterMaxUs,
             ticks > 1 ? (long)(g_jitterSumUs / (ticks - 1)) : 0L);
    report("timer_jitter", ok, detail);
    return ok;
}

} // namespace

int main() {
    stdio_init_all();
    sleep_ms(1500);

    printf("[ELRS-HAL] Phase 1 proof harness role=%s ELRS=%s\n",
           XLRS_ELRS_ROLE, "3.5.6");
    printf("[ELRS-HAL] pins NSS=%d BUSY=%d DIO1=%d TXEN=%d RXEN=%d CRSF TX/RX=%d/%d\n",
           GPIO_PIN_NSS, GPIO_PIN_BUSY, GPIO_PIN_DIO1,
           GPIO_PIN_TX_ENABLE, GPIO_PIN_RX_ENABLE,
           GPIO_PIN_RCSIGNAL_TX, GPIO_PIN_RCSIGNAL_RX);

    uint32_t pass = 0;
    uint32_t total = 0;

    total++;
    if (testGpioSwitch()) {
        pass++;
    }
    total++;
    if (testSpiEx()) {
        pass++;
    }

    xlrs::Sx1280NativePhy phy;
    total++;
    if (testRadioInit(phy)) {
        pass++;
    }
    total++;
    if (testRadioRxArm(phy)) {
        pass++;
    }
    total++;
    if (testTimerJitter()) {
        pass++;
    }

    printf("[ELRS-HAL] summary %lu/%lu passed\n", (unsigned long)pass, (unsigned long)total);
    return pass == total ? 0 : 1;
}
