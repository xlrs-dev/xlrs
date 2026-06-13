#ifdef PLATFORM_RP2040

#include "hwTimer.h"
#include "ArduinoCompat.h"

#include <pico/time.h>
#include <hardware/timer.h>

void (*hwTimer::callbackTick)() = nullptr;
void (*hwTimer::callbackTock)() = nullptr;

volatile bool hwTimer::running = false;
volatile bool hwTimer::isTick = false;

volatile uint32_t hwTimer::HWtimerInterval = TimerIntervalUSDefault;
volatile int32_t hwTimer::PhaseShift = 0;
volatile int32_t hwTimer::FreqOffset = 0;

namespace {

alarm_pool_t* s_pool = nullptr;
alarm_id_t s_alarmId = 0;
bool s_inited = false;

int64_t alarmTrampoline(alarm_id_t id, void* user_data) {
    (void)id;
    (void)user_data;
    if (!hwTimer::running) {
        return 0;
    }

    if (hwTimer::isTick) {
#if !defined(TARGET_TX)
        if (hwTimer::callbackTick) {
            hwTimer::callbackTick();
        }
#endif
    } else {
        if (hwTimer::callbackTock) {
            hwTimer::callbackTock();
        }
    }
    hwTimer::isTick = !hwTimer::isTick;

    int32_t half = (int32_t)(hwTimer::HWtimerInterval >> 1);
#if !defined(TARGET_RX)
    (void)half;
#else
    if (!hwTimer::isTick) {
        half += hwTimer::PhaseShift;
        hwTimer::PhaseShift = 0;
    }
    half += hwTimer::FreqOffset;
#endif
    if (half < 50) {
        half = 50;
    }
    return half;
}

} // namespace

void hwTimer::init(void (*cbTick)(), void (*cbTock)()) {
    if (s_inited) {
        return;
    }
    callbackTick = cbTick;
    callbackTock = cbTock;
    s_pool = alarm_pool_create_with_unused_hardware_alarm(8);
    s_inited = s_pool != nullptr;
}

void hwTimer::stop() {
    running = false;
    if (s_pool && s_alarmId > 0) {
        alarm_pool_cancel_alarm(s_pool, s_alarmId);
        s_alarmId = 0;
    }
}

void hwTimer::resume() {
#if defined(TARGET_RX)
    isTick = false;
#endif
    running = true;
    if (!s_pool) {
        return;
    }
    if (s_alarmId > 0) {
        alarm_pool_cancel_alarm(s_pool, s_alarmId);
    }
    isTick = true;
    s_alarmId = alarm_pool_add_alarm_in_us(
        s_pool, HWtimerInterval >> 1, alarmTrampoline, nullptr, true);
    if (callbackTock) {
        callbackTock();
    }
}

void hwTimer::updateInterval(uint32_t newTimerInterval) {
    HWtimerInterval = newTimerInterval;
}

void hwTimer::phaseShift(int32_t newPhaseShift) {
    const int32_t minVal = -(int32_t)(HWtimerInterval >> 2);
    const int32_t maxVal = (int32_t)(HWtimerInterval >> 2);
    PhaseShift = constrain(newPhaseShift, minVal, maxVal);
}

void hwTimer::callback() {
    // Unused on RP2040 — alarmTrampoline implements tick/tock inline.
}

#endif
