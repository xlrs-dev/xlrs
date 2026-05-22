// HwTimer target-specific implementation for RP2040 using Pico SDK.
#include "timing/HwTimer.h"

#if defined(ARDUINO) || defined(PICO_BOARD)
#include <pico/time.h>
#include <hardware/timer.h>

namespace xlrs {

class PicoHwTimer : public HwTimer {
public:
    PicoHwTimer() = default;
    ~PicoHwTimer() override { stop(); }

    bool begin(uint32_t intervalUs, TimerTickCallback onTick) override {
        _intervalUs = intervalUs;
        _onTick = onTick;
        _running = true;

        // Create an alarm pool bound to the calling core (Core 1).
        // Using alarm_pool_create_with_unused_hardware_alarm is safe because it automatically
        // allocates an available hardware timer.
        _pool = alarm_pool_create_with_unused_hardware_alarm(16);
        if (!_pool) {
            return false;
        }

        // Schedule the first alarm to fire in intervalUs.
        // alarm_pool_add_alarm_in_us returns the alarm_id.
        _alarmId = alarm_pool_add_alarm_in_us(_pool, _intervalUs, alarmCallback, this, true);
        return _alarmId > 0;
    }

    void setIntervalUs(uint32_t intervalUs) override {
        _intervalUs = intervalUs;
    }

    uint32_t nowUs() const override {
        return (uint32_t)time_us_64();
    }

    void stop() override {
        if (_running) {
            _running = false;
            if (_pool && _alarmId > 0) {
                alarm_pool_cancel_alarm(_pool, _alarmId);
                _alarmId = 0;
            }
            if (_pool) {
                alarm_pool_destroy(_pool);
                _pool = nullptr;
            }
        }
    }

private:
    static int64_t alarmCallback(alarm_id_t id, void* user_data) {
        auto* self = static_cast<PicoHwTimer*>(user_data);
        if (!self->_running) {
            return 0;
        }

        if (self->_onTick) {
            self->_onTick();
        }

        // Return the positive microsecond interval to schedule relative to the previous
        // scheduled fire time. This prevents cumulative scheduling drift.
        return (int64_t)self->_intervalUs;
    }

    uint32_t _intervalUs = 4000;
    TimerTickCallback _onTick = nullptr;
    bool _running = false;
    alarm_pool_t* _pool = nullptr;
    alarm_id_t _alarmId = 0;
};

HwTimer* createHwTimer() {
    return new PicoHwTimer();
}

} // namespace xlrs

#else

// Host-native stub for unit tests. In the host-native test environment,
// timing is driven synchronously by the simulation tick loop.
namespace xlrs {

static uint32_t s_simNowUs = 0;
static uint32_t s_simIntervalUs = 4000;

void setSimulatedTimeUs(uint32_t us) {
    s_simNowUs = us;
}

uint32_t getSimulatedIntervalUs() {
    return s_simIntervalUs;
}

class DummyHwTimer : public HwTimer {
public:
    bool begin(uint32_t intervalUs, TimerTickCallback onTick) override {
        s_simIntervalUs = intervalUs;
        (void)onTick;
        return true;
    }
    void setIntervalUs(uint32_t intervalUs) override {
        s_simIntervalUs = intervalUs;
    }
    uint32_t nowUs() const override {
        return s_simNowUs;
    }
    void stop() override {}
};

HwTimer* createHwTimer() {
    return new DummyHwTimer();
}

} // namespace xlrs

#endif
