#include "ArduinoCompat.h"

#if defined(XLRS_PICO_SDK) || defined(PICO_BOARD)

#include <map>
#include <utility>

namespace {

struct IsrSlot {
    IsrHandler handler = nullptr;
    int mode = RISING;
};

std::map<int, IsrSlot> g_isrs;

void gpioIrqHandler(uint gpio, uint32_t events) {
    auto it = g_isrs.find((int)gpio);
    if (it == g_isrs.end() || !it->second.handler) {
        return;
    }
    const int mode = it->second.mode;
    if (mode == RISING && !(events & GPIO_IRQ_EDGE_RISE)) {
        return;
    }
    if (mode == FALLING && !(events & GPIO_IRQ_EDGE_FALL)) {
        return;
    }
    it->second.handler();
}

} // namespace

void pinMode(int pin, int mode) {
    if (pin < 0) {
        return;
    }
    gpio_init((uint)pin);
    switch (mode) {
        case INPUT:
            gpio_set_dir((uint)pin, GPIO_IN);
            gpio_disable_pulls((uint)pin);
            break;
        case INPUT_PULLUP:
            gpio_set_dir((uint)pin, GPIO_IN);
            gpio_pull_up((uint)pin);
            break;
        case OUTPUT:
            gpio_set_dir((uint)pin, GPIO_OUT);
            break;
        default:
            break;
    }
}

int digitalRead(int pin) {
    if (pin < 0) {
        return LOW;
    }
    return gpio_get((uint)pin) ? HIGH : LOW;
}

void digitalWrite(int pin, int val) {
    if (pin < 0) {
        return;
    }
    gpio_put((uint)pin, val ? 1u : 0u);
}

void attachInterrupt(int pin, IsrHandler handler, int mode) {
    if (pin < 0 || !handler) {
        return;
    }
    g_isrs[pin] = {handler, mode};
    uint32_t events = GPIO_IRQ_EDGE_RISE;
    if (mode == FALLING) {
        events = GPIO_IRQ_EDGE_FALL;
    } else if (mode == CHANGE) {
        events = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    }
    gpio_set_irq_enabled_with_callback((uint)pin, events, true, gpioIrqHandler);
}

void detachInterrupt(int pin) {
    if (pin < 0) {
        return;
    }
    gpio_set_irq_enabled((uint)pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
    g_isrs.erase(pin);
}

unsigned long micros() {
    return (unsigned long)time_us_64();
}

void delay(unsigned long ms) {
    sleep_ms(ms);
}

void delayMicroseconds(unsigned int us) {
    sleep_us(us);
}

#else

void pinMode(int, int) {}
int digitalRead(int) { return LOW; }
void digitalWrite(int, int) {}
void attachInterrupt(int, IsrHandler, int) {}
void detachInterrupt(int) {}
unsigned long micros() { return 0; }
void delay(unsigned long) {}
void delayMicroseconds(unsigned int) {}

#endif
