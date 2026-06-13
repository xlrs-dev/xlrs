#pragma once

#include <stdint.h>

unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);

// Arduino-style min/max/constrain used throughout ELRS.
template<typename T>
constexpr T elrs_min(T a, T b) { return (a < b) ? a : b; }
template<typename T>
constexpr T elrs_max(T a, T b) { return (a > b) ? a : b; }

#ifndef min
#define min(a, b) elrs_min((a), (b))
#endif
#ifndef max
#define max(a, b) elrs_max((a), (b))
#endif

template<typename T>
constexpr T constrain(T val, T lo, T hi) {
    return elrs_min(elrs_max(val, lo), hi);
}
