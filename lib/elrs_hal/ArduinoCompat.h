#pragma once

#include <stdint.h>

#if defined(XLRS_PICO_SDK) || defined(PICO_BOARD)
#include <pico/time.h>
#include <hardware/gpio.h>
#include <hardware/sync.h>
#endif

#ifndef INPUT
#define INPUT 0
#endif
#ifndef OUTPUT
#define OUTPUT 1
#endif
#ifndef INPUT_PULLUP
#define INPUT_PULLUP 2
#endif
#ifndef LOW
#define LOW 0
#endif
#ifndef HIGH
#define HIGH 1
#endif
#ifndef RISING
#define RISING 0x01
#endif
#ifndef FALLING
#define FALLING 0x02
#endif
#ifndef CHANGE
#define CHANGE 0x03
#endif
#ifndef MSBFIRST
#define MSBFIRST 1
#endif
#ifndef SPI_MODE0
#define SPI_MODE0 0
#endif

typedef void (*IsrHandler)(void);

void pinMode(int pin, int mode);
int digitalRead(int pin);
void digitalWrite(int pin, int val);
void attachInterrupt(int pin, IsrHandler handler, int mode);
void detachInterrupt(int pin);
unsigned long micros();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
