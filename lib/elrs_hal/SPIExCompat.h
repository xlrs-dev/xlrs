#pragma once

#include "ArduinoCompat.h"
#include <stdint.h>

#define ICACHE_RAM_ATTR
#define WORD_ALIGNED_ATTR __attribute__((aligned(4)))
#define WORD_PADDED(size) (((size) + 3) & ~3)

class SPIExClass {
public:
    void begin(int sck, int miso, int mosi, int ss);
    void begin();
    void end();
    void setFrequency(uint32_t hz);
    void setHwCs(bool) {}
    void setBitOrder(int) {}
    void setDataMode(int) {}
    void setClockDivider(int) {}

    void write(uint8_t csMask, uint8_t* data, uint32_t size);
    void read(uint8_t csMask, uint8_t* data, uint32_t size);

private:
    void transfer(uint8_t csMask, uint8_t* data, uint32_t size, bool reading);
    bool _started = false;
    int _sck = -1;
    int _miso = -1;
    int _mosi = -1;
    int _ss = -1;
    uint32_t _baud = 8000000;
};

extern SPIExClass SPIEx;
