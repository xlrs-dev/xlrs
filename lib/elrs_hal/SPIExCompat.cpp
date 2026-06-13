#include "SPIExCompat.h"

#if defined(XLRS_PICO_SDK) || defined(PICO_BOARD)

#include <hardware/spi.h>

namespace {
spi_inst_t* kSpi = spi0;
}

void SPIExClass::begin(int sck, int miso, int mosi, int ss) {
    _sck = sck;
    _miso = miso;
    _mosi = mosi;
    _ss = ss;
    spi_init(kSpi, _baud);
    gpio_set_function((uint)sck, GPIO_FUNC_SPI);
    gpio_set_function((uint)miso, GPIO_FUNC_SPI);
    gpio_set_function((uint)mosi, GPIO_FUNC_SPI);
    pinMode(ss, OUTPUT);
    digitalWrite(ss, HIGH);
    _started = true;
}

void SPIExClass::begin() {
    if (_ss >= 0) {
        begin(_sck, _miso, _mosi, _ss);
    }
}

void SPIExClass::end() {
    _started = false;
}

void SPIExClass::setFrequency(uint32_t hz) {
    _baud = hz;
    if (_started) {
        spi_init(kSpi, _baud);
    }
}

void SPIExClass::transfer(uint8_t csMask, uint8_t* data, uint32_t size, bool reading) {
    (void)csMask;
    if (!_started || _ss < 0 || !data || size == 0) {
        return;
    }
    digitalWrite(_ss, LOW);
    if (reading) {
        spi_write_read_blocking(kSpi, data, data, size);
    } else {
        spi_write_blocking(kSpi, data, size);
    }
    digitalWrite(_ss, HIGH);
}

void SPIExClass::write(uint8_t csMask, uint8_t* data, uint32_t size) {
    transfer(csMask, data, size, false);
}

void SPIExClass::read(uint8_t csMask, uint8_t* data, uint32_t size) {
    transfer(csMask, data, size, true);
}

SPIExClass SPIEx;

#endif
