#pragma once

#include <stddef.h>
#include <stdint.h>

#if defined(XLRS_PICO_SDK)
#include <hardware/uart.h>
#endif

namespace xlrs::hal {

class SerialPort {
public:
#if defined(XLRS_PICO_SDK)
    SerialPort(uart_inst_t* uart, uint txPin, uint rxPin);
#endif

    void begin(uint32_t baud);
    void end();
    int available() const;
    int read();
    size_t write(uint8_t b);
    size_t write(const uint8_t* data, size_t len);

private:
#if defined(XLRS_PICO_SDK)
    uart_inst_t* _uart = nullptr;
    uint _txPin = 0;
    uint _rxPin = 0;
    uint32_t _baud = 0;
#endif
};

} // namespace xlrs::hal
