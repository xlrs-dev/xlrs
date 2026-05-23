#include "hal/SerialPort.h"

#if defined(XLRS_PICO_SDK)
#include <hardware/gpio.h>

namespace xlrs::hal {

SerialPort::SerialPort(uart_inst_t* uart, uint txPin, uint rxPin)
    : _uart(uart), _txPin(txPin), _rxPin(rxPin) {}

void SerialPort::begin(uint32_t baud) {
    _baud = baud;
    uart_init(_uart, baud);
    gpio_set_function(_txPin, GPIO_FUNC_UART);
    gpio_set_function(_rxPin, GPIO_FUNC_UART);
    uart_set_format(_uart, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(_uart, true);
}

void SerialPort::end() {
    if (_uart) {
        uart_deinit(_uart);
    }
}

int SerialPort::available() const {
    return (_uart && uart_is_readable(_uart)) ? 1 : 0;
}

int SerialPort::read() {
    return (_uart && uart_is_readable(_uart)) ? uart_getc(_uart) : -1;
}

size_t SerialPort::write(uint8_t b) {
    if (!_uart) return 0;
    uart_putc_raw(_uart, b);
    return 1;
}

size_t SerialPort::write(const uint8_t* data, size_t len) {
    if (!_uart || !data) return 0;
    uart_write_blocking(_uart, data, len);
    return len;
}

} // namespace xlrs::hal
#endif
