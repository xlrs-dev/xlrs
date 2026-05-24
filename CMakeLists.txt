cmake_minimum_required(VERSION 3.13)

include(cmake/pico_sdk_import.cmake)

project(xlrs C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

set(XLRS_DEFAULT_BINDING_PHRASE "Kikobot-02" CACHE STRING "Default TX/RX binding phrase")
set(XLRS_TX_CONTROLLER_PROTOCOL "UART" CACHE STRING "TX controller protocol: UART or CRSF")
set_property(CACHE XLRS_TX_CONTROLLER_PROTOCOL PROPERTY STRINGS UART CRSF)
set(XLRS_UART_TX_PIN 8 CACHE STRING "TX firmware UART TX pin toward controller")
set(XLRS_UART_RX_PIN 9 CACHE STRING "TX firmware UART RX pin from controller")
set(XLRS_CRSF_TX_PIN 8 CACHE STRING "RX firmware CRSF TX pin toward flight controller")
set(XLRS_CRSF_RX_PIN 9 CACHE STRING "RX firmware CRSF RX pin from flight controller")
set(XLRS_STATUS_LED_PIN 13 CACHE STRING "RX firmware status LED pin")
set(XLRS_SX128X_SPI_SCK 18 CACHE STRING "SX1280 SPI SCK pin")
set(XLRS_SX128X_SPI_MOSI 19 CACHE STRING "SX1280 SPI MOSI pin")
set(XLRS_SX128X_SPI_MISO 16 CACHE STRING "SX1280 SPI MISO pin")
set(XLRS_SX128X_SPI_CS 17 CACHE STRING "SX1280 SPI chip-select pin")
set(XLRS_SX128X_SPI_BUSY 20 CACHE STRING "SX1280 BUSY pin")
set(XLRS_SX128X_SPI_DIO1 21 CACHE STRING "SX1280 DIO1 pin")
set(XLRS_SX128X_SPI_RST 22 CACHE STRING "SX1280 reset pin")
set(XLRS_SX128X_RXEN 14 CACHE STRING "SX1280 RX enable pin")
set(XLRS_SX128X_TXEN 15 CACHE STRING "SX1280 TX enable pin")
set(XLRS_SX128X_SPI_BAUD 8000000 CACHE STRING "SX1280 SPI baud rate")

add_library(xlrs_core
    lib/crc8/crc8.cpp
    lib/UARTProtocol/UARTProtocol.cpp
    lib/crsfSerial/crsfSerial.cpp
    lib/xlrs/hal/FlashStore.cpp
    lib/xlrs/hal/SerialPort.cpp
    lib/xlrs/link/Link.cpp
    lib/xlrs/link/RfConfig.cpp
    lib/xlrs/link/RfScheduler.cpp
    lib/xlrs/phy/Sx1280NativePhy.cpp
    lib/xlrs/timing/HwTimer.cpp
    lib/xlrs/util/FlashBootCounter.cpp
)

target_include_directories(xlrs_core PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/lib
    ${CMAKE_CURRENT_LIST_DIR}/lib/xlrs
    ${CMAKE_CURRENT_LIST_DIR}/lib/crc8
    ${CMAKE_CURRENT_LIST_DIR}/lib/UARTProtocol
    ${CMAKE_CURRENT_LIST_DIR}/lib/crsfSerial
)

target_compile_definitions(xlrs_core PUBLIC
    XLRS_PICO_SDK=1
    DEFAULT_BINDING_PHRASE=\"${XLRS_DEFAULT_BINDING_PHRASE}\"
    UART_PROTOCOL_TX=${XLRS_UART_TX_PIN}
    UART_PROTOCOL_RX=${XLRS_UART_RX_PIN}
    CRSF_TX_PIN=${XLRS_CRSF_TX_PIN}
    CRSF_RX_PIN=${XLRS_CRSF_RX_PIN}
    STATUS_LED_PIN=${XLRS_STATUS_LED_PIN}
    SX128X_SPI_SCK=${XLRS_SX128X_SPI_SCK}
    SX128X_SPI_MOSI=${XLRS_SX128X_SPI_MOSI}
    SX128X_SPI_MISO=${XLRS_SX128X_SPI_MISO}
    SX128X_SPI_CS=${XLRS_SX128X_SPI_CS}
    SX128X_SPI_BUSY=${XLRS_SX128X_SPI_BUSY}
    SX128X_SPI_DIO1=${XLRS_SX128X_SPI_DIO1}
    SX128X_SPI_RST=${XLRS_SX128X_SPI_RST}
    SX128X_RXEN=${XLRS_SX128X_RXEN}
    SX128X_TXEN=${XLRS_SX128X_TXEN}
    SX128X_SPI_BAUD=${XLRS_SX128X_SPI_BAUD}
)

if(XLRS_TX_CONTROLLER_PROTOCOL STREQUAL "CRSF")
    target_compile_definitions(xlrs_core PUBLIC XLRS_TX_CONTROLLER_CRSF=1)
else()
    target_compile_definitions(xlrs_core PUBLIC XLRS_TX_CONTROLLER_CRSF=0)
endif()

target_link_libraries(xlrs_core PUBLIC
    pico_stdlib
    pico_multicore
    pico_flash
    hardware_flash
    hardware_irq
    hardware_spi
    hardware_sync
    hardware_timer
    hardware_uart
    hardware_watchdog
)

function(add_xlrs_firmware target_name source role_define)
    add_executable(${target_name} ${source})
    target_link_libraries(${target_name} PRIVATE xlrs_core)
    target_compile_definitions(${target_name} PRIVATE ${role_define}=1)
    pico_enable_stdio_usb(${target_name} 1)
    pico_enable_stdio_uart(${target_name} 0)
    pico_add_extra_outputs(${target_name})
endfunction()

add_xlrs_firmware(xlrs_tx apps/tx/main.cpp TX_SIDE)
add_xlrs_firmware(xlrs_rx apps/rx/main.cpp RX_SIDE)
