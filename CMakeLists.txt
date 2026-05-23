cmake_minimum_required(VERSION 3.13)

include(cmake/pico_sdk_import.cmake)

project(xlrs C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

set(XLRS_DEFAULT_BINDING_PHRASE "Kikobot-02" CACHE STRING "Default TX/RX binding phrase")

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
    SX128X_SPI_SCK=18
    SX128X_SPI_MOSI=19
    SX128X_SPI_MISO=16
    SX128X_SPI_CS=17
    SX128X_SPI_BUSY=20
    SX128X_SPI_DIO1=21
    SX128X_SPI_RST=22
    SX128X_RXEN=14
    SX128X_TXEN=15
)

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
