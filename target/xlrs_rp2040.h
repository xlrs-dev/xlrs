#pragma once

// RP2040 + SX1280 target map for future ExpressLRS builds.
// Pin defaults match CMakeLists.txt / xlrs_core compile definitions.

#ifndef PLATFORM_RP2040
#define PLATFORM_RP2040 1
#endif

#define RADIO_SX128X 1
#define Regulatory_Domain_ISM_2400 1

#ifndef TARGET_NAME
#define TARGET_NAME XLRS_RP2040
#endif

#ifndef DEVICE_NAME
#define DEVICE_NAME "XLRS RP2040"
#endif

// SX1280 SPI (spi0)
#ifndef GPIO_PIN_SCK
#define GPIO_PIN_SCK 18
#endif
#ifndef GPIO_PIN_MOSI
#define GPIO_PIN_MOSI 19
#endif
#ifndef GPIO_PIN_MISO
#define GPIO_PIN_MISO 16
#endif
#ifndef GPIO_PIN_NSS
#define GPIO_PIN_NSS 17
#endif
#ifndef GPIO_PIN_BUSY
#define GPIO_PIN_BUSY 20
#endif
#ifndef GPIO_PIN_DIO1
#define GPIO_PIN_DIO1 21
#endif
#ifndef GPIO_PIN_RST
#define GPIO_PIN_RST 22
#endif

// RF front-end switch
#ifndef GPIO_PIN_TX_ENABLE
#define GPIO_PIN_TX_ENABLE 15
#endif
#ifndef GPIO_PIN_RX_ENABLE
#define GPIO_PIN_RX_ENABLE 14
#endif

// Wired CRSF / UART toward controller (TX) or flight controller (RX)
#ifndef GPIO_PIN_RCSIGNAL_TX
#define GPIO_PIN_RCSIGNAL_TX 8
#endif
#ifndef GPIO_PIN_RCSIGNAL_RX
#define GPIO_PIN_RCSIGNAL_RX 9
#endif

#ifndef GPIO_PIN_LED
#define GPIO_PIN_LED 10
#endif

#ifndef POWER_OUTPUT_FIXED
#define POWER_OUTPUT_FIXED 13
#endif

// Flash-time binding phrase (CMake: XLRS_DEFAULT_BINDING_PHRASE)
#ifndef MY_BINDING_PHRASE
#define MY_BINDING_PHRASE "Kikobot-02"
#endif

#if defined(TARGET_TX)
#define XLRS_ELRS_ROLE "TX"
#elif defined(TARGET_RX)
#define XLRS_ELRS_ROLE "RX"
#else
#define XLRS_ELRS_ROLE "HAL"
#endif
