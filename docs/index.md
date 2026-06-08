# Documentation

The active firmware is a clean PlatformIO RP2040/Pico + SX1280 LoRa TX/RX pair.
It is ELRS-like, but not ExpressLRS OTA compatible.

## Start Here

- [Build, Test, Flash](build-test-flash.md)
- [Getting Started](getting-started.md)
- [Pinout](hardware/pinout.md)

## Current Interfaces

- TX: CRSF handset input on GP8/GP9 UART.
- RX: CRSF RC and link-stat output on GP8/GP9 UART.
- USB serial CLI on both roles for binding phrase, rate, status, and reboot.

Older XLRS/Pico SDK design notes may still exist in this tree for reference, but
they are no longer the active build contract.
