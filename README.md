# XLRS TX/RX Pico Firmware

Native Raspberry Pi Pico SDK firmware for the XLRS radio modules.

This repository is intentionally limited to the radio link modules:

- `xlrs_tx`: reads channel/control frames from a UART peer and transmits them over SX1280.
- `xlrs_rx`: receives SX1280 frames and emits CRSF to the flight controller.

Handset firmware, RC controller UI, battery-management code, and web configuration tooling are out of scope.

## Layout

| Path | Purpose |
|------|---------|
| `apps/tx/` | TX firmware entry point |
| `apps/rx/` | RX firmware entry point |
| `lib/xlrs/link/` | Link state machine, scheduler, binding, RF config |
| `lib/xlrs/phy/` | Native SX1280 PHY |
| `lib/xlrs/hal/` | Pico SDK hardware adapters |
| `lib/UARTProtocol/` | UART protocol consumed by the TX module |
| `lib/crsfSerial/` | CRSF output support used by the RX module |
| `cmake/` | Pico SDK import helper |

## Hardware

Default UART/CRSF pins:

| Signal | Pin |
|--------|-----|
| TX | GP8 |
| RX | GP9 |
| Baudrate | 420000 |

Default SX1280 pin map:

| Signal | Pin |
|--------|-----|
| SPI SCK | GP18 |
| SPI MOSI | GP19 |
| SPI MISO | GP16 |
| SPI CS | GP17 |
| BUSY | GP20 |
| DIO1 | GP21 |
| RST | GP22 |
| RXEN | GP14 |
| TXEN | GP15 |

Pin defaults live in `CMakeLists.txt` as compile definitions and can be overridden from CMake cache or target definitions as the board layout evolves.

## Build

This workspace has been verified with:

- Pico SDK: `/Users/sawan/projects/pico/pico-sdk`
- Arm toolchain: `/Users/sawan/projects/pico/toolchain/xpack-arm-none-eabi`

Check the local machine before building:

```bash
scripts/check-env.sh
```

Set the paths explicitly if you are not using `scripts/build.sh`:

```bash
export PICO_SDK_PATH=/Users/sawan/projects/pico/pico-sdk
export PICO_TOOLCHAIN_PATH=/Users/sawan/projects/pico/toolchain/xpack-arm-none-eabi
```

Then configure and build:

```bash
cmake -S . -B build -G Ninja -DPICO_BOARD=pico
cmake --build build --target xlrs_tx xlrs_rx
```

Or use the helper:

```bash
scripts/build.sh
```

If you do not have a local SDK checkout yet, CMake can fetch it into `build/`:

```bash
cmake -S . -B build -G Ninja -DPICO_BOARD=pico -DPICO_SDK_FETCH_FROM_GIT=ON
cmake --build build --target xlrs_tx xlrs_rx
```

Build outputs are written under `build/`, including `xlrs_tx.uf2` and `xlrs_rx.uf2`.

## Test

The pure-logic layers (UID/FHSS, OTA packing, AEAD crypto, PFD timing, the link
state machine, scheduler, and telemetry transport) have a host-native test suite
that runs off-device — no Pico SDK or hardware required:

```bash
scripts/test.sh
```

This configures the standalone project in `test/`, fetches the Unity framework,
builds `xlrs_native_tests`, and runs it under CTest. To do it by hand:

```bash
cmake -S test -B build-tests
cmake --build build-tests
ctest --test-dir build-tests --output-on-failure
```

The suite compiles the `lib/xlrs` sources with their native fallbacks selected
(neither `XLRS_PICO_SDK` nor `PICO_BOARD` defined), so the link logic runs
against a simulated clock, in-RAM flash, and a `MockPhy` two-node radio sim.

## Binding

TX and RX must use the same binding phrase. The phrase is hashed into a Link UID used for FHSS seeding, SX1280 sync-word configuration, and sync-frame UID checks.

Override it at configure time:

```bash
cmake -S . -B build -DXLRS_DEFAULT_BINDING_PHRASE="your-phrase"
```

## Runtime

```text
UART peer -> TX module -> SX1280 2.4 GHz link -> RX module -> CRSF -> Flight controller
```

The RX outputs CRSF frames while the link is valid. On failsafe, throttle is driven low and the remaining channels are centered.
