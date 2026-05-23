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
| `scripts/` | Build, test, flash, and serial monitor helpers |

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

The RX status LED defaults to GP13.

Pin defaults are CMake cache variables, so custom boards can override them at configure time:

```bash
cmake -S . -B build -G Ninja \
  -DXLRS_UART_TX_PIN=8 \
  -DXLRS_UART_RX_PIN=9 \
  -DXLRS_CRSF_TX_PIN=8 \
  -DXLRS_CRSF_RX_PIN=9 \
  -DXLRS_STATUS_LED_PIN=13 \
  -DXLRS_SX128X_SPI_SCK=18 \
  -DXLRS_SX128X_SPI_MOSI=19 \
  -DXLRS_SX128X_SPI_MISO=16 \
  -DXLRS_SX128X_SPI_CS=17 \
  -DXLRS_SX128X_SPI_BUSY=20 \
  -DXLRS_SX128X_SPI_DIO1=21 \
  -DXLRS_SX128X_SPI_RST=22 \
  -DXLRS_SX128X_RXEN=14 \
  -DXLRS_SX128X_TXEN=15
```

## Build

Requirements:

- CMake
- Ninja
- Python 3
- clang-tidy, available from LLVM (`brew install llvm` on macOS)
- Arm embedded GCC (`arm-none-eabi-gcc`)
- Raspberry Pi Pico SDK
- Optional for flashing: a USB-capable `picotool`

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
PICO_SDK_FETCH_FROM_GIT=ON scripts/build.sh
```

Manual equivalent:

```bash
cmake -S . -B build -G Ninja -DPICO_BOARD=pico -DPICO_SDK_FETCH_FROM_GIT=ON
cmake --build build --target xlrs_tx xlrs_rx
```

Build outputs are written under `build/`, including `xlrs_tx.uf2` and `xlrs_rx.uf2`.

## Flash

Use `scripts/flash.sh` for flashing. It uses a USB-capable `picotool` when available and falls back to copying the UF2 onto the `RPI-RP2` BOOTSEL volume.

Build first:

```bash
scripts/build.sh
```

Flash one board at a time. Hold BOOTSEL while plugging in the Pico, then run:

```bash
scripts/flash.sh tx
scripts/flash.sh rx
```

To flash both, the helper pauses between boards:

```bash
scripts/flash.sh both
```

Force a method if needed:

```bash
FLASH_METHOD=uf2 scripts/flash.sh tx
FLASH_METHOD=picotool scripts/flash.sh rx
```

When using UF2 copy, the helper auto-detects a single mounted `RPI-RP2` volume. If more than one is mounted, pass the volume explicitly:

```bash
TX_UF2_VOLUME=/Volumes/RPI-RP2 scripts/flash.sh tx
RX_UF2_VOLUME=/Volumes/RPI-RP2 scripts/flash.sh rx
```

## Monitor

Diagnostics print over USB CDC stdio. The GP8/GP9 UART is reserved for the controller side on TX and CRSF on RX.

After flashing and rebooting, identify the Pico USB serial ports:

```bash
ls /dev/cu.usbmodem* /dev/tty.usbmodem* 2>/dev/null
```

Monitor one board:

```bash
TX_PORT=/dev/cu.usbmodem101 scripts/monitor.sh tx
RX_PORT=/dev/cu.usbmodem102 scripts/monitor.sh rx
```

Monitor both with prefixed logs:

```bash
TX_PORT=/dev/cu.usbmodem101 RX_PORT=/dev/cu.usbmodem102 scripts/monitor.sh both
```

Build, flash, and monitor in one flow:

```bash
scripts/flash-monitor.sh tx
scripts/flash-monitor.sh rx
scripts/flash-monitor.sh both
```

For `both`, flash TX first, then RX, and provide the serial ports after both boards reboot.

Expected startup banners:

- TX: `=== XLRS Pico SDK Transmitter ===`
- RX: `=== XLRS Pico SDK Receiver ===`

The RX also prints periodic status lines like `[RX STATUS] State: ...` while running.

`scripts/monitor.sh` uses 115200 baud by default. Override with `BAUD=...` if needed.

## Test

The pure-logic layers (UID/FHSS, OTA packing, AEAD crypto, PFD timing, the link
state machine, scheduler, and telemetry transport) have a host-native test suite
that runs off-device, with no Pico SDK or hardware required:

```bash
scripts/test.sh
```

Run clang-tidy against the host-native compile database:

```bash
scripts/lint.sh
```

If `clang-tidy` is not on `PATH`, point the helper at it:

```bash
CLANG_TIDY=/path/to/clang-tidy scripts/lint.sh
```

On Apple Silicon Homebrew installs LLVM as keg-only; the helper auto-detects
`/opt/homebrew/opt/llvm/bin/clang-tidy`.

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

If the build directory already exists, reconfigure both TX and RX from the same build directory after changing the phrase:

```bash
cmake -S . -B build -G Ninja -DXLRS_DEFAULT_BINDING_PHRASE="your-phrase"
cmake --build build --target xlrs_tx xlrs_rx
```

## Runtime

```text
UART peer -> TX module -> SX1280 2.4 GHz link -> RX module -> CRSF -> Flight controller
```

The RX outputs CRSF frames while the link is valid. On failsafe, throttle is driven low and the remaining channels are centered.
