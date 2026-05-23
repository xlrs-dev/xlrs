# Build, Test, Flash

## Requirements

- CMake
- Ninja
- Python 3
- clang-tidy, available from LLVM (`brew install llvm` on macOS)
- Arm embedded GCC (`arm-none-eabi-gcc`)
- Raspberry Pi Pico SDK
- Optional for flashing: a USB-capable `picotool`

Check the local machine:

```bash
scripts/check-env.sh
```

## Build

Use the helper:

```bash
scripts/build.sh
```

Or configure and build manually:

```bash
cmake -S . -B build -G Ninja -DPICO_BOARD=pico
cmake --build build --target xlrs_tx xlrs_rx
```

If you do not have a local SDK checkout yet, CMake can fetch it into `build/`:

```bash
PICO_SDK_FETCH_FROM_GIT=ON scripts/build.sh
```

Build outputs are written under `build/`, including `xlrs_tx.uf2` and
`xlrs_rx.uf2`.

## Test

The pure-logic layers have a host-native test suite that runs off-device, with
no Pico SDK or hardware required:

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

## Flash

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

## Monitor

Diagnostics print over USB CDC stdio. The GP8/GP9 UART is reserved for the
controller side on TX and CRSF on RX.

Identify the Pico USB serial ports:

```bash
ls /dev/cu.usbmodem* /dev/tty.usbmodem* 2>/dev/null
```

Monitor both boards:

```bash
TX_PORT=/dev/cu.usbmodem101 RX_PORT=/dev/cu.usbmodem102 scripts/monitor.sh both
```

Build, flash, and monitor in one flow:

```bash
scripts/flash-monitor.sh tx
scripts/flash-monitor.sh rx
scripts/flash-monitor.sh both
```
