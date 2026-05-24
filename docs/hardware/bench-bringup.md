# Bench Bring-Up

Run local checks before touching hardware:

```bash
scripts/check-env.sh
scripts/test.sh
scripts/lint.sh
scripts/build.sh
```

Flash and monitor both boards:

```bash
scripts/flash.sh tx
scripts/flash.sh rx

TX_PORT=/dev/cu.usbmodem101 RX_PORT=/dev/cu.usbmodem102 scripts/monitor.sh both
```

The TX and RX must report the same computed identity and sync word. If they do
not, fix binding/config before investigating RF behavior.

Expected debug order:

```text
build/config -> boot identity -> radio init -> sync/acquisition
             -> uplink RC -> RX CRSF output -> downlink telemetry
```

See [../troubleshooting/index.md](../troubleshooting/index.md) for the symptom
matrix and detailed flow.

## Status LED (GP10 / Pico pin 13)

Both TX and RX drive a single-color GPIO status LED through
[`lib/xlrs/app/LinkStatusLed.h`](../lib/xlrs/app/LinkStatusLed.h).

| Item | Value |
| --- | --- |
| GPIO | GP10 (Pico physical pin 13) |
| Polarity | Active-low by default (`XLRS_STATUS_LED_ACTIVE_LOW=ON`) |
| Boot pattern | Five slow blinks at power-up |

**Bench note (May 2026):** The pre-migration PlatformIO RX firmware drove a
WS2812 NeoPixel on **GP13** (Pico physical pin **17**), not this LED. If the LED
never toggles after a fresh flash, verify the build picked up GP10 — stale CMake
cache may still have `XLRS_STATUS_LED_PIN=13`. Reconfigure with:

```bash
cmake -S . -B build -G Ninja -DXLRS_STATUS_LED_PIN=10 -DXLRS_STATUS_LED_ACTIVE_LOW=ON
cmake --build build --target xlrs_tx xlrs_rx
```

If GP10 is confirmed in the serial boot line but the LED stays dark, try
`-DXLRS_STATUS_LED_ACTIVE_LOW=OFF` for active-high wiring.
