# XLRS TX/RX Pico Firmware

XLRS is native Raspberry Pi Pico SDK firmware for paired TX/RX radio modules
built around an SX1280 2.4 GHz RF link.

This repository is intentionally limited to the radio bridge:

```text
Controller UART or CRSF -> xlrs_tx -> SX1280 RF link -> xlrs_rx -> CRSF UART -> Flight controller
```

Handset firmware, RC controller UI, model memory, battery-management code, web
configuration tooling, and flight-controller policy are out of scope.

## Quick Start

Check the local environment, run the host-native tests, and build both firmware
images:

```bash
scripts/check-env.sh
scripts/test.sh
scripts/build.sh
```

Flash one board at a time while holding BOOTSEL during plug-in:

```bash
scripts/flash.sh tx
scripts/flash.sh rx
```

Monitor USB CDC logs after flashing:

```bash
TX_PORT=/dev/cu.usbmodem101 RX_PORT=/dev/cu.usbmodem102 scripts/monitor.sh both
```

Expected startup banners:

- TX: `=== XLRS Pico SDK Transmitter ===`
- RX: `=== XLRS Pico SDK Receiver ===`

For the full workflow, see [Build, Test, Flash](docs/build-test-flash.md).

## TX Controller Protocol

The TX firmware supports two controller-facing protocols selected at build time:

| Protocol | Build flag | Status |
| --- | --- | --- |
| Custom UART | `-DXLRS_TX_CONTROLLER_PROTOCOL=UART` | Default |
| CRSF | `-DXLRS_TX_CONTROLLER_PROTOCOL=CRSF` | Supported |

The CRSF build accepts controller RC channels, publishes link statistics,
supports device discovery and RF config parameters, and exposes Bind RX for
unconnected RX pairing. See [CRSF Features](docs/crsf/index.md).

## Repository Layout

| Path | Purpose |
| --- | --- |
| `apps/tx/` | TX firmware entry point |
| `apps/rx/` | RX firmware entry point |
| `lib/xlrs/link/` | Link lifecycle, scheduler, binding, RF config |
| `lib/xlrs/phy/` | Native SX1280 PHY |
| `lib/xlrs/hal/` | Pico SDK hardware adapters |
| `lib/UARTProtocol/` | Controller UART protocol consumed by the TX module |
| `lib/crsfSerial/` | CRSF serial support used by TX controller mode and RX output |
| `test/` | Host-native CMake/CTest suite |
| `scripts/` | Build, test, flash, lint, and monitor helpers |
| `docs/` | Developer, hardware, interface, and troubleshooting docs |

## Hardware At A Glance

Default controller/CRSF UART pins:

| Signal | Pin |
| --- | --- |
| TX | GP8 |
| RX | GP9 |
| Baudrate | 420000 |

Default SX1280 pin map:

| Signal | Pin |
| --- | --- |
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

Pin defaults are CMake cache variables and can be overridden at configure time.
See [Pinout](docs/hardware/pinout.md) for the full list.

## Documentation

- [Documentation Index](docs/index.md)
- [Getting Started](docs/getting-started.md)
- [Build, Test, Flash](docs/build-test-flash.md)
- [Architecture](docs/developer/architecture.md)
- [Terminology](docs/developer/terminology.md)
- [Interfaces](docs/interfaces/index.md)
- [CRSF Features](docs/crsf/index.md)
- [CRSF Binding](docs/crsf/binding.md)
- [Troubleshooting](docs/troubleshooting/index.md)

## Binding

TX and RX must use the same Link UID. The UID can be derived locally from a
shared binding phrase or learned by RX through the CRSF Bind RX OTA workflow.
The Link UID is used for FHSS seeding, SX1280 sync-word configuration, and
sync-frame UID checks.

Override the default phrase at configure time:

```bash
cmake -S . -B build -G Ninja -DXLRS_DEFAULT_BINDING_PHRASE="your-phrase"
cmake --build build --target xlrs_tx xlrs_rx
```

Flash both TX and RX from builds that use the same phrase.

When TX is built with `-DXLRS_TX_CONTROLLER_PROTOCOL=CRSF`, Bind RX can pair an
unconnected RX by sending the TX Link UID during a temporary OTA bind window.
See [CRSF Binding](docs/crsf/binding.md).

## Failsafe

The RX outputs CRSF RC channel frames while the link is valid. With the default
`NoPulses` failsafe mode, RC channel output stops on failsafe so the flight
controller can enter RXLOSS. `Hold` mode is available as an explicit
configuration option.

## Contributing

Start with [CONTRIBUTING.md](CONTRIBUTING.md). In particular, use
[docs/developer/terminology.md](docs/developer/terminology.md) as the source of
truth for naming in code, tests, comments, docs, and review feedback.

## RF Safety

Use antennas or suitable loads before transmitting. Keep bench tests at low
power until the RF path, region configuration, and regulatory constraints are
understood. Firmware-controlled FHSS tables and TX power are part of the
compliance surface.
