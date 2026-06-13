# Configuration

Configuration for the **active PlatformIO firmware** (`src/`). Build-time options
are flags in [platformio.ini](../../platformio.ini); runtime options are set over
the USB serial CLI and persisted in flash. (The legacy XLRS core under `_legacy/`
used CMake cache variables instead — those do not apply here.)

## 1. Build-time configuration

All flags below are defined per-environment in `platformio.ini`. Override them by
editing that file or layering your own environment.

### Role selection

Exactly one role must be set; it picks the code path in `src/main.cpp`.

- `LORA_TX_ROLE=1` — TX (time master): CRSF in, RC uplink, telemetry listen.
- `LORA_RX_ROLE=1` — RX: RC receive, CRSF + link-stats out, failsafe gating.

### Radio / link

- `DEFAULT_BINDING_PHRASE` (default `"Kikobot-02"`) — fallback binding phrase
  when none is persisted. Derives the Link UID, sync word, FHSS sequence, and
  `uid_check`.
- `RF_FIXED_CHANNEL` (default `-1`) — `-1` enables FHSS across all 40 channels.
  Set a non-negative channel index only for bench/debug fixed-channel operation.

### SX1280 pins

| Flag | Default | Signal |
| --- | --- | --- |
| `SX128X_SPI_SCK` | 18 | SPI clock |
| `SX128X_SPI_MOSI` | 19 | SPI MOSI |
| `SX128X_SPI_MISO` | 16 | SPI MISO |
| `SX128X_SPI_CS` | 17 | Chip select |
| `SX128X_SPI_BUSY` | 20 | Busy |
| `SX128X_SPI_DIO1` | 21 | DIO1 (TX/RX-done IRQ) |
| `SX128X_SPI_RST` | 22 | Reset |
| `SX128X_RXEN` | 14 | RX front-end enable |
| `SX128X_TXEN` | 15 | TX front-end enable |
| `STATUS_LED_PIN` | TX: 25, RX: 10 | Status LED, active high |

On the RX, the active-high GPIO status LED is solid when connected, blinks with a
500 ms period while bound but waiting for uplink, and blinks with a 1 s period
when the RX is disconnected after falling back to default/unbound config.

### CRSF UART

- `CRSF_UART_TX_PIN` (default `8`), `CRSF_UART_RX_PIN` (default `9`).
- `CRSF_UART_BAUD` (default `400000`).

On the TX this UART takes CRSF in from the handset; on the RX it emits CRSF to the
flight controller.

### Diagnostic builds

- `LORA_USB_DIAG=1` (`tx_usb_diag` env) — extra USB diagnostic logging.
- `LORA_CRSF_DIAG=1` (`tx_crsf_diag` env) — CRSF diagnostic logging.

### RC handset (`rc-rp2350`)

Built for the RP2350 board with its own flags, including
`RC_CRSF_SIMPLETX_FRAME_US`, `RC_HANDSET_ENABLE_CORE1`, and
`RC_SKIP_POWER_ON_SEQUENCE`. See [platformio.ini](../../platformio.ini) and
[../rc-rp2350/index.md](../rc-rp2350/index.md).

## 2. Runtime configuration

Set over the USB serial CLI (115200 baud) and persisted in flash as a
`ConfigRecord` (magic `LRL1`, CRC-protected). See
[../build-test-flash.md](../build-test-flash.md) for the full command list.

### Binding

A binding phrase (1–32 characters) derives the link identity. Both ends must use
the same phrase.

```text
bind get                 show phrase, UID, uid_check
bind set <phrase>        set + persist (requires reboot to take effect)
bind clear               revert to DEFAULT_BINDING_PHRASE
```

- **Link UID** — 8 bytes derived from the phrase (`deriveUid`), seeds the FHSS
  sequence and the SX1280 sync word (`syncWordFromUid`).
- **`uid_check`** — 32-bit value (`uidCheck`) stamped into every OTA frame so a
  receiver rejects traffic from any other binding. It is an anti-crosstalk
  identity filter, **not** authentication.

Binding can also be driven over `rc.v1` USB commands and the CRSF binding-control
frame (`0x7D`). See [../crsf/binding.md](../crsf/binding.md).

### Rate

Two rates, defined in the `kRates` table in `src/protocol.cpp`:

| Rate | Packet period | Modulation | Bandwidth | Telemetry |
| --- | --- | --- | --- | --- |
| `L250` | 4000 µs (≈250 Hz) | LoRa SF5 | 812.5 kHz | none |
| `L100` | 40000 µs (≈25 Hz) | LoRa SF6 | 812.5 kHz | every 16th slot (downlink, ≈640 ms) |

Both use coding rate 5 and a FHSS hop interval of 4 ticks. Switch at runtime:

```text
rate L250
rate L100
```

### FHSS

40 channels spanning 2404–2482 MHz at 2 MHz spacing
(`fhssFrequencyMHz(ch) = 2404 + ch*2`). The hop sequence is a UID-seeded
coprime-stride walk (`fhssChannelFor`), so a bound TX/RX hop together. Disable
hopping for the bench with `RF_FIXED_CHANNEL` (build-time).

### Failsafe

The RX stops emitting CRSF RC frames when the uplink is stale (no valid frame
within `kFailsafeTimeoutMs`, default 250 ms), so the flight controller enters
RXLOSS and owns failsafe policy. See [safety-and-failsafe.md](safety-and-failsafe.md).

## See also

- [build-test-flash.md](../build-test-flash.md) — build flags in practice and the CLI.
- [architecture.md](architecture.md) — how these settings drive the link.
- [ota-protocol.md](ota-protocol.md) — frame format and `uid_check`.
- [terminology.md](terminology.md) — naming.
