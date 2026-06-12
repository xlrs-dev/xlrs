# Clean LoRa Link

PlatformIO firmware for a paired RP2040/Pico + SX1280 radio link. It is
ELRS-like in shape, but it is not ExpressLRS OTA compatible.

```text
RC handset or CRSF handset -> TX Pico -> SX1280 LoRa/FHSS -> RX Pico -> CRSF flight controller
```

## Quick Start

```bash
scripts/check-env.sh
scripts/test.sh
scripts/build.sh
```

Manual PlatformIO commands:

```bash
pio test -e native
pio run -e tx_lora_pico
pio run -e rx_lora_pico
pio run -e rc-rp2350
```

Flash one Pico at a time in BOOTSEL mode:

```bash
scripts/flash.sh tx
scripts/flash.sh rx
scripts/flash.sh rc
```

Targeted direct flash helpers build and upload one role. Use these when a
single matching board is connected:

```bash
scripts/flash-tx.sh
scripts/flash-rx.sh
scripts/flash-rc.sh
```

When multiple USB serial devices are connected, pass the target port explicitly:

```bash
scripts/flash-tx.sh /dev/cu.usbmodem1101
scripts/flash-rx.sh /dev/cu.usbmodem5
scripts/flash-rc.sh /dev/cu.usbmodem14101
```

The same ports can be supplied through environment variables:

```bash
TX_PORT=/dev/cu.usbmodem1101 scripts/flash-tx.sh
RX_PORT=/dev/cu.usbmodem5 scripts/flash-rx.sh
RC_PORT=/dev/cu.usbmodem14101 scripts/flash-rc.sh
```

Monitor all connected modules in one color-tagged terminal view:

```bash
scripts/monitor-all.sh
```

The monitor probes USB serial ports with `status`, identifies `role=tx`,
`role=rx`, and `role=rc-rp2350`, ignores non-module ports, and prints timestamped
TX/RX/RC logs. It sends `status` every 5 seconds by default. Useful variants:

```bash
scripts/monitor-all.sh /dev/cu.usbmodem1101 /dev/cu.usbmodem5 /dev/cu.usbmodem14101
scripts/monitor-all.sh --status-interval 1
scripts/monitor-all.sh --status-interval 0
scripts/monitor-all.sh --no-clear
```

## Runtime CLI

USB serial runs at 115200 baud on both TX and RX.

```text
bind get
bind set <1..32 byte phrase>
bind clear
rate L250
rate L100
status
reboot
```

Both devices must use the same binding phrase. The phrase derives the UID used
for the LoRa sync word, FHSS sequence, and OTA packet validation.

The `rc-rp2350` handset bring-up firmware has its own USB console:

```text
status
channels
power
reboot
```

It also accepts the implemented `rc.v1` WebUI subset for discovery, state, and
TX binding proxy:

```text
rc.v1 hello
rc.v1 tx_hello
rc.v1 state
rc.v1 binding_set target=tx phrase=<phrase>
```

## PlatformIO Environments

| Environment | Role |
| --- | --- |
| `tx_lora_pico` | CRSF handset input, LoRa uplink, CRSF telemetry back to handset |
| `rx_lora_pico` | LoRa receiver, CRSF RC/link-stat output to flight controller |
| `rc-rp2350` | RP2350 handset bring-up firmware, CRSF output to TX module |
| `native` | Host tests for protocol/config helpers |

The default phrase and pins are build flags in [platformio.ini](platformio.ini).
RC handset architecture and WebUI protocol notes live in
[docs/rc-rp2350](docs/rc-rp2350/index.md).

## Default Pins

| Signal | Pico pin |
| --- | --- |
| CRSF UART TX | GP8 |
| CRSF UART RX | GP9 |
| CRSF baud | 400000 |
| SX1280 SCK | GP18 |
| SX1280 MOSI | GP19 |
| SX1280 MISO | GP16 |
| SX1280 CS | GP17 |
| SX1280 BUSY | GP20 |
| SX1280 DIO1 | GP21 |
| SX1280 RST | GP22 |
| SX1280 RXEN | GP14 |
| SX1280 TXEN | GP15 |

## Notes

- RF is SX1280 LoRa only, with `L250` and `L100` rates.
- FHSS is enabled by default with `RF_FIXED_CHANNEL=-1`. Set that build flag to
  a non-negative channel index only for bench/debug fixed-channel operation.
- OTA frames are plaintext. CRC16 plus the binding-derived `uid_check` reject
  corruption and wrong-link traffic, but they are not authentication or
  encryption.
- RX stops CRSF RC output when uplink is stale so the flight controller can see
  RXLOSS.
- The active firmware is under [src/main.cpp](src/main.cpp) and shared helpers
  are under [include/lora_link/protocol.h](include/lora_link/protocol.h).
