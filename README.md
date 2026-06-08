# Clean LoRa Link

PlatformIO firmware for a paired RP2040/Pico + SX1280 radio link. It is
ELRS-like in shape, but it is not ExpressLRS OTA compatible.

```text
CRSF handset -> TX Pico -> SX1280 LoRa/FHSS -> RX Pico -> CRSF flight controller
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
```

Flash one Pico at a time in BOOTSEL mode:

```bash
scripts/flash.sh tx
scripts/flash.sh rx
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

## PlatformIO Environments

| Environment | Role |
| --- | --- |
| `tx_lora_pico` | CRSF handset input, LoRa uplink, CRSF telemetry back to handset |
| `rx_lora_pico` | LoRa receiver, CRSF RC/link-stat output to flight controller |
| `native` | Host tests for protocol/config helpers |

The default phrase and pins are build flags in [platformio.ini](platformio.ini).

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
- RX stops CRSF RC output when uplink is stale so the flight controller can see
  RXLOSS.
- The active firmware is under [src/main.cpp](src/main.cpp) and shared helpers
  are under [include/lora_link/protocol.h](include/lora_link/protocol.h).
