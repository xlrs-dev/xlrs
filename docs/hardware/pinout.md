# Pinout

Pin defaults are PlatformIO build flags in [platformio.ini](../../platformio.ini).

## CRSF UART

| Signal | Default |
| --- | --- |
| UART TX | GP8 |
| UART RX | GP9 |
| Baud | 400000 |

TX uses this UART for CRSF handset input and telemetry back to the handset. RX
uses it for CRSF RC and link-stat output to the flight controller.

The `rc-rp2350` handset firmware uses the same UART pins for CRSF output to the
TX module.

## SX1280

| Signal | Default |
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

The TX status LED defaults to GP25. The RX status LED defaults to GP10 and is
active high.

## TX/RX Module Power

`tx_lora_pico` and `rx_lora_pico` share one hardware design (`tx rx pcb.pdf`,
`RP2040_new_V1.3`). Confirmed from that schematic:

- `VCC_5V` is a single net fed by **both** the USB-C connector (`PWR1` VBUS)
  and the CRSF connector's 5V pin, with no diode-OR and no bulk/backup
  capacitor between them — only `C24` (4.7 uF) on the 3.3V LDO input.
- The 3.3V rail (`U2`, `LDL112PV33R`) is enabled from `VCC_5V` via a 10k
  pull-up on `EN`, so it's live whenever either 5V source is present.
- In normal flight use the board is powered through the CRSF connector's 5V
  pin (handset battery on TX, FC/drone battery on RX); USB is an alternate
  bench-power path onto the *same* net, not a separate rail.
- Because there's no isolation or backup capacitance, switching power
  sources (e.g. unplugging USB while relying on CRSF-side 5V, or the
  reverse) without both being live at the same instant causes a brief gap on
  `VCC_5V` and a full MCU reset. This is a hardware characteristic, not a
  firmware bug — see the symptom-matrix entry in
  [docs/troubleshooting/symptom-matrix.md](../troubleshooting/symptom-matrix.md).

## RC RP2350 Handset Inputs

The `rc-rp2350` bring-up firmware currently reads four ADC inputs and four
3-position switches:

| Function | Default |
| --- | --- |
| Aileron ADC | GP26 |
| Elevator ADC | GP27 |
| Rudder ADC | GP28 |
| Throttle ADC | GP29 |
| Switch A | GP1, GP2 |
| Switch B | GP3, GP6 |
| Switch C | GP7, GP10 |
| Switch D | GP11, GP12 |
| Power button | GP22 |
| Power I2C SDA | GP4 |
| Power I2C SCL | GP5 |

## RC Handset Power

| Signal | Default |
| --- | --- |
| Power/QON button | GP22, active low |
| Power I2C SDA | GP4 |
| Power I2C SCL | GP5 |
| BQ2562X I2C address | `0x6A` |

`rc-rp2350` can gate boot on a held power button via `RC_SKIP_POWER_ON_SEQUENCE=0`.
Long-press power-off uses the same GPIO as the POWMAN wake source and requests
BQ2562X ship mode when the charger is present.

## Override Example

Edit or extend the relevant environment in `platformio.ini`:

```ini
build_flags =
    ${lora_pico_base.build_flags}
    -DLORA_RX_ROLE=1
    -DDEFAULT_BINDING_PHRASE=\"your-phrase\"
    -DCRSF_UART_TX_PIN=8
    -DCRSF_UART_RX_PIN=9
    -DSTATUS_LED_PIN=10
```
