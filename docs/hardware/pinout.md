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
