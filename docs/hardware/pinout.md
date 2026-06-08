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

The status LED defaults to GP25.

## Override Example

Edit or extend the relevant environment in `platformio.ini`:

```ini
build_flags =
    ${lora_pico_base.build_flags}
    -DLORA_TX_ROLE=1
    -DDEFAULT_BINDING_PHRASE=\"your-phrase\"
    -DCRSF_UART_TX_PIN=8
    -DCRSF_UART_RX_PIN=9
    -DSTATUS_LED_PIN=25
```
