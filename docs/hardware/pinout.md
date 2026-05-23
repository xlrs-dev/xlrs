# Pinout

Pin defaults are CMake cache variables and can be overridden at configure time.

## UART / CRSF

| Signal | Default |
| --- | --- |
| UART TX | GP8 |
| UART RX | GP9 |
| Baud | 420000 |

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

RX status LED defaults to GP13.

## Override Example

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
