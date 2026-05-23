# SX1280 Wiring

Default SX1280 pins:

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

If PHY timeouts increase:

- Check BUSY is not stuck.
- Check CS/SCK/MOSI/MISO activity during boot.
- Check DIO1 pulses on TX-done/RX-done.
- Check the SX1280 power rail under TX current.
- Check antenna/load before transmitting at higher power.

See [../troubleshooting/index.md](../troubleshooting/index.md) for the full
bring-up sequence.
