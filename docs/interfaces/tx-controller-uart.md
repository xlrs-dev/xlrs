# TX Controller UART

The default TX-side controller protocol is a framed UART protocol:

```text
[sync 0xA5][payload_length][message_type][payload bytes...][crc8]
```

CRC is CRC8 with polynomial `0xD5`, calculated over:

```text
[payload_length][message_type][payload bytes...]
```

Maximum payload length is 60 bytes.

Default UART:

| Signal | Default |
| --- | --- |
| UART TX | GP8 |
| UART RX | GP9 |
| Baud | 420000 |

`UART_MSG_CHANNELS` carries 8 controller channel values expected around
1000-2000 us by the app layer. The TX app copies all 8 channels into the RF
mailbox, and the link layer masks values to 11 bits before OTA packing.

See [index.md](index.md) for the complete current interface reference.

For controller-facing CRSF, build with `-DXLRS_TX_CONTROLLER_PROTOCOL=CRSF` and
see [tx-controller-crsf.md](tx-controller-crsf.md).
