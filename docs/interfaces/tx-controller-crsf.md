# TX Controller CRSF

The TX firmware can be built with a controller-facing CRSF port:

```bash
cmake -S . -B build -G Ninja -DXLRS_TX_CONTROLLER_PROTOCOL=CRSF
cmake --build build --target xlrs_tx
```

Default UART pins and baud match the existing controller UART:

| Signal | Default |
| --- | --- |
| UART TX | GP8 |
| UART RX | GP9 |
| Baud | 420000 |

Implemented today:

- `CRSF_FRAMETYPE_RC_CHANNELS_PACKED` input from the controller.
- Channels 1-8 are translated from CRSF channel units to 1000-2000 us and copied
  into the TX RF mailbox.
- CRSF `LINK_STATISTICS` output from TX to the controller, built from internal
  `link_stats`.

Current limitations:

- The default TX controller protocol is still the custom UART protocol unless
  `XLRS_TX_CONTROLLER_PROTOCOL=CRSF` is selected at configure time.
- CRSF device discovery, parameter read/write, Lua configuration, and bind/config
  commands are not implemented yet.
- XLRS currently carries 8 OTA `rc_channel` values. CRSF channels 9-16 are parsed
  by the CRSF decoder but are not transmitted over the XLRS uplink.

See [index.md](index.md) for the complete current interface reference.
