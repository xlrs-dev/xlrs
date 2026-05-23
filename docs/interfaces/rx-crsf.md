# RX CRSF

The RX module outputs CRSF to the flight controller on the CRSF UART.

When the RF link is connected, RX emits packed CRSF RC channels. Channels 1-8
come from the RF link. Channels 9-16 are held at CRSF midpoint.

RX also emits CRSF `LINK_STATISTICS` about every 500 ms.

Failsafe behavior is controlled by RF config:

| Mode | Value | Behavior |
| --- | ---: | --- |
| `NoPulses` | `0` | Stop CRSF RC output when link is not valid |
| `Hold` | `1` | Continue CRSF output during failsafe using preset failsafe channel values |

Default mode is `NoPulses`.

See [index.md](index.md) for the complete current interface reference.
