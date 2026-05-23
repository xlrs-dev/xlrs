# XLRS Assistant Memory

Use `plans/naming.md` as the source of truth for project terminology.

Important defaults:

- `TX` and `RX` are device roles only.
- `uplink` and `downlink` are over-the-air directions.
- `UART`, `CRSF`, and `SPI` are wired transports or buses.
- `rf_channel` means a radio frequency slot.
- `rc_channel` means a stick/switch/control channel.
- `tick` means the scheduler packet counter.
- `slot` means the scheduled action assigned to a tick.
- `frame` is transport-qualified: `ota_frame`, `crsf_frame`, or `uart_frame`.
- `payload` is the data inside a frame, not the whole frame.

Prefer direction-qualified names for telemetry:

- `downlink_telemetry`: RX to TX over the radio.
- `uplink_telemetry`: TX to RX over the radio, if added later.
- `crsf_link_statistics`: wired CRSF stats frame sent from RX to the flight controller.
- `link_stats`: internal health data, not serialized frame bytes.

When editing code, avoid expanding ambiguous short names such as `ch`, `tlm`,
`pkt`, `rxStats`, or `txPacket` into public APIs. Pick names from the glossary.
