# OTA Protocol

The OTA layer is versioned and type-multiplexed. It carries RC, sync, telemetry,
bind, and MSP-oriented payloads without exposing radio-driver details above the
PHY boundary.

Terminology:

- `ota_frame` is the whole over-the-air frame.
- `payload` is only the data inside the frame.
- `rc_payload` is packed RC channel data.
- `telemetry_payload` is telemetry data inside an OTA telemetry frame.

The detailed frame implementation lives under `lib/xlrs/ota/`.

See [architecture.md](architecture.md), [configuration.md](configuration.md), and
[terminology.md](terminology.md) for current constraints.
