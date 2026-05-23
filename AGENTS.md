# XLRS — Agent Instructions

XLRS is timing-sensitive radio-link firmware for a TX/RX pair (SX1280, Pico SDK).
Small changes to naming, slot timing, failsafe, or persisted config can have large
effects, so start from the project vocabulary and the tests.

## Build, Test, Lint

Run from the repo root:

```bash
scripts/check-env.sh   # verify the toolchain
scripts/test.sh        # host-native Unity tests; run before and after changes
scripts/build.sh       # firmware build
scripts/lint.sh        # clang-tidy; run when changing C++ logic
```

Flashing and monitoring: `scripts/flash.sh`, `scripts/monitor.sh`,
`scripts/flash-monitor.sh`.

## Where Things Live

| Path | Responsibility |
| --- | --- |
| `apps/tx/`, `apps/rx/` | TX and RX role applications |
| `lib/xlrs/link/` | Link lifecycle (bind/connect/failsafe) + per-tick RF scheduler |
| `lib/xlrs/ota/`, `fhss/`, `timing/`, `phy/`, `crypto/` | OTA frames, hopping, timing, radio PHY, cipher |
| `test/` | Host-native Unity/CMake tests |

Full map: [docs/developer/code-map.md](docs/developer/code-map.md). Module ownership
and architecture: [docs/developer/architecture.md](docs/developer/architecture.md).
Doc index: [docs/index.md](docs/index.md).

## Naming

[docs/developer/terminology.md](docs/developer/terminology.md) is the source of truth
for project vocabulary. Key defaults:

- `TX` and `RX` are device roles only.
- `uplink` and `downlink` are over-the-air directions.
- `UART`, `CRSF`, and `SPI` are wired transports or buses.
- `rf_channel` is a radio frequency slot; `rc_channel` is a stick/switch/control channel.
- `tick` is the scheduler packet counter; `slot` is the action assigned to a tick.
- `frame` is transport-qualified: `ota_frame`, `crsf_frame`, or `uart_frame`.
- `payload` is the data inside a frame, not the whole frame.

Telemetry is direction-qualified: `downlink_telemetry` (RX to TX), `uplink_telemetry`
(TX to RX, if added later), `crsf_link_statistics` (wired RX-to-FC frame), and
`link_stats` (internal health data, not serialized frame bytes).

Do not expand ambiguous short names such as `ch`, `tlm`, `pkt`, `rxStats`, or
`txPacket` into public APIs. Pick names from the glossary.

## Safety-Sensitive Areas

Treat these as safety-sensitive; include tests or a clear validation note when changing:

- Failsafe behavior and CRSF output gating.
- RF scheduler timing, slot selection, FHSS advance, and PFD updates.
- Binding identity, Link UID derivation, sync words, and UID CRC checks.
- RF region tables, TX power caps, and dynamic power policy.
- Flash-backed config schemas and storage offsets.
- OTA frame formats and crypto nonce/counter handling.

When behavior changes, update the matching docs — see the table in
[CONTRIBUTING.md](CONTRIBUTING.md).
