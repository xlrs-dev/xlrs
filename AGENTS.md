# XLRS — Agent Instructions

XLRS is timing-sensitive radio-link firmware for a TX/RX pair (SX1280, RP2040/Pico).
The firmware that builds and flashes is the **PlatformIO `src/` tree**; the role
(TX or RX) is selected at build time. Small changes to naming, slot timing,
failsafe, or persisted config can have large effects, so start from the project
vocabulary and the tests.

## Build, Test, Lint

Everything runs through PlatformIO. From the repo root:

```bash
scripts/check-env.sh   # verify the toolchain (pio, python3, ...)
scripts/test.sh        # host-native Unity tests (pio test -e native); run before and after changes
scripts/build.sh       # build the tx_lora_pico / rx_lora_pico firmware pair
scripts/lint.sh        # clang-tidy; run when changing C++ logic
```

Flashing and monitoring: `scripts/flash.sh`, `scripts/monitor.sh`,
`scripts/flash-monitor.sh`. Build environments and pin/rate flags are in
[platformio.ini](platformio.ini).

## Where Things Live

| Path | Responsibility |
| --- | --- |
| `src/main.cpp` | TX/RX role application (role selected by `LORA_TX_ROLE` / `LORA_RX_ROLE`) |
| `src/protocol.cpp`, `include/lora_link/protocol.h` | OTA frames, CRC, UID/FHSS, CRSF, RC packing, config |
| `src/lora_link/rf/scheduler.cpp`, `include/lora_link/rf/scheduler.h` | Per-tick slot selection, FHSS advance, connection state, RX PFD |
| `src/rc_handset/`, `include/rc_handset/` | RP2350 RC handset firmware (`rc-rp2350`) |
| `lib/` | Vendored CRSF / UART / CRC helpers |
| `test/` | Host-native Unity tests (`pio test -e native`) |
| `_legacy/` | Earlier XLRS core (`xlrs/`, `apps/`) — reference only, **not built** |

Full map: [docs/developer/code-map.md](docs/developer/code-map.md). Module ownership
and architecture: [docs/developer/architecture.md](docs/developer/architecture.md).
Doc index: [docs/index.md](docs/index.md).

Do not treat `_legacy/` as the build contract: nothing in the active build depends
on it. Where it disagrees with `src/`, `src/` is authoritative.

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
