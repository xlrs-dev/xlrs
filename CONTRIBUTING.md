# Contributing To XLRS

XLRS is a timing-sensitive radio-link firmware project. Small changes in naming,
slot timing, failsafe behavior, or persistent configuration can have large
effects, so contributors should start from the project vocabulary and tests.

## Start Here

1. Read [docs/developer/terminology.md](docs/developer/terminology.md).
2. Read [docs/developer/architecture.md](docs/developer/architecture.md).
3. Run the local checks:

```bash
scripts/check-env.sh
scripts/test.sh
scripts/build.sh
```

Run clang-tidy when changing C++ logic:

```bash
scripts/lint.sh
```

## Naming Rules

[docs/developer/terminology.md](docs/developer/terminology.md) is the source of
truth for project vocabulary.

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

Avoid expanding ambiguous short names such as `ch`, `tlm`, `pkt`, `rxStats`, or
`txPacket` into public APIs.

## Safety-Sensitive Areas

Treat these areas as safety-sensitive:

- Failsafe behavior and CRSF output gating.
- RF scheduler timing, slot selection, FHSS advancement, and PFD updates.
- Binding identity, Link UID derivation, sync words, and UID CRC checks.
- RF region tables, TX power caps, and dynamic power policy.
- Flash-backed config schemas and storage offsets.
- OTA frame formats and crypto nonce/counter handling.

Changes in those areas should include tests or a clear validation note.

## Documentation Expectations

Update docs in the same change when behavior changes:

| Change | Docs to update |
| --- | --- |
| Build, scripts, toolchain | `docs/build-test-flash.md` |
| Pin or hardware wiring | `docs/hardware/` |
| UART, CRSF, OTA, or config storage | `docs/interfaces/` |
| Naming or API ownership | `docs/developer/terminology.md`, `docs/developer/architecture.md` |
| Failsafe, timing, RF bring-up | `docs/troubleshooting/`, relevant developer guide |

## Pull Request Checklist

- I used the terminology from `docs/developer/terminology.md`.
- I ran `scripts/test.sh`.
- I ran `scripts/lint.sh` when changing C++ logic.
- I updated interface docs for protocol or storage changes.
- I updated troubleshooting docs for user-visible behavior changes.
- I documented any change to failsafe, RF timing, FHSS, or config persistence.
