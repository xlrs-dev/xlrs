# Code Map

The firmware that builds and flashes is the PlatformIO `src/` tree. Roles are
selected at build time (`-DLORA_TX_ROLE=1` / `-DLORA_RX_ROLE=1`), so the TX and
RX share one `main.cpp`.

## Active firmware (`src/`, `include/`)

| Path | Responsibility |
| --- | --- |
| `src/main.cpp` | TX/RX role application: radio setup (RadioLib SX1280), tick loop, CRSF in/out, binding CLI, failsafe gating, status output |
| `include/lora_link/protocol.h` | Protocol surface: constants, OTA frame/sync structs, rate table, CRC, UID/FHSS, CRSF frames, RC packing, config records |
| `src/protocol.cpp` | Implementations of the protocol helpers above |
| `include/lora_link/rf/scheduler.h` | Per-tick scheduler types: connection state, RX timer/PFD events, `SchedulerStats` |
| `src/lora_link/rf/scheduler.cpp` | Slot selection, FHSS advance, sync handling, RX clock discipline (PFD) |
| `include/lora_link/control/frame.h`, `src/control_frame.cpp` | Control-frame helpers |

## RC handset firmware (`src/rc_handset/`, `include/rc_handset/`)

| Path | Responsibility |
| --- | --- |
| `src/rc_rp2350_crsf_main.cpp` | Handset entry point (RP2350), CRSF output to the TX module |
| `src/rc_handset/input/` | Stick/switch sampling and input pipeline |
| `src/rc_handset/core1/` | Core1 runtime + snapshot handoff |
| `src/rc_handset/display/` | OLED rendering |
| `src/rc_handset/power/` | Battery, BQ2562x charger, power button, RP2350 power |
| `src/rc_handset/telemetry/` | Handset telemetry aggregation |
| `src/rc_handset/usb/` | USB `rc.v1` protocol (WebUI discovery/state/binding proxy) |
| `src/rc_handset/config/`, `migration/` | Persisted config and legacy calibration migration |

## Vendored PlatformIO libraries (`lib/`)

| Path | Responsibility |
| --- | --- |
| `lib/CrsfProtocol/` | CRSF transmit helpers (`SimpleTxCrsf`, handset telemetry types) |
| `lib/crsfSerial/` | CRSF serial parsing and protocol constants |
| `lib/UARTProtocol/` | Handset ↔ TX UART protocol |
| `lib/crc8/` | CRSF CRC8 |

## Tests, tooling, docs

| Path | Responsibility |
| --- | --- |
| `test/` | Host-native Unity tests (`pio test -e native`); also the CMake project `scripts/lint.sh` uses |
| `tools/rc-webui/` | Browser-based RC config tool |
| `scripts/` | `build.sh`, `test.sh`, `flash.sh`, `lint.sh`, `monitor.sh`, `check-env.sh` |
| `datasheets/` | Vendor datasheets (`scripts/fetch-datasheets.sh`) |

## Legacy XLRS core (`lib/xlrs/`, `apps/`)

`lib/xlrs/` and `apps/tx`, `apps/rx` hold an earlier clean-slate, layered
"XLRS core" written against a CMake/Pico-SDK design (see
[architecture.md](architecture.md) for that design's intent). It is **not** built
by PlatformIO and does not flash. It is currently kept for reference and is
exercised only by the CMake lint/test project under `test/`. Do not assume the
flashing firmware behaves as those modules describe — `src/` is the source of
truth.

Naming follows [terminology.md](terminology.md).
