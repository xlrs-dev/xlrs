# XLRS — 2.4 GHz LoRa Control Link

XLRS is timing-sensitive radio-link firmware for a paired **TX/RX** running on
Raspberry Pi Pico (RP2040) boards with **Semtech SX1280** 2.4 GHz radios. It is
ExpressLRS-*like* in shape — CRSF in, CRSF out, FHSS, link statistics — but it
is **not** ExpressLRS OTA compatible. The over-the-air format, binding, and
hopping are XLRS-native.

```text
RC handset / CRSF handset ──▶ TX Pico ──▶ SX1280 LoRa + FHSS ──▶ RX Pico ──▶ CRSF flight controller
                                              (2.4 GHz)
              downlink telemetry  ◀──────────────────────────────  link stats / RSSI / LQ
```

The repository also includes an **RC handset** firmware target (RP2350) that
reads sticks/switches and drives a TX module over CRSF.

---

## Quick Start

Everything is driven by [PlatformIO](https://platformio.org/) and a small set of
wrapper scripts. From the repository root:

```bash
scripts/check-env.sh   # verify the toolchain (pio, python3, ...)
scripts/test.sh        # host-native Unity tests (pio test -e native)
scripts/build.sh       # build the TX/RX firmware pair
```

Flash one Pico at a time in BOOTSEL mode:

```bash
scripts/flash.sh tx    # → .pio/build/tx_lora_pico/firmware.uf2
scripts/flash.sh rx    # → .pio/build/rx_lora_pico/firmware.uf2
scripts/flash.sh rc    # → .pio/build/rc-rp2350/firmware.uf2  (handset)
```

Targeted direct flash helpers build and upload one role. Use these when a
single matching board is connected:

```bash
scripts/flash-tx.sh
scripts/flash-rx.sh
scripts/flash-rc.sh
```

When multiple USB serial devices are connected, pass the target port explicitly:

```bash
scripts/flash-tx.sh /dev/cu.usbmodem1101
scripts/flash-rx.sh /dev/cu.usbmodem5
scripts/flash-rc.sh /dev/cu.usbmodem14101
```

The same ports can be supplied through environment variables:

```bash
TX_PORT=/dev/cu.usbmodem1101 scripts/flash-tx.sh
RX_PORT=/dev/cu.usbmodem5 scripts/flash-rx.sh
RC_PORT=/dev/cu.usbmodem14101 scripts/flash-rc.sh
```

Monitor all connected modules in one color-tagged terminal view:

```bash
scripts/monitor-all.sh
```

The monitor probes USB serial ports with `status`, identifies `role=tx`,
`role=rx`, and `role=rc-rp2350`, ignores non-module ports, and prints timestamped
TX/RX/RC logs. It sends `status` every 5 seconds by default. Useful variants:

```bash
scripts/monitor-all.sh /dev/cu.usbmodem1101 /dev/cu.usbmodem5 /dev/cu.usbmodem14101
scripts/monitor-all.sh --status-interval 1
scripts/monitor-all.sh --status-interval 0
scripts/monitor-all.sh --no-clear
```

Equivalent manual PlatformIO commands:

```bash
pio test -e native
pio run  -e tx_lora_pico
pio run  -e rx_lora_pico
pio run  -e rc-rp2350
```

See [docs/build-test-flash.md](docs/build-test-flash.md) for the full workflow,
including `FLASH_METHOD=uf2|picotool` and serial monitoring.

---

## PlatformIO Environments

| Environment | Board | Role |
| --- | --- | --- |
| `tx_lora_pico` | RP2040 | CRSF handset input, LoRa uplink, telemetry back to handset |
| `rx_lora_pico` | RP2040 | LoRa receiver, CRSF RC + link-stats output to a flight controller |
| `rc-rp2350` | RP2350 | RC handset firmware: stick/switch sampling, OLED, CRSF out to a TX module |
| `tx_usb_diag` | RP2040 | `tx_lora_pico` with USB diagnostic logging (`LORA_USB_DIAG`) |
| `tx_crsf_diag` | RP2040 | `tx_lora_pico` with CRSF diagnostic logging (`LORA_CRSF_DIAG`) |
| `native` | host | Off-device Unity tests for protocol/scheduler/config helpers |

`tx_lora_pico` and `rx_lora_pico` are the default build targets. The default
binding phrase, pins, and rates are build flags in
[platformio.ini](platformio.ini).

---

## How the Link Works

- **Roles are fixed at build time.** One firmware image is the time-master TX,
  the other is the RX, selected by `-DLORA_TX_ROLE=1` / `-DLORA_RX_ROLE=1`.
- **Binding is phrase-based.** Both ends must share the same binding phrase. The
  phrase derives an 8-byte **Link UID**, which seeds the SX1280 sync word, the
  FHSS hop sequence, and a `uid_check` value carried in every OTA frame. There is
  no over-the-air pairing handshake — matching phrases bind the pair.
- **FHSS is on by default** across 40 channels (2404–2482 MHz, 2 MHz spacing).
  The TX is the time master; the RX disciplines its own tick clock to the TX with
  a phase/frequency detector (PFD) so both ends hop together.
- **Two rates** are available: `L250` (≈250 Hz, SF6) and `L100` (≈100 Hz, SF5
  with periodic downlink telemetry slots). Both use 812.5 kHz bandwidth.
- **OTA frames are plaintext.** A 33-byte frame carries a 22-byte payload plus a
  CRC16-CCITT and the binding-derived `uid_check`. These reject corruption and
  wrong-link traffic, but they are **not** authentication or encryption.
- **Failsafe is loss-driven.** When the uplink goes stale (no valid frame within
  the failsafe window, default 250 ms) the RX **stops emitting CRSF RC frames**
  so the flight controller sees RXLOSS and runs its own failsafe policy.

Protocol constants live in
[include/lora_link/protocol.h](include/lora_link/protocol.h) and the per-tick
scheduler/state machine in
[include/lora_link/rf/scheduler.h](include/lora_link/rf/scheduler.h).

---

## Runtime CLI (TX / RX)

USB serial runs at **115200 baud** on both TX and RX. Commands:

```text
bind get                 show the current binding phrase, UID, and uid_check
bind set <1..32 chars>   set and persist a new binding phrase (requires reboot)
bind clear               clear the persisted phrase back to the build default
rate L250                switch to the 250 Hz rate
rate L100                switch to the 100 Hz rate (with downlink telemetry)
status                   print role, link health, and scheduler diagnostics
channels                 print the current RC channel values
reboot                   reboot the device
```

TX and RX also accept the `rc.v1` binding commands over direct USB
(`binding_get`/`binding_set`/`binding_clear`/`binding_verify` with
`target=tx|rx`), used by the WebUI binding proxy.

---

## RC Handset (`rc-rp2350`)

The RP2350 handset firmware samples sticks and switches, drives an OLED, manages
battery/charging, and emits CRSF to the TX module. It has its own USB console:

```text
status      handset state summary
channels    live channel values
power       battery / charger status
reboot      reboot the handset
```

It also implements the `rc.v1` WebUI subset for discovery, live state, and a
TX-binding proxy:

```text
rc.v1 hello
rc.v1 tx_hello
rc.v1 state
rc.v1 binding_set target=tx phrase=<phrase>
```

Handset architecture and the WebUI protocol are documented under
[docs/rc-rp2350/](docs/rc-rp2350/index.md). The browser config tool lives in
[tools/rc-webui/](tools/rc-webui).

---

## Default Pins

Radio and CRSF pins are build flags in [platformio.ini](platformio.ini).

| Signal | Pico pin |
| --- | --- |
| CRSF UART TX | GP8 |
| CRSF UART RX | GP9 |
| CRSF baud | 400000 |
| SX1280 SCK | GP18 |
| SX1280 MOSI | GP19 |
| SX1280 MISO | GP16 |
| SX1280 CS | GP17 |
| SX1280 BUSY | GP20 |
| SX1280 DIO1 | GP21 |
| SX1280 RST | GP22 |
| SX1280 RXEN | GP14 |
| SX1280 TXEN | GP15 |
| Status LED | GP25 |

Full pinout and SX1280 wiring: [docs/hardware/pinout.md](docs/hardware/pinout.md)
and [docs/hardware/sx1280-wiring.md](docs/hardware/sx1280-wiring.md).

---

## Repository Layout

```text
include/
  lora_link/          public protocol + scheduler + control-frame headers
  rc_handset/         RC handset module headers
src/
  main.cpp            TX/RX role firmware (role selected by build flag)
  protocol.cpp        OTA frames, CRC, UID/FHSS, CRSF, RC packing, config
  lora_link/rf/       per-tick RF scheduler + RX timing/PFD
  control_frame.cpp   control-frame helpers
  rc_handset/         RC handset firmware (input, display, power, USB rc.v1)
lib/
  CrsfProtocol/       CRSF transmit helpers (SimpleTxCrsf)
  crsfSerial/         CRSF serial parsing
  UARTProtocol/       handset↔TX UART protocol
  crc8/               CRSF CRC8
test/                 host-native Unity tests (pio test -e native)
tools/rc-webui/       browser-based RC config tool
docs/                 developer + hardware + interface + troubleshooting docs
scripts/              build/test/flash/lint/monitor helpers
datasheets/           vendor datasheets (fetch with scripts/fetch-datasheets.sh)
platformio.ini        build environments and pin/rate flags
_legacy/              earlier XLRS core — reference only, not built (see below)
```

> **Note on `_legacy/`.** `_legacy/xlrs/` (formerly `lib/xlrs/`) and
> `_legacy/apps/` (formerly `apps/`) hold an earlier clean-slate "XLRS core" — a
> CMake/Pico-SDK layered design. Nothing in the active build depends on it and
> PlatformIO does not scan it; the firmware that builds and flashes is the
> PlatformIO `src/` tree above. See [_legacy/README.md](_legacy/README.md).

A fuller map is in [docs/developer/code-map.md](docs/developer/code-map.md).

---

## Documentation

Start at the [documentation index](docs/index.md).

- [Getting Started](docs/getting-started.md)
- [Build, Test, Flash](docs/build-test-flash.md)
- Developer: [code map](docs/developer/code-map.md) ·
  [architecture](docs/developer/architecture.md) ·
  [OTA protocol](docs/developer/ota-protocol.md) ·
  [timing & scheduler](docs/developer/timing-and-scheduler.md) ·
  [terminology](docs/developer/terminology.md)
- Hardware: [pinout](docs/hardware/pinout.md) ·
  [SX1280 wiring](docs/hardware/sx1280-wiring.md) ·
  [bench bring-up](docs/hardware/bench-bringup.md)
- Interfaces: [RX CRSF](docs/interfaces/rx-crsf.md) ·
  [TX controller CRSF](docs/interfaces/tx-controller-crsf.md)
- RC handset: [overview](docs/rc-rp2350/index.md)
- Troubleshooting: [symptom matrix](docs/troubleshooting/symptom-matrix.md) ·
  [serial logs](docs/troubleshooting/serial-logs.md)

Contributor workflow and the docs-update contract are in
[CONTRIBUTING.md](CONTRIBUTING.md). Agent/automation instructions are in
[AGENTS.md](AGENTS.md).

---

## Safety Notes

This firmware drives RC links. Treat these areas as safety-sensitive and validate
changes on the bench:

- Failsafe behavior and CRSF output gating.
- RF scheduler timing, slot selection, FHSS advance, and PFD updates.
- Binding identity, Link UID derivation, sync words, and `uid_check`.
- RF region/frequency tables and TX power.
- OTA frame formats and the flash-backed config schema.

FHSS is enabled by default (`RF_FIXED_CHANNEL=-1`). Set `RF_FIXED_CHANNEL` to a
non-negative channel index only for bench/debug fixed-channel operation, and
review regional RF regulations before transmitting above bench power levels.
