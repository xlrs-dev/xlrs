# XLRS CRSF Features

XLRS supports CRSF in two places:

- TX controller interface: optional build-time controller protocol for the TX
  module.
- RX flight-controller interface: always used by the RX module to emit RC and
  link statistics to the flight controller.

## Build Selection

The TX controller protocol is selected with `XLRS_TX_CONTROLLER_PROTOCOL`.

| Value | Behavior |
| --- | --- |
| `UART` | Custom XLRS controller UART protocol. This is the default. |
| `CRSF` | Controller-facing CRSF RC for any compatible CRSF RC controller. |

```bash
cmake -S . -B build -G Ninja -DXLRS_TX_CONTROLLER_PROTOCOL=CRSF
cmake --build build --target xlrs_tx xlrs_rx
```

The RX flight-controller CRSF output is not optional in the current firmware.

## TX Controller CRSF

When TX is built with `-DXLRS_TX_CONTROLLER_PROTOCOL=CRSF`, it uses the same
controller UART pins and 420000 baud rate as the custom UART build.

Supported TX-side CRSF features:

| Feature | Status |
| --- | --- |
| RC channel input | Supported for CRSF channels 1-8 |
| Link statistics to controller | Supported about every 200 ms |
| Device discovery | Supported with `DEVICE_PING` and `DEVICE_INFO` |
| Parameter read | Supported for critical RF settings |
| Parameter write | Supported, persisted to flash |
| Bind RX command | Supported with OTA bind scan/transmit |
| Reboot command | Supported |
| Flight-controller telemetry bridge | Supported from RX to TX to controller |

TX CRSF RC status is visible in USB serial logs:

```text
[TX STATUS] ... | CRSF rc:<n> age:<ms> ping:<n> pr:<n> pw:<n> fc:<n> bad:<n> qdrop:<n> dldrop:<n>
```

- `rc` and `age` prove CRSF RC input is reaching TX.
- `ping`, `pr`, and `pw` prove CRSF discovery/config traffic is reaching TX.
- `fc` proves bridged flight-controller CRSF telemetry is returning to the CRSF
  RC controller.
- `qdrop`, `dldrop`, and `bad` should stay zero during normal operation.

Supported parameters:

| Parameter | Behavior |
| --- | --- |
| Rate | Persisted to TX and forwarded to RX; applies after reboot |
| Max Power | Persisted to TX and forwarded to RX; applies after reboot |
| Dynamic Power | Persisted to TX and forwarded to RX; applies after reboot |
| Region | Persisted to TX and forwarded to RX; applies after reboot |
| Failsafe | Persisted to TX and forwarded to RX; applies after reboot |
| Bind RX | Starts temporary OTA bind-transmit mode on TX |
| Reboot | Reboots TX and forwards reboot command to RX when possible |

## RX Flight-Controller CRSF

The RX module emits CRSF to the flight controller on the CRSF UART.

Supported RX-side CRSF features:

| Feature | Status |
| --- | --- |
| Packed RC output | Supported for OTA `rc_channel` 1-8 |
| CRSF channels 9-16 | Emitted at midpoint |
| Link statistics to flight controller | Supported about every 500 ms |
| Failsafe `NoPulses` | Supported; stops RC output when link is invalid |
| Failsafe `Hold` | Supported; continues RC output with preset failsafe channels |
| Flight-controller telemetry ingestion | Supported for valid non-RC, non-link-stat CRSF frames |

RX CRSF status is visible in USB serial logs:

```text
[RX STATUS] ... | out:<0|1> crsf_rc:<n> age:<ms> stats:<n> fc:<n> fcq:<n> fcdrop:<n> fcage:<ms> qdrop:<n>
```

- `out` shows whether CRSF RC output is currently enabled.
- `crsf_rc` and `stats` prove RX is writing CRSF frames to the flight controller.
- `fc`, `fcq`, and `fcdrop` prove flight-controller telemetry ingress and queueing
  into XLRS downlink telemetry.

## Binding

CRSF Bind RX is supported. The command starts an XLRS OTA bind workflow that can
bind an RX before it has made a normal connection in the current boot. See
[CRSF Binding](binding.md).

## Current Limits

- XLRS currently carries 8 OTA `rc_channel` values, so CRSF channels 9-16 are
  not transmitted over the uplink.
- RF parameter writes are persisted immediately but rate/region/power/failsafe
  changes apply after reboot.
- A dedicated XLRS CRSF RC configuration script is not implemented yet.
- Bind scan is a discovery/pairing workflow, not a cryptographic ownership
  proof.
