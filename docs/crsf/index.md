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
| `CRSF` | Controller-facing CRSF for EdgeTX/OpenTX-style controllers. |

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

## Binding

CRSF Bind RX is supported. The command starts an XLRS OTA bind workflow that can
bind an unconnected RX. See [CRSF Binding](binding.md).

## Current Limits

- XLRS currently carries 8 OTA `rc_channel` values, so CRSF channels 9-16 are
  not transmitted over the uplink.
- RF parameter writes are persisted immediately but rate/region/power/failsafe
  changes apply after reboot.
- A dedicated XLRS Lua script is not implemented yet.
- Bind scan is a discovery/pairing workflow, not a cryptographic ownership
  proof.
