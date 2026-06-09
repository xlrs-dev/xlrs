# RC RP2350 Architecture

The RC RP2350 board is a controller, not a radio endpoint. It should produce CRSF
RC input for `xlrs_tx` and expose configuration to a host over USB. The TX and RX
modules remain responsible for binding identity, RF scheduling, FHSS, failsafe,
and OTA telemetry.

## Module Layout

```text
Handset controls
  -> input sampler
  -> calibration and mapping
  -> channel safety filters
  -> CRSF encoder
  -> CRSF UART to TX module

USB host / WebUI
  -> rc.v1 USB protocol
  -> RC settings store
  -> calibration workflow
  -> TX bind proxy commands
```

Recommended ownership:

| Module | Owns | Must not own |
| --- | --- | --- |
| Input sampler | ADC/switch reads, debouncing, raw input snapshots | RF timing, Link UID derivation |
| Calibration | Min/mid/max, inversion, deadband, channel mapping | CRSF frame parsing from TX |
| Channel safety | Range clamps, spike gate, optional slew limits | Failsafe policy after RF loss |
| CRSF output | Packed CRSF RC frames to the TX module | OTA frames or SX1280 state |
| USB protocol | Host commands, WebUI snapshots, calibration writes | Direct flash layout for TX/RX |
| Settings store | RC-local handset profile, calibration, display, and power options | RX binding/config records |
| Display | OLED boot/status/channel/config/binding pages | RF scheduler ownership |
| Power manager | BQ2562x battery/charger, power button, low-power UX | RF TX power control |

## Boundaries

The RC board connects to the TX module over CRSF UART on GP8/GP9 by default. It
does not speak XLRS OTA frames and should not inspect radio scheduler state except
through telemetry/status reported by the TX module.

The USB interface is for host tools and the WebUI. Binding through the RC USB port
is a proxied workflow:

1. The WebUI sends an `rc.v1` binding command to the RC board.
2. The RC board forwards `target=tx` binding get/set/clear/verify requests to the
   attached TX module over the CRSF module-control path.
3. The RX is connected separately over its direct USB serial port and receives the
   matching `target=rx` binding command.
4. TX and RX reboot if either reply reports `requires_reboot=1`, then connect
   over the XLRS link identity derived from the shared binding phrase.

## Current Implementation

`src/rc_rp2350_crsf_main.cpp` and `src/rc_handset/` currently implement:

- Four ADC primary channels through `InputPipeline`.
- Four 3-position switches mapped to auxiliary channels.
- Channels 9-16 held at CRSF midpoint.
- CRSF RC frame output on `Serial2`.
- Range clamping, deadzone, trim, cutoff, spike gate, and slew limiting helpers.
- RC handset config records with CRC validation.
- Legacy calibration migration helpers.
- OLED display pages for boot, status, channel monitor, config, and binding.
- BQ2562x battery/charger support and power-button handling.
- A host-tested `rc.v1` line parser/formatter with WebUI utility commands.
- RC USB `rc.v1` handlers for `hello`, `tx_hello`, `state`, and TX binding
  proxy commands.
- TX/RX direct USB `rc.v1` handlers for role discovery and local binding
  get/set/clear/verify.
- A simple bring-up console with `status`, `channels`, `power`, and `reboot`.

The following are reserved for the modular handset port:

- Runtime handlers for `get_config`, `set_config`, `apply`, `save`,
  `reset_defaults`, `stream_state`, calibration commands, and `special_mode`.
- Model memory.
