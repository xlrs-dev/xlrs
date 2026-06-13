# rc.v1 USB Protocol

`rc.v1` is the line-oriented USB protocol for RC handset host tools and the
WebUI bridge. The parser and formatter live in
`include/rc_handset/usb/protocol.h` and `src/rc_handset/usb/protocol.cpp`; the
current coverage is in `test/test_rc_usb_protocol.cpp`.

## Transport

- USB CDC serial.
- Default terminal baud: 115200.
- One ASCII command per line.
- Lines use space-separated tokens:

  ```text
  rc.v1 <command> [key=value ...]
  ```

- Values are percent-encoded when they contain spaces or reserved characters.
- `seq=<number>` is optional. Replies echo it when present.
- `target=<tx|rx|rc_config>` selects the device or config domain for commands
  that can address more than one endpoint.
- Binding writes accept `phrase=<text>` or `value=<text>`; the parser normalizes
  both into the command value.
- Unknown keys are preserved by the parser but ignored by command handlers that
  do not need them.

Success reply:

```text
rc.v1 ok seq=42 version=usb%20rc.v1
```

Error reply:

```text
rc.v1 err seq=77 code=bad_request message=needs%20field%3Dvalue
```

## Parse Errors

| Parser error | Meaning |
| --- | --- |
| `empty_line` | No command was present. |
| `bad_protocol` | Line did not start with `rc.v1`. |
| `missing_command` | Protocol token was present without a command. |
| `unknown_command` | Command name is not in the parser table. |
| `bad_argument` | A token was not `key=value` or a numeric value was invalid. |
| `bad_percent_encoding` | A percent escape was incomplete or not hex. |
| `missing_required_argument` | Command was missing a required `field`, `value`, or `mode`. |

Runtime handlers may also return higher-level response codes such as
`bad_request`, `unsupported`, `busy`, `denied`, or `io_error`.

## Commands

### hello

Host/device handshake. TX/RX direct USB replies with `caps=binding`; RC USB
replies with its WebUI capabilities.

```text
rc.v1 hello seq=42
rc.v1 ok seq=42 role=rc_handset fw=0.1 caps=state,stream_state,config,calibration,binding,tx_proxy
```

### tx_hello

RC USB discovery for the TX module attached behind the handset CRSF UART.

```text
rc.v1 tx_hello seq=43
rc.v1 ok seq=43 tx_present=1 phrase=Field%20Radio uid=0011223344556677 uid_check=89ABCDEF persisted=1 requires_reboot=0
```

### get_config

Reads a named RC handset setting. `field` is optional at the parser level, but
handlers may require it unless they intentionally return a full snapshot.

```text
rc.v1 get_config seq=10 field=rate
rc.v1 ok seq=10 field=rate value=L100
```

### set_config

Writes a named RC handset setting. Requires `field` and `value`. `set_config`
validates the candidate config and publishes it to the runtime output path
immediately; use `save` after verification to persist the active runtime config
to EEPROM. The WebUI uses fields such as atomic
`cal.axis.0=<min,center,max>`, `axis.invert.0`, `channel.trim.0`, and compact
`filter=<oversample,low_pass,temporal,highpass>`. `filter.low_pass` is a
0..100 aggressiveness percentage where `0` passes raw ADC values through and
higher values apply more smoothing. `filter.high_pass` is a 0..100
response-boost percentage mixed into the low-passed ADC signal; legacy version-1
records decode it as `0`.

```text
rc.v1 set_config seq=11 field=cal.axis.0 value=120,2048,3900
rc.v1 ok seq=11 field=cal.axis.0 value=120,2048,3900 persisted=0 generation=7
```

### apply, save, reset_defaults

Compatibility utility commands for RC-local config. Current WebUI live-edit flows
do not stage runtime edits: `set_config` publishes runtime config immediately,
while `save`, `reset_defaults`, and `cal_finish` persist to EEPROM. `apply`
remains accepted for older tools.

```text
rc.v1 apply seq=14
rc.v1 save seq=15
rc.v1 reset_defaults seq=16 target=rc_config
```

### state

Returns a current handset state snapshot for the WebUI. RC firmware returns raw
ADC values in `adc`, the filtered ADC values used by channel mapping in
`adc_filtered`, mapped channel preview values in `ch`, and link/battery
telemetry when present. `safety_hold=1` means physical CRSF output is being held
at the disarmed preset even though `ch` still shows the live mapped preview.
Host tools should accept both request replies and event-style `rc.v1 state`
lines.

```text
rc.v1 state seq=12
rc.v1 ok seq=12 adc=2048,2049,2001,2050 adc_filtered=2048,2049,2001,2050 ch=992,992,172,992 have_channels=1 safety_hold=0 lq=98 rssi=-47
rc.v1 state adc=2048,2049,2001,2050 adc_filtered=2048,2049,2001,2050 ch=992,992,172,992 have_channels=1 safety_hold=0 lq=98 rssi=-47
```

### stream_state

Enables or disables periodic state events.

```text
rc.v1 stream_state seq=13 interval_ms=100 enabled=1
rc.v1 ok seq=13 enabled=1
```

WebUI implementations should coalesce these events and update only live-value
DOM nodes. Rebuilding the full page for every state event can starve the browser
while the handset is streaming.

### cal_start

Starts calibration. The RC firmware samples current ADC values as the initial
endpoint set and puts CRSF output into a disarmed safety hold.

```text
rc.v1 cal_start seq=20
rc.v1 ok seq=20 adc=2048,2047,2049,2046
```

### cal_sample

Adds a sample to the active calibration session. The firmware samples live ADC
state directly and updates the collected min/max endpoints.

```text
rc.v1 cal_sample seq=21
rc.v1 ok seq=21 adc=100,2030,3990,2040 cal_min=100,2030,2048,2040 cal_max=2048,2047,3990,2046
```

### cal_finish

Validates and commits the active calibration. Firmware derives each ADC center
from the collected min/max endpoints, saves the active config to EEPROM, and
clears the calibration safety hold so the new mapping is visible on CRSF output
immediately.

```text
rc.v1 cal_finish seq=22
rc.v1 ok seq=22 cal_min=100,120,130,140 cal_center=2100,2110,2120,2130 cal_max=4100,4100,4110,4120
```

Manual calibration edits use the same config path as other fields:

```text
rc.v1 set_config seq=24 field=cal.axis.0 value=120,2048,3900
```

### binding_get

Reads the binding phrase or identity exposed by the selected target. On RC USB,
`target=tx` is proxied to the attached TX module. On direct TX/RX USB, omit
`target` or set it to the role hosted by that USB device.

```text
rc.v1 binding_get seq=30 target=tx
rc.v1 ok seq=30 target=tx op=binding_get result=ok phrase=Field%20Radio uid=0011223344556677 uid_check=89ABCDEF persisted=1 requires_reboot=0
```

### binding_set

Sets the binding phrase. Requires `phrase` or `value`.

```text
rc.v1 binding_set seq=31 target=tx phrase=Field%20Radio
rc.v1 ok seq=31 target=tx op=binding_set result=ok phrase=Field%20Radio uid=0011223344556677 uid_check=89ABCDEF persisted=1 requires_reboot=1
```

### binding_clear

Clears the binding phrase back to the firmware default for the selected target.

```text
rc.v1 binding_clear seq=32 target=rx
rc.v1 ok seq=32 target=rx op=binding_clear result=ok phrase=default uid=8899AABBCCDDEEFF uid_check=01234567 persisted=1 requires_reboot=1
```

### binding_verify

Verifies that a candidate phrase maps to the expected identity. Requires `value`.

```text
rc.v1 binding_verify seq=33 target=rx phrase=Field%20Radio
rc.v1 ok seq=33 target=rx op=binding_verify result=ok phrase=Field%20Radio uid=0011223344556677 uid_check=89ABCDEF persisted=0 requires_reboot=0
```

### special_mode

Requests a special mode such as bootloader entry. Requires `mode` or `value`.

```text
rc.v1 special_mode seq=40 mode=bootloader
rc.v1 ok seq=40 mode=bootloader
```

## WebUI Mapping

The WebUI should map user-facing actions to protocol commands without exposing
raw parser details:

| WebUI action | `rc.v1` command |
| --- | --- |
| Connect | `hello` |
| Refresh overview | `state` |
| Discover attached TX via RC USB | `tx_hello` |
| Live channel monitor | `stream_state interval_ms=100 enabled=1` |
| Read setting | `get_config field=<name>` |
| Write and publish runtime setting | `set_config field=<name> value=<encoded>` |
| Persist active runtime settings | `save` |
| Reset RC config defaults | `reset_defaults target=rc_config` |
| Start calibration | `cal_start` |
| Capture calibration sample | `cal_sample` |
| Finish calibration | `cal_finish` |
| TX binding through RC USB | `binding_get/set/clear/verify target=tx` |
| RX binding through direct USB | `binding_get/set/clear/verify target=rx` |
| Bootloader/update mode | `special_mode mode=bootloader` |

## Current Firmware Support

| Endpoint | Implemented `rc.v1` handlers |
| --- | --- |
| RC USB | `hello`, `tx_hello`, `get_config`, `set_config`, `apply`, `save`, `reset_defaults`, `state`, `stream_state`, `cal_start`, `cal_sample`, `cal_finish`, `binding_get/set/clear/verify target=tx` |
| TX direct USB | `hello`, `binding_get/set/clear/verify target=tx` |
| RX direct USB | `hello`, `binding_get/set/clear/verify target=rx` |

The parser still accepts `special_mode`, but RC firmware does not currently wire
that handler.

## Validation Fixtures

Keep host tests and WebUI mocks aligned with the parser tests:

```text
rc.v1 hello seq=42
rc.v1 tx_hello seq=3
rc.v1 set_config seq=7 field=cal.axis.0 value=120,2048,3900
rc.v1 get_config field=rate
rc.v1 reset_defaults target=rc_config
rc.v1 cal_start seq=1 field=aileron
rc.v1 cal_sample seq=2 field=aileron value=2048
rc.v1 cal_finish seq=3
rc.v1 binding_get target=tx
rc.v1 binding_set target=tx phrase=Field%20Radio
rc.v1 binding_clear target=rx
rc.v1 binding_verify target=rx phrase=Field%20Radio
rc.v1 state seq=9
rc.v1 stream_state interval_ms=100 enabled=1
rc.v1 special_mode mode=bootloader
```

Negative fixtures:

```text
rc.v2 hello
rc.v1 hello seq
rc.v1 set_config field=name value=Bad%XZ
rc.v1 set_config field=name
rc.v1 reboot
```
