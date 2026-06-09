# RC RP2350 Handset Port

This section documents the modular RC handset port that feeds the existing XLRS
TX module. The active radio path stays unchanged:

```text
RC RP2350 handset -> CRSF UART -> TX Pico -> SX1280 FHSS link -> RX Pico -> CRSF flight controller
```

The current `rc-rp2350` firmware reads local handset inputs, emits CRSF RC frames
on `Serial2`, updates an OLED display, services battery/power hardware, and
exposes both a small bring-up console and a partial `rc.v1` USB command surface.
The `rc.v1` parser is host-tested and is the protocol contract used by the
browser WebUI in `tools/rc-webui`.

## Documents

- [Architecture](architecture.md)
- [rc.v1 USB Protocol](usb-protocol-rc-v1.md)
- [WebUI and Workflows](webui-workflows.md)
- [Validation](validation.md)

## Current Firmware Surface

PlatformIO environment:

```bash
pio run -e rc-rp2350
```

UF2 output:

```text
.pio/build/rc-rp2350/firmware.uf2
```

Default RC pins:

| Function | Pin |
| --- | --- |
| Aileron ADC | GP26 |
| Elevator ADC | GP27 |
| Rudder ADC | GP28 |
| Throttle ADC | GP29 |
| 3-position switch A | GP1, GP2 |
| 3-position switch B | GP3, GP6 |
| 3-position switch C | GP7, GP10 |
| 3-position switch D | GP11, GP12 |
| CRSF UART TX | GP8 |
| CRSF UART RX | GP9 |
| USB CDC console | 115200 baud |
| Power button | GP22 |
| Power I2C SDA | GP4 |
| Power I2C SCL | GP5 |

Current USB console commands:

```text
status
channels
power
reboot
```

Current `rc.v1` commands wired in RC firmware:

```text
rc.v1 hello
rc.v1 tx_hello
rc.v1 state
rc.v1 binding_get target=tx
rc.v1 binding_set target=tx phrase=<1..32 byte phrase>
rc.v1 binding_clear target=tx
rc.v1 binding_verify target=tx phrase=<1..32 byte phrase>
```

The parser and WebUI also define config, calibration, streaming, save/apply, and
bootloader commands. Those commands are protocol/UI scaffolding until the RC
firmware runtime handlers are wired.
