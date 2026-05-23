# RX CRSF

The RX module outputs CRSF to the flight controller on the CRSF UART.

When the RF link is connected, RX emits packed CRSF RC channels. Channels 1-8
come from the RF link. Channels 9-16 are held at CRSF midpoint.

RX also emits CRSF `LINK_STATISTICS` about every 500 ms.

Valid CRSF frames received from the flight controller, except RC channels and
link statistics, are forwarded to TX over XLRS downlink telemetry. In CRSF
controller mode, TX writes those frames back to the controller CRSF port.

USB serial diagnostics expose RX-side CRSF flow:

```text
[RX STATUS] ... | out:<0|1> crsf_rc:<n> age:<ms> stats:<n> fc:<n> fcq:<n> fcdrop:<n> fcage:<ms> qdrop:<n>
```

| Field | Meaning |
| --- | --- |
| `out` | CRSF RC output gate; `1` means RX is allowed to emit RC frames |
| `crsf_rc` / `age` | CRSF RC frames emitted to the flight controller, and age of the most recent one |
| `stats` | CRSF `LINK_STATISTICS` frames emitted to the flight controller |
| `fc` | Valid telemetry CRSF frames received from the flight controller |
| `fcq` / `fcdrop` | Flight-controller telemetry frames queued or dropped before downlink telemetry |
| `fcage` | Age of the most recent flight-controller telemetry frame |
| `qdrop` | RF -> RX app queue drops |

Failsafe behavior is controlled by RF config:

| Mode | Value | Behavior |
| --- | ---: | --- |
| `NoPulses` | `0` | Stop CRSF RC output when link is not valid |
| `Hold` | `1` | Continue CRSF output during failsafe using preset failsafe channel values |

Default mode is `NoPulses`.

See [index.md](index.md) for the complete current interface reference.
