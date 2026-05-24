# Serial Logs

Expected boot markers:

```text
[TX] === XLRS Pico SDK Transmitter ===
[TX] Binding identity store initialized.
[TX] RF Region: US
[TX] Max Power Cap: ... dBm
[TX] [UID] Computed identity: 0x...
[TX] [PHY] Mode: FLRC|LoRa, Sync Word: 0x....
[TX] Controller UART initialized.      # Custom UART build
[TX] Controller CRSF initialized.      # CRSF RC build

[RX] === XLRS Pico SDK Receiver ===
[RX] Binding identity store initialized.
[RX] RF Region: US
[RX] Failsafe Mode: NoPulses|Hold
[RX] [UID] Computed identity: 0x...
[RX] [PHY] Mode: FLRC|LoRa, Sync Word: 0x....
[RX] CRSF UART and status LED initialized.
```

Periodic status lines:

```text
[TX STATUS] State: <n> LQdown: <n>% RSSI: <n> dBm | PHY timeouts: <n> CRC: <n> Phase: <phase>/<status> LastOp: 0xNN LastOk: 0xNN LastFailOp: 0xNN | tick:<n> fhss:<n> exp:<n> tmr:<actual>/<nom> dlrx:<n>
[TX STATUS] ... | CRSF rc:<n> age:<ms> ping:<n> pr:<n> pw:<n> fc:<n> bad:<n> qdrop:<n> dldrop:<n>
[RX STATUS] State: <n> LQ: <n>% RSSI: <n> dBm | PHY timeouts: <n> CRC: <n> Phase: <phase>/<status> LastOp: 0xNN LastOk: 0xNN LastFailOp: 0xNN | lock:<0|1> sync:<0|1> tick:<n> fhss:<n> exp:<n> skew:<n> pfd:<us>us tmr:<actual>/<nom> | out:<0|1> crsf_rc:<n> age:<ms> stats:<n> fc:<n> fcq:<n> fcdrop:<n> fcage:<ms> qdrop:<n>
```

The CRSF section in the TX status line appears only when TX is built with
`XLRS_TX_CONTROLLER_PROTOCOL=CRSF`.

During CRSF Bind RX:

```text
[TX BIND] OTA bind transmit window open for <n> seconds.
[TX STATUS] State: <n> ... [BIND TX <n>s]
[RX STATUS] State: <n> ... [BIND SCAN]
[RX STATUS] State: <n> ... [BIND RX]
```

Current state values:

| Value | LinkState |
| --- | --- |
| 0 | Disconnected |
| 1 | Binding |
| 2 | Connecting |
| 3 | Connected |
| 4 | Failsafe |

`[HW FAULT]`, increasing PHY timeouts, non-zero `LastFailOp`, or increasing CRC
errors mean the problem is at or below the radio/PHY layer, not in CRSF or
controller input. Opcode reference: [sx1280-phy-init.md](sx1280-phy-init.md).

Counter meanings:

| Counter | Meaning |
| --- | --- |
| `Phase` / `LastOp` / `LastOk` / `LastFailOp` | SX1280 init/runtime phase and SPI opcode diagnostics |
| TX `rc` / `age` | CRSF RC frames received from the controller, and age of the most recent one |
| TX `ping` | CRSF device discovery frames received |
| TX `pr` / `pw` | CRSF parameter reads/writes received |
| TX `fc` | Flight-controller CRSF telemetry frames forwarded back to the CRSF RC controller |
| RX `out` | Whether RX is currently allowed to emit CRSF RC output |
| `tick` | Scheduler tick counter (both roles) |
| `fhss` / `exp` | Current FHSS sequence index and tick-derived expected index (should match when locked) |
| RX `lock` / `sync` | FHSS locked to TX sequence; valid Sync beacon seen this session |
| RX `skew` | Last Sync beacon `fhssIndex − txPos(txTick)` (should be 0) |
| RX `pfd` | Last PFD phase error in µs (packet arrival vs expected tick) |
| `tmr` | Actual hardware timer period vs nominal `RateConfig.intervalUs` |
| TX `dlrx` | Scheduler ticks since last downlink telemetry decode |
| RX `crsf_rc` / `stats` | CRSF RC and link-statistics frames emitted to the flight controller |
| RX `fc` / `fcq` / `fcdrop` | Flight-controller telemetry frames received, queued, or dropped before downlink |
