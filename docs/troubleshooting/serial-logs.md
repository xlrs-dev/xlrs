# Serial Logs

Expected boot markers:

```text
[TX] === XLRS Pico SDK Transmitter ===
[TX] Binding identity store initialized.
[TX] RF Region: US
[TX] Max Power Cap: ... dBm
[TX] [UID] Computed identity: 0x...
[TX] [PHY] Mode: FLRC|LoRa, Sync Word: 0x....
[TX] Controller UART initialized.

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
[TX STATUS] State: <n> LQdown: <n>% RSSI: <n> dBm | PHY timeouts: <n> CRC: <n>
[RX STATUS] State: <n> LQ: <n>% RSSI: <n> dBm | PHY timeouts: <n> CRC: <n>
```

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

`[HW FAULT]`, increasing PHY timeouts, or increasing CRC errors mean the problem
is at or below the radio/PHY layer, not in CRSF or controller input.
