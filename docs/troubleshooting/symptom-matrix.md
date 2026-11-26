# Symptom Matrix

| Symptom | Most Likely Cause | What To Check |
| --- | --- | --- |
| No USB serial port appears | Board not booted, bad cable, stuck in BOOTSEL, firmware not flashed | Try another cable, check `/dev/cu.usbmodem*`, reflash UF2, verify board exits BOOTSEL |
| Repeated reboot every ~1 second | RF core heartbeat stopped, often radio init failure | Watch for `[HW FAULT]`; inspect SX1280 wiring, BUSY/DIO1, SPI pins, power rail |
| TX/RX identities differ | Binding phrase/config mismatch | Rebuild both with same `XLRS_DEFAULT_BINDING_PHRASE` or reset/persist same binding phrase |
| Same identity, never connects | RF init, sync word, RF channel table, antenna/path issue | Compare `[PHY]` lines, check antennas/load, inspect PHY timeout/CRC counters |
| Connects then drops | PFD/timing instability, weak RF path, CRC errors, radio recovery | Watch LQ/RSSI/CRC, test boards close together at low power, then with attenuation |
| RX status reaches Connected but no FC input | CRSF wiring, baud, output gating, FC serial config | Check RX GP8/GP9 wiring to FC, CRSF baud, FC serial protocol, `outputActive` behavior |
| Failsafe too early during telemetry | Uplink-slot accounting bug or poor uplink RF | Check RX LQ and telemetry ratio; native tests cover uplink-slot accounting |
| TX telemetry stale or zero | Downlink telemetry slots missing, RX not transmitting telemetry | Watch TX `LQdown`, RX state, and PHY counters on both sides |
| Controller input ignored | TX UART wiring/protocol issue | Verify controller UART at 420000 8N1 and TX app ACK/PONG behavior |
| RX status LED dark or never changes | Wrong LED pin/polarity or LED wiring | Confirm `rx_lora_pico` uses `STATUS_LED_PIN=10`, active high; check GP10 wiring |
| Sticks/channels unresponsive for many seconds after switching TX or RX power source (e.g. USB to battery, or the reverse) | A momentary gap on `VCC_5V` causes a full MCU reboot (USB and the CRSF connector's 5V pin share one un-isolated net with no backup cap — see [docs/hardware/pinout.md](../hardware/pinout.md) Power section). Firmware's own reconnect path finishes in well under 1s (`kFailsafeTimeoutMs` = 250 ms, disarm burst 500 ms), so a stall much longer than that is downstream of firmware | Keep both supplies briefly overlapping during the switch if a clean handoff matters; separately check the flight controller's Betaflight Failsafe tab → "Stage 2 failsafe procedure", which is the most likely source of a multi-second hold |

See [index.md](index.md) for the full troubleshooting flow.
