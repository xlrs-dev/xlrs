# XLRS Troubleshooting Guide

This guide is for bench bring-up and field debugging of the XLRS TX/RX radio
modules. It follows the project glossary in
[../developer/terminology.md](../developer/terminology.md):

- **TX/RX** are device roles.
- **Uplink** is TX -> RX over the radio.
- **Downlink** is RX -> TX over the radio.
- **RC channel** means stick/switch data.
- **RF channel** means a frequency-hop table entry.
- **CRSF** is a wired protocol. XLRS can use it on the TX controller side and
  always uses it on the RX flight-controller side.

The fastest way to debug XLRS is to isolate the path in order:

```text
build/config -> TX controller input -> boot identity -> radio init
             -> acquisition -> uplink RC -> RX CRSF output
             -> FC telemetry ingress -> downlink telemetry -> TX controller output
```

Do not start by changing RF timing constants. Most failures are configuration,
identity, UART wiring, SPI/radio init, or CRSF output gating.

---

## 1. First Five Minutes

Run these before touching hardware:

```bash
scripts/check-env.sh
scripts/test.sh
scripts/lint.sh
```

Build the TX for the controller protocol you are actually wiring:

```bash
# Custom XLRS controller UART, default
scripts/build.sh

# CRSF RC controller input
cmake -S . -B build-crsf -G Ninja -DXLRS_TX_CONTROLLER_PROTOCOL=CRSF
cmake --build build-crsf --target xlrs_tx xlrs_rx
```

Flash and monitor both boards:

```bash
scripts/flash.sh tx
scripts/flash.sh rx

TX_PORT=/dev/cu.usbmodem101 RX_PORT=/dev/cu.usbmodem102 scripts/monitor.sh both
```

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

The TX and RX **must** report the same computed identity and sync word. If they
do not, stop and fix binding/config before investigating RF behavior.

Periodic status lines:

```text
[TX STATUS] State: <n> LQdown: <n>% RSSI: <n> dBm | PHY timeouts: <n> CRC: <n>
[TX STATUS] ... | CRSF rc:<n> age:<ms> ping:<n> pr:<n> pw:<n> fc:<n> bad:<n> qdrop:<n> dldrop:<n>
[RX STATUS] State: <n> LQ: <n>% RSSI: <n> dBm | PHY timeouts: <n> CRC: <n> | out:<0|1> crsf_rc:<n> age:<ms> stats:<n> fc:<n> fcq:<n> fcdrop:<n> fcage:<ms> qdrop:<n>
```

The second TX status form appears only in CRSF RC builds.

During CRSF Bind RX, TX also logs one-shot bind state transitions:

```text
[TX BIND] OTA bind transmit window open for <n> seconds.
[TX BIND] OTA bind transmit window closed.
```

While the TX bind window is open, periodic TX status lines append
`[BIND TX <n>s]`, where `<n>` is the approximate seconds remaining. In the
custom controller UART build, `UART_MSG_STATUS.pairingState` is also set to `2`
while bind transmit is active so a PC-side tool can monitor it without parsing
USB logs.

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

### CRSF RC Bring-Up Counters

In a CRSF RC build, the TX status counters answer whether the wired controller
side works before you debug RF:

| Field | Meaning | Healthy bring-up signal |
| --- | --- | --- |
| `rc` | Valid CRSF RC frames received by TX from the CRSF RC controller | Increments continuously while the controller is on |
| `age` | Milliseconds since last CRSF RC frame | Stays low; a large value means controller UART/wiring/protocol trouble |
| `ping` | CRSF `DEVICE_PING` frames received | Increments when the controller discovers the module |
| `pr` / `pw` | CRSF parameter reads/writes | Increments when the controller opens/edits the device parameters |
| `fc` | FC telemetry CRSF frames forwarded by TX back to the CRSF RC controller | Increments after RX receives FC telemetry and downlink works |
| `bad` | Downlink app messages TX could not parse as CRSF frame messages | Should stay zero |
| `qdrop` | TX app -> RF queue drops for config/bind/reboot messages | Should stay zero |
| `dldrop` | RF -> TX app downlink queue drops | Should stay zero |

The RX status counters answer whether the flight-controller side works:

| Field | Meaning | Healthy bring-up signal |
| --- | --- | --- |
| `out` | Whether RX is allowed to emit CRSF RC frames to the flight controller | `1` when connected, or in `Hold` failsafe |
| `crsf_rc` | CRSF RC frames emitted by RX to the flight controller | Increments while `out:1` |
| `stats` | CRSF link-statistics frames emitted by RX to the flight controller | Increments about every 500 ms |
| `fc` | Valid non-RC, non-link-stat CRSF frames received from the flight controller | Increments when FC telemetry is present |
| `fcq` | FC telemetry CRSF frames queued into XLRS downlink telemetry | Tracks `fc` unless queueing is saturated |
| `fcdrop` | FC telemetry frames dropped before OTA queueing | Should stay zero |
| `fcage` | Milliseconds since last accepted FC telemetry frame | Low when FC telemetry is active |
| `qdrop` | RF -> RX app queue drops | Should stay zero |

---

## 2. Symptom Matrix

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
| CRSF RC controller never discovers module | TX built for custom UART, wrong UART wiring, no CRSF `DEVICE_PING` | Confirm boot says `Controller CRSF initialized`, check TX status `ping`, probe TX UART at 420000 8N1 |
| CRSF RC input ignored | TX controller wiring/protocol issue | In CRSF build, check TX status `rc` increments and `age` stays low; in UART build, verify custom UART ACK/PONG |
| Status LED solid on RX before link | Config fault or hardware fault | Check RX boot log for default config write or `[HW FAULT]` |

---

## 3. Configuration And Identity

### Build-Time Checks

Both firmware images must be produced from compatible configuration:

```bash
cmake -S . -B build -G Ninja \
  -DPICO_BOARD=pico \
  -DXLRS_DEFAULT_BINDING_PHRASE="bench-pair-01"
cmake --build build --target xlrs_tx xlrs_rx
```

Important values:

- `XLRS_DEFAULT_BINDING_PHRASE`
- `XLRS_UART_TX_PIN` / `XLRS_UART_RX_PIN`
- `XLRS_CRSF_TX_PIN` / `XLRS_CRSF_RX_PIN`
- `XLRS_SX128X_*` SPI/BUSY/DIO/RST/RXEN/TXEN pins
- RF region and default rate stored in flash config

### Binding Identity

The binding phrase derives the 64-bit Link UID. The Link UID seeds FHSS and the
radio sync word. A mismatch guarantees acquisition failure.

Check both logs:

```text
[UID] Computed identity: 0x...
[PHY] Mode: FLRC|LoRa, Sync Word: 0x....
```

If they differ:

1. Reconfigure/rebuild both images with the same default phrase.
2. If a persisted phrase exists, set the same phrase through the TX binding
   command path or erase/reset stored config.
3. Reflash both boards and compare boot logs again.

---

## 4. Hardware Wiring

### Controller UART Or CRSF RC To TX Module

The TX module consumes either custom UART frames or CRSF RC frames over UART at
420000 baud, depending on `XLRS_TX_CONTROLLER_PROTOCOL`.

```text
Controller TX  -> XLRS TX module RX pin
Controller RX  <- XLRS TX module TX pin
Controller GND -- XLRS TX module GND
```

Default pins:

```text
XLRS TX module UART TX: GP8
XLRS TX module UART RX: GP9
```

Use 3.3 V logic. On a logic analyzer, one bit at 420000 baud is about 2.38 us.
In CRSF RC builds, a healthy controller side shows TX status `rc` increasing and
`age` staying low. If the CRSF RC controller supports device discovery, `ping`
should increment when it scans the module.

### RX Module CRSF To Flight Controller

The RX module emits CRSF to the flight controller while `Link::outputActive()`
is true.

```text
XLRS RX module CRSF TX -> FC serial RX
XLRS RX module CRSF RX <- FC serial TX, if used
XLRS RX module GND ---- FC GND
```

Default pins:

```text
XLRS RX module CRSF TX: GP8
XLRS RX module CRSF RX: GP9
CRSF baud: 420000
```

If `FailsafeMode::NoPulses` is active, RX stops emitting RC channel frames after
failsafe. This is expected and is how Betaflight/iNav-style RXLOSS is triggered.
Link statistics may still be emitted by the app path.

### SX1280 Radio Wiring

Default SX1280 pins:

| Signal | Pin |
| --- | --- |
| SPI SCK | GP18 |
| SPI MOSI | GP19 |
| SPI MISO | GP16 |
| SPI CS | GP17 |
| BUSY | GP20 |
| DIO1 | GP21 |
| RST | GP22 |
| RXEN | GP14 |
| TXEN | GP15 |

If PHY timeouts increase:

- Check BUSY is not stuck.
- Check CS/SCK/MOSI/MISO activity during boot.
- Check DIO1 pulses on TX-done/RX-done.
- Check the SX1280 power rail under TX current.
- Check antenna/load before transmitting at higher power.

---

## 5. Link Bring-Up Flow

### Stage A: Boot And Radio Init

Success looks like:

```text
[UID] Computed identity: same on both boards
[PHY] Mode: same on both boards
[PHY] Sync Word: same on both boards
No [HW FAULT]
PHY timeouts: stable
```

Failure here is below the link layer. Do not debug CRSF or telemetry yet.

### Stage B: Acquisition

Before lock, RX dwells on the acquisition RF channel and waits for a Sync frame.
After a valid Sync with matching UID CRC, RX adopts the FHSS position/rate and
starts following the sequence.

Expected status progression:

```text
RX State: Connecting -> Connected
TX State: Connecting -> Connected
RX LQ rises above 0
```

If both boards have the same identity but never connect:

1. Put boards close together with antennas attached.
2. Compare RF region/rate config.
3. Watch CRC and timeout counters.
4. Use a logic analyzer on SPI/DIO1 to verify RX actually arms receive windows.

### Stage C: Uplink RC

On uplink slots, TX sends packed RC channels to RX. RX unpacks the RC payload and
emits CRSF while output is active.

If link is Connected but the FC sees no input:

1. Verify TX CRSF `rc` increments, or custom UART channel input is active.
2. Verify RX state is 3 (`Connected`).
3. Verify RX status `out:1`.
4. Verify RX status `crsf_rc` increments.
5. Probe RX CRSF TX pin at 420000 baud.
6. Confirm FC serial port is configured for CRSF.
7. Confirm ground is shared between RX module and FC.

### Stage D: Downlink Telemetry

On telemetry slots, RX transmits downlink telemetry to TX. TX reports this as
`LQdown`. In CRSF RC builds, valid flight-controller CRSF telemetry is bridged
from RX to TX and then written back to the CRSF RC controller port.

If uplink works but TX telemetry is zero/stale:

1. Check TX status `LQdown`.
2. Check RX status is Connected and RX PHY counters are stable.
3. Check TX PHY CRC/timeout counters; downlink can fail independently of uplink.
4. Verify telemetry ratio/rate config is the same on both sides.

If flight-controller telemetry is missing in a CRSF RC controller:

1. Check RX status `fc` increments when the flight controller is connected.
2. Check RX `fcq` tracks `fc` and `fcdrop` stays zero.
3. Check TX status `fc` increments.
4. Check TX status `dldrop` and RX `qdrop` stay zero.
5. Probe FC TX -> RX CRSF RX wiring at 420000 baud.

---

## 6. Timing And RF Diagnostics

XLRS depends on small ISRs and task-context RF work.

Rules:

- Timer ISR: latch timestamp / increment event counter only.
- DIO ISR: latch event/timestamp only.
- RF task: slot decision, OTA encode/decode, SPI, and PHY operations.
- Core 1 must not call `printf`; USB stdio is blocking and not multicore-safe.

If the system freezes or UART data drops:

1. Audit recent changes for SPI, logging, allocation, or crypto inside ISR paths.
2. Scope a temporary GPIO pulse around the timer ISR; target less than 5 us.
3. Scope a GPIO pulse around RF task slot processing.
4. Watch `missedDeadlines` via TX/RX status plumbing where available.

PFD/timing lock guidance:

- TX is the time master.
- RX runs the PFD/PI loop.
- TX must not adjust its master clock based on downlink telemetry.
- Packet-start timing is recovered as RX timestamp minus airtime.

Symptoms of timing sign problems:

- RX briefly connects, then repeatedly loses lock.
- LQ drops even with boards close together.
- Scope shows RX slot tick moving away from RX-done timing instead of toward it.

---

## 7. RX Status LED

Current RX LED behavior:

| Condition | LED |
| --- | --- |
| Config fault or hardware fault | Solid on |
| Bind scan open | Double flash, pause |
| Valid bind frame received | Very fast blink until flash persistence/reboot |
| Bind UID persisted | Five short flashes, then reboot |
| Generic binding state | Fast blink |
| Connected and output active | Solid on |
| Connecting | Medium blink |
| Other/disconnected | Slow blink |

During CRSF Bind RX, periodic RX status lines also append `[BIND SCAN]` while
the RX is open to bind packets and `[BIND RX]` after a valid bind frame has been
received. Bind scan is active only before the RX has made a normal connection in
the current boot. Power-cycle RX before repeating a first-time bind scan. The LED
is only a coarse hint. Trust serial logs and counters first.

---

## 8. Watchdog And Hardware Faults

Both apps feed the hardware watchdog only while the RF core heartbeat advances.
If the RF core stalls, the watchdog intentionally reboots the module.

Common causes:

- SX1280 init failure.
- SPI/BUSY timeout.
- RF core stuck in recovery.
- Blocking work accidentally added to core 1.

Debug steps:

1. Monitor serial output from boot.
2. Look for `[HW FAULT]` in status lines.
3. Check `PHY timeouts` and `CRC`.
4. If reboot loops are too fast for observation, temporarily add a boot delay or
   publish more fault detail through the core-0 mailbox.

---

## 9. Native Test Coverage

Before bench testing, run:

```bash
scripts/test.sh
```

The native suite covers:

- Link UID determinism and FHSS behavior.
- OTA packing/unpacking.
- AEAD crypto vectors and tamper rejection.
- PFD timing math.
- Link connect/failsafe/recovery flows.
- Rate switching and dynamic power logic.
- Corrupt RC rejection.
- Telemetry transport simulation.
- PHY recovery simulation.

If a behavior fails in hardware but passes natively, suspect:

- SX1280 driver/SPI timing.
- DIO interrupt wiring.
- Power integrity.
- RF path/antenna.
- Real oscillator drift.
- App-side UART/CRSF wiring.
- CRSF RC controller discovery/input counters.
- RX CRSF output/FC telemetry counters.

---

## 10. Field Notes To Capture

When filing a bug or bench note, capture:

- Commit hash and build command.
- TX/RX boot logs through identity and PHY lines.
- TX/RX periodic status lines for at least 10 seconds.
- RF region, rate, max power, failsafe mode.
- Board type and SX1280 module type.
- Antenna/load setup.
- Distance/attenuation/interference setup.
- Logic analyzer captures for SPI, DIO1, UART, or CRSF when relevant.

Good troubleshooting logs should make it clear whether the failure is:

```text
configuration -> controller input -> radio init -> acquisition -> uplink
              -> RX CRSF output -> FC telemetry -> downlink -> controller output
```
