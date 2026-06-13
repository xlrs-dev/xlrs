# Bench Bring-Up

Run local checks before touching hardware:

```bash
scripts/check-env.sh
scripts/test.sh
scripts/lint.sh
scripts/build.sh
```

Flash and monitor both boards:

```bash
scripts/flash.sh tx
scripts/flash.sh rx

TX_PORT=/dev/cu.usbmodem101 RX_PORT=/dev/cu.usbmodem102 scripts/monitor.sh both
```

The TX and RX must report the same computed identity and sync word. If they do
not, fix binding/config before investigating RF behavior.

Expected debug order:

```text
build/config -> boot identity -> radio init -> sync/acquisition
             -> uplink RC -> RX CRSF output -> downlink telemetry
```

See [../troubleshooting/index.md](../troubleshooting/index.md) for the symptom
matrix and detailed flow.

## RX Status LED (GP10 / Pico physical pin 14)

The active PlatformIO RX firmware drives a single-color GPIO status LED from
`src/main.cpp`. The `rx_lora_pico` environment sets `STATUS_LED_PIN=10`, and the
LED is active high.

| RX condition | LED |
| --- | --- |
| Uplink fresh / connected | Solid on |
| Disconnected with valid binding config | 500 ms blink period |
| Disconnected with config fault or invalid binding config | 1 s blink period |

If the LED never changes after a fresh RX flash, verify the firmware was built
with `rx_lora_pico`, then check the GP10 LED wiring and active-high polarity.

## Rate selection

Flash-backed default rate is **D250** (250 Hz, 4000 µs tick, index 2 in `kRates`).
The name follows ELRS-style labeling; there is no separate `F250` row — use **D250**.

| Rate | Interval | Why use it |
| --- | --- | --- |
| F1000 | 1000 µs | Lowest latency; tightest slot budget |
| **D250** | 4000 µs | **Default** — 4× longer slots, more margin for async PHY + PFD on bench |
| F500 | 2000 µs | Middle ground |

Both boards must match (stored in RF config flash). After a firmware update that bumps
`RF_CONFIG_VERSION`, invalid flash config is replaced with defaults on first boot.

Confirm D250 on the wire: `tmr:4000/4000` in status lines (nominal interval).

## Link diagnostics (USB status lines)

Both TX and RX append tick/FHSS/timer fields to the periodic `[TX STATUS]` /
`[RX STATUS]` lines. Shared fill logic lives in
[`lib/xlrs/app/LinkRuntimeDiag.h`](../lib/xlrs/app/LinkRuntimeDiag.h).

When debugging FHSS alignment on the bench:

1. Confirm both boards show the same `fhss` value at the same time once RX
   reports `lock:1 sync:1`.
2. On RX, `fhss` and `exp` should always match when locked; wild `fhss` with
   `lock:1` was the pre-fix symptom of a tick-derived hop index diverging from TX.
3. Watch `pfd` on RX — it should stay small (tens of µs) once the PI loop has
   converged; large sustained error means phase lock is failing. While **Connected**,
   `adj` shows the last timer nudge and `n` counts PFD updates (should climb).
4. Compare TX `tick`/`fhss` with RX — they track once acquisition succeeds.

Clock sync summary: TX is the master timer; RX snaps its tick from every Sync
beacon's `txTick` and uses the PFD to nudge its timer period between beacons **only
after Connected**. See [../troubleshooting/index.md](../troubleshooting/index.md) §6
(PFD/timing) and the full pass-by-pass log history in
[bench-link-acquisition-retrospective.md](bench-link-acquisition-retrospective.md).

## Next: AIO + RC handset

After the USB-only bench pair reaches **State 3** on D250 with `out:1` and stable LQ
(see `tools/bench-run-capture.sh`), move to real wired I/O:

| Role | Wiring | Build notes |
| --- | --- | --- |
| **TX** | CRSF RC controller → TX CRSF/UART pins | `-DXLRS_TX_CONTROLLER_PROTOCOL=CRSF`; **`XLRS_BENCH_TX=OFF`** (production TX) |
| **RX** | RX CRSF TX → AIO / FC CRSF RX @ 420000 baud | Standard RX build; confirm `crsf_rc age` stays low and `out:1` |

Checklist:

1. Same binding phrase and D250 rate on both boards (`tmr:4000/4000`).
2. TX status: `CRSF rc` increments when sticks move; TX reaches **State 3** from handset RC.
3. RX status: **State 3**, `out:1`, `LQ` non-zero while sticks move.
4. FC telemetry (optional): RX `fc`/`fcq` track; TX `fc` increments when AIO sends sensors.

Automated bench capture uses `-DXLRS_BENCH_TX=ON` and midstick autostick — do **not**
use that build for handset testing.
