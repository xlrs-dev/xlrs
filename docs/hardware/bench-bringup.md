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

## Status LED (GP10 / Pico pin 13)

Both TX and RX drive a single-color GPIO status LED through
[`lib/xlrs/app/LinkStatusLed.h`](../lib/xlrs/app/LinkStatusLed.h).

| Item | Value |
| --- | --- |
| GPIO | GP10 (Pico physical pin 13) |
| Polarity | Active-low by default (`XLRS_STATUS_LED_ACTIVE_LOW=ON`) |
| Boot pattern | Five slow blinks at power-up |

**Bench note (May 2026):** The pre-migration PlatformIO RX firmware drove a
WS2812 NeoPixel on **GP13** (Pico physical pin **17**), not this LED. If the LED
never toggles after a fresh flash, verify the build picked up GP10 — stale CMake
cache may still have `XLRS_STATUS_LED_PIN=13`. Reconfigure with:

```bash
cmake -S . -B build -G Ninja -DXLRS_STATUS_LED_PIN=10 -DXLRS_STATUS_LED_ACTIVE_LOW=ON
cmake --build build --target xlrs_tx xlrs_rx
```

If GP10 is confirmed in the serial boot line but the LED stays dark, try
`-DXLRS_STATUS_LED_ACTIVE_LOW=OFF` for active-high wiring.

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
