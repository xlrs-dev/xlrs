# Bench Link Acquisition Retrospective (May 2026)

This document records how a bound XLRS TX/RX pair on RP2040 + SX1280 was brought
from **no RF link** to **Connected with CRSF RC output** on the bench. It is
written for anyone debugging similar symptoms: Sync lock without RC decode, LQ stuck
at 0%, or `[HW FAULT]` during bring-up.

Hardware used throughout:

| Item | Value |
| --- | --- |
| Boards | Two RP2040 + SX1280 modules |
| Binding phrase | `Kikobot-02` |
| Link UID | `0x72E16DB67E0C894D` |
| Sync word | `0x894D` |
| Rate | F1000 (1000 µs tick, hop every 4 packets) |
| Bench distance | ~1 m, no RC handset (TX sends midstick RC autonomously) |
| USB serial (typical) | TX `/dev/cu.usbmodem1201`, RX `/dev/cu.usbmodem1101` |
| Flash by serial | TX `E4654C16430F4223`, RX `E4654C16432C3B22` |

See also [bench-bringup.md](bench-bringup.md), [../troubleshooting/index.md](../troubleshooting/index.md),
and [../developer/timing-and-scheduler.md](../developer/timing-and-scheduler.md).

---

## Symptom progression (one-line summary)

```text
[HW FAULT] + PHY timeout storm
  → clean PHY, still Disconnected
  → Sync lock (lock:1 sync:1) but LQ=0 / wild FHSS
  → FHSS aligned (fhss==exp) but still no RC (Failsafe)
  → Connected, LQ ~40%, out:1, CRSF RC streaming
```

---

## Architecture: old mode vs fixed mode

### Old mode (broken on hardware)

Several independent bugs stacked on top of each other:

1. **PHY layer** — `SetTx` waited for BUSY to clear (never does during TX); init called
   `setSyncWord()` while radio was in RX; SPI at 10 MHz vs working 8 MHz.
2. **FLRC length** — TX sent variable-length frames; PHY expected fixed 16-byte FLRC
   → PHY CRC errors, no OTA decode.
3. **Async delivery gap** — Sim delivers packets inside `onTick()`; hardware delivers
   on DIO **after** `service()` already ran → LQ/state never updated from late packets.
4. **FHSS index** — TX derived hop index from tick (`txPos(tick)`); RX incremented
   `_rxPos` on hop boundaries → same Sync lock could still listen/transmit on wrong
   channel after tick drift.
5. **AEAD nonce tick** — RC frames use `Nonce96.build(sessionSalt, txTick, pos)`.
   RX decoded with **local scheduler tick**, which diverged from TX tick when PFD
   shortened the RX timer (~942–953 µs vs 1000 µs nominal).
6. **PFD during acquisition** — Sync beacons fed the PFD before RC worked; biased
   timer period and desynced tick numbers from TX.

### Fixed mode (current `main`, commit `a9fa15d`)

| Layer | Behavior |
| --- | --- |
| **PHY** | Async `SetTx`/`SetRx`, standby before register writes, 8 MHz SPI, recover backoff |
| **FLRC** | Fixed 16-byte air length; bind-scan deferred 30 s on bound RX |
| **Async RX** | `Link::service(_armedTick)` re-run from `onRxDone` on firmware |
| **Tick snap** | Every valid Sync re-snaps RX scheduler tick to beacon `txTick` |
| **FHSS** | When locked, RX uses `txPos(tick)` — same formula as TX |
| **Nonce** | RX derives AEAD tick from Sync anchor: `anchorTxTick + (armedTick − anchorLocalTick)` |
| **PFD** | Nominal timer during **Connecting**; PFD nudges only after **Connected** |
| **Diagnostics** | Shared `LinkRuntimeDiag.h` on TX/RX status lines (`fhss`, `exp`, `skew`, `pfd`, `tmr`) |

### Clock sync model (TX master / RX slave)

```text
TX: HwTimer @ intervalUs (1000 µs) → tick++ → txPos(tick) → frequency + slot + nonce

RX acquisition:
  Sync beacon → snap scheduler tick to txTick
              → anchor (anchorTxTick, anchorLocalTick) for nonce
              → hold timer at nominal intervalUs

RX connected:
  PFD adjusts timer from RC/telemetry packet-start vs expected tick
  (Sync beacons excluded from PFD — different airtime)
```

---

## Fix passes with bench logs

Each pass lists **before → patched → after** status lines captured on the bench.
Lines are trimmed to the diagnostic tail; full captures were taken with
`scripts/monitor.sh` or dual-port `read` loops.

### Pass 0 — Baseline (pre-fix CMake/Pico SDK bring-up)

**Commits:** before `2891737`

**Problem:** `[HW FAULT]`, climbing PHY timeouts (~17k–20k), BUSY wedge on GP20.

| Phase | Representative log |
| --- | --- |
| **Before** | `[TX STATUS] ... PHY timeouts: 20143 CRC: 0 ... [HW FAULT]` |
| **Before** | `[RX STATUS] State: 4 LQ: 0% ... PHY timeouts: 17402 CRC: 0 ... [HW FAULT]` |

---

### Pass 1 — SX1280 PHY init (`2891737`)

**Changes:** Async `SetTx`, standby before `setSyncWord`, 8 MHz SPI, recover backoff,
init leaves radio in standby.

| Phase | Representative log |
| --- | --- |
| **Before** | (Pass 0 — timeout storm) |
| **After** | `[TX STATUS] State: 2 LQdown: 0% RSSI: 0 dBm \| PHY timeouts: 0 CRC: 0 ...` (no `[HW FAULT]`) |
| **After** | `[RX STATUS] State: 2 LQ: 0% RSSI: -94 dBm \| PHY timeouts: 0 CRC: 0 ...` |

PHY clean; RF link still not up.

---

### Pass 2 — FLRC fixed length + bind-scan grace (`6356e42`)

**Changes:** Pad TX to nominal FLRC length; restore RX packet params; 30 s bind-scan
grace on bound RX.

| Phase | Representative log |
| --- | --- |
| **Before** | PHY clean but `CRC: 1–4` on RX intermittently; LQ 0% |
| **After** | `[RX STATUS] State: 2 LQ: 0% ... PHY timeouts: 0 CRC: 0 ...` (PHY CRC settled) |

Still no Connected state.

---

### Pass 3 — Scheduler tick resync + failsafe lock (`9a80c43`, `7e015f2`)

**Changes:** First Sync lock snaps RX tick to beacon `txTick`; RX keeps FHSS lock
through Failsafe (does not drop `_locked` on Connected→Failsafe).

| Phase | Representative log |
| --- | --- |
| **Before** | `[RX STATUS] State: 4 ... lock:0 sync:0 ...` or repeated re-acquisition |
| **Half-way** | `[RX STATUS] State: 4 ... lock:1 sync:1 tick:35676 fhss:39 ...` (no `exp` field yet) |
| **After** | Sync seen more reliably; still `LQ: 0%`, `out:0` |

---

### Pass 4 — Async `service()` + status LED (`3f2517f`)

**Changes:** Re-run `Link::service(_armedTick)` from `onRxDone` on hardware; GP10
status LED; `snapSchedulerTick()` on first lock.

| Phase | Representative log |
| --- | --- |
| **Before** | `[TX STATUS] State: 2 ...` / `[RX STATUS] State: 4 LQ: 0% ... lock:1 sync:1 tick:41204 fhss:62 ...` |
| **After** | `[RX STATUS] State: 4 LQ: 0% ... lock:1 sync:1 tick:30947 fhss:56 ...` — Sync stable, still no RC |

**Note (pre-FHSS-fix):** Earlier in this session, RX `fhss` jumped arbitrarily
(`39 → 10 → 62 → 33 …`) while `lock:1 sync:1` because `_rxPos` was incremented
locally instead of derived from tick.

---

### Pass 5 — FHSS tick alignment + shared diagnostics (`8d97268`)

**Changes:** Locked RX hop index = `txPos(tick)`; re-snap tick on every Sync;
`LinkRuntimeDiag.h`; TX/RX status fields `fhss`, `exp`, `skew`, `pfd`, `tmr`.

| Phase | TX | RX |
| --- | --- | --- |
| **Before (pass 4)** | `State: 4 LQdown: 0% ... tick:29876 fhss:29 ... dlrx:25780` | `State: 4 ... lock:1 sync:1 fhss:29 exp:29 skew:0 pfd:-213us tmr:942/1000 out:0` |
| **After (8d97268)** | Same — still no RC | **FHSS fixed:** `fhss:56 exp:56 skew:0` but **`LQ: 0%`**, `tmr:942/1000`, stale `crsf_rc age:25224ms` |

FHSS alignment verified; RC crypto still failing.

---

### Pass 6 — Nonce tick anchor + defer PFD until Connected (`a9fa15d`)

**Changes:**

- RX AEAD nonce uses Sync anchor offset, not raw local tick.
- PFD skipped on Sync beacons; timer held at nominal during Connecting.
- PFD enabled only when `LinkState::Connected`.

| Phase | TX | RX |
| --- | --- | --- |
| **Half-way (nonce only, PFD still active)** | `State: 2 LQdown: 0% ... tmr:1000/1000 dlrx:0` | `State: 2 ... lock:1 sync:1 fhss:23 exp:23 pfd:-191us tmr:953/1000 out:0` |
| **Half-way (lost lock after ~70 s)** | — | `lock:0 sync:0 fhss:0 exp:36 ... [BIND SCAN]` |
| **After (PFD deferred, `tmr:1000/1000`)** | `State: 4 LQdown: 50% RSSI: -67 dBm ... tick:25757 fhss:39 tmr:1000/1000 dlrx:29` | **`State: 3 LQ: 42% ... lock:1 sync:1 fhss:13 exp:13 skew:0 pfd:0us tmr:1000/1000 out:1 crsf_rc:21491 age:1ms`** |

**Healthy Connected RX (final):**

```text
[RX STATUS] State: 3 LQ: 42% RSSI: -67 dBm | PHY timeouts: 0 CRC: 25 Phase: StartRx/begin LastOp: 0x8C LastOk: 0x86 LastFailOp: 0x00 | lock:1 sync:1 tick:25975 fhss:13 exp:13 skew:0 pfd:0us tmr:1000/1000 | out:1 crsf_rc:21491 age:1ms stats:46 fc:0 fcq:0 fcdrop:0 fcage:0ms qdrop:0
```

TX shows **State 4 (Failsafe)** on a bench TX with no RC handset — expected, because
TX failsafe is driven by uplink/controller path. RF downlink still works (`LQdown: 50%`,
`RSSI: -67 dBm`, `dlrx:29`).

---

## Diagnostic field cheat sheet

Use these on every bench session (see [serial-logs.md](../troubleshooting/serial-logs.md)):

| Field | Healthy when locked / connected |
| --- | --- |
| `lock` / `sync` | `1` / `1` |
| `fhss` vs `exp` | Equal |
| `skew` | `0` (Sync beacon `fhssIndex` matches `txPos(txTick)`) |
| `tmr` | `1000/1000` at F1000 during acquisition; small delta after Connected OK |
| `pfd` | Small tens of µs once Connected; `0us` during acquisition is normal |
| RX `out` | `1` when State 3 |
| RX `crsf_rc age` | Low ms (actively emitting CRSF) |
| RX `LQ` | Non-zero when RC decodes |

---

## Reproduce on hardware

```bash
scripts/test.sh
cmake -S . -B build -G Ninja -DXLRS_STATUS_LED_PIN=10 -DXLRS_STATUS_LED_ACTIVE_LOW=ON
cmake --build build --target xlrs_tx xlrs_rx

picotool load -x -f --ser E4654C16430F4223 build/xlrs_tx.elf
picotool load -x -f --ser E4654C16432C3B22 build/xlrs_rx.elf

TX_PORT=/dev/cu.usbmodem1201 RX_PORT=/dev/cu.usbmodem1101 scripts/monitor.sh both
```

Allow 5–10 s after boot for acquisition. Expect RX **State: 3**, **out:1** within ~30 s
at 1 m separation.

---

## Follow-up work

Tracked in GitHub issue [#8](https://github.com/xlrs-dev/xlrs/issues/8). Summary:

1. **Re-enable PFD during Connected** — validate it converges without pulling timer
   back to ~953 µs; consider clamping max adjustment.
2. **TX bench failsafe** — TX reaches Failsafe without handset; consider bench mode
   or downlink-only Connected criteria for TX status/LED.
3. **Status LED** — verify GP10 active-low wiring; user reported no blinks early in
   bring-up (stale CMake `XLRS_STATUS_LED_PIN=13` vs GP10).
4. **Bind-scan vs lock loss** — after ~70 s without RC, RX dropped lock and cycled
   bind-scan; tune timeout or keep lock longer during partial acquisition.
5. **Host sim parity** — add async delivery test that calls `onRxDone` after `service()`
   to catch regressions without hardware.
6. **Document PFD Sync exclusion** in [timing-and-scheduler.md](../developer/timing-and-scheduler.md)
   (partially done in troubleshooting §6).

---

## Commit map

| Commit | Summary |
| --- | --- |
| `2891737` | Harden SX1280 init (async TX, standby, SPI) |
| `6356e42` | FLRC fixed air length; bind-scan grace |
| `9a80c43` | Resync RX tick from Sync `txTick` |
| `7e015f2` | Keep RX FHSS lock through failsafe |
| `3f2517f` | GP10 LED; async RX `service()` on `onRxDone` |
| `8d97268` | FHSS tick alignment; `LinkRuntimeDiag` |
| `a9fa15d` | Nonce tick anchor; defer PFD until Connected |
