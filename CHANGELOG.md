# Changelog

Notable behavior changes to the XLRS TX/RX firmware, newest first. Entries here
predate this project's move to git version control (see commit history for
changes after that point).

## 2026-07-30 — RC handset power on/off (QON) unreliable after sleep; OLED battery drain

**Component:** RC handset (`src/rc_handset/power/Rp2350Power.cpp`,
`src/rc_handset/display/OledDisplay.*`, `platformio.ini`)

**Problem 1 (production power gate never exercised):** The only handset build
env, `rc-rp2350`, set `-DRC_SKIP_POWER_ON_SEQUENCE=1` unconditionally, so the
5s hold-to-boot gate (`RC_POWER_ON_HOLD_MS`) was bypassed on every build —
nothing actually exercised production power-button behavior on battery+QON
hardware.

**Fix:** Added `[env:rc-rp2350-prod]`, identical to `rc-rp2350` minus the skip
flag. `scripts/flash-rc.sh` now accepts `RC_ENV=rc-rp2350-prod` to target it.

**Problem 2 (OLED stayed lit through sleep):** Deep sleep only power-gates the
RP2350 itself; the OLED (a separate I2C device) kept drawing full current
through "sleep."

**Fix:** Added `OledDisplay::sleep()` (`SH110X_DISPLAYOFF`) and a
`rcPowerSetSleepHook()` callback so `enterDeepSleep()` blanks the panel before
sleeping. Wake relies on the existing full reboot-through-`setup()` path to
re-`begin()` the display.

**Problem 3 (ship mode bricked wake) — hardware-confirmed:** `powerOff()`
called BQ2562X `requestShipModeForPowerOff()` (BATFET ship-mode, no delay,
forced even with VBUS present) immediately before `enterDeepSleep()`. This
opens the battery FET essentially instantly, killing the RP2350's own supply
rail before POWMAN's GPIO wake was even armed. Confirmed on hardware: power
off worked, power on never recovered.

**Fix:** Removed the ship-mode request. Power-off now relies solely on
POWMAN deep sleep + GPIO wake on `RC_POWER_BUTTON_PIN`, the same mechanism
already working on the boot-hold-abort path. Trade-off: gives up full
battery-disconnect savings for actually waking up; ship mode should only be
reintroduced once the power button's hardware wiring into the BQ2562X's own
wake circuit is confirmed via schematic.

**Problem 4 (still open after Problem 3's fix) — hardware-confirmed:**
Removing ship mode alone did not fix wake; reflashed and retested on
hardware, power-on still failed with no recovery.

**Added as a safety net (unconfirmed on hardware):** `watchdog_reboot(0, 0, 1)`
right before the fallback `wfi` loop in `enterDeepSleep()`, in case
`powman_set_power_state()` silently fails to engage sleep, or wake resumes
execution in place rather than resetting. Either way this forces a clean
reboot through `setup()` instead of hanging with Serial/OLED/CRSF all
uninitialized.

**Root cause found by diffing against a known-working reference firmware**
(an earlier, monolithic version of this codebase, `rc_crsf_main.cpp` /
`bq2562x.cpp`): that firmware calls `multicore_reset_core1()` immediately
before the POWMAN sleep calls in its equivalent of `enterDeepSleep()`. This
codebase launches an equivalent core (`rc_handset::core1::begin()` ->
`core1Main()`, active whenever `RC_HANDSET_ENABLE_CORE1=1`, set in both
`rc-rp2350` envs) but never halted it before core0 tried to sleep — core1
kept running and touching shared state while core0 tried to power down
shared POWMAN domains.

**Fix (latest, NOT yet confirmed on hardware):** Added
`multicore_reset_core1()` in `enterDeepSleep()` right after the OLED-sleep
hook, guarded by `#if defined(RC_HANDSET_ENABLE_CORE1)`.

**Validated:** `pio run -e rc-rp2350` and `-e rc-rp2350-prod` build clean
after every step above. The full on/off/on hardware cycle is validated only
through Problems 1-2's fixes (production env + OLED sleep). Problems 3 and
4's fixes are **unconfirmed on hardware** as of this entry — re-test the
complete power-off -> power-on cycle on `rc-rp2350-prod`, on battery only,
and capture the serial log if it still fails.

## 2026-07-30 — TX cold-boot spike gate regression test added

**Component:** `test/test_lora_link.cpp`

Added `test_spike_gate_accepts_cold_boot_first_frame_without_previous`,
covering the scenario the 2026-07-29 TX cold-boot entry below describes:
TX boots with failsafe channels, the handset is already streaming real
sticks, and the first-ever CRSF frame (`havePrevious=false`) must be
accepted immediately by both `sanitizeRcChannels` and
`acceptRcChannelsWithSpikeGate`. No prior test covered this path directly.

## 2026-07-29 — TX cold-boot spike gate holding sticks at center

**Component:** TX (`src/main.cpp`, `serviceTxCrsfInput`)

**Problem:** When the RC handset powered up after TX/RX (the "bad order" —
TX/RX first, handset second, or simultaneous), the first live CRSF RC frame
was compared against stale failsafe channel data by the input spike gate
(`RcSpikeGate` / `acceptRcChannelsWithSpikeGate` in `src/protocol.cpp`). A
full-throw stick move right after arming looked like a spike, so the gate held
it: roll/pitch/yaw stuck at center (992) and throttle crept up slowly instead
of tracking the stick immediately. `crsf_hold` climbed continuously on the TX
`status` output while this was happening.

**Fix (initial, validated on hardware):** Tracked TX failsafe state explicitly
(`g_txCrsfInFailsafe`, defaults `true` at boot and re-armed on every failsafe
writeback) and passed `havePrevious = !g_txCrsfInFailsafe` into the spike
gate, so the bypass applied to cold boot **and** to any mid-flight
failsafe-then-reacquire. This was the version validated against hardware
below.

**Fix (as shipped):** Replaced with a one-shot flag, `g_txCrsfEverFresh`
(false at boot, set permanently true on the first accepted live CRSF frame,
never reset again). `havePrevious = g_txCrsfEverFresh`. This bypasses the
gate only for the very first CRSF frame ever received after boot.

**Known trade-off of the one-shot version:** because the flag never re-arms,
a mid-flight failsafe event followed by reacquire is gated the same as any
other frame — if the stick position has moved far enough from the failsafe
values during the drop, the *same* centered-sticks/creeping-throttle symptom
this entry describes can recur after an in-flight link loss, not just on cold
boot. The initial fix (above) did not have this gap. Native unit tests in
`test/test_lora_link.cpp` (~line 901) were written against the initial fix's
reacquire-bypass behavior and no longer reflect what `main.cpp` does for that
scenario.

**Validation (cold boot, bad order, via `scripts/monitor.sh` on TX USB,
`status` polled repeatedly while arming):**

| Counter | Before | After |
| --- | --- | --- |
| `rc_failsafe` | climbing | 0 / clears immediately on CRSF arrival |
| `crsf_hold` | climbing | 0 |
| `crsf_rej` | — | 0 |
| `ch=...` | stuck at 992,992,992 / creeping throttle | all 4 channels track sticks immediately |

Also confirmed:
- Good boot order (handset first, then TX/RX) still works as before.
- Spike gate regression check: after a normal flight with a fresh link,
  unplugging/replugging a servo still ticks `crsf_hold` once or twice and
  rejects the spurious jump, then clears — the gate still does its job once a
  real previous frame exists.

**Follow-up, resolved 2026-07-29:** a related but distinct symptom — sticks
unresponsive for ~30s after switching a module from USB to battery power (or
the reverse) — turned out to be two separate things, neither of which is a
firmware bug:

- The schematic (`tx rx pcb.pdf`, shared by TX and RX) shows `VCC_5V` is one
  un-isolated net fed by both USB and the CRSF connector's 5V pin, with no
  diode-OR and no backup capacitor. Switching supplies without both briefly
  overlapping causes a genuine full MCU reboot — see
  [docs/hardware/pinout.md](docs/hardware/pinout.md) (Power section).
- That reboot does not itself explain 30s: `setup()` has no long blocking
  calls (a 300 ms CLI grace window plus an EEPROM read), and reconnect/RF
  reacquire completes in well under 1s (`kFailsafeTimeoutMs` = 250 ms,
  disarm burst 500 ms). The remaining delay is downstream of this firmware —
  most likely the flight controller's own Stage 2 failsafe holding inputs
  after it detects a link loss.

See the new symptom-matrix row in
[docs/troubleshooting/symptom-matrix.md](docs/troubleshooting/symptom-matrix.md)
for the diagnostic steps.

## 2026-07-29 — RX output spike gate creeping in for 30-40s after RX reboot

**Component:** RX (`src/main.cpp`, `rxLoop`)

**Problem:** Reported on hardware: pulling and reconnecting the drone
battery (RX reboots; TX/handset stay powered and never stop sending live
CRSF) let the craft arm, but roll/pitch/throttle stayed unresponsive and then
crept in gradually over 30-40s — the same creeping-input signature as the TX
cold-boot bug above, but on RX's output-to-FC path instead.

**Root cause:** RX's output spike gate had only a single-frame bypass, not a
persistent one:
- `reacquired = !isLinkFresh(nowMs, g_lastUplinkMs)` is true only for the
  first RC frame after reconnect, because `g_lastUplinkMs` is updated to
  `nowMs` immediately after that same frame — by frame 2 it's already fresh
  again.
- The other bypass, `g_rxFailsafeDisarmSentForLoss`, never arms on a fresh
  reboot at all: `maybeStartRxFailsafeDisarmBurst` explicitly bails out when
  `g_lastUplinkMs == 0`, which is exactly true right after boot.

Net effect: the arm frame (sticks near neutral) passes fine as frame 1. The
first real stick move on frame 2+ is no longer bypassed, so it's treated as a
spike and walked in gradually — matching the reported symptom exactly.

**Fix, attempt 1 (insufficient — did not resolve the symptom on hardware):**
Added `g_rxCrsfInFailsafe`, a persistent flag re-armed any time uplink is
stale, including before `g_lastUplinkMs` is ever set. This was a no-op fix in
practice: it still cleared on the very first accepted frame, exactly like the
`reacquired`/`g_rxFailsafeDisarmSentForLoss` bypasses it was meant to
supplement, so frame 2+ was just as unprotected as before. Confirmed on
hardware (3-4 repeat cycles): arming still worked (frame 1 was never the
broken part), but roll/pitch/throttle still crept in over ~30s on every
drone-battery reboot cycle.

**Why single-frame bypass isn't enough for RX** (unlike the TX fix above):
for TX, the handset's sticks are already sitting still by the time TX's UART
parser sees its first frame, so one bypassed frame captures a stable true
value and nothing after it looks like a jump. For RX, FHSS reacquisition
itself takes a little time, and that's exactly the window where the pilot
arms and then actively moves the stick — so the real stick motion often
lands on frame 2, 3, 4..., not frame 1, while still bypassed=false. The
spike gate's own multi-frame "confirm the move" convergence logic is what
produces the ~30s creep once that happens.

**Fix, attempt 2 (as shipped):** Instead of clearing the bypass the instant
frame 1 is accepted, hand off to a ~1s time-boxed grace window
(`g_rxCrsfFailsafeGraceUntilMs`, `kRxCrsfFailsafeGraceMs = 1000`) during which
the gate stays bypassed regardless of frame count, covering "arm, then
immediately move sticks." Normal spike protection resumes once the window
elapses.

**Validated:** `pio run -e rx_lora_pico` builds clean. Not yet re-tested on
hardware for this attempt — re-run the drone-battery reboot cycle and confirm
roll/pitch/throttle respond immediately with no creep, across multiple
cycles as before.
