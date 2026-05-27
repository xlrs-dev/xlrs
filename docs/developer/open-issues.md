# Open Issues

This document tracks prospective issues found by code review after the May 2026
bench bring-up fixes. The issues are grouped by failure domain so related fixes
can be planned together. Severity describes expected operational impact, not
blame. Line references are approximate and should be refreshed before patching.

## Triage Matrix

| ID | Category | Severity | Status | Main Risk |
| --- | --- | --- | --- | --- |
| OI-001 | Timebase / counters | P0 | Confirmed | RC decode stops after tick crosses `INT32_MAX` |
| OI-002 | Persistence / identity | P0 | Confirmed mechanism | Power loss during flash commit erases binding/config |
| OI-003 | OTA decode / sync | P1 | Confirmed | Truncated Sync can snap RX to `txTick = 0` |
| OI-004 | Timing / PFD | P1 | Confirmed discrepancy | PFD diagnostics update but hardware timer may not nudge |
| OI-005 | Timing / failsafe recovery | P1 | Confirmed logic hazard | Stale timer bias can survive failsafe |
| OI-006 | Telemetry reliability | P1 | Confirmed | Stubborn telemetry deadlocks after one peer reboots |
| OI-007 | Diagnostics / LED | P2 | Confirmed | Bind-received LED aliases as solid on |
| OI-008 | Build config | P2 | Confirmed | Bad `XLRS_BENCH_RATE` indexes outside `kRates` |
| OI-009 | Timebase / counters | P2 | Confirmed pattern | ISR event counters are not wrap-safe |
| OI-010 | Timing / test gap | P2 | Confirmed | `rxPacketStartUs` is passed but unused by Link |
| OI-011 | Failsafe semantics | P1 | Confirmed | LQ holdoff can delay real failsafe |
| OI-012 | Failsafe / LQ ledger | P1 | Confirmed | `lqUp` can freeze without time-based decay |
| OI-013 | Security / power control | P1 | Confirmed | Unauthenticated telemetry can steer dynamic power |
| OI-014 | Build config / safety | P2 | Confirmed | Bench TX mode can weaken shipped failsafe |
| OI-015 | Scheduler recovery | P2 | Confirmed | RX-locked overrun leaves scheduler tick stale |
| OI-016 | Test coverage | P2 | Confirmed pattern | Hardware-only paths are mostly untested on host |
| OI-017 | PHY identity sync | P1 | Confirmed contract gap | Scheduler can mark identity synced when PHY skipped/failed the write |
| OI-018 | Binding / acquisition | P2 | Confirmed logic hazard | Bind-scan exit can retain lock/timing from the bind identity |
| OI-019 | Security / nonce lifecycle | P1 if AEAD ships | Confirmed app gap | AEAD session salt defaults to zero and is not negotiated in production |

## Priority Groups

1. **Safety and identity first:** OI-002, OI-011, OI-012, OI-013, OI-014, OI-017, OI-018.
2. **Long-uptime correctness:** OI-001, OI-009.
3. **Sync/timing stability:** OI-003, OI-004, OI-005, OI-010, OI-015.
4. **Security lifecycle:** OI-019.
5. **Reliability and observability:** OI-006, OI-007, OI-008, OI-016.

## Branch Fix Status

On branch `codex/open-issues-fixes`, the concrete firmware defects have matching
code fixes and host regression tests where practical. Two items are intentionally
recorded as mitigations rather than complete product features: OI-016 still needs
the larger async simulation architecture described in
[sim_test_enhancements_plan.md](sim_test_enhancements_plan.md), and OI-019 now
fails closed unless a nonzero session salt is set, but a production salt handshake
is still future work before AEAD can ship.

| ID | Branch status | Verification |
| --- | --- | --- |
| OI-001 | Fixed | `test_rc_decode_across_int32_tick_boundary` |
| OI-002 | Fixed | two-slot flash journal, active-link persistence gate, and flash capacity checks |
| OI-003 | Fixed | `test_sync_decode_rejects_truncated_tx_tick` |
| OI-004 | Code fixed; bench validation pending | Pico build applies PFD adjustment; native PFD suite remains green |
| OI-005 | Fixed | timing reset is requested on RX Connected-to-Failsafe regardless of retained lock |
| OI-006 | Fixed | peer-reboot reset plus stale-ACK rewind coverage |
| OI-007 | Fixed | LED bind-packet pattern no longer aliases at the 50 ms sample period |
| OI-008 | Fixed | CMake validates `XLRS_BENCH_RATE`; app startup clamps the selected rate |
| OI-009 | Fixed | scheduler event drains use modular counter differences; `test_scheduler_tick_event_counter_wrap` |
| OI-010 | Fixed | packet-start timing stays owned by `RfScheduler`; unused Link parameter removed |
| OI-011 | Fixed | RX failsafe now follows consecutive RC misses, not healthy/stale LQ |
| OI-012 | Partially fixed | failsafe unblocked from stale LQ; LQ display freeze still needs async-ledger decay |
| OI-013 | Fixed for power steering; auth incomplete | unauthenticated `TlmDown` no longer drives dynamic power; telemetry authenticity waits for AEAD |
| OI-014 | Fixed | bench TX mode is rejected in Release builds and warns loudly otherwise |
| OI-015 | Code fixed; bench validation pending | RX-locked overrun fast-forwards to the latest scheduler tick |
| OI-016 | Partially mitigated | added targeted host coverage; full async hardware-path sim remains planned |
| OI-017 | Fixed | `setSyncWord()` reports success/failure and scheduler retries failed writes |
| OI-018 | Fixed | bind-scan exit now resets acquisition under the restored bound identity |
| OI-019 | Mitigated | `setCipher()` fails closed until a nonzero session salt is configured |

## Firmware Area Map

This map groups the same bugs by the part of the XLRS firmware someone would
likely inspect or patch. Issues can appear in more than one area when the fault
crosses module boundaries.

| Firmware area | Issues | Why this area owns part of the fix | User impact from current bugs |
| --- | --- | --- | --- |
| Binding / identity | OI-002, OI-017, OI-018 | Binding state depends on persisted UID/phrase, PHY sync-word programming, and acquisition reset during bind-scan transitions. | Pair forgets binding after a write-time brownout; bind/rebind can appear randomly broken when software identity and radio/acquisition state disagree. |
| Security / authentication | OI-013, OI-019 | Default plaintext, unauthenticated `TlmDown`, and unfinished AEAD session-salt lifecycle all live in the link crypto boundary. | Nearby spoofing can manipulate displayed telemetry; default builds are plaintext, and AEAD is not field-ready until nonce lifecycle is finished. |
| TX behavior | OI-006, OI-013, OI-014, OI-019 | TX consumes downlink telemetry for dynamic power, owns bench-TX behavior, and participates in stubborn telemetry/AEAD nonce state. | Telemetry can freeze after peer reboot; unauthenticated downlink telemetry must not steer power; bench builds can make TX stay connected on telemetry alone. |
| RX behavior | OI-001, OI-002, OI-003, OI-005, OI-011, OI-012, OI-015, OI-018 | RX owns acquisition, RC decode, failsafe output, bind scanning, and the async LQ paths that decide whether control output remains active. | Receiver can drop after long uptime, delay failsafe, never failsafe after a stall, or extend a brief obstruction into a longer control loss. |
| RF scheduler / timing | OI-004, OI-005, OI-009, OI-010, OI-015, OI-017 | Scheduler tick/event handling, PFD correction, packet-start timing, overrun recovery, and PHY identity sync are coupled here. | Fast modes can feel fragile; long-uptime event wrap can stop event handling; stale timing can cause dropouts that outlast the original hiccup. |
| SX1280 PHY / radio driver | OI-004, OI-017 | Hardware timer behavior and sync-word writes must reflect actual SX1280 command success, not just software intent. | Radio may not apply the identity or timing correction the logs imply, so pairing or fast-rate performance can fail in misleading ways. |
| Persistence / config | OI-002, OI-008, OI-014 | Flash commit safety and compile-time bench overrides can silently change identity, rate, or failsafe behavior. | Binding can be lost after power interruption; developer builds can look like RF faults or ship with weakened failsafe if bench flags leak. |
| Telemetry / CRSF / MSP | OI-006, OI-013 | Stubborn telemetry sequencing and unauthenticated link-stat telemetry can stall telemetry or steer TX power. | OSD/sensor data can freeze until reboot; spoofed link stats can show bogus health or alter TX power decisions. |
| Diagnostics / tests | OI-007, OI-010, OI-016 | LED states, unused timing inputs, and host-vs-hardware test gaps affect how quickly the real fault is found. | LED-only bench checks can misread bind/fault/connected; host tests can miss hardware timing regressions that users see as dropouts. |

## Debugging And Severity Lens

Some issues are dangerous because of the direct failure. Others are dangerous
because they create the wrong diagnostic story and push the engineer toward the
wrong fix. Keep both dimensions visible during triage.

| Issue | Debugging / troubleshooting hazard | User-facing severity note |
| --- | --- | --- |
| OI-002 | Looks like a normal "lost binding" or user setup mistake after reboot. | More severe than inconvenience if the fallback identity is a shared default phrase. |
| OI-003 | Looks like random RF noise, but the decoder accepted an invalid Sync shape. | Usually a glitch, but can be severe if it happens close to the ground or during a high-load maneuver. |
| OI-004 / OI-010 / OI-016 | Host tests can pass while the hardware timing path remains wrong. | Fast-rate users may experience this as range loss or unexplained dropouts rather than a clear bug. |
| OI-005 / OI-015 | Looks like poor RF recovery after an obstacle, but the scheduler may be carrying stale timing. | Can turn a brief obstruction into a longer loss of control. |
| OI-007 | LED aliases bind-packet activity as solid on, overlapping fault/connected meanings. | Low direct safety risk, high bench-confusion risk. |
| OI-008 / OI-014 | Bad build flags masquerade as RF or failsafe behavior. | Severe only if bench builds escape into field use. |
| OI-011 / OI-012 | Telemetry/LQ can say the link is healthier than the RC freshness really is. | Highest direct operator risk: stale control output can persist too long or indefinitely. |
| OI-013 / OI-019 | Security posture can be misread because cipher hooks exist in code. | Default builds are plaintext; AEAD must not be considered field-ready until nonce lifecycle is finished. |
| OI-017 / OI-018 | Logs can say identity changed while the PHY or acquisition state still reflects the previous identity. | Binding/rebinding can fail intermittently and look like hardware or range trouble. |

## Timebase, Counters, And Long Uptime

These are failures that appear only after enough ticks/events have accumulated.
They are easy to miss in bench sessions and native tests unless wrap boundaries
are exercised directly.

### OI-001: RC Decode Fails After Tick Count Crosses `INT32_MAX`

- **Severity:** P0 for long-running units.
- **Status:** Confirmed from code.
- **Evidence:** `Link::processRxPayload()` tries nearby nonce ticks by casting
  `nonceTick` to `int32_t` in `tryNonceTicks()` (`lib/xlrs/link/Link.cpp`, RC
  decode path).
- **Why it matters:** At D250, `2^31` ticks is about 99.4 days. At F1000, it is
  about 24.9 days. After that, `(int32_t)nonceTick` is negative and every
  candidate tick can be skipped by `if (tryTick < 0) continue;`.
- **Likely symptom:** RX stops decrypting RC frames, enters failsafe, and does
  not recover until reboot.
- **Proposed fix:** Keep nonce candidate arithmetic in `uint32_t` modular space.
  Apply signed deltas without casting the base tick through `int32_t`; add tests
  around `0x7FFFFFFF`, `0x80000000`, and `0xFFFFFFFF`.

### OI-009: ISR Event Counters Are Not Wrap-Safe

- **Severity:** P2 field-endurance risk.
- **Status:** Confirmed pattern.
- **Evidence:** `RfScheduler` compares `current > last` for 32-bit tick/RX/TX
  event counters (`lib/xlrs/link/RfScheduler.cpp`).
- **Why it matters:** At F1000, a 32-bit tick counter wraps after about 49.7 days;
  at D250, about 198.8 days. Plain greater-than comparisons are not modular.
- **Likely symptom:** Long-uptime unit stops processing some event stream after
  counter wrap.
- **Proposed fix:** Use modular subtraction (`uint32_t diff = current - last`) and
  treat nonzero diff as pending, with existing catch-up limits.

## Persistence, Binding, And Build Configuration

These issues can make the firmware behave differently from the operator's mental
model: identity can reset, bench-only behavior can leak into production, or stale
CMake cache values can masquerade as RF faults.

### OI-002: Flash Commit Can Erase Binding/Config On Power Loss

- **Severity:** P0 safety/configuration risk.
- **Status:** Fixed on this branch with a two-slot journal, explicit flash-store
  capacity, persisted-write failure propagation, and app-level blocking while
  the link/output is active.
- **Evidence:** The old `FlashStore::commit()` erased the single flash sector and
  then programmed `s_cache` (`lib/xlrs/hal/FlashStore.cpp`). `BindingStore::begin()`
  wrote the default phrase when the persisted record was invalid
  (`lib/xlrs/link/BindingStore.h`).
- **Why it matters:** Power loss or reset between erase and program can leave the
  sector as `0xFF`. On next boot, binding/config validation fails and the device
  can silently fall back to the compiled default identity.
- **Likely symptom:** A custom-bound TX/RX pair reverts to default identity after
  a brownout during a write, causing mismatch or unintended default binding.
- **Fix:** Replace the single-sector store with a two-slot journal: sequence
  number, payload, checksum, and commit marker. Program the inactive slot fully,
  verify it, then mark it active. App persistence paths now refuse binding/RF
  config writes while the link/output is active.

### OI-008: `XLRS_BENCH_RATE` Override Is Not Range-Checked

- **Severity:** P2.
- **Status:** Confirmed from code.
- **Evidence:** TX/RX compile-time bench override is used directly as
  `kRates[rfRateIndex]` in app startup (`apps/tx/main.cpp`, `apps/rx/main.cpp`).
  The normal flash-config path clamps rates; the CMake override path does not.
- **Why it matters:** A stale or invalid cache value can select out-of-bounds
  rate data and look like an RF or timing failure.
- **Likely symptom:** Bad PHY config, bad timer interval, boot instability, or no
  acquisition after a bench rebuild.
- **Proposed fix:** Validate `XLRS_BENCH_RATE` at compile time in CMake and at
  runtime before indexing `kRates`.

### OI-014: `XLRS_BENCH_TX` Compile Flag Silently Weakens TX Failsafe

- **Severity:** P2 config safety.
- **Status:** Confirmed from code.
- **Evidence:** When the flag is defined, `setBenchTxMode(true)` is called
  (`apps/tx/main.cpp`). In bench mode `txLinkValid` depends only on telemetry
  (`Link.cpp`) and `lqDown > 0` suppresses failsafe.
- **Why it matters:** A release accidentally built with `XLRS_BENCH_TX` has
  materially altered TX failsafe semantics, with nothing at runtime making that
  obvious.
- **Likely symptom:** TX stays "Connected" on telemetry alone and is slow or
  unwilling to failsafe in a shipped image.
- **Proposed fix:** Guard bench flags out of release builds and print a loud boot
  banner whenever any bench mode is compiled in.

### OI-018: Leaving Bind Scan Does Not Reset Acquisition State

- **Severity:** P2 normally; P1 if bind-scan windows are enabled near normal
  acquisition in shipped RX firmware.
- **Status:** Confirmed logic hazard.
- **Evidence:** `Link::startBindScan()` switches to the bind identity and calls
  `resetAcquisition(Binding)`, but `Link::endBindScan()` only restores the bound
  identity and clears bind flags. It intentionally does not reset lock, sync anchors,
  `_lastRxTick`, LQ, or pending resync state (`lib/xlrs/link/Link.cpp`).
- **Why it matters:** If a bind-mode Sync or bind packet is accepted during the scan
  window, leaving scan can keep timing/lock state that was derived under the bind
  identity while the FHSS seed and sync word have already switched back to the bound
  identity.
- **Likely symptom:** RX exits a bind-scan window looking "Connecting" but reacquires
  normal traffic worse than a cold start, or waits for a later Sync to repair stale
  anchors.
- **Proposed fix:** Treat bind-scan exit like an identity change: restore the bound
  UID and call `resetAcquisition(Connecting)`, while preserving only the explicit
  `receivedBindUid` handoff state that the app needs.

## OTA Decode, Sync, And Nonce Alignment

These issues affect whether TX/RX agree on the same tick, nonce, and FHSS
position. They are especially dangerous because the radio may still look "alive"
while decode silently fails.

### OI-003: Sync Decoder Accepts Truncated Beacons With `txTick = 0`

- **Severity:** P1.
- **Status:** Confirmed from code.
- **Evidence:** `otaDecodeSync()` accepts `len >= 10`; when `len < 14`, it returns
  true with `s.txTick = 0` (`lib/xlrs/ota/OtaCodec.h`).
- **Why it matters:** Current acquisition depends on Sync `txTick` for scheduler
  snap and nonce anchor. A short Sync frame that survives lower-layer checks can
  snap RX to tick zero while TX is at its real tick.
- **Likely symptom:** Sudden lock loss after a malformed/truncated Sync frame;
  `fhss`/`exp` and nonce matching diverge immediately.
- **Proposed fix:** Require the full encoded Sync length (`14`) for decode. Add
  codec tests that reject 10-13 byte Sync frames.

### OI-010: `rxPacketStartUs` Is Passed To Link But Unused

- **Severity:** P2 design/test gap.
- **Status:** Confirmed; compiler warns in native tests.
- **Evidence:** `Link::processRxPayload(..., rxPacketStartUs)` does not use the
  parameter; `txTickForNonce()` also ignores its packet-start argument
  (`lib/xlrs/link/Link.cpp`).
- **Why it matters:** The precise packet-start timestamp is load-bearing for PFD
  in the scheduler, but nonce/tick matching still relies on the armed tick plus a
  broad search. That can hide late-DIO or F1000 timing problems from host tests.
- **Likely symptom:** Decode works by search margin at D250 but remains fragile at
  higher rates or under ISR/task jitter.
- **Proposed fix:** Either remove the unused parameter to make ownership clear, or
  use it to derive/validate the expected TX tick and add async delivery tests that
  deliver RX done after service.

### OI-017: PHY Identity Sync Can Be Marked Complete Without Programming The Radio

- **Severity:** P1 for bind/rebind/rate-recovery flows.
- **Status:** Confirmed contract gap.
- **Evidence:** `RfScheduler::syncPhyIdentity()` calls `IRadioPhy::setSyncWord()` and
  then unconditionally records `_syncedIdentityRevision = revision`. The concrete
  SX1280 implementation returns early when `_txInProgress` or `_hardwareError` is
  set, and can also fail during the register writes, without reporting failure to the
  scheduler (`lib/xlrs/link/RfScheduler.cpp`, `lib/xlrs/phy/Sx1280NativePhy.cpp`).
- **Why it matters:** The link can believe the PHY sync word matches the current UID
  while the radio is still programmed with the previous identity. Because the
  revision is marked synced, the scheduler has no reason to retry until another
  identity change or recovery path forces it.
- **Likely symptom:** Bind/rebind or identity restore intermittently fails after a
  slot is already in progress; logs show the link identity changed, but RF packets are
  still filtered by the old sync word.
- **Proposed fix:** Make `IRadioPhy::setSyncWord()` return `bool`, only advance
  `_syncedIdentityRevision` on success, and add tests with a mock PHY that rejects a
  sync-word write while busy.

## Timing, PFD, And Scheduler Recovery

These are the most bench-visible follow-ons from the recent bring-up work. They
control whether RX stays phase-aligned after lock, after failsafe, and after RF
core stalls.

### OI-004: Hardware PFD Records Corrections But Does Not Apply Them

- **Severity:** P1, especially for F1000 and marginal clocks.
- **Status:** Code fixed; bench validation pending.
- **Evidence:** On Pico builds, `RfScheduler::applyLockedRxPhaseResync()` used to
  call `_pfd.update(offsetUs)` and record `adjUs`, then set the timer to nominal
  `_rate.intervalUs` instead of `_rate.intervalUs + adjUs`
  (`lib/xlrs/link/RfScheduler.cpp`). The branch now applies `interval + adjUs`
  on the Pico path, but F1000 bench evidence is still required before closing the
  hardware risk.
- **Why it matters:** Diagnostics can show `pfd`, `adj`, and `n` as active while
  the hardware timer is not actually nudged between Sync snaps.
- **Likely symptom:** D250 works due to wide slots, but F1000 acquisition or long
  holds remain fragile; host tests overstate hardware timing behavior.
- **Fix:** Apply `interval + adjUs` on Pico. Remaining work: add a bench note for
  F1000 PFD convergence / timer clamping, or a future hardware-path simulation
  seam that can reproduce it off-board.

### OI-005: Connected-To-Failsafe Timer Reset Is Gated Off By Retained RX Lock

- **Severity:** P1 if OI-004 re-enables timer nudging.
- **Status:** Confirmed logic hazard.
- **Evidence:** `Link::service()` sets `_rxTimingResetPending` only on
  Connected->Failsafe when `!_locked`, but current recovery intentionally keeps RX
  locked through failsafe (`lib/xlrs/link/Link.cpp`).
- **Why it matters:** If the timer is biased by PFD before failsafe, the reset
  path usually will not run, so the biased timer can survive into reacquisition.
- **Likely symptom:** Reconnect after a brief link drop is worse than a cold boot.
- **Proposed fix:** Request the timing reset on RX Connected->Failsafe regardless
  of `_locked`, or explicitly document and test retained-bias behavior.

### OI-015: Scheduler Overrun While RX-Locked Leaves `_tick` Stale Until Next Sync

- **Severity:** P2; interacts with OI-001 and OI-012.
- **Status:** Code fixed; bench validation pending.
- **Evidence:** The RX-locked overrun path used to advance `_lastProcessedTickEvent`
  but not `_tick` (`lib/xlrs/link/RfScheduler.cpp`). The branch now fast-forwards
  to the latest scheduler tick and has host coverage for the happy path; a brief
  stall/reconnect bench run is still prudent.
- **Why it matters:** Until the next Sync snap, the scheduler arms RX on FHSS
  channels derived from a stale position. If accumulated drift exceeds the RC
  nonce search window, RX cannot re-anchor on RC alone and must wait for Sync.
- **Likely symptom:** Prolonged decode loss after a core-1 stall on a locked RX,
  recovering only when the next Sync lands.
- **Proposed fix:** On RX-locked overrun, either re-derive `_tick` from the latest
  tick count or widen/clamp the nonce search to the measured gap.

## Failsafe And Link-Quality Semantics

These are safety-sensitive because they decide when stale RC output stops. The
recent fixes reduced spurious failsafe, but the same mechanisms can also delay a
real failsafe.

### OI-011: Failsafe Onset Is Delayed By The LQ Holdoff Window Drain

- **Severity:** P1 safety, worst on slow rates.
- **Status:** Fixed.
- **Evidence:** `Link::service()` forces `misses = 0` whenever
  `_stats.lqUp >= FAILSAFE_LQ_HOLDOFF` (25%). Uplink LQ is a 100-slot sliding
  window (`LqTracker<100>`).
- **Why it matters:** After a link that was near 100% loses RC, the window must
  drain below 25% before `FAILSAFE_MISS` can act. That is roughly 75 missed uplink
  slots before normal failsafe counting resumes.
- **Likely symptom:** RC output keeps streaming stale/last values for up to about
  75 ms at F1000, 300 ms at D250, 0.5 s at L150, or 1.5 s at L50 after true
  signal loss.
- **Fix:** RX failsafe now follows consecutive missed uplinks directly; the LQ
  window is diagnostic and no longer masks RC freshness.

### OI-012: Async Uplink LQ Ledger Has No Time-Based Flush; `lqUp` Can Freeze

- **Severity:** P1; feeds OI-011.
- **Status:** Partially fixed.
- **Evidence:** `Link::advanceHwUplinkLqSlot()` closes a window only when the next
  uplink slot arrives; there is no tick/time watchdog. The RX-locked overrun path
  can discard backlog without calling `onTick()`/`service()`.
- **Why it matters:** If the uplink-slot stream stalls, `_lq` stops updating and
  `lqUp` holds its last value. The failsafe path no longer trusts this stale value,
  but CRSF/OSD/status can still display stale healthy LQ.
- **Likely symptom:** RX can correctly enter failsafe while still showing stale
  healthy LQ in diagnostics.
- **Remaining fix:** Flush/decay the LQ ledger on elapsed ticks, not solely on the
  next slot; close any window older than a bounded number of ticks as a miss.

## Telemetry, Security, And Power-Control Integrity

These issues affect downlink telemetry, authenticated payload assumptions, and the
information used for dynamic power control.

### OI-006: Stubborn Telemetry Can Deadlock After Independent Peer Reboot

- **Severity:** P1 for bidirectional telemetry reliability.
- **Status:** Fixed on this branch.
- **Evidence:** `StubbornSender::receiveAck()` only advances when
  `ackSeq == _currentSeq + 1`; `StubbornReceiver` resets `_expectedSeq` to zero
  on construction (`lib/xlrs/link/StubbornTelemetry.h`).
- **Why it matters:** If one side reboots mid-payload, receiver ACKs its reset
  expectation while sender keeps retransmitting the old sequence forever.
- **Likely symptom:** MSP/CRSF telemetry stream stalls permanently until both
  endpoints reboot or the sender state is otherwise reset.
- **Fix:** `ackSeq == 0` rewinds the current transfer to fragment 0 after a peer
  reboot. Repeated stale ACKs for an earlier sequence rewind to that fragment
  instead of dropping the payload.

### OI-013: Unauthenticated Downlink Telemetry Steers TX Dynamic Power

- **Severity:** P1 security.
- **Status:** Fixed for dynamic-power steering; telemetry authenticity remains
  incomplete until AEAD is active.
- **Evidence:** Standard downlink telemetry is sent in clear text
  (`otaEncodeTlmDown`) and used to feed dynamic power via
  `_txPowerDbm = _power.update(upLq, upRssi)` (`lib/xlrs/link/Link.cpp`). The
  branch now seals/opens `TlmDown` through the cipher layer and refuses to let
  unauthenticated/default-plaintext `TlmDown` update dynamic power.
- **Why it matters:** An attacker who knows or guesses the UID-derived sync word
  could inject `TlmDown` frames. Sync-word plus FHSS-channel filtering is not
  cryptographic, so unauthenticated telemetry must not control safety-relevant
  behavior.
- **Likely symptom:** Bogus telemetry display with no decode-failure indication;
  dynamic power is no longer steered by unauthenticated `TlmDown`.
- **Fix:** Refuse to let unauthenticated telemetry drive power. Remaining work:
  ship a real AEAD/session-salt handshake so `TlmDown` stats themselves are
  authenticated.

### OI-019: AEAD Session Salt Is Not Negotiated In Production

- **Severity:** P1 if AEAD is enabled for shipped firmware; P3 while `NullCipher`
  remains the deliberate production default.
- **Status:** Confirmed app gap.
- **Evidence:** `Link::_sessionSalt` defaults to zero and production app code never
  calls `Link::setSessionSalt()`. The docs already note that negotiation is not
  implemented, while `AeadCipher` requires a never-reused 96-bit nonce made from
  `session_salt`, packet counter, and FHSS index (`lib/xlrs/link/Link.h`,
  `lib/xlrs/crypto/ICipher.h`, `docs/developer/elrs-comparison.md`).
- **Why it matters:** If AEAD is enabled with the current app lifecycle, a reboot or
  reconnect can reuse the same key/nonce sequence for the same UID, tick, and hop
  position. ChaCha20-Poly1305 nonce reuse under one key breaks the security promise.
- **Likely symptom:** No bench-visible failure; this is a latent security regression
  that appears only once the secure-link path is turned on.
- **Proposed fix:** Before enabling AEAD in production, add a per-session salt
  handshake or persisted monotonic session counter, include it in both Sync/acquisition
  and nonce construction, and test reboot/reconnect nonce uniqueness.

## Diagnostics And Test Coverage

These do not directly break the RF link, but they make bugs harder to see and
increase the chance that host tests pass while hardware remains broken.

### OI-007: Bind-Packet LED Pattern Samples As Solid On

- **Severity:** P2 bench-debug hazard.
- **Status:** Confirmed from timing constants.
- **Evidence:** `linkStatusLedUpdate()` updates every 50 ms and uses
  `(msCounter % 100) < 75` for `bindPacketReceived`
  (`lib/xlrs/app/LinkStatusLed.h`). The sampled phases are 0 and 50 ms, both on.
- **Why it matters:** "Bind packet received" looks like solid ON, which overlaps
  with fault and connected indications.
- **Likely symptom:** LED-only bench diagnosis is misleading during binding.
- **Proposed fix:** Use a period that is not aliased by the 50 ms update, for
  example 150 or 200 ms, or a two/three sample pattern.

### OI-016: Hardware-Only Paths Are Unexercised By Host Tests

- **Severity:** P2 process/test-coverage.
- **Status:** Confirmed pattern.
- **Evidence:** Timing-critical logic such as `applyLockedRxPhaseResync()`, async
  LQ ledgers, telemetry-RX arming, and the `onRxDone` hardware branch lives under
  `#if defined(XLRS_PICO_SDK)`, while `scripts/test.sh` compiles the non-Pico
  branch.
- **Why it matters:** Every hardware-only branch is a place a regression hides
  until a bench session. This is the systemic reason the same class of issue can
  reappear only on hardware.
- **Likely symptom:** Host suite passes; behavior differs on hardware.
- **Proposed fix:** Introduce a seam that lets a host harness drive the
  DIO-after-`service()` delivery ordering and exercise the ledger/PFD hardware
  branches.

## User-Facing Impact (Plain Language)

The entries above are engineer-facing. This section restates each one as the
real-world failure an operator (pilot / installer) would actually experience, so
the risk can be judged without reading the code. "When it bites" is the trigger;
"What the operator notices" is the observable symptom.

| ID | When it bites (real-world trigger) | What the operator notices |
| --- | --- | --- |
| OI-001 | Gear left powered on continuously for weeks with no reboot (~25 days on the fast rate, ~99 days on the slow rate) | Receiver suddenly drops to failsafe with the transmitter right beside it at full signal; only a power-cycle brings control back |
| OI-002 | Power/battery cut at the instant config is being written (just after changing the bind phrase, or a brown-out during a save) | The pair "forgets" its binding and reverts to the factory default identity; your bound link no longer connects until you re-bind |
| OI-003 | A corrupted or short beacon slips through during RF noise/interference | A brief control glitch or failsafe blink mid-use even though you are well in range |
| OI-004 | Using the fastest packet rate, especially at longer range or on long holds | Fast mode feels twitchy — harder to connect and drops link sooner than expected; medium/slow rates are fine |
| OI-005 | A momentary dropout (flying briefly behind an obstacle, antenna shadowing) | After the blip it takes noticeably longer than a fresh power-on to regain control |
| OI-006 | Power-cycling one end mid-session while telemetry is mid-transfer | Telemetry / OSD / sensor data freezes and never resumes (stick control still works) until both ends are restarted |
| OI-007 | Reading the status LED during binding | The LED just sits solid on — you cannot tell binding from connected from hardware fault by the light alone |
| OI-008 | Custom/developer builds only, with a bad rate override | Unit won't boot or won't link and looks like an RF/hardware fault; does not affect standard released firmware |
| OI-009 | Always-on installation (ground station / fixed link) running for weeks to months | Radio stops handling events after very long uptime; needs a reboot |
| OI-010 | Internal test-coverage gap; surfaces as OI-004's fragility | No distinct symptom of its own — fast-rate reliability is worse than the test suite implies |
| OI-011 | A genuine loss of signal (out of range, antenna failure, crash behind cover) | The aircraft keeps obeying the last stick command for a fraction of a second up to ~1.5 s on the long-range mode before failsafe triggers — it keeps doing whatever it was last told for longer than it should |
| OI-012 | A rare processing stall while the link still reads "connected" | Failsafe now follows RC misses, but LQ/status can still look healthier than reality until the async ledger decays |
| OI-013 | A nearby attacker who can guess or learn your link's ID | Spoofed telemetry readings remain possible until AEAD ships; unauthenticated frames no longer steer dynamic power |
| OI-014 | Only if a unit is accidentally built/flashed with the bench flag | Transmitter is reluctant to failsafe and stays "connected" on telemetry alone; correct release builds are unaffected |
| OI-015 | Receiver CPU briefly bogged down (heavy load, flash write) | A control dropout that lasts noticeably longer than the hiccup itself, recovering only at the next beacon |
| OI-016 | Internal process/test gap | No direct symptom — it is the reason several of the above could reach the field without a host test catching them |
| OI-017 | Bind/rebind or identity restore happens while the radio is busy or a register write fails | Link software says the identity changed, but the radio may still listen/transmit under the old sync word, so pairing appears randomly broken |
| OI-018 | RX exits an automatic bind-scan window after seeing bind-mode traffic | Normal link acquisition can be worse than a cold start until the next valid Sync repairs stale timing state |
| OI-019 | Only if the optional AEAD security path is enabled before session-salt negotiation exists | No visible bench symptom, but encrypted packets can reuse nonces across reboot/reconnect and lose their security guarantee |

### The few a pilot should care about most

- **Control keeps moving after signal loss (OI-011), or never failsafes at all
  (OI-012).** These decide how fast outputs are cut when the link genuinely dies —
  the most safety-relevant behavior for anything that flies or drives.
- **Lost binding after a brown-out (OI-002).** A pair that silently reverts to the
  default identity will not reconnect when you expect it to.
- **Identity transitions that only half-apply (OI-017 / OI-018).** These can make
  binding or post-bind acquisition fail in ways that look random because the link
  state and the radio state are no longer the same story.
- **Mystery dropout after long uptime (OI-001 / OI-009).** Affects "leave it on"
  installations rather than a normal flight, but fails in a way that looks like a
  fault with no obvious cause.

Everything else is either developer/bench-only (OI-008, OI-014, OI-016), telemetry
or cosmetic (OI-006, OI-007), security-lifecycle work before AEAD ships (OI-019),
or a less-likely / in-range glitch (OI-003, OI-004, OI-005, OI-010, OI-013,
OI-015).
