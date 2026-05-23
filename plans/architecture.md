# XLRS Link Core Architecture

Clean-slate architecture for the XLRS 2.4 GHz radio link, optimised for **scalability**
and **reliability**. The production firmware now builds around one XLRS core and OTA
format; the older `SX128xLink`/`Protocol`/legacy SX128x mains were removed at cutover.

---

## 1. Goals & non-goals

**Goals**
- Deterministic, low-jitter control link (hardware-timed cadence + RX↔TX clock lock).
- Interference immunity and multi-system coexistence (FHSS).
- Defined failsafe and fault recovery (flight-safety baseline).
- Extensible: new packet rates, telemetry types, channel counts, hardware targets,
  regions, and an optional security layer — without touching unrelated layers.
- Testable off-hardware: every pure-logic layer has native unit tests.

**Non-goals (for the core; handled elsewhere or later)**
- Regulatory certification, RF/hardware validation, manufacturing — out of firmware scope.
  (But the firmware *controls* the FHSS region tables and TX power, so there is a compliance
  review gate before high-power / range testing — see roadmap.)
- Controller input hardware and configuration tools — they sit outside this TX/RX repository.

---

## 2. Layering

Strict layering; each layer depends only on the interface of the one below.

| Layer | Module (`lib/xlrs/...`) | Responsibility | Swappable? |
|-------|-------------------------|----------------|------------|
| Application | `apps/tx`, `apps/rx` | CRSF out + link stats + failsafe (RX); channel source (TX) | yes |
| Link | `link/Link` | Connection LIFECYCLE (bind/connect/failsafe); LinkStats consumer | — |
| RF scheduler | `link/RfScheduler` | **Per-tick coordinator**: slot timing, FHSS-advance, TX/RX turnaround, feeds LinkStats | — |
| Timing | `timing/HwTimer`, `timing/Pfd` | Packet cadence; RX PLL locking to TX | impl per MCU |
| FHSS | `fhss/Fhss` | UID-seeded hop sequence over a region channel table | tables per region |
| OTA | `ota/OtaPacket`, `ota/ChannelPack` | Versioned, type-multiplexed frame; channel (un)packing | format versioned |
| Crypto | `crypto/ICipher` (`NullCipher`, future `AeadCipher`) | Optional seal/open over OTA payload | yes (pluggable) |
| PHY | `phy/IRadioPhy` (`Sx1280NativePhy`) | Async radio chip control (TX/RX/freq/power) | yes (interface) |
| Util | `util/RingBuffer` (events), `util/Mailbox` (latest-value) | Lock-free inter-core handoff | — |

Timing / FHSS / OTA are **not independent peers** — they are coordinated by `RfScheduler`,
which owns the per-tick sequence. Without a single owner, "clean" modules still race.

---

## 3. Key design decisions

1. **PHY is an interface (`IRadioPhy`).** The production implementation is
   `Sx1280NativePhy`, a Pico SDK SX1280 driver using SPI and DIO1 interrupts. Keeping the
   interface narrow prevents radio-driver details from leaking above the PHY line.
2. **Crypto is a pluggable layer (`ICipher`), default `NullCipher`.** This defers the
   "secure-link wedge vs. drop crypto" decision: ship plaintext now, slot in real AEAD
   later by swapping the cipher — link/PHY untouched (CTR alone lacks authenticity).
   **Mode lean for M8:** prefer **ChaCha20-Poly1305** — RP2040/RP2350 have no hardware AES,
   and ChaCha is faster than AES-GCM in software; avoid GCM specifically, whose nonce-reuse
   failure is catastrophic (leaks the auth key). The 16-byte tag won't fit an 8-byte frame,
   so the tag is **truncated** (e.g. 4 bytes) — a deliberate airtime ↔ forgery-resistance knob.
   At 4 bytes the per-attempt forgery probability is 2⁻³²; document the accepted bound, the
   packets-per-session / rekey interval (per-session salt, §3.10), and auth-failure handling
   (drop the frame, do not desync the counter). For an RC link, **MAC-only** (integrity, no
   confidentiality) is often preferable to encryption — secrecy of stick positions rarely matters.
3. **TX is the time master; RX runs a PFD/PLL** (`timing/Pfd`) that nudges its hardware
   timer using the measured arrival offset of each packet. Foundation for both
   determinism and reliable hopping. The PFD is a **PI loop** (Kp=1/4, Ki=1/256, anti-windup):
   the proportional term locks phase; the integral term tracks the persistent crystal
   frequency mismatch (±10–30 ppm) that a P-only loop would leave as a constant steady-state
   offset (≈4× the per-interval drift). Lock quality ultimately depends on oscillator
   stability (a TCXO helps). *(Implemented + tested in M0; tuned on hardware at M2.)*
4. **Dual-core split.** RF state machine + timing ISR on core 1; application (CRSF/UART,
   display, telemetry aggregation) on core 0. Isolates control timing from application
   jitter. Channels cross via a latest-value mailbox (decision 9), discrete events via a
   FIFO ring. Cross-core primitives use **C++11 acquire/release atomics** (`std::atomic` +
   thread fences), NOT bare `volatile`: `volatile` blocks compiler reordering but emits no
   hardware barrier, so Core 0 could publish the seqlock counter before the payload is
   visible to Core 1 (torn read). Atomics compile to `dmb` on M0+/M33 and stay host-testable.
   *(Implemented in M0 in `util/Mailbox.h` + `util/RingBuffer.h`.)*
5. **Versioned, type-multiplexed OTA frame** (`ota/OtaPacket`). One small frame format
   (target 8 bytes, 16-byte variant) carries RC / sync / telemetry / bind / MSP via a
   type field, with an explicit version. Addressing is by FHSS sequence + radio sync
   word seeded from the UID — **no per-packet device ID**, unlike the legacy format.
6. **Rates are data** (`link/RateConfig`). Each rate is a table row (interval, hop
   interval, telemetry ratio, modulation params). Adding/removing a rate is a table edit.
7. **A single `RfScheduler` owns the per-tick sequence** — slot decision, FHSS advance,
   codec call, PHY TX/RX, turnaround. `Link` is only the connection lifecycle. The two
   timescales are kept separate so the microsecond-critical path has one owner and the
   modules can't race. The half-duplex TX↔RX turnaround (tens of µs) is budgeted inside
   the slot by the scheduler, which caps the achievable packet rate.
   **FHSS index semantics (avoid off-by-one desync):** the channel for tick `t` is
   `Fhss::at((t / fhssHopInterval) % seqLen)`, computed identically on both ends from the
   shared tick; `SyncPayload.fhssIndex` carries the **current** packet's index and the RX
   sets its index to match before advancing. **Slot precedence is Sync > Telemetry > Uplink**,
   but sync metadata is **piggybacked into RC frames where the 8-byte budget allows** (no RC
   gap); a dedicated `Sync` slot is used only when it doesn't fit — and since it is infrequent,
   the one displaced RC frame is negligible.
8. **Three-tier execution; the ISR stays tiny.** The hardware-timer ISR (`onTimerIsr()`) and DIO interrupt do only minimal work: latch the hardware timestamp and publish a **monotonic event counter** via a release-ordered atomic — never a bare boolean, which would coalesce back-to-back TX/RX events and lose one (no SPI, no logging, no allocation). The core-1 RF task (`RfTask::poll()`) observes the counter (acquire) and runs the slot decision, codec, and async native PHY commands. ISRs are bound to core 1. **Timing reference — one canonical point, used everywhere:** the PFD locks to **packet start, recovered as `RxPacket.timestampUs − airtime`**, compared against the expected scheduler tick. Airtime is the *full* on-air time — preamble + sync word + header + payload + CRC, per modulation/rate — precomputed per rate and frame size in `RateConfig.airtime8Us` / `airtime16Us`. Because it differs by frame length, feeding the PFD from a **single frame type (RC)** lets the constant bias self-cancel and avoids needing the per-frame correction at all.
9. **Channels use a latest-value mailbox, not a FIFO** (`util/Mailbox`). The RF side wants
   the freshest sticks, never a backlog of stale ones; FIFO rings (`util/RingBuffer`) are
   only for discrete events (config, bind, telemetry frames). Separate typed rings per
   direction: `AppToRf` (channels, config, bind) and `RfToApp` (stats, state, telemetry).
   **Ring overflow policy:** `push()` drops the *newest* item when full and counts it
   (`dropped()`). Control/bind rings are sized so they never overflow and a rising count is a
   fault (surfaced as `LinkStats.rxQueueDrops`); lossy telemetry streams may ignore drops.
   **Channel handoff — no lock on the RF path:** the mailbox is single-producer (Core 0) /
   single-consumer (Core 1). The RF task copies the snapshot out **once** per tick
   (`LatestValue::loadOrKeep`, lock-free + bounded) into a private local, then `ChannelPack`
   runs on that copy — nothing is shared during packing, so there is no mutex to block the RF
   deadline (a read/write lock would stall the RF core on the app core → missed slots). The
   whole channel set is copied as one snapshot (all-or-nothing; per-element atomics would
   allow a mixed-age set). On a rare failed read the consumer keeps its last-good snapshot
   (stale by one frame, never a stall).
10. **The nonce is mostly implicit but session-salted.** Because TX/RX are timing-locked and
    share the FHSS index, the per-packet nonce is derived from `(session_salt, packet_counter,
    fhss_index)` both sides track — NOT spent as on-air bytes in the 8-byte frame. Concretely,
    the 96-bit nonce (for IETF ChaCha20 / AES-CCM) is constructed by structured byte-concatenation
    to completely eliminate any bit-level overlap risks: `[4-byte session_salt][6-byte packet_counter][2-byte fhss_index]`.
    Two caveats that make or break it:
    - **Counter width:** `packet_counter` must be wide enough never to wrap within a session
      — a 16-bit counter wraps in ~65 s at 1 kHz and would reuse nonces. Use ≥32-bit, derived
      from the synchronized tick (not a received-packet count, so TX/RX stay aligned through loss).
    - **Salt uniqueness:** the salt only protects if actually unique, and **RP2040 has no
      hardware RNG** (RP2350 has a TRNG). Seed `session_salt` from a **persisted monotonic
      reboot counter** in flash — **wear-leveled** (rotate cells / flash KV store, never
      rewrite one cell every boot) — not weak boot-time entropy.
    - **Session lifecycle:** the TX (time master) generates `session_salt` at connection start;
      it is exchanged in the Connect handshake (Bind/Connect frame); `packet_counter` is the
      synchronized tick, reset to 0 at connect. A reconnect is a NEW session with a fresh salt,
      so resetting the counter never reuses a (key, nonce) pair.
    Default integrity is the PHY hardware CRC; an optional "tiny keyed integrity" cipher
    (MAC-only) is a valid middle ground short of full AEAD.
11. **Slot deadline vs. timer tick — prepare early, commit on time.** Task-context prep
    (encode / seal / SPI) takes time, so the scheduler timer fires `prepareLeadTimeUs` BEFORE
    the slot boundary (`slotStartUs`): the frame is prepared into the radio FIFO during the
    lead time and the `startTx`/`startRx` command is issued *before* the hardware slot
    deadline. Otherwise task wake-latency becomes RF timing jitter. Missed deadlines are
    counted in `LinkStats.missedDeadlines` as a timing-health signal.

---

## 4. Data flow

Driven by `RfScheduler` on every timer tick (the central sequence both roles follow):

A. **ISR Context (`onTimerIsr` / DIO interrupts):**
   1. Timer fires at `RateConfig.intervalUs` → Capture timer tick timestamp, set timer flag.
   2. DIO pin interrupt fires (TX-done or RX-done) → Capture DIO timestamp, set DIO flag.
   *No SPI transactions, codec work, or state machine updates occur here.*

B. **Task Context (`RfTask::poll()` on Core 1):**
   1. When the timer flag is observed, the task:
      a. Increments the packet tick counter.
      b. Picks the slot for this tick (`slotForTick`): Uplink / Telemetry / Sync / Bind / Idle.
      c. Decides if a frequency hop is due (`tick % RateConfig.fhssHopInterval == 0`). If so, advances the FHSS sequence to the next RF channel frequency.
      d. Asks the codec to encode/decode the frame type for the active slot.
      e. Triggers async PHY operations (`startTx` or `startRx`) on the active frequency.
   2. When the DIO RX-done flag is observed:
      a. Task calls `readRx` to drain the packet over SPI.
      b. Normalizes the RX timestamp by subtracting the known packet airtime (dependent on frame length).
      c. Feeds the normalized arrival offset into the PFD to correct the timer period.
      d. Relays data to the lifecycle manager (`Link`) to update connection state and `LinkStats`.

**TX role specifics:** pull newest channels from the `AppToRf` mailbox; pack
(`ChannelPack`); build + seal (`ICipher`) the OTA frame; transmit. On a Telemetry slot,
arm RX (turnaround budgeted) to receive the downlink.

**RX role specifics:** on non-Telemetry slots, dwell in RX; on RX-done, `ICipher::open` + validate; dispatch by `OtaType`. RC payload → `ChannelPack::unpack` → `RfToApp` mailbox → `CrsfAdapter` emits to the FC on arrival; loss/age → failsafe. Sync frames seed the FHSS index / rate so the RX stays locked. On a Telemetry slot, the RX role switches to TX: pull the latest telemetry from the `AppToRf` queue, build + seal an `OtaType::TlmDown` frame, and transmit it.

**Cold-start acquisition (bounded):** before lock, the RX does not know the hop phase, so it
dwells on a single UID-derived **acquisition channel**; the TX emits a sync/acquisition beacon
on that channel at a known period, so **worst-case lock time ≤ the beacon period** (independent
of sequence length). The first received Sync packet conveys the current FHSS index + rate,
after which the RX jumps into the sequence and the PFD takes over. (M3 brings this up on a
fixed channel before FHSS is enabled at M4.)

### Link statistics (one source of truth)

- **Expected count** = uplink-slot ticks only — Telemetry / Sync / Idle slots are excluded so a
  high telemetry ratio doesn't read as packet loss.
- **LQ** = received ÷ expected over a sliding window of **100 uplink slots** (`LqTracker<100>`);
  a missed slot counts as not-received.
- **RSSI / SNR** = exponential moving average over received frames (SNR is LoRa-only; 0 in FLRC).
- **Failsafe threshold** = N consecutive missed uplink slots, debounced (§8.1).
- `missedDeadlines` and `rxQueueDrops` surface timing / queue health.

The OSD, dynamic power (M6), and failsafe all read these, so the *same* expected-count source
must be used everywhere or they disagree.

---

## 5. Directory layout

```
lib/xlrs/
  phy/      IRadioPhy.h  Sx1280NativePhy.{h,cpp}
  timing/   Pfd.h  HwTimer.{h,cpp}
  fhss/     Fhss.h
  ota/      OtaPacket.h  ChannelPack.h
  crypto/   ICipher.h
  link/     Link.h  RfScheduler.h  RateConfig.h
  util/     RingBuffer.h  Mailbox.h
apps/
  tx/main.cpp   rx/main.cpp
test/
  test_xlrs_native/            (Unity tests for pure logic)
```
Production role mains are `apps/tx/main.cpp` and `apps/rx/main.cpp`.

---

## 6. Build & test

- Pico SDK CMake builds `xlrs_tx` and `xlrs_rx`.
- Host tests are kept as pure-logic coverage and should move to a CMake/CTest runner.
- Hardware soak tooling should watch RX serial for dropouts over long runs.

---

## 7. Implementation roadmap (milestones)

- **M0 — Scaffold:** layer contracts (headers), pure-logic implementations
  (channel packing, FHSS sequence, PFD math, ring buffer), native test coverage.
- **M0.5 — Legacy safety patch:** superseded by the production cutover.
- **M1 — PHY:** `Sx1280NativePhy` async (DIO-driven TX/RX, freq/power). Bench loopback.
- **M2 — Timing:** `HwTimer` (RP2040/RP2350) + integrate `Pfd`; TX master cadence, RX lock.
  **Acceptance criteria (measured, with targets):** max scheduling jitter, missed-deadline
  rate, RX lock convergence time, drift tolerance (ppm), telemetry-slot turnaround margin.
- **M3 — Link bring-up:** `Link` state machine on fixed frequency first (bind, connect,
  connected, failsafe, LinkStats). Cut RC over single-channel to validate end-to-end.
- **M4 — FHSS:** enable hopping + sync packet carrying fhss index/rate/tlm ratio.
- **M5 — Telemetry slotting:** deterministic downlink; CRSF `LINK_STATISTICS` to FC.
- **M6 — Rates + dynamic power:** rate switching via `RateConfig`; power control driven by
  **LQ first, RSSI second, with hysteresis** (RSSI-only control is weak).
- **⚠ RF compliance gate (before any high-power / range testing):** the FHSS region channel
  tables and TX power are firmware-controlled — review against the target region (FCC §15.247 /
  ETSI EN 300 328: dwell time, occupied bandwidth, EIRP) before transmitting above bench levels.
  Certification stays a non-goal, but these firmware knobs gate it.
- **M7 — Cutover:** complete. New role mains replaced the legacy link; duplicated identity,
  crypto, and OTA packet-format paths were removed. Recoverable via git history if needed.
- **M8 (forked) — Security:** `AeadCipher` if pursuing the secure-link wedge.

---

## 8. Decisions

**Resolved**
1. **Failsafe default = `NoPulses`** — precise CRSF contract on link loss:
   1. On loss detection (debounced: N consecutive missed uplink slots, never a single miss),
      **immediately stop `RC_CHANNELS_PACKED`** — Betaflight/INAV key RXLOSS on the *absence*
      of RC frames, so this is what triggers FC failsafe and lets the FC own policy.
   2. *Optionally* keep emitting `LINK_STATISTICS` with LQ→0 for a short grace window so the
      OSD shows the degradation — **only if validated** that the target FC still enters RXLOSS
      without RC frames (true on Betaflight; verify per FC). If unsure, stop everything.
   3. After the grace window, **stop all CRSF output** unless explicitly configured otherwise.
   `Hold` (emit preset positions) is **opt-in, never the default.**
2. **Link UID = 8 bytes (64-bit).** Two distinct IDs, do not conflate:
   - **Link UID** (seeds FHSS sequence + sync word) is **derived from the binding phrase**
     (`hash(phrase)` → 8 bytes) so a bound TX/RX compute the *same* value and hop together.
     This preserves the phrase-based auto-bind UX (`QP-XXXX-…`).
   - **Device serial** = the Pico hardware flash unique ID (`pico_get_unique_board_id`,
     8 bytes) — used for identity / model-match / logging, NOT for FHSS addressing.
   `Fhss::generate()` folds the 8-byte UID into its PRNG seed (M4 detail).

**Deferred (non-blocking)**
3. **Crypto fate:** ship `NullCipher`; decide AEAD vs MAC-only vs none at M8. The pluggable
   `ICipher` layer means this blocks nothing.
