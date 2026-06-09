# Architecture

This describes the **active PlatformIO firmware** under `src/` — the image that
builds and flashes. For the earlier clean-slate layered design (the `lib/xlrs/`
core), see [the design-intent appendix](#appendix-legacy-xlrs-core-design); that
tree is reference-only and is not what runs on hardware.

## Overview

XLRS is a half-duplex 2.4 GHz control link between a paired TX and RX, both
RP2040/Pico boards driving Semtech SX1280 radios through
[RadioLib](https://github.com/jgromes/RadioLib).

```text
CRSF handset ─▶ TX (time master) ─▶ SX1280 LoRa + FHSS ─▶ RX ─▶ CRSF flight controller
                                       (2.4 GHz)
                downlink telemetry ◀──────────────────────────  link statistics
```

One `src/main.cpp` builds into both roles; the role is fixed at compile time:

- `-DLORA_TX_ROLE=1` → reads CRSF channels from the handset, transmits RC
  uplink, listens on telemetry slots, drives status output.
- `-DLORA_RX_ROLE=1` → receives RC uplink, disciplines its clock to the TX,
  emits CRSF RC + link statistics to the flight controller, gates output on
  link loss.

## Components

| Component | Files | Responsibility |
| --- | --- | --- |
| Role application | `src/main.cpp` | Radio bring-up, tick loop, CRSF in/out, binding CLI, failsafe gating, status |
| Protocol helpers | `include/lora_link/protocol.h`, `src/protocol.cpp` | OTA frame/sync codec, CRC16, UID/FHSS derivation, CRSF frames, RC packing, config records |
| RF scheduler | `include/lora_link/rf/scheduler.h`, `src/lora_link/rf/scheduler.cpp` | Per-tick slot selection, FHSS advance, connection state machine, RX clock discipline (PFD) |
| Control frame | `include/lora_link/control/frame.h`, `src/control_frame.cpp` | Control-frame helpers |
| CRSF / UART libs | `lib/CrsfProtocol/`, `lib/crsfSerial/`, `lib/UARTProtocol/`, `lib/crc8/` | Wired-transport encode/decode |

## Timing model

The TX is the **time master**. Its hardware timer fires every `intervalUs`
(4000 µs at `L250`, 10000 µs at `L100`); each firing is a **tick**. The interrupt
does only minimal work — it latches a timestamp and posts an event — while the
main loop services ticks: pick the slot, advance FHSS if due, encode, and start
the SX1280 TX/RX via RadioLib. The DIO1 line signals TX-done / RX-done.

The RX runs an alarm-driven timer of its own and **disciplines it to the TX**
with a phase/frequency detector (PFD). Each received frame's arrival offset feeds
the PFD, which nudges the RX tick period (and a frequency-offset trim) so the two
ends stay phase-locked and hop together. The RX timer walks a small state machine
(`Disconnected → Tentative → Locked`) mirrored by the connection state
(`Disconnected → Tentative → Connected`).

## Tick / slot sequence

Each tick the scheduler chooses an action ("slot") for that tick:

1. **Sync** — periodically the frame carries an `OtaSyncPayload` (nonce, FHSS
   index, current/next rate, telemetry ratio, hop interval, TX tick) so the RX
   can acquire and stay aligned. The sync cadence is `kSyncFrameInterval`.
2. **Telemetry** — at `L100`, every `telemetryRatio`-th slot is a downlink
   window: the RX transmits link stats / telemetry, the TX briefly listens.
3. **RC uplink** — otherwise the TX sends packed RC channels.

FHSS advances every `fhssHopInterval` ticks. The channel for a hop is
`fhssChannelFor(uid, hop)` — a UID-seeded coprime-stride walk over 40 channels —
and the frequency is `2404 + channel*2` MHz. Both ends compute the same channel
from the shared tick/hop and the binding-derived UID.

## OTA frame

A frame is **33 bytes**: a 9-byte header, a 22-byte payload, and a CRC16-CCITT.
The header carries the `OtaType` (`Rc`, `Telemetry`, `Sync`), a 16-bit sequence,
and the 32-bit `uid_check` derived from the binding phrase. Decode rejects a
frame whose `uid_check` does not match the local binding or whose CRC fails, so a
receiver only accepts traffic from its bound partner.

RC payloads pack 16 channels at 11 bits each (CRSF raw range 172–1811). Frames
are **plaintext**: the CRC + `uid_check` provide integrity and link selection,
**not** authentication or confidentiality. See
[ota-protocol.md](ota-protocol.md).

## Binding identity

A binding phrase (1–32 chars) derives:

- an 8-byte **Link UID** (`deriveUid`),
- the SX1280 **sync word** (`syncWordFromUid`),
- the **FHSS sequence** (`fhssChannelFor`), and
- the 32-bit **`uid_check`** stamped into every frame.

Matching phrases on both ends bind the pair — there is no OTA pairing handshake.
The phrase and selected rate persist in flash (`ConfigRecord`, magic `LRL1`, with
a CRC). Binding can be changed over the USB CLI (`bind set …`) or the `rc.v1` /
CRSF binding-control path (CRSF frame type `0x7D`). See
[../crsf/binding.md](../crsf/binding.md).

## CRSF interfaces

- **TX side:** consumes `RC_CHANNELS_PACKED` (`0x16`) from the handset and can
  carry binding control (`0x7D`).
- **RX side:** emits `RC_CHANNELS_PACKED` (`0x16`) and `LINK_STATISTICS`
  (`0x14`) to the flight controller at 400000 baud.

See [../interfaces/rx-crsf.md](../interfaces/rx-crsf.md) and
[../interfaces/tx-controller-crsf.md](../interfaces/tx-controller-crsf.md).

## Failsafe

Loss-driven and FC-owned: when no valid uplink frame arrives within the failsafe
window (`kFailsafeTimeoutMs`, default 250 ms), the RX **stops emitting
`RC_CHANNELS_PACKED`**. Betaflight/INAV key RXLOSS on the absence of RC frames,
so this hands failsafe policy to the flight controller rather than holding stale
sticks. See [safety-and-failsafe.md](safety-and-failsafe.md).

## Diagnostics

`status` prints role, binding, link health (LQ, RSSI, SNR, good/bad frame
counts) and a wide set of scheduler counters (`SchedulerStats`): tick/missed-tick
counts, FHSS index, sync accept/reject, PFD events and phase shift, connection
promotions, queue drops, and CRSF output health. These are the same counters used
to reason about timing and acquisition on the bench. See
[timing-and-scheduler.md](timing-and-scheduler.md) and
[telemetry-and-link-stats.md](telemetry-and-link-stats.md).

---

## Appendix: legacy XLRS-core design

`lib/xlrs/` and `apps/tx`, `apps/rx` contain an earlier, more ambitious
clean-slate design: a strictly layered core (PHY / timing / FHSS / OTA / crypto /
link / util) with a dual-core split, a pluggable `ICipher` for a future AEAD
secure link, and Pico-SDK hardware adapters. That design is **not** the firmware
that flashes today — it is linted/unit-tested through the CMake project under
`test/` and kept for reference. Its layering, crypto/nonce reasoning, and PFD
tuning notes remain useful background, but where it disagrees with the `src/`
firmware above, **`src/` is authoritative**.
