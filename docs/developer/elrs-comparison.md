# XLRS And ExpressLRS Engineering Comparison

This note compares XLRS with ExpressLRS as an engineering reference point. It is
not a compatibility promise and it is not a full audit of the current ExpressLRS
codebase. The XLRS side is grounded in this repository's current code.

ExpressLRS is useful as a reference because it proves the main problem classes
that matter in modern RC links: low-latency scheduling, efficient OTA packing,
FHSS, failsafe behavior, telemetry, power control, configuration, and field
diagnostics. XLRS solves a smaller, more controlled version of that problem.

## Scope And Platform

ExpressLRS is designed around a broad hardware ecosystem. That ecosystem has a
real advantage: many transmitters, receivers, vendors, and board variants can
share one firmware family. The tradeoff is that portability tends to increase
HAL, target, and conditional-build complexity.

XLRS is intentionally narrower. The current firmware is a Pico SDK project with
two role binaries:

- `xlrs_tx`, implemented by [apps/tx/main.cpp](../../apps/tx/main.cpp).
- `xlrs_rx`, implemented by [apps/rx/main.cpp](../../apps/rx/main.cpp).

The radio boundary is still abstracted through
[IRadioPhy](../../lib/xlrs/phy/IRadioPhy.h), and the current production PHY is
[Sx1280NativePhy](../../lib/xlrs/phy/Sx1280NativePhy.h). That keeps the upper
link code testable and avoids leaking SX1280 details upward, but this repo does
not currently try to support the wide target matrix that ExpressLRS supports.

## Execution Model

XLRS uses a dual-core split in both role apps:

- Core 0 handles application-facing work: controller UART on TX, CRSF output on
  RX, status logs, watchdog service, and mailboxes.
- Core 1 runs `rf_core_main()`, initializes the RF stack, and repeatedly calls
  `g_scheduler.poll()`.

This is visible in [apps/tx/main.cpp](../../apps/tx/main.cpp) and
[apps/rx/main.cpp](../../apps/rx/main.cpp), where `multicore_launch_core1()` is
used to start the RF core.

The scheduler contract is in
[RfScheduler.h](../../lib/xlrs/link/RfScheduler.h): timer and DIO interrupt paths
publish small atomic event counters, while task context performs slot work,
SPI/PHY operations, and packet processing. This reduces application jitter
coupling, but it does not by itself prove zero jitter. Hardware timing still has
to be measured on real boards.

## OTA Channel Packing

XLRS currently carries 8 OTA RC channels:

```cpp
static constexpr uint8_t RC_CHANNELS = 8;
```

That constant lives in [Link.h](../../lib/xlrs/link/Link.h). The standard RC
path packs each channel as an 11-bit value using
[ChannelPack.h](../../lib/xlrs/ota/ChannelPack.h). For 8 channels,
`packedSize(8)` is 11 bytes, and [OtaCodec.h](../../lib/xlrs/ota/OtaCodec.h)
adds a 1-byte OTA header. So the standard RC frame is:

```text
1-byte OTA header + 11-byte packed RC payload = 12 bytes
```

XLRS also has a compact 8-byte RC frame in
[OtaFrameShrink.h](../../lib/xlrs/ota/OtaFrameShrink.h):

```text
1-byte OTA header + 5 x 10-bit primary channels + 6-bit sequence/ack field = 8 bytes
```

The compact path is not a general ELRS-style auxiliary-channel multiplexing
scheme. In [Link.cpp](../../lib/xlrs/link/Link.cpp), TX only enters compact mode
after 25 consecutive eligible RC frames. Eligibility requires channels 5-7 to be
centered and all channels to be even so the 10-bit compact representation is
lossless for the transmitted primary values. The first ineligible frame exits
compact mode immediately.

That means the present XLRS tradeoff is:

| Path | Current Behavior |
| --- | --- |
| Standard RC | 8 uniform 11-bit channels in a 12-byte OTA frame. |
| Compact RC | 5 primary 10-bit channels plus sequence/ack in an 8-byte OTA frame, used only when eligibility is stable. |

This differs from ExpressLRS-style hybrid packing, which is optimized around
very compact primary controls plus special treatment for auxiliary channels.

## Slotting, FHSS, And Telemetry

XLRS has one authoritative slot decision in
[Link::slotForTick](../../lib/xlrs/link/Link.cpp). The schedule is:

```text
pos == 0                       -> Sync slot
else tick % tlmRatioDenom == 0 -> Telemetry slot
else                           -> Uplink slot
```

The rate table in [RateConfig.h](../../lib/xlrs/link/RateConfig.h) defines
packet interval, hop interval, telemetry ratio, modulation, and representative
airtime values. `RfScheduler` asks `Link` for the slot and frequency, then arms
TX or RX through `IRadioPhy`.

Telemetry is bidirectional at the OTA layer:

- RX to TX downlink link telemetry is encoded as `OtaType::TlmDown`.
- MSP-like chunks can be carried as `OtaType::Msp` through
  [StubbornTelemetry.h](../../lib/xlrs/link/StubbornTelemetry.h).

The tests in [telemetry_tests.cpp](../../test/telemetry_tests.cpp) verify
stubborn telemetry reassembly and ensure MSP bursts do not count as link-quality
loss in the simulated link.

## Binding And Link Identity

XLRS binding is phrase-based. [Uid.h](../../lib/xlrs/link/Uid.h) derives an
8-byte Link UID with 64-bit FNV-1a. That UID drives:

- FHSS seed selection.
- SX1280 sync word derivation.
- UID CRC validation in sync frames.

The interface documentation describes the runtime behavior in
[docs/interfaces/index.md](../interfaces/index.md): both modules must use the
same binding phrase, and `UART_MSG_CMD_SET_BIND_TX` can update the TX binding
phrase at runtime. Runtime RX binding update is still reserved/not implemented.

The native tests in [core_tests.cpp](../../test/core_tests.cpp) verify that equal
phrases produce the same UID/FHSS behavior and different phrases isolate links.
[link_tests.cpp](../../test/link_tests.cpp) also verifies that mismatched
phrases do not connect in the two-node simulation.

## Crypto And Authentication

XLRS defaults to plaintext through `NullCipher` in
[ICipher.h](../../lib/xlrs/crypto/ICipher.h). That means the default shipped
behavior is still non-cryptographic, with integrity left to the PHY hardware CRC.

The repo also includes an opt-in [AeadCipher](../../lib/xlrs/crypto/AeadCipher.h)
based on ChaCha20-Poly1305 with a truncated 4-byte tag. The nonce layout is:

```text
[4-byte session_salt][6-byte packet_counter][2-byte fhss_index]
```

This is tested in [crypto_tests.cpp](../../test/crypto_tests.cpp) and integrated
in link simulation by [link_tests.cpp](../../test/link_tests.cpp), including a
wrong-key rejection case.

Important current limitation: `Link::setSessionSalt()` exists, but production
session-salt negotiation is not implemented in the app flow yet. The encrypted
link path is therefore present and tested as a core capability, not a complete
fielded pairing/security lifecycle.

## Failsafe Contract

XLRS has two configured failsafe modes:

- `NoPulses`, the default.
- `Hold`, an explicit option.

The mode is represented by `FailsafeMode` in
[Link.h](../../lib/xlrs/link/Link.h), persisted in RF config, and applied by the
RX app. In [apps/rx/main.cpp](../../apps/rx/main.cpp), CRSF RC frames are queued
only when `outputActive` is true. Under default `NoPulses`, `outputActive()`
returns false when the link is not connected, allowing the flight controller to
enter RXLOSS based on missing RC frames.

RX still emits CRSF `LINK_STATISTICS` periodically from the app path. That is a
separate frame from CRSF RC channel output.

## Power Control

XLRS dynamic power is implemented in
[DynamicPower.h](../../lib/xlrs/link/DynamicPower.h). It is LQ-first and
RSSI-second:

- Jump to max power when telemetry is lost or LQ is critically low.
- Raise power when LQ is weak.
- Lower power only when LQ is strong and RSSI has margin.
- Use hysteresis to avoid rapid oscillation.

[Link.cpp](../../lib/xlrs/link/Link.cpp) applies the policy on TX when downlink
telemetry reports the RX-side uplink view, and `RfScheduler` syncs the requested
power into the PHY with `setOutputPowerDbm()`.

## Testability

XLRS has a host-native test suite under [test/](../../test). The tests cover:

- UID and FHSS determinism.
- OTA channel packing and compact 8-byte packing.
- PFD timing math.
- Link connect, failsafe, recovery, rate switch, and downlink telemetry.
- AEAD encryption, tamper/wrong-key rejection, and crypto vectors.
- Dynamic power behavior.
- Stubborn telemetry transport.
- Mock PHY recovery scenarios.

This is a deliberate difference from target-only validation. Native tests cannot
prove RF timing, power integrity, oscillator behavior, or real SX1280 register
behavior, but they do keep pure link logic fast to verify.

## Summary

| Dimension | ExpressLRS Reference Point | XLRS Current Code |
| --- | --- | --- |
| Hardware scope | Broad ecosystem across many commercial targets. | Narrow Pico SDK TX/RX firmware with a narrow `IRadioPhy` boundary. |
| Execution model | Target-dependent scheduling model. | Core 0 app work, Core 1 RF scheduler via `multicore_launch_core1()`. |
| Standard RC packing | Highly airtime-optimized hybrid layouts. | 8 uniform 11-bit channels in a 12-byte OTA frame. |
| Compact RC packing | Uses specialized auxiliary treatment in common modes. | Optional 8-byte compact frame for 5 primary channels when eligibility is stable. |
| Binding identity | Binding phrase separates links and seeds radio behavior. | 8-byte FNV-1a Link UID seeds FHSS/sync word and validates sync frames. |
| Crypto | Commonly treated as non-cryptographic link isolation. | Defaults to `NullCipher`; opt-in ChaCha20-Poly1305 core exists and is tested. |
| Failsafe | Ecosystem/config dependent behavior. | Default `NoPulses` stops CRSF RC output; `Hold` is explicit. |
| Test strategy | Strong hardware/field validation culture. | Host-native simulation and unit tests for pure logic, with hardware validation still required. |

