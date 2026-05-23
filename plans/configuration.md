# XLRS Link Core Configuration

This document provides a comprehensive guide to the compile-time (build) and runtime configurations of the clean-slate XLRS 2.4 GHz link core.

---

## 1. Build-Time (Compile-Time) Configuration

Build-time parameters are hardcoded into the binary during compilation. They dictate hardware targets, pin connections, SPI/UART assignments, and default software features.

### A. Environment Selection (`platformio.ini`)
Firmware builds are configured into isolated environments to target different hardware blocks:

* **`[env:xlrs_tx]` / `[env:xlrs_rx]`**
  * Production TX/RX firmware using the `lib/xlrs/` core with `RadioLibPhy`.
* **`[env:xlrs_tx_native]` / `[env:xlrs_rx_native]`**
  * Production TX/RX firmware using the `lib/xlrs/` core with `Sx1280NativePhy`.
* **`[env:rc-rp2350]`**
  * Targets the RC handset (RP2350 Pico 2). Configures native stick ADC inputs and display peripherals.
* **`[env:native]`**
  * Host environment. Compiles the new core's pure-logic layers for Unity unit tests on the dev
    machine (no hardware).

---

### B. Hardware & Pinout Macros (Build-Flags)
Passed as preprocessor flags (`-D`) in `platformio.ini` to map physical microcontroller pins to the `IRadioPhy` layer:

#### 1. Radio Module SPI Configuration (SX1280)
* **`SX128X_SPI_SCK`** (e.g. `18`) — SPI clock pin.
* **`SX128X_SPI_MOSI`** (e.g. `19`) — SPI Master-Out Slave-In pin.
* **`SX128X_SPI_MISO`** (e.g. `16`) — SPI Master-In Slave-Out pin.
* **`SX128X_SPI_CS`** (e.g. `17`) — SPI Chip Select pin.
* **`SX128X_SPI_BUSY`** (e.g. `20`) — SX1280 busy indicator pin (critical for state transition timing).
* **`SX128X_SPI_DIO1`** (e.g. `21`) — DIO1 pin (configured as a hardware interrupt to signal TX-done or RX-done immediately).
* **`SX128X_SPI_RST`** (e.g. `22`) — Reset pin.
* **`SX128X_RXEN`** (e.g. `14`) / **`SX128X_TXEN`** (e.g. `15`) — Front-end LNA/PA active switching pins.

#### 2. Handset Interface Configuration
* **`UART_PROTOCOL_TX`** (e.g. `8`) / **`UART_PROTOCOL_RX`** (e.g. `9`) — UART serial routing pins for communicating with the handset controller using CRSF.
* **`INTERNAL_ADC`** — Set to `1` on RP2350 targets to enable stick ADC inputs.
* **`RC_STICK_ADC_12BIT`** — Set to `1` to enable 12-bit ADC reading resolution.
* **`CRSF_CORE1_CHANNELS`** — Set to `1` to delegate CRSF channel generation tasks to Core 1.

---

### C. Feature Control Macros
Configure high-level software settings:

* **`DEFAULT_BINDING_PHRASE`** (e.g. `"Kikobot-02"`)
  * String literal used to seed the default **Link UID** if a customized bind phrase is not active.
* **`DEBUG_LINK_STATS`**
  * Set to `1` to periodically output loop timings, serial rates, and link statistics over the hardware serial interface once per second.

---

## 2. Runtime Configuration

Runtime configurations are dynamically resolved on boot, computed mathematically, or negotiated over-the-air between the TX and RX.

### A. Core Identifiers
* **Link UID (8 bytes / 64-bit)**
  * **Source:** Derived by hashing the binding phrase with 64-bit **FNV-1a** (fast, non-cryptographic):
    $$\text{Link UID} = \text{FNV-1a}(\text{Binding Phrase})$$
  * **Implemented:** `linkUidFromPhrase()` in [`lib/xlrs/link/Uid.h`](../lib/xlrs/link/Uid.h)
    (big-endian split of the hash). Verified in the native suite: deterministic, phrase-sensitive,
    and the bind property (same phrase ⇒ identical FHSS sequence; else isolation).
  * **Role:** Seeds the FHSS pseudorandom hop sequence and forms the radio's hardware Sync Word
    (`syncWordFromUid()` — the **low** 16 bits, since FNV-1a's high bits don't avalanche for
    phrases differing only in the trailing byte; `IRadioPhy::setSyncWord`) to prevent
    cross-operator interference.
* **Device Serial (8 bytes / 64-bit)**
  * **Source:** Read directly from the microcontroller's unique board ID (`flash_get_unique_board_id()`).
  * **Role:** Identifies the specific hardware board for logging and remote diagnostic operations. It is *never* used for link-addressing or FHSS seeding to preserve bind-phrase flexibility.

---

### B. Dynamically Swappable Rates (`RateConfig`)
Packet rates are stored in a static constant lookup table `kRates` inside [RateConfig.h](file:///Users/sawan/projects/xlrs-dev/xlrs/lib/xlrs/link/RateConfig.h). Rates can be switched dynamically at runtime:

```cpp
struct RateConfig {
    const char* name;
    uint16_t    intervalUs;       // Packet period / timer cadence
    uint8_t     fhssHopInterval;  // Frequency hop interval (packets)
    uint8_t     tlmRatioDenom;    // Telemetry ratio (e.g., 1:8 slots)
    Modulation  modulation;       // LoRa vs. FLRC modulation
    float       bwKHz;            // LoRa bandwidth (0.0f for FLRC)
    uint8_t     sf;               // LoRa Spreading Factor (0 for FLRC)
    uint8_t     cr;               // Coding Rate
    uint16_t    flrcBitrateKbps;  // FLRC bitrate (0 for LoRa)
    uint8_t     payloadLen;       // Nominal payload size (8 bytes)
    uint16_t    airtime8Us;       // Pre-calculated 8-byte frame airtime
    uint16_t    airtime16Us;      // Pre-calculated 16-byte frame airtime
};
```
* The TX requests a rate change via `Link::requestRate(idx)`; the new `rateIndex` rides in the
  periodic `OtaType::Sync` beacon and the RX adopts `kRates[idx]` on decode (`Link::service`). All
  current rows share `fhssHopInterval = 4`, so hop timing is unchanged across a switch and the link
  survives it without re-acquiring (a future rate with a different hop interval would instead trigger
  a brief re-acquisition). Verified in the native sim.

---

### C. Frequency-Hopping Spread Spectrum (FHSS) Table
* **Channel Sequence:** Generated using the `Fhss` class seeded with the lower 32 bits of the
  **Link UID** (`fhssSeedFromUid()` in `Uid.h` → `Fhss::generate()`). Uses a seeded Linear
  Congruential Generator (LCG) combined with a Fisher-Yates shuffle to build a balanced sequence
  of channel indices.
* **Frequency Mapping:** Channel indices map to a regional frequency table —
  [`fhss/channels_2g4.h`](../lib/xlrs/fhss/channels_2g4.h) (⚠ placeholder set; the shipping table
  must be the regulator-approved channels per the RF compliance gate). `fhssFreqForIndex()`.
* **Cadence:** Frequency advances on hop boundaries: `tick % RateConfig.fhssHopInterval == 0`.
* **Acquisition (Link.{h,cpp}, M4):** the TX sends an `OtaType::Sync` beacon at sequence
  position 0 (the acquisition channel); an unlocked RX dwells on that channel until it catches a
  Sync, adopts the conveyed `fhssIndex`, locks, and follows the sequence. On link loss the RX
  unlocks and falls back to the acquisition channel to re-sync — worst-case lock time ≤ one
  sequence cycle. Verified in the native sim (acquire → hop → loss → re-acquire).

---

### D. Cipher Nonce Layout (`ICipher` Integration)
When pluggable security (`AeadCipher` at M8) is active, a unique 96-bit nonce is constructed dynamically on both sides using structured byte-concatenation to completely eliminate bit-level overlaps:

$$\text{Nonce} = \big[\,4\text{-byte } \text{session\_salt}\,\big] \,\, \big|\big| \,\, \big[\,6\text{-byte } \text{packet\_counter}\,\big] \,\, \big|\big| \,\, \big[\,2\text{-byte } \text{fhss\_index}\,\big]$$

* **`session_salt`:** 32-bit random salt negotiated during connection handshake. Seeded from weak hardware entropy combined with a flash-persisted monotonic boot counter.
* **`packet_counter`:** 48-bit rolling tick counter to prevent wrap-arounds.
* **`fhss_index`:** 16-bit current hop index.

**Implemented (M8):** `AeadCipher` ([`crypto/AeadCipher.h`](../lib/xlrs/crypto/AeadCipher.h)) =
ChaCha20-Poly1305 ([`crypto/Chacha20Poly1305.h`](../lib/xlrs/crypto/Chacha20Poly1305.h), verified
against the RFC 8439 §2.8.2 known-answer vector) with a **4-byte truncated tag**. The `Nonce96`
is the 12-byte ChaCha20 IETF nonce, built as `Nonce96::build(session_salt, packet_counter, fhss_index)`.
It's wired into the Link RC path opt-in (`Link::setCipher`) — `seal()` on TX `slotSend`, `open()`
on RX `service`, defaulting to `NullCipher` (plaintext) so it's off unless enabled. Verified in the
native sim: sealed RC round-trips; tamper and wrong key are rejected (channels never decode).
⚠ Scope: M8 seals the **RC payload**; sealing Sync/Telemetry frames and negotiating a per-session
random `session_salt` at Connect (vs. the fixed sim value) are follow-ups.

---

### E. Clock Synchronization (Pfd) & Failsafes
* **Timing Corrections:** The Phase-Frequency Detector (`Pfd`) tracks crystal ppm offsets using a Proportional-Integral (PI) loop (`Kp=1/4`, `Ki=1/256`). It takes the normalized arrival offset (`actual_arrival - expected_arrival - precomputed_airtime`) and nudges the local hardware timer registers dynamically to null out frequency drift.
* **Failsafe Operations:** `NoPulses` (stops the CRSF stream → FC failsafe) or `Hold` (holds last
  valid sticks). Implemented in [`Link.{h,cpp}`](../lib/xlrs/link/Link.cpp): RX state machine
  Disconnected → Connecting → Connected → Failsafe; falls to `Failsafe` after `FAILSAFE_MISS`
  consecutive missed uplink slots, and `Link::outputActive()` returns false under `NoPulses` so
  the app stops emitting CRSF. Verified in the native two-node sim (connect → channel flow →
  induced loss → NoPulses failsafe → recovery; mismatched phrase never connects).

---

### F. Telemetry Slotting & Link Statistics (M5)

* **Slot schedule (both ends agree per tick):** `pos = (tick / fhssHopInterval) % seqLen`.
  `pos == 0` → **Sync** (TX→RX beacon on the acquisition channel); else `tick % tlmRatioDenom == 0`
  → **Telemetry** (RX→TX downlink); else → **Uplink** (TX→RC→RX). Each slot has exactly one sender
  and one receiver, computed identically on both ends — **collision-free TDM**, no legacy "holdoff."
* **Telemetry downlink (`OtaType::TlmDown`, RX→TX):** on telemetry slots the RX transmits its
  uplink measurement (LQ / RSSI / SNR) so the TX/handset can show "how well the craft hears me."
  The TX tracks the downlink reception ratio as `LinkStats.lqDown`.
* **Link statistics to the FC:** the RX packs `LinkStats` into a CRSF `LINK_STATISTICS` (0x14)
  10-byte frame via `buildCrsfLinkStatistics()`
  ([`app/CrsfLinkStats.h`](../lib/xlrs/app/CrsfLinkStats.h)). Uplink LQ is counted over **uplink
  slots only** (Sync/Telemetry excluded, so a high telemetry ratio isn't read as packet loss).
  Verified in the native sim (downlink reaches the TX without disturbing uplink RC; CRSF bytes correct).

---

### G. Dynamic TX Power (M6)

* **Policy:** LQ-first, RSSI-second, with hysteresis ([`link/DynamicPower.h`](../lib/xlrs/link/DynamicPower.h)).
  Raise power when uplink LQ sags (`< 80`); lower it only when LQ is high (`>= 99`) **and** there is
  RSSI margin (`> -65 dBm`); hold in the deadband between. A step requires N consecutive readings, so
  the controller doesn't oscillate (RSSI-only control would be twitchy).
* **Inputs:** the TX feeds the controller the RX-reported uplink LQ/RSSI from `TlmDown` frames and
  applies the result via `IRadioPhy::setOutputPowerDbm`. Verified as a unit (raise→max / lower→min /
  deadband-hold) plus the TX↔power wiring in the sim (power steps down under a strong link). ⚠ The sim
  does not close the power→RSSI feedback loop (MockPhy RSSI is fixed) — settling is validated on hardware.
