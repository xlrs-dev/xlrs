# RC firmware — transient issues & required fixes

Tracking doc for the intermittent ("transient") issues seen on the RC board
(`rc-rp2350` / `rc-crsf` builds). Root cause theme: the dual-core design
(Core 0 = USB config/display/telemetry, Core 1 = sticks→channels→UART at 250 Hz)
shares mutable state across cores, and recent feature commits widened that
shared surface without matching synchronization.

Status legend: `OPEN` / `IN PROGRESS` / `FIXED` / `WONTFIX` / `NEEDS-VERIFY` /
`CONFIRMED` (validated against source/datasheet, fix not yet landed) / `RESOLVED`
(investigated and found correct / not-a-bug)
Severity: `HIGH` (can affect flight / crash) · `MED` · `LOW` (quality/maintainability)

---

## High severity

### F1 — Torn cross-core read of live config  · HIGH · OPEN
- **Where:** `src/rc_crsf_main.cpp:1712` (`rc_sync_runtime_from_g_rc_config`) writes
  `s_core1_config` via `memcpy` + `__dmb()` from Core 0; Core 1 reads it
  field-by-field in `loop1()` (`:718-770`).
- **Problem:** `__dmb()` is a memory barrier, not mutual exclusion. Core 1 can read
  a half-updated 132-byte struct → for one frame a stick maps to the wrong output
  (`channel_function[]`) → momentary servo/throttle twitch. Fires on every Apply /
  Save / Calibrate-finish.
- **Fix:** Make the snapshot atomic from Core 1's view. Preferred: Core 0 stages
  config + sets `volatile bool s_core1_config_dirty`; Core 1 copies into its own
  struct **once at the top of `loop1()`** (safe point between frames). Or guard the
  copy on both sides with a `critical_section_t` (pico/mutex.h already linked).
  Note: `loop1()` reads `s_core1_config` from **two** sites — the `cfg` pointer
  (`:718`) *and* the `&s_core1_config` arg passed to
  `internal_read_sticks_filtered()` (`:721`). The local-snapshot fix must redirect
  **both** to the per-frame copy, or it's only half-fixed.

### F2 — Proxy passthrough fights Core 1 for the UART · HIGH · OPEN
- **Where:** `rc_proto_enter_usb_uart_proxy` sets `proxy_mode_enabled` (`:1896`);
  Core 0 does raw `Serial2`↔USB passthrough (`:651-656`), but `loop1()` never checks
  the flag and keeps writing CRSF frames to `Serial2` at 250 Hz.
- **Problem:** Two cores write the same UART → interleaved bytes → corrupted
  ELRS/EdgeTX passthrough (exactly the case where Core 1 is sending channels).
  Two adjacent hazards in the same path: (1) `proxy_mode_enabled` is set `true`
  (`:1896`) but **never reset to `false`** anywhere — entering proxy is a one-way
  latch until reboot. (2) `rc_proto_enter_usb_uart_proxy()` re-calls
  `Serial2.begin()` (`:1891`) *while Core 1 is actively writing `Serial2` at
  250 Hz* — that re-init is itself a race, separate from the byte interleaving.
- **Fix:** (a) In `loop1()`, skip the UART write (early return) while
  `proxy_mode_enabled` is true; (b) add an exit path that clears the flag; and
  (c) park/quiesce Core 1 before the `Serial2.begin()` re-init in proxy entry.

### F6 — Flash write (EEPROM.commit) with no multicore lockout · HIGH · NEEDS-VERIFY
- **Where:** `rc_config_save()` → `EEPROM.commit()` (`lib/RCConfig/RCConfig.cpp:132`)
  called at runtime from `rc_proto_save` (`:1739`) and `rc_proto_cal_finish` (`:1765`).
- **Problem:** On RP2040/RP2350, `commit()` does `flash_range_erase` + `program`.
  During flash erase/program XIP is inaccessible; if Core 1 fetches an instruction
  from flash mid-erase → hard fault / hang. No `flash_safe_execute` /
  `multicore_lockout` / core-park is present. Strong candidate for "random lockup
  when saving config or finishing calibration."
- **Verify:** Confirm whether the installed arduino-pico EEPROM lib already parks
  Core 1 (`flash_safe_execute`) internally for this core version.
- **Fix (if not handled):** Wrap save in `flash_safe_execute`, or park Core 1
  (`multicore_lockout_start_blocking` + `..._end_blocking`) around the commit. The
  setup-time save (`:568`) is safe (Core 1 not started yet).

---

## Medium severity

### F3 — Filter reset is a cross-core data race · MED · OPEN
- **Where:** `internal_adc_reset_stick_filter()` called from Core 0 inside
  `rc_sync_runtime_from_g_rc_config()` (`:1717`) clears
  `s_internal_stick_lpf_init_mask` / `s_frame_blend_init_mask` — plain (non-volatile)
  statics owned/used by Core 1 (`:88-93`).
- **Problem:** Unsynchronized write to another core's state (UB); can drop/reset the
  filter unexpectedly.
- **Fix:** Fold the reset into the F1 Core-1-reload path (Core 1 resets its own
  filter when it picks up new config).

### F7 — ADC treated as 12-bit but resolution never set · MED · NEEDS-VERIFY
- **Where:** `INTERNAL_ADC_MAX = 4095` (`:75-76`); `analogRead()` used directly
  (`:95`, `:2058-2062`); `analogReadResolution(12)` is never called anywhere.
- **Problem:** arduino-pico's default `analogRead` resolution is 10-bit (0–1023).
  If not overridden, raw values are ~¼ of the assumed 0–4095 range → stick range
  compression / scale mismatch vs the 12-bit calibration defaults
  (`RCConfig.h:21-23`). User calibration partially masks it, but defaults and scaling
  assumptions are wrong.
- **Verify:** Read back actual `analogRead` range on hardware.
- **Fix:** Call `analogReadResolution(12)` in ADC init (`init_internal_adc`).

### F8 — Debug build flag mismatch vs production · MED · OPEN
- **Where:** `platformio.ini` `[env:rc-rp2350-debug]` (`:134-138`) sets `INTERNAL_ADC`
  + `CRSF_CORE1_CHANNELS` + `DEBUG_LINK_STATS` but **omits `RC_STICK_ADC_12BIT`**,
  which production `[env:rc-rp2350]` (`:118-120`) has.
- **Problem:** Debug build uses the 16-bit scaling path (`RC_ADC_CENTER=16384`,
  calib defaults 2917/23420/13199) while production uses 12-bit raw
  (`RC_ADC_CENTER=2048`, 365/2927/1650). Behavior and stored calibration differ →
  can't faithfully reproduce production transient issues on the debug build.
- **Fix:** Add `-DRC_STICK_ADC_12BIT=1` to the debug env so it matches production.

---

## Low severity / quality

### F4 — Debug instrumentation on hot RX path · LOW · OPEN
- **Where:** `lib/CrsfProtocol/CRSFSerialConnector.cpp:108-117` logs every RX byte
  (`debug_log_ndjson` + `Serial.printf("[CRSF RX RAW] ...")`), gated by
  `s_debug_rx_logs < 32` or verbose mode.
- **Fix:** Compile out behind a debug flag; shouldn't ship in flight firmware.

### F5 — Incoherent multi-element cross-core arrays · LOW · OPEN
- **Where:** `s_core1_channels[8]` / `s_core1_stick_filtered[4]` written by Core 1,
  read element-wise by Core 0 (`:772`, `:787-791`).
- **Problem:** Per-element 16-bit access won't tear a value, but the *set* isn't a
  coherent snapshot. Display/calibration only → low impact.
- **Fix:** Snapshot under the same mechanism as F1 if convenient.

### F9 — `channel_function` not validated as a permutation · LOW · OPEN
- **Where:** `rc_config_validate()` (`lib/RCConfig/RCConfig.cpp:48-49`) only clamps
  each entry to `< RC_NUM_AXES`.
- **Problem:** Two axes can map to the same function, leaving one of A/E/R/T
  unmapped and stuck at MID. Config foot-gun (silent).
- **Fix:** Reject/repair non-permutation mappings, or warn in the WebUI.

### F10 — Leftover AI/debug scaffolding · LOW · OPEN
- **Where:** ~16 `// #region agent log` / `debug_log_ndjson` blocks across `src/` and
  `lib/`.
- **Fix:** Gate behind a single debug macro or remove.

### F11 — Structure / dead code · LOW · OPEN
- `src/rc_crsf_main.cpp` is ~2200 lines with ~200 file-scope globals (god file).
- Frame/protocol logic duplicated across `*_ble.cpp` and `*_sx128x.cpp` variants.
- Commit `d960813` ("remove obsolete Bluetooth and BLE implementation files")
  actually only deleted BLE *docs* and the Flutter app (`BLE_IMPLEMENTATION.md`,
  `BLUETOOTH_IMPLEMENTATION.md`, `fpv_rx_config/…`) — **no `*_ble.cpp` was removed**,
  so the message is misleading. `rx_main_ble.cpp` / `tx_main_ble.cpp` still exist and
  are still built (`[env:rx-ble]` `platformio.ini:13`, `[env:tx-ble]` `:23`).
  Clarify whether BLE is supported or remove the envs/files.

### F12 — Config-size assert placement / WebUI coupling · LOW · OPEN
- **Where:** `static_assert(sizeof(rc_config_data_t) == 132, ...)` lives in
  `src/rc_crsf_main.cpp:22`, away from the struct (`lib/RCConfig/RCConfig.h:38`).
- **Problem:** WebUI `CONFIG_PAYLOAD_SIZE` is only tied by a comment; struct/protocol
  drift can't be caught at compile time on the firmware side beyond size.
- **Fix:** Move the assert next to the struct; consider a shared
  schema-version/size check.

### F13 — Absolute millis() comparisons (rollover) · LOW · OPEN
- **Where:** e.g. `tx_main_sx128x.cpp` pairing fallback compares
  `lastSyncAckReceived < connectingStartTime`; similar absolute-timestamp logic
  elsewhere.
- **Problem:** Breaks around the ~49.7-day `millis()` rollover. Not the current
  symptom, but latent.
- **Fix:** Use signed-difference comparisons: `(long)(now - start) > timeout`.

---

## IC datasheet validation (BQ2562x charger)

Validated the one hand-written driver in the RC (`src/bq2562x.*`) against TI's
BQ25620/BQ25622 family documentation. (OLED `SH110X` and `ADS1115` use mature
Adafruit libraries — lower config risk; the RP2350 internal-ADC resolution issue is
F7.) The SX1280 sits on the separate TX/RX modules, not the RC board.

> Method update (2026-05-21): the exact TI PDF was retrieved and read directly
> (`curl` with a browser User-Agent — the 403 was only a User-Agent block on
> `ti.com/lit`; the actual datasheet at `https://www.ti.com/lit/ds/symlink/bq25622.pdf`
> serves a static PDF, and TI ships one combined BQ25620/BQ25622 doc). Findings below
> are now validated against the datasheet text, not indexed snippets. F14 and F16 are
> closed against the datasheet; F15's address is confirmed wrong vs the datasheet but
> still wants a hardware I2C scan before the silicon address is flipped. See
> `datasheets/` for the archived PDF and `scripts/fetch-datasheets.sh` for re-pulling.

### F14 — Charger I2C watchdog never disabled or serviced · MED · CONFIRMED
- **Where:** `bq2562x_init()` (`src/bq2562x.cpp:94`) calls
  `bq2562x_reset_watchdog_timer()` exactly once. `bq2562x_set_watchdog_timer()` is
  defined but **never called**; the main loop's `update_tx_battery()`
  (`rc_crsf_main.cpp:2125`) only reads.
- **Datasheet behavior — CONFIRMED** (TI BQ25620/BQ25622 combined datasheet, §8.3.x
  Digital Clock and Watchdog + Table 8-17 REG0x16; `WATCHDOG` defaults to enabled,
  `1b`): a write to any register moves the part from default mode to host mode and
  starts the watchdog. On expiry the part returns to default mode — **`ICHG` is halved
  (rounded down)**, a set of fields reset to their POR defaults, and `WD_STAT`/
  `WD_FLAG` are set. Host must write `WD_RST=1` before timeout, or disable it with
  `WATCHDOG=00`.
- **Impact:** Charger periodically reverts to defaults. Benign today only because the
  firmware never programs custom charge current/voltage/input limits — but any such
  setting will silently revert ~tens of seconds later, and charge-status/telemetry
  can fluctuate. (The ICHG-halving on expiry also directly couples to F16's ICHG
  field.) Latent foot-gun.
- **Fix:** After `bq2562x_init()`, either disable the watchdog (`WATCHDOG=00` via
  `bq2562x_set_watchdog_timer`) for this read-mostly host, or pet it periodically
  (`bq2562x_reset_watchdog_timer()` in the battery-update cadence).

### F15 — Charger I2C address does not match datasheet · MED → HIGH · CONFIRMED (datasheet); HW scan still needed
- **Where:** `src/bq2562x.h:17` — `#define BQ2562X_I2C_ADDR 0x6A  // Match existing
  codebase (was 0x6B in original)`.
- **Datasheet — CONFIRMED WRONG:** the BQ25620/BQ25622 datasheet states the 7-bit
  address in three places (§8.5.1 I2C interface, §8.5.1.5 Target Address, and the
  device-address summary): the 7-bit address is `1101011b` = **`0x6B`**. The code's
  `0x6A` does not match the part. (`0x6A` is believed to be the BQ25180's address —
  the suspected mix-up — but this attribution is background knowledge, *not*
  confirmed from an archived datasheet (no BQ25180 PDF is in `datasheets/`).)
- **Problem:** If the silicon is at `0x6B`, `bq2562x_init()`'s `Wire.endTransmission()`
  probe NACKs → returns -1 → charger never initialized → battery telemetry/status
  silently disabled.
- **Verify (still required before flipping):** an I2C bus scan on hardware to confirm
  which address actually ACKs. Per datasheet the fix is `0x6A → 0x6B`, but do not flip
  blind — if a board is somehow working today, confirm the silicon's address first.

### F16 — ICHG (REG0x02) + VBAT ADC scaling · LOW · RESOLVED (scaling correct)
- **Where:** `src/bq2562x.h` — charge-current macros use 80 mA/LSB, no offset, mask
  bits 11:6 (`0b0000111111000000`); VBAT ADC `(raw & 0x1FFE) >> 1` ×
  `BQ2562X_VBAT_ADC_LSB_MV` (= `1.99f`).
- **Datasheet — CONFIRMED CORRECT** (BQ25620/BQ25622 combined datasheet):
  - ICHG (Table 8-9, REG0x02): field is bits **11:6**, **Bit Step 80 mA**, no offset,
    range 80–3520 mA (1h–2Ch), POR 1040 mA (Dh) → code's `(reg & mask)>>6 * 80`
    matches exactly. The "TI E2E erratum" worry does **not** apply to this revision.
  - VBAT ADC (Electrical Characteristics ADC table + Table 8-42, REG0x30): field is
    bits **12:1**, **LSB 1.99 mV** → `(raw & 0x1FFE) >> 1` extracts bits 12:1 and
    `× 1.99` matches. (The 3.97 mV figure is the **VBUS/VPMID** ADC LSB, not VBAT —
    no mismatch.)
- **Remaining nit (LOW):** ICHG is physically split across REG0x02[7:6] and
  REG0x03[3:0] (little-endian per the datasheet note). The driver reads a 16-bit word
  and masks 11:6 contiguously — fine **iff** the word read assembles bytes in that
  order; worth one glance at `bq2562x_read_word()` byte order, but not a transient
  suspect.

### (Open) SX1280 radio driver — not yet validated
- The SX1280 (on TX/RX modules, `lib/SX128xLink`) is the project's other
  custom-driver IC. Not part of the RC board, so out of scope for "RC transient
  issues," but worth a separate datasheet pass (RF regs, BUSY/DIO1 timing, SPI
  command sequencing) if RX-side glitches are also in play.

---

## IC datasheet validation (SX1280 radio, TX/RX modules)

`lib/SX128xLink` wraps **RadioLib** (mature) in **FLRC** mode, so low-level
command/register sequencing and BUSY/DIO1 handling are library-managed. Config runs
header defaults (no `platformio.ini` overrides): 2420 MHz, 1.3 Mb/s, CR 1/2, +10 dBm,
16-bit preamble, GFSK BT=0.5 shaping, fixed 33-byte packets.

> Method update (2026-05-21): the official Semtech SX1280/SX1281 datasheet is now
> archived locally (`datasheets/sx1280-sx1281-semtech.pdf`, retrieved via the
> Salesforce session flow in `scripts/fetch-datasheets.sh` — the fetch-tool 403 was a
> User-Agent block plus a JS/session gate, not a true block). The parameter findings
> below remain **validated-by-construction**: `beginFLRC()` returns
> `RADIOLIB_ERR_INVALID_*` for illegal freq/bitrate/coding-rate and the code checks
> the return (`SX128xLink.cpp:108`), so an out-of-spec value would fail init, not run
> silently. A deeper register-level pass against the now-archived PDF (RF regs,
> BUSY/DIO1 timing, AGC preamble minimum) is still open but not a transient suspect.
> **Conclusion: the radio *parameters* are within SX1280 spec — the transient issues
> are real-time/firmware behavior and the single-channel design, not illegal config.**

Parameter check (all within datasheet limits):
- Frequency 2420 MHz ∈ [2400, 2500] MHz ✓
- Output power +10 dBm ∈ [-18, +13] dBm ✓
- Bitrate 1.3 Mb/s ↔ 1.2 MHz bandwidth (datasheet-paired) ✓
- FLRC coding rate 1/2 (valid options 1/2, 3/4, 1) ✓
- Preamble 16 bits (FLRC supports 4–32 in steps of 4) ✓ — AGC preamble minimum
  (datasheet Table 14-34) not re-read from PDF; fine at 16, confirm before going lower.

### F17 — Per-packet Serial logging in the radio hot path · HIGH · OPEN
- **Where:** `SX128xLink::send()` logs unconditionally on every transmit — the
  blocking `transmit()` result is logged at `SX128xLink.cpp:189-200`, and a separate
  pre-transmit "Calling transmit()" log fires at `:181-188` (87 `Serial.print*` calls
  in the file total, none rate-limited); TX sends channel data at
  `DATA_INTERVAL_MS = 6` ms → **~167 Hz** (`PacketHandler.h:12`).
- **Problem:** ~2–3 `Serial.print` calls × 167 Hz = ~350–500 formatted prints/sec on
  the TX. That steals CPU from the 6 ms budget and can block on a full USB-CDC TX
  buffer, adding large/variable latency. The sync-ACK listen window is only **10 ms**
  (`tx_main_sx128x.cpp:39`), so jitter here makes the TX miss SYNC_ACKs →
  `syncAckMisses` climbs → connection state flaps → **transient disconnects**.
  (`PacketHandler` logging is rate-limited; `SX128xLink::send()` is not.)
- **Fix:** Remove the per-packet prints or gate them behind a debug macro
  (`#if SX128X_DEBUG`). Strongest single SX-side transient fix.

### F18 — Single fixed RF channel, no frequency hopping · MED · OPEN
- **Where:** `SX128X_FREQ_MHZ 2420.0f` (`SX128xLink.h:37`); no channel-hopping logic
  anywhere.
- **Problem:** A static 2.4 GHz channel sits in the middle of WiFi/BT/microwave
  traffic. Without FHSS, a single interferer on/near 2420 MHz causes intermittent
  dropouts that look exactly like "transient RC issues," even at short range.
  (ELRS-class links hop across the band for this reason.)
- **Fix:** Implement FHSS (hop sequence seeded by the binding UID, synced via the
  existing sync packets), or at minimum make the channel configurable and pick a
  quiet one. FHSS is the larger, higher-value change.

### F19 — Blocking `transmit()` in the TX loop · MED · OPEN
- **Where:** `SX128xLink::send()` uses RadioLib's blocking `radio->transmit()`
  (`SX128xLink.cpp:189`), then `startReceive()`.
- **Problem:** The loop stalls for the full TX duration (mode switch + airtime +
  BUSY waits) every packet, adding latency/jitter and capping the achievable rate.
- **Fix:** Use async `startTransmit()` + DIO1 TX-done interrupt; return to RX in the
  ISR/handler.

### F20 — SPI clock far below device capability · LOW · OPEN
- **Where:** `SPISettings spiSettings(2000000, ...)` = 2 MHz (`SX128xLink.cpp:54`);
  SX1280 supports up to ~18 MHz.
- **Problem:** Every command/FIFO transfer is ~9× slower than necessary, compounding
  F19's latency.
- **Fix:** Raise to ~8–16 MHz after confirming wiring/length; keep a margin.

### F21 — `getSNR()` used in FLRC mode · LOW · OPEN
- **Where:** `SX128xLink::receive()` reads/reports `radio->getSNR()`
  (`SX128xLink.cpp:229`).
- **Problem:** SNR is a LoRa-only metric; in FLRC/GFSK it's undefined → misleading
  telemetry/logs. Harmless to the link. **Fix:** drop SNR for FLRC, rely on RSSI.

---

## Notes
- `test/` is empty — none of the above (especially the concurrency items) is covered
  by CI. A host-side unit test of the config-snapshot / validate logic and a
  documented hardware soak test for save/calibrate would catch regressions.
- Suggested first batch to land: **F1 + F3** (one commit, Core-1-owns-the-reload),
  **F2** (proxy guard), and **F6** (flash-safe save) — these are the most likely
  sources of the observed transients.
