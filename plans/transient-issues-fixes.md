# RC firmware — transient issues & required fixes

Tracking doc for the intermittent ("transient") issues seen on the RC board
(`rc-rp2350` / `rc-crsf` builds). Root cause theme: the dual-core design
(Core 0 = USB config/display/telemetry, Core 1 = sticks→channels→UART at 250 Hz)
shares mutable state across cores, and recent feature commits widened that
shared surface without matching synchronization.

Status legend: `OPEN` / `IN PROGRESS` / `FIXED` / `WONTFIX` / `NEEDS-VERIFY`
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

### F2 — Proxy passthrough fights Core 1 for the UART · HIGH · OPEN
- **Where:** `rc_proto_enter_usb_uart_proxy` sets `proxy_mode_enabled` (`:1896`);
  Core 0 does raw `Serial2`↔USB passthrough (`:651-656`), but `loop1()` never checks
  the flag and keeps writing CRSF frames to `Serial2` at 250 Hz.
- **Problem:** Two cores write the same UART → interleaved bytes → corrupted
  ELRS/EdgeTX passthrough (exactly the case where Core 1 is sending channels).
- **Fix:** In `loop1()`, skip the UART write (early return) while
  `proxy_mode_enabled` is true.

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
- Commit `d960813` ("remove obsolete Bluetooth/BLE files") yet `rx_main_ble.cpp` /
  `tx_main_ble.cpp` still exist and are still built (`[env:rx-ble]`, `[env:tx-ble]`).
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

## Notes
- `test/` is empty — none of the above (especially the concurrency items) is covered
  by CI. A host-side unit test of the config-snapshot / validate logic and a
  documented hardware soak test for save/calibrate would catch regressions.
- Suggested first batch to land: **F1 + F3** (one commit, Core-1-owns-the-reload),
  **F2** (proxy guard), and **F6** (flash-safe save) — these are the most likely
  sources of the observed transients.
