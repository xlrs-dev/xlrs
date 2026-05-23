# XLRS Firmware Production Roadmap

This document tracks the firmware-owned work needed to make XLRS a production-grade, flight-ready remote-control link for commercial and industrial drones.

The goal is not to copy ExpressLRS feature-for-feature. ELRS is a useful reference because it proves which problem classes matter in real RC links: timing, OTA efficiency, FHSS, failsafe behavior, telemetry, power control, configuration, and diagnostics. XLRS solves those problems with its own architectural strengths: deterministic dual-core scheduling, narrow interfaces, native testability, lockless multi-core execution, and strict product-grade configuration discipline.

---

## Completed Milestones (Production Hardening Phase)

We have successfully completed a comprehensive production-hardening sweep, addressing critical safety, configuration, and user-experience gaps:

1.  **[COMPLETE] Binding UID Consistency**: Unified the binding UID derivation via a lightweight **64-bit FNV-1a hash** to generate an 8-byte `Link UID`. Derived a 16-bit sync word that provides physical, hardware-level correlator filtering to isolate different models. Integrated it into the XLRS `BindingStore` with handset UART command updates (`UART_MSG_CMD_SET_BIND_TX`) and auto-reboots. Sync beacons now carry a UID CRC and the RX rejects mismatched sync frames before declaring lock.
2.  **[COMPLETE] Config Safety Partitioning**: Formally partitioned the EEPROM into safe, non-overlapping, checksum-validated blocks: `RfConfigData` at base `0` (range 0–119), `BindingStore` UID state at base `120` (range 120–199), and stick calibrations `RCConfig` at base `200` (range 200–340). Handled magic validation, version schema, RF CRC16 validation, binding-store checksum validation, and automatic defaults rollback.
3.  **[COMPLETE] ELRS-Grade Dynamic Power Safety Overrides**: hard-coded immediate safety boosts into `DynamicPower`. If the telemetry Link Quality (LQ) sags below `50%` or if `_consecutiveMissedTelemetry >= 2` (indicating lost downlink), the transmitter instantly jumps to maximum configured power (`maxPowerDbm`) to prevent a critical failsafe.
4.  **[COMPLETE] Failsafe Behavior**: Configured both `NoPulses` (default) and `Hold` failsafe modes in `RfConfig`. Upon `FAILSAFE_MISS = 10` missed slots (a rapid `10ms` latency threshold at `F1000`), the receiver completely silences CRSF serial UART output to trigger immediate Flight Controller failsafes.
5.  **[COMPLETE] Advanced User-Visible State UX**: 
    *   **Receiver FastLED Indicator**: Programmed distinct WS2812 blinking patterns: solid Red (Config Fault), fast double-blinking Red (Radio Hardware Fault), fast blinking Blue (Binding Mode), breathing Orange (Searching), slow blinking Red (Failsafe/RF Loss), and solid Green (Connected).
    *   **Handset State Mapping**: Mapped `xlrs::LinkState` to handset UART connection codes (`0` to `4`), allowing the pilot's handset LCD screen to display the exact status name (`PAIRING`, `CONNECTING`, `CONNECTED`, `LOST`) in real-time.

---

## Scope

This roadmap includes only items firmware can solve directly or materially control. Hardware RF layout, antenna design, PA/LNA choice, oscillator quality, certification lab work, and manufacturing process are outside firmware scope, but firmware must expose the controls and evidence needed to validate those areas.

## Priority Definitions

- **P0:** Required before serious flight testing or production positioning.
- **P1:** Required before production release candidate.
- **P2:** Required for robust commercial operations, fleet support, or regulatory readiness.
- **P3:** Cleanup/process work that prevents long-term drift and support burden.

---

## Roadmap

| Priority | Area | What XLRS Does Today | What To Do | Why This Is Superior |
|---|---|---|---|---|
| **P0** | **Binding UID consistency** | **[COMPLETE]** Derives FNV-1a UIDs consistently on TX/RX from EEPROM, calculates 16-bit hardware sync words, accepts UART commands, persists through `BindingStore`, carries UID CRC in sync beacons, and reboots after phrase updates. | None for core identity. Maintain this unified pipeline as new modulation rates and bind flows are introduced. | Removes split identity behavior. TX/RX identity is deterministic, cheap, host-testable, and shared by FHSS + sync-word derivation. |
| **P0** | **Hardware timing validation** | Has `HwTimer`, `RfScheduler`, `Pfd`, async PHY hooks, and native timing tests. Turnaround compensations are modeled (55us Native, 85us RadioLib). | Measure real scheduler jitter, RX PFD lock time, drift tolerance, missed deadlines, hop-boundary timing, and telemetry turnaround on physical TX/RX hardware. | Turns architecture into evidence. Timing is the foundation for FHSS, collision-free telemetry, latency, and failsafe timing. |
| **P0** | **SX1280 native driver hardening** | Has `RadioLibPhy` plus an early `Sx1280NativePhy` with async TX/RX, command timeout checks, physical `resetRadio()` recovery, and `_hardwareError` tracking. | Validate all SX1280 register values and command sequences: high-sensitivity mode, AutoFS/FS behavior, IRQ masks, packet params, CRC errors, RSSI/SNR decode, timeouts, power commit, and recovery. | Owning the driver gives deterministic SPI timing and lower overhead than RadioLib while preserving the `IRadioPhy` swap boundary. |
| **P0** | **OTA packet shrink** | **[CORE COMPLETE / PRODUCT PARTIAL]** Adds an 8-byte compact RC path for lossless primary-control cases and falls back to the full 8-channel 11-bit frame when compact packing would lose data. Sync remains 10 bytes. | Finish product policy: switch multiplexing, acceptable quantization rules if desired, CRSF AUX mapping, and flight testing across all channel patterns. | Reduces airtime when safe while preserving full-resolution fallback. This avoids chasing an 8-byte target at the cost of silent control precision loss. |
| **P0** | **Real FHSS region tables** | **[PARTIAL]** Has UID-seeded FHSS, 80-channel 2.4 GHz coverage, and TX/RX region plumbing from `RfConfig` into `Link`. | Add production regional/domain tables, channel spacing rules, dwell accounting, region enforcement, and CE LBT support. | FHSS becomes both interference-resistant and compliance-ready instead of only demonstrating that hopping works. |
| **P0** | **Failsafe verification** | **[COMPLETE]** Configured `NoPulses` vs `Hold` failsafe modes in `RfConfig` and successfully mapped CRSF UART silencing. | Prove failsafe timing at every rate and with Betaflight, iNav, and ArduPilot. Add explicit CRSF link/failsafe signaling where supported. | Safety behavior becomes measurable and predictable across aircraft stacks, not just correct in internal state. |
| **P1** | **Reliable telemetry transport** | **[CORE COMPLETE / PRODUCT PARTIAL]** Sends downlink link statistics and includes `StubbornSender` / `StubbornReceiver` chunking with sequence/ack behavior integrated into MSP OTA frames. | Wire the transport to real MSP/config/sensor/MAVLink producers and consumers, enforce bandwidth budgets per rate, and test under loss/reorder conditions on hardware. | Enables configuration, diagnostics, and telemetry without corrupting RC timing or relying on best-effort single packets. |
| **P1** | **Rate table calibration** | Has representative FLRC/LoRa rates and estimated airtime values in `RateConfig`. | Replace representative values with measured sensitivity, airtime, guard time, preamble, modulation, and TX/RX turnaround values for each rate. | Lets XLRS select rates based on measured latency/range tradeoffs instead of assumptions. |
| **P1** | **Dynamic power productionization** | **[COMPLETE]** Implemented the ELRS-grade dynamic power emergency boost (severe LQ sag < 50%) and telemetry-loss boost. | Add target-calibrated power tables, rate-aware SNR/RSSI thresholds, PA/LNA constraints, and optional PDET feedback. | Software policy becomes tied to actual RF output and hardware limits, avoiding false confidence from nominal dBm values. |
| **P1** | **Config safety** | **[COMPLETE]** Formally partitioned EEPROM (RfConfig at base 0, BindingStore at 120, RCConfig at 200), added magic/version validation, RF CRC16 validation, binding-store checksum validation, and auto-rollback defaults. | Define explicit schema migrations and write lockout rules during RF-critical operation. | Prevents field-bricking and cross-config corruption as schemas evolve. |
| **P1** | **Model match and bind lifecycle** | **[COMPLETE]** Seeding sync word and FHSS from 8-byte binding phrase UID. | Add bind mode, bind timeout, model match, persisted authorization, safe bind rules, and migration behavior. | Separates "same phrase can communicate" from "this aircraft is authorized for this model." |
| **P1** | **Link-state machine hardening** | **[CORE COMPLETE]** Uses `Connecting`, `Connected`, `Failsafe`, explicit lock/sync gates, UID CRC validation on sync, reacquisition behavior, and RX output gating until sync/FHSS lock plus a valid post-sync RC frame are present. | Add richer product states if the UI needs them: tentative lock, authorized model, RF degraded, and recovery countdown. | Prevents premature RC output during partial sync, wrong model, or unstable timing conditions. |
| **P1** | **Link diagnostics** | **[PARTIAL]** Visual FastLED blinking UX, handset UART connection state mapping, and internal stats for telemetry misses, FHSS index, current TX power, queue drops, and PHY recovery history. | Expose CRC errors, PFD offset, lock state, hardware health, and recovery history through CRSF/UI/debug streams. | Makes flight testing and field support practical. Silent failure modes become observable. |
| **P2** | **CRSF ecosystem support** | RX emits RC channels and link statistics; TX sends basic telemetry/status to the handset side. | Add CRSF device info, parameter routing, richer telemetry, config commands, and optional MSP/MAVLink bridging where product requirements need them. | Integrates cleanly with FCs, handsets, and tooling without pushing app complexity into the RF core. |
| **P2** | **Diversity / dual-radio path** | Current core is single-radio. The PHY abstraction could support more later, but the API is not yet designed around diversity. | Decide whether industrial reliability requires antenna diversity or dual-radio operation (e.g. Gemini). If yes, extend PHY/link abstractions before APIs harden. | Avoids locking production firmware into a single-radio design if the reliability target requires diversity. |
| **P2** | **Regulatory firmware features** | Has a simple region field and a basic EU power cap. | Add CE LBT where required, dwell-time accounting, region lock behavior, and compliance test modes. | Makes compliance an explicit firmware behavior rather than a late paperwork exercise. |
| **P2** | **CI and native tests** | **[COMPLETE]** Has a fast native test suite for core logic and firmware builds for multiple environments. | Add tests for telemetry-loss power boost, UID migration, bind mismatch, corrupt packets, EEPROM validation, recovery failure, rate switching, and long simulated soaks. | Keeps the clean architecture honest as complexity grows. Fast tests catch regressions before bench time. |
| **P2** | **Hardware-in-loop tests** | Has native `MockPhy` simulation and manual firmware build validation. | Build automated TX/RX bench tests with attenuation, interference, brownout, power-cycle, packet loss, and recovery cases. | Verifies real RF behavior that mocks cannot: SPI timing, IRQ timing, radio state transitions, power, and recovery. |
| **P3** | **Remove legacy contradictions** | **[COMPLETE]** Production firmware now has one XLRS link core. Legacy `Security`, `Protocol`, `SX128xLink`, legacy SX128x mains, and stale PlatformIO envs were removed. | Keep future experimental paths behind explicit new names and avoid duplicated identity, crypto, or OTA packet logic. | Production firmware has one source of truth, eliminating misleading claims and split behavior. |
| **P3** | **Production observability** | Has serial logs, status packets, and link stats. | Add build fingerprint, config dump, field diagnostic command, compact debug stream, and flight-test trace mode. | Makes support and fleet debugging possible without reflashing custom debug builds. |
| **P3** | **Safety process** | The architecture is audit-friendly, but formal process artifacts are not yet present. | Add requirements traceability, hazard analysis, release checklist, deterministic builds, signed firmware, and verification artifacts. | Converts good engineering into repeatable product engineering suitable for commercial review. |

---

## Immediate Firmware Focus

With Task 1, Task 9, Task 10, and Task 11 successfully **completed**, the immediate firmware milestones shift to validating our hardware and expanding regulatory / data options:

1.  **[P0] Validate timing, PFD lock, FHSS, and telemetry slots on hardware**: Measure actual RP2040 hardware timer jitter and PFD lock times using logic analyzer probes.
2.  **[P0] Replace the placeholder FHSS table with production region/domain handling**: Build regional 80-channel balanced sequences for US/EU regulatory compliance.
3.  **[P0] Harden and bench-test the native SX1280 driver**: Run extensive register audits to ensure high-sensitivity registers and timeouts are fully calibrated.
4.  **[P0] Finish production FHSS region enforcement**: Convert the current region plumbing into region-specific tables, dwell accounting, and CE LBT behavior.
5.  **[P1] Wire reliable telemetry to real app flows**: Connect the existing `StubbornSender` / `StubbornReceiver` core to MSP/config/sensor/MAVLink producers and consumers.

---

## Positioning

XLRS is a clean, deterministic, highly auditable link core designed for professional commercial and industrial production. With our recent completion of persistent config safety, dynamic power safety overrides, custom failsafe modes, and dynamic binding integrations, the link core has established a robust safety baseline. We should continue to maintain strict target and logic separation as we tackle timing lock validations and FHSS expansion.
