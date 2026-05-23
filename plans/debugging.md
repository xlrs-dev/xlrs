# XLRS Host Interfacing & Step-by-Step Troubleshooting Guide

This guide outlines the system integration, chronological validation flow, and bench troubleshooting protocols for the XLRS **TX/RX radio modules**. The TX module consumes channel/control frames over UART; the RX module emits CRSF to the flight controller.

Debugging embedded real-time transceivers is notoriously difficult because radio frequency (RF) packets are microsecond-critical and cross multiple hardware, serial, and multicore boundaries. This document provides a highly structured, step-by-step procedure to isolate and systematically verify the entire signal path—from static build-time parameters up to full end-to-end bidirectional loop telemetry.

---

## 1. System-Wide Timing Hazards & Known Architectural Constraints

Before performing physical hookups, you must understand several critical timing and scheduler constraints discovered during engineering audits of the initial scaffold. Failing to guard against these hazards will cause clock desynchronization, serial buffer overruns, and link instability:

> [!WARNING]
> ### 1. Timer ISR Overhead ("Tiny ISR" Rule)
> **The Hazard:** If the `HwTimer` interrupt service routine (ISR) directly executes the full scheduler path (`RfScheduler::onTick`), it performs slot selection, FHSS channel updates, packet encoding/encryption, and SPI calls to the SX1280 transceiver. At a standard 10 MHz SPI bus speed, these operations require **150–350 µs** to complete. Blocking the CPU in an ISR context for this duration will starve high-speed UART interrupts (like the CRSF parser running at 420,000 bps, which requires processing a new character every **23.8 µs**), leading to serial buffer overruns and packet loss.
>
> **The Solution:** The timer ISR must *only* increment a monotonic volatile/atomic counter and latch a microsecond timestamp, then immediately exit. The Core 1 background task (`RfTask::poll()`) must observe this counter, wake up, and execute the heavy SPI and scheduling operations in task context.

> [!IMPORTANT]
> ### 2. DIO Interrupt & SPI-in-ISR Safety
> **The Hazard:** When the SX1280 transceiver raises the `DIO1` line to signal TX-done or RX-done, executing `_phy->readRx(pkt)` or processing packet payloads inside the DIO ISR context violates the "Tiny ISR" rule and will freeze the system.
>
> **The Solution:** The DIO interrupt handler must only capture the high-precision hardware timestamp (`pkt.timestampUs`) and post an event flag to a lock-free queue. The background Core 1 task then drains the SPI FIFO and executes the decoding logic.

> [!CAUTION]
> ### 3. Transmitter (TX) Master Clock Pulling
> **The Hazard:** In a synchronized RF link, the **TX module is the absolute time master**. Only the **RX module** runs a Phase-Frequency Detector (PFD) PI-loop to lock its local clock phase to the transmitter. If the TX module erroneously applies PFD corrections upon receiving `OtaType::TlmDown` telemetry packets from the RX, the master clock will hunt and drift dynamically, destabilizing the entire link schedule.
>
> **The Solution:** Gate all PFD timing corrections strictly to `Role::Rx`. In `RfScheduler::onRxDone()`, the PFD PI loop update must be skipped if the active role is `Role::Tx`.

---

## 2. Chronological Flow of Operations & Step-by-Step Verification

To achieve a verified, fully operational system, follow this chronological 4-phase bring-up sequence. **Do not skip ahead**; each phase forms the diagnostic foundation for the next.

```
+-------------------------------------------------------+
|  PHASE 1: Configuration & Static Parameter Validation |
|  - Verify identical UIDs, Sync Words, & FHSS Tables   |
+---------------------------+---------------------------+
                            |
                            v
+-------------------------------------------------------+
|  PHASE 2: Transmitter (TX) Host Interfacing & Tuning   |
|  - Verify 420K UART, Mailbox, and Slot Jitter         |
+---------------------------+---------------------------+
                            |
                            v
+-------------------------------------------------------+
|  PHASE 3: Receiver (RX) Dwell, Lock & Failsafe        |
|  - Verify Acquisition, PFD Sign, & NoPulses Failsafe  |
+---------------------------+---------------------------+
                            |
                            v
+-------------------------------------------------------+
|  PHASE 4: End-to-End Bidirectional System Validation  |
|  - Verify Telemetry Downlink & Power Hysteresis       |
+-------------------------------------------------------+
```

---

### Phase 1: Configuration & Static Parameter Verification

Before flashing firmware to the hardware modules, verify that the compiled configuration structures on both nodes mathematically align. If the static parameters mismatch, the nodes will fail to establish an RF link.

Recommended preflight:

```bash
scripts/check-env.sh
scripts/build.sh
scripts/test.sh
```

Flash and monitor:

```bash
scripts/flash.sh tx
scripts/flash.sh rx
TX_PORT=/dev/cu.usbmodem101 RX_PORT=/dev/cu.usbmodem102 scripts/monitor.sh both
```

#### 1. FNV-1a Link UID Verification
* **Why it matters:** The 64-bit Link UID seeds the FHSS pseudorandom hop sequence and the transceiver sync words. Both the TX and RX must compute the identical 64-bit hash from the binding phrase.
* **Verification Action:** Use `scripts/monitor.sh` at boot. Verify that both modules use the same binding phrase and computed identity. If richer UID logging is needed, add it around `BindingStore::begin()` during bench bring-up.
  ```text
  [UID] Binding Phrase: "Kikobot-02"
  [UID] Computed 64-bit Hash: 0xE36C2F8A4B9D1E0F
  ```

#### 2. Derived Hardware Sync Word Validation
* **Why it matters:** The SX1280 uses a hardware sync word to filter out noise. In LoRa mode, this is a 16-bit value. In FLRC mode, this is padded to 4 bytes using a bitwise-NOT convention to ensure excellent bit transition density (which prevents false correlation triggers):
  $$\text{FLRC Sync} = \big[\, \text{uid}_{\text{msb}} \,\big]\,\|\,\big[\, \text{uid}_{\text{lsb}} \,\big]\,\|\,\big[\, \sim\!\text{uid}_{\text{msb}} \,\big]\,\|\,\big[\, \sim\!\text{uid}_{\text{lsb}} \,\big]$$
* **Verification Action:** Check that the boot logs output matching register values:
  ```text
  [PHY] Mode: FLRC, Sync Word configured: 0x2F8A4B9D -> registers programmed: 2F 8A D0 75
  ```

#### 3. Channel Count Constraint
* **Current behavior:** the link core carries 8 primary channels (`Link::RC_CHANNELS = 8`). The RX emits a 16-channel CRSF packed frame: channels 0-7 come from the RF link and channels 8-15 are centered placeholders. The host-side TX UART should provide 8 channel values scaled to the expected 11-bit/control range.

---

### Phase 2: TX UART Interfacing & Bench Debugging

In this phase, wire a controller-side UART peer to the XLRS TX module and verify the high-speed channel/control interface.

```
   Controller UART Peer                  XLRS TX Module
+--------------------+                 +--------------------+
|                GP8 |---------------->| GP9 (RX)           |
|                GP9 |<----------------| GP8 (TX)           |
|                GND |-----------------| GND                |
|                    |                 |                    |
|      [Phase 2 GPIO]|---> Logic       |      [Slot GPIO]   |---> Logic
|      Diagnostic Pin|     Analyzer    |      Diagnostic Pin|     Analyzer
+--------------------+                 +--------------------+
```

#### 1. Physical Interface Verification
* **Wiring Requirements:** Connect GP8 (TX) of the Host to GP9 (RX) of the TX module, and GP9 (RX) of the Host to GP8 (TX) of the TX module. Ensure a clean, short ground path. Use 3.3V logic levels only.
* **Baud Rate Auditing:** The UART interface runs at **420,000 bps** (8N1). Connect a logic analyzer to the UART lines and measure the duration of a single serial bit. It must measure exactly **$2.38\,\mu\text{s}$**.

#### 2. Ping-Pong Heartbeat Check
* **Verification Action:** Configure the Host to emit `UART_MSG_PING` frames. The TX module must respond with `UART_MSG_PONG` frames.
* **Expected Outcome:** The Host debug log should print:
  ```text
  [UART] Sent PING -> Received PONG (RTT: 420 us)
  ```
  If PONG is not received, check the RX frame state machine state using the logic analyzer. Ensure `0xA5` is sent as the sync byte.

#### 3. Multi-Core Mailbox Handoff Verification
* **Why it matters:** The UART parser runs on Core 0, while the `RfScheduler` runs on Core 1. Channels are handed off via a lock-free `LatestValue` mailbox. If there is a compilation reordering or memory barrier issue, the RF core may read torn or stale data.
* **Verification Action:** Monitor TX/RX USB logs and, if needed, add temporary counters around `LatestValue<AppToRfData>` in `apps/tx/main.cpp`. Verify that channel updates are loaded into the RF task without latency slips:
  ```text
  [STATS] Tick: 12000, AppToRf Mailbox updates: 100Hz, Slot Jitter: < 2 us, Queue Drops: 0
  ```

#### 4. Logic Analyzer Profiling (Jitter & Latency)
To verify real-time scheduling constraints, program the Host and TX modules to toggle dedicated GPIO pins:

```
                  Timer Interrupt Fired
                            |
GPIO 14 (Timer ISR):   +----+                    (ISR Execution time < 5us)
                       |    |
                       +----+--------------------
                            |<- Task Wake Latency ->|
GPIO 15 (RF Task):     -----+                       +-----------------------+
                                                    |                       | (Slot Processing
                                                    |                       |  duration < 150us)
                                                    +-----------------------+
```

1. **Measure ISR Execution Time:** Toggle a GPIO pin high at the entry of the `HwTimer` ISR callback, and low at the exit. The pulse width on the logic analyzer must be **$< 5\,\mu\text{s}$**.
2. **Measure Slot Jitter:** Toggle a GPIO pin high at the start of `RfScheduler::onTick()` and low at the end. The interval between pulses must match the rate interval (e.g. $10,000\,\mu\text{s}$ for 100 Hz) with a jitter of **$< \pm 2\,\mu\text{s}$**.

---

### Phase 3: Receiver (RX) Acquisition, Timing Lock & Output Verification

Power up the Receiver module in isolation and verify its synchronization and output failsafe features.

#### 1. Cold-Start Dwell & Acquisition Verification
* **Why it matters:** At power-on, the RX does not know the TX's hopping phase. It must dwell on **Channel 0** (the UID-derived acquisition channel) until it receives its first `OtaType::Sync` packet.
* **Verification Action:** Hook a logic analyzer to the RX SPI bus (SCK, CS) and monitor frequency hopping.
* **Expected Outcome:** At power-on, the RX must remain on Channel 0 frequency. Immediately upon receiving the first valid Sync packet, it must extract the FHSS index and begin hopping in unison with the TX. The maximum acquisition time must be **$\le$ the Sync packet beacon period (typically 40 ms)**.

#### 2. PFD Phase Lock & Sign Convention Tuning
* **Why it matters:** To maintain a solid, low-jitter RF link, the RX hardware timer must lock to the TX clock. An inverted PFD sign will cause positive feedback, pushing the clocks apart and preventing lock.
* **Verification Action:** The mathematical sign for phase error calculation is:
  $$\text{offsetUs} = \text{Actual Packet Start Time} - \text{Expected Slot Tick Time}$$
  If the packet is late ($\text{Actual} > \text{Expected}$), $\text{offsetUs}$ is positive, which must yield a **negative correction** from `Pfd::update()` to shorten the next interval and fire the timer earlier.
* **Scope Trace Validation:** Connect a dual-trace oscilloscope:
  * Channel 1: RX hardware slot tick (GPIO pulse).
  * Channel 2: SX1280 DIO1 RX-done pulse.
  * *Success Criteria:* On startup, the RX-done pulse will drift relative to the slot tick. Within **$100\,\text{ms}$**, the PFD loop must converge, locking the RX-done pulse to a rock-solid, constant position relative to the slot tick with a jitter of **$< \pm 12\,\mu\text{s}$**.

#### 3. Failsafe "NoPulses" Verification
* **Why it matters:** On connection loss, the RX must immediately stop emitting CRSF frames. Flight controllers (Betaflight/iNav) rely on this *absence* of frames to trigger autonomous Return-to-Home (RTH) protocols.
* **Verification Action:**
  1. Power on both TX and RX. Verify that the RX outputs a steady 50 Hz CRSF stream to the flight controller (or logic analyzer).
  2. Disconnect the TX antenna or power off the TX.
  3. Measure the time until the CRSF serial stream stops.
* **Success Criteria:**
  * **Uplink Slot Counting:** The failsafe accountant must count missed *uplink slots only* (excluding sync and telemetry slots).
  * **Cutoff Time:** The CRSF output must cease after the configured consecutive missed-uplink threshold (`Link::FAILSAFE_MISS`). The output must remain flat (`NoPulses`); it must never emit "hold last state" channels unless explicitly configured.

---

### Phase 4: End-to-End Bidirectional Link & Telemetry Validation

With the TX and RX synchronized, verify the complete bidirectional telemetry loop and dynamic RF power settings.

#### 1. Telemetry Downlink Verification
* **Operational Flow:** On designated telemetry slots, the RX module switches to TX mode and transmits a `OtaType::TlmDown` packet containing RF link statistics such as RSSI, SNR, LQ, current rate, and queue/recovery counters.
* **Verification Action:** Monitor the TX USB log and the controller-side UART telemetry consumer. The TX app forwards telemetry/status over its UART protocol at its configured app interval:
  ```text
  [TELEM] RSSI: -62 dBm, SNR: 8.5 dB, LQ: 98%, Rate: 100Hz
  ```

#### 2. Dynamic Power Hysteresis Verification
* **Why it matters:** To save battery and reduce thermal load, the TX dynamically adjusts its power output. To prevent the power level from oscillating rapidly, the controller uses a dual-threshold hysteresis algorithm based on Link Quality (LQ) and RSSI.
* **Verification Action:** Monitor the TX serial console while introducing signal attenuation (e.g. walking away or placing the RX in a shielded container).
* **Expected State Machine Flow:**
  * **Power Step-Up:** If the reported RX Link Quality drops **$< 80\%$**, the TX must immediately step up its output power by $+3\,\text{dBm}$ per cycle up to its maximum configuration.
  * **Power Step-Down:** The TX must hold its power level stable until the LQ recovers to **$\ge 99\%$** *and* the RSSI rises above **$-65\,\text{dBm}$**, after which it may step down.

---

## 3. Bench Troubleshooting Checklist & Failure Mode Matrix

If you encounter issues during bench testing, use this matrix to quickly isolate and diagnose the root cause:

| Observed Symptom | Likely Root Cause | Diagnostic Steps & Actions |
|------------------|-------------------|----------------------------|
| **Host prints `PING Timeout`** | 1. Incorrect UART wiring.<br>2. Baud rate mismatch.<br>3. Sync byte mismatch. | * Check that TX/RX lines are crossed between Host and TX module.<br>* Use a logic analyzer to verify baud is exactly 420K bps.<br>* Verify Host transmits `0xA5` as the first byte of every frame. |
| **No-Connect (Acquisition Failure)** | 1. Link UID phrase mismatch.<br>2. Frequency-hopping table discrepancy.<br>3. SX1280 sync-word remap issue. | * Verify both modules were built with the same `XLRS_DEFAULT_BINDING_PHRASE` or have the same persisted binding phrase.<br>* Verify that both TX and RX compiled the same regional channel table.<br>* Check the FLRC sync word path, including the §16.4 forbidden sync-word guard. |
| **Link connects briefly, then drops out repeatedly** | 1. PFD feedback sign inversion.<br>2. Excessive crystal reference frequency mismatch. | * Check that `RfScheduler.cpp` computes `offsetUs` with negative feedback sign.<br>* Use a scope to verify the RX slot tick pulls *toward* the RX-done pulse, not away from it.<br>* Check PFD integral wind-up. If it hits limits (+/- interval * 4) rapidly, replace the crystal/TCXO. |
| **High UART packet loss or system freezes** | 1. Heavy SPI or encryption calls running in the HwTimer ISR. | * Audit `HwTimer.cpp` and `RfScheduler.cpp`. Ensure `onTick` is deferred to Core 1 task context.<br>* Toggle GPIO pins in the ISR and verify execution duration is **$< 5\,\mu\text{s}$**. |
| **Failsafe triggers prematurely during telemetry** | 1. Failsafe accountant is tracking raw ticks instead of uplink slots. | * Verify that missed slots are only incremented on ticks allocated to Uplink. |
| **Telemetry is received but sticks feel sluggish or laggy** | 1. Cross-core synchronization is using blocking mutexes instead of lock-free mailboxes. | * Verify that `AppToRf` and `RfToApp` use single-producer single-consumer lock-free `LatestValue` mailboxes.<br>* Verify that no blocking read/write locks are used on the RF core. |
