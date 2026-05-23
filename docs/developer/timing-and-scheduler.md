# Timing And Scheduler

`RfScheduler` owns the fast per-tick sequence:

- Slot selection.
- FHSS advancement.
- OTA encode/decode handoff.
- TX/RX turnaround.
- PHY TX/RX calls.
- Timing health counters.

`Link` owns slower lifecycle policy: bind, connect, connected, failsafe, and
link stats. Do not move per-tick scheduling decisions into `Link`, and do not
move lifecycle policy into `RfScheduler`.

Rules:

- TX is the time master.
- RX runs the PFD/PI loop.
- Timer ISR latches timestamp / increments an event counter only.
- DIO ISR latches event/timestamp only.
- RF task handles slot work, SPI, OTA codec, and PHY operations.
- Packet-start timing is recovered as RX timestamp minus airtime.

```mermaid
graph TD
    classDef hardware fill:#fee2e2,stroke:#ef4444,stroke-width:2px;
    classDef calculate fill:#e0f2fe,stroke:#0284c7,stroke-width:2px;
    classDef control fill:#f0fdf4,stroke:#16a34a,stroke-width:2px;
    classDef output fill:#ffedd5,stroke:#f97316,stroke-width:2px;

    PacketRx["Packet Received &<br/>DIO1 Interrupt Fires"]:::hardware
    LatchTime["Latch RX Timestamp<br/>(RxPacket.timestampUs)"]:::hardware
    CalcAirtime["Lookup Frame Airtime<br/>(RateConfig.airtimeUs)"]:::calculate
    RecoverStart["Recover Packet Start:<br/>packetStartUs = timestampUs - airtime"]:::calculate
    CalcError["Calculate Phase Error:<br/>phaseErrorUs = packetStartUs - expectedStartUs"]:::calculate

    subgraph PI_Controller["PI Loop (PFD Core)"]
        P_Term["Proportional Term (Kp = 1/4)<br/>Corrects Phase Jitter"]:::control
        I_Term["Integral Term (Ki = 1/256)<br/>Tracks Crystal Mismatch (PPM)<br/>Includes Anti-Windup"]:::control
        Sum["Sum Correction:<br/>correction = P_Term + I_Term"]:::control
    end

    ApplyNudge["Adjust Local Hardware Timer Period<br/>(HwTimer: nudge period)"]:::output
    AlignCadence["Locked Cadence<br/>(RX Aligning with TX Master Clock)"]:::output

    PacketRx --> LatchTime
    LatchTime --> CalcAirtime
    CalcAirtime --> RecoverStart
    RecoverStart --> CalcError
    CalcError --> P_Term
    CalcError --> I_Term
    P_Term --> Sum
    I_Term --> Sum
    Sum --> ApplyNudge
    ApplyNudge --> AlignCadence
```

See [architecture.md](architecture.md) for the full design rationale.
