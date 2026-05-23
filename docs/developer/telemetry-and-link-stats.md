# Telemetry And Link Stats

Use direction-qualified names for telemetry:

- `downlink_telemetry`: RX to TX over the radio.
- `uplink_telemetry`: TX to RX over the radio, if added later.
- `crsf_link_statistics`: wired CRSF stats frame sent from RX to the flight controller.
- `link_stats`: internal health data, not serialized frame bytes.

Link quality accounting excludes Sync/Telemetry/Idle slots from uplink expected
counts. This prevents telemetry ratio from looking like packet loss.

```mermaid
graph TD
    classDef tick fill:#e0f2fe,stroke:#0284c7,stroke-width:2px;
    classDef decision fill:#fff7ed,stroke:#f97316,stroke-width:2px;
    classDef action fill:#f0fdf4,stroke:#16a34a,stroke-width:2px;
    classDef stat fill:#eff6ff,stroke:#2563eb,stroke-width:2px;

    Tick["Timer Tick (t)"]:::tick
    SlotType{"slotForTick(t)"}:::decision

    SyncSlot["Sync Slot<br/>(pos == 0)"]:::decision
    TlmSlot["Telemetry Slot<br/>(t % tlmRatioDenom == 0)"]:::decision
    UplinkSlot["Uplink Slot<br/>(Otherwise)"]:::decision

    ExcludeSync["Exclude from LQ Expected Count<br/>(No expected count penalty)"]:::action
    ExcludeTlm["Exclude from LQ Expected Count<br/>(Allows 1:8, 1:4 ratios without artificial loss)"]:::action

    IncExpected["Increment Expected Count<br/>expectedCount++"]:::action

    RxSuccess{"Packet Successfully<br/>Received?"}:::decision
    IncReceived["Increment Received Count<br/>receivedCount++"]:::action
    MissSlot["Count as Missed Packet<br/>(No received increment)"]:::action

    ComputeLQ["LQ Tracker Window (Last 100 Uplink Slots)<br/>LQ = (Received / Expected) * 100%"]:::stat

    SlotType -->|Sync| SyncSlot
    SlotType -->|Telemetry| TlmSlot
    SlotType -->|Uplink| UplinkSlot

    SyncSlot --> ExcludeSync
    TlmSlot --> ExcludeTlm

    UplinkSlot --> IncExpected
    IncExpected --> RxSuccess

    RxSuccess -->|Yes| IncReceived
    RxSuccess -->|No / Jitter / Interference| MissSlot

    IncReceived --> ComputeLQ
    MissSlot --> ComputeLQ
```

See [architecture.md](architecture.md) and [configuration.md](configuration.md) for
current telemetry slotting and CRSF link-statistics behavior.
