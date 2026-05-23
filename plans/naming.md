# XLRS Naming Glossary

This document is the project vocabulary source of truth. Prefer these terms in
code, tests, comments, docs, and review feedback. The goal is to make direction,
layer boundaries, and radio timing obvious at the call site.

## Core Rules

- `TX` and `RX` name device roles only.
- `uplink` and `downlink` name over-the-air direction.
- `UART`, `CRSF`, and `SPI` name wired protocols or buses.
- `rf_channel` means a radio frequency slot.
- `rc_channel` means a stick/switch/control channel.
- `tick` means the scheduler packet counter.
- `slot` means the scheduled action assigned to a tick.
- `frame` means a typed protocol unit on a specific transport.
- `payload` means the bytes/data carried inside a frame.

Avoid short names like `rx`, `tx`, `tlm`, `ch`, `pkt`, or `stats` in public APIs
when the direction or layer is not already explicit.

## Device Roles And Directions

| Term | Meaning | Avoid Using For |
| --- | --- | --- |
| `TX` | Ground-side radio module. Reads controller data and sends OTA uplink frames. | UART transmit pin, generic sender |
| `RX` | Aircraft-side radio module. Receives OTA uplink frames and emits CRSF to the FC. | UART receive pin, generic receiver |
| `uplink` | OTA direction from TX to RX. Mostly RC control data. | CRSF output, UART input |
| `downlink` | OTA direction from RX to TX. Mostly telemetry and link-health data. | RX-side CRSF output |
| `controller` | Handset/controller-side peer that feeds channel data into the TX module. | Flight controller |
| `flight_controller`, `fc` | Aircraft-side controller that receives CRSF from the RX module. | Handset/controller |
| `sender` | A role-local temporary actor in a narrow function. | Device role, persistent API name |
| `receiver` | A role-local temporary actor in a narrow function. | Device role, persistent API name |

Preferred examples:

```cpp
send_uplink_frame();
handle_downlink_telemetry();
emit_crsf_to_fc();
read_controller_uart();
```

Avoid:

```cpp
send_rx_packet();   // Role and direction are ambiguous.
handle_tlm();       // Direction and transport are missing.
channel_count;      // RF channel or RC channel?
```

## Transports And Frame Names

| Term | Meaning | Avoid Using For |
| --- | --- | --- |
| `ota_frame` | Versioned frame carried over the radio. | CRSF frame, UART frame |
| `crsf_frame` | Wired CRSF frame between RX module and flight controller. | OTA frame |
| `uart_frame` | Wired controller-side frame consumed by the TX module. | OTA or CRSF frame |
| `spi_transaction` | SPI exchange with the SX1280 or another peripheral. | Radio frame |
| `payload` | Data inside a frame after transport/type metadata. | Whole frame |
| `rc_payload` | Packed RC channels inside an OTA uplink frame. | CRSF bytes |
| `telemetry_payload` | Telemetry bytes/data inside an OTA telemetry frame. | `LinkStats` object |

Use transport-qualified names whenever a function crosses a layer boundary:

```cpp
build_ota_rc_uplink_frame();
decode_ota_downlink_telemetry();
build_crsf_link_statistics_frame();
```

## Radio Timing

| Term | Meaning | Avoid Using For |
| --- | --- | --- |
| `tick` | Monotonic scheduler packet counter. One tick is one RF slot opportunity. | Wall-clock time, loop iteration |
| `slot` | Scheduled action for a tick: uplink, downlink telemetry, sync, bind, idle. | Queue entry, buffer index |
| `slot_start_us` | Canonical time boundary for the RF slot. | ISR fire time |
| `prepare_lead_time_us` | Time before slot start used to prepare FIFO/PHY state. | Airtime |
| `interval_us` | Packet period / scheduler cadence from `RateConfig`. | Airtime |
| `airtime_us` | Time a frame physically occupies the RF channel. | Scheduler interval |
| `turnaround_us` | Half-duplex TX/RX direction-switch budget. | Hop interval |
| `deadline` | Latest time a prep or PHY action can complete without missing the slot. | Expected packet arrival |

Slot names should include direction when useful:

```cpp
enum class SlotType {
    Uplink,
    DownlinkTelemetry,
    Sync,
    Bind,
    Idle,
};
```

## RF, FHSS, And Rates

| Term | Meaning | Avoid Using For |
| --- | --- | --- |
| `rate` | One `RateConfig` row: interval, hop interval, telemetry ratio, modulation, airtime. | UART baud rate, SPI clock |
| `modulation` | Radio modulation mode, e.g. LoRa or FLRC. | Packet rate |
| `hop_index` | Index into the generated FHSS sequence for the current tick. | Raw RF channel number |
| `rf_channel` | Regional frequency-table entry used by FHSS. | RC control channel |
| `frequency_hz` | Actual tuned radio frequency. | FHSS index |
| `fhss_sequence` | UID-seeded sequence of RF channel indices. | Region frequency table |
| `acquisition_channel` | UID-derived fixed channel where unlocked RX waits for sync. | First RC channel |
| `sync_word` | Radio hardware sync word derived from Link UID. | Sync frame |

Use `rc_channel` for sticks/switches and `rf_channel` for frequencies. Never use
plain `channel` in public APIs unless the type name already disambiguates it.

## RC Control Data

| Term | Meaning | Avoid Using For |
| --- | --- | --- |
| `rc_channel` | One stick/switch/control channel value. | RF channel |
| `rc_channels` | Complete control-channel snapshot. | Backlog or stream |
| `channel_snapshot` | All RC channels copied as one all-or-nothing value. | Per-channel atomic values |
| `latest_channels` | Most recent controller channel snapshot available to RF. | FIFO of old channel sets |
| `failsafe` | Link-loss behavior after debounced missed uplink slots. | Any transient packet miss |
| `no_pulses` | Failsafe mode that stops CRSF RC frames. | Zero-valued channel output |
| `hold` | Optional failsafe mode that holds last valid channel values. | Normal stale snapshot reuse |

## Telemetry And Link Health

| Term | Meaning | Avoid Using For |
| --- | --- | --- |
| `telemetry` | Generic category for non-control data moving through the link. | A specific frame type without context |
| `downlink_telemetry` | OTA telemetry sent RX to TX. Primary telemetry direction. | CRSF link statistics sent to the FC |
| `uplink_telemetry` | OTA telemetry sent TX to RX, if added later. | RC channels |
| `telemetry_slot` | RF slot where RX transmits downlink telemetry. | Payload, queue, or frame |
| `telemetry_payload` | Data carried inside an OTA telemetry frame. | Whole OTA frame |
| `telemetry_frame` | OTA frame whose type is telemetry. | CRSF `LINK_STATISTICS` frame |
| `link_stats` | Internal link-health data: LQ, RSSI, SNR, drops, deadlines. | Serialized CRSF frame bytes |
| `crsf_link_statistics` | CRSF frame emitted by RX to the flight controller. | OTA telemetry |
| `uplink_lq` | How well RX hears TX over uplink slots. | Downlink reception ratio |
| `downlink_lq` | How well TX hears RX telemetry slots. | Uplink quality |
| `uplink_rssi_dbm` | RSSI measured at RX for uplink frames. | TX-side downlink RSSI |
| `downlink_rssi_dbm` | RSSI measured at TX for downlink frames, if tracked. | RX-side uplink RSSI |
| `uplink_snr_db` | SNR measured at RX for uplink frames. | Downlink SNR |
| `downlink_snr_db` | SNR measured at TX for downlink frames, if tracked. | Uplink SNR |

Preferred examples:

```cpp
struct LinkStats {
    uint8_t uplink_lq;
    uint8_t downlink_lq;
    int8_t uplink_rssi_dbm;
    int8_t downlink_rssi_dbm;
    int8_t uplink_snr_db;
    int8_t downlink_snr_db;
};
```

## Identity, Binding, And Sessions

| Term | Meaning | Avoid Using For |
| --- | --- | --- |
| `binding_phrase` | Shared human-entered phrase used to derive link identity. | Device serial |
| `link_uid` | 64-bit value derived from binding phrase; seeds FHSS and sync word. | Hardware board ID |
| `device_serial` | Hardware-specific MCU ID for diagnostics. | Pairing or FHSS seeding |
| `bind` | Operation that establishes or stores shared link identity. | Runtime connection only |
| `connect` | Runtime session establishment after identity is known. | First-time binding |
| `session` | One connected runtime lifetime with its own counters/salt. | Persistent binding |
| `session_salt` | Per-session nonce salt for crypto. | Link UID, binding phrase |
| `packet_counter` | Wide synchronized tick/counter used for crypto nonces. | Received-packet count |
| `nonce` | Per-packet cryptographic nonce derived from salt/counter/hop index. | Packet ID |

Keep persistent identity (`binding_phrase`, `link_uid`, `device_serial`) separate
from runtime session values (`session_salt`, `packet_counter`, `nonce`).

## Module Names And Ownership

| Term | Meaning |
| --- | --- |
| `Link` | Slow lifecycle owner: bind, connect, connected, failsafe, link stats. |
| `RfScheduler` | Fast per-tick owner: slot selection, FHSS advance, turnaround, PHY calls. |
| `RateConfig` | Data-only rate table row and airtime parameters. |
| `Fhss` | Hop sequence generation and RF channel lookup. |
| `OtaPacket` | Versioned radio frame representation. |
| `ChannelPack` | RC channel packing/unpacking for OTA payloads. |
| `ICipher` | Pluggable seal/open interface for OTA payload protection. |
| `IRadioPhy` | Narrow async radio-chip interface. |
| `Sx1280NativePhy` | Pico SDK SX1280 implementation of `IRadioPhy`. |
| `HwTimer` | Hardware timer adapter for scheduler cadence. |
| `Pfd` | Phase/frequency loop that locks RX timing to TX. |
| `Mailbox` | Latest-value handoff for RC channel snapshots. |
| `RingBuffer` | FIFO handoff for discrete events. |

Do not move scheduling decisions into `Link`, and do not move lifecycle/failsafe
policy into `RfScheduler`.

## Abbreviations

Allowed when the surrounding type or module is explicit:

| Abbreviation | Meaning |
| --- | --- |
| `rf` | Radio-frequency / radio side |
| `rc` | Remote-control channel data |
| `ota` | Over-the-air radio frame format |
| `crsf` | CRSF wired protocol |
| `fhss` | Frequency-hopping spread spectrum |
| `lq` | Link quality |
| `rssi` | Received signal strength indicator |
| `snr` | Signal-to-noise ratio |
| `fc` | Flight controller |
| `phy` | Radio physical-layer driver interface |

Avoid `tlm` in public APIs. Prefer `telemetry`, `downlink_telemetry`, or
`uplink_telemetry`.

## Migration Guidance

- Rename one domain at a time: timing, FHSS, OTA, telemetry, link lifecycle, app handoff.
- Prefer semantic renames over casing-only churn.
- Update tests with the code they describe.
- Use compatibility aliases only when a broad rename would obscure behavior changes.
- Add clang-tidy identifier checks after this glossary is stable, not before.
