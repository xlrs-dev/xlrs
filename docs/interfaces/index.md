# XLRS TX/RX Interface

This document describes how external hardware and software interface with the
current XLRS TX and RX firmware.

## System Boundary

The firmware in this repository is only the radio bridge:

```text
Controller UART or CRSF -> xlrs_tx -> SX1280 RF link -> xlrs_rx -> CRSF UART -> Flight controller
```

Handset UI, model memory, web configuration, battery management, and flight
controller policy are outside this firmware.

## Physical Interfaces

Default controller/CRSF UART:

| Signal | Default |
| --- | --- |
| UART TX | GP8 |
| UART RX | GP9 |
| Baud | 420000 |

Default SX1280 interface:

| Signal | Default |
| --- | --- |
| SPI SCK | GP18 |
| SPI MOSI | GP19 |
| SPI MISO | GP16 |
| SPI CS | GP17 |
| BUSY | GP20 |
| DIO1 | GP21 |
| RST | GP22 |
| RXEN | GP14 |
| TXEN | GP15 |

RX status LED defaults to GP13.

All defaults are CMake cache variables and can be overridden when configuring
the build. The TX controller protocol is selected with
`XLRS_TX_CONTROLLER_PROTOCOL`:

| Value | Controller-facing behavior |
| --- | --- |
| `UART` | Custom XLRS controller UART protocol. This is the default. |
| `CRSF` | Controller-facing CRSF for EdgeTX/OpenTX-style handsets and module bays. |

Examples:

```bash
cmake -S . -B build -G Ninja -DXLRS_TX_CONTROLLER_PROTOCOL=UART
cmake -S . -B build-crsf -G Ninja -DXLRS_TX_CONTROLLER_PROTOCOL=CRSF
```

## Binding

Binding is Link-UID based. A Link UID can be derived from a local binding phrase
or learned through the OTA bind workflow.

Both modules must have the same binding phrase. On boot each module:

1. Loads the persisted binding UID from flash.
2. If no valid binding record exists, writes the default binding phrase.
3. Derives an 8-byte Link UID from the phrase using 64-bit FNV-1a.
4. Uses that UID to derive:
   - FHSS sequence seed.
   - SX1280 sync word.
   - UID CRC carried in sync beacons.

If the phrases differ, the modules use different sync words and FHSS sequences,
and the RX rejects sync frames whose UID CRC does not match.

### OTA Bind

When the controller-facing CRSF Bind RX command is triggered, TX temporarily
switches to a shared XLRS bind identity for about 30 seconds and transmits a
bind frame containing its current Link UID. An unconnected RX periodically
alternates between normal acquisition and short bind-scan windows on that shared
bind identity. If it receives a valid bind frame while scanning, it persists the
offered Link UID and reboots.

The RX does not check the normal binding phrase during bind scan; doing so would
prevent first-time pairing. It only accepts bind frames while explicitly in
bind-scan mode on the shared bind identity. This is discovery/pairing, not a
cryptographic ownership proof.

### Compile-Time Binding

The default phrase is set at configure time:

```bash
cmake -S . -B build -G Ninja -DXLRS_DEFAULT_BINDING_PHRASE="your-phrase"
cmake --build build --target xlrs_tx xlrs_rx
```

Flash both TX and RX with builds that use the same phrase.

### Runtime TX Binding Command

The TX module currently supports setting its binding phrase over the controller
UART using `UART_MSG_CMD_SET_BIND_TX` (`0x16`). The payload is the ASCII binding
phrase, 1 to 32 bytes, with no trailing NUL required.

When accepted, TX:

1. Sends `ACK` for `UART_MSG_CMD_SET_BIND_TX`.
2. Persists the derived UID to flash.
3. Reboots.

Current limitation: `UART_MSG_CMD_SET_BIND_RX` (`0x17`) exists in the protocol
enum/parser, but `xlrs_rx` does not run `UARTProtocol`, and `xlrs_tx` ignores
that command. There is no implemented external command path for setting the RX
binding phrase at runtime.

## TX Controller UART Protocol

The default TX-side controller protocol is a simple framed UART protocol:

```text
[sync 0xA5][payload_length][message_type][payload bytes...][crc8]
```

CRC is CRC8 with polynomial `0xD5`, calculated over:

```text
[payload_length][message_type][payload bytes...]
```

Maximum payload length is 60 bytes.

### Message Types

| Type | Value | Direction | Status |
| --- | ---: | --- | --- |
| `UART_MSG_CHANNELS` | `0x01` | Controller -> TX | Implemented |
| `UART_MSG_CMD_PAIR` | `0x10` | Controller -> TX | ACK only; no RF behavior |
| `UART_MSG_CMD_BOND` | `0x11` | Controller -> TX | ACK only |
| `UART_MSG_CMD_RESTART` | `0x12` | Controller -> TX | ACK, then reboot |
| `UART_MSG_CMD_STATUS_REQ` | `0x13` | Controller -> TX | ACK only |
| `UART_MSG_PING` | `0x14` | Either | Responds with PONG |
| `UART_MSG_PONG` | `0x15` | Either | Callback support |
| `UART_MSG_CMD_SET_BIND_TX` | `0x16` | Controller -> TX | Implemented |
| `UART_MSG_CMD_SET_BIND_RX` | `0x17` | Reserved | Parsed, not wired to RX |
| `UART_MSG_TELEMETRY` | `0x20` | TX -> Controller | Implemented |
| `UART_MSG_STATUS` | `0x21` | TX -> Controller | Implemented |
| `UART_MSG_ACK` | `0x22` | TX -> Controller | Implemented |
| `UART_MSG_ERROR` | `0x23` | Either | Frame support only |

### Channel Input

`UART_MSG_CHANNELS` payload:

```c
struct ChannelData {
    uint16_t channels[8];  // 1000-2000 us expected by the app layer
} __attribute__((packed));
```

The current TX app copies all 8 channels into the RF mailbox. The link layer then
masks values to 11 bits before OTA packing.

### TX Telemetry Output

The TX sends `UART_MSG_TELEMETRY` about every 200 ms when RF data is available:

```c
struct TelemetryData {
    int16_t rssi;        // dBm
    float   snr;         // dB
    uint16_t rxBattMv;   // currently not populated by TX app
    uint8_t rxBattPct;   // currently not populated by TX app
    uint8_t linkQuality; // downlink LQ, 0-100
} __attribute__((packed));
```

### TX Status Output

The TX sends `UART_MSG_STATUS` about every 1000 ms when RF data is available:

```c
struct StatusData {
    uint8_t  connectionState; // 0 Disconnected, 1 Binding, 2 Connecting,
                              // 3 Connected, 4 Failsafe
    uint8_t  pairingState;    // 0 Unpaired, 1 Normal, 2 Bind TX active
    uint32_t packetsReceived; // currently not populated by TX app
    uint32_t packetsLost;     // currently mapped from missedDeadlines
} __attribute__((packed));
```

## TX Controller CRSF Protocol

When built with `-DXLRS_TX_CONTROLLER_PROTOCOL=CRSF`, the TX module uses the same
controller UART pins and baud for CRSF.

Implemented today:

- `CRSF_FRAMETYPE_RC_CHANNELS_PACKED` input from the controller.
- Channels 1-8 are translated from CRSF channel units to 1000-2000 us and copied
  into the TX RF mailbox.
- CRSF `LINK_STATISTICS` output from TX to the controller about every 200 ms when
  RF data is available.
- CRSF device ping/device info for TX module discovery.
- CRSF parameter read/write for Rate, Max Power, Dynamic Power, Region, Failsafe,
  Bind RX, and Reboot.
- TX forwards critical RF config writes to RX over XLRS uplink telemetry. RX
  persists them to flash.
- RX forwards valid flight-controller CRSF telemetry frames to TX over XLRS
  downlink telemetry. TX writes those frames back to the controller CRSF port.

Current limitation: parameter writes persist config to flash, but RF
rate/region/power/failsafe changes apply after reboot. A dedicated XLRS Lua
script is not implemented yet. XLRS currently carries 8 OTA `rc_channel` values,
so CRSF channels 9-16 are not transmitted over the XLRS uplink. See
[CRSF Features](../crsf/index.md) and [CRSF Binding](../crsf/binding.md) for the
CRSF-specific support matrix.

## RX Flight Controller Interface

The RX module outputs CRSF to the flight controller on the CRSF UART.

When the RF link is connected, RX emits packed CRSF RC channels. Channels 1-8 are
mapped from 1000-2000 us into CRSF channel units. Channels 9-16 are held at CRSF
midpoint.

RX also emits CRSF `LINK_STATISTICS` about every 500 ms.

Failsafe behavior is controlled by RF config:

| Mode | Value | Behavior |
| --- | ---: | --- |
| `NoPulses` | `0` | Stop CRSF RC output when link is not valid |
| `Hold` | `1` | Continue CRSF output during failsafe using the app's preset failsafe channel values |

Default mode is `NoPulses`.

The current RX app's failsafe preset is roll/pitch/yaw centered, throttle low,
and channels 5-8 centered.

## Over-the-Air Cycle

TX is the time master. RX locks its timer to received packet timing.

Each scheduler tick chooses one slot:

| Slot | Sender | Receiver | Purpose |
| --- | --- | --- | --- |
| Sync | TX | RX | Acquisition, FHSS phase, active/pending rate, UID CRC |
| Uplink | TX | RX | RC channels or uplink MSP chunk |
| Telemetry | RX | TX | Link telemetry or downlink MSP chunk |

The hop position is derived from the synchronized tick and the configured rate's
hop interval. Once RX receives a valid sync beacon, both sides compute the same
slot and frequency sequence.

## RF Configuration

RF configuration is stored in flash at offset `0` as `RfConfigData`, validated by
magic, schema version, and CRC16. Binding state is stored separately starting at
offset `120`, so RF config and binding records do not overlap.

Current RF config fields:

```c
struct RfConfigData {
    uint32_t magic;        // "RFCG"
    uint8_t  version;      // 1
    uint8_t  region;       // 0 US, 1 EU
    uint8_t  defaultRate;  // index into kRates
    int8_t   maxPowerDbm;  // -18..13, EU capped to 10
    uint8_t  failsafeMode; // 0 NoPulses, 1 Hold
    uint8_t  dynamicPower; // 0 disabled, 1 enabled
    uint8_t  reserved[2];
    uint16_t checksum;
} __attribute__((packed));
```

Defaults:

| Field | Default |
| --- | --- |
| Region | US |
| Rate | `F1000` |
| Max power | 10 dBm |
| Failsafe | `NoPulses` |
| Dynamic power | Enabled |

Rate table:

| Index | Name | Interval | Telemetry ratio | Modulation |
| ---: | --- | ---: | ---: | --- |
| 0 | `F1000` | 1000 us | 1:64 | FLRC |
| 1 | `F500` | 2000 us | 1:32 | FLRC |
| 2 | `D250` | 4000 us | 1:16 | FLRC |
| 3 | `L150` | 6666 us | 1:8 | LoRa |
| 4 | `L50` | 20000 us | 1:4 | LoRa |

Current limitation: there is no implemented external UART/CRSF/MSP command that
edits `RfConfigData`. The firmware loads it on boot, writes defaults if invalid,
and uses it to initialize the link. A configuration tool would need a new command
path or a separate flashing/provisioning step.

## Implemented vs Reserved

Implemented today:

- TX controller channel input over UART.
- TX controller channel input over CRSF when `XLRS_TX_CONTROLLER_PROTOCOL=CRSF`
  is selected.
- TX UART telemetry/status output.
- TX CRSF link-statistics output when `XLRS_TX_CONTROLLER_PROTOCOL=CRSF` is
  selected.
- TX CRSF device info and critical RF config parameters when
  `XLRS_TX_CONTROLLER_PROTOCOL=CRSF` is selected.
- TX-mediated RX RF config persistence and RX-to-TX CRSF telemetry bridge.
- TX runtime binding phrase update.
- CRSF Bind RX for an unconnected RX using temporary OTA bind scan/transmit.
- RX CRSF RC and link-statistics output.
- Flash-backed RF config with validated defaults.
- Flash-backed binding UID with validated defaults.

Reserved or incomplete:

- Runtime RF config application without reboot.
- Cryptographic pair/bond ownership proof.
- Dedicated EdgeTX/OpenTX Lua script.
- CRSF bind phrase commands.
- External MSP/config passthrough API.
- RX battery telemetry population into TX telemetry fields.
