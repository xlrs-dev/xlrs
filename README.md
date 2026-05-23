# RP2040 SX1280 TX/RX for FPV Drone

A 2.4GHz radio-based remote control system for FPV drones using Raspberry Pi Pico (RP2040) boards with SX1280 radio modules. The system uses a split architecture where an RC board reads analog sticks and switches, communicates with the TX module via UART, and the TX module transmits to the RX module over radio. The receiver (RX) outputs CRSF protocol signals compatible with Betaflight/ELRS flight controllers.

## Documentation

User-focused guides for the **RC handset**, **browser config tool**, **USB HID gamepad**, and **firmware targets** are in **[`docs/`](docs/README.md)**:

| Guide | Topics |
|-------|--------|
| [**RC overview**](docs/rc-overview.md) | Stick/switch/OLED capabilities, **custom Pico TX (XLRS-style UART)** vs **ELRS CRSF handset**, USB roles |
| [**RC Web UI**](docs/rc-webui.md) | `tools/rc-webui`: **Live**, **Calibrate**, **Mapping**, **Curves**, **Save/Apply**, **Auto Pair**, **USB-UART proxy** (ELRS Buddy) |
| [**USB HID**](docs/rc-usb-hid.md) | Optional composite gamepad (`rc-HID-rp2350`), axis/button mapping, host notes |
| [**Build & flash (RC)**](docs/rc-build-flash.md) | `rc-crsf`, `rc-rp2350`, `rc-HID-rp2350`, Dhanush USB product string, UART pins |

## Architecture Overview

The system consists of three main components:
1. **RC Board**: Reads analog inputs (joysticks, switches) and sends channel data to TX module via UART
2. **TX Module**: Receives channels from RC board via UART and transmits over SX1280 radio to RX
3. **RX Module**: Receives radio signals from TX and outputs CRSF to flight controller

**Communication Flow:**
```
RC Board (Inputs) → UART (420kbps) → TX Module → SX1280 Radio (2.4GHz) → RX Module → CRSF (420kbps) → Flight Controller
                                                                                        ↓
                                                                              Battery Telemetry (CRSF)
                                                                                        ↓
RC Board (Display) ← UART Telemetry ← TX Module ← SX1280 Radio ← RX Module ← Flight Controller
```

## Hardware Requirements

### TX Side (Radio Transmitter Module)
- Raspberry Pi Pico (RP2040)
- SX1280 radio module with SPI interface
- UART connection to RC board (420000 baud)

**SX1280 Radio Wiring (Default):**
| Signal | Pin | Description |
|--------|-----|-------------|
| SPI SCK | GP18 | SPI Clock |
| SPI MOSI | GP19 | SPI Data Out |
| SPI MISO | GP16 | SPI Data In |
| SPI CS | GP17 | Chip Select |
| BUSY | GP20 | Radio Busy Signal |
| DIO1 | GP21 | Data Interrupt |
| RST | GP22 | Reset |
| RXEN | GP14 | RX Enable (optional) |
| TXEN | GP15 | TX Enable (optional) |

**UART Protocol (to RC Board):**
| Signal | Pin | Description |
|--------|-----|-------------|
| UART TX | GP8 | Transmit to RC board |
| UART RX | GP9 | Receive from RC board |
| Baudrate | 420000 | High-speed UART |

### RX Side (Flight Controller Receiver)
- Raspberry Pi Pico W (RP2040 with WiFi - for future features)
- SX1280 radio module (same wiring as TX)
- CRSF output: TX → GP8, RX → GP9 (connect to flight controller CRSF RX/TX)

**CRSF Wiring:**
| Signal | Pin | Description |
|--------|-----|-------------|
| CRSF TX | GP8 | To flight controller CRSF RX |
| CRSF RX | GP9 | From flight controller CRSF TX (telemetry) |
| Baudrate | 420000 | CRSF protocol speed |

### RC Board (Input Controller)
The RC board is a separate module that reads analog inputs and communicates with the TX module. See `rc-crsf` environment for the RC board implementation.

**Typical RC Board Components:**
- Raspberry Pi Pico
- SH1106G OLED display (128x64, I2C)
- ADS1115 16-bit ADC module (I2C)
- 2x 2-axis analog joysticks (4 analog inputs)
- 4x toggle switches
- 2x navigation buttons

**Note:** The RC board implementation is separate and communicates with the TX module via the UART protocol at 420000 baud.

## Software Setup

1. Install PlatformIO
2. Clone this repository
3. (Recommended) Use the **XLRS Developer Helper CLI Utility** to detect, flash, test, and monitor your boards with a single unified command:
   ```bash
   uv run scripts/xlrs_helper.py
   ```
   *Note: If you don't have `uv` installed, see instructions [here](https://astral.sh/uv).*

   Alternatively, you can build and upload manually:
   - For TX Module: `pio run -e xlrs_tx_native -t upload` (native SX1280 PHY) or `pio run -e xlrs_tx -t upload` (RadioLib PHY)
   - For RX Module: `pio run -e xlrs_rx_native -t upload` (native SX1280 PHY) or `pio run -e xlrs_rx -t upload` (RadioLib PHY)
   - For RC Board: `pio run -e rc-crsf -t upload` (RP2040 + ADS1115) or `pio run -e rc-rp2350 -t upload` / `pio run -e rc-HID-rp2350 -t upload` (RP2350 Dhanush; see [`docs/rc-build-flash.md`](docs/rc-build-flash.md))

### Developer Helper CLI (`scripts/xlrs_helper.py`)

A comprehensive, zero-dependency setup (via `uv`) helper to simplify hardware development. 

#### Core Features
- **Auto-Detection**: Scans USB ports and classifies connected devices into `TX` and `RX` via real-time boot log sniffing.
- **Flashing Modes**: Instantly builds and flashes either **Native PHY Mode** (`xlrs_tx_native`/`xlrs_rx_native`) or **RadioLib PHY Mode** (`xlrs_tx`/`xlrs_rx`) to the targeted boards.
- **Parallel Multi-Device Log Streaming**: Listens to both `TX` and `RX` serial feeds in parallel, printing beautifully color-coded console logs (`[TX]` in green, `[RX]` in cyan).
- **Persistent Session Logs**: Writes all activities and streamed board logs to `xlrs_session.log`.
- **Unit Test Runner**: Integrates with PlatformIO's Unity framework to run host tests.

#### Command Examples
* **Interactive CLI Menu**:
  ```bash
  uv run scripts/xlrs_helper.py
  ```
* **Direct Non-Interactive Flash & Monitor (Native Mode)**:
  ```bash
  uv run scripts/xlrs_helper.py --flash native --monitor
  ```
* **Direct Non-Interactive Flash & Monitor (RadioLib Mode)**:
  ```bash
  uv run scripts/xlrs_helper.py --flash radiolib --monitor
  ```
* **Target Specific Ports Manually**:
  ```bash
  uv run scripts/xlrs_helper.py --tx-port /dev/cu.usbmodem1101 --rx-port /dev/cu.usbmodem1201 --monitor
  ```
* **Run Host Unit Tests**:
  ```bash
  uv run scripts/xlrs_helper.py --test
  ```

**Note:** The TX module expects to receive channel data from the RC board via UART protocol. Make sure the RC board is running and connected before the TX module will transmit.

## Pairing TX and RX

The TX and RX use a binding phrase-based pairing system for secure communication. Both devices must be configured with the same binding phrase to pair.

### Pairing Process:

1. **Configure Binding Phrase (Optional):**
   - By default, both devices use the phrase: `"FPV_BIND_2024"`
   - To customize, add build flag: `-DDEFAULT_BINDING_PHRASE=\"YourCustomPhrase\"`
   - The binding phrase is hashed to create a unique binding UID

2. **Put RX in pairing mode:**
   - Hold the BOOTSEL button on the RX Pico for 5 seconds
   - RX enters pairing mode for 60 seconds (LED indicators may vary)
   - RX will also auto-enter pairing mode after a configurable timeout (default: 5 seconds) if not paired

3. **Initiate pairing from TX:**
   - TX automatically enters pairing mode if not already paired
   - TX sends pairing packets containing the binding UID
   - RX validates the binding UID and responds with pairing ACK
   - Both devices exchange device IDs and generate a shared encryption key
   - The pairing key and device IDs are stored in EEPROM

4. **After successful pairing:**
   - Devices will automatically reconnect on subsequent power-ups
   - Connection is established via SYNC/SYNC_ACK handshake
   - Channel data transmission begins once connected

### Pairing via RC Board (if implemented):
- Send `UART_MSG_CMD_PAIR` command from RC board to TX module
- TX module enters pairing mode and attempts to pair with RX

## Channel Mapping

The channel mapping is determined by the RC board implementation. Typical mapping:

| Channel | Source | Description |
|---------|--------|-------------|
| 0 | RC Board Ch0 | Aileron (1000-2000) |
| 1 | RC Board Ch1 | Elevator (1000-2000) |
| 2 | RC Board Ch2 | Rudder (1000-2000) |
| 3 | RC Board Ch3 | Throttle (1000-2000) |
| 4 | RC Board Ch4 | Aux1 (1000/2000) |
| 5 | RC Board Ch5 | Aux2 (1000/2000) |
| 6 | RC Board Ch6 | Aux3 (1000/2000) |
| 7 | RC Board Ch7 | Aux4 (1000/2000) |

**Note:** The RC board sends channel data to the TX module via UART protocol. The TX module forwards this data over radio without modification.

## Radio Protocol (SX1280)

The production firmware uses the clean XLRS link core in `lib/xlrs/`: hardware-timed slots, UID-seeded FHSS, deterministic sync/telemetry/uplink scheduling, true packet-success LQ, dynamic power, and CRSF link statistics.

**Radio Configuration:**
- **Rates:** table-driven FLRC/LoRa options in `lib/xlrs/link/RateConfig.h`
- **FHSS:** UID-seeded 2.4 GHz hop sequence with region plumbing
- **Addressing/isolation:** binding phrase → 8-byte Link UID → FHSS seed + SX1280 sync word + sync UID CRC
- **OTA:** versioned type-multiplexed frames with compact RC support and hardware CRC
- **Optional cipher:** `ICipher`/AEAD hook exists, but the production control path uses UID-seeded isolation plus PHY CRC by default

**Update Rate:**
- Channel data: ~50Hz (20ms intervals, limited by radio half-duplex timing)
- Sync packets: Periodic to maintain connection state
- Battery telemetry: 5Hz (200ms intervals)

## Battery Telemetry

The RX receives battery telemetry from the flight controller via CRSF and sends it to the TX over radio:

- **Source:** Flight controller sends `CRSF_FRAMETYPE_BATTERY_SENSOR` (0x08) frames
- **CRSF Wiring:** GP8 (TX to FC), GP9 (RX from FC for telemetry)
- **Update rate:** 5Hz (200ms intervals) from RX to TX
- **Display:** RC board OLED shows voltage and remaining percentage (if implemented)

**Telemetry Data (MSG_BATTERY):**
| Field | Format | Description |
|-------|--------|-------------|
| Voltage | V × 10 (16-bit BE) | Battery voltage in 0.1V units |
| Current | A × 10 (16-bit BE) | Battery current in 0.1A units |
| Remaining | 8-bit | Battery percentage (0-100) |

**UART Telemetry (TX to RC Board):**
The TX module also sends telemetry to the RC board via UART:
- RSSI (signal strength in dBm)
- SNR (signal-to-noise ratio in dB)
- RX Battery voltage and percentage (if received from RX)
- Link quality (0-100% calculated from RSSI)

## Usage

1. **Power on the RX module first:**
   - RX initializes radio and enters pairing mode if not already paired
   - RX will auto-enter pairing mode after timeout (default: 5 seconds) if unpaired
   - RX outputs CRSF signals on GP8 once connected and receiving data

2. **Power on the RC board:**
   - RC board initializes display and inputs
   - RC board begins sending channel data to TX module via UART

3. **Power on the TX module:**
   - TX module initializes radio and UART protocol
   - If previously paired, TX automatically attempts to connect to paired RX
   - If not paired, TX enters pairing mode and sends pairing packets
   - Once connected, TX forwards channel data from RC board to RX over radio

4. **Connection States:**
   - **DISCONNECTED:** No pairing, waiting for pairing
   - **PAIRING:** Sending/receiving pairing packets
   - **CONNECTING:** Paired, establishing connection via SYNC handshake
   - **CONNECTED:** Active link, transmitting channel data
   - **LOST:** Connection timeout, attempting to reconnect

### Connection Indicators

- **Serial Monitor (115200 baud):** Shows connection state, RSSI, SNR, and packet statistics
- **RC Board Display (if implemented):** Shows connection status, signal strength, and channel values
- **RX Serial Monitor:** Shows received channels, connection state, and CRSF output status

### UART Protocol Commands (RC Board to TX Module)

The RC board can send commands to the TX module:
- `UART_MSG_CMD_PAIR` (0x10): Enter pairing mode
- `UART_MSG_CMD_BOND` (0x11): Check bonding status
- `UART_MSG_CMD_RESTART` (0x12): Restart TX module
- `UART_MSG_CMD_STATUS_REQ` (0x13): Request status update

## Troubleshooting

- **TX won't connect to RX:**
  - Ensure RX is powered and in pairing mode (hold BOOTSEL for 5 seconds)
  - Verify both devices use the same binding phrase (check Serial output for binding UID)
  - Check radio wiring (SPI, BUSY, DIO1, RST pins)
  - Monitor Serial output (115200 baud) for connection state and error messages

- **Pairing fails / TX and RX not pairing anymore:**
  - Put **RX** in pairing mode: hold BOOTSEL for 5 seconds (60 second timeout).
  - Put **TX** in pairing mode in one of two ways:
    - **Option A:** On the RC main screen, press **ENTER** to send the PAIR command to the TX (TX then sends pairing packets).
    - **Option B:** Leave the system on; if the TX was already paired but gets no response from the RX, after about 60 seconds the TX automatically switches to pairing mode and sends pairing packets.
  - Ensure both use the same binding phrase (e.g. `Kikobot-02` in build flags; check Serial for binding UID).
  - Check radio init on both sides (Serial: "SX128x ready") and that devices are in range.

- **No CRSF output from RX:**
  - Verify RX is connected (check Serial for "CONNECTED" state)
  - Check CRSF wiring (GP8 to FC RX, GP9 from FC TX)
  - Verify CRSF baudrate is 420000 on flight controller
  - Monitor Serial for "[CRSF OUT]" messages showing frame transmission

- **No channel data from RC board:**
  - Verify UART wiring between RC board and TX module (GP8/GP9, 420000 baud)
  - Check RC board is powered and sending data
  - Monitor TX Serial for UART protocol messages
  - Verify RC board implementation is correct

- **Radio not initializing:**
  - Check all SPI connections (SCK, MOSI, MISO, CS)
  - Verify BUSY and DIO1 pins are connected
  - Check RST pin connection
  - Ensure proper power supply (3.3V)
  - Check Serial for "SX128x init failed" messages

- **Connection drops frequently:**
  - Check RSSI and SNR values in Serial (should be > -90 dBm RSSI)
  - Verify antenna connection
  - Check for interference on 2.4GHz band
  - Monitor sync ACK misses (should be 0 when connected)

## Technical Details

### Radio Stack (SX1280)
- Uses [RadioLib](https://github.com/jgromes/RadioLib) library (v7.4.0+)
- SX1280 radio module with FLRC modulation
- Half-duplex communication (TX and RX alternate)
- Interrupt-driven packet reception (DIO1 pin)
- Hardware SPI for radio communication

### Binding Identity
- Binding phrase-based Link UID derivation
- UID seeds the FHSS sequence and SX1280 sync word
- Sync beacons carry a UID CRC and mismatches are rejected before lock
- Binding UID is stored by `xlrs::BindingStore`; legacy pairing keys/device IDs are not part of the production link

### Connection Management
- Hardware-timed sync, uplink, and telemetry slots
- Connection state machine with sync/FHSS lock and valid-RC gating before output
- Failsafe via CRSF NoPulses by default
- True LQ from received-vs-expected packet windows

### CRSF Output
- Standard CRSF protocol at 420000 baud
- 16 channels (8 active from radio, 8 centered at 1500µs)
- 50Hz frame rate (20ms intervals)
- Bidirectional: receives battery telemetry from flight controller
- Compatible with Betaflight, ELRS, and other CRSF receivers

### UART Protocol (RC Board ↔ TX Module)
- High-speed UART at 420000 baud
- Frame-based protocol with CRC8 checksum
- Message types: Channels, Telemetry, Status, Commands, ACK/Error
- Non-blocking state machine for reception
- Callback-based message handling

### Radio Configuration
- Rate/modulation table: `lib/xlrs/link/RateConfig.h`
- Persistent runtime RF config: `xlrs::RfConfigData`
- Output power cap and dynamic-power enable are persisted in EEPROM
- Region is persisted and passed into the link/FHSS layer

### Pin Configuration (Configurable via Build Flags)
All SX1280 pins can be overridden in `platformio.ini`:
- SPI pins: `SX128X_SPI_SCK`, `SX128X_SPI_MOSI`, `SX128X_SPI_MISO`, `SX128X_SPI_CS`
- Control pins: `SX128X_SPI_BUSY`, `SX128X_SPI_DIO1`, `SX128X_SPI_RST`
- Enable pins: `SX128X_RXEN`, `SX128X_TXEN` (optional)

## Dependencies

- [arduino-pico](https://github.com/earlephilhower/arduino-pico) core
- [RadioLib](https://github.com/jgromes/RadioLib) v7.4.0+ - SX1280 radio driver
- EEPROM library (built-in) - For RF config, binding UID, and RC calibration

**Note:** The RC board implementation (rc-crsf environment) has additional dependencies:
- [Adafruit SH110X](https://github.com/adafruit/Adafruit_SH110X) - OLED display driver
- [Adafruit GFX](https://github.com/adafruit/Adafruit-GFX-Library) - Graphics library
- [Adafruit ADS1X15](https://github.com/adafruit/Adafruit_ADS1X15) - ADC driver

### USB-UART Proxy Mode (ELRS module passthrough)

The RC can bridge **USB CDC ↔ handset UART** so tools like [ELRS Buddy](https://github.com/fourflies/elrsbuddy) talk directly to the **ELRS TX module** on **GP8/GP9** (baud per your `platformio.ini`, often 400000 for ELRS handset). Full step-by-step and caveats (HID pauses, Web Serial) are in **[`docs/rc-webui.md`](docs/rc-webui.md)**.

## Build Environments

The project includes multiple build environments in `platformio.ini`:

- **`xlrs_tx` / `xlrs_tx_native`**: TX module, clean XLRS link core
- **`xlrs_rx` / `xlrs_rx_native`**: RX module, clean XLRS link core with CRSF output
- **`rc-crsf`**: RC board — RP2040, ADS1115 ADC, OLED, UART to TX / ELRS module
- **`rc-rp2350`**: RC — RP2350 (Dhanush), internal ADC, Core1 CRSF handset path
- **`rc-HID-rp2350`**: Same as `rc-rp2350` plus **USB HID gamepad** (composite with CDC)
- **`rc-rp2350-debug`**: `rc-rp2350` with extra link-rate logging on Serial

RC-centric details: **[`docs/rc-build-flash.md`](docs/rc-build-flash.md)**.

## Important Notes

### Radio Half-Duplex Operation
The SX1280 radio operates in half-duplex mode. XLRS schedules sync, uplink, and telemetry into deterministic slots so TX/RX do not collide.

### Binding Phrase
- The default binding phrase is set with `DEFAULT_BINDING_PHRASE`
- Both TX and RX must use the same binding phrase
- The binding phrase is hashed into the Link UID used for FHSS, sync word, and sync UID CRC

### UART Protocol
The UART protocol between RC board and TX module uses:
- 420000 baud (high-speed for low latency)
- Frame-based protocol with CRC8 checksum
- Non-blocking state machine for reliable reception
- Callback-based message handling for channels, telemetry, and commands

### Radio Range and Performance
- Default output power: 10 dBm (configurable up to 13 dBm)
- Typical range: 1-2 km line-of-sight (depends on antenna and environment)
- RSSI values: Good connection typically > -90 dBm
- SNR: Higher is better, typically 5-15 dB in good conditions

### Connection State Machine
The system uses a robust connection state machine:
- **DISCONNECTED**: Initial state, waiting for pairing
- **PAIRING**: Exchanging pairing packets with binding UID
- **CONNECTING**: Paired, establishing connection via SYNC handshake
- **CONNECTED**: Active link, transmitting channel data
- **LOST**: Connection timeout detected, attempting to reconnect

Connection loss is detected by monitoring SYNC_ACK reception with hysteresis to prevent false disconnects.
