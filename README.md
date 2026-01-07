# RP2040 SX1280 TX/RX for FPV Drone

A 2.4GHz radio-based remote control system for FPV drones using Raspberry Pi Pico (RP2040) boards with SX1280 radio modules. The system uses a split architecture where an RC board reads analog sticks and switches, communicates with the TX module via UART, and the TX module transmits to the RX module over radio. The receiver (RX) outputs CRSF protocol signals compatible with Betaflight/ELRS flight controllers.

> **Note:** This implementation replaces the previous BLE-based system. The SX1280 radio provides better range, lower latency, and more reliable communication compared to BLE. The legacy BLE implementations (`tx-ble`, `rx-ble`) are still available but deprecated.

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
3. Build and upload:
   - For TX Module: `pio run -e tx_sx128x -t upload`
   - For RX Module: `pio run -e rx_sx128x -t upload`
   - For RC Board: `pio run -e rc-crsf -t upload`

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

The system uses SX1280 radio modules with FLRC (Fast Long Range Communication) modulation:

**Radio Configuration:**
- **Frequency:** 2420 MHz (default, configurable via `SX128X_FREQ_MHZ`)
- **Modulation:** FLRC
- **Bitrate:** 1300 kbps (configurable via `SX128X_FLRC_BR_KBPS`)
- **Coding Rate:** 1/2 (configurable via `SX128X_FLRC_CR`)
- **Output Power:** 10 dBm (configurable via `SX128X_OUTPUT_POWER_DBM`)
- **Packet Length:** 33 bytes fixed (configurable via `SX128X_FIXED_PACKET_LEN`)

**Message Types:**
| Type | ID | Description |
|------|-----|-------------|
| MSG_CHANNELS | 0x01 | 8-channel control data (encrypted) |
| MSG_BATTERY | 0x02 | Battery telemetry from RX to TX |
| MSG_PAIRING | 0x03 | Pairing request with binding UID |
| MSG_PAIRING_ACK | 0x04 | Pairing acknowledgment |
| MSG_SYNC | 0x05 | Connection sync packet |
| MSG_SYNC_ACK | 0x06 | Sync acknowledgment |

**Frame Format (MSG_CHANNELS):**
- Message type byte (1 byte)
- Device ID (8 bytes)
- Channel data (16 bytes: 8 channels × 2 bytes, big-endian)
- Sequence number (2 bytes)
- HMAC-SHA256 truncated to 4 bytes
- Total: 31 bytes payload + 2 bytes overhead = 33 bytes

**Security:**
- AES-128 encryption (using pairing key)
- HMAC-SHA256 authentication (truncated to 4 bytes)
- Sequence number protection against replay attacks
- Device ID validation

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

- **Pairing fails:**
  - Make sure RX is in pairing mode (hold BOOTSEL for 5 seconds, 60 second timeout)
  - Verify binding phrase matches on both devices (check Serial for binding UID)
  - Check radio initialization (should see "SX128x ready" in Serial)
  - Ensure devices are within range (SX1280 has good range but test close first)

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

### Security
- Binding phrase-based pairing (default: "FPV_BIND_2024")
- 16-byte shared encryption key derived from pairing
- AES-128 encryption for channel data
- HMAC-SHA256 authentication (truncated to 4 bytes)
- Sequence number protection against replay attacks
- Device ID validation (8 bytes per device)
- Pairing key and device IDs stored in EEPROM

### Connection Management
- SYNC/SYNC_ACK handshake for connection establishment
- Connection state machine: DISCONNECTED → PAIRING → CONNECTING → CONNECTED
- Automatic reconnection on power-up if paired
- Connection loss detection via sync ACK timeout
- Hysteresis to prevent false disconnects (requires 3 missed sync ACKs)

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

### Radio Configuration (Configurable via Build Flags)
- **Frequency:** `SX128X_FREQ_MHZ` (default: 2420.0 MHz)
- **Bandwidth:** `SX128X_BW_KHZ` (default: 812.5 kHz)
- **Spreading Factor:** `SX128X_SF` (default: 7)
- **Coding Rate:** `SX128X_FLRC_CR` (default: 2 = 1/2)
- **Bitrate:** `SX128X_FLRC_BR_KBPS` (default: 1300 kbps)
- **Output Power:** `SX128X_OUTPUT_POWER_DBM` (default: 10 dBm)
- **Packet Length:** `SX128X_FIXED_PACKET_LEN` (default: 33 bytes)

### Pin Configuration (Configurable via Build Flags)
All SX1280 pins can be overridden in `platformio.ini`:
- SPI pins: `SX128X_SPI_SCK`, `SX128X_SPI_MOSI`, `SX128X_SPI_MISO`, `SX128X_SPI_CS`
- Control pins: `SX128X_SPI_BUSY`, `SX128X_SPI_DIO1`, `SX128X_SPI_RST`
- Enable pins: `SX128X_RXEN`, `SX128X_TXEN` (optional)

## Dependencies

- [arduino-pico](https://github.com/earlephilhower/arduino-pico) core
- [RadioLib](https://github.com/jgromes/RadioLib) v7.4.0+ - SX1280 radio driver
- EEPROM library (built-in) - For storing pairing keys and device IDs

**Note:** The RC board implementation (rc-crsf environment) has additional dependencies:
- [Adafruit SH110X](https://github.com/adafruit/Adafruit_SH110X) - OLED display driver
- [Adafruit GFX](https://github.com/adafruit/Adafruit-GFX-Library) - Graphics library
- [Adafruit ADS1X15](https://github.com/adafruit/Adafruit_ADS1X15) - ADC driver

## Build Environments

The project includes multiple build environments in `platformio.ini`:

- **`tx_sx128x`**: TX module (SX1280 radio, UART protocol)
- **`rx_sx128x`**: RX module (SX1280 radio, CRSF output)
- **`rc-crsf`**: RC board (inputs, display, UART to TX module)
- **`tx-ble`**: Legacy BLE TX (deprecated)
- **`rx-ble`**: Legacy BLE RX (deprecated)

## Important Notes

### Radio Half-Duplex Operation
The SX1280 radio operates in half-duplex mode, meaning it cannot transmit and receive simultaneously. The implementation includes timing logic to:
- Pause channel transmission briefly after sending SYNC packets to allow SYNC_ACK reception
- Alternate between TX and RX operations to maintain connection state
- Use hysteresis (3 missed sync ACKs) to prevent false disconnects from timing collisions

### Binding Phrase Security
- The default binding phrase is `"FPV_BIND_2024"` - change this for production use
- Both TX and RX must use the same binding phrase to pair
- The binding phrase is hashed to create a binding UID that is exchanged during pairing
- After pairing, devices use a shared encryption key derived from the pairing process

### Auto-Pairing Timeout
The RX module can automatically enter pairing mode after a timeout if not paired:
- Configured via `AUTO_PAIR_TIMEOUT_SEC` build flag (default: 5 seconds in `rx_sx128x` environment)
- Set to 0 to disable auto-pairing
- Maximum timeout is 120 seconds

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
