# RP2040 PPM/CRSF TX/RX for FPV Drones

## Project Overview

This project implements a complete remote control (RC) system for FPV (First Person View) drones using Raspberry Pi Pico W (RP2040) microcontrollers. The system consists of a **transmitter (TX)** that reads analog joysticks and buttons, and a **receiver (RX)** that outputs standard RC control signals compatible with flight controllers like Betaflight.

### Key Features

- **Multiple Communication Protocols**: Supports WiFi, Bluetooth Classic, and Bluetooth Low Energy (BLE)
- **Multiple Output Formats**: Generates both PPM (Pulse Position Modulation) and CRSF (Crossfire) signals
- **Security**: HMAC-based authentication and sequence number protection against replay attacks
- **Configurable**: WiFi credentials, device names, and pairing codes can be configured via serial or mobile app
- **OLED Display**: TX side includes an OLED display showing connection status, signal strength, and channel values
- **Web Interface**: WiFi version includes a web-based control interface for testing
- **Mobile App**: Flutter app for configuring RX devices via Bluetooth

## Architecture

### Transmitter (TX) Side
- Reads 2x 2-axis analog joysticks (4 channels)
- Reads 4 digital buttons/switches (4 channels)
- Displays status on SSD1306 OLED (128x64)
- Connects to RX via WiFi, Bluetooth Classic, or BLE
- Sends channel data at 50Hz (20ms intervals)
- Implements security with HMAC and sequence numbers

### Receiver (RX) Side
- Receives channel data from TX
- Outputs PPM or CRSF signals to flight controller
- Can operate as WiFi Access Point or connect to existing WiFi network
- Supports web interface for configuration and testing (WiFi version)
- Implements security validation on received frames

## Hardware Requirements

### TX Side (Remote Control)
- **Microcontroller**: Raspberry Pi Pico W (RP2040 with WiFi)
- **Display**: SSD1306 OLED (128x64, I2C)
- **Inputs**:
  - 2x 2-axis analog joysticks (4 analog inputs)
  - 4x buttons/switches
- **Wiring**:
  - OLED: SDA → GP4, SCL → GP5
  - Stick 1 X: GP26 (A0)
  - Stick 1 Y: GP27 (A1)
  - Stick 2 X: GP28 (A2)
  - Stick 2 Y: GP29 (A3)
  - Button 1: GP6
  - Button 2: GP7
  - Button 3: GP8
  - Button 4: GP9

### RX Side (Flight Controller Receiver)
- **Microcontroller**: Raspberry Pi Pico W (RP2040 with WiFi)
- **Outputs**:
  - PPM: GP0 (for PPM version)
  - CRSF: GP8 (TX), GP9 (RX) (for CRSF version)

## Communication Protocols

### 1. WiFi (Original Implementation)
- **TX**: Connects to RX's WiFi Access Point
- **RX**: Creates WiFi AP or connects to existing network
- **Protocol**: Binary TCP frames on port 8888
- **Discovery**: UDP broadcasts on port 8889
- **Web Interface**: HTTP server on port 80 (RX side)
- **Features**: Web-based control interface, UDP discovery

### 2. Bluetooth Classic
- **TX**: BLE Central role, scans and connects to RX
- **RX**: Bluetooth Server/Peripheral role
- **Pairing**: 6-digit hex PIN (0-F)
- **Protocol**: Serial Port Profile (SPP)
- **Configuration**: Via USB serial or Flutter mobile app
- **Features**: Menu-driven pairing on TX OLED display

### 3. Bluetooth Low Energy (BLE)
- **TX**: BLE Central role with OLED menu system
- **RX**: BLE Peripheral with GATT services
- **Pairing**: 6-digit numeric passkey (0-9)
- **Protocol**: Custom GATT service with characteristics
- **Features**: Auto-reconnect to bonded devices, lower power consumption

## Output Formats

### PPM (Pulse Position Modulation)
- Standard RC protocol supported by most flight controllers
- 8 channels, 50Hz frame rate (20ms frames)
- Channel range: 1000-2000 microseconds
- Configurable polarity (normal/inverted)

### CRSF (Crossfire)
- Modern digital protocol used by TBS Crossfire
- Higher precision and lower latency than PPM
- Supports up to 16 channels (8 used, rest centered)
- Bidirectional (can receive telemetry from flight controller)

## Channel Mapping

1. **Stick 1 X-axis** → Aileron (Roll)
2. **Stick 1 Y-axis** → Elevator (Pitch)
3. **Stick 2 X-axis** → Throttle
4. **Stick 2 Y-axis** → Rudder (Yaw)
5. **Button 1** → Aux 1 (1000/2000)
6. **Button 2** → Aux 2 (1000/2000)
7. **Button 3** → Aux 3 (1000/2000)
8. **Button 4** → Aux 4 (1000/2000)

## Security Features

- **HMAC Authentication**: Each frame includes HMAC-SHA256 (truncated to 4 bytes) for message integrity
- **Sequence Numbers**: Prevents replay attacks by tracking frame sequence
- **Pairing Keys**: Shared secret key stored in EEPROM (WiFi version)
- **Pairing PINs/Passkeys**: Bluetooth versions use PIN/passkey for initial pairing
- **Bonding**: BLE version stores bond keys for automatic reconnection

## Building and Flashing

### Prerequisites
- PlatformIO installed
- USB cable for programming

### Build Commands

**WiFi Versions:**
```bash
# TX (WiFi)
pio run -e tx -t upload

# RX (WiFi, PPM output)
pio run -e rx -t upload

# RX (WiFi, CRSF output)
pio run -e rx-crsf -t upload
```

**Bluetooth Classic:**
```bash
# TX (Bluetooth Classic)
pio run -e tx-bt -t upload

# RX (Bluetooth Classic)
pio run -e rx-bt -t upload
```

**Bluetooth Low Energy:**
```bash
# TX (BLE)
pio run -e tx-ble -t upload

# RX (BLE)
pio run -e rx-ble -t upload
```

## Configuration

### WiFi Version
- **Default SSID**: `FPV_RX_AP`
- **Default Password**: `fpv12345678`
- **Server Port**: 8888
- **Web UI**: Access via `http://<RX_IP>` when RX is connected to WiFi network

### Bluetooth Classic
- **Device Name**: `FPV_RX_XXX` (XXX = 3-digit suffix)
- **Pairing PIN**: 6-digit hex (0-F), configurable via serial or Flutter app
- **Serial Commands** (115200 baud):
  - `SETPIN:123456` - Set 6-digit hex PIN
  - `GETPIN` - Get current PIN
  - `SETNAME:001` - Set device name suffix
  - `GETNAME` - Get device name

### BLE Version
- **Device Name**: `FPV_RX_XXX` (XXX = 3-digit suffix)
- **Pairing Passkey**: 6-digit numeric (0-9), configurable via serial
- **Serial Commands** (115200 baud):
  - `SETPASSKEY:123456` - Set 6-digit numeric passkey
  - `GETPASSKEY` - Get current passkey
  - `SETNAME:001` - Set device name suffix
  - `GETNAME` - Get device name

## Usage

### WiFi Version
1. Power on RX first - it creates a WiFi Access Point
2. Power on TX - it automatically connects to RX AP
3. TX OLED displays connection status, RSSI, and channel values
4. RX outputs PPM/CRSF signals to flight controller

### Bluetooth Versions
1. Configure RX pairing PIN/passkey via serial or Flutter app
2. Power on RX - it starts advertising
3. Power on TX - press Button 3 to enter menu
4. Navigate to "Pairing Mode" and scan for devices
5. Select RX device and enter PIN/passkey
6. Connection is established automatically

## Project Structure

```
rp2040-ppm-tx-rx/
├── src/                    # Main application code
│   ├── tx_main.cpp        # WiFi TX
│   ├── tx_main_bt.cpp    # Bluetooth Classic TX
│   ├── tx_main_ble.cpp   # BLE TX
│   ├── rx_main.cpp       # WiFi RX (CRSF)
│   ├── rx_main_bt.cpp    # Bluetooth Classic RX
│   ├── rx_main_ble.cpp   # BLE RX
│   └── rx_main_ppm.cpp   # WiFi RX (PPM)
├── lib/                   # Library code
│   ├── PPMGenerator/     # PPM signal generation
│   ├── CRSF/             # CRSF protocol implementation
│   ├── Protocol/         # Binary protocol encoding/decoding
│   ├── Security/         # HMAC and security functions
│   └── WiFiConfig/       # WiFi configuration management
├── fpv_rx_config/        # Flutter mobile app for RX configuration
├── platformio.ini        # Build configuration
└── README.md             # Original README
```

## Technical Details

### Protocol Frame Format (WiFi)
- **Total Size**: 22 bytes
- **Channel Data**: 16 bytes (8 channels × 2 bytes, big-endian)
- **Sequence Number**: 2 bytes
- **HMAC**: 4 bytes (truncated HMAC-SHA256)
- **Update Rate**: 50Hz (20ms intervals)

### PPM Signal Generation
- Uses RP2040 hardware timers and interrupts
- Precise microsecond-level timing
- Non-blocking interrupt-driven operation
- Configurable frame rate and channel count

### CRSF Protocol
- Uses hardware UART (Serial2) at 420000 baud
- Implements full CRSF packet structure
- Supports bidirectional communication (telemetry)
- Channel values converted from 1000-2000μs to CRSF 11-bit format

## Mobile App (Flutter)

The `fpv_rx_config/` directory contains a Flutter mobile application for configuring RX devices:
- Scan for Bluetooth devices
- Connect to RX
- View and set pairing PIN (Bluetooth Classic)
- Configure device name

## Future Enhancements

- Multiple TX support with channel assignment
- Telemetry support (battery voltage, GPS, etc.)
- Configurable channel mapping
- Multiple bonded device support (BLE)
- Factory reset functionality
- RSSI display improvements
- Connection status indicators

## Documentation

- `README.md` - Original WiFi implementation guide
- `BLUETOOTH_IMPLEMENTATION.md` - Bluetooth Classic details
- `BLE_IMPLEMENTATION.md` - BLE implementation guide

## License

[Add license information if applicable]

## Contributing

[Add contribution guidelines if applicable]

