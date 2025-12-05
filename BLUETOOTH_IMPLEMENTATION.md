# Bluetooth Implementation Guide

## Overview

This project now supports Bluetooth communication between TX and RX devices using Serial Port Profile (SPP) with a 6-digit hex pairing code system.

## Architecture

### RX Side (`rx_main_bt.cpp`)
- **Bluetooth Role**: Server/Peripheral
- **Device Name**: `FPV_RX_XXX` (where XXX is a 3-digit suffix stored in EEPROM)
- **Pairing PIN**: 6-digit hex code (0-F) stored in EEPROM
- **Configuration**: Can be set via USB Serial commands or Flutter app

### TX Side (`tx_main_bt.cpp`)
- **Bluetooth Role**: Client/Central
- **Menu System**: OLED-based navigation using analog sticks
- **Features**:
  - Device scanning
  - Device selection from list
  - PIN entry (6 hex digits using analog sticks)
  - Connection management

### Flutter App (`fpv_rx_config/`)
- **Purpose**: Configure RX pairing PIN from phone
- **Features**:
  - Scan for FPV RX devices
  - Connect to RX
  - View/set pairing PIN

## Building

### RX (Bluetooth)
```bash
pio run -e rx-bt -t upload
```

### TX (Bluetooth)
```bash
pio run -e tx-bt -t upload
```

## RX Configuration

### Via USB Serial
Connect RX to computer via USB and use serial monitor (115200 baud):

```
SETPIN:123456    - Set 6-digit hex PIN
GETPIN           - Get current PIN
SETNAME:001      - Set device name suffix (e.g., FPV_RX_001)
GETNAME          - Get device name
HELP             - Show all commands
```

### Via Flutter App
1. Install Flutter app on phone
2. Scan for devices
3. Connect to RX
4. Set PIN through UI

## TX Usage

1. **Enter Menu**: Press Button 3
2. **Pairing Mode**: Navigate with Stick 1, select with Button 2
3. **Scan Devices**: Wait for scan to complete
4. **Select Device**: Use Stick 1 up/down, Button 2 to select
5. **Enter PIN**: 
   - Stick 1 left/right: Navigate between digits
   - Stick 2 up/down: Scroll through hex chars (0-F)
   - Button 2: Confirm digit, move to next
   - Button 1: Go back
6. **Connect**: After entering all 6 digits, connection is attempted

## PIN Format

- **Length**: 6 characters
- **Characters**: Hex digits (0-9, A-F)
- **Example**: `A1B2C3`, `000000`, `FFFFFF`

## Bluetooth API Notes

The implementation uses Arduino-Pico's Bluetooth APIs:
- `SerialBT` - Serial Port Profile for data transfer
- `BluetoothHCI` - Device scanning and discovery
- `BTDeviceInfo` - Device information from scans

**Note**: The exact API may vary slightly depending on Arduino-Pico version. Check the [Arduino-Pico Bluetooth documentation](https://arduino-pico.readthedocs.io/en/latest/bluetooth.html) for the latest API.

## Security

- **Pairing PIN**: Prevents unauthorized connections
- **HMAC Authentication**: Each frame includes HMAC for message integrity
- **Sequence Numbers**: Prevents replay attacks

## Limitations

- **Range**: ~10 meters (Bluetooth Classic)
- **One Connection**: Bluetooth Classic supports one active connection
- **No Web Client**: Web client only works with WiFi version

## Troubleshooting

1. **RX not found**: Ensure RX is powered on and Bluetooth is initialized
2. **Connection fails**: Check PIN is correct (6 hex digits)
3. **Menu not responding**: Check analog stick calibration
4. **PIN not saving**: Check EEPROM is initialized properly

## Future Enhancements

- BLE GATT service on RX for direct PIN configuration
- Multiple device support (store multiple pairings)
- Connection status indicators
- RSSI display on TX

