# BLE Implementation Guide

## Overview

This project now supports Bluetooth Low Energy (BLE) communication between TX and RX devices using GATT services with 6-digit numeric passkey pairing.

## Architecture

### RX Side (`rx_main_ble.cpp`)
- **Bluetooth Role**: BLE Peripheral
- **Device Name**: `FPV_RX_XXX` (where XXX is a 3-digit suffix stored in EEPROM)
- **Pairing Passkey**: 6-digit numeric code (0-9) stored in EEPROM
- **GATT Service**: Custom 128-bit UUID service with:
  - **TX_CHAR**: Write Without Response characteristic for receiving channel data
  - **RX_CHAR**: Notify characteristic (for future use)
- **Pairing**: Uses BLE LE Passkey Entry pairing - returns stored passkey programmatically
- **Bonding**: Stores bond keys to enable auto-reconnect

### TX Side (`tx_main_ble.cpp`)
- **Bluetooth Role**: BLE Central
- **Menu System**: OLED-based navigation using analog sticks
- **Features**:
  - BLE device scanning
  - Device selection from list (shows name, RSSI)
  - Passkey entry (6 numeric digits using analog sticks)
  - Connection management
  - Auto-reconnect to bonded devices
- **GATT Client**: Discovers custom service and uses TX_CHAR for sending channel data

## Building

### RX (BLE)
```bash
pio run -e rx-ble -t upload
```

### TX (BLE)
```bash
pio run -e tx-ble -t upload
```

## RX Configuration

### Via USB Serial
Connect RX to computer via USB and use serial monitor (115200 baud):

```
SETPASSKEY:123456    - Set 6-digit numeric passkey
GETPASSKEY           - Get current passkey
SETNAME:001          - Set device name suffix (e.g., FPV_RX_001)
GETNAME              - Get device name
HELP                 - Show all commands
```

**Note**: Passkey must be 6 numeric digits (0-9), not hex.

## TX Usage

1. **Enter Menu**: Press Button 3
2. **Pairing Mode**: Navigate with Stick 1, select with Button 2
3. **Scan Devices**: Wait for scan to complete (10 seconds)
4. **Select Device**: Use Stick 1 up/down, Button 2 to select
5. **Enter Passkey**: 
   - Stick 1 left/right: Navigate between digits
   - Stick 2 up/down: Scroll through digits 0-9
   - Button 2: Confirm digit, move to next
   - Button 1: Go back
6. **Connect**: After entering all 6 digits, connection is attempted
7. **Auto-reconnect**: On next boot, TX will automatically reconnect to the last bonded device

## Passkey Format

- **Length**: 6 characters
- **Characters**: Numeric digits only (0-9)
- **Example**: `123456`, `000000`, `999999`

**Important**: This is different from the old Bluetooth Classic implementation which used hex digits (0-F).

## BLE APIs & Integration Notes

The implementation uses the high-level **BTstackLib** wrapper API from Arduino-Pico, as shown in the [official examples](https://github.com/earlephilhower/arduino-pico/tree/master/libraries/BTstackLib/examples):

### RX (Peripheral) - Based on LEPeripheral example
- `BTstack.setup()` - Initialize BLE stack
- `BTstack.addService()` - Add GATT service
- `BTstack.addCharacteristic()` - Add GATT characteristic
- `BTstack.setGATTCharacteristicWrittenCallback()` - Handle writes
- `BTstack.setBLEDeviceConnectedCallback()` - Handle connections
- `BTstack.bleStartAdvertising()` - Start advertising
- `BTstack.loop()` - Must be called in loop()

### TX (Central) - Based on LECentral example
- `BTstack.setup()` - Initialize BLE stack
- `BTstack.setBLEAdvertisementCallback()` - Handle discovered devices
- `BTstack.bleStartScanning()` / `bleStopScanning()` - Scan control
- `BTstack.bleConnect()` - Connect to device
- `BLEDevice.discoverGATTServices()` - Discover services
- `BLEDevice.discoverCharacteristicsForService()` - Discover characteristics
- `BLEDevice.writeCharacteristicWithoutResponse()` - Send data
- `BTstack.loop()` - Must be called in loop()

**Note**: The implementation follows the patterns from the official examples. For passkey pairing, you may need to configure security settings through BTstackLib or access lower-level Security Manager APIs if needed.

## Key Differences from Bluetooth Classic

1. **Lower Power**: BLE consumes significantly less power
2. **Simpler Pairing**: Passkey entry is more reliable than Classic Bluetooth PIN
3. **Better Range**: BLE typically has better range characteristics
4. **GATT Services**: Uses standard GATT service/characteristic model instead of SPP
5. **Numeric Passkey**: Uses 6-digit numeric passkey instead of 6-hex-digit PIN

## Security

- **Pairing Passkey**: Prevents unauthorized connections
- **Bonding**: Devices store bond keys for automatic reconnection
- **HMAC Authentication**: Each frame includes HMAC for message integrity (same as Classic)
- **Sequence Numbers**: Prevents replay attacks (same as Classic)

## Troubleshooting

1. **RX not found**: 
   - Ensure RX is powered on and BLE is initialized
   - Check that device name starts with "FPV_RX_"
   - Verify advertising is active (check serial output)

2. **Connection fails**: 
   - Check passkey is correct (6 numeric digits)
   - Ensure both devices support BLE (not just Classic)
   - Check serial output for error messages

3. **Pairing fails**:
   - Verify passkey matches on both devices
   - Check that Security Manager is initialized correctly
   - Ensure both devices are in pairing mode

4. **Menu not responding**: 
   - Check analog stick calibration
   - Verify button connections

5. **Auto-reconnect not working**:
   - Check that bonding was successful (check serial output)
   - Verify EEPROM is working correctly
   - Try manual pairing again

## Implementation Details

The code is based on the official Arduino-Pico BTstackLib examples:
- **RX**: Based on [LEPeripheral.ino](https://github.com/earlephilhower/arduino-pico/blob/master/libraries/BTstackLib/examples/LEPeripheral/LEPeripheral.ino)
- **TX**: Based on [LECentral.ino](https://github.com/earlephilhower/arduino-pico/blob/master/libraries/BTstackLib/examples/LECentral/LECentral.ino)
- **Scanner**: Reference from [LEDeviceScanner.ino](https://github.com/earlephilhower/arduino-pico/blob/master/libraries/BTstackLib/examples/LEDeviceScanner/LEDeviceScanner.ino)

### Key API Classes Used:
- `BTstack` - Main BLE stack object
- `UUID` - Service/Characteristic UUIDs
- `BLEDevice` - Connected device object
- `BLEService` - GATT service object
- `BLECharacteristic` - GATT characteristic object
- `BLEAdvertisement` - Advertisement data

### Passkey Pairing Note:
The examples don't explicitly show passkey pairing. The current implementation stores passkeys and may need additional Security Manager configuration. If pairing fails, you may need to:
1. Check if BTstackLib exposes pairing methods
2. Access lower-level Security Manager APIs if needed
3. Configure pairing requirements in `btstack_config.h` if available

## Future Enhancements

- Multiple bonded device support (store multiple pairings)
- Connection status indicators
- RSSI display on TX
- Unpair functionality via menu
- Factory reset button on RX

