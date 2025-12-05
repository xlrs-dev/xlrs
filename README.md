# RP2040 PPM TX/RX for FPV Drone

A WiFi-based remote control system for FPV drones using Raspberry Pi Pico W (RP2040) boards. The transmitter (TX) side reads analog sticks and buttons, and the receiver (RX) side outputs standard PPM signals compatible with Betaflight flight controllers.

## Hardware Requirements

### TX Side (Remote Control)
- Raspberry Pi Pico W
- SSD1306 OLED display (128x64, I2C)
- 2x 2-axis analog joysticks (4 analog inputs)
- 4x buttons/switches
- Wiring:
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
- Raspberry Pi Pico W
- PPM output pin: GP0 (connect to flight controller)

## Software Setup

1. Install PlatformIO
2. Clone this repository
3. Build and upload:
   - For TX: `pio run -e tx -t upload`
   - For RX: `pio run -e rx -t upload`

## Default Configuration

- **WiFi SSID**: `FPV_RX_AP`
- **WiFi Password**: `fpv12345678`
- **Server Port**: 8888
- **PPM Frame Rate**: 50Hz (20ms frames)
- **Channel Range**: 1000-2000 microseconds (standard RC)

## Channel Mapping

1. Stick 1 X-axis
2. Stick 1 Y-axis
3. Stick 2 X-axis
4. Stick 2 Y-axis
5. Button 1 (1000/2000)
6. Button 2 (1000/2000)
7. Button 3 (1000/2000)
8. Button 4 (1000/2000)

## WiFi Configuration

WiFi credentials are stored in EEPROM and can be changed programmatically. Default values are set on first boot if no configuration exists.

## Protocol

The system uses a binary protocol with fixed-length frames:
- 16 bytes per frame (8 channels × 2 bytes)
- Big-endian encoding
- 50Hz update rate (20ms intervals)

## Usage

1. Power on the RX side first - it will create a WiFi access point
2. Power on the TX side - it will automatically connect to the RX AP
3. The TX OLED will display connection status, signal strength, and channel values
4. The RX outputs PPM signals on GP0 that can be connected to your flight controller

## Troubleshooting

- **TX won't connect**: Check that RX AP is running and SSID/password match
- **No PPM output**: Verify RX is connected to TX and receiving data (check Serial monitor)
- **Channels not responding**: Check analog stick wiring and button connections
- **OLED not working**: Verify I2C wiring (SDA/SCL) and address (default 0x3C)

## Technical Details

### PPM Generation
The PPM signal is generated using RP2040 hardware timers and interrupts for precise timing. The implementation uses:
- Hardware alarm pool for microsecond-precise timing
- State machine-based PPM frame generation
- Non-blocking interrupt-driven operation
- Automatic frame timing calculation

### WiFi Protocol
- Binary protocol with 16-byte fixed frames
- 50Hz update rate (20ms intervals)
- TCP connection on port 8888
- Automatic reconnection on disconnect

## Future Enhancements

- Configurable WiFi credentials via serial/USB
- Telemetry support (battery voltage, GPS, etc.)
- Multiple TX support with channel assignment
- PIO-based PPM generation for even better precision (optional)

