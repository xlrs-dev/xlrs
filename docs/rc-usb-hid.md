# USB HID gamepad (optional)

Some RC builds compile in **USB HID** alongside the normal **CDC serial** port. The host then sees **one USB device, two interfaces**:

1. **CDC ACM** — same serial port the [RC Web UI](rc-webui.md) uses (`Serial` / Web Serial).
2. **HID** — a **16-bit gamepad** (GENERIC_GAMEPAD-style report) for games, simulators, or hosts that expose USB gadget / OTG device mode.

## When it is enabled

- **`rc-HID-rp2350`** PlatformIO environment defines `RC_USB_HID_JOYSTICK=1`.
- **`rc-rp2350`** / **`rc-rp2350-debug`** do **not** include HID by default (smaller surface; use `rc-HID-rp2350` if you need the joystick).

You can also add `-DRC_USB_HID_JOYSTICK=1` to another env if you accept the extra USB descriptor and Joystick library.

## Axis mapping (CRSF µs → HID)

After calibration and mapping, logical channels are in the usual **1000–2000 µs** range. HID uses signed 16-bit axes:

| CRSF function | HID usage |
|---------------|-----------|
| Aileron (ch 0) | **X** |
| Elevator (ch 1) | **Y** |
| Rudder (ch 2) | **Z** |
| Throttle (ch 3) | **Rz** |
| Triggers **Rx** / **Ry** | Held at **0** |
| Hat | **Neutral** |

## Buttons

Aux / toggle channels **4–7** map to HID buttons **1–4**: **pressed** when channel value **≥ 1700 µs** (mid ~1500 releases).

## Report rate

- Firmware throttles reports with `RC_USB_HID_MIN_INTERVAL_US` (default **4000 µs** ≈ 250 Hz). Override with a compile-time define if needed.
- Endpoint interval uses `usb_hid_poll_interval` (milliseconds) set in firmware (e.g. **2** ms).

## USB-UART proxy interaction

In **proxy mode**, the main loop only forwards USB ↔ UART bytes; **HID reports are not updated** until you reset back to normal mode.

## Host notes (PC, Meta Quest, etc.)

- **Windows / macOS / Linux:** Should show a standard gamepad. You may need to calibrate or invert axes in the game.
- **Android / VR:** Depends on OEM USB host support. Quest and similar devices vary; HID is standard but not guaranteed for every mode or cable.

## USB product string

RC environments set the USB string **Dhanush RC** via `board_build.arduino.earlephilhower.usb_product` in `platformio.ini`. ELRS Buddy and OS device lists use this name for the composite device.

See [Build & flash](rc-build-flash.md) for flashing `rc-HID-rp2350`.
