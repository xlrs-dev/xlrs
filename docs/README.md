# Dhanush / RC documentation

User-facing documentation for the **RC** (handset) firmware, USB behavior, and the **RC Config** browser tool.

| Document | What it covers |
|----------|----------------|
| [RC overview](rc-overview.md) | What the RC does, two radio paths (custom XLRS-style TX vs ELRS module), inputs, OLED, power |
| [RC Web UI](rc-webui.md) | Running the config app, Web Serial, tabs (Live, Calibrate, Mapping, Curves, Save/Apply, Auto Pair), USB-UART proxy |
| [USB HID gamepad](rc-usb-hid.md) | Optional composite HID (PC / Quest), axis mapping, build env `rc-HID-rp2350`, USB product name |
| [Build & flash](rc-build-flash.md) | PlatformIO environments (`rc-crsf`, `rc-rp2350`, `rc-HID-rp2350`), UART baud notes |

For **TX / RX** radio link, SX1280 wiring, pairing phrases, and CRSF to the flight controller, see the main [README](../README.md) and its architecture sections.
