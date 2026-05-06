# RC handset overview

The **RC** board reads sticks and switches, shows status on an OLED, and talks to a **transmitter** over UART. There are two different transmitter setups; the firmware behavior and Web UI differ slightly depending on which you use.

## Capabilities

| Feature | Description |
|--------|-------------|
| **Sticks** | Four analog axes (aileron, elevator, rudder, throttle). Mapping to CRSF channels A/E/R/T is configurable. RP2040 builds may use an external ADS1115; RP2350 “Dhanush” typically uses internal 12-bit ADC. |
| **Switches** | Four 3-position–style toggles exposed as CRSF channels (typically 1000 / 1500 / 2000 µs). |
| **OLED** | 128×64 SH1106 over I2C: link, battery, menus (binding / ELRS actions on CRSF handset builds). |
| **USB** | **CDC serial** for the [RC Config Web UI](rc-webui.md) (binary `0xA5` protocol). Optional **USB HID gamepad** when built with `RC_USB_HID_JOYSTICK` — see [USB HID](rc-usb-hid.md). |
| **Handset UART** | Fixed pins **GP8 (TX from RC)** and **GP9 (RX into RC)** toward the transmitter module. Default baud is **400000** (ELRS handset); custom TX path historically used **420000** — match `platformio.ini` to your wiring and TX firmware. |
| **Charger / power (Dhanush)** | BQ2562x reporting and power sequencing; see `include/rc_power_config.h` and [build notes](rc-build-flash.md). |

## Two transmitter paths

### A) Custom Pico TX (XLRS-style UART)

- The **TX** is another Pico running this repo’s `tx_sx128x` firmware.
- It speaks the project’s **high-speed UART frame protocol** (420000 baud by default in older docs; check `tx_sx128x` / RC `platformio.ini` for the value you flashed).
- The RC can send channel payloads and receive telemetry over that protocol.

### B) ELRS TX module (CRSF handset)

- The **TX** is an **ExpressLRS (or compatible) transmitter module** connected to the RC UART.
- The RC acts as a **CRSF handset**: it sends RC channel packets in the format the module expects (see `SimpleTxCrsf` / `CRSFSerialConnector` in firmware).
- Default handset baud is often **400000**; some Lua / packet-rate settings expect **921600** — align `RC_TX_MODULE_UART_BAUD` in `platformio.ini` with your module docs.
- The on-device **Binding** menu and Web UI features can **read/write the ELRS binding phrase**, enter **Wi‑Fi / bind / pairing** modes via CRSF, and use **USB-UART proxy** so PC tools talk through the RC straight to the module.

You do not need both paths at once — flash RC and TX for the stack you actually use.

## Device name on USB

When you connect the RC over USB, the composite device may report a product string such as **Dhanush RC** (configured in `platformio.ini` for RC environments). The vendor ID is still Raspberry Pi’s unless you change it.

## Where to go next

- [RC Web UI](rc-webui.md) — calibration, mapping, curves, apply/save, auto-pair, ELRS Buddy proxy  
- [USB HID](rc-usb-hid.md) — joystick / gamepad to PC or tablet  
- [Build & flash](rc-build-flash.md) — which `pio` environment to use  
