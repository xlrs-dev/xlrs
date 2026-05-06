# RC build & flash

## PlatformIO environments (RC)

| Environment | Board | Role |
|-------------|-------|------|
| **`rc-crsf`** | `rpipico` (RP2040) | Classic RC: ADS1115 + OLED, CRSF handset or custom UART TX depending on detection and wiring. |
| **`rc-rp2350`** | `rpipico2` (RP2350) | **Dhanush**-style: internal ADC sticks, Core1 CRSF pipeline, power helper headers. **No** USB HID. |
| **`rc-HID-rp2350`** | `rpipico2` | Same as `rc-rp2350` plus **`RC_USB_HID_JOYSTICK`** (composite CDC + gamepad). |
| **`rc-rp2350-debug`** | `rpipico2` | Like `rc-rp2350` with extra link diagnostics on serial. |

Shared traits for RP2350 RC envs (see `platformio.ini`):

- `RC_SKIP_POWER_ON_SEQUENCE=1` by default (USB bench setups often need this; production with QON can remove it — see `include/rc_power_config.h`).
- `RC_TX_MODULE_UART_BAUD=400000` default for ELRS handset; increase only if module + Lua allow higher handset baud.

## Build & upload

```bash
pio run -e rc-HID-rp2350 -t upload
```

Replace the `-e` value for your board. On macOS, upload usually auto-finds `/dev/cu.usbmodem*`.

## USB strings (marketing name)

RC targets set:

```ini
board_build.arduino.earlephilhower.usb_product = Dhanush RC
```

Optional:

```ini
board_build.arduino.earlephilhower.usb_manufacturer = Your Brand
```

VID/PID remain board defaults (Raspberry Pi assignment) unless you override `usb_vid` / `usb_pid` similarly — only do that if you understand USB-IF rules and your own descriptor.

## Handset UART pins

- **GP8** — RC UART TX (toward module RX)
- **GP9** — RC UART RX (from module TX)

Ground common. **Baud** must match your ELRS Lua “handset” / packet rate pairing or your custom TX firmware.

## Related

- [RC overview](rc-overview.md) — feature summary  
- [Web UI](rc-webui.md) — config over USB  
- [USB HID](rc-usb-hid.md) — `rc-HID-rp2350` details  
