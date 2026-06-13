# XLRS RC Web UI

Browser Web Serial UI for the `rc.v1` USB line protocol. It runs as a local
Vite app and does not require a backend service.

## Requirements

- Chrome or Edge with Web Serial support.
- A local secure origin. Vite's `http://127.0.0.1` dev server works.
- RC handset USB for calibration/config/TX binding.
- Direct RX USB for RX binding.

## Usage

```bash
cd tools/rc-webui
npm install
npm run dev
```

Open the printed local URL, connect the relevant USB device, and use the screens:

- **Calibration Wizard** sends `cal_start`, repeated `cal_sample`, and
  `cal_finish`; it also has min/center/max sliders for direct EEPROM-backed
  tuning.
- **Channels** edits axis mapping, invert, deadzone, channel trim, and cutoffs,
  saving each change to EEPROM and runtime immediately; it includes live stick
  pads and channel bars for the two physical joysticks.
- **Filters** edits oversampling, low-pass aggressiveness, and high-pass
  aggressiveness with immediate EEPROM-backed runtime updates.
- **Binding Wizard** writes TX through RC USB (`target=tx`), writes RX through
  direct RX USB (`target=rx`), and compares `uid_check` values with warnings for
  partial updates, missing checks, mismatches, and required reboot.

Current firmware support: discovery, live state streaming, immediate RC config
editing/defaults, calibration, TX binding through RC USB, and TX/RX direct-USB
binding are wired.

## Scripts

```bash
npm run dev      # start Vite on 127.0.0.1
npm run build    # type-check and produce dist/
npm run preview  # serve the production build locally
```

The UI logs raw serial lines for both RC and RX sessions so firmware-side
protocol issues can be diagnosed without a separate terminal.

## rc.v1 Integration Notes

When a port opens, the transport sends an unsolicited discovery line:

```text
rc.v1 hello client=rc-webui
```

The **Discover** button sends a sequenced request:

```text
rc.v1 hello seq=<n>
```

The parser accepts either `rc.v1 <kind> key=value` or `<kind> key=value` lines.
Recognized inbound kinds are `ok`, `err`, `state`, `config`, and `binding`.
Field values are percent-decoded with `decodeURIComponent`; outbound request
fields are percent-encoded with `encodeURIComponent`.

Example inbound lines:

```text
rc.v1 ok seq=1 role=rc_handset fw=0.1 caps=state,config,binding
rc.v1 state adc=2048,2049,2001,2050 adc_filtered=2048,2049,2001,2050 ch=992,992,172,992 lq=98 rssi=-47
rc.v1 config cal_center=2048,2048,2048,2048 filter=8,25,0,0
rc.v1 binding target=tx uid=0011223344556677 uid_check=89ab persisted=1
rc.v1 err seq=7 code=busy message=bind%20already%20active
```

After a physical disconnect or read failure, the UI keeps the selected
`SerialPort` object and attempts to reopen it with a small backoff. Browser
permission rules still apply; if permission is lost or the USB path changes,
click **Connect** again and select the new port.
