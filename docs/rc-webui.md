# RC Config Web UI (frontend)

The **RC Config** app lives under `tools/rc-webui`. It talks to the handset over **USB serial** using a binary protocol (frame sync `0xA5`, CRC8). It does **not** replace ELRS Lua; it configures stick behavior, trims, curves, and orchestrates binding phrase / pairing helpers.

## Requirements

- **Browser:** **Chrome** or **Edge** (Chromium) — **Web Serial** is required. Firefox/Safari are not supported for device access.
- **Connection:** RC powered and plugged in via USB. Select the port that shows your RC (e.g. product name **Dhanush RC** if that string is flashed).
- **HTTPS / localhost:** For development, run `npm run dev` and open the local URL. Deployed sites must be **secure context** (HTTPS) for Web Serial.

## Run locally

```bash
cd tools/rc-webui
npm install
npm run dev
```

Open the URL Vite prints (usually `http://localhost:5173`).

## Tabs at a glance

| Tab | Purpose |
|-----|---------|
| **Live** | Live ADC and channel preview over USB (uses streaming state frames). |
| **Calibrate** | Three-step stick calibration: **Start** → move sticks through full travel → **Sample** (repeat as needed) → **Finish & save** to device. |
| **Mapping** | Axis-to-channel assignment (which physical stick axis drives A/E/R/T), invert, deadzone, trims, cutoffs. Load from device before editing. |
| **Curves** | Per-axis rate / expo style adjustments (dual-rate, expo as exposed in protocol). |
| **Save / Apply** | Load config from device, edit draft, **Apply** (RAM) or **Save** (EEPROM). Import/export JSON. ELRS binding phrase tools, **USB-UART Proxy** entry. |
| **Auto Pair** | Guided flow: verify RC + TX (via RC) + RX (second USB serial connection), generate phrase, push to RX and through RC to TX, enter pairing modes. |

## Typical workflows

### First-time calibration

1. Connect RC USB; select the device in the header.
2. Open **Calibrate** → **Start**.
3. Move both sticks smoothly through full range several times.
4. Click **Sample** a few times while covering extremes and center.
5. **Finish & save** — min/max/center are stored for mapping.

### Adjust channel mapping and save

1. **Mapping** → **Load** (or load from **Save / Apply**) to pull current EEPROM config.
2. Change axis order, invert, deadzone, or trims.
3. **Save / Apply** → **Apply** for immediate test, then **Save** to persist across reboot.

### ELRS TX: read binding phrase or align RX + TX

Use **Save / Apply** for phrase read/write when the firmware exposes it, or **Auto Pair** for the full multi-step flow with a separate USB cable on the **RX** board.

If the RC detects the TX as ELRS in **RX role** (wrong UART wiring or module mode), the Auto Pair panel will warn — fix module orientation / CRSF port roles first.

### USB-UART proxy (ELRS Buddy, etc.)

Used to pass **raw bytes** between the PC’s USB CDC and the **ELRS module UART** (GP8/GP9).

1. Open **Save / Apply**.
2. Click **Enter USB-UART Proxy Mode** and confirm.
3. The UI switches to an embedded [ELRS Buddy](https://fourflies.mooo.com/elrsbuddy/) frame (or use your own tool).
4. In Buddy, connect to the **same USB serial port** — you are now talking directly to the module.
5. **Power-cycle or reset the RC** to leave proxy mode and restore normal Web UI access.

**Note:** While in proxy mode, the RC is **not** processing the `0xA5` config protocol and **HID updates pause** on HID-capable builds.

### Live tuning

Open **Live** with streaming enabled to verify sticks and toggles before flight. On some builds, very aggressive Core1 timing may affect streaming — if values look stale, reconnect or check firmware notes.

## Protocol reference (high level)

Defined in `tools/rc-webui/src/lib/protocol.ts` and `lib/RCConfigProtocol/` on firmware:

- `GET_DEVICE_INFO`, `GET_CONFIG`, `SET_CONFIG_DRAFT`, `APPLY_CONFIG`, `SAVE_CONFIG`
- `START_CALIBRATION` / `CALIBRATION_SAMPLE` / `FINISH_CALIBRATION`
- `STREAM_STATE_START` / `STOP`, `STATE_FRAME`
- Binding / link: `SET_BINDING_PHRASE_RX`, `SET_BINDING_PHRASE_TX`, `GET_LINK_STATUS`, `ENTER_PAIRING_MODE`, `RE_DETECT_TX`, `GET_ELRS_BINDING_PHRASE`
- `ENTER_USB_UART_PROXY`

Payload layouts and CRC must match the firmware; when in doubt, use the same git revision for Web UI and RC binaries.

## Troubleshooting

| Issue | Things to try |
|-------|----------------|
| No serial port | Cable data-capable? Driver installed? Try another USB port. |
| Wrong device selected | Unplug other serial gadgets; product string **Dhanush RC** helps identification. |
| Apply works but reboot loses settings | Use **Save** (EEPROM), not only **Apply**. |
| ELRS phrase greyed / fails | TX must be reachable on handset UART; module in TX mode; baud matches Lua/handset expectations. |
| Auto Pair RX step fails | Second serial connection must be the **RX** firmware exposing the phrase protocol, not the RC. |
