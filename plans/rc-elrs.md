# RC ↔ ELRS (ExpressLRS) TX compatibility

Plan for making the RC (rc-rp2350 / rc-crsf) work with an **ExpressLRS transmitter module** over UART: output CRSF channels, optional telemetry, **pairing/binding**, and **binding phrase** on the RC display with a **usable menu**.

---

## 1. Current vs required

### 1.1 Channel output


| Item        | Current                                     | Required for ELRS                                                |
| ----------- | ------------------------------------------- | ---------------------------------------------------------------- |
| Protocol    | Custom UART (sync `0xA5`, custom msg types) | **CRSF** (standard frames)                                       |
| Frame       | `UART_MSG_CHANNELS` (0x01), 8× uint16       | **CRSF_RC_CHANNELS_PACKED** (0x16), 16× 11-bit packed (22 bytes) |
| Destination | N/A (custom)                                | **0xEE** (CRSF_ADDRESS_CRSF_TRANSMITTER)                         |
| Baud        | 420000                                      | 420000 (same)                                                    |
| Rate        | 250 Hz (4 ms)                               | ~250 Hz (4 ms)                                                   |
| Pins        | GP8 (TX), GP9 (RX)                          | Same (RC TX → ELRS RX, RC RX → ELRS TX)                          |


**Gap:** RC must send **CRSF** channel frames to address **0xEE** at ~250 Hz instead of (or selectable with) the custom UART protocol. Use existing `lib/CRSF/CRSF.cpp` or `crsfSerial.queuePacket()` with `crsf_protocol.h` types.

### 1.2 Telemetry input (optional)

- **Current:** RC expects custom UART messages (pong, telemetry, status) for “TX connected” and display (link, RX battery).
- **With ELRS:** Module sends **CRSF** (link statistics 0x14, battery 0x08, etc.). To keep “connected” and display when using ELRS, add a **CRSF RX parser** on the same UART (e.g. reuse `CrsfSerial` in RX direction) and drive display from CRSF link/battery frames.

### 1.3 TX type selection (auto-detect at boot)

- Support both **custom TX** (this project) and **ELRS TX** in one firmware with **automatic detection at boot** (no user selection required). The RC identifies which board is connected, shows it on the display, and uses the correct protocol for the rest of the session.
- Optional: allow manual override in menu (“TX: Auto / Custom / ELRS”) for edge cases or testing.

---

## 1.4 Automatic TX type detection at boot

### 1.4.1 Goal

- At **boot**, the RC determines whether the UART is connected to the **custom TX board** or an **ELRS TX module**.
- Show the result on the **display** (e.g. “TX: Custom” or “TX: ELRS” and optionally device name).
- Use the **appropriate protocol** for the rest of the session: custom UARTProtocol (channels, ping/pong, telemetry) vs CRSF (channels to 0xEE, CRSF telemetry/params).

### 1.4.2 Why detection is possible

- **Different protocols, no overlap:**
  - **Custom TX** uses a **sync byte 0xA5**; frames are `0xA5` + length + type + payload + CRC. Only our custom TX responds to `UART_MSG_PING` (0x14) with `UART_MSG_PONG` (0x15).
  - **CRSF** (ELRS) uses **address byte first** (e.g. 0xC8, 0xEE); frames are `[addr][len][type][payload][crc]`. ELRS does not use 0xA5 as sync. ELRS TX responds to **CRSF DEVICE_PING (0x28)** with **CRSF DEVICE_INFO (0x29)**.
- **No cross-response:** Sending custom PING (0xA5…) does not trigger an ELRS response (ELRS sees invalid CRSF or wrong address). Sending CRSF DEVICE_PING does not trigger a custom PONG (custom TX only looks for 0xA5 sync).
- **ELRS does not send unsolicited** at boot: the TX module does not output data until it receives a CRSF stream, so we can run detection without being flooded by ELRS traffic.

### 1.4.3 Detection sequence (recommended order)

1. **Phase 1 – Try custom TX**
  - Send **custom PING** (existing: sync 0xA5 + type 0x14, no payload, CRC).
  - Listen for **custom PONG** (0xA5 + type 0x15) with existing UARTProtocol RX parser.
  - **Timeout:** e.g. 300–500 ms (allow 1–2 round-trips).
  - **If PONG received** → **Detected: Custom TX.** Set `tx_type = Custom`, show “TX: Custom” (and optionally “Connected”) on display. Use UARTProtocol for channels, telemetry, ping/pong for the rest of the session. **Stop detection.**
2. **Phase 2 – Try ELRS (only if Phase 1 failed)**
  - Flush RX buffer (discard any leftover bytes from custom PING).
  - Send **CRSF DEVICE_PING (0x28)** to destination **0xEE** (CRSF transmitter). Use standard or extended frame format per [CRSF_FRAMETYPE_DEVICE_PING](https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_DEVICE_PING) (e.g. extended: dest 0x00 broadcast, source 0xEA handset).
  - Listen for **CRSF DEVICE_INFO (0x29)**. Parse incoming bytes with a CRSF frame parser (length, type, payload, CRC). DEVICE_INFO payload (see [CRSF_FRAMETYPE_DEVICE_INFO](https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_DEVICE_INFO)) includes:
    - Parameter protocol version, parameter count, software/hardware version, **serial number (4 bytes)**, **null-terminated display name**.
  - **ELRS identification:** ExpressLRS **always sends serial number** as the ASCII string **"ELRS"** (0x45 0x4C 0x52 0x53). Display name is a short string (e.g. “ExpressLRS”, “SIYI FM30”, model name). Use one or both:
    - **Reliable:** Check serial number field == `"ELRS"` (4 bytes) → **Detected: ELRS.**
    - **Optional extra:** Check display name contains `"ELRS"` or `"ExpressLRS"` for robustness.
  - **ELRS role (TX vs RX):** The **Extended Source** field in the DEVICE_INFO response identifies the device type ([CRSF Addresses](https://github.com/crsf-wg/crsf/wiki/CRSF-Addresses)): **0xEE** = CRSF transmitter (correct for our RC), **0xEC** = CRSF receiver (wrong — user plugged an ELRS RX into the RC). When serial is "ELRS", check Extended Source:
    - **0xEE** → **Detected: ELRS TX (correct).** Set `tx_type = ELRS`, `elrs_role = TX`, show “TX: ELRS” and device name. Use CRSF for channels and telemetry. **Stop detection.**
    - **0xEC** → **Detected: ELRS RX (wrong mode).** Set `tx_type = ELRS`, `elrs_role = RX`, show **“Wrong mode: ELRS RX”** (or “Need TX, not RX”) on display and do **not** use this device for channel output (see §1.4.7). **Stop detection.**
  - **Timeout:** e.g. 300–500 ms.
  - **If valid DEVICE_INFO with ELRS identity but unknown Extended Source** → Treat as ELRS; if source is not 0xEE, treat as wrong role and show warning.
3. **If neither responds**
  - Set `tx_type = Unknown` (or “No TX”). Show “TX: —” or “No TX” / “Detecting…” on display.
  - **Retry policy:** Either retry detection periodically (e.g. every 5 s) until a TX is detected, or retry on next boot / user action (e.g. “Detect TX” in menu). Avoid blocking the main loop indefinitely.

### 1.4.4 Display behaviour

- **During detection:** Show “Detecting TX…” (or similar) so the user knows the RC is probing.
- **After detection:** Show clearly:
  - **Custom:** e.g. “TX: Custom” and normal status (link, batt, AERT).
  - **ELRS TX (correct):** e.g. “TX: ELRS” and optionally device name from DEVICE_INFO; then link/batt from CRSF telemetry when available.
  - **ELRS RX (wrong mode):** e.g. **“Wrong mode: ELRS RX”** or **“Need TX, not RX”** (see §1.4.7); do not use for channel output.
  - **Unknown:** “TX: —” or “No TX” and optionally “Replug?” or “Detect again in menu”.

### 1.4.5 Edge cases and notes

- **Unplugged cable:** No PONG, no DEVICE_INFO → Unknown. Retry or show “No TX”.
- **Baud rate:** Both custom TX and ELRS use **420000**; no baud switching needed for detection.
- **Custom TX RX parser:** When we send CRSF (Phase 2), the custom TX might see 0xEE and then other bytes; its parser expects 0xA5 sync so it stays in sync state and eventually times out. No need to change custom TX firmware for detection.
- **Extended frames:** DEVICE_PING and DEVICE_INFO may use CRSF “extended” header (extra dest/source bytes). Implementation should follow crsf-wg wiki so ELRS responds correctly; ELRS uses extended source **0xEE** (tx module) and expects handset as **0xEA**.
- **Single detection at boot:** Detection runs once after init; if the user swaps the module with the RC powered, they can use a menu item “Re-detect TX” to run the sequence again (optional).

### 1.4.6 References (detection)

- [CRSF_FRAMETYPE_DEVICE_PING](https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_DEVICE_PING) – Host sends 0x28 (extended frame) to discover devices.
- [CRSF_FRAMETYPE_DEVICE_INFO](https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_DEVICE_INFO) – Device replies with 0x29; **serial number "ELRS"** and display name identify ExpressLRS.
- [CRSF Addresses](https://github.com/crsf-wg/crsf/wiki/CRSF-Addresses) – **0xEE** = transmitter module, **0xEC** = receiver; use Extended Source in DEVICE_INFO to distinguish.

### 1.4.7 ELRS module in wrong role (RX instead of TX)

**Can we change TX/RX mode over UART?** **No.** ExpressLRS TX and RX are **different firmware builds** (different targets in the Configurator). There is no CRSF command or parameter to “switch” a running device from RX to TX; the device is compiled and flashed as one or the other. Mode cannot be changed on the fly over UART.

**What we can do:** Detect the role using the **Extended Source** in DEVICE_INFO (0xEE = TX, 0xEC = RX) and **inform the user clearly** so they can reflash or swap the module.

- **RC display (when ELRS RX detected):**
  - Show a clear, persistent message: e.g. **“Wrong mode: ELRS RX”** or **“Need TX module, this is RX”** or **“Module is RX — flash as TX”**.
  - Do **not** use the device for channel output (do not send CRSF channels to it as if it were a TX; it won’t work and could be confusing). Optionally show “Plug in ELRS **TX** module” or “Reflash this module as TX”.
- **Web tool:**
  - Extend link status (or TX info) to include **elrsRole**: `'tx' | 'rx' | null`. When `txType === 2` (ELRS) and `elrsRole === 'rx'`:
    - Show a **warning** in AutoPairPanel and status: e.g. “TX: ELRS **RX (wrong mode)** — need transmitter module” or “Connected device is an ELRS receiver; use an ELRS transmitter module.”
    - Optionally disable or grey out “Set TX phrase” / “Enter pairing” with tooltip “Requires ELRS TX module”.
  - Display the same message in any status area that shows TX type, so the user sees it as soon as they connect the RC to the WebUI.
- **Summary:** We **cannot** change mode over UART. We **can** detect wrong role (RX) and tell the user on both the **RC display** and the **Web tool**; do not treat an ELRS RX as a valid TX for channel output.

---

## 2. Pairing / bonding and binding phrase

### 2.1 Goals

- **Initiate pairing/binding** from the RC (so the user can trigger ELRS bind from the controller).
- **Show binding phrase on RC display** (current phrase, and optionally set a new one).
- **Usable menu** for: TX mode (Custom vs ELRS), Binding phrase (view/edit), Pair/Bind action.

### 2.2 How ELRS binding works (reference)

- **Binding phrase:** Both TX and RX must use the **same phrase** (typically 8+ alphanumeric). It can be set at flash time (Configurator) or at runtime via **WebUI** (WiFi) on capable hardware. Over the **CRSF** link, the Lua script and radios use the **CRSF parameter protocol** to read/write parameters, including the binding phrase.
- **Pairing/bind:** With a phrase set, TX and RX with the same phrase auto-bind. “Traditional” bind (no phrase) is RX in bind mode, then power TX. So “initiate pairing” from the RC could mean: (1) send a CRSF command that puts the **ELRS TX module** into a bind/listen state, and/or (2) set the binding phrase on the TX module via CRSF so it matches the RX.

### 2.3 CRSF protocol pieces to implement (lookup)

Implementing pairing/binding and binding phrase on the RC means having the RC talk **CRSF parameter and command** protocol to the ELRS TX module:


| Frame type                   | Hex  | Purpose                                                           |
| ---------------------------- | ---- | ----------------------------------------------------------------- |
| **DEVICE_PING**              | 0x28 | Request device info from module (optional; confirm ELRS present). |
| **DEVICE_INFO**              | 0x29 | Response: name, version, serial (PING response).                  |
| **PARAMETER_SETTINGS_ENTRY** | 0x2B | Chunk of parameter metadata (param ID, name, type, value).        |
| **PARAMETER_READ**           | 0x2C | Request parameter value by ID (chunk index).                      |
| **PARAMETER_WRITE**          | 0x2D | Write parameter (e.g. binding phrase string).                     |
| **COMMAND**                  | 0x32 | Execute command (e.g. reboot, “start bind” if ELRS defines one).  |


**Lookup needed:**

- **ELRS parameter field indexes:** Not fixed; we must **PARAMETER_READ** (0x2C) and parse **PARAMETER_SETTINGS_ENTRY** (0x2B) to get the list. Find the entry whose label is the binding phrase parameter (e.g. “Binding Phrase”) and the “Bind” **COMMAND** entry. Use those field indexes for PARAMETER_WRITE.
- **ELRS CRSF COMMAND (0x32):** Subcommand bytes for “reboot”, “enter bind mode”, “WiFi” if any. Check ELRS source for `CRSF_FRAMETYPE_COMMAND` and command IDs.
- **Destination address:** Parameter and command frames from RC to ELRS TX are sent to **0xEE** (CRSF transmitter). Extended header (dest/origin) may use **0xEA** (handset) / **0xEE** (tx module) in ELRS Lua; see crsf-wg wiki for extended frame layout.

#### CRSF parameter format (from crsf-wg wiki)

- **PARAMETER_WRITE (0x2D):** Payload = `[field index] [value]`. Value length and encoding depend on the parameter type (same as in PARAMETER_SETTINGS_ENTRY). **Field indexes are not fixed** — obtain them by PARAMETER_READ and parsing 0x2B responses.
- **PARAMETER_SETTINGS_ENTRY (0x2B):** Response to 0x2C. Payload = `[field index] [chunks remaining] [parent] [type/hidden] [label] [value]`. Strings are null-terminated.
  - **type/hidden:** High bit = hidden. Low 7 bits: 0x00 UINT8, 0x01 INT8, 0x02 UINT16, 0x03 INT16, 0x08 FLOAT, 0x09 SELECT, **0x0A STRING**, 0x0B FOLDER, 0x0C INFO, **0x0D COMMAND**.
  - **STRING (0x0A):** value = `[max length optional] [current value null-terminated]` — use for binding phrase.
  - **COMMAND (0x0D):** value = `uint8_t step`. Steps: 0 IDLE, 1 CLICK (user clicked), 2 EXECUTING, 3 ASKCONFIRM, 4 CONFIRMED, 5 CANCEL, 6 QUERY. To “start bind” from RC: PARAMETER_WRITE that field index with value **0x01** (CLICK). ELRS then responds with 0x2B updates (e.g. step 2 “Binding…”).
- Example (from wiki): Bind command at field index **0x11**; write value **0x01** to trigger. Binding phrase will be a different field index (STRING type); discover via 0x2C → 0x2B.
- Refs: [CRSF_FRAMETYPE_PARAMETER_WRITE](https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_PARAMETER_WRITE), [CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY](https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY).

### 2.4 Implementation outline

- **CRSF TX path (RC → ELRS):**
  - Send **RC_CHANNELS_PACKED** (0x16) to 0xEE at 250 Hz (already planned above).
  - Send **PARAMETER_READ** (0x2C) to 0xEE to request parameter list/entry that contains binding phrase.
  - Send **PARAMETER_WRITE** (0x2D) to 0xEE to set binding phrase (string).
  - Optionally send **COMMAND** (0x32) to 0xEE for reboot/bind if ELRS supports it.
- **CRSF RX path (ELRS → RC):**
  - Parse **PARAMETER_SETTINGS_ENTRY** (0x2B) to get param list and find binding phrase param/chunk.
  - Parse **DEVICE_INFO** (0x29) to show module name/version and confirm link.
  - Use received parameter values to show current binding phrase on the RC display.
- **“Initiate pairing” from menu:** Either (1) set same phrase on TX (via PARAMETER_WRITE) and optionally send COMMAND to trigger bind, or (2) just set phrase and instruct user to power-cycle RX; exact flow depends on ELRS COMMAND support (to be confirmed in source).

---

## 3. RC display and menu

### 3.1 Current hardware

- **Display:** SH1106 (I2C), 128×64, used for status (link, TX/RX batt, AERT values).
- **Buttons:** ENTER (GP13), BACK (GP14), POWER (GP19). ENTER currently sends PAIR, BACK sends STATUS_REQ over custom UART.

### 3.2 Menu requirements

- **Usable menu** for:
  - **TX mode:** Show **auto-detected** type (Custom / ELRS) on main screen; optional manual override in menu (Auto / Custom / ELRS) for testing or if detection fails.
  - **Binding phrase:** View current phrase (from ELRS via CRSF params), optionally edit and write back (PARAMETER_WRITE). Only relevant when TX is ELRS.
  - **Pairing/Bind:** Action to “Initiate bind” / “Set phrase and bind” (calls CRSF parameter + command as above). Only relevant when TX is ELRS.
- Navigation: ENTER = select/confirm, BACK = back/cancel. Optional: use one of the toggles or a long-press for “menu” if we want to keep short ENTER/BACK for quick actions in the main screen.

### 3.3 Menu structure (draft)

- **Main screen** (today’s status): **TX: Custom / ELRS / —** (from auto-detect), Link, TX/RX batt, AERT. Optional: “Menu” hint (e.g. “BACK long-press = Menu”).
- **Top-level menu** (list):
  - TX Mode: [Auto | Custom | ELRS] — Auto = use boot detection result; Custom/ELRS = force protocol (e.g. for re-detect or override).
  - Re-detect TX (optional) — Run detection sequence again without reboot.
  - Binding / ELRS (shown when TX is ELRS or override ELRS)
  - Calibration (existing WebUI flow can stay; menu can point to “use WebUI” or future on-device cal)
  - Back → main screen
- **Binding / ELRS submenu:**
  - View binding phrase (show string from ELRS; “—?” if not ELRS or not yet read).
  - Set binding phrase (sub-screen: input or “Generate” then send PARAMETER_WRITE to ELRS).
  - Initiate pairing (trigger COMMAND + phrase flow; show “Pairing…” on display).
  - Back → top-level menu

### 3.4 Display constraints

- 128×64, small font: keep lines short, 2–4 items per menu screen; scroll or multi-page if needed.
- “View binding phrase” can show first N characters + “…” if long; “Set” can be character grid or generate-only initially to avoid full keyboard on device.

---

## 4. Implementation checklist

- **Automatic TX type detection at boot**  
  - After UART init, run detection sequence before entering main loop (or in first N ms of loop).  
  - Phase 1: Send custom PING; wait for PONG (use existing UARTProtocol RX) with timeout ~300–500 ms.  
  - Phase 2 (if no PONG): Send CRSF DEVICE_PING (0x28) to 0xEE (extended frame per wiki if required).  
  - Parse incoming bytes for CRSF frame; on DEVICE_INFO (0x29) check serial number == "ELRS" (or display name contains ELRS).  
  - **ELRS role:** Read **Extended Source** from DEVICE_INFO: 0xEE = TX (correct), 0xEC = RX (wrong mode). Set `elrs_role` (TX/RX); when RX, treat as “wrong mode” (§1.4.7), show “Wrong mode: ELRS RX” and do not use for channel output.  
  - Set `tx_type` (Custom / ELRS / Unknown); show “TX: Custom”, “TX: ELRS”, or “TX: —” (or “Wrong mode: ELRS RX”) on display.  
  - Optional: “Re-detect TX” menu item to run sequence again.  
  - Optional: Store “TX mode override” (Auto / Custom / ELRS) in EEPROM; when Auto, use detected type; when Custom/ELRS, force that protocol.
- **CRSF channel output**  
  - When ELRS mode (detected or override): send CRSF RC_CHANNELS_PACKED (0x16) to 0xEE at 250 Hz on Serial2.  
  - Reuse `lib/CRSF` or `crsfSerial.queuePacket()` + `crsf_protocol.h`; map RC `channels[0..7]` to 11-bit + ch8–15 center.
- **TX mode and protocol switching**  
  - When Custom (detected or override): use UARTProtocol for channels, ping/pong, telemetry; “TX connected” from pong.  
  - When ELRS **TX** (correct role): use CRSF for channels; disable custom ping/pong; “TX connected” from CRSF traffic or DEVICE_INFO; optional CRSF telemetry for display.  
  - When ELRS **RX** (wrong mode): do **not** use for channel output; show wrong-mode message on display and in WebUI (§1.4.7).
- **CRSF telemetry input (optional)**  
  - Parse CRSF on Serial2 RX (link 0x14, battery 0x08).  
  - Drive “link up” and display (RSSI, LQ, RX battery) from CRSF when in ELRS mode.
- **CRSF parameter protocol (for binding phrase)**  
  - Implement sending PARAMETER_READ (0x2C) to 0xEE.  
  - Implement parsing PARAMETER_SETTINGS_ENTRY (0x2B) and storing/displaying binding phrase param.  
  - Implement PARAMETER_WRITE (0x2D) to write binding phrase string to ELRS TX.  
  - Look up ELRS param index/chunk for binding phrase in ELRS source and document here.
- **CRSF COMMAND (0x32)**  
  - Look up ELRS command bytes for “reboot”, “bind” (if any).  
  - Add “Initiate pairing” menu action that sends appropriate COMMAND + phrase flow.
- **RC menu**  
  - Menu state machine: main screen ↔ top-level menu ↔ submenus (TX mode, Binding/ELRS).  
  - Main screen shows detected TX type (“TX: Custom” / “TX: ELRS” / “TX: —”).  
  - ENTER = select/confirm, BACK = back; optional long-press BACK for “Menu”.  
  - Top-level: TX Mode (Auto/Custom/ELRS), Re-detect TX (optional), Binding/ELRS, Calibration, Back.  
  - Binding/ELRS (when ELRS): View phrase, Set phrase, Initiate pairing, Back.  
  - Show binding phrase on RC display (from CRSF param read).  
  - Set phrase: generate or simple input → PARAMETER_WRITE; show “Writing…” / “Done” or error.
- **Docs**  
  - Document in this file: ELRS param ID for binding phrase, COMMAND bytes (after lookup).  
  - Add wiring note: RC GP8/GP9 to ELRS module TX/RX, 420000 baud.
- **Web tool (RC WebUI)**  
  - Extend GET_LINK_STATUS response and WebUI to include TX type, optional ELRS device name, and **elrsRole** (TX vs RX) when ELRS (§5).  
  - When ELRS RX (wrong mode): show warning on display and WebUI; do not use for channels (§1.4.7).  
  - Optional: add RE_DETECT_TX and GET_ELRS_BINDING_PHRASE commands; WebUI UI for re-detect and “View phrase (from ELRS)”.  
  - AutoPairPanel and status areas show “TX: Custom” / “TX: ELRS” (or “Wrong mode: ELRS RX”); ELRS-specific copy where relevant.

---

## 5. Web tool (RC WebUI) updates / upgrades

The RC WebUI (`tools/rc-webui`) talks to the RC over USB using the **RCConfigProtocol** (sync 0xA5, binary frames). To support ELRS, auto-detection, and a consistent experience, the following protocol and UI updates are needed.

### 5.1 Protocol (RCConfigProtocol) changes

- **Extend GET_LINK_STATUS (0x42) response**  
  - **Current:** Payload is 3 bytes: `[txConnected, txPaired, txState]`.  
  - **New:** Extend to at least 4 bytes: `[txConnected, txPaired, txState, txType]`.  
    - `txType`: 0 = unknown, 1 = custom TX, 2 = ELRS.  
    - When `txType == 2`, optionally add **`elrsRole`**: e.g. 1 = TX (correct), 2 = RX (wrong mode). See §1.4.7.  
  - **Optional:** Append ELRS device name when `txType == 2`: e.g. 1 byte length + N bytes UTF-8 name (max length cap, e.g. 20 bytes).  
  - **Firmware:** Extend the link-status getter callback to supply `txType`, `elrsRole` (when ELRS), and optionally device name. RC fills these from boot detection (Extended Source in DEVICE_INFO: 0xEE = TX, 0xEC = RX).
- **Optional new commands**  
  - **RE_DETECT_TX (e.g. 0x44):** No payload. RC runs the detection sequence again (custom PING then CRSF DEVICE_PING), updates internal `tx_type`, and optionally returns OK. WebUI can then call GET_LINK_STATUS again to show updated TX type.  
  - **GET_ELRS_BINDING_PHRASE (e.g. 0x45):** No payload (or optional). RC requests current binding phrase from ELRS via CRSF PARAMETER_READ / 0x2B, then sends response with status and phrase string (length-prefixed or null-terminated, max length e.g. 32). If TX is not ELRS or ELRS doesn’t respond, return error. WebUI uses this for “View phrase (from ELRS)” when TX type is ELRS.

### 5.2 WebUI (tools/rc-webui) code changes

- **protocol.ts (and types)**  
  - Extend `LinkStatus`: add `txType: 0 | 1 | 2` (unknown / custom / elrs), optionally `elrsDeviceName?: string`, and **`elrsRole?: 'tx' | 'rx'`** (when txType === 2: correct TX module vs wrong-mode RX).  
  - Update `parseLinkStatus(payload)` to read the 4th byte as `txType`; if payload is longer and `txType === 2`, parse optional length-prefixed device name and optional byte as `elrsRole` (e.g. 1 = TX, 2 = RX).  
  - Keep backward compatibility: if payload length is 3, treat as old format and set `txType = 0` (unknown), `elrsRole` undefined.
- **AutoPairPanel**  
  - **Preflight / status:** Show TX type in the “TX reachable via RC” line, e.g. “TX: Custom — Paired” or “TX: ELRS — Reachable”. Use `linkStatus.txType` when available.  
  - **Wrong mode (ELRS RX):** When `txType === 2` and `elrsRole === 'rx'`, show a **warning**: e.g. “TX: ELRS **RX (wrong mode)** — need transmitter module” or “Connected device is an ELRS receiver; use an ELRS transmitter.” Disable or grey out “Auto pair” with tooltip that a TX module is required (see §1.4.7).  
  - **Step labels (optional):** When `txType === 2` and `elrsRole === 'tx'`, optionally show “Apply phrase to TX (via CRSF)” and “Enter TX bind mode” for the existing steps.  
  - **State label:** For ELRS TX, “Paired” may be shown as “Bound” where it makes sense (optional copy tweak).  
  - **Re-detect:** If RE_DETECT_TX is implemented, add a “Re-detect TX” button (e.g. next to “Refresh TX status”) that sends the command and then refreshes link status.
- **ConfigPanel (binding phrase)**  
  - When `linkStatus?.txType === 2` (ELRS), the existing “Set TX binding phrase” still applies (RC forwards to ELRS via CRSF PARAMETER_WRITE). Optionally show a short note: “Phrase will be set on ELRS TX via CRSF.”  
  - If GET_ELRS_BINDING_PHRASE is implemented: add a “View phrase (from ELRS)” or “Refresh phrase from TX” control that calls the new command and displays the returned string (read-only). Useful to confirm what phrase the ELRS module currently has.
- **LivePanel / header / global status**  
  - Wherever RC or TX status is summarized (e.g. header or a status bar), show “TX: Custom” or “TX: ELRS” (and optionally device name) when `linkStatus.txType` is available, so the user always knows which protocol is in use.
- **Error and edge cases**  
  - If RC returns `txType === 0` (unknown) but `txConnected === true`, show something like “TX connected (type unknown)” so the user knows the RC sees something but detection didn’t identify it.  
  - When ELRS is selected but GET_ELRS_BINDING_PHRASE fails (e.g. timeout), show a clear message (“Could not read phrase from ELRS” / “ELRS not responding”).

### 5.3 Behaviour summary


| Feature                      | Protocol                                                 | WebUI                                                                 |
| ---------------------------- | -------------------------------------------------------- | --------------------------------------------------------------------- |
| Show TX type (Custom / ELRS) | GET_LINK_STATUS extended with `txType` (+ optional name) | AutoPairPanel, status area, optional header                          |
| Show ELRS role (TX vs RX)    | GET_LINK_STATUS extended with `elrsRole` when txType=ELRS | When `elrsRole === 'rx'`: warning “Need TX module, this is RX”; disable pairing (§1.4.7) |
| Re-detect TX without reboot  | New RE_DETECT_TX (0x44)                                  | “Re-detect TX” button; then refresh GET_LINK_STATUS                   |
| View ELRS binding phrase     | New GET_ELRS_BINDING_PHRASE (0x45)                       | “View phrase (from ELRS)” when TX is ELRS           |
| Set TX phrase (unchanged)    | SET_BINDING_PHRASE_TX (0x41)                             | Same; RC sends to custom TX or ELRS via CRSF        |
| Enter pairing / bind         | ENTER_PAIRING_MODE (0x43)                                | Same; RC sends custom PAIR or ELRS Bind command     |


### 5.4 Implementation checklist (Web tool)

- **Protocol:** Extend GET_LINK_STATUS response to 4+ bytes (txType; optional device name; optional elrsRole when txType=ELRS). Update firmware getter and RCConfigProtocol handler.  
- **protocol.ts:** Extend `LinkStatus` (txType, elrsRole, optional name) and `parseLinkStatus`; backward compatibility for 3-byte payload.  
- **AutoPairPanel:** Display TX type; when ELRS RX (wrong mode), show warning and disable/grey pairing; optional ELRS TX step labels and “Re-detect TX” if command exists.  
- **ConfigPanel:** Optional “View phrase (from ELRS)” when GET_ELRS_BINDING_PHRASE is implemented.  
- **Status / header:** Show “TX: Custom” / “TX: ELRS” where status is shown.  
- **Optional:** Implement RE_DETECT_TX (firmware + protocol + WebUI button).  
- **Optional:** Implement GET_ELRS_BINDING_PHRASE (firmware + protocol + WebUI “View phrase”).

---

## 6. References

- [Packet Types · crsf-wg/crsf Wiki](https://github.com/crsf-wg/crsf/wiki/Packet-Types) – CRSF frame types (0x16, 0x28, 0x29, 0x2B, 0x2C, 0x2D, 0x32).
- [CRSF_FRAMETYPE_PARAMETER_WRITE](https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_PARAMETER_WRITE) – 0x2D payload: field index + value.
- [CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY](https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY) – 0x2B payload: field index, type, label, value (STRING, COMMAND, etc.).
- [ExpressLRS Wiki – CRSF Protocol](https://github.com/ExpressLRS/ExpressLRS/wiki/CRSF-Protocol) – ELRS-specific CRSF usage.
- [ExpressLRS Binding](https://www.expresslrs.org/quick-start/binding/) – Binding phrase and methods.
- [ExpressLRS Lua Script](https://www.expresslrs.org/quick-start/transmitters/lua-howto) – Lua uses CRSF to talk to TX module; same parameter/command ideas apply to our RC.
- [CRSF_FRAMETYPE_DEVICE_PING](https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_DEVICE_PING) – Send 0x28 to discover devices (extended frame).
- [CRSF_FRAMETYPE_DEVICE_INFO](https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_DEVICE_INFO) – Response 0x29; serial "ELRS" and display name identify ExpressLRS.
- ExpressLRS source (GitHub): search for `PARAMETER_`*, `binding`, `COMMAND`, `0x32` to get param IDs and command subtypes.

---

## 7. Implementation notes (feature/rc-elrs)

- **TX detection:** Implemented at boot: Phase 1 custom PING → PONG (Custom TX); Phase 2 CRSF DEVICE_PING to 0xEE → DEVICE_INFO with serial "ELRS" (ELRS). Timeout 400 ms per phase.
- **ELRS role (TX vs RX):** DEVICE_INFO is accepted from both 0xEE (transmitter) and 0xEC (receiver). Frame source (`device_addr`) sets `elrs_role`: 0xEE = TX (correct), 0xEC = RX (wrong mode). When RX: display shows "Wrong: ELRS RX"; no CRSF channel output; Binding menu and phrase/pairing actions disabled. GET_LINK_STATUS includes `elrsRole` (1=TX, 2=RX) when txType=2; response layout: [conn, paired, state, txType, elrsRole, nameLen, name...].
- **CRSF channel output:** When ELRS **TX** (elrs_role == TX), RC sends CRSF RC_CHANNELS_PACKED (0x16) to 0xEE at 250 Hz. Not sent when ELRS RX (wrong mode).
- **Display:** Main screen shows "TX: Custom" / "TX: ELRS" (+ device name) / "Wrong: ELRS RX" / "TX: --" or "Detecting TX...". Menu: BACK long-press opens; TX Mode, Re-detect TX, Binding/ELRS (only when ELRS TX), Back.
- **Protocol:** GET_LINK_STATUS extended with `txType`, `elrsRole` (when ELRS), and optional device name. RE_DETECT_TX (0x44), GET_ELRS_BINDING_PHRASE (0x45) implemented.
- **WebUI:** LinkStatus has txType, elrsRole ('tx'|'rx'), and elrsDeviceName; parseLinkStatus reads new 6-byte+ layout when txType=2. AutoPairPanel shows TX type; when elrsRole==='rx' shows warning "ELRS RX (wrong mode) — need transmitter module" and disables Auto pair. ConfigPanel "View phrase (from ELRS)" rejects when elrsRole==='rx'.
- **ELRS binding phrase / COMMAND:** Implemented. CrsfSerial parses PARAMETER_SETTINGS_ENTRY (0x2B) from 0xEE; RC discovers binding phrase (STRING, label contains "inding" and "hrase") and bind command (COMMAND, label contains "ind" but not "hrase"). PARAMETER_READ (0x2C) requests chunks 0..N; PARAMETER_WRITE (0x2D) sets phrase string or bind step 0x01 (CLICK). rc_proto_get_elrs_binding_phrase triggers param read and returns phrase; rc_proto_set_binding_phrase_tx and rc_proto_enter_pairing_mode use ELRS CRSF params when TX is ELRS.
- **Binding/ELRS menu:** Top menu shows "Binding/ELRS" when TX is ELRS. Submenu: View phrase (requests param read, shows cached phrase), Set phrase (generate QP-XXXX-XXXX-XXXX + PARAMETER_WRITE), Initiate pairing (PARAMETER_WRITE bind 0x01), Back.
- **WebUI ConfigPanel:** "View phrase (from ELRS)" button when TX is ELRS; calls GET_ELRS_BINDING_PHRASE and displays result. Note "Phrase set via CRSF" for Set TX phrase when ELRS.

## 8. Notes (original)

- **CRC:** CRSF uses CRC8 (poly 0xD5) over length + type + payload (not address). Existing `lib/CRSF` and `crsfSerial` already use this.
- **ELRS version:** TX and RX firmware major version must match for binding; display can show ELRS version from DEVICE_INFO (0x29) if we implement PING.
- **Phrase rules:** ELRS expects alphanumeric (Latin); length ≥ 8 recommended. Our WebUI already generates phrases like `QP-XXXX-XXXX-XXXX`; same can be used for “Set phrase” from RC menu (e.g. “Generate” button that creates one and sends PARAMETER_WRITE).

