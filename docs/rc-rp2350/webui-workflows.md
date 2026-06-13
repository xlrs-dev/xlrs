# WebUI and Workflows

The WebUI is a host-side tool that talks to the RC handset over the `rc.v1` USB
protocol. It should not connect directly to SX1280 state or rewrite TX/RX flash
layouts. For low-level bring-up, use a serial terminal and the current RC console
commands in [RC RP2350 Handset Port](index.md).

## WebUI Usage

Expected connection flow:

1. Connect the RC RP2350 board over USB.
2. Start the WebUI:

   ```bash
   cd tools/rc-webui
   npm install
   npm run dev
   ```

3. Open the printed `127.0.0.1` URL in Chrome or Edge.
4. Connect the **RC USB** session. The WebUI sends `rc.v1 hello` and checks for
   an `rc.v1 ok` response.
5. Use **Discover** or **Stream** to request `hello`, `tx_hello`, `state`, and
   `stream_state`.
6. User actions call calibration, config, or bind commands.

Suggested WebUI pages:

| Page | Purpose |
| --- | --- |
| Connection panels | Device identity, firmware version, capability discovery, raw send box, and serial log. |
| Calibration Wizard | Sample full ADC travel, derive center from endpoints, and apply or save calibration. |
| Channels | Axis mapping, inversion, deadzone, trims, cutoffs, and live channel output. |
| Filters / Save | Filter settings, apply/save controls, and RC config default reset. |
| Binding Wizard | TX binding through RC USB, RX binding through direct USB, UID comparison, and reboot warnings. |

## Binding TX via RC USB and RX via Direct USB

This workflow keeps the RC board as the user-facing controller while the RX stays
observable on its own USB serial console.

1. Build and flash the TX and RX firmware.
2. Wire RC CRSF UART TX/RX to the TX module CRSF UART pins.
3. Connect the RC board to the host over USB and open the WebUI.
4. Connect the RX board to the host over direct USB serial at 115200 baud.
5. In the WebUI, connect both **RC USB** and **RX USB** sessions, then run
   **Discover** on each.
6. In **Binding Wizard**, enter the shared binding phrase and verify or set TX
   through RC USB:

   ```text
   rc.v1 tx_hello seq=30
   rc.v1 binding_verify seq=31 target=tx phrase=Field%20Radio
   rc.v1 binding_set seq=32 target=tx phrase=Field%20Radio
   ```

7. Set or verify RX over its own direct USB session:

   ```text
   rc.v1 binding_verify seq=33 target=rx phrase=Field%20Radio
   rc.v1 binding_set seq=34 target=rx phrase=Field%20Radio
   ```

8. Compare `uid_check` values. They must match for TX and RX.
9. Reboot any target that returned `requires_reboot=1`.
10. Confirm TX/RX status logs report the same binding identity and the RX reaches
    connected state.

Fallback serial CLI:

```text
bind get
bind set <phrase>
bind clear
```

Both TX and RX must use the same phrase unless OTA bind has already persisted the
RX UID.

## Calibration Workflow

The WebUI collects calibration into a temporary firmware session before
committing anything to active config or flash.

1. Connect **RC USB** and run **Load current config**.
2. Send `cal_start`. The firmware starts a CRSF safety hold so live output stays
   disarmed during calibration.
3. Move every stick to every endpoint and press **Sample** repeatedly while the
   WebUI sends `cal_sample`.
4. Finish with **Apply only** or **Apply + save**. The firmware derives each
   calibration center from the collected ADC min/max endpoints, then maps min to
   CRSF 1000 and max to CRSF 2000.
5. Move throttle and arm/aux controls low. The CRSF safety hold releases only
   after the runtime sees safe low throttle and aux values.
6. Preview raw ADC and channel output with `state`.

Example:

```text
rc.v1 get_config seq=19
rc.v1 cal_start seq=20
rc.v1 cal_sample seq=21
rc.v1 state seq=22
rc.v1 cal_finish seq=23 save=1
```

Validation rules should reject:

- Min/max spans that are too small.
- Centers outside the collected endpoints; normal WebUI calibration derives the
  center from the endpoints.
- Throttle low value that maps above the configured low threshold.
- Switches that did not visit all expected positions.
- Calibration writes while a bind or previous flash write is active.

Cancellation handling is not represented by a dedicated parser command yet; until
one is added, host tools should abort locally and avoid sending `cal_finish`.

## Power Behavior

The RC board should continue CRSF output whenever it is powered and the TX module
is attached. USB connection to the WebUI must not be required for control output.

Expected behavior:

| Condition | Behavior |
| --- | --- |
| USB only | WebUI and CRSF output available if hardware powers the TX path. |
| Battery only | CRSF output available; WebUI unavailable until USB connects. |
| USB plus battery | Report charging/source state when hardware supports it. |
| Low battery warning | Keep CRSF output active and surface warning through the `power` console output and display state. |
| Critical battery | Prefer explicit user warning and orderly shutdown over silent channel changes. |
| USB disconnect | Keep last committed settings and continue CRSF output. |

Power policy belongs to the RC board. RF transmit power, dynamic power, failsafe,
and region remain TX/RX radio configuration.

Bring-up checks:

```text
power
status
rc.v1 state seq=50
```

## Build and Flash

Build all firmware:

```bash
scripts/build.sh all
```

Build only the RC handset:

```bash
scripts/build.sh rc
```

Manual PlatformIO command:

```bash
pio run -e rc-rp2350
```

Flash the RC handset in BOOTSEL mode:

```bash
scripts/flash.sh rc
```

Manual UF2 path:

```text
.pio/build/rc-rp2350/firmware.uf2
```

Docs/WebUI validation commands:

```bash
npm --prefix tools/rc-webui run build
scripts/test.sh
```
