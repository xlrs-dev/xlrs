# RC RP2350 Validation

Use this checklist when changing the RC handset port. The goal is to validate the
RC board without duplicating TX/RX radio tests.

## Host Checks

Run from the repository root:

```bash
scripts/check-env.sh
scripts/test.sh
scripts/build.sh rc
```

For WebUI-only changes:

```bash
npm --prefix tools/rc-webui run build
```

If TX/RX behavior changed too, run the full build:

```bash
scripts/build.sh
```

`scripts/build.sh rc` runs the RC PlatformIO build with one job and retries once.
This works around observed RP2350/Arduino package build races where a clean build
can miss an object directory or object file inside generated framework/library
artifacts.

## Current Firmware Smoke Test

1. Flash the RC board:

   ```bash
   scripts/flash.sh rc
   ```

2. Open USB serial at 115200 baud.
3. Confirm boot text includes:

   ```text
   RC RP2350 CRSF output
   ```

4. Run:

   ```text
   status
   channels
   power
   ```

5. Move sticks and switches. Confirm `channels` changes and `status` shows
   increasing `frames`.
6. Attach CRSF UART to the TX module. Confirm the TX receives CRSF RC frames.

## rc.v1 Parser Fixtures

The parser already has host tests. Keep these fixtures covered when adding
runtime handlers:

| Fixture | Expected result |
| --- | --- |
| `rc.v1 hello seq=42` | Parses and echoes `seq=42`. |
| `rc.v1 tx_hello seq=3` | Parses attached-TX discovery. |
| `rc.v1 set_config seq=7 field=binding_phrase value=Bench%20Rig` | Decodes percent-encoded value. |
| `rc.v1 get_config field=rate` | Accepts field selector. |
| `rc.v1 apply` | Parses runtime apply command. |
| `rc.v1 save` | Parses persistent save command. |
| `rc.v1 reset_defaults target=rc_config` | Parses target selector. |
| `rc.v1 cal_start seq=1 field=aileron` | Parses calibration start. |
| `rc.v1 cal_sample seq=2 field=aileron value=2048` | Parses calibration sample. |
| `rc.v1 cal_finish seq=3 save=1` | Parses calibration finish with save flag. |
| `rc.v1 binding_set target=tx phrase=Field%20Radio` | Parses target and phrase alias. |
| `rc.v1 binding_verify target=rx value=Field%20Radio` | Parses direct RX verification request. |
| `rc.v2 hello` | `bad_protocol`. |
| `rc.v1 hello seq` | `bad_argument`. |
| `rc.v1 set_config field=name value=Bad%XZ` | `bad_percent_encoding`. |
| `rc.v1 set_config field=name` | `missing_required_argument`. |
| `rc.v1 reboot` | `unknown_command`. |

## Calibration Acceptance

Before accepting a calibration write:

- Every primary axis has a usable min/max span.
- Centered sticks map near CRSF midpoint.
- Throttle low maps near CRSF low endpoint.
- Switches report all expected positions.
- A failed or canceled session leaves the previous calibration active.
- Power loss during write leaves either the previous valid record or the complete
  new record; never a partially accepted calibration.

## Binding Acceptance

For the RC USB plus RX direct USB workflow:

1. Connect the RC USB session and run `rc.v1 tx_hello`.
2. Set or verify the TX through RC USB with `target=tx`.
3. Connect the RX direct USB session and set or verify the RX with `target=rx`.
4. Compare `uid_check` values returned by TX and RX; they must match.
5. Reboot any target that returned `requires_reboot=1`.
6. After reboot, verify TX and RX connect without changing the RX binding phrase
   manually.
7. Confirm the RX produces CRSF RC output only after link acquisition.

## Power Acceptance

Validate each hardware power source that exists on the board:

- USB-only boot.
- Battery-only boot.
- USB attach while already running from battery.
- USB detach while running from battery.
- Low-battery warning threshold.
- Critical-battery behavior.

In all non-critical states, CRSF output should continue and channel values should
not jump because a host connected or disconnected.
