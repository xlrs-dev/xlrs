# Switchable TX/RX module firmware

The `txrx_module` PlatformIO env builds **one** image that contains both the TX and
RX applications. The role lives in EEPROM and is changed from the integrated WebUI
(**TX/RX Module** tab) or the CLI; no reflash is needed to switch. A module with no
stored role boots as **RX** (build with `-DTXRX_DEFAULT_ROLE_TX` for the opposite).

## How it works

- `src/role_app.inl` is the TX/RX application (previously the body of `main.cpp`).
- `src/main.cpp` includes it once for fixed-role envs (`tx_lora_pico`, `rx_lora_pico`,
  ... - behaviour unchanged) or twice, inside `namespace txrole` / `namespace rxrole`,
  when `TXRX_SWITCHABLE` is defined. Namespacing keeps each role's file-static state
  separate; the role logic itself is not modified.
- `include/lora_link/role_store.h` defines the role record. It lives at EEPROM offset
  128 (magic + role + inverted role). `ConfigRecord` (offset 0) is untouched, so the
  binding phrase survives a role switch. An unset or corrupt record falls back to RX.
- Only the selected role's `setup()`/`loop()` ever run; the other role is dead code
  in flash (about +8 KB flash, +1 KB RAM vs. a fixed-role build).

## rc.v1 commands (switchable build only)

| Command | Reply |
| --- | --- |
| `rc.v1 hello` | `role=tx\|rx fw=<ver> module=txrx switchable=1 build=<date> caps=binding,role_switch,bootsel` |
| `rc.v1 role_get` | `role=tx\|rx` |
| `rc.v1 role_set role=tx\|rx` | `ok ... persisted=1 rebooting=1`, then reboots into the new role |
| `rc.v1 bootsel` | `ok bootsel=1`, then reboots into the RP2040 USB drive (`RPI-RP2`) |

## WebUI

The **TX/RX Module** tab uses the "RX USB" connection card (a standalone module is
its own USB device). Role switch and flash both go through a themed confirmation
dialog (default focus on Cancel; Esc cancels). Flash: pick a `.uf2` (validated), the
module enters `bootsel`, the user selects the `RPI-RP2` drive, the file is copied.
Chrome/Edge only; other browsers use "Reboot to flash mode only" and drag the file.

## Shipping

Flash `.pio/build/txrx_module/firmware.uf2` to every unit. Bump `TXRX_FW_VERSION` in
`platformio.ini` per release.

## Not yet validated on hardware

Built and unit-tested only. Before shipping, bench-test: TX<->RX link in both roles
from the one image, role switch persisting across power cycles, binding phrase
surviving a role switch, reconnect timing after reboot, and the UF2 flow on
Windows/macOS/Linux.
