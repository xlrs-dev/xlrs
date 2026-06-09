# Build, Test, Flash

## Requirements

- PlatformIO CLI (`pio`)
- Python 3, normally installed with PlatformIO
- Optional for flashing: USB-capable `picotool`

Check the local machine:

```bash
scripts/check-env.sh
```

## Build

```bash
scripts/build.sh
```

By default, the helper builds the TX/RX radio pair. Build all known firmware
targets, including the RC handset, with:

```bash
scripts/build.sh all
```

Manual commands:

```bash
pio run -e tx_lora_pico
pio run -e rx_lora_pico
pio run -e rc-rp2350
```

UF2 outputs:

```text
.pio/build/tx_lora_pico/firmware.uf2
.pio/build/rx_lora_pico/firmware.uf2
.pio/build/rc-rp2350/firmware.uf2
```

Build one role:

```bash
scripts/build.sh tx
scripts/build.sh rx
scripts/build.sh rc
```

## Test

```bash
scripts/test.sh
```

Equivalent:

```bash
pio test -e native
```

## Flash

Hold BOOTSEL while plugging in one Pico, then run:

```bash
scripts/flash.sh tx
scripts/flash.sh rx
scripts/flash.sh rc
```

The helper flashes:

```text
tx -> .pio/build/tx_lora_pico/firmware.uf2
rx -> .pio/build/rx_lora_pico/firmware.uf2
rc -> .pio/build/rc-rp2350/firmware.uf2
```

Force a method if needed:

```bash
FLASH_METHOD=uf2 scripts/flash.sh tx
FLASH_METHOD=picotool scripts/flash.sh rx
FLASH_METHOD=uf2 scripts/flash.sh rc
```

## Serial CLI

USB serial runs at 115200 baud:

```text
bind get
bind set <phrase>
bind clear
rate L250
rate L100
status
reboot
```

The `rc-rp2350` bring-up firmware exposes a smaller USB serial console:

```text
status
channels
power
reboot
```

It also accepts the implemented `rc.v1` subset used by the WebUI:

```text
rc.v1 hello
rc.v1 tx_hello
rc.v1 state
rc.v1 binding_get target=tx
rc.v1 binding_set target=tx phrase=<phrase>
rc.v1 binding_clear target=tx
rc.v1 binding_verify target=tx phrase=<phrase>
```

TX/RX direct USB accepts local binding commands with `target=tx` or `target=rx`.
