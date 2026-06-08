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

Manual commands:

```bash
pio run -e tx_lora_pico
pio run -e rx_lora_pico
```

UF2 outputs:

```text
.pio/build/tx_lora_pico/firmware.uf2
.pio/build/rx_lora_pico/firmware.uf2
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
```

The helper flashes:

```text
tx -> .pio/build/tx_lora_pico/firmware.uf2
rx -> .pio/build/rx_lora_pico/firmware.uf2
```

Force a method if needed:

```bash
FLASH_METHOD=uf2 scripts/flash.sh tx
FLASH_METHOD=picotool scripts/flash.sh rx
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
