# Getting Started

This firmware is a paired RP2040/Pico + SX1280 LoRa bridge:

```text
CRSF handset -> tx_lora_pico -> SX1280 LoRa/FHSS -> rx_lora_pico -> CRSF flight controller
```

Build and test from the repository root:

```bash
scripts/check-env.sh
scripts/test.sh
scripts/build.sh
```

Flash TX and RX separately in BOOTSEL mode:

```bash
scripts/flash.sh tx
scripts/flash.sh rx
```

After flashing, open the USB serial console at 115200 baud and check both
devices:

```text
status
bind get
```

Both devices must report the same binding phrase. Change and persist it with:

```text
bind set your-phrase
reboot
```

See [Build, Test, Flash](build-test-flash.md) for the complete workflow.
