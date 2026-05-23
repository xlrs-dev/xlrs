# Getting Started

XLRS is the firmware for a paired TX/RX radio bridge:

```text
Controller UART -> xlrs_tx -> SX1280 RF link -> xlrs_rx -> CRSF UART -> Flight controller
```

The fastest path for a new contributor is:

1. Read [developer/terminology.md](developer/terminology.md).
2. Run the host-native test suite.
3. Build both firmware images.
4. Flash and monitor TX/RX boards.
5. Use [troubleshooting/index.md](troubleshooting/index.md) for bring-up.

```bash
scripts/check-env.sh
scripts/test.sh
scripts/build.sh
```

If the firmware builds and the native tests pass, continue with
[Build, Test, Flash](build-test-flash.md).
