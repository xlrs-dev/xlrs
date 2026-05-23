# Bench Bring-Up

Run local checks before touching hardware:

```bash
scripts/check-env.sh
scripts/test.sh
scripts/lint.sh
scripts/build.sh
```

Flash and monitor both boards:

```bash
scripts/flash.sh tx
scripts/flash.sh rx

TX_PORT=/dev/cu.usbmodem101 RX_PORT=/dev/cu.usbmodem102 scripts/monitor.sh both
```

The TX and RX must report the same computed identity and sync word. If they do
not, fix binding/config before investigating RF behavior.

Expected debug order:

```text
build/config -> boot identity -> radio init -> sync/acquisition
             -> uplink RC -> RX CRSF output -> downlink telemetry
```

See [../troubleshooting/index.md](../troubleshooting/index.md) for the symptom
matrix and detailed flow.
