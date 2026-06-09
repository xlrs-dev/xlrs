# Documentation

XLRS is timing-sensitive firmware for a paired RP2040/Pico + SX1280 2.4 GHz LoRa
TX/RX, plus an RP2350 RC handset. It is ExpressLRS-*like* (CRSF in/out, FHSS,
link stats) but **not** ExpressLRS OTA compatible. The firmware that builds and
flashes is the PlatformIO `src/` tree.

```text
CRSF handset ─▶ TX ─▶ SX1280 LoRa + FHSS ─▶ RX ─▶ CRSF flight controller
```

## Start here

- [Getting Started](getting-started.md)
- [Build, Test, Flash](build-test-flash.md)
- [Pinout](hardware/pinout.md) · [SX1280 wiring](hardware/sx1280-wiring.md)

## Developer

- [Code map](developer/code-map.md) — where everything lives in `src/`/`lib/`.
- [Architecture](developer/architecture.md) — roles, timing, slots, framing.
- [OTA protocol](developer/ota-protocol.md) — frame layout and `uid_check`.
- [Timing & scheduler](developer/timing-and-scheduler.md) — tick cadence and PFD.
- [Telemetry & link stats](developer/telemetry-and-link-stats.md)
- [Safety & failsafe](developer/safety-and-failsafe.md)
- [Configuration](developer/configuration.md) — build flags and runtime CLI.
- [Terminology](developer/terminology.md) — naming source of truth.
- [Testing](developer/testing.md)

## Interfaces

- [RX CRSF output](interfaces/rx-crsf.md)
- [TX controller CRSF](interfaces/tx-controller-crsf.md) ·
  [TX controller UART](interfaces/tx-controller-uart.md)
- [CRSF binding](crsf/binding.md)
- [RF config storage](interfaces/rf-config-storage.md)

## RC handset (RP2350)

- [Overview](rc-rp2350/index.md) · [Architecture](rc-rp2350/architecture.md)
- [USB `rc.v1` protocol](rc-rp2350/usb-protocol-rc-v1.md) ·
  [WebUI workflows](rc-rp2350/webui-workflows.md)

## Hardware & troubleshooting

- [Bench bring-up](hardware/bench-bringup.md)
- [Symptom matrix](troubleshooting/symptom-matrix.md) ·
  [Serial logs](troubleshooting/serial-logs.md) ·
  [SX1280 PHY init](troubleshooting/sx1280-phy-init.md)

---

The current build contract is the PlatformIO `src/` firmware. An earlier
clean-slate design (the XLRS core) now lives under
[`_legacy/`](../_legacy/README.md), reference-only and not built; some developer
notes still describe that design's intent. Where they disagree with `src/`,
`src/` wins.
