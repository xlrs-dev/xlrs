# Code Map

| Path | Responsibility |
| --- | --- |
| `apps/tx/main.cpp` | TX role app, controller UART, telemetry/status output, status LED |
| `apps/rx/main.cpp` | RX role app, CRSF output, status LED |
| `lib/xlrs/app/LinkStatusLed.h` | Shared GPIO status LED patterns for TX/RX link state |
| `lib/xlrs/app/LinkRuntimeDiag.h` | Shared tick/FHSS/PFD bench diagnostics for TX/RX status lines |
| `docs/hardware/bench-link-acquisition-retrospective.md` | May 2026 bench log history: PHY → FHSS → nonce → Connected |
| `lib/xlrs/link/Link.*` | Link lifecycle, bind/connect/failsafe, link stats |
| `lib/xlrs/link/RfScheduler.*` | Per-tick slot coordination, FHSS advance, PHY calls |
| `lib/xlrs/link/RateConfig.h` | Rate table and airtime values |
| `lib/xlrs/link/RfConfig.*` | Flash-backed RF configuration |
| `lib/xlrs/link/BindingStore.h` | Persisted binding identity |
| `lib/xlrs/ota/` | OTA frame representation and RC channel packing |
| `lib/xlrs/fhss/` | UID-seeded frequency hopping |
| `lib/xlrs/timing/` | Hardware timer adapter and PFD timing loop |
| `lib/xlrs/phy/` | Radio PHY interface and SX1280 implementation |
| `lib/xlrs/crypto/` | Pluggable cipher interface and AEAD implementation |
| `lib/xlrs/hal/` | Pico SDK hardware adapters |
| `lib/xlrs/util/` | Lock-free mailbox/ring handoff helpers |
| `lib/UARTProtocol/` | Controller UART protocol |
| `lib/crsfSerial/` | CRSF serial output support |
| `test/` | Host-native Unity/CMake tests |

Module ownership follows [architecture.md](architecture.md). Naming follows
[terminology.md](terminology.md).
