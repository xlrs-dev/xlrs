# `_legacy/` — reference-only, not built

This directory holds an earlier clean-slate **XLRS core** design that is **not**
the firmware that builds or flashes today. The active firmware is the PlatformIO
`src/` tree (see the repo [README](../README.md) and
[docs/developer/architecture.md](../docs/developer/architecture.md)).

| Path | Was | Contents |
| --- | --- | --- |
| `_legacy/xlrs/` | `lib/xlrs/` | Layered XLRS core: PHY / timing / FHSS / OTA / crypto / link / util, plus Pico-SDK HAL adapters |
| `_legacy/apps/` | `apps/` | CMake/Pico-SDK role mains (`tx/main.cpp`, `rx/main.cpp`) for that core |
| `_legacy/cmake/` | `cmake/` | `pico_sdk_import.cmake` used by the legacy CMake/Pico-SDK build |

## Why it's kept

The design notes here (strict layering, a pluggable `ICipher`/AEAD secure-link
path, PFD tuning, and nonce reasoning) are useful background. Nothing in `src/`,
`lib/`, `test/`, or `platformio.ini` depends on this directory, and PlatformIO
does not scan it. Where these files disagree with the active firmware, **`src/`
is authoritative**.

Revive or delete as needed — full history is in git.
