#!/usr/bin/env bash
set -u

failures=0

ok() {
  printf '[OK]   %s\n' "$1"
}

fail() {
  failures=$((failures + 1))
  printf '[FAIL] %s\n' "$1"
}

version_line() {
  "$@" 2>/dev/null | head -n 1
}

printf 'Clean LoRa Link PlatformIO environment check\n\n'

if command -v pio >/dev/null 2>&1; then
  ok "PlatformIO found: $(command -v pio) ($(version_line pio --version))"
else
  fail "PlatformIO CLI not found. Install it and make sure 'pio' is on PATH."
fi

if command -v python3 >/dev/null 2>&1; then
  ok "Python 3 found: $(command -v python3) ($(version_line python3 --version))"
else
  fail "python3 not found."
fi

if command -v picotool >/dev/null 2>&1; then
  ok "picotool found: $(command -v picotool)"
else
  ok "picotool not found; UF2 copy flashing is still supported."
fi

if [[ -f platformio.ini ]]; then
  ok "platformio.ini found."
else
  fail "platformio.ini is missing."
fi

if [[ "${failures}" -eq 0 ]]; then
  ok "Environment is ready for scripts/test.sh and scripts/build.sh."
else
  fail "Environment is not ready: ${failures} failure(s)."
  exit 1
fi
