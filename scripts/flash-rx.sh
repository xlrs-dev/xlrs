#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"

usage() {
  printf 'Usage: %s [PORT]\n' "$0"
  printf '\n'
  printf 'Build and flash the RX firmware to the only connected Pico, or to PORT.\n'
  printf '\n'
  printf 'Examples:\n'
  printf '  %s\n' "$0"
  printf '  %s /dev/cu.usbmodem5\n' "$0"
  printf '  RX_PORT=/dev/cu.usbmodem5 %s\n' "$0"
  printf '  PORT=/dev/cu.usbmodem5 %s\n' "$0"
}

case "${1:-}" in
  -h|--help)
    usage
    exit 0
    ;;
esac

upload_port="${1:-${RX_PORT:-${PORT:-}}}"
cmd=(pio run -e rx_lora_pico -t upload)
if [[ -n "${upload_port}" ]]; then
  cmd+=(--upload-port "${upload_port}")
fi

printf 'Flashing RX firmware'
if [[ -n "${upload_port}" ]]; then
  printf ' via %s' "${upload_port}"
else
  printf ' to the single connected RP2040 device'
fi
printf '.\n'

"${cmd[@]}"
