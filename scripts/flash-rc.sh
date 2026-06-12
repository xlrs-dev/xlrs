#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"

usage() {
  printf 'Usage: %s [PORT]\n' "$0"
  printf '\n'
  printf 'Build and flash the RC Pico 2 firmware to the only connected board, or to PORT.\n'
  printf '\n'
  printf 'Examples:\n'
  printf '  %s\n' "$0"
  printf '  %s /dev/cu.usbmodem14101\n' "$0"
  printf '  RC_PORT=/dev/cu.usbmodem14101 %s\n' "$0"
  printf '  PORT=/dev/cu.usbmodem14101 %s\n' "$0"
}

case "${1:-}" in
  -h|--help)
    usage
    exit 0
    ;;
esac

upload_port="${1:-${RC_PORT:-${PORT:-}}}"
cmd=(pio run -e rc-rp2350 -t upload)
if [[ -n "${upload_port}" ]]; then
  cmd+=(--upload-port "${upload_port}")
fi

printf 'Flashing RC Pico 2 firmware'
if [[ -n "${upload_port}" ]]; then
  printf ' via %s' "${upload_port}"
else
  printf ' to the single connected RP2350 device'
fi
printf '.\n'

"${cmd[@]}"
