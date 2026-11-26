#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"

usage() {
  printf 'Usage: %s [PORT]\n' "$0"
  printf '\n'
  printf 'Build and flash the RC Pico 2 firmware to the only connected board, or to PORT.\n'
  printf '\n'
  printf 'Optional env:\n'
  printf '  RC_ENV=rc-rp2350|rc-rp2350-prod   Default: rc-rp2350 (USB bench build,\n'
  printf '                                     boot-hold gate skipped). Use\n'
  printf '                                     rc-rp2350-prod for battery + real QON\n'
  printf '                                     button hardware.\n'
  printf '\n'
  printf 'Examples:\n'
  printf '  %s\n' "$0"
  printf '  %s /dev/cu.usbmodem14101\n' "$0"
  printf '  RC_PORT=/dev/cu.usbmodem14101 %s\n' "$0"
  printf '  PORT=/dev/cu.usbmodem14101 %s\n' "$0"
  printf '  RC_ENV=rc-rp2350-prod %s\n' "$0"
}

case "${1:-}" in
  -h|--help)
    usage
    exit 0
    ;;
esac

rc_env="${RC_ENV:-rc-rp2350}"
upload_port="${1:-${RC_PORT:-${PORT:-}}}"
cmd=(pio run -e "${rc_env}" -t upload)
if [[ -n "${upload_port}" ]]; then
  cmd+=(--upload-port "${upload_port}")
fi

printf 'Flashing RC Pico 2 firmware (env: %s)' "${rc_env}"
if [[ -n "${upload_port}" ]]; then
  printf ' via %s' "${upload_port}"
else
  printf ' to the single connected RP2350 device'
fi
printf '.\n'

"${cmd[@]}"
