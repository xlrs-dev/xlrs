#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  printf 'Usage: %s tx|rx|both\n' "$0"
  printf '\n'
  printf 'Flashes the selected firmware, then starts serial monitoring.\n'
  printf 'For monitoring, set TX_PORT and/or RX_PORT after the board reboots as USB serial.\n'
}

target="${1:-}"
case "${target}" in
  tx|rx|both) ;;
  -h|--help|"")
    usage
    exit 0
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac

"${repo_root}/scripts/build.sh"
"${repo_root}/scripts/flash.sh" "${target}"

printf '\nAfter the board(s) reboot, identify serial ports with:\n'
printf '  ls /dev/cu.usbmodem* /dev/tty.usbmodem* 2>/dev/null\n\n'

case "${target}" in
  tx)
    if [[ -z "${TX_PORT:-}" ]]; then
      read -r -p 'TX serial port: ' TX_PORT
      export TX_PORT
    fi
    "${repo_root}/scripts/monitor.sh" tx
    ;;
  rx)
    if [[ -z "${RX_PORT:-}" ]]; then
      read -r -p 'RX serial port: ' RX_PORT
      export RX_PORT
    fi
    "${repo_root}/scripts/monitor.sh" rx
    ;;
  both)
    if [[ -z "${TX_PORT:-}" ]]; then
      read -r -p 'TX serial port: ' TX_PORT
      export TX_PORT
    fi
    if [[ -z "${RX_PORT:-}" ]]; then
      read -r -p 'RX serial port: ' RX_PORT
      export RX_PORT
    fi
    "${repo_root}/scripts/monitor.sh" both
    ;;
esac
