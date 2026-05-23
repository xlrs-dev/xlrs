#!/usr/bin/env bash
set -euo pipefail

target="${1:-}"
baud="${BAUD:-115200}"

usage() {
  printf 'Usage: %s tx|rx|both|PORT [PORT]\n' "$0"
  printf '\n'
  printf 'Examples:\n'
  printf '  TX_PORT=/dev/cu.usbmodem101 RX_PORT=/dev/cu.usbmodem102 %s both\n' "$0"
  printf '  %s /dev/cu.usbmodem101\n' "$0"
  printf '  BAUD=115200 %s tx\n' "$0"
  printf '\n'
  printf 'Available serial ports:\n'
  ls -1 /dev/cu.* /dev/tty.* 2>/dev/null | sed 's/^/  /' || true
}

configure_port() {
  local port="$1"
  if [[ "$(uname -s)" == "Darwin" ]]; then
    stty -f "${port}" "${baud}" cs8 -cstopb -parenb -ixon -ixoff raw -echo
  else
    stty -F "${port}" "${baud}" cs8 -cstopb -parenb -ixon -ixoff raw -echo
  fi
}

monitor_port() {
  local label="$1"
  local port="$2"

  if [[ ! -e "${port}" ]]; then
    printf '[%s] Missing serial port: %s\n' "${label}" "${port}" >&2
    return 1
  fi

  configure_port "${port}"
  printf '[%s] Monitoring %s at %s baud. Ctrl-C to stop.\n' "${label}" "${port}" "${baud}" >&2

  while IFS= read -r line; do
    printf '[%s] %s\n' "${label}" "${line}"
  done < "${port}"
}

case "${target}" in
  tx)
    if [[ -z "${TX_PORT:-}" ]]; then
      printf 'TX_PORT is required for tx monitoring.\n' >&2
      usage >&2
      exit 2
    fi
    monitor_port TX "${TX_PORT}"
    ;;
  rx)
    if [[ -z "${RX_PORT:-}" ]]; then
      printf 'RX_PORT is required for rx monitoring.\n' >&2
      usage >&2
      exit 2
    fi
    monitor_port RX "${RX_PORT}"
    ;;
  both)
    if [[ -z "${TX_PORT:-}" || -z "${RX_PORT:-}" ]]; then
      printf 'TX_PORT and RX_PORT are required for both monitoring.\n' >&2
      usage >&2
      exit 2
    fi
    trap 'pids="$(jobs -p)"; [[ -z "${pids}" ]] || kill ${pids} 2>/dev/null || true' INT TERM EXIT
    monitor_port TX "${TX_PORT}" &
    monitor_port RX "${RX_PORT}" &
    wait
    ;;
  -h|--help|"")
    usage
    ;;
  *)
    if [[ -n "${2:-}" && -e "${target}" && -e "${2}" ]]; then
      trap 'pids="$(jobs -p)"; [[ -z "${pids}" ]] || kill ${pids} 2>/dev/null || true' INT TERM EXIT
      monitor_port TX "${target}" &
      monitor_port RX "${2}" &
      wait
    elif [[ -e "${target}" ]]; then
      monitor_port LOG "${target}"
    else
      printf 'Unknown target or missing port: %s\n' "${target}" >&2
      usage >&2
      exit 2
    fi
    ;;
esac
