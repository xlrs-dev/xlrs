#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ELRS_BUILD_DIR="${ROOT}/third_party/ExpressLRS/src/.pio/build"
TARGET="${1:-}"
FLASH_MODE="${FLASH_MODE:-runtime}"
TX_SER="${TX_SER:-E46488B28B561830}"
RX_SER="${RX_SER:-E4654C16430F4223}"
TX_BUS="${TX_BUS:-1}"
TX_ADDR="${TX_ADDR:-2}"
RX_BUS="${RX_BUS:-0}"
RX_ADDR="${RX_ADDR:-2}"

usage() {
  printf 'Usage: %s tx|rx|both\n' "$0"
  printf '\n'
  printf 'Flashes ExpressLRS RP2040 firmware with picotool.\n'
  printf 'Builds must include -DENABLE_PICOTOOL_USB (see targets/xlrs_rp2040.ini).\n'
  printf 'First flash after a non-picoboot build requires BOOTSEL; runtime -f works after that.\n'
  printf '\n'
  printf 'Env vars:\n'
  printf '  FLASH_MODE=runtime|bootsel   runtime adds -f and uses per-board serials\n'
  printf '  TX_SER=%s\n' "${TX_SER}"
  printf '  RX_SER=%s\n' "${RX_SER}"
  printf '\n'
  printf 'Firmware paths:\n'
  printf '  TX: %s/XLRS_RP2040_TX/firmware.elf\n' "${ELRS_BUILD_DIR}"
  printf '  RX: %s/XLRS_RP2040_RX/firmware.elf\n' "${ELRS_BUILD_DIR}"
}

find_picotool() {
  if command -v picotool >/dev/null 2>&1; then
    command -v picotool
    return 0
  fi
  return 1
}

upper() {
  printf '%s' "$1" | tr '[:lower:]' '[:upper:]'
}

flash_one() {
  local role="$1"
  local elf="$2"
  local serial="$3"
  local bus="$4"
  local addr="$5"
  local args=(load -x)
  local role_label
  role_label="$(upper "${role}")"

  if [[ ! -f "${elf}" ]]; then
    printf 'Missing %s. Run scripts/build-elrs.sh %s first.\n' "${elf}" "${role}" >&2
    exit 1
  fi

  case "${FLASH_MODE}" in
    runtime)
      args+=(-f --ser "${serial}")
      ;;
    bootsel)
      printf 'Put the %s board in BOOTSEL mode, then press Enter to flash %s.\n' "${role_label}" "${elf}"
      read -r
      ;;
    *)
      printf 'Invalid FLASH_MODE=%s (expected runtime or bootsel).\n' "${FLASH_MODE}" >&2
      exit 2
      ;;
  esac

  printf '==> Flashing %s from %s (%s)\n' "${role_label}" "${elf}" "${FLASH_MODE}"
  if ! "${PICOTOOL_BIN}" "${args[@]}" "${elf}"; then
    if [[ "${FLASH_MODE}" == "runtime" ]]; then
      printf '==> Retrying %s flash via bus %s address %s\n' "${role_label}" "${bus}" "${addr}" >&2
      "${PICOTOOL_BIN}" load -x -f -F --bus "${bus}" --address "${addr}" "${elf}"
    else
      return 1
    fi
  fi
}

case "${TARGET}" in
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

if ! PICOTOOL_BIN="$(find_picotool)"; then
  printf 'picotool was not found in PATH.\n' >&2
  exit 1
fi

TX_ELF="${ELRS_BUILD_DIR}/XLRS_RP2040_TX/firmware.elf"
RX_ELF="${ELRS_BUILD_DIR}/XLRS_RP2040_RX/firmware.elf"

case "${TARGET}" in
  tx)
    flash_one tx "${TX_ELF}" "${TX_SER}" "${TX_BUS}" "${TX_ADDR}"
    ;;
  rx)
    flash_one rx "${RX_ELF}" "${RX_SER}" "${RX_BUS}" "${RX_ADDR}"
    ;;
  both)
    flash_one tx "${TX_ELF}" "${TX_SER}" "${TX_BUS}" "${TX_ADDR}"
    flash_one rx "${RX_ELF}" "${RX_SER}" "${RX_BUS}" "${RX_ADDR}"
    ;;
esac
