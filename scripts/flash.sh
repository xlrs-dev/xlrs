#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${BUILD_DIR:-${repo_root}/build}"
target="${1:-}"

usage() {
  printf 'Usage: %s tx|rx|both\n' "$0"
  printf '\n'
  printf 'Put one Pico in BOOTSEL mode before flashing each target.\n'
}

find_picotool() {
  if command -v picotool >/dev/null 2>&1; then
    command -v picotool
    return 0
  fi
  return 1
}

flash_one() {
  local name="$1"
  local uf2="${build_dir}/xlrs_${name}.uf2"

  if [[ ! -f "${uf2}" ]]; then
    printf 'Missing %s. Run scripts/build.sh first.\n' "${uf2}" >&2
    exit 1
  fi

  printf 'Flashing %s. Put the %s Pico in BOOTSEL mode now.\n' "${uf2}" "${name^^}"
  read -r -p 'Press Enter to continue... '
  "${picotool_bin}" load -f "${uf2}"
  "${picotool_bin}" reboot
}

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

if ! picotool_bin="$(find_picotool)"; then
  printf 'picotool is not on PATH. Install a USB-capable picotool, or drag the UF2 files from build/ onto the RPI-RP2 drive.\n' >&2
  exit 1
fi

if ! "${picotool_bin}" help 2>/dev/null | grep -Eq '^[[:space:]]+load[[:space:]]'; then
  printf '%s does not expose USB load/reboot commands. Install a USB-capable picotool.\n' "${picotool_bin}" >&2
  exit 1
fi

case "${target}" in
  tx)
    flash_one tx
    ;;
  rx)
    flash_one rx
    ;;
  both)
    flash_one tx
    flash_one rx
    ;;
esac
