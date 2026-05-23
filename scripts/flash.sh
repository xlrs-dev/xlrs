#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${BUILD_DIR:-${repo_root}/build}"
target="${1:-}"

usage() {
  printf 'Usage: %s tx|rx|both\n' "$0"
  printf '\n'
  printf 'Put one Pico in BOOTSEL mode before flashing each target.\n'
  printf '\n'
  printf 'Optional env:\n'
  printf '  TX_UF2_VOLUME=/Volumes/RPI-RP2       Flash TX by UF2 copy\n'
  printf '  RX_UF2_VOLUME=/Volumes/RPI-RP2       Flash RX by UF2 copy\n'
  printf '  UF2_VOLUME=/Volumes/RPI-RP2          Flash current target by UF2 copy\n'
  printf '  FLASH_METHOD=picotool|uf2|auto       Default: auto\n'
}

find_picotool() {
  if command -v picotool >/dev/null 2>&1; then
    command -v picotool
    return 0
  fi
  return 1
}

picotool_has_usb_load() {
  local tool="$1"
  "${tool}" help 2>/dev/null | grep -Eq '^[[:space:]]+load[[:space:]]'
}

find_uf2_volume() {
  local name="$1"
  local role_var
  role_var="$(printf '%s_UF2_VOLUME' "${name^^}")"

  if [[ -n "${!role_var:-}" ]]; then
    printf '%s\n' "${!role_var}"
    return 0
  fi
  if [[ -n "${UF2_VOLUME:-}" ]]; then
    printf '%s\n' "${UF2_VOLUME}"
    return 0
  fi

  local volumes=()
  if [[ -d /Volumes ]]; then
    while IFS= read -r volume; do
      volumes+=("${volume}")
    done < <(find /Volumes -maxdepth 1 -type d -name 'RPI-RP2*' 2>/dev/null | sort)
  fi

  if [[ "${#volumes[@]}" -eq 1 ]]; then
    printf '%s\n' "${volumes[0]}"
    return 0
  fi

  return 1
}

flash_with_picotool() {
  local name="$1"
  local uf2="$2"
  printf 'Flashing %s with picotool. Put the %s Pico in BOOTSEL mode now.\n' "${uf2}" "${name^^}"
  read -r -p 'Press Enter to continue... '
  "${picotool_bin}" load -f "${uf2}"
  "${picotool_bin}" reboot
}

flash_with_uf2_copy() {
  local name="$1"
  local uf2="$2"
  local volume

  printf 'Flashing %s by UF2 copy. Put the %s Pico in BOOTSEL mode now.\n' "${uf2}" "${name^^}"
  read -r -p 'Press Enter when the RPI-RP2 volume is mounted... '

  if ! volume="$(find_uf2_volume "${name}")"; then
    printf 'Could not find a unique RPI-RP2 volume.\n' >&2
    printf 'Set %s_UF2_VOLUME or UF2_VOLUME and retry.\n' "${name^^}" >&2
    exit 1
  fi

  if [[ ! -d "${volume}" ]]; then
    printf 'UF2 volume does not exist: %s\n' "${volume}" >&2
    exit 1
  fi

  printf 'Copying %s -> %s/\n' "${uf2}" "${volume}"
  cp "${uf2}" "${volume}/"
  sync
  printf '%s flashed. The board should reboot and remount as USB serial.\n' "${name^^}"
}

flash_one() {
  local name="$1"
  local uf2="${build_dir}/xlrs_${name}.uf2"

  if [[ ! -f "${uf2}" ]]; then
    printf 'Missing %s. Run scripts/build.sh first.\n' "${uf2}" >&2
    exit 1
  fi

  case "${FLASH_METHOD:-auto}" in
    picotool)
      if [[ -z "${picotool_bin:-}" ]]; then
        printf 'USB-capable picotool is not available.\n' >&2
        exit 1
      fi
      flash_with_picotool "${name}" "${uf2}"
      ;;
    uf2)
      flash_with_uf2_copy "${name}" "${uf2}"
      ;;
    auto)
      if [[ -n "${picotool_bin:-}" ]]; then
        flash_with_picotool "${name}" "${uf2}"
      else
        flash_with_uf2_copy "${name}" "${uf2}"
      fi
      ;;
    *)
      printf 'Invalid FLASH_METHOD: %s\n' "${FLASH_METHOD}" >&2
      usage >&2
      exit 2
      ;;
  esac
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

picotool_bin=""
if found_picotool="$(find_picotool)" && picotool_has_usb_load "${found_picotool}"; then
  picotool_bin="${found_picotool}"
elif [[ "${FLASH_METHOD:-auto}" == "picotool" ]]; then
  printf 'No USB-capable picotool found. Use FLASH_METHOD=uf2 or install picotool with USB support.\n' >&2
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
