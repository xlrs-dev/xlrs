#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ELRS_SRC="${ROOT}/third_party/ExpressLRS/src"
TARGET="${1:-both}"
EXTRA_ARGS=()

usage() {
  printf 'Usage: %s [tx|rx|both] [extra pio args...]\n' "$0"
  printf '\n'
  printf 'Builds ExpressLRS RP2040 firmware from %s\n' "${ELRS_SRC}"
  printf '\n'
  printf 'Examples:\n'
  printf '  %s both\n' "$0"
  printf '  %s tx -j 8\n' "$0"
}

if [[ ! -d "${ELRS_SRC}" ]]; then
  printf 'Missing ExpressLRS checkout at %s\n' "${ELRS_SRC}" >&2
  printf 'Run scripts/vendor-elrs.sh first.\n' >&2
  exit 1
fi

case "${TARGET}" in
  tx)
    ENVS=(XLRS_RP2040_TX)
    shift
    ;;
  rx)
    ENVS=(XLRS_RP2040_RX)
    shift
    ;;
  both)
    ENVS=(XLRS_RP2040_TX XLRS_RP2040_RX)
    shift
    ;;
  -h|--help)
    usage
    exit 0
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac

if (($#)); then
  EXTRA_ARGS=("$@")
fi

if ! command -v pio >/dev/null 2>&1; then
  printf 'PlatformIO CLI not found in PATH (missing `pio`).\n' >&2
  exit 1
fi

for env_name in "${ENVS[@]}"; do
  printf '==> Building %s\n' "${env_name}"
  pio run -d "${ELRS_SRC}" -e "${env_name}" ${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}
done
