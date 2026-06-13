#!/usr/bin/env bash
# Flash xlrs_tx + xlrs_rx (or elrs_hal_test) to connected Picos.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/build"
TARGET="${1:-both}"
FORCE="${FORCE:-1}"

load_one() {
  local elf="$1"
  local label="$2"
  shift 2
  local extra=("$@")
  local args=(-x)
  if [[ "$FORCE" == "1" ]]; then
    args+=(-f)
  fi
  echo "==> Flashing $label: $elf"
  if ((${#extra[@]})); then
    picotool load "${args[@]}" "${extra[@]}" "$elf"
  else
    picotool load "${args[@]}" "$elf"
  fi
}

case "$TARGET" in
  tx)
    load_one "$BUILD/xlrs_tx.elf" "TX" --ser "${TX_SER:-E46488B28B561830}"
    ;;
  rx)
    load_one "$BUILD/xlrs_rx.elf" "RX" --ser "${RX_SER:-E4654C16430F4223}"
    ;;
  hal)
    load_one "$BUILD/elrs_hal_test.elf" "HAL test" --ser "${TX_SER:-E46488B28B561830}"
    ;;
  both)
    load_one "$BUILD/xlrs_tx.elf" "TX" --ser "${TX_SER:-E46488B28B561830}"
    load_one "$BUILD/xlrs_rx.elf" "RX" --ser "${RX_SER:-E4654C16430F4223}"
    ;;
  *)
    echo "Usage: $0 [tx|rx|both|hal]" >&2
    exit 2
    ;;
esac

echo "Done."
