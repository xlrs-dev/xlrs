#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"

usage() {
  printf 'Usage: %s [tx|rx|rc|both|all]\n' "$0"
}

build_one() {
  case "$1" in
    tx) pio run -e tx_lora_pico ;;
    rx) pio run -e rx_lora_pico ;;
    rc) pio run -e rc-rp2350 -j1 || pio run -e rc-rp2350 -j1 ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
}

target="${1:-both}"
case "${target}" in
  both)
    build_one tx
    build_one rx
    ;;
  all)
    build_one tx
    build_one rx
    build_one rc
    ;;
  tx|rx|rc)
    build_one "${target}"
    ;;
  -h|--help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
