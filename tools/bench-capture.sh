#!/usr/bin/env bash
# Canonical 5-minute bench LQ capture (D250, XLRS_BENCH_TX=ON).
# TX serial E4654C16432C3B22, RX serial E4654C16430F4223 (verified on this bench).
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
OUTDIR="${1:-$REPO/tools/bench-capture-main-run}"
CAPTURE_SECS="${CAPTURE_SECS:-300}"
TX_SER="${TX_SER:-E4654C16432C3B22}"
RX_SER="${RX_SER:-E4654C16430F4223}"

mkdir -p "$OUTDIR"

echo "== Tests =="
bash "$REPO/scripts/test.sh" 2>&1 | tail -3

echo "== Build (Debug XLRS_BENCH_TX) =="
cmake -S "$REPO" -B "$REPO/build" -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DXLRS_BENCH_TX=ON \
  -UXLRS_BENCH_RATE 2>&1 | tail -3
cmake --build "$REPO/build" --target xlrs_tx xlrs_rx 2>&1 | tail -3

echo "== Flash TX ($TX_SER) =="
picotool load -x -f --ser "$TX_SER" "$REPO/build/xlrs_tx.elf"
sleep 25
echo "== Flash RX ($RX_SER) =="
picotool load -x -f --ser "$RX_SER" "$REPO/build/xlrs_rx.elf"
sleep 20

python3 "$REPO/tools/bench-capture-once.py" "$OUTDIR"
