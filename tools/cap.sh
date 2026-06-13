#!/bin/bash
# cap.sh <dur_seconds> <outfile>
DUR="${1:-12}"
OUT="${2:-/tmp/xlrs_cap.txt}"
P1=/dev/cu.usbmodem101
P2=/dev/cu.usbmodem1101
for P in "$P1" "$P2"; do
  stty -f "$P" 115200 cs8 -cstopb -parenb -ixon -ixoff raw -echo 2>/dev/null
done
python3 /Users/ishan/dev/quoppo/dsi/rp2040-ppm-tx-rx/tools/capture.py "$P1" "$P2" "$DUR" | tee "$OUT"
