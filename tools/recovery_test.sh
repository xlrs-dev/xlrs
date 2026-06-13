#!/bin/bash
# Capture RX while power-cycling the TX mid-run to measure reacquisition time.
OUT=/tmp/xlrs_recovery.txt
P1=/dev/cu.usbmodem101
P2=/dev/cu.usbmodem1101
for P in "$P1" "$P2"; do
  stty -f "$P" 115200 cs8 -cstopb -parenb -ixon -ixoff raw -echo 2>/dev/null
done
python3 /Users/ishan/dev/quoppo/dsi/rp2040-ppm-tx-rx/tools/capture.py "$P1" "$P2" 22 > "$OUT" 2>&1 &
CAP_PID=$!
sleep 5
echo ">>> TX -> BOOTSEL (off air) <<<"
picotool reboot -u -f --ser E46488B28B561830 2>&1
sleep 4
echo ">>> TX -> application (back on air) <<<"
picotool reboot -a 2>&1
wait $CAP_PID
echo "=== capture done ==="
