#!/bin/bash
# Force a true cold boot of both modules via BOOTSEL (reboot -a from app proved to be a no-op).
cd /Users/ishan/dev/quoppo/dsi/rp2040-ppm-tx-rx/build
RX=E4654C16430F4223
TX=E46488B28B561830

echo "--- RX -> bootsel ---"
picotool reboot -u -f --ser $RX 2>&1 | tail -1
sleep 3
echo "--- RX -> app (from bootsel) ---"
picotool reboot -a 2>&1 | tail -1
sleep 3

echo "--- TX -> bootsel ---"
picotool reboot -u -f --ser $TX 2>&1 | tail -1
sleep 3
echo "--- TX -> app (from bootsel) ---"
picotool reboot -a 2>&1 | tail -1
sleep 4
ls /dev/cu.usbmodem* 2>&1
