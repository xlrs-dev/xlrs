#!/bin/bash
cd /Users/ishan/dev/quoppo/dsi/rp2040-ppm-tx-rx/build
echo "--- reboot RX to bootsel ---"
picotool reboot -u -f --ser E4654C16430F4223 2>&1
sleep 3
echo "--- flash RX ---"
picotool load -x xlrs_rx.uf2 2>&1 | tail -2
sleep 4
echo "--- ports ---"
ls /dev/cu.usbmodem* 2>&1
