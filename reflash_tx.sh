#!/bin/bash
cd /Users/ishan/dev/quoppo/dsi/rp2040-ppm-tx-rx/build
echo "--- reboot TX to bootsel ---"
picotool reboot -u -f --ser E46488B28B561830 2>&1
sleep 3
echo "--- flash TX ---"
picotool load -x xlrs_tx.uf2 2>&1 | tail -2
sleep 4
echo "--- ports ---"
ls /dev/cu.usbmodem* 2>&1
