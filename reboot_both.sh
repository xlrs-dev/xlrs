#!/bin/bash
cd /Users/ishan/dev/quoppo/dsi/rp2040-ppm-tx-rx/build
echo "--- reboot BOTH to app ---"
picotool reboot -a -f --ser E4654C16430F4223 2>&1 | tail -1
sleep 1
picotool reboot -a -f --ser E46488B28B561830 2>&1 | tail -1
sleep 4
ls /dev/cu.usbmodem* 2>&1
