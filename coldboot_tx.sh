#!/bin/bash
cd /Users/ishan/dev/quoppo/dsi/rp2040-ppm-tx-rx/build
echo "--- cold boot TX only (bootsel->app) ---"
picotool reboot -u -f --ser E46488B28B561830 2>&1 | tail -1
sleep 3
picotool reboot -a 2>&1 | tail -1
sleep 3
ls /dev/cu.usbmodem* 2>&1
