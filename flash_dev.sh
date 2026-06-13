#!/bin/bash
# usage: flash_dev.sh <uf2> [serial]
cd /Users/ishan/dev/quoppo/dsi/rp2040-ppm-tx-rx/build
UF2="$1"
SER="$2"
if [ -n "$SER" ]; then
  picotool load -x "$UF2" --ser "$SER" 2>&1
else
  picotool load -x "$UF2" 2>&1
fi
echo "EXIT:$?"
