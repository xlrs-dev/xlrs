#!/bin/bash
export PATH="/Users/ishan/dev/quoppo/dsi/toolchain/xpack-arm-none-eabi-gcc-13.3.1-1.1/bin:/usr/bin:/bin"
export PICO_SDK_PATH="/Users/ishan/.platformio/packages/framework-arduinopico/pico-sdk"
cd /Users/ishan/dev/quoppo/dsi/rp2040-ppm-tx-rx/build
/Users/ishan/.platformio/packages/tool-ninja/ninja xlrs_tx xlrs_rx -j4 > /tmp/build_both_out.txt 2>&1
echo "EXIT:$?"
tail -20 /tmp/build_both_out.txt
