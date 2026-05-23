#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${BUILD_DIR:-${repo_root}/build}"
pico_board="${PICO_BOARD:-pico}"

if [[ -z "${PICO_SDK_PATH:-}" && -d /Users/sawan/projects/pico/pico-sdk ]]; then
  export PICO_SDK_PATH=/Users/sawan/projects/pico/pico-sdk
fi

if [[ -z "${PICO_TOOLCHAIN_PATH:-}" && -x /Users/sawan/projects/pico/toolchain/xpack-arm-none-eabi/bin/arm-none-eabi-gcc ]]; then
  export PICO_TOOLCHAIN_PATH=/Users/sawan/projects/pico/toolchain/xpack-arm-none-eabi
fi

"${repo_root}/scripts/check-env.sh"

cmake_args=(-S "${repo_root}" -B "${build_dir}" -DPICO_BOARD="${pico_board}")
if [[ "${PICO_SDK_FETCH_FROM_GIT:-}" == "ON" || "${PICO_SDK_FETCH_FROM_GIT:-}" == "1" ]]; then
  cmake_args+=(-DPICO_SDK_FETCH_FROM_GIT=ON)
fi

cmake "${cmake_args[@]}"
cmake --build "${build_dir}" --target xlrs_tx xlrs_rx
