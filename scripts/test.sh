#!/usr/bin/env bash
# Configure, build, and run the native (host) XLRS test suite.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${TEST_BUILD_DIR:-${repo_root}/build-tests}"

cmake -S "${repo_root}/test" -B "${build_dir}"
cmake --build "${build_dir}"
ctest --test-dir "${build_dir}" --output-on-failure
