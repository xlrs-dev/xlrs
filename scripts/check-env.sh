#!/usr/bin/env bash
set -u

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
default_sdk="/Users/sawan/projects/pico/pico-sdk"
default_toolchain="/Users/sawan/projects/pico/toolchain/xpack-arm-none-eabi"

failures=0
warnings=0

ok() {
  printf '[OK]   %s\n' "$1"
}

warn() {
  warnings=$((warnings + 1))
  printf '[WARN] %s\n' "$1"
}

fail() {
  failures=$((failures + 1))
  printf '[FAIL] %s\n' "$1"
}

have_command() {
  command -v "$1" >/dev/null 2>&1
}

print_version() {
  "$@" 2>/dev/null | head -n 1
}

printf 'XLRS build environment check\n'
printf 'Repo: %s\n\n' "${repo_root}"

if have_command cmake; then
  ok "CMake found: $(command -v cmake) ($(print_version cmake --version))"
else
  fail "CMake is not installed or not on PATH."
fi

if have_command ninja; then
  ok "Ninja found: $(command -v ninja) ($(print_version ninja --version))"
else
  fail "Ninja is not installed or not on PATH."
fi

if have_command python3; then
  ok "Python 3 found: $(command -v python3) ($(print_version python3 --version))"
else
  fail "python3 is not installed or not on PATH."
fi

clang_tidy_path=""
if have_command clang-tidy; then
  clang_tidy_path="$(command -v clang-tidy)"
elif [[ -x /opt/homebrew/opt/llvm/bin/clang-tidy ]]; then
  clang_tidy_path="/opt/homebrew/opt/llvm/bin/clang-tidy"
elif [[ -x /usr/local/opt/llvm/bin/clang-tidy ]]; then
  clang_tidy_path="/usr/local/opt/llvm/bin/clang-tidy"
fi

if [[ -n "${clang_tidy_path}" ]]; then
  ok "clang-tidy found: ${clang_tidy_path} ($("${clang_tidy_path}" --version | head -n 1))"
else
  warn "clang-tidy is not installed or not on PATH. Install LLVM for scripts/lint.sh (macOS: brew install llvm)."
fi

sdk_path="${PICO_SDK_PATH:-}"
if [[ -z "${sdk_path}" && -d "${default_sdk}" ]]; then
  sdk_path="${default_sdk}"
  ok "PICO_SDK_PATH not set; using local default ${sdk_path}"
elif [[ -n "${sdk_path}" ]]; then
  ok "PICO_SDK_PATH set: ${sdk_path}"
elif [[ "${PICO_SDK_FETCH_FROM_GIT:-}" == "ON" || "${PICO_SDK_FETCH_FROM_GIT:-}" == "1" ]]; then
  warn "PICO_SDK_PATH is not set; CMake will fetch the Pico SDK because PICO_SDK_FETCH_FROM_GIT is enabled."
else
  fail "PICO_SDK_PATH is not set and ${default_sdk} does not exist."
fi

if [[ -n "${sdk_path}" ]]; then
  if [[ -f "${sdk_path}/pico_sdk_init.cmake" ]]; then
    ok "Pico SDK import file found."
  else
    fail "${sdk_path} does not look like a Pico SDK checkout."
  fi

  for submodule in lib/tinyusb; do
    if [[ -d "${sdk_path}/${submodule}" ]]; then
      ok "Pico SDK submodule present: ${submodule}"
    else
      fail "Pico SDK submodule missing: ${submodule}. Run: git -C \"${sdk_path}\" submodule update --init --recursive"
    fi
  done
fi

toolchain_path="${PICO_TOOLCHAIN_PATH:-}"
gcc_path=""
if [[ -n "${toolchain_path}" ]]; then
  gcc_path="${toolchain_path}/bin/arm-none-eabi-gcc"
  ok "PICO_TOOLCHAIN_PATH set: ${toolchain_path}"
elif [[ -x "${default_toolchain}/bin/arm-none-eabi-gcc" ]]; then
  toolchain_path="${default_toolchain}"
  gcc_path="${toolchain_path}/bin/arm-none-eabi-gcc"
  ok "PICO_TOOLCHAIN_PATH not set; using local default ${toolchain_path}"
elif have_command arm-none-eabi-gcc; then
  gcc_path="$(command -v arm-none-eabi-gcc)"
  ok "arm-none-eabi-gcc found on PATH: ${gcc_path}"
else
  fail "Arm embedded GCC not found. Set PICO_TOOLCHAIN_PATH or install arm-none-eabi-gcc."
fi

if [[ -n "${gcc_path}" ]]; then
  if [[ -x "${gcc_path}" ]]; then
    ok "Arm GCC works: $(${gcc_path} --version | head -n 1)"
    specs_path="$("${gcc_path}" -print-file-name=nosys.specs 2>/dev/null || true)"
    if [[ -n "${specs_path}" && "${specs_path}" != "nosys.specs" && -f "${specs_path}" ]]; then
      ok "newlib nosys.specs found: ${specs_path}"
    else
      fail "Arm GCC is missing nosys.specs; Pico SDK linking will fail."
    fi
  else
    fail "arm-none-eabi-gcc is not executable at ${gcc_path}."
  fi
fi

if have_command picotool; then
  picotool_path="$(command -v picotool)"
  ok "picotool found: ${picotool_path} ($(print_version picotool version))"
  picotool_help="$(picotool help 2>/dev/null || true)"
  if printf '%s\n' "${picotool_help}" | grep -Eq '^[[:space:]]+load[[:space:]]'; then
    ok "picotool USB load support is available."
  else
    warn "picotool is present but does not expose USB load/reboot commands. Install a USB-capable picotool for flashing."
  fi
else
  warn "picotool is not on PATH. Build still works, but flashing with scripts/flash.sh requires a USB-capable picotool."
fi

if [[ -f "${repo_root}/CMakeLists.txt" ]]; then
  ok "Top-level CMakeLists.txt found."
else
  fail "Top-level CMakeLists.txt is missing."
fi

if [[ -f "${repo_root}/cmake/pico_sdk_import.cmake" ]]; then
  ok "Pico SDK CMake import helper found."
else
  fail "cmake/pico_sdk_import.cmake is missing."
fi

if [[ -d "${repo_root}/apps/tx" && -d "${repo_root}/apps/rx" ]]; then
  ok "TX/RX app entry directories found."
else
  fail "Expected apps/tx and apps/rx directories are missing."
fi

printf '\n'
if [[ "${failures}" -eq 0 ]]; then
  ok "Environment is ready for scripts/build.sh."
  if [[ -z "${PICO_SDK_PATH:-}" && -n "${sdk_path}" ]]; then
    printf 'Hint: export PICO_SDK_PATH=%q\n' "${sdk_path}"
  fi
  if [[ -z "${PICO_TOOLCHAIN_PATH:-}" && -n "${toolchain_path}" ]]; then
    printf 'Hint: export PICO_TOOLCHAIN_PATH=%q\n' "${toolchain_path}"
  fi
else
  fail "Environment is not ready: ${failures} failure(s), ${warnings} warning(s)."
  exit 1
fi
