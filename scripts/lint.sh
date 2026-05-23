#!/usr/bin/env bash
# Run clang-tidy against the host-native test build.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${LINT_BUILD_DIR:-${repo_root}/build-tests}"
clang_tidy="${CLANG_TIDY:-}"
cmake_generator="${CMAKE_GENERATOR:-}"

if [[ -z "${clang_tidy}" ]]; then
  if command -v clang-tidy >/dev/null 2>&1; then
    clang_tidy="clang-tidy"
  elif [[ -x /opt/homebrew/opt/llvm/bin/clang-tidy ]]; then
    clang_tidy="/opt/homebrew/opt/llvm/bin/clang-tidy"
  elif [[ -x /usr/local/opt/llvm/bin/clang-tidy ]]; then
    clang_tidy="/usr/local/opt/llvm/bin/clang-tidy"
  else
    clang_tidy="clang-tidy"
  fi
fi

if ! command -v "${clang_tidy}" >/dev/null 2>&1; then
  printf 'clang-tidy not found: %s\n' "${clang_tidy}" >&2
  printf 'Install clang-tidy or set CLANG_TIDY=/path/to/clang-tidy.\n' >&2
  exit 127
fi

if ! command -v python3 >/dev/null 2>&1; then
  printf 'python3 not found; needed to filter compile_commands.json.\n' >&2
  exit 127
fi

cmake_args=(-S "${repo_root}/test" -B "${build_dir}" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON)
if [[ -n "${cmake_generator}" ]]; then
  cmake_args+=(-G "${cmake_generator}")
fi

cmake "${cmake_args[@]}"

if [[ ! -f "${build_dir}/compile_commands.json" ]]; then
  printf 'Missing compile database: %s/compile_commands.json\n' "${build_dir}" >&2
  exit 1
fi

tidy_dir="${build_dir}/clang-tidy"
mkdir -p "${tidy_dir}"

python3 - "${repo_root}" "${build_dir}/compile_commands.json" "${tidy_dir}/compile_commands.json" <<'PY'
import json
import pathlib
import sys

repo_root = pathlib.Path(sys.argv[1]).resolve()
input_path = pathlib.Path(sys.argv[2])
output_path = pathlib.Path(sys.argv[3])

entries = json.loads(input_path.read_text())
seen = set()
filtered = []

for entry in entries:
    source = pathlib.Path(entry["file"]).resolve()
    try:
        relative = source.relative_to(repo_root)
    except ValueError:
        continue

    if not relative.parts or relative.parts[0] not in {"lib", "test"}:
        continue
    if "_deps" in relative.parts:
        continue
    if source in seen:
        continue

    seen.add(source)
    filtered.append(entry)

output_path.write_text(json.dumps(filtered, indent=2) + "\n")
PY

files=()
while IFS= read -r file; do
  files+=("${file}")
done < <(python3 - "${tidy_dir}/compile_commands.json" <<'PY'
import json
import sys

for entry in json.load(open(sys.argv[1])):
    print(entry["file"])
PY
)

if [[ "${#files[@]}" -eq 0 ]]; then
  printf 'No source files found for clang-tidy.\n'
  exit 0
fi

args=(-p "${tidy_dir}")
if [[ "${LINT_FIX:-0}" == "1" ]]; then
  args+=(-fix)
fi

sdk_path="$(xcrun --show-sdk-path 2>/dev/null || true)"
if [[ -n "${sdk_path}" ]]; then
  args+=(--extra-arg=-isysroot --extra-arg="${sdk_path}")
fi

"${clang_tidy}" "${args[@]}" "${files[@]}"
