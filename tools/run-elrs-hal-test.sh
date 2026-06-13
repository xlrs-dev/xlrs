#!/usr/bin/env bash
# Build, flash, and capture Phase 1 ELRS HAL proof harness output.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOG_PATH="$ROOT/.cursor/debug-40c48c.log"
BUILD_DIR="$ROOT/build"
ELF="$BUILD_DIR/elrs_hal_test.elf"
SERIAL="${1:-}"
BAUD="${BAUD:-115200}"

usage() {
  echo "Usage: $0 [SERIAL|PORT]"
  echo "  SERIAL  picotool --ser value (optional; flashes sole device if omitted)"
  echo "  PORT    USB CDC port to read (default: auto after flash)"
  echo ""
  echo "Examples:"
  echo "  $0 E4654C16430F4223"
  echo "  $0 /dev/cu.usbmodem5"
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

cmake -S "$ROOT" -B "$BUILD_DIR" -G Ninja -DXLRS_DEFAULT_BINDING_PHRASE="${XLRS_DEFAULT_BINDING_PHRASE:-Kikobot-02}"
cmake --build "$BUILD_DIR" --target elrs_hal_test

rm -f "$LOG_PATH"

flash_args=(-x -f)
if [[ -n "$SERIAL" && "$SERIAL" == /dev/* ]]; then
  PORT="$SERIAL"
elif [[ -n "$SERIAL" ]]; then
  flash_args+=(--ser "$SERIAL")
  PORT="${PORT:-}"
else
  PORT="${PORT:-}"
fi

picotool load "${flash_args[@]}" "$ELF"

if [[ -z "${PORT:-}" ]]; then
  sleep 2
  for p in /dev/cu.usbmodem*; do
    [[ -e "$p" ]] || continue
    PORT="$p"
    break
  done
fi

if [[ -z "${PORT:-}" || ! -e "$PORT" ]]; then
  echo "No serial port found; flash succeeded." >&2
  exit 1
fi

if [[ "$(uname -s)" == "Darwin" ]]; then
  stty -f "$PORT" "$BAUD" cs8 -cstopb -parenb raw -echo
else
  stty -F "$PORT" "$BAUD" cs8 -cstopb -parenb raw -echo
fi

echo "Reading $PORT for ELRS-HAL lines..."
lines=()
end=$((SECONDS + 12))
while (( SECONDS < end )); do
  if IFS= read -r -t 1 line <"$PORT"; then
    echo "$line"
    lines+=("$line")
    if [[ "$line" == *"[ELRS-HAL] summary"* ]]; then
      break
    fi
  fi
done

ts="$(python3 - <<'PY'
import time
print(int(time.time()*1000))
PY
)"

{
  printf '{"sessionId":"40c48c","timestamp":%s,"location":"tools/run-elrs-hal-test.sh","message":"hal_test_capture","runId":"phase1","hypothesisId":"HAL","data":{"port":"%s","lines":%s}}\n' \
    "$ts" "$PORT" "$(printf '%s\n' "${lines[@]}" | python3 -c 'import json,sys; print(json.dumps(sys.stdin.read().splitlines()))')"
} >>"$LOG_PATH"

pass_count=0
fail_count=0
for line in "${lines[@]}"; do
  if [[ "$line" == *" PASS "* ]]; then
    pass_count=$((pass_count + 1))
  elif [[ "$line" == *" FAIL "* ]]; then
    fail_count=$((fail_count + 1))
  fi
done

echo "Captured ${#lines[@]} lines → $LOG_PATH (pass=$pass_count fail=$fail_count)"
