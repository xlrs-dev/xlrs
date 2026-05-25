#!/usr/bin/env bash
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"

OUTDIR="${1:-$REPO/tools/bench-capture-tickanchor}"
BENCH_RATE="${2:-}"
mkdir -p "$OUTDIR"

CMAKE_BENCH_ARGS=(-DXLRS_BENCH_TX=ON)
if [[ -n "$BENCH_RATE" ]]; then
  CMAKE_BENCH_ARGS+=(-DXLRS_BENCH_RATE="$BENCH_RATE")
fi

bash scripts/test.sh 2>&1 | tail -3
cmake -S "$REPO" -B "$REPO/build" -G Ninja "${CMAKE_BENCH_ARGS[@]}" 2>&1 | tail -3
cmake --build build --target xlrs_tx xlrs_rx 2>&1 | tail -3
picotool load -x -f --ser E4654C16430F4223 "$REPO/build/xlrs_tx.elf"
picotool load -x -f --ser E4654C16432C3B22 "$REPO/build/xlrs_rx.elf"
sleep 20

# Drain boot spew so capture starts from a clean serial window.
python3 - <<'PY'
import os, time
for port in ("/dev/cu.usbmodem1201", "/dev/cu.usbmodem1101"):
    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        end = time.time() + 2.0
        while time.time() < end:
            try:
                os.read(fd, 8192)
            except BlockingIOError:
                time.sleep(0.05)
        os.close(fd)
    except OSError:
        pass
PY

# Wait for RX Connected (State 3) before measuring steady-state LQ.
python3 - <<'PY'
import os, sys, termios, time, select

def wait_state3(port, label, timeout_s=90):
    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as e:
        print(f"{label}: open failed: {e}", file=sys.stderr)
        return False
    attrs = termios.tcgetattr(fd)
    attrs[4] = attrs[5] = termios.B115200
    attrs[2] = (attrs[2] & ~termios.CSIZE) | termios.CS8
    attrs[2] &= ~(termios.PARENB | termios.CSTOPB)
    attrs[3] &= ~(termios.ECHO | termios.ICANON)
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    end = time.time() + timeout_s
    buf = b""
    ok = False
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.2)
        if r:
            try:
                chunk = os.read(fd, 4096)
                if chunk:
                    buf = (buf + chunk)[-8192:]
                    if b"State: 3" in buf and (label != "RX" or b"out:1" in buf):
                        ok = True
                        break
            except BlockingIOError:
                pass
    os.close(fd)
    print(f"{label}: {'Connected' if ok else 'timeout waiting for State 3'}")
    return ok

rx_ok = wait_state3("/dev/cu.usbmodem1101", "RX")
wait_state3("/dev/cu.usbmodem1201", "TX")
if not rx_ok:
    print("WARNING: continuing capture without confirmed RX connect", file=sys.stderr)
PY

python3 - "$OUTDIR" <<'PY'
import os, sys, termios, time, select, threading, re

outdir = sys.argv[1]

def capture(path, port, secs):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[4] = attrs[5] = termios.B115200
    attrs[2] = (attrs[2] & ~termios.CSIZE) | termios.CS8
    attrs[2] &= ~(termios.PARENB | termios.CSTOPB)
    attrs[3] &= ~(termios.ECHO | termios.ICANON)
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    end = time.time() + secs
    out = b""
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.2)
        if r:
            try:
                out += os.read(fd, 8192)
            except BlockingIOError:
                pass
    os.close(fd)
    open(path, "wb").write(out)

tx = os.path.join(outdir, "tx.log")
rx = os.path.join(outdir, "rx.log")
threads = [
    threading.Thread(target=capture, args=(tx, "/dev/cu.usbmodem1201", 180)),
    threading.Thread(target=capture, args=(rx, "/dev/cu.usbmodem1101", 180)),
]
for t in threads:
    t.start()
for t in threads:
    t.join()

for role, path in [("TX", tx), ("RX", rx)]:
    lines = [l for l in open(path).read().splitlines() if "STATUS" in l]
    key = r"LQdown: (\d+)%" if role == "TX" else r"LQ: (\d+)%"
    lq = [int(m.group(1)) for l in lines for m in [re.search(key, l)] if m]
    ge70 = sum(1 for v in lq if v >= 70)
    best = cur = 0
    for v in lq:
        cur = cur + 1 if v >= 70 else 0
        best = max(best, cur)
    states = {}
    for l in lines:
        m = re.search(r"State: (\d+)", l)
        if m:
            states[m.group(1)] = states.get(m.group(1), 0) + 1
    print(f"{role}: max={max(lq) if lq else 0}% >=70={ge70}/{len(lq)} best_run={best} states={states}")
    if role == "RX" and lines:
        for label, pat in [("crsf_rc", r"crsf_rc:(\d+)"), ("pfd", r"pfd:(-?\d+)us"), ("n", r"n:(\d+)")]:
            m = re.search(pat, lines[-1])
            print(f"  {label}: {m.group(1) if m else '?'}")
    if role == "TX" and lines:
        m = re.search(r"tlmOk:(\d+)", lines[-1])
        print(f"  tlmOk: {m.group(1) if m else '?'}")
    if lq:
        idx = lq.index(max(lq))
        print(f"  peak: {lines[idx][:140]}")
PY
