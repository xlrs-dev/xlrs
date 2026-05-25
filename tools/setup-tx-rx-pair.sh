#!/usr/bin/env bash
# Flash TX/RX pair (D250, Kikobot-02) and verify RF link stability.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"

TX_SER="${TX_SER:-E4654C16432C3B22}"
RX_SER="${RX_SER:-E4654C16430F4223}"
WAIT_RX_USB_SEC="${WAIT_RX_USB_SEC:-300}"
LINK_WAIT_SEC="${LINK_WAIT_SEC:-180}"
# Production TX (handset / AIO path). Set BENCH=1 for XLRS_BENCH_TX midstick captures.
BENCH="${BENCH:-0}"

if [[ "$BENCH" == "1" ]]; then
  CMAKE_ARGS=(-DXLRS_BENCH_TX=ON -DXLRS_BENCH_RATE=2)
else
  CMAKE_ARGS=(-DXLRS_BENCH_TX=OFF -UXLRS_BENCH_RATE)
fi

echo "== Build TX/RX (D250) bench=${BENCH} =="
bash scripts/test.sh 2>&1 | tail -2
cmake -S "$REPO" -B "$REPO/build" -G Ninja "${CMAKE_ARGS[@]}" 2>&1 | tail -2
cmake --build build --target xlrs_tx xlrs_rx 2>&1 | tail -2

usb_serial_present() {
  ioreg -p IOUSB -l 2>/dev/null | grep -q "$1"
}

flash_if_present() {
  local role="$1" ser="$2" elf="$3"
  if usb_serial_present "$ser"; then
    echo "Flashing ${role} (${ser})..."
    picotool load -x -f --ser "$ser" "$elf"
    return 0
  fi
  return 1
}

echo "== Flash TX (${TX_SER}) =="
if ! flash_if_present TX "$TX_SER" "$REPO/build/xlrs_tx.elf"; then
  echo "ERROR: TX not on USB (serial ${TX_SER})" >&2
  exit 1
fi

echo "== Wait for RX USB (${RX_SER}), up to ${WAIT_RX_USB_SEC}s =="
rx_flashed=0
for ((i = 0; i < WAIT_RX_USB_SEC; i += 2)); do
  if usb_serial_present "$RX_SER"; then
    echo "RX detected — flashing..."
    picotool load -x -f --ser "$RX_SER" "$REPO/build/xlrs_rx.elf"
    rx_flashed=1
    break
  fi
  sleep 2
done

if [[ "$rx_flashed" == "0" ]]; then
  echo "WARNING: RX not on USB — flash skipped. Power RX and re-run." >&2
else
  echo "RX flashed; waiting 20s for boot..."
  sleep 20
fi

python3 - "$LINK_WAIT_SEC" <<'PY'
import os, sys, termios, time, select, re, threading
from collections import Counter

cap = int(sys.argv[1])
ports = sorted(p for p in os.listdir("/dev") if p.startswith("cu.usbmodem") and "0x80000001" not in p)
ports = ["/dev/" + p for p in ports]
if not ports:
    print("No usbmodem ports")
    sys.exit(1)

def sniff(port, out):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[4] = attrs[5] = termios.B115200
    attrs[2] = (attrs[2] & ~termios.CSIZE) | termios.CS8
    attrs[2] &= ~(termios.PARENB | termios.CSTOPB)
    attrs[3] &= ~(termios.ECHO | termios.ICANON)
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    end = time.time() + cap
    tx, rx = [], []
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.2)
        if not r:
            continue
        try:
            data = os.read(fd, 8192).decode("utf-8", "replace")
        except BlockingIOError:
            continue
        for line in data.splitlines():
            if "[TX STATUS]" in line:
                tx.append(line.strip())
            if "[RX STATUS]" in line:
                rx.append(line.strip())
    os.close(fd)
    out["tx"], out["rx"] = tx, rx

def report(role, lines):
    if not lines:
        return
    key = r"LQdown: (\d+)%" if role == "TX" else r"LQ: (\d+)%"
    lqs = [int(m.group(1)) for l in lines for m in [re.search(key, l)] if m]
    states = Counter(re.search(r"State: (\d+)", l).group(1) for l in lines if re.search(r"State: (\d+)", l))
    spurious = sum(1 for l in lines if "State: 4" in l and (m := re.search(key, l)) and int(m.group(1)) >= 25)
    print(f"{role}: states={dict(states)} max_lq={max(lqs) if lqs else 0} spurious_fs(lq>=25)={spurious}")
    print(f"  last: {lines[-1][:220]}")

results = {}
threads = []
for p in ports:
    results[p] = {}
    t = threading.Thread(target=sniff, args=(p, results[p]))
    t.start()
    threads.append(t)
for t in threads:
    t.join()

for p in ports:
    r = results.get(p, {})
    if r.get("tx"):
        print(f"\n{p}")
        report("TX", r["tx"])
    if r.get("rx"):
        if not r.get("tx"):
            print(f"\n{p}")
        report("RX", r["rx"])
PY
