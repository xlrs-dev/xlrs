#!/usr/bin/env python3
"""Capture TX/RX STATUS while putting TX into BOOTSEL mid-run."""
from __future__ import annotations

import glob
import os
import re
import select
import subprocess
import sys
import termios
import threading
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TX_SER = os.environ.get("TX_SER", "E4654C16432C3B22")
RX_SER = os.environ.get("RX_SER", "E4654C16430F4223")
PRE_SECS = float(os.environ.get("PRE_SECS", "15"))
POST_SECS = float(os.environ.get("POST_SECS", "60"))
TX_BOOT_AT = float(os.environ.get("TX_BOOT_AT", "15"))


def open_serial(port: str) -> int:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[4] = attrs[5] = termios.B115200
    attrs[2] = (attrs[2] & ~termios.CSIZE) | termios.CS8
    attrs[2] &= ~(termios.PARENB | termios.CSTOPB)
    attrs[3] &= ~(termios.ECHO | termios.ICANON)
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def discover_roles() -> tuple[str, str]:
    tx_port = rx_port = ""
    for port in sorted(glob.glob("/dev/cu.usbmodem*")):
        if "0x80000001" in port:
            continue
        try:
            fd = open_serial(port)
        except OSError:
            continue
        end = time.time() + 2.5
        buf = b""
        while time.time() < end:
            r, _, _ = select.select([fd], [], [], 0.2)
            if r:
                try:
                    buf += os.read(fd, 4096)
                except BlockingIOError:
                    pass
        os.close(fd)
        if b"[TX STATUS]" in buf:
            tx_port = port
        elif b"[RX STATUS]" in buf:
            rx_port = port
    return tx_port, rx_port


def capture(port: str, label: str, path: str, stop: threading.Event, t0: float) -> None:
    fd = open_serial(port)
    lines: list[str] = []
    with open(path, "w") as out:
        while not stop.is_set():
            r, _, _ = select.select([fd], [], [], 0.2)
            if not r:
                continue
            try:
                data = os.read(fd, 8192).decode("utf-8", "replace")
            except BlockingIOError:
                continue
            for raw in data.splitlines():
                if "STATUS" not in raw:
                    continue
                ts = time.time() - t0
                line = f"[{ts:7.1f}s] [{label}] {raw.strip()}\n"
                out.write(line)
                out.flush()
                lines.append(line)
    os.close(fd)


def summarize(path: str, label: str, boot_at: float) -> dict:
    pre, post = [], []
    states_pre: dict[str, int] = {}
    states_post: dict[str, int] = {}
    with open(path) as f:
        for line in f:
            m = re.match(r"\[(\d+\.\d+)s\]", line)
            if not m:
                continue
            t = float(m.group(1))
            bucket = pre if t < boot_at else post
            bucket.append(line)
            sm = re.search(r"State: (\d+)", line)
            if sm:
                st = sm.group(1)
                if t < boot_at:
                    states_pre[st] = states_pre.get(st, 0) + 1
                else:
                    states_post[st] = states_post.get(st, 0) + 1

    def rx_stats(lines: list[str]) -> dict:
        out1 = sum(1 for ln in lines if "out:1" in ln)
        crsf = []
        ages = []
        for ln in lines:
            if m := re.search(r"crsf_rc:(\d+)", ln):
                crsf.append(int(m.group(1)))
            if m := re.search(r"age:(\d+)ms", ln):
                ages.append(int(m.group(1)))
        return {
            "lines": len(lines),
            "out1": out1,
            "crsf_increments": (crsf[-1] - crsf[0]) if len(crsf) >= 2 else 0,
            "last_crsf": crsf[-1] if crsf else None,
            "max_age_ms": max(ages) if ages else None,
        }

    info = {
        "label": label,
        "pre_states": states_pre,
        "post_states": states_post,
        "pre_lines": len(pre),
        "post_lines": len(post),
    }
    if label == "RX":
        info["pre_rx"] = rx_stats(pre)
        info["post_rx"] = rx_stats(post)
        if pre:
            info["last_pre"] = pre[-1].strip()
        if post:
            info["last_post"] = post[-1].strip()
    return info


def main() -> int:
    out_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        REPO, "tools", "test-run-logs", f"hw-disconnect-{time.strftime('%Y%m%d-%H%M%S')}"
    )
    os.makedirs(out_dir, exist_ok=True)

    tx_port, rx_port = discover_roles()
    if not tx_port or not rx_port:
        print(f"Could not find TX/RX ports (tx={tx_port!r} rx={rx_port!r})", file=sys.stderr)
        return 2
    print(f"TX: {tx_port}  RX: {rx_port}")
    print(f"Output: {out_dir}")

    total = PRE_SECS + POST_SECS
    stop = threading.Event()
    t0 = time.time()
    tx_path = os.path.join(out_dir, "tx.log")
    rx_path = os.path.join(out_dir, "rx.log")
    markers_path = os.path.join(out_dir, "markers.txt")

    threads = [
        threading.Thread(target=capture, args=(tx_port, "TX", tx_path, stop, t0)),
        threading.Thread(target=capture, args=(rx_port, "RX", rx_path, stop, t0)),
    ]
    for t in threads:
        t.start()

    # Wait for link up
    print(f"Waiting {PRE_SECS}s for stable link...")
    time.sleep(PRE_SECS)

    with open(markers_path, "w") as mk:
        mk.write(f"tx_boot_at_s: {TX_BOOT_AT}\n")
        mk.write(f"command: picotool reboot --ser {TX_SER} -f -u\n")
    print(f"Putting TX into BOOTSEL at t={TX_BOOT_AT}s...")
    boot_t = time.time()
    proc = subprocess.run(
        ["picotool", "reboot", "--ser", TX_SER, "-f", "-u"],
        capture_output=True,
        text=True,
    )
    with open(markers_path, "a") as mk:
        mk.write(f"boot_elapsed_s: {time.time() - t0:.1f}\n")
        mk.write(f"exit_code: {proc.returncode}\n")
        if proc.stdout:
            mk.write(f"stdout: {proc.stdout.strip()}\n")
        if proc.stderr:
            mk.write(f"stderr: {proc.stderr.strip()}\n")

    remaining = total - (time.time() - t0)
    if remaining > 0:
        print(f"Observing RX for {remaining:.0f}s more...")
        time.sleep(remaining)

    stop.set()
    for t in threads:
        t.join(timeout=5)

    tx_sum = summarize(tx_path, "TX", TX_BOOT_AT)
    rx_sum = summarize(rx_path, "RX", TX_BOOT_AT)

    result_path = os.path.join(out_dir, "result.txt")
    with open(result_path, "w") as f:
        f.write(f"tx_port: {tx_port}\n")
        f.write(f"rx_port: {rx_port}\n")
        f.write(f"tx_boot_at_s: {TX_BOOT_AT}\n")
        f.write(f"capture_total_s: {total}\n")
        f.write(f"TX summary: {tx_sum}\n")
        f.write(f"RX summary: {rx_sum}\n")

        post_s3 = rx_sum["post_states"].get("3", 0)
        post_s4 = rx_sum["post_states"].get("4", 0)
        post_total = sum(rx_sum["post_states"].values()) or 1
        detected = post_s4 > 0 or rx_sum["post_rx"].get("out1", 0) == 0
        f.write(f"rx_detected_tx_loss: {detected}\n")
        f.write(f"post_boot_rx_state3_pct: {100 * post_s3 // post_total}%\n")

    print("\n=== TX disconnect test ===")
    print(f"picotool exit: {proc.returncode}")
    print(f"RX pre-boot states:  {rx_sum['pre_states']}")
    print(f"RX post-boot states: {rx_sum['post_states']}")
    print(f"RX post-boot out:1 count: {rx_sum['post_rx'].get('out1', 0)}/{rx_sum['post_lines']}")
    print(f"RX post-boot crsf delta: {rx_sum['post_rx'].get('crsf_increments', 0)}")
    if rx_sum.get("last_post"):
        print(f"RX last post-boot: {rx_sum['last_post'][:200]}")
    print(f"Wrote {result_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
