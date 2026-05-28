#!/usr/bin/env python3
"""Wait for TX/RX connect, capture STATUS logs, print summary with >=90% gate."""
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

CAPTURE_SECS = int(os.environ.get("CAPTURE_SECS", "300"))
WAIT_SECS = int(os.environ.get("WAIT_SECS", "240"))
SETTLE_SECS = int(os.environ.get("SETTLE_SECS", "30"))
TX_SER = os.environ.get("TX_SER", "E4654C16432C3B22")
RX_SER = os.environ.get("RX_SER", "E4654C16430F4223")
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def open_serial(port: str) -> int:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[4] = attrs[5] = termios.B115200
    attrs[2] = (attrs[2] & ~termios.CSIZE) | termios.CS8
    attrs[2] &= ~(termios.PARENB | termios.CSTOPB)
    attrs[3] &= ~(termios.ECHO | termios.ICANON)
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def drain_port(port: str, secs: float = 2.0) -> None:
    try:
        fd = open_serial(port)
    except OSError:
        return
    end = time.time() + secs
    while time.time() < end:
        readable, _, _ = select.select([fd], [], [], 0.1)
        if readable:
            try:
                os.read(fd, 8192)
            except BlockingIOError:
                pass
    os.close(fd)


def discover_roles() -> tuple[str, str]:
    tx_port = rx_port = ""
    for port in sorted(glob.glob("/dev/cu.usbmodem*")):
        try:
            fd = open_serial(port)
        except OSError:
            continue
        end = time.time() + 3.0
        buf = b""
        while time.time() < end:
            readable, _, _ = select.select([fd], [], [], 0.2)
            if readable:
                try:
                    buf += os.read(fd, 4096)
                except BlockingIOError:
                    pass
        os.close(fd)
        if b"[TX STATUS]" in buf:
            tx_port = port
        elif b"[RX STATUS]" in buf:
            rx_port = port
    if not tx_port or not rx_port:
        raise RuntimeError(f"Could not map TX/RX ports (tx={tx_port!r}, rx={rx_port!r})")
    return tx_port, rx_port


def status_line_ok(line: bytes, label: str) -> bool:
    if b"STATUS" not in line or b"State: 3" not in line:
        return False
    if label == "RX" and b"out:1" not in line:
        return False
    return True


def wait_state3(port: str, label: str, timeout_s: int = WAIT_SECS) -> bool:
    try:
        fd = open_serial(port)
    except OSError as exc:
        print(f"{label}: open failed: {exc}", file=sys.stderr)
        return False
    end = time.time() + timeout_s
    buf = b""
    ok = False
    last_line = b""
    while time.time() < end:
        readable, _, _ = select.select([fd], [], [], 0.2)
        if readable:
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                continue
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if status_line_ok(line, label):
                        ok = True
                        last_line = line
                        break
                if ok:
                    break
    os.close(fd)
    print(f"{label} ({port}): {'Connected' if ok else 'timeout waiting for State 3'}")
    if ok and last_line:
        print(f"  {label} line: {last_line.decode(errors='replace')[:160]}")
    return ok


def reboot_boards() -> None:
    print("Rebooting both boards...", flush=True)
    for ser in (TX_SER, RX_SER):
        subprocess.run(
            ["picotool", "reboot", "-f", "--ser", ser],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    time.sleep(20)


def ensure_connected(tx_port: str, rx_port: str) -> bool:
    for attempt in range(1, 4):
        print(f"Pair attempt {attempt}/3", flush=True)
        drain_port(rx_port)
        drain_port(tx_port)
        time.sleep(12)
        rx_ok = wait_state3(rx_port, "RX", WAIT_SECS)
        tx_ok = wait_state3(tx_port, "TX", WAIT_SECS)
        if rx_ok and tx_ok:
            print(f"Settling {SETTLE_SECS}s after connect...", flush=True)
            time.sleep(SETTLE_SECS)
            return True
        print("Not connected; rebooting...", flush=True)
        reboot_boards()
        tx_port, rx_port = discover_roles()
    return False


def capture(path: str, port: str, secs: int) -> None:
    fd = open_serial(port)
    end = time.time() + secs
    out = b""
    while time.time() < end:
        readable, _, _ = select.select([fd], [], [], 0.2)
        if readable:
            try:
                out += os.read(fd, 8192)
            except BlockingIOError:
                pass
    os.close(fd)
    with open(path, "wb") as fh:
        fh.write(out)


def summarize(role: str, path: str, key: str) -> dict:
    lines = [line for line in open(path).read().splitlines() if "STATUS" in line]
    lq = [int(match.group(1)) for line in lines for match in [re.search(key, line)] if match]
    ge70 = sum(1 for value in lq if value >= 70)
    ge90 = sum(1 for value in lq if value >= 90)
    states: dict[str, int] = {}
    prev = None
    drops_34 = rec_43 = 0
    for line in lines:
        match = re.search(r"State: (\d+)", line)
        if not match:
            continue
        state = match.group(1)
        states[state] = states.get(state, 0) + 1
        if prev == "3" and state == "4":
            drops_34 += 1
        if prev == "4" and state == "3":
            rec_43 += 1
        prev = state
    avg = round(sum(lq) / len(lq), 1) if lq else 0.0
    row = {
        "samples": len(lq),
        "avg": avg,
        "max": max(lq) if lq else 0,
        "ge70": ge70,
        "ge90": ge90,
        "states": states,
        "drops_34": drops_34,
        "rec_43": rec_43,
    }
    print(
        f"{role}: samples={row['samples']} avg={avg}% max={row['max']}% "
        f">=70={ge70}/{len(lq)} >=90={ge90}/{len(lq)} "
        f"states={states} 3->4={drops_34} 4->3={rec_43}"
    )
    return row


def main() -> int:
    outdir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(REPO, "tools/bench-capture-main-run")
    os.makedirs(outdir, exist_ok=True)

    tx_port, rx_port = discover_roles()
    print(f"TX port: {tx_port}")
    print(f"RX port: {rx_port}")

    if not ensure_connected(tx_port, rx_port):
        print("ERROR: could not establish TX+RX State 3", file=sys.stderr)
        return 1

    tx_path = os.path.join(outdir, "tx.log")
    rx_path = os.path.join(outdir, "rx.log")
    print(f"Capturing {CAPTURE_SECS}s...", flush=True)
    threads = [
        threading.Thread(target=capture, args=(tx_path, tx_port, CAPTURE_SECS)),
        threading.Thread(target=capture, args=(rx_path, rx_port, CAPTURE_SECS)),
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    tx = summarize("TX", tx_path, r"LQdown: (\d+)%")
    rx = summarize("RX", rx_path, r"LQ: (\d+)%")

    rx_pass = rx["ge90"] >= int(rx["samples"] * 0.9) if rx["samples"] else False
    tx_pass = tx["ge90"] >= int(tx["samples"] * 0.9) if tx["samples"] else False
    s3_rx = rx["states"].get("3", 0)
    s3_tx = tx["states"].get("3", 0)
    state_pass = (
        s3_rx >= int(rx["samples"] * 0.95)
        and s3_tx >= int(tx["samples"] * 0.95)
        if rx["samples"] and tx["samples"]
        else False
    )
    passed = rx_pass and tx_pass and state_pass

    commit = subprocess.check_output(["git", "-C", REPO, "rev-parse", "--short", "HEAD"], text=True).strip()
    branch = subprocess.check_output(["git", "-C", REPO, "branch", "--show-current"], text=True).strip()
    result_path = os.path.join(outdir, "result.txt")
    with open(result_path, "w") as fh:
        fh.write(f"branch: {branch}\n")
        fh.write(f"commit: {commit}\n")
        fh.write(f"capture_secs: {CAPTURE_SECS}\n")
        fh.write(f"tx_port: {tx_port}\n")
        fh.write(f"rx_port: {rx_port}\n")
        fh.write(f"TX: {tx}\n")
        fh.write(f"RX: {rx}\n")
        fh.write(f"pass_90pct: {passed}\n")

    print(f"PASS (>90% LQ both sides): {passed}")
    print(f"Wrote {result_path}")
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
