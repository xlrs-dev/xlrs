#!/usr/bin/env python3
"""Fast link health check — fail fast if not stable in the first few seconds."""
from __future__ import annotations

import os
import re
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def analyze_log(log_path: str) -> tuple[list[str], list[str], dict, dict]:
    tx_lines = []
    rx_lines = []
    with open(log_path) as f:
        for line in f:
            if line.startswith("[TX]") and "State:" in line:
                tx_lines.append(line)
            elif line.startswith("[RX]") and "State:" in line:
                rx_lines.append(line)

    def summarize(lines: list[str], lq_key: str) -> dict:
        states: dict[str, int] = {}
        lq_vals: list[int] = []
        drops_34 = 0
        prev = None
        for ln in lines:
            m = re.search(r"State: (\d+)", ln)
            if m:
                st = m.group(1)
                states[st] = states.get(st, 0) + 1
                if prev == "3" and st == "4":
                    drops_34 += 1
                prev = st
            lm = re.search(rf"{lq_key}: (\d+)%", ln)
            if lm:
                lq_vals.append(int(lm.group(1)))
        n = len(lines) or 1
        s3 = states.get("3", 0)
        return {
            "samples": len(lines),
            "s3_pct": 100 * s3 // n,
            "s4": states.get("4", 0),
            "s2": states.get("2", 0),
            "drops_34": drops_34,
            "avg_lq": sum(lq_vals) // len(lq_vals) if lq_vals else 0,
            "states": states,
        }

    tx = summarize(tx_lines, "LQdown")
    rx = summarize(rx_lines, "LQ")
    return tx_lines, rx_lines, tx, rx


def verdict(min_ok: int, tx_lines: list[str], tx: dict, rx: dict, rx_lines: list[str]) -> bool:
    def rx_rc_fresh(line: str) -> bool:
        if "out:1" in line:
            return True
        m = re.search(r"crsf_rc:\d+ age:(\d+)ms", line)
        return bool(m and int(m.group(1)) <= 120)

    rx_out_active = 0
    if rx_lines:
        tail = rx_lines[-min(10, len(rx_lines)) :]
        rx_out_active = sum(1 for ln in tail if "out:1" in ln or rx_rc_fresh(ln))

    ok = (
        tx["s3_pct"] >= min_ok
        and rx["s3_pct"] >= min_ok
        and tx["s4"] == 0
        and rx["s4"] == 0
        and tx["drops_34"] == 0
        and rx["drops_34"] == 0
        and (not rx_lines or rx_out_active >= max(2, len(rx_lines[-min(10, len(rx_lines)) :]) // 2))
    )
    if rx_lines and ok:
        last = rx_lines[-1]
        if "State: 3" in last and "out:0" in last and not rx_rc_fresh(last):
            ok = False
            print("  RX last line: State 3, stale RC (out:0 and age>120ms)")

    return ok


def run_capture(secs: int, tx_port: str, rx_port: str, out_dir: str) -> bool:
    env = {**os.environ, "CAPTURE_SECS": str(secs), "TX_PORT": tx_port, "RX_PORT": rx_port}
    proc = subprocess.run(
        [sys.executable, os.path.join(REPO, "tools", "capture-status.py"), out_dir],
        env=env,
        cwd=REPO,
        capture_output=True,
        text=True,
    )
    print(proc.stdout, end="")
    if proc.stderr:
        print(proc.stderr, file=sys.stderr, end="")
    return os.path.isfile(os.path.join(out_dir, "capture.log"))


def main() -> int:
    secs = int(os.environ.get("CHECK_SECS", "20"))
    fail_fast = int(os.environ.get("FAIL_FAST_SECS", "5"))
    tx_port = os.environ.get("TX_PORT", "/dev/cu.usbmodem1101")
    rx_port = os.environ.get("RX_PORT", "/dev/cu.usbmodem6")
    min_ok = int(os.environ.get("MIN_OK_PCT", "90"))
    out_dir = os.path.join(REPO, "tools", "bench-quick-check")

    if fail_fast > 0 and secs > fail_fast:
        print(f"Fail-fast window ({fail_fast}s)...")
        if not run_capture(fail_fast, tx_port, rx_port, out_dir):
            print(f"FAIL: no capture log ({fail_fast}s)")
            return 2
        log_path = os.path.join(out_dir, "capture.log")
        tx_lines, rx_lines, tx, rx = analyze_log(log_path)
        ok = verdict(min_ok, tx_lines, tx, rx, rx_lines)
        print(f"\n{fail_fast}s summary (need >={min_ok}% State 3, 0x State 4, 0x 3->4):")
        print(f"  TX: {tx}")
        print(f"  RX: {rx}")
        print("PASS" if ok else "FAIL (bailing — link not stable in first few seconds)")
        if not ok:
            return 2
        print(f"\nContinuing full check ({secs}s)...")

    if not run_capture(secs, tx_port, rx_port, out_dir):
        print(f"FAIL: no capture log ({secs}s)")
        return 2

    log_path = os.path.join(out_dir, "capture.log")
    tx_lines, rx_lines, tx, rx = analyze_log(log_path)
    ok = verdict(min_ok, tx_lines, tx, rx, rx_lines)
    print(f"\n{secs}s summary (need >={min_ok}% State 3, 0x State 4, 0x 3->4):")
    print(f"  TX: {tx}")
    print(f"  RX: {rx}")
    print("PASS" if ok else "FAIL")
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
