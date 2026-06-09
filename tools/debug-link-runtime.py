#!/usr/bin/env python3
"""Capture runtime link evidence for Cursor debug mode."""
from __future__ import annotations

import json
import re
import serial
import struct
import time
from pathlib import Path

LOG_PATH = Path("/Users/ishan/.codex/worktrees/1455/rp2040-ppm-tx-rx/.cursor/debug-9e1c8a.log")
SESSION_ID = "9e1c8a"
DEFAULT_TUPLE = (1500, 1500, 1500, 885, 1000, 1000, 1000, 1500)
PORTS = {
    "rx": "/dev/cu.usbmodem5",
    "tx": "/dev/cu.usbmodem1101",
    "rc": "/dev/cu.usbmodem14101",
    "fc": "/dev/cu.usbmodem0x80000001",
}
MSP_RC = 105


def log_event(run_id: str, hypothesis_id: str, location: str, message: str, data: dict) -> None:
    # region agent log
    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "sessionId": SESSION_ID,
        "runId": run_id,
        "hypothesisId": hypothesis_id,
        "location": location,
        "message": message,
        "data": data,
        "timestamp": int(time.time() * 1000),
    }
    with LOG_PATH.open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(payload, separators=(",", ":")) + "\n")
    # endregion


def query_status(port: str) -> str:
    try:
        with serial.Serial(port, 115200, timeout=0.2) as ser:
            time.sleep(0.25)
            ser.reset_input_buffer()
            ser.write(b"status\n")
            ser.flush()
            time.sleep(0.35)
            return ser.read(4096).decode("utf-8", "replace").strip()
    except Exception as exc:  # noqa: BLE001 - diagnostic script
        return f"ERROR {exc!r}"


def parse_status(text: str) -> dict:
    out: dict[str, int | str] = {"raw": text}
    for key in (
        "lq",
        "good",
        "bad",
        "ticks",
        "nonce",
        "fhss",
        "rx_done",
        "rf_miss",
        "phase_us",
        "tim",
        "off",
        "dx",
        "trim",
        "sync_ok",
        "rc_no_sync",
        "pfd_ext",
        "pfd_int",
        "pfd_res",
        "pshift",
        "last_shift",
        "lost",
        "tent_lost",
        "conn",
        "lock",
        "prom",
        "prom_fr",
        "prom_off",
        "prom_dx",
        "max_rc",
        "drops",
        "hold",
        "fc",
        "fc_gap_us",
        "tx_done",
        "tx_ok",
        "tx_try",
        "rc_failsafe",
        "crsf_age",
    ):
        match = re.search(rf"\b{key}=(-?\d+)", text)
        if match:
            out[key] = int(match.group(1))
    return out


def msp_req(cmd: int) -> bytes:
    size = 0
    return b"$M<" + bytes([size, cmd, size ^ cmd])


def read_msp_response(ser: serial.Serial, want: int, timeout: float = 0.12) -> bytes | None:
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        chunk = ser.read(64)
        if chunk:
            buf.extend(chunk)
        idx = buf.find(b"$M>")
        if idx >= 0 and len(buf) >= idx + 5:
            size = buf[idx + 3]
            cmd = buf[idx + 4]
            total = idx + 6 + size
            if len(buf) >= total:
                payload = bytes(buf[idx + 5 : idx + 5 + size])
                csum = buf[idx + 5 + size]
                calc = size ^ cmd
                for byte in payload:
                    calc ^= byte
                if cmd == want and calc == csum:
                    return payload
                buf = buf[idx + 1 :]
    return None


def capture_msp(run_id: str, seconds: float) -> dict:
    samples = 0
    drops = 0
    no_rc = 0
    transitions = 0
    last_defaultish: bool | None = None
    first_drops: list[dict] = []
    with serial.Serial(PORTS["fc"], 115200, timeout=0.005) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()
        start = time.time()
        while time.time() - start < seconds:
            elapsed = time.time() - start
            ser.write(msp_req(MSP_RC))
            ser.flush()
            payload = read_msp_response(ser, MSP_RC)
            if payload and len(payload) >= 8:
                channels = struct.unpack("<" + "H" * (len(payload) // 2), payload[: (len(payload) // 2) * 2])[:8]
                samples += 1
                defaultish = (
                    channels == DEFAULT_TUPLE
                    or channels[2] < 950
                    or (900 <= channels[0] <= 1100 and 900 <= channels[1] <= 1100 and channels[2] < 1100)
                )
            else:
                channels = None
                defaultish = True
                no_rc += 1
            if defaultish:
                drops += 1
                if len(first_drops) < 12:
                    first_drops.append({"t": round(elapsed, 3), "channels": channels})
            if last_defaultish is not None and defaultish != last_defaultish:
                transitions += 1
                log_event(
                    run_id,
                    "H5",
                    "tools/debug-link-runtime.py:capture_msp",
                    "msp default/live transition",
                    {"t": round(elapsed, 3), "defaultish": defaultish, "channels": channels},
                )
            last_defaultish = defaultish
            time.sleep(0.02)
    return {"samples": samples, "drops": drops, "noRc": no_rc, "transitions": transitions, "firstDrops": first_drops}


def main() -> int:
    run_id = f"run-{int(time.time())}"
    seconds = 60.0
    pre = {role: parse_status(query_status(port)) for role, port in PORTS.items() if role != "fc"}
    log_event(run_id, "H1,H2,H3,H4,H5", "tools/debug-link-runtime.py:main", "pre bench firmware status", pre)
    summary = capture_msp(run_id, seconds)
    log_event(run_id, "H5", "tools/debug-link-runtime.py:main", "betaflight msp summary", summary)
    post = {role: parse_status(query_status(port)) for role, port in PORTS.items() if role != "fc"}
    log_event(run_id, "H1,H2,H3,H4,H5", "tools/debug-link-runtime.py:main", "post bench firmware status", post)
    print(json.dumps({"runId": run_id, "msp": summary, "pre": pre, "post": post}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
