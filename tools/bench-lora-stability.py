#!/usr/bin/env python3
"""LoRa bench stability run — up to 5 min, early abort after >2 degradations."""
from __future__ import annotations

import json
import os
import re
import sys
import threading
import time

SESSION_ID = "40c48c"
LOG_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    ".cursor",
    f"debug-{SESSION_ID}.log",
)

MAX_SECS = float(os.environ.get("STABILITY_SECS", "300"))
MAX_DEGRADATIONS = int(os.environ.get("MAX_DEGRADATIONS", "2"))
LQ_FLOOR = int(os.environ.get("LQ_FLOOR", "80"))
TX_PORT = os.environ.get("TX_PORT", "/dev/cu.usbmodem1101")
RX_PORT = os.environ.get("RX_PORT", "/dev/cu.usbmodem5")

STATE_CONNECTED = 3
STATE_CONNECTING = 2


def ndjson(location: str, message: str, data: dict, hypothesis_id: str = "", run_id: str = "baseline") -> None:
    rec = {
        "sessionId": SESSION_ID,
        "runId": run_id,
        "hypothesisId": hypothesis_id,
        "location": location,
        "message": message,
        "data": data,
        "timestamp": int(time.time() * 1000),
    }
    with open(LOG_PATH, "a") as f:
        f.write(json.dumps(rec) + "\n")


def parse_status(line: str) -> dict | None:
    m = re.search(r"State: (\d+)", line)
    if not m:
        return None
    st = int(m.group(1))
    lq = None
    for key in ("LQdown", "LQ"):
        lm = re.search(rf"{key}: (\d+)%", line)
        if lm:
            lq = int(lm.group(1))
            break
    tlm_ok = tlm_bad = sync_rej = None
    tm = re.search(r"tlmOk:(\d+) tlmBad:(\d+)", line)
    if tm:
        tlm_ok, tlm_bad = int(tm.group(1)), int(tm.group(2))
    sm = re.search(r"syncRej:(\d+)", line)
    if sm:
        sync_rej = int(sm.group(1))
    return {"state": st, "lq": lq, "tlmOk": tlm_ok, "tlmBad": tlm_bad, "syncRej": sync_rej}


class RoleTracker:
    def __init__(self, role: str) -> None:
        self.role = role
        self.prev_state: int | None = None
        self.degradations = 0
        self.connected_samples = 0
        self.total_samples = 0
        self.last_lq: int | None = None

    def update(self, info: dict, rel_s: float) -> list[str]:
        events: list[str] = []
        st = info["state"]
        lq = info.get("lq")
        self.total_samples += 1
        if st == STATE_CONNECTED:
            self.connected_samples += 1

        if self.prev_state == STATE_CONNECTED and st != STATE_CONNECTED:
            self.degradations += 1
            msg = f"{self.role} drop Connected->{st}"
            events.append(msg)
            ndjson(
                f"bench:{self.role}",
                msg,
                {"from": STATE_CONNECTED, "to": st, "rel_s": round(rel_s, 2)},
                hypothesis_id="H3" if self.role == "RX" else "H1",
            )
            self.last_lq = lq  # skip paired LQ degrade on same sample
            self.prev_state = st
            return events

        if lq is not None and self.last_lq is not None:
            if self.prev_state == STATE_CONNECTED and lq < LQ_FLOOR and self.last_lq >= LQ_FLOOR:
                self.degradations += 1
                msg = f"{self.role} LQ degrade {self.last_lq}->{lq}"
                events.append(msg)
                ndjson(
                    f"bench:{self.role}",
                    msg,
                    {"lq": lq, "prev_lq": self.last_lq, "rel_s": round(rel_s, 2)},
                    hypothesis_id="H3",
                )

        if lq is not None:
            self.last_lq = lq
        self.prev_state = st
        return events


def reader(port: str, role: str, tracker: RoleTracker, t0: float, stop: threading.Event) -> None:
    try:
        fd = os.open(port, os.O_RDONLY | os.O_NONBLOCK)
    except OSError as e:
        ndjson(f"serial:{role}", "open_error", {"port": port, "err": str(e)})
        stop.set()
        return

    buf = b""
    while not stop.is_set() and (time.time() - t0) < MAX_SECS:
        try:
            data = os.read(fd, 1024)
        except BlockingIOError:
            time.sleep(0.02)
            continue
        except OSError as e:
            ndjson(f"serial:{role}", "read_error", {"port": port, "err": str(e)})
            break
        if not data:
            time.sleep(0.02)
            continue
        buf += data
        while b"\n" in buf:
            line_b, buf = buf.split(b"\n", 1)
            line = line_b.decode(errors="replace").rstrip()
            rel = time.time() - t0
            if "[TX STATUS]" in line or "[RX STATUS]" in line:
                info = parse_status(line)
                if info:
                    for ev in tracker.update(info, rel):
                        print(f"[{rel:7.1f}s] DEGRADE: {ev}", flush=True)
            if "[DBG40" in line:
                ndjson(f"fw:{role}", line[:120], {"line": line, "rel_s": round(rel, 2)}, hypothesis_id="H1")
    os.close(fd)


def main() -> int:
    if os.path.exists(LOG_PATH):
        os.remove(LOG_PATH)

    ndjson("bench", "start", {"tx": TX_PORT, "rx": RX_PORT, "max_s": MAX_SECS, "max_deg": MAX_DEGRADATIONS})

    t0 = time.time()
    stop = threading.Event()
    tx_tr = RoleTracker("TX")
    rx_tr = RoleTracker("RX")

    threads = [
        threading.Thread(target=reader, args=(TX_PORT, "TX", tx_tr, t0, stop), daemon=True),
        threading.Thread(target=reader, args=(RX_PORT, "RX", rx_tr, t0, stop), daemon=True),
    ]
    for t in threads:
        t.start()

    reason = "timeout"
    while (time.time() - t0) < MAX_SECS and not stop.is_set():
        time.sleep(1.0)
        rel = time.time() - t0
        total_deg = tx_tr.degradations + rx_tr.degradations
        if total_deg >= MAX_DEGRADATIONS:
            reason = f"early_abort degradations={total_deg}"
            break
        if rel >= 30 and tx_tr.total_samples > 0 and rx_tr.total_samples > 0:
            tx_pct = 100 * tx_tr.connected_samples // tx_tr.total_samples
            rx_pct = 100 * rx_tr.connected_samples // rx_tr.total_samples
            if rel >= 60 and tx_pct < 50 and rx_pct < 50:
                reason = f"early_abort no_stable_link tx_s3={tx_pct}% rx_s3={rx_pct}%"
                break

    stop.set()
    for t in threads:
        t.join(timeout=2.0)

    elapsed = time.time() - t0
    summary = {
        "elapsed_s": round(elapsed, 1),
        "reason": reason,
        "tx_samples": tx_tr.total_samples,
        "rx_samples": rx_tr.total_samples,
        "tx_connected_pct": (100 * tx_tr.connected_samples // tx_tr.total_samples) if tx_tr.total_samples else 0,
        "rx_connected_pct": (100 * rx_tr.connected_samples // rx_tr.total_samples) if rx_tr.total_samples else 0,
        "tx_degradations": tx_tr.degradations,
        "rx_degradations": rx_tr.degradations,
    }
    ndjson("bench", "summary", summary)

    print("\n=== stability summary ===", flush=True)
    for k, v in summary.items():
        print(f"  {k}: {v}", flush=True)

    passed = (
        reason == "timeout"
        and summary["tx_connected_pct"] >= 90
        and summary["rx_connected_pct"] >= 90
        and (tx_tr.degradations + rx_tr.degradations) <= MAX_DEGRADATIONS
    )
    print("PASS" if passed else "FAIL", flush=True)
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
