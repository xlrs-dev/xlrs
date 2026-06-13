#!/usr/bin/env python3
"""Minimal ELRS link bench: watch TX/RX USB serial for 2 minutes."""
from __future__ import annotations

import os
import re
import select
import sys
import time
from dataclasses import dataclass, field

MAX_SECS = float(os.environ.get("STABILITY_SECS", "120"))
POLL_SECS = float(os.environ.get("POLL_SECS", "0.2"))
CONNECT_DEADLINE_SECS = float(os.environ.get("CONNECT_DEADLINE_SECS", "60"))
MAX_RX_DROPS = int(os.environ.get("MAX_RX_DROPS", "1"))
TX_PORT = os.environ.get("TX_PORT", "")
RX_PORT = os.environ.get("RX_PORT", "")
TX_SER = os.environ.get("TX_SER", "E46488B28B561830")
RX_SER = os.environ.get("RX_SER", "E4654C16430F4223")

TX_CONNECTED_RE = re.compile(r"\bgot downlink conn\b", re.IGNORECASE)
RX_CONNECTED_RE = re.compile(r"\bgot conn\b", re.IGNORECASE)
RX_TENTATIVE_RE = re.compile(r"\btentative conn\b", re.IGNORECASE)
RX_LOST_RE = re.compile(r"\blost conn\b", re.IGNORECASE)
RX_LINK_STATS_RE = re.compile(
    r"^\s*(\d+),(\d+),(-?\d+),(\d+),(-?\d+),(\d+),(\d+),(-?\d+)\s*$"
)


@dataclass
class RoleState:
    role: str
    port: str
    connected: bool = False
    saw_connected: bool = False
    saw_tentative: bool = False
    connect_at_s: float | None = None
    drops: int = 0
    lines: int = 0
    last_line: str = ""
    link_stats_samples: int = 0
    last_link_stats: dict[str, int] | None = None
    events: list[str] = field(default_factory=list)

    def note(self, rel_s: float, message: str) -> None:
        entry = f"[{rel_s:6.1f}s] {self.role}: {message}"
        self.events.append(entry)
        print(entry, flush=True)


def resolve_port(role: str, explicit: str, serial: str, fallback: str) -> str:
    if explicit:
        return explicit
    try:
        import serial.tools.list_ports as list_ports

        for port in list_ports.comports():
            if port.serial_number == serial and port.device:
                return port.device
    except ImportError:
        pass
    if os.path.exists(fallback):
        return fallback
    raise OSError(f"could not find {role} port (serial {serial}); set {role}_PORT")


def open_port(path: str) -> int:
    try:
        return os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    except OSError as exc:
        print(f"failed to open {path}: {exc}", file=sys.stderr)
        raise


def consume_line(state: RoleState, line: str, rel_s: float) -> None:
    state.lines += 1
    state.last_line = line

    if state.role == "TX" and TX_CONNECTED_RE.search(line):
        if not state.connected:
            state.connected = True
            state.saw_connected = True
            state.connect_at_s = rel_s
            state.note(rel_s, "connected")
        return

    if state.role == "RX":
        if RX_TENTATIVE_RE.search(line):
            state.saw_tentative = True
            state.note(rel_s, "tentative")
            return
        if RX_CONNECTED_RE.search(line):
            if not state.connected:
                state.connected = True
                state.saw_connected = True
                state.connect_at_s = rel_s
                state.note(rel_s, "connected")
            return
        if RX_LOST_RE.search(line):
            state.connected = False
            state.drops += 1
            state.note(rel_s, f"lost connection (drops={state.drops})")
            return

        stats_match = RX_LINK_STATS_RE.match(line)
        if stats_match:
            packet_counter, antenna, rssi_dbm, lq, snr, tx_power, fhss, pfd = (
                int(value) for value in stats_match.groups()
            )
            state.link_stats_samples += 1
            state.last_link_stats = {
                "packet_counter": packet_counter,
                "antenna": antenna,
                "rssi_dbm": rssi_dbm,
                "lq": lq,
                "snr": snr,
                "tx_power": tx_power,
                "fhss": fhss,
                "pfd": pfd,
            }
            if state.link_stats_samples == 1:
                state.note(
                    rel_s,
                    (
                        "saw RX link stats "
                        f"(rssi={rssi_dbm}dBm lq={lq} snr={snr} txp={tx_power})"
                    ),
                )


def drain_fd(fd: int, state: RoleState, buffers: dict[int, bytes], rel_s: float) -> None:
    try:
        chunk = os.read(fd, 4096)
    except BlockingIOError:
        return
    except OSError as exc:
        state.note(rel_s, f"read error: {exc}")
        raise

    if not chunk:
        return

    buffers[fd] += chunk
    while b"\n" in buffers[fd]:
        raw_line, buffers[fd] = buffers[fd].split(b"\n", 1)
        line = raw_line.decode(errors="replace").rstrip()
        if line:
            consume_line(state, line, rel_s)


def main() -> int:
    try:
        tx_port = resolve_port("TX", TX_PORT, TX_SER, "/dev/cu.usbmodem1101")
        rx_port = resolve_port("RX", RX_PORT, RX_SER, "/dev/cu.usbmodem5")
    except OSError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    tx_state = RoleState(role="TX", port=tx_port)
    rx_state = RoleState(role="RX", port=rx_port)

    try:
        tx_fd = open_port(tx_state.port)
        rx_fd = open_port(rx_state.port)
    except OSError:
        return 1

    print(f"Monitoring ELRS logs for up to {MAX_SECS:.0f}s", flush=True)
    print(f"  TX: {tx_state.port}", flush=True)
    print(f"  RX: {rx_state.port}", flush=True)
    print("Waiting for TX='got downlink conn' and RX='got conn'...", flush=True)

    buffers = {tx_fd: b"", rx_fd: b""}
    state_by_fd = {tx_fd: tx_state, rx_fd: rx_state}
    start = time.time()
    deadline = start + MAX_SECS
    reason = "timeout"

    try:
        while time.time() < deadline:
            now = time.time()
            rel_s = now - start
            readable, _, _ = select.select([tx_fd, rx_fd], [], [], POLL_SECS)
            for fd in readable:
                drain_fd(fd, state_by_fd[fd], buffers, rel_s)

            if rel_s >= CONNECT_DEADLINE_SECS and not (tx_state.saw_connected and rx_state.saw_connected):
                reason = "connect_timeout"
                break
            if rx_state.drops > MAX_RX_DROPS:
                reason = f"rx_drops>{MAX_RX_DROPS}"
                break
    finally:
        os.close(tx_fd)
        os.close(rx_fd)

    summary = {
        "reason": reason,
        "elapsed_s": round(min(time.time() - start, MAX_SECS), 1),
        "tx_connected": tx_state.saw_connected,
        "rx_tentative": rx_state.saw_tentative,
        "rx_connected": rx_state.saw_connected,
        "rx_drops": rx_state.drops,
        "rx_link_stats_samples": rx_state.link_stats_samples,
        "rx_last_link_stats": rx_state.last_link_stats,
    }

    print("\n=== elrs bench summary ===", flush=True)
    for key, value in summary.items():
        print(f"  {key}: {value}", flush=True)

    if rx_state.link_stats_samples == 0:
        print(
            "  note: no RX link-stats CSV seen; that is expected unless ELRS is built with DEBUG_RCVR_LINKSTATS",
            flush=True,
        )

    passed = (
        reason == "timeout"
        and tx_state.saw_connected
        and rx_state.saw_connected
        and rx_state.drops <= MAX_RX_DROPS
    )
    print("PASS" if passed else "FAIL", flush=True)
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
