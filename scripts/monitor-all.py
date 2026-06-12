#!/usr/bin/env python3
import argparse
import glob
import os
import queue
import re
import signal
import sys
import threading
import time
from dataclasses import dataclass

try:
    import serial
except ImportError:
    print("pyserial is required. Install it with: python3 -m pip install pyserial", file=sys.stderr)
    sys.exit(1)


COLORS = {
    "tx": "\033[38;5;39m",
    "rx": "\033[38;5;82m",
    "rc": "\033[38;5;214m",
    "log": "\033[38;5;245m",
    "error": "\033[38;5;203m",
    "muted": "\033[38;5;244m",
    "reset": "\033[0m",
    "bold": "\033[1m",
    "clear": "\033[2J\033[H",
}


@dataclass
class Module:
    role: str
    port: str
    serial_port: serial.Serial


def list_default_ports():
    patterns = [
        "/dev/cu.usbmodem*",
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
    ]
    ports = []
    for pattern in patterns:
        ports.extend(glob.glob(pattern))
    return sorted(set(ports))


def env_ports():
    ports = []
    for name in ("TX_PORT", "RX_PORT", "RC_PORT"):
        value = os.environ.get(name)
        if value:
            ports.append(value)
    return ports


def normalize_role(status_line):
    match = re.search(r"\brole=([^ \r\n]+)", status_line)
    if not match:
        return None
    role = match.group(1).lower()
    if role == "rc-rp2350":
        return "rc"
    if role in ("tx", "rx", "rc"):
        return role
    return None


def open_and_probe(port, baud, probe_timeout):
    try:
        ser = serial.Serial(port, baud, timeout=0.05, write_timeout=0.2)
    except Exception as exc:
        return None, f"open failed: {exc}"

    try:
        time.sleep(0.15)
        ser.reset_input_buffer()
        ser.write(b"status\n")
        deadline = time.monotonic() + probe_timeout
        text = ""
        while time.monotonic() < deadline:
            chunk = ser.read(256)
            if chunk:
                text += chunk.decode(errors="replace")
                role = normalize_role(text)
                if role:
                    return Module(role, port, ser), None
            else:
                time.sleep(0.02)
        ser.close()
        return None, "no role=status response"
    except Exception as exc:
        try:
            ser.close()
        except Exception:
            pass
        return None, f"probe failed: {exc}"


def print_header(modules, baud, status_interval, use_color, clear_screen):
    if use_color and clear_screen:
        print(COLORS["clear"], end="")
    bold = COLORS["bold"] if use_color else ""
    muted = COLORS["muted"] if use_color else ""
    reset = COLORS["reset"] if use_color else ""
    print(f"{bold}XLRS module monitor{reset}")
    print(f"{muted}baud={baud} status_interval={status_interval:.1f}s  Ctrl-C to stop{reset}")
    for module in sorted(modules, key=lambda item: item.role):
        color = COLORS.get(module.role, "") if use_color else ""
        print(f"{color}[{module.role.upper():2}] {module.port}{reset}")
    print()


def reader(module, output, stop_event):
    buffer = bytearray()
    while not stop_event.is_set():
        try:
            data = module.serial_port.read(128)
        except Exception as exc:
            output.put((module.role, module.port, f"<read error: {exc}>"))
            return
        if not data:
            continue
        for byte in data:
            if byte in (10, 13):
                if buffer:
                    output.put((module.role, module.port, buffer.decode(errors="replace")))
                    buffer.clear()
            else:
                buffer.append(byte)
                if len(buffer) >= 240:
                    output.put((module.role, module.port, buffer.decode(errors="replace")))
                    buffer.clear()


def poller(modules, status_interval, stop_event):
    if status_interval <= 0:
        return
    while not stop_event.wait(status_interval):
        for module in modules:
            try:
                module.serial_port.write(b"status\n")
            except Exception:
                pass


def render_loop(output, stop_event, use_color):
    while not stop_event.is_set():
        try:
            role, port, line = output.get(timeout=0.1)
        except queue.Empty:
            continue
        timestamp = time.strftime("%H:%M:%S")
        color = COLORS.get(role, COLORS["log"]) if use_color else ""
        reset = COLORS["reset"] if use_color else ""
        muted = COLORS["muted"] if use_color else ""
        print(f"{muted}{timestamp}{reset} {color}[{role.upper():2}]{reset} {line}", flush=True)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Colorized multi-module USB serial monitor for TX, RX, and RC."
    )
    parser.add_argument("ports", nargs="*", help="Ports to probe. Defaults to TX_PORT/RX_PORT/RC_PORT or USB modem ports.")
    parser.add_argument("--baud", type=int, default=int(os.environ.get("BAUD", "115200")))
    parser.add_argument(
        "--status-interval",
        type=float,
        default=float(os.environ.get("STATUS_INTERVAL", "5")),
        help="Seconds between automatic status commands. Use 0 to disable.",
    )
    parser.add_argument("--probe-timeout", type=float, default=1.0)
    parser.add_argument("--no-color", action="store_true")
    parser.add_argument("--no-clear", action="store_true", help="Do not clear the terminal on start.")
    return parser.parse_args()


def main():
    args = parse_args()
    use_color = not args.no_color and sys.stdout.isatty()
    ports = args.ports or env_ports() or list_default_ports()
    if not ports:
        print("No candidate serial ports found.", file=sys.stderr)
        return 1

    modules = []
    seen_roles = set()
    print("Probing serial ports:", " ".join(ports), file=sys.stderr)
    for port in ports:
        module, error = open_and_probe(port, args.baud, args.probe_timeout)
        if not module:
            print(f"  skip {port}: {error}", file=sys.stderr)
            continue
        if module.role in seen_roles:
            print(f"  skip {port}: duplicate role {module.role}", file=sys.stderr)
            module.serial_port.close()
            continue
        seen_roles.add(module.role)
        modules.append(module)

    if not modules:
        print("No TX/RX/RC modules responded to status.", file=sys.stderr)
        return 1

    output = queue.Queue()
    stop_event = threading.Event()

    def stop(_signum=None, _frame=None):
        stop_event.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    print_header(modules, args.baud, args.status_interval, use_color, not args.no_clear)

    threads = []
    for module in modules:
        thread = threading.Thread(target=reader, args=(module, output, stop_event), daemon=True)
        thread.start()
        threads.append(thread)

    poll_thread = threading.Thread(target=poller, args=(modules, args.status_interval, stop_event), daemon=True)
    poll_thread.start()

    try:
        render_loop(output, stop_event, use_color)
    finally:
        stop_event.set()
        for module in modules:
            try:
                module.serial_port.close()
            except Exception:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
