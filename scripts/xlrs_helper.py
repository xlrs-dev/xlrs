#!/usr/bin/env python3
# /// script
# dependencies = [
#   "pyserial",
# ]
# ///

"""
XLRS Developer Helper Utility
An interactive & non-interactive utility to:
1. Detect connected transmitter (TX) and receiver (RX) RP2040 boards.
2. Confirm device identities via boot signature sniffing or manual designation.
3. Build and flash firmware using PlatformIO (Native PHY or RadioLib PHY modes).
4. Monitor color-coded serial logs from both boards concurrently in real time.
5. Log all activity to a local file.
6. Invoke host-side unit tests.
"""

import argparse
import datetime
import os
import re
import subprocess
import sys
import threading
import time

# Attempt to import serial tools
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("\033[93m[Dependency Warning] 'pyserial' not found. Installing via uv/pip...\033[0m")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pyserial"])
    import serial
    import serial.tools.list_ports

# Console ANSI styling
COLOR_TX = "\033[92m"       # Green
COLOR_RX = "\033[96m"       # Cyan
COLOR_WARN = "\033[93m"     # Yellow
COLOR_ERROR = "\033[91m"    # Red
COLOR_HEADER = "\033[1m\033[95m" # Bold Magenta
COLOR_INFO = "\033[97m"      # White
COLOR_RESET = "\033[0m"

stop_monitoring = threading.Event()

def log_message(log_file, prefix, text, color=COLOR_INFO):
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    msg = f"[{prefix} {timestamp}] {text}"
    # Print to console
    print(f"{color}{msg}{COLOR_RESET}")
    # Write to file
    if log_file:
        log_file.write(msg + "\n")
        log_file.flush()

def scan_serial_ports():
    ports = serial.tools.list_ports.comports()
    # Filter commonly used RP2040 / Pico USB-to-UART identifiers
    detected = []
    for p in ports:
        # Match standard macOS/Linux serial devices
        if any(x in p.device.lower() for x in ["usbmodem", "usbserial", "ttyacm", "ttyusb"]):
            detected.append(p.device)
    return detected if detected else [p.device for p in ports]

def detect_device_role(port, timeout=3.0, log_file=None):
    log_message(log_file, "SYSTEM", f"Probing port {port} at 115200 baud for up to {timeout}s...", COLOR_INFO)
    try:
        ser = serial.Serial(port, 115200, timeout=0.5)
        ser.reset_input_buffer()
        start = time.time()
        buffer = ""
        
        while time.time() - start < timeout:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                buffer += data
                
                # Check for TX signatures
                if any(sig in buffer for sig in ["=== XLRS Dual-Core Transmitter ===", "UART Protocol on Core 0", "[UART]"]):
                    ser.close()
                    return "TX"
                
                # Check for RX signatures
                if any(sig in buffer for sig in ["=== XLRS Dual-Core Receiver ===", "CRSF and LED on Core 0", "[RX STATUS]"]):
                    ser.close()
                    return "RX"
                    
            time.sleep(0.05)
        ser.close()
    except Exception as e:
        log_message(log_file, "SYSTEM", f"Could not read from {port}: {e}", COLOR_WARN)
    return None

def auto_detect_devices(log_file=None):
    ports = scan_serial_ports()
    tx_port = None
    rx_port = None
    
    if not ports:
        log_message(log_file, "SYSTEM", "No USB serial ports detected. Please ensure devices are plugged in.", COLOR_WARN)
        return None, None
        
    log_message(log_file, "SYSTEM", f"Detected ports: {ports}", COLOR_INFO)
    log_message(log_file, "SYSTEM", "Tip: If the devices are already running, click Reset on the board to send boot banners.", COLOR_WARN)
    
    # Try to sniff roles
    for p in ports:
        role = detect_device_role(p, timeout=2.5, log_file=log_file)
        if role == "TX":
            log_message(log_file, "SYSTEM", f"Auto-detected TX on {p}", COLOR_TX)
            tx_port = p
        elif role == "RX":
            log_message(log_file, "SYSTEM", f"Auto-detected RX on {p}", COLOR_RX)
            rx_port = p
            
    return tx_port, rx_port

def run_pio_command(args, log_file=None):
    log_message(log_file, "SYSTEM", f"Running PlatformIO: {' '.join(args)}", COLOR_INFO)
    try:
        process = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        while True:
            line = process.stdout.readline()
            if not line:
                break
            # Remove trailing newlines and print
            line_str = line.rstrip('\r\n')
            log_message(log_file, "PIO", line_str, COLOR_INFO)
            
        process.wait()
        return process.returncode == 0
    except Exception as e:
        log_message(log_file, "SYSTEM", f"Failed to run PIO: {e}", COLOR_ERROR)
        return False

def flash_devices(tx_port, rx_port, mode, log_file=None):
    tx_env = "xlrs_tx_native" if mode == "native" else "xlrs_tx"
    rx_env = "xlrs_rx_native" if mode == "native" else "xlrs_rx"
    
    success = True
    if tx_port:
        log_message(log_file, "SYSTEM", f"Building and Flashing TX environment '{tx_env}' to {tx_port}...", COLOR_TX)
        cmd = ["pio", "run", "-e", tx_env, "-t", "upload", "--upload-port", tx_port]
        if not run_pio_command(cmd, log_file):
            log_message(log_file, "SYSTEM", "TX Flash FAILED!", COLOR_ERROR)
            success = False
        else:
            log_message(log_file, "SYSTEM", "TX Flash SUCCEEDED!", COLOR_TX)
            
    if rx_port:
        log_message(log_file, "SYSTEM", f"Building and Flashing RX environment '{rx_env}' to {rx_port}...", COLOR_RX)
        cmd = ["pio", "run", "-e", rx_env, "-t", "upload", "--upload-port", rx_port]
        if not run_pio_command(cmd, log_file):
            log_message(log_file, "SYSTEM", "RX Flash FAILED!", COLOR_ERROR)
            success = False
        else:
            log_message(log_file, "SYSTEM", "RX Flash SUCCEEDED!", COLOR_RX)
            
    return success

def serial_monitor_thread(port, name, color, log_file):
    log_message(log_file, name, f"Starting serial log stream on {port}...", color)
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
        ser.reset_input_buffer()
        
        while not stop_monitoring.is_set():
            if ser.in_waiting:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').rstrip('\r\n')
                    if line:
                        log_message(log_file, name, line, color)
                except Exception as e:
                    log_message(log_file, name, f"Read error: {e}", COLOR_ERROR)
            else:
                time.sleep(0.01)
        ser.close()
    except Exception as e:
        log_message(log_file, name, f"Could not open port {port}: {e}", COLOR_ERROR)

def monitor_logs(tx_port, rx_port, log_file=None):
    if not tx_port and not rx_port:
        log_message(log_file, "SYSTEM", "No ports designated for monitoring.", COLOR_WARN)
        return
        
    stop_monitoring.clear()
    threads = []
    
    if tx_port:
        t = threading.Thread(target=serial_monitor_thread, args=(tx_port, "TX", COLOR_TX, log_file), daemon=True)
        threads.append(t)
        t.start()
        
    if rx_port:
        t = threading.Thread(target=serial_monitor_thread, args=(rx_port, "RX", COLOR_RX, log_file), daemon=True)
        threads.append(t)
        t.start()
        
    log_message(log_file, "SYSTEM", "Logs are streaming. Press Ctrl+C to exit monitoring.", COLOR_HEADER)
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        log_message(log_file, "SYSTEM", "Stopping log monitoring...", COLOR_WARN)
        stop_monitoring.set()
        for t in threads:
            t.join(timeout=1.0)

def run_unit_tests(log_file=None):
    log_message(log_file, "SYSTEM", "Running host unit tests (pio test -e native)...", COLOR_INFO)
    cmd = ["pio", "test", "-e", "native"]
    success = run_pio_command(cmd, log_file)
    if success:
        log_message(log_file, "SYSTEM", "All host unit tests PASSED!", COLOR_TX)
    else:
        log_message(log_file, "SYSTEM", "Some host unit tests FAILED!", COLOR_ERROR)
    return success

def interactive_mode(log_file=None):
    print(f"{COLOR_HEADER}===================================================={COLOR_RESET}")
    print(f"{COLOR_HEADER}           XLRS Developer Helper CLI Utility        {COLOR_RESET}")
    print(f"{COLOR_HEADER}===================================================={COLOR_RESET}")
    
    # 1. Device Scanning
    tx_port, rx_port = auto_detect_devices(log_file)
    
    ports = scan_serial_ports()
    
    if not tx_port:
        print(f"\n{COLOR_WARN}[TX Detection Failed]{COLOR_RESET}")
        print("Available ports:")
        for idx, p in enumerate(ports):
            print(f"  [{idx}] {p}")
        print("  [s] Skip TX")
        choice = input("Select TX Serial Port index (or 's' to skip): ").strip()
        if choice.isdigit() and int(choice) < len(ports):
            tx_port = ports[int(choice)]
            log_message(log_file, "SYSTEM", f"Manually selected TX: {tx_port}", COLOR_TX)
            
    if not rx_port:
        print(f"\n{COLOR_WARN}[RX Detection Failed]{COLOR_RESET}")
        print("Available ports:")
        for idx, p in enumerate(ports):
            if ports[idx] != tx_port:
                print(f"  [{idx}] {p}")
        print("  [s] Skip RX")
        choice = input("Select RX Serial Port index (or 's' to skip): ").strip()
        if choice.isdigit() and int(choice) < len(ports):
            rx_port = ports[int(choice)]
            log_message(log_file, "SYSTEM", f"Manually selected RX: {rx_port}", COLOR_RX)

    # Print current detected config
    print(f"\n{COLOR_HEADER}--- Current Configuration ---{COLOR_RESET}")
    print(f"TX Port: {COLOR_TX}{tx_port if tx_port else 'Not Connected'}{COLOR_RESET}")
    print(f"RX Port: {COLOR_RX}{rx_port if rx_port else 'Not Connected'}{COLOR_RESET}")
    
    # 2. Select Flash Mode
    print(f"\n{COLOR_HEADER}--- Flash Mode Selection ---{COLOR_RESET}")
    print("  [1] Native PHY Mode (USE_NATIVE_PHY = 1)")
    print("  [2] RadioLib PHY Mode (RadioLib backend)")
    print("  [3] Skip Flashing")
    mode_choice = input("Select Flash Mode [1-3]: ").strip()
    
    mode = None
    if mode_choice == "1":
        mode = "native"
    elif mode_choice == "2":
        mode = "radiolib"
        
    if mode:
        flash_devices(tx_port, rx_port, mode, log_file)
        
    # 3. Unit Tests Choice
    test_choice = input("\nWould you like to run host unit tests? (y/n): ").strip().lower()
    if test_choice == 'y':
        run_unit_tests(log_file)
        
    # 4. Monitor logs Choice
    if tx_port or rx_port:
        monitor_choice = input("\nWould you like to start streaming serial logs? (y/n): ").strip().lower()
        if monitor_choice == 'y':
            monitor_logs(tx_port, rx_port, log_file)

def main():
    parser = argparse.ArgumentParser(description="XLRS Dev Tools & Hardware Assistant")
    parser.add_argument("--interactive", "-i", action="store_true", help="Launch interactive CLI menu")
    parser.add_argument("--detect", "-d", action="store_true", help="Auto-detect connected TX and RX ports")
    parser.add_argument("--flash", "-f", choices=["native", "radiolib"], help="Flash firmware in native or radiolib mode")
    parser.add_argument("--tx-port", help="Directly specify TX serial port device path")
    parser.add_argument("--rx-port", help="Directly specify RX serial port device path")
    parser.add_argument("--monitor", "-m", action="store_true", help="Start real-time serial log streaming and configuration inspection")
    parser.add_argument("--test", "-t", action="store_true", help="Run PlatformIO native unit tests")
    parser.add_argument("--log-file", default="xlrs_session.log", help="Path to write log outputs (default: xlrs_session.log)")
    
    args = parser.parse_args()
    
    # Open Session Log File
    try:
        log_file = open(args.log_file, "a", encoding="utf-8")
    except Exception as e:
        print(f"\033[91mCould not create session log file: {e}\033[0m")
        log_file = None
        
    # Print launch header
    log_message(log_file, "SYSTEM", f"=== Session Started: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===", COLOR_HEADER)
    
    # If no arguments specified, fallback to interactive
    if not (args.interactive or args.detect or args.flash or args.monitor or args.test):
        args.interactive = True
        
    if args.interactive:
        interactive_mode(log_file)
        return
        
    # Non-interactive executions
    tx_port = args.tx_port
    rx_port = args.rx_port
    
    if args.detect or (args.flash and not (tx_port or rx_port)):
        auto_tx, auto_rx = auto_detect_devices(log_file)
        if not tx_port:
            tx_port = auto_tx
        if not rx_port:
            rx_port = auto_rx
            
    if args.flash:
        if not tx_port and not rx_port:
            log_message(log_file, "SYSTEM", "Error: No devices specified or detected for flashing. Use --tx-port / --rx-port.", COLOR_ERROR)
            sys.exit(1)
        flash_devices(tx_port, rx_port, args.flash, log_file)
        
    if args.test:
        run_unit_tests(log_file)
        
    if args.monitor:
        if not tx_port and not rx_port:
            log_message(log_file, "SYSTEM", "Error: No devices specified or detected for monitoring. Use --tx-port / --rx-port.", COLOR_ERROR)
            sys.exit(1)
        monitor_logs(tx_port, rx_port, log_file)
        
    if log_file:
        log_file.close()

if __name__ == "__main__":
    main()
