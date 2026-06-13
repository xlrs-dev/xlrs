#!/usr/bin/env python3
import sys, time, threading, os, json

ports = [sys.argv[1], sys.argv[2]]
dur = float(sys.argv[3]) if len(sys.argv) > 3 else 12.0
t0 = time.time()
lock = threading.Lock()
LOGPATH = "/Users/ishan/dev/quoppo/dsi/rp2040-ppm-tx-rx/.cursor/debug-3b0792.log"
try:
    logf = open(LOGPATH, "a")
except Exception:
    logf = None

def emit(port, text):
    ts = time.time() - t0
    with lock:
        print(f"[{ts:7.3f}][{port[-4:]}] {text}", flush=True)
        if logf is not None:
            rec = {"sessionId": "3b0792", "location": f"rx_serial:{port[-4:]}",
                   "message": text, "data": {"port": port[-4:], "rel": round(ts, 3)},
                   "timestamp": int(time.time() * 1000)}
            logf.write(json.dumps(rec) + "\n"); logf.flush()

def reader(port):
    try:
        fd = os.open(port, os.O_RDONLY | os.O_NONBLOCK)
    except Exception as e:
        with lock:
            print(f"{port}: OPEN ERROR {e}", flush=True)
        return
    buf = b""
    while time.time() - t0 < dur:
        try:
            data = os.read(fd, 512)
        except BlockingIOError:
            time.sleep(0.02); continue
        except Exception as e:
            with lock:
                print(f"{port}: READ ERROR {e}", flush=True)
            break
        if data:
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                emit(port, line.decode(errors='replace').rstrip())
        else:
            time.sleep(0.02)
    os.close(fd)

threads = [threading.Thread(target=reader, args=(p,)) for p in ports]
for t in threads: t.start()
for t in threads: t.join()
print("=== capture done ===", flush=True)
