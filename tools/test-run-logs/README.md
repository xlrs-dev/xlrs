# Hardware in-loop test logs

Captured TX/RX `[TX STATUS]` / `[RX STATUS]` serial logs from bench runs.

| Run | Date | Duration | Result | Path |
| --- | --- | --- | --- | --- |
| D250 bench pair | 2026-06-01 | 120s | PASS | `hw-inloop-20260601-000858/` |
| TX BOOTSEL disconnect | 2026-06-01 | 75s (boot @15s) | PARTIAL | `hw-disconnect-20260601-002205/` |

Build: `-DXLRS_BENCH_TX=ON -DXLRS_BENCH_RATE=2` (D250).  
TX serial `E4654C16432C3B22`, RX serial `E4654C16430F4223`.  
Tool: `tools/bench-capture-once.py`.
