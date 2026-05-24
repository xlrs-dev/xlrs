# SX1280 PHY Init And Diagnostics

This note covers bench-visible SX1280 bring-up failures after the Pico SDK
migration: `[HW FAULT]`, rising `PHY timeouts`, and `LastFailOp: 0x00` on TX/RX
status lines. It applies to `Sx1280NativePhy` and the RF scheduler path that
calls it during boot and recovery.

See also [serial-logs.md](serial-logs.md) for status-line format and
[sx1280-wiring.md](../hardware/sx1280-wiring.md) for pin checks.

---

## Symptoms

Healthy boot:

```text
[PHY] Mode: FLRC|LoRa, Sync Word: 0x....
PHY timeouts: 0
LastFailOp: 0x00
No [HW FAULT]
```

Failure pattern fixed on this path:

```text
[HW FAULT]
PHY timeouts: increasing
Phase: SyncWord/fail or Reset/fail
LastFailOp: 0x00 or 0x80 (SetStandby)
```

Boot may also emit one-shot `[PHY DIAG]` lines while init tracing is enabled.
Those name the init phase (`Reset`, `Standby`, `SyncWord`, …) and the opcode
that failed.

---

## Root Causes Addressed

### 1. Register writes while the radio is in RX

After `init()`, the scheduler could call `setSyncWord()` while the chip was still
in receive mode. SX1280 register writes require standby. The driver now:

1. Enters standby before sync-word or packet-parameter register writes when not idle.
2. Restores RX if the caller had the radio listening.

`init()` leaves the radio in standby; the scheduler arms RX/TX on the first tick
instead of calling `startRx()` at the end of init.

### 2. Reset sequence not aligned with RadioLib

The native driver reset path matches the working PlatformIO/RadioLib bench setup:

- 1 ms reset pulse, then keep `RST` driven high (do not tri-state).
- Retry `SetStandby` for up to 3 s via `trySetStandby()` before marking hardware fault.
- SPI at 8 MHz, mode 0 (CPOL 0, CPHA 0).
- Disable pulls on the `BUSY` input.

### 3. Post-command BUSY waits on mode entry

`SetRx` and `SetTx` skip the post-command BUSY wait (`waitPostBusy=false`). The
chip may hold BUSY through RX/TX entry; completion is signaled on DIO1
(`RX_DONE` / `TX_DONE`).

### 4. Scheduler init and recovery

- `RfScheduler::begin()` no longer calls `syncPhyIdentity(true)` immediately after
  `init()` — init already applied sync word and packet params.
- `recover()` only refreshes identity-revision tracking; it does not re-call
  `setSyncWord()` on a radio that `init()` just reprogrammed.
- PHY recovery is rate-limited (`PHY_RECOVERY_BACKOFF_US`) so a wedged radio is not
  hammered with reset+SPI every poll loop.
- Slot work is skipped while `healthy()` is false.

---

## PHY Status Fields

TX and RX periodic status lines include PHY diagnostics:

```text
... | PHY timeouts: <n> CRC: <n> Phase: <phase>/<status> LastOp: 0xNN LastOk: 0xNN LastFailOp: 0xNN
```

| Field | Meaning |
| --- | --- |
| `Phase` | Last init/runtime phase (`Reset`, `SyncWord`, `StartRx`, …) and `begin`/`ok`/`fail` |
| `LastOp` | Opcode of the SPI command most recently started |
| `LastOk` | Opcode of the last SPI command that completed without BUSY timeout |
| `LastFailOp` | Opcode of the last SPI operation that hit a BUSY timeout or set `_hardwareError` |

Common opcodes from `lib/xlrs/phy/Sx1280Regs.h`:

| Opcode | Meaning |
| --- | --- |
| `0x00` | No recorded failure yet, or counter cleared on successful `init()` |
| `0x80` | `SetStandby` — often first failure after reset or stuck BUSY |
| `0x82` | `SetRx` |
| `0x83` | `SetTx` |
| `0x18` | `WriteRegister` — sync word / packet params path |
| `0x19` | `ReadRegister` |
| `0x1A` | `WriteBuffer` |
| `0x1B` | `ReadBuffer` |

Use `Phase`, `LastFailOp`, and `PHY timeouts` together:

- `Reset/fail` or `LastFailOp: 0x80` after boot → wiring, reset timing, or BUSY stuck.
- `SyncWord/fail` with `LastFailOp: 0x18` → identity sync while not in standby (should
  no longer occur after the standby guard).
- `StartRx/fail` or `StartTx/fail` during link bring-up → mode entry or DIO1 path; scope
  BUSY and DIO1 if timeouts rise but CRC stays zero.

---

## Bench Validation Checklist

1. Flash matching TX/RX images; confirm identical `[UID]` and `[PHY]` lines.
2. Watch 10+ seconds of status: `PHY timeouts` stable at 0, no `[HW FAULT]`.
3. If faults return, capture `Phase`, `LastFailOp`, full boot log, and SPI/BUSY/DIO1 captures.
4. Run `scripts/test.sh` before chasing RF link issues — scheduler PHY recovery backoff
   is covered natively.

Link acquisition failures with clean PHY counters are a separate RF/timing problem;
do not change FHSS or slot timing until init-path counters are stable.
