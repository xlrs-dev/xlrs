# OTA Protocol

The active PlatformIO rewrite uses a compact fixed-size SX1280 LoRa OTA frame
implemented in `src/protocol.cpp`. It carries packed RC channels and downlink
telemetry without exposing radio-driver details above the PHY boundary.

The current OTA frame is intentionally plaintext. It is protected by:

- SX1280/RadioLib PHY CRC.
- A firmware CRC16 over the encoded OTA bytes.
- A 32-bit `uid_check` hashed from all 8 binding phrase UID bytes.

`uid_check` is an anti-crosstalk identity filter, not authentication. The UID is
derived from the binding phrase with FNV-1a, which is fast and
non-cryptographic. There is no active ChaCha20-Poly1305/AEAD layer in the
PlatformIO rewrite. Reintroducing authentication or encryption should be treated
as a safety-sensitive OTA format change.

Terminology:

- `ota_frame` is the whole over-the-air frame.
- `payload` is only the data inside the frame.
- `rc_payload` is packed RC channel data.
- `telemetry_payload` is telemetry data inside an OTA telemetry frame.

## Active Frame Layout

```text
byte 0      magic
byte 1      type: 1=RC, 2=Telemetry, 3=Sync
byte 2..3   sequence, big-endian
byte 4..7   uid_check, big-endian
byte 8      payload length
byte 9..30  payload, padded to 22 bytes
byte 31..32 CRC16-CCITT over bytes 0..30
```

RC payloads carry 16 CRSF 11-bit channel values packed into 22 bytes.
Telemetry payloads currently carry link quality, RSSI magnitude, SNR, and rate.
Sync payloads (`OtaSyncPayload`) carry the nonce, FHSS index, current/next rate,
telemetry ratio, hop interval, and TX tick so the RX can acquire and stay aligned.

## FHSS / Fixed Channel

The firmware uses UID-derived FHSS by default. Shipped PlatformIO environments
define `RF_FIXED_CHANNEL=-1`, which enables hopping through `fhssChannelFor()`.
Set `RF_FIXED_CHANNEL` to a non-negative channel index only for bench/debug fixed
channel operation.

`g_hop` follows OTA RC frame sequence numbers. On RX re-acquisition, the first
valid RC frame resets local sequence tracking, receives on `hopForSequence()`,
and then starts listening on `nextHopAfterReceivedSequence()` so TX/RX alignment
recovers from link drops.

The legacy CMake/Pico SDK frame implementation under `lib/xlrs/ota/` remains in
the tree for reference, but it is not the active PlatformIO OTA contract.

See [architecture.md](architecture.md), [configuration.md](configuration.md), and
[terminology.md](terminology.md) for current constraints.
