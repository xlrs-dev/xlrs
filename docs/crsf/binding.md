# CRSF Binding

This page describes the CRSF-facing Bind RX behavior.

## Summary

When TX is built with `-DXLRS_TX_CONTROLLER_PROTOCOL=CRSF`, the CRSF parameter
menu exposes a Bind RX command. Triggering that command starts a temporary OTA
bind workflow:

1. TX reads its current persisted Link UID.
2. TX switches the RF core to a shared XLRS bind identity for about 30 seconds.
3. TX transmits XLRS `Bind` OTA frames containing its real Link UID.
4. An unconnected RX periodically enters short bind-scan windows on the shared
   bind identity.
5. If RX receives a valid bind frame while scanning, it persists the offered
   Link UID and reboots.
6. After reboot, RX uses the learned Link UID for normal acquisition.

This means Bind RX can pair an RX that is not already connected to the TX.

## Validation Boundary

The RX does not check its normal binding phrase while in bind scan. That is
intentional: first-time pairing means the RX may not yet know the TX identity.

Instead, RX only accepts bind frames when all of these are true:

| Check | Purpose |
| --- | --- |
| RX is in bind-scan mode | Avoid accepting bind data during normal operation |
| Radio sync word matches the shared bind identity | Avoid normal-link or random same-frequency frames |
| OTA frame type is `Bind` | Accept only the binding frame type |
| Bind payload magic/version is valid | Reject malformed bind-looking payloads |
| Offered Link UID CRC matches | Reject corrupted UID payloads |

This protects against accidental noise or malformed packets. It is not a secure
ownership proof.

## Relationship To Binding Phrase

The binding phrase still exists as a local way to derive a Link UID. OTA bind
does not exchange the phrase; it exchanges the derived Link UID. After a
successful bind, the RX stores the learned UID directly in flash.

Compile-time phrase binding remains supported:

```bash
cmake -S . -B build -G Ninja -DXLRS_DEFAULT_BINDING_PHRASE="your-phrase"
cmake --build build --target xlrs_tx xlrs_rx
```

Use the same phrase on both modules when relying on compile-time binding.

## Operator Flow

1. Build the TX with CRSF enabled.
2. Power the TX and open the controller CRSF device parameters.
3. Power the RX.
4. Trigger Bind RX from the controller.
5. Wait for the RX to receive the bind frame, persist the UID, and reboot.
6. After reboot, TX and RX should acquire normally using the same Link UID.

## Current Limits

- There is no physical bind-button workflow yet.
- There is no cryptographic challenge/response yet.
- The custom UART controller protocol can still set the TX binding phrase, but
  CRSF Bind RX is the implemented unconnected RX pairing path.
