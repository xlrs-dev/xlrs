# RF Config Storage

RF configuration is stored in flash at offset `0` as `RfConfigData`, validated
by magic, schema version, and CRC16. Binding state is stored separately starting
at offset `120`, so RF config and binding records do not overlap.

Current RF config fields:

```c
struct RfConfigData {
    uint32_t magic;        // "RFCG"
    uint8_t  version;      // 1
    uint8_t  region;       // 0 US, 1 EU
    uint8_t  defaultRate;  // index into kRates
    int8_t   maxPowerDbm;  // -18..13, EU capped to 10
    uint8_t  failsafeMode; // 0 NoPulses, 1 Hold
    uint8_t  dynamicPower; // 0 disabled, 1 enabled
    uint8_t  reserved[2];
    uint16_t checksum;
} __attribute__((packed));
```

Current limitation: there is no implemented external UART/CRSF/MSP command that
edits `RfConfigData`. The firmware loads it on boot, writes defaults if invalid,
and uses it to initialize the link.

See [index.md](index.md) for the complete current interface reference.
