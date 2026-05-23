// OtaPacket — versioned, type-multiplexed over-the-air frame.
//
// One compact frame format carries every kind of traffic (RC, sync, telemetry, bind,
// MSP) selected by a type field, with an explicit protocol version so the format can
// evolve without silent breakage. Addressing is by FHSS sequence + radio sync word
// (both UID-seeded) — there is NO per-packet device ID (unlike the legacy format).
//
// Target sizes: 8-byte frame (OTA8) for high-rate RC, 16-byte (OTA16) variant for
// telemetry/params. The PHY appends/checks the hardware CRC; an optional ICipher seals
// the payload region.
#pragma once
#include <stdint.h>
#include <string.h>

namespace xlrs {

static constexpr uint8_t  OTA_VERSION   = 1;
static constexpr uint8_t  OTA8_LEN      = 8;
static constexpr uint8_t  OTA16_LEN     = 16;

// Link UID: 8 bytes (64-bit), derived from the binding phrase (hash) so a bound TX/RX
// share it and seed the SAME FHSS sequence + sync word. NOT a board's own hardware ID
// (the hardware flash unique ID is the per-device *serial*, used for identity, not
// addressing). See architecture.md §8.
static constexpr uint8_t  LINK_UID_SIZE = 8;

// Header byte layout: [ ver:2 | type:3 | flags:3 ]
enum class OtaType : uint8_t {
    Rc       = 0,  // packed RC channels
    Sync     = 1,  // TX→RX timing/FHSS/rate beacon
    TlmDown  = 2,  // RX→TX telemetry (link stats, battery, sensors)
    TlmUp    = 3,  // TX→RX uplink (commands)
    Bind     = 4,  // pairing/bind handshake
    Msp      = 5,  // MSP / parameter passthrough
};

inline uint8_t otaMakeHeader(OtaType type, uint8_t flags = 0) {
    return (uint8_t)(((OTA_VERSION & 0x3) << 6) | (((uint8_t)type & 0x7) << 3) | (flags & 0x7));
}
inline uint8_t  otaVersion(uint8_t hdr) { return (hdr >> 6) & 0x3; }
inline OtaType  otaType(uint8_t hdr)    { return (OtaType)((hdr >> 3) & 0x7); }
inline uint8_t  otaFlags(uint8_t hdr)   { return hdr & 0x7; }

// Sync beacon payload: lets the RX acquire timing, FHSS phase, rate, and telemetry slot.
// Sent periodically by the TX (interleaved into the RC stream).
struct __attribute__((packed)) SyncPayload {
    uint8_t  fhssIndex;    // current hop index
    uint8_t  rateIndex;    // active rate index
    uint8_t  nextRateIndex;// pending rate index (switch target)
    uint32_t switchTick;   // sequence cycles until activation (0 if none)
    uint8_t  tlmRatioDenom;// telemetry slot ratio (1:N)
    uint8_t  uidCrc;       // CRC of the bind UID
};


} // namespace xlrs
