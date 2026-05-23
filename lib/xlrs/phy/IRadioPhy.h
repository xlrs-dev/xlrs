// IRadioPhy — abstract async radio PHY contract. Deliberately NARROW.
//
// The layers above depend only on this interface, never on a specific radio chip.
// That keeps the link core host-testable and lets us swap PHY implementations without
// leaking driver concepts or types across this line.
//
// Async + tiny-ISR model:
//   - startTx/startRx return immediately.
//   - The PHY latches the hardware arrival timestamp at IRQ time into the next
//     RxPacket.timestampUs. The callback signature carries no args BY DESIGN — event type
//     is known from WHICH callback fired, and the timestamp travels in RxPacket. That
//     timestamp is load-bearing: the PFD needs true arrival time normalized to packet
//     start (RxPacket.timestampUs - airtime), not when the RF task wakes.
//   - The TxDone/RxDone callbacks run in ISR context and must do ONLY minimal work: publish
//     a MONOTONIC event counter via a release-ordered atomic (a boolean would coalesce
//     back-to-back TX/RX events and lose one). No SPI work, no logging, no allocation.
//   - The RF task (core-1 loop), after observing the counter (acquire), calls readRx() to
//     drain the packet from the radio. All SPI work happens here, never in the ISR.
//
// Frequency is passed per-operation (not a separate setter) so the FHSS hop is atomic with
// arming RX/TX and can never be forgotten.
#pragma once
#include <stdint.h>
#include <stddef.h>

namespace xlrs {

enum class Modulation : uint8_t { Flrc, Lora };

// Init-time configuration for one rate. Frequency and sync word can change at runtime
// (hop / re-bind) via the per-op freq argument and setSyncWord().
struct PhyConfig {
    float       freqMHz;          // initial centre frequency
    Modulation  modulation;
    float       bwKHz;            // bandwidth (LoRa) / context (FLRC)
    uint8_t     sf;               // LoRa spreading factor (ignored for FLRC)
    uint8_t     cr;               // coding rate
    uint16_t    flrcBitrateKbps;  // FLRC bitrate (ignored for LoRa)
    int8_t      powerDbm;
    uint16_t    syncWord;         // UID-derived — provides addressing/isolation
    uint8_t     payloadLen;       // fixed OTA payload length
};

// One received frame. RSSI/SNR are returned WITH the packet (atomic; avoids a stale-read
// race that a separate getLastRssi() would invite). SNR is valid only for LoRa; 0 for FLRC.
struct RxPacket {
    uint8_t  data[32];
    uint8_t  len;
    int16_t  rssiDbm;
    int8_t   snr;
    uint32_t timestampUs;   // captured in the RxDone ISR, for the PFD
};

using PhyIsrCallback = void (*)();

class IRadioPhy {
public:
    virtual ~IRadioPhy() = default;

    virtual bool init(const PhyConfig& cfg) = 0;

    // Async I/O — return immediately; freq passed per-op so the hop is atomic with arming.
    virtual void startRx(float freqMHz) = 0;
    virtual void startTx(float freqMHz, const uint8_t* data, uint8_t len) = 0;

    // Drain the last received packet. Called from the RF task (NOT the ISR).
    virtual bool readRx(RxPacket& out) = 0;

    // Cheap runtime controls.
    virtual void setOutputPowerDbm(int8_t dbm) = 0;
    virtual void setSyncWord(uint16_t uidDerived) = 0;
    virtual void reconfigure(const PhyConfig& cfg) = 0;
    virtual uint32_t txLatencyUs() const = 0;
    virtual bool healthy() const = 0;
    virtual bool recover() = 0;

    // Tiny ISR-context completion hooks (flag + timestamp only — see header note).
    virtual void setOnTxDone(PhyIsrCallback cb) = 0;
    virtual void setOnRxDone(PhyIsrCallback cb) = 0;
};

} // namespace xlrs
