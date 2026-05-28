#pragma once

#include <stdint.h>

/** Shared display/USB telemetry payloads (formerly from UARTProtocol for custom TX). ELRS CRSF fills the same TelemetryData fields. */

struct TelemetryData {
    int16_t rssi;
    float snr;
    uint16_t rxBattMv;
    uint8_t rxBattPct;
    uint8_t linkQuality;
} __attribute__((packed));

struct StatusData {
    uint8_t connectionState;
    uint8_t pairingState;
    uint32_t packetsReceived;
    uint32_t packetsLost;
} __attribute__((packed));
