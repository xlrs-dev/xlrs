// CrsfLinkStats — pack LinkStats into a CRSF LINK_STATISTICS (0x14) 10-byte payload.
//
// The RX app (CrsfAdapter) queues this to the flight controller so the OSD/pilot see real
// RF health. Pure / host-testable (no Arduino, no crsfSerial). RSSI fields are the positive
// magnitude of the negative dBm, per CRSF convention.
#pragma once
#include <stdint.h>
#include "link/Link.h"   // LinkStats

namespace xlrs {

inline void buildCrsfLinkStatistics(const LinkStats& s, uint8_t out[10]) {
    out[0] = (uint8_t)(s.rssiDbm <= 0 ? -s.rssiDbm : 0);   // uplink_RSSI_1 (-dBm magnitude)
    out[1] = 0;                                            // uplink_RSSI_2 (no antenna diversity)
    out[2] = s.lqUp;                                       // uplink_LQ
    out[3] = (uint8_t)s.snr;                               // uplink_SNR
    out[4] = 0;                                            // active_antenna
    out[5] = s.rateIndex;                                  // rf_mode
    out[6] = 0;                                            // uplink_TX_power index
    out[7] = (uint8_t)(s.downlinkRssiDbm <= 0 ? -s.downlinkRssiDbm : 0);
    out[8] = s.lqDown;                                     // downlink_LQ
    out[9] = (uint8_t)s.downlinkSnr;
}

} // namespace xlrs
