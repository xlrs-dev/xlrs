// LinkRuntimeDiag — shared bench diagnostics for TX/RX status lines.
#pragma once
#include "link/Link.h"
#include "link/RfScheduler.h"
#include "link/RateConfig.h"

namespace xlrs::app {

struct LinkRuntimeDiag {
    uint32_t schedulerTick;
    uint16_t fhssIndex;
    uint16_t fhssExpected;
    int8_t   syncFhssSkew;   // RX: fhssIndex in last Sync beacon minus txPos(txTick)
    bool     fhssLocked;
    bool     syncSeen;
    int32_t  pfdPhaseUs;
    uint32_t timerIntervalUs;
    uint32_t nomIntervalUs;
};

inline void fillLinkRuntimeDiag(LinkRuntimeDiag& d,
                                const Link& link,
                                const RfScheduler& sched) {
    d.schedulerTick = sched.processedTick();
    d.fhssIndex = link.stats().fhssIndex;
    d.fhssExpected = link.txPos(d.schedulerTick);
    d.syncFhssSkew = link.syncFhssSkew();
    d.fhssLocked = link.isLocked();
    d.syncSeen = link.syncSeen();
    d.pfdPhaseUs = sched.pfdPhaseErrorUs();
    d.timerIntervalUs = sched.timerIntervalUs();
    d.nomIntervalUs = kRates[link.stats().rateIndex].intervalUs;
}

} // namespace xlrs::app
