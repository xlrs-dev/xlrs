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
    int32_t  pfdAdjUs;
    uint32_t pfdUpdates;
    uint32_t timerIntervalUs;
    uint32_t nomIntervalUs;
    uint32_t tlmRxArmed;
    uint32_t tlmRxArmDeferred;
    uint32_t tlmRxArmDropped;
    uint32_t tlmRxPhyOk;
    uint32_t tlmRxDecodeOk;
    uint32_t tlmRxDecodeFail;
};

inline void fillLinkRuntimeDiag(LinkRuntimeDiag& d,
                                const Link& link,
                                const RfScheduler& sched) {
    d.schedulerTick = sched.processedTick();
    d.fhssIndex = link.stats().fhssIndex;
    d.fhssExpected = link.txPos(link.role() == Role::Rx && link.isLocked()
                                ? link.effectiveTxTick(d.schedulerTick)
                                : d.schedulerTick);
    d.syncFhssSkew = link.syncFhssSkew();
    d.fhssLocked = link.isLocked();
    d.syncSeen = link.syncSeen();
    d.pfdPhaseUs = sched.pfdPhaseErrorUs();
    d.pfdAdjUs = sched.pfdLastAdjUs();
    d.pfdUpdates = sched.pfdUpdateCount();
    d.timerIntervalUs = sched.timerIntervalUs();
    d.nomIntervalUs = kRates[link.stats().rateIndex].intervalUs;
    d.tlmRxArmed = sched.tlmRxArmedCount();
    d.tlmRxArmDeferred = sched.tlmRxArmDeferredCount();
    d.tlmRxArmDropped = sched.tlmRxArmDroppedCount();
    d.tlmRxPhyOk = sched.tlmRxPhyOkCount();
    d.tlmRxDecodeOk = sched.tlmRxDecodeOkCount();
    d.tlmRxDecodeFail = sched.tlmRxDecodeFailCount();
}

} // namespace xlrs::app
