#pragma once

#include <stdint.h>
#include "link/Link.h"
#include "link/RfScheduler.h"
#include "phy/MockPhy.h"

namespace xlrs_test {

struct SimEnvironment {
    xlrs::MockPhy txPhy;
    xlrs::MockPhy rxPhy;
    xlrs::Link tx;
    xlrs::Link rx;
    xlrs::RfScheduler txSched;
    xlrs::RfScheduler rxSched;

    void setup(const uint8_t uid[xlrs::LINK_UID_SIZE], uint8_t rateIndex) {
        xlrs::PhyConfig cfg{};
        cfg.freqMHz = 2420.0f;
        cfg.payloadLen = 8;
        txPhy.init(cfg);
        rxPhy.init(cfg);
        xlrs::MockPhy::connect(txPhy, rxPhy);

        tx.begin(xlrs::Role::Tx, uid, rateIndex);
        rx.begin(xlrs::Role::Rx, uid, rateIndex);

        txSched.begin(&txPhy, &tx, rateIndex);
        rxSched.begin(&rxPhy, &rx, rateIndex);
    }
};

static xlrs::RfScheduler* g_txSched = nullptr;
static xlrs::RfScheduler* g_rxSched = nullptr;

static void onTxRxDone() { if (g_txSched) g_txSched->onRxDone(); }
static void onTxTxDone() { if (g_txSched) g_txSched->onTxDone(); }
static void onRxRxDone() { if (g_rxSched) g_rxSched->onRxDone(); }
static void onRxTxDone() { if (g_rxSched) g_rxSched->onTxDone(); }

static void simTick(SimEnvironment& env, uint32_t tick, bool txOn = true) {
    g_txSched = &env.txSched;
    g_rxSched = &env.rxSched;

    env.txPhy.setOnRxDone(onTxRxDone);
    env.txPhy.setOnTxDone(onTxTxDone);
    env.rxPhy.setOnRxDone(onRxRxDone);
    env.rxPhy.setOnTxDone(onRxTxDone);

    xlrs::Slot slot = env.tx.slotForTick(tick);   // authoritative (FHSS-pos-based) slot decision
    if (slot == xlrs::Slot::Telemetry) {
        if (txOn) env.txSched.onTick(tick);
        env.rxSched.onTick(tick);
    } else {
        env.rxSched.onTick(tick);
        if (txOn) env.txSched.onTick(tick);
    }

    if (txOn) env.tx.service(tick);
    env.rx.service(tick);
}

} // namespace xlrs_test
