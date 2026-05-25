// RfScheduler implementation (M3).
#include "link/RfScheduler.h"
#include "link/Link.h"
#include "phy/IRadioPhy.h"
#include "fhss/Fhss.h"
#include "hal/Time.h"

namespace xlrs {

static RfScheduler* s_activeScheduler = nullptr;

RfScheduler::RfScheduler() = default;

RfScheduler::~RfScheduler() {
    if (_timer) {
        _timer->stop();
        delete _timer;
        _timer = nullptr;
    }
    if (s_activeScheduler == this) {
        s_activeScheduler = nullptr;
    }
}

bool RfScheduler::begin(IRadioPhy* phy, Link* link, uint8_t rateIndex) {
    _phy = phy;
    _link = link;
    if (rateIndex >= kNumRates) rateIndex = 0;
    _rateIndex = rateIndex;
    _rate = kRates[rateIndex];

    _pfd.begin(_rate.intervalUs);
    _currentSlot = Slot::Idle;
    _tick = 0;
    _tickStartUs = 0;
    _nextPhyRecoveryAttemptUs = 0;
    _armedTick = 0;
    _armedPos = 0;
    _armedTickStartUs = 0;
    _syncedIdentityRevision = 0;
    _identitySynced = false;
    _tickEvents.store(0, std::memory_order_release);
    _rxDoneEvents.store(0, std::memory_order_release);
    _txDoneEvents.store(0, std::memory_order_release);
    _lastProcessedTickEvent = 0;
    _lastProcessedRxEvent = 0;
    _lastProcessedTxEvent = 0;

    // init() already applied sync word and packet params; track revision for bind/rate changes.
    _syncedIdentityRevision = _link->identityRevision();
    _identitySynced = true;

    if (!_timer) {
        _timer = createHwTimer();
    }

    s_activeScheduler = this;

#if defined(XLRS_PICO_SDK)
    // On hardware, run a true Tiny ISR: latch the fire timestamp, then bump the event counter.
    // The timestamp is captured HERE (in ISR context) — not later in onTick() — so the PFD's
    // phase reference is the true tick time, free of task wake-latency jitter.
    if (_timer) {
        _timer->begin(_rate.intervalUs, []() {
            if (s_activeScheduler) {
                s_activeScheduler->_lastTickFireUs.store((uint32_t)hal::nowUs(),
                                                         std::memory_order_release);
                s_activeScheduler->_tickEvents.fetch_add(1, std::memory_order_acq_rel);
            }
        });
    }
    if (_phy) {
        _phy->setOnRxDone([]() {
            if (s_activeScheduler) {
                s_activeScheduler->_rxDoneEvents.fetch_add(1, std::memory_order_acq_rel);
            }
        });
        _phy->setOnTxDone([]() {
            if (s_activeScheduler) {
                s_activeScheduler->_txDoneEvents.fetch_add(1, std::memory_order_acq_rel);
            }
        });
    }
#else
    // In simulated/host test environments, invoke the callbacks directly and synchronously
    // because simulation has no background task runner, so direct execution is expected.
    if (_timer) {
        // Match the hardware tiny-ISR pattern: the timer callback only bumps the event
        // counter; poll() drains it into onTick()+service(). (Sim drives it via fireSimTimerTick.)
        _timer->begin(_rate.intervalUs, []() {
            if (s_activeScheduler) {
                s_activeScheduler->_tickEvents.fetch_add(1, std::memory_order_acq_rel);
            }
        });
    }
    if (_phy) {
        _phy->setOnRxDone([]() {
            if (s_activeScheduler) {
                s_activeScheduler->onRxDone();
            }
        });
        _phy->setOnTxDone([]() {
            if (s_activeScheduler) {
                s_activeScheduler->onTxDone();
            }
        });
    }
#endif

    return true;
}

void RfScheduler::onTick(uint32_t tick) {
    if (!_link) return;

    _tick = tick;
#if defined(XLRS_PICO_SDK)
    _tickStartUs = _lastTickFireUs.load(std::memory_order_acquire);
#else
    _tickStartUs = _timer ? _timer->nowUs() : 0;
#endif

    // 1. Advance timing/hop position at the start of the tick slot.
    _link->onTick(tick);
    syncPhyIdentity();

    if (_link->role() == Role::Rx) {
        if (_link->takeRxTimingResetPending()) {
            _pfd.begin(_rate.intervalUs);
            _lastPfdAdjUs = 0;
        }
        // Hold nominal cadence only before FHSS lock. While Connecting/Failsafe with lock,
        // PFD and Sync phase-resync own the timer interval — resetting here was undoing
        // every PFD nudge each tick (n:1 on the bench, telemetry window drift).
        if (!_link->isLocked() && _timer) {
            _timer->setIntervalUs(_rate.intervalUs);
        }
    }

    // Dynamically adopt rate changes initiated by Link after onTick(), because
    // synchronized rate switches are applied exactly on the slot boundary.
    if (_link->stats().rateIndex != _rateIndex) {
        _rateIndex = _link->stats().rateIndex;
        _rate = kRates[_rateIndex];
        _pfd.begin(_rate.intervalUs);
        if (_timer) {
            _timer->setIntervalUs(_rate.intervalUs);
        }
        if (_phy && _phy->healthy()) {
            float freq = _link->freqForTick(tick);
            PhyConfig phyCfg = makePhyConfig(_rate, freq, _link->txPowerDbm(), _link->syncWord());
            _phy->reconfigure(phyCfg);
        }
    }

    // 2. Determine frequency and slot for this tick.
    float freq = _link->freqForTick(tick);
    _currentSlot = _link->slotForTick(tick);

    // 3. Keep physical output power synchronized.
    if (_phy && _phy->healthy()) {
        _phy->setOutputPowerDbm(_link->txPowerDbm());
    }

    // 4. Sequence half-duplex transmit/receive turnaround.
    if (!_phy || !_phy->healthy()) {
        return;
    }

    if (_link->role() == Role::Tx) {
        if (_currentSlot == Slot::Sync || _currentSlot == Slot::Uplink) {
#if defined(XLRS_PICO_SDK)
            // After a telemetry slot, wait out any extended listen window before uplink TX
            // so downlink can land without aborting RX — but still send this slot's RC frame.
            if (_currentSlot == Slot::Uplink && tick > 0 &&
                _link->slotForTick(tick - 1) == Slot::Telemetry) {
                if (telemetryListenActive() && _timer) {
                    const int32_t waitUs = (int32_t)(_tlmListenUntilUs - _timer->nowUs());
                    if (waitUs > 0) {
                        hal::sleepUs((uint32_t)waitUs);
                    }
                    _tlmListenUntilUs = 0;
                }
            }
#endif
            uint8_t buf[OTA16_LEN];
            uint8_t len = 0;
            uint16_t pos = _link->txPos(tick);
            if (_link->getTxPayload(tick, pos, buf, len)) {
                if (_phy) {
#if defined(XLRS_PICO_SDK)
                    uint32_t latency = _phy->txLatencyUs();
                    uint32_t delayUs = (TX_GUARD_US > latency) ? (TX_GUARD_US - latency) : 0;
                    if (delayUs > 0) {
                        hal::sleepUs(delayUs);
                    }
#endif
                    _phy->startTx(freq, buf, len);
                }
            }
        } else if (_currentSlot == Slot::Telemetry) {
            _armedTick = tick;
            _armedPos = _link->txPos(tick);
            _armedTickStartUs = _tickStartUs;
#if defined(XLRS_PICO_SDK)
            _link->advanceHwTelemetryLqSlot(tick);
#endif
            _pendingTlmRx = true;
            _pendingTlmRxFreq = freq;
            _pendingTlmArmedTick = tick;
            _pendingTlmArmedPos = _armedPos;
            _pendingTlmArmedTickStartUs = _tickStartUs;
            tryArmTelemetryRx();
        }
    } else {
        // RX Role
        if (_currentSlot == Slot::Sync || _currentSlot == Slot::Uplink) {
            if (_phy) {
                _armedTick = tick;
                _armedPos = _link->isLocked()
                            ? _link->txPos(_link->effectiveTxTick(tick))
                            : _link->rxPos();
                _armedTickStartUs = _tickStartUs;
#if defined(XLRS_PICO_SDK)
                if (_link->isLocked() && _currentSlot == Slot::Uplink) {
                    _link->advanceHwUplinkLqSlot(tick);
                }
                const uint32_t nowUs = _timer ? _timer->nowUs() : _tickStartUs;
                const int32_t elapsed = (int32_t)(nowUs - _tickStartUs);
                if (elapsed >= 0 && elapsed < (int32_t)TX_GUARD_US) {
                    hal::sleepUs((uint32_t)(TX_GUARD_US - (uint32_t)elapsed));
                }
#endif
                _phy->startRx(freq);
            }
        } else if (_currentSlot == Slot::Telemetry) {
            if (_link->isLocked()) {
                uint8_t buf[OTA16_LEN];
                uint8_t len = 0;
                uint16_t pos = _link->rxPos();
                if (_link->getTxPayload(tick, pos, buf, len)) {
                    if (_phy) {
#if defined(XLRS_PICO_SDK)
                        uint32_t delayUs = (_rate.intervalUs / 8) + TX_GUARD_US;
                        uint32_t latency = _phy->txLatencyUs();
                        if (delayUs > latency) {
                            delayUs -= latency;
                        } else {
                            delayUs = 0;
                        }
                        if (delayUs > 0) {
                            hal::sleepUs(delayUs);
                        }
#endif
                        _phy->startTx(freq, buf, len);
                    }
                }
            }
        }
    }
}

void RfScheduler::applyLockedRxPhaseResync(uint32_t packetStartUs) {
#if defined(XLRS_PICO_SDK)
    if (!_timer || !_link || _link->role() != Role::Rx || !_link->isLocked()) {
        return;
    }
    // TX airtime begins TX_GUARD after the ISR-latched slot boundary; both peers schedule from
    // tickStartUs so PFD expects arrival at that offset, not at the raw boundary.
    const int32_t offsetUs =
        (int32_t)packetStartUs - (int32_t)(_armedTickStartUs + TX_GUARD_US);
    const int32_t adjUs = _pfd.update(offsetUs);
    _lastPfdAdjUs = adjUs;
    _pfdUpdateCount++;
    // Track phase only — timer resync is reserved for Sync beacons (hard snap).
    _timer->setIntervalUs(_rate.intervalUs);
#else
    (void)packetStartUs;
#endif
}

bool RfScheduler::telemetrySlotStillOpen() const {
    if (!_pendingTlmRx) {
        return false;
    }
    const uint32_t nowUs = _timer ? _timer->nowUs() : 0;
    const int32_t elapsed = (int32_t)(nowUs - _pendingTlmArmedTickStartUs);
    return elapsed >= 0 && elapsed < (int32_t)(_rate.intervalUs + (_rate.intervalUs / 2));
}

void RfScheduler::onRxDone() {
    if (!_phy || !_link) return;
    const bool tlmListen = (_link->role() == Role::Tx &&
                            (_link->slotForTick(_armedTick) == Slot::Telemetry ||
                             telemetryListenActive()));
    RxPacket pkt;
    if (_phy->readRx(pkt)) {
        if (tlmListen) {
            _tlmRxPhyOkCount++;
        }
        // Decode/authenticate against the slot the radio was ARMED with (not the current tick),
        // so a packet drained after a tick advance is still matched to its own (tick,pos) nonce
        // and PFD reference. More reachable now that poll() can fast-forward the tick.
        uint32_t airtime = (pkt.len <= 8) ? _rate.airtime8Us : _rate.airtime16Us;
        uint32_t packetStartUs = pkt.timestampUs - airtime;
        const bool success = _link->processRxPayload(_armedTick, _armedPos, pkt.data, pkt.len, pkt.rssiDbm, pkt.snr,
                                                     packetStartUs);
        if (success) {
            const bool isSyncSlot =
                _link->slotForTick(_armedTick) == Slot::Sync;
            if (_link->role() == Role::Rx && _link->isLocked() && !isSyncSlot) {
                applyLockedRxPhaseResync(packetStartUs);
            }
        }
        if (tlmListen) {
            if (success) {
                _tlmRxDecodeOkCount++;
                _tlmListenUntilUs = 0;
                _tlmDecodeSlotTick = 0;
            } else {
                _tlmRxDecodeFailCount++;
#if defined(XLRS_PICO_SDK)
                if (_link->role() == Role::Tx) {
                    const uint32_t tlmTick =
                        _tlmDecodeSlotTick ? _tlmDecodeSlotTick : _armedTick;
                    if (_link->slotForTick(tlmTick) == Slot::Telemetry) {
                        _link->noteHwTelemetryDecode(tlmTick, false);
                    }
                }
#endif
            }
        }
#if defined(XLRS_PICO_SDK)
        if (!success && _link->role() == Role::Rx && _link->isLocked() &&
            _link->slotForTick(_armedTick) == Slot::Uplink) {
            _link->noteHwUplinkDecode(_armedTick, false);
        }
#endif
        if (success) {
#if defined(XLRS_PICO_SDK)
            if (_link->role() == Role::Rx) {
                _link->refreshRxPosForTick(_armedTick);
                if (_link->isLocked() && _link->slotForTick(_armedTick) == Slot::Uplink) {
                    _link->noteHwUplinkDecode(_armedTick, true);
                }
                _link->service(_armedTick, false);
            } else if (_link->role() == Role::Tx && tlmListen) {
                const uint32_t tlmTick =
                    _tlmDecodeSlotTick ? _tlmDecodeSlotTick : _armedTick;
                if (_link->slotForTick(tlmTick) == Slot::Telemetry) {
                    _link->noteHwTelemetryDecode(tlmTick, true);
                }
                _link->service(tlmTick, false);
            }
#else
            if (_link->role() == Role::Rx) {
                _link->refreshRxPosForTick(_armedTick);
            }
            // Sim: service() runs from simTick() after onTick completes.
#endif
            uint32_t txTick = 0;
            bool resetPfd = false;
            if (_link->role() == Role::Rx && _link->takeSyncResyncTick(txTick, resetPfd)) {
                _link->snapSchedulerTick(txTick, _armedTick);
                if (resetPfd) {
                    _pfd.begin(_rate.intervalUs);
                }
                if (_timer) {
                    if (resetPfd) {
                        _timer->setIntervalUs(_rate.intervalUs);
                    }
                    int32_t offsetUs =
                        (int32_t)packetStartUs - (int32_t)(_armedTickStartUs + TX_GUARD_US);
                    const int32_t half = (int32_t)_rate.intervalUs / 2;
                    while (offsetUs > half) offsetUs -= (int32_t)_rate.intervalUs;
                    while (offsetUs < -half) offsetUs += (int32_t)_rate.intervalUs;
                    const int32_t nextUs =
                        (int32_t)(_armedTickStartUs + _rate.intervalUs) - offsetUs;
                    _timer->resyncNextTickUs((uint32_t)nextUs);
                }
            }
            // Sim: flags are consumed by SimEnvironment::simTick() after onTick returns.
#if !defined(XLRS_PICO_SDK)
            if (_link->role() == Role::Rx && _link->isLocked() &&
                _link->slotForTick(_armedTick) != Slot::Sync) {
                int32_t offsetUs = (int32_t)(pkt.timestampUs - airtime) - (int32_t)_armedTickStartUs;
                int32_t adjUs = _pfd.update(offsetUs);
                _lastPfdAdjUs = adjUs;
                _pfdUpdateCount++;
                if (_timer) {
                    _timer->setIntervalUs(_rate.intervalUs + adjUs);
                }
            } else
#endif
            if (_link->role() == Role::Rx && _timer && !_link->isLocked()) {
                _lastPfdAdjUs = 0;
                _timer->setIntervalUs(_rate.intervalUs);
            }
        }
    }
}

void RfScheduler::onTxDone() {
    tryArmTelemetryRx();
}

void RfScheduler::tryArmTelemetryRx() {
    if (!_pendingTlmRx || !_phy || !_link || _link->role() != Role::Tx) {
        return;
    }
    if (!telemetrySlotStillOpen()) {
        _pendingTlmRx = false;
        _tlmRxArmAtUs = 0;
        _tlmRxArmDroppedCount++;
        return;
    }
    if (_phy->txInProgress()) {
        _tlmRxArmDeferredCount++;
        return;
    }
#if defined(XLRS_PICO_SDK)
    const uint32_t nowUs = _timer ? _timer->nowUs() : 0;
    const uint32_t downlinkEarliestUs =
        _pendingTlmArmedTickStartUs + (_rate.intervalUs / 8) + TX_GUARD_US;
    if (nowUs < downlinkEarliestUs) {
        _tlmRxArmAtUs = downlinkEarliestUs;
        return;
    }
#endif
    _tlmRxArmAtUs = 0;
    _armedTick = _pendingTlmArmedTick;
    _armedPos = _pendingTlmArmedPos;
    _armedTickStartUs = _pendingTlmArmedTickStartUs;
    _tlmDecodeSlotTick = _pendingTlmArmedTick;
    _phy->startRx(_pendingTlmRxFreq);
    _pendingTlmRx = false;
    if (_timer) {
        const uint32_t listenUs =
            (_rate.intervalUs / 2) + _rate.airtime16Us + TX_GUARD_US + 800;
        _tlmListenUntilUs = _timer->nowUs() + listenUs;
    }
    _tlmRxArmedCount++;
}

bool RfScheduler::telemetryListenActive() const {
    if (!_timer || _tlmListenUntilUs == 0) {
        return false;
    }
    return (int32_t)(_timer->nowUs() - _tlmListenUntilUs) < 0;
}

Slot RfScheduler::currentSlot() const {
    return _currentSlot;
}

bool RfScheduler::recoverPhyIfNeeded() {
    if (!_phy || _phy->healthy()) return true;
    uint32_t nowUs = _timer ? _timer->nowUs() : 0;
    if (_nextPhyRecoveryAttemptUs != 0 &&
        (int32_t)(nowUs - _nextPhyRecoveryAttemptUs) < 0) {
        return false;
    }
    const bool ok = _phy->recover();
    if (ok) {
        _syncedIdentityRevision = _link->identityRevision();
        _identitySynced = true;
    }
    if (_link) {
        _link->notePhyRecovery(ok);
    }
    _nextPhyRecoveryAttemptUs = ok ? 0 : nowUs + PHY_RECOVERY_BACKOFF_US;
    return ok;
}

void RfScheduler::syncPhyIdentity(bool force) {
    if (!_phy || !_link || !_phy->healthy()) return;
    const uint32_t revision = _link->identityRevision();
    if (!force && _identitySynced && revision == _syncedIdentityRevision) return;

    _phy->setSyncWord(_link->syncWord());
    _syncedIdentityRevision = revision;
    _identitySynced = true;
}

void RfScheduler::drainPendingRxDone() {
    uint32_t currentRx = _rxDoneEvents.load(std::memory_order_acquire);
    if (currentRx > _lastProcessedRxEvent) {
        const uint32_t diff = currentRx - _lastProcessedRxEvent;
        for (uint32_t i = 0; i < diff; ++i) {
            onRxDone();
        }
        _lastProcessedRxEvent = currentRx;
    }
}

void RfScheduler::poll() {
    if (!recoverPhyIfNeeded()) {
        return;
    }

    // 1. Process RX Done
    drainPendingRxDone();

    // 2. Process TX Done
    uint32_t currentTx = _txDoneEvents.load(std::memory_order_acquire);
    if (currentTx > _lastProcessedTxEvent) {
        uint32_t diff = currentTx - _lastProcessedTxEvent;
        for (uint32_t i = 0; i < diff; ++i) {
            onTxDone();
        }
        _lastProcessedTxEvent = currentTx;
    }

    // 3. Process ticks
    uint32_t currentTicks = _tickEvents.load(std::memory_order_acquire);
    if (currentTicks > _lastProcessedTickEvent) {
        uint32_t diff = currentTicks - _lastProcessedTickEvent;
        if (diff > MAX_TICK_CATCHUP) {
            // Core 1 fell badly behind (long stall). Replaying every backlogged slot would
            // burst blocking SPI + TX_GUARD sleeps for stale FHSS positions and keep us behind.
            if (_link && _link->role() == Role::Rx && _link->isLocked()) {
                // Stay hop-locked: discard stale backlog and wait for the next Sync snap
                // instead of fast-forwarding a local tick that diverges from TX cadence.
                _link->noteMissedDeadlines((uint16_t)diff);
            } else {
                if (_link) _link->noteSchedulerOverrun((uint16_t)diff);
                const uint32_t targetTick = _tick + diff;
                onTick(targetTick);
                if (_link) _link->service(targetTick);
            }
        } else {
            for (uint32_t i = 0; i < diff; ++i) {
                uint32_t nextTick = _tick + 1;
                onTick(nextTick);
                if (_link) {
#if defined(XLRS_PICO_SDK)
                    _link->service(nextTick, false);
#else
                    _link->service(nextTick);
#endif
                }
                // Late DIO: count uplink decode before the next tick's service().
                drainPendingRxDone();
            }
        }
        _lastProcessedTickEvent = currentTicks;
    }

    tryArmTelemetryRx();

    recoverPhyIfNeeded();
}

} // namespace xlrs
