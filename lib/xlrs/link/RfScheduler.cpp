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

    syncPhyIdentity(true);

    if (!_timer) {
        _timer = createHwTimer();
    }

    s_activeScheduler = this;

#if defined(XLRS_PICO_SDK)
    // On hardware, run a true Tiny ISR: latch the fire timestamp, then bump the event counter.
    // The timestamp is captured HERE (in ISR context) — not later in onTick() — so the PFD's
    // phase reference is the true tick time, free of task wake-latency jitter (docs/troubleshooting/index.md §1.1).
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
    // Use the ISR-latched fire time (set in the timer callback above), not nowUs() sampled here:
    // this slot is being processed some wake-latency after the timer actually fired.
    _tickStartUs = _lastTickFireUs.load(std::memory_order_acquire);
#else
    // Sim/host: tests call onTick() directly with the clock already set, so nowUs() IS the
    // expected tick time. (The timer ISR path isn't exercised for these direct calls.)
    _tickStartUs = _timer ? _timer->nowUs() : 0;
#endif

    // 1. Advance timing/hop position at the start of the tick slot.
    _link->onTick(tick);
    syncPhyIdentity();

    // Dynamically adopt rate changes initiated by Link after onTick(), because
    // synchronized rate switches are applied exactly on the slot boundary.
    if (_link->stats().rateIndex != _rateIndex) {
        _rateIndex = _link->stats().rateIndex;
        _rate = kRates[_rateIndex];
        _pfd.begin(_rate.intervalUs);
        if (_timer) {
            _timer->setIntervalUs(_rate.intervalUs);
        }
        if (_phy) {
            float freq = _link->freqForTick(tick);
            PhyConfig phyCfg = makePhyConfig(_rate, freq, _link->txPowerDbm(), _link->syncWord());
            _phy->reconfigure(phyCfg);
        }
    }

    // 2. Determine frequency and slot for this tick.
    float freq = _link->freqForTick(tick);
    _currentSlot = _link->slotForTick(tick);

    // 3. Keep physical output power synchronized.
    if (_phy) {
        _phy->setOutputPowerDbm(_link->txPowerDbm());
    }

    // 4. Sequence half-duplex transmit/receive turnaround.
    if (_link->role() == Role::Tx) {
        if (_currentSlot == Slot::Sync || _currentSlot == Slot::Uplink) {
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
            if (_phy) {
                _armedTick = tick; _armedPos = _link->txPos(tick); _armedTickStartUs = _tickStartUs;
                _phy->startRx(freq);
            }
        }
    } else {
        // RX Role
        if (_currentSlot == Slot::Sync || _currentSlot == Slot::Uplink) {
            if (_phy) {
                _armedTick = tick; _armedPos = _link->rxPos(); _armedTickStartUs = _tickStartUs;
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
                        uint32_t latency = _phy->txLatencyUs();
                        uint32_t delayUs = (TX_GUARD_US > latency) ? (TX_GUARD_US - latency) : 0;
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

void RfScheduler::onRxDone() {
    if (!_phy || !_link) return;
    RxPacket pkt;
    if (_phy->readRx(pkt)) {
        // Decode/authenticate against the slot the radio was ARMED with (not the current tick),
        // so a packet drained after a tick advance is still matched to its own (tick,pos) nonce
        // and PFD reference. More reachable now that poll() can fast-forward the tick.
        bool success = _link->processRxPayload(_armedTick, _armedPos, pkt.data, pkt.len, pkt.rssiDbm, pkt.snr);
        if (success) {
            // Apply Phase-Frequency Detector (PFD) correction to timer interval (RX only).
            if (_link->role() == Role::Rx) {
                uint32_t airtime = (pkt.len <= 8) ? _rate.airtime8Us : _rate.airtime16Us;
                // Correct PFD negative feedback sign: offsetUs = actual - expected
#if defined(XLRS_PICO_SDK)
                int32_t offsetUs = (int32_t)(pkt.timestampUs - airtime - TX_GUARD_US) - (int32_t)_armedTickStartUs;
#else
                int32_t offsetUs = (int32_t)(pkt.timestampUs - airtime) - (int32_t)_armedTickStartUs;
#endif
                int32_t adjUs = _pfd.update(offsetUs);
                if (_timer) {
                    _timer->setIntervalUs(_rate.intervalUs + adjUs);
                }
            }
        }
    }
}

void RfScheduler::onTxDone() {
    // No-op or minor timing/telemetry tracking.
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
        syncPhyIdentity(true);
    }
    if (_link) {
        _link->notePhyRecovery(ok);
    }
    _nextPhyRecoveryAttemptUs = ok ? 0 : nowUs + PHY_RECOVERY_BACKOFF_US;
    return ok;
}

void RfScheduler::syncPhyIdentity(bool force) {
    if (!_phy || !_link) return;
    const uint32_t revision = _link->identityRevision();
    if (!force && _identitySynced && revision == _syncedIdentityRevision) return;

    _phy->setSyncWord(_link->syncWord());
    _syncedIdentityRevision = revision;
    _identitySynced = true;
}

void RfScheduler::poll() {
    if (!recoverPhyIfNeeded()) {
        return;
    }

    // 1. Process RX Done
    uint32_t currentRx = _rxDoneEvents.load(std::memory_order_acquire);
    if (currentRx > _lastProcessedRxEvent) {
        uint32_t diff = currentRx - _lastProcessedRxEvent;
        for (uint32_t i = 0; i < diff; ++i) {
            onRxDone();
        }
        _lastProcessedRxEvent = currentRx;
    }

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
            // Fast-forward to the latest tick instead: TX FHSS is tick-derived so a jump stays
            // correct, and an RX this far behind has lost its slot lock and re-acquires from the
            // next Sync beacon. Record the gap as missed deadlines so the overrun is observable.
            if (_link) _link->noteSchedulerOverrun(diff);
            onTick(currentTicks);
            if (_link) _link->service(currentTicks);
        } else {
            for (uint32_t i = 0; i < diff; ++i) {
                uint32_t nextTick = _tick + 1;
                onTick(nextTick);
                if (_link) {
                    _link->service(nextTick);
                }
            }
        }
        _lastProcessedTickEvent = currentTicks;
    }

    recoverPhyIfNeeded();
}

} // namespace xlrs
