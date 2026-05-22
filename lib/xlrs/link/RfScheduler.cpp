// RfScheduler implementation (M3).
#include "link/RfScheduler.h"
#include "link/Link.h"
#include "phy/IRadioPhy.h"
#include "fhss/Fhss.h"

#if defined(ARDUINO)
#include <Arduino.h>
#endif

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

bool RfScheduler::begin(IRadioPhy* phy, Fhss* fhss, Link* link, uint8_t rateIndex) {
    _phy = phy;
    _fhss = fhss;
    _link = link;
    if (rateIndex >= kNumRates) rateIndex = 0;
    _rateIndex = rateIndex;
    _rate = kRates[rateIndex];
    
    _pfd.begin(_rate.intervalUs);
    _currentSlot = Slot::Idle;
    _tick = 0;
    _tickStartUs = 0;
    _tickEvents.store(0, std::memory_order_release);
    _rxDoneEvents.store(0, std::memory_order_release);
    _txDoneEvents.store(0, std::memory_order_release);
    _lastProcessedTickEvent = 0;
    _lastProcessedRxEvent = 0;
    _lastProcessedTxEvent = 0;

    if (_phy && _link) {
        _phy->setSyncWord(_link->syncWord());
    }

    if (!_timer) {
        _timer = createHwTimer();
    }

    s_activeScheduler = this;

#if defined(ARDUINO) || defined(PICO_BOARD)
    // On hardware, run a true Tiny ISR: only increment the event counter.
    if (_timer) {
        _timer->begin(_rate.intervalUs, []() {
            if (s_activeScheduler) {
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
        _timer->begin(_rate.intervalUs, []() {
            if (s_activeScheduler) {
                s_activeScheduler->onTick(s_activeScheduler->_tick + 1);
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

    // Dynamically adopt rate changes initiated by Link (either TX rate request or RX rate adoption)
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

    _tick = tick;
    if (_timer) {
        _tickStartUs = _timer->nowUs();
    } else {
        _tickStartUs = 0;
    }

    // 1. Advance timing/hop position at the start of the tick slot.
    _link->onTick(tick);

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
#if defined(ARDUINO) || defined(PICO_BOARD)
                    uint32_t latency = _phy->txLatencyUs();
                    uint32_t delayUs = (TX_GUARD_US > latency) ? (TX_GUARD_US - latency) : 0;
                    if (delayUs > 0) {
                        delayMicroseconds(delayUs);
                    }
#endif
                    _phy->startTx(freq, buf, len);
                }
            }
        } else if (_currentSlot == Slot::Telemetry) {
            if (_phy) {
                _phy->startRx(freq);
            }
        }
    } else {
        // RX Role
        if (_currentSlot == Slot::Sync || _currentSlot == Slot::Uplink) {
            if (_phy) {
                _phy->startRx(freq);
            }
        } else if (_currentSlot == Slot::Telemetry) {
            if (_link->isLocked()) {
                uint8_t buf[OTA16_LEN];
                uint8_t len = 0;
                uint16_t pos = _link->rxPos();
                if (_link->getTxPayload(tick, pos, buf, len)) {
                    if (_phy) {
#if defined(ARDUINO) || defined(PICO_BOARD)
                        uint32_t latency = _phy->txLatencyUs();
                        uint32_t delayUs = (TX_GUARD_US > latency) ? (TX_GUARD_US - latency) : 0;
                        if (delayUs > 0) {
                            delayMicroseconds(delayUs);
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
        // Unpack, decode, and authenticate through Link.
        uint16_t pos = (_link->role() == Role::Tx) ? _link->txPos(_tick) : _link->rxPos();
        bool success = _link->processRxPayload(_tick, pos, pkt.data, pkt.len, pkt.rssiDbm, pkt.snr);
        if (success) {
            // Apply Phase-Frequency Detector (PFD) correction to timer interval (RX only).
            if (_link->role() == Role::Rx) {
                uint32_t airtime = (pkt.len <= 8) ? _rate.airtime8Us : _rate.airtime16Us;
                // Correct PFD negative feedback sign: offsetUs = actual - expected
#if defined(ARDUINO) || defined(PICO_BOARD)
                int32_t offsetUs = (int32_t)(pkt.timestampUs - airtime - TX_GUARD_US) - (int32_t)_tickStartUs;
#else
                int32_t offsetUs = (int32_t)(pkt.timestampUs - airtime) - (int32_t)_tickStartUs;
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
    const bool ok = _phy->recover();
    if (_link) {
        _link->notePhyRecovery(ok);
    }
    return ok;
}

void RfScheduler::poll() {
    recoverPhyIfNeeded();

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
        for (uint32_t i = 0; i < diff; ++i) {
            uint32_t nextTick = _tick + 1;
            onTick(nextTick);
            if (_link) {
                _link->service(nextTick);
            }
        }
        _lastProcessedTickEvent = currentTicks;
    }

    recoverPhyIfNeeded();
}

} // namespace xlrs
