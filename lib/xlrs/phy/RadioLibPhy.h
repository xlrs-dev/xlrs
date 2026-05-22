// RadioLibPhy — IRadioPhy backed by RadioLib's SX1280 driver, in ASYNC mode.
//
// Backed by RadioLib's SX128x driver, this physical layer handles high-performance FLRC vs LoRa
// configuration, sync-word calls, packet-status reads, the DIO1 timestamp latch, and TX↔RX
// turnaround timing. The whole body is ARDUINO-guarded so the native test build (which
// uses MockPhy) never tries to compile RadioLib.
#pragma once
#include "phy/IRadioPhy.h"
#include <atomic>
#if defined(ARDUINO)
#include <Arduino.h>
#include <RadioLib.h>
#endif

namespace xlrs {

class RadioLibPhy : public IRadioPhy {
public:
    bool init(const PhyConfig& cfg) override;
    void startRx(float freqMHz) override;
    void startTx(float freqMHz, const uint8_t* data, uint8_t len) override;
    bool readRx(RxPacket& out) override;
    void setOutputPowerDbm(int8_t dbm) override;
    void setSyncWord(uint16_t uidDerived) override;
    void reconfigure(const PhyConfig& cfg) override;
    uint32_t txLatencyUs() const override;
    bool healthy() const override { return _healthy.load(std::memory_order_acquire); }
    bool recover() override;
    void setOnTxDone(PhyIsrCallback cb) override { _onTxDone = cb; }
    void setOnRxDone(PhyIsrCallback cb) override { _onRxDone = cb; }

private:
    enum class Mode : uint8_t { Idle, Rx, Tx };

#if defined(ARDUINO)
    static void onDio1();              // single DIO1 ISR: TX-done or RX-done (mode-routed)
    static RadioLibPhy* s_self;        // single radio instance per build
    Module* _module = nullptr;
    SX1280* _radio  = nullptr;
#endif
    PhyConfig _cfg{};
    std::atomic<Mode>     _mode{Mode::Idle};
    std::atomic<uint32_t> _lastIrqUs{0};  // HW arrival timestamp latched in the ISR
    std::atomic<bool>     _rxReady{false};
    std::atomic<bool>     _healthy{false};
    PhyIsrCallback _onTxDone = nullptr;
    PhyIsrCallback _onRxDone = nullptr;
};

} // namespace xlrs
