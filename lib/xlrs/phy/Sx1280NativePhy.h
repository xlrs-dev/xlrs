#pragma once
#include "phy/IRadioPhy.h"
#include "phy/Sx1280Regs.h"
#include <atomic>

#if defined(ARDUINO)
#include <Arduino.h>
#endif

namespace xlrs {

class Sx1280NativePhy : public IRadioPhy {
public:
    Sx1280NativePhy() = default;
    ~Sx1280NativePhy() override = default;

    bool init(const PhyConfig& cfg) override;

    // Async I/O — return immediately; freq passed per-op so the hop is atomic with arming.
    void startRx(float freqMHz) override;
    void startTx(float freqMHz, const uint8_t* data, uint8_t len) override;

    // Drain the last received packet. Called from the RF task (NOT the ISR).
    bool readRx(RxPacket& out) override;

    // Cheap runtime controls.
    void setOutputPowerDbm(int8_t dbm) override;
    void setSyncWord(uint16_t uidDerived) override;
    void reconfigure(const PhyConfig& cfg) override;
    uint32_t txLatencyUs() const override;
    bool healthy() const override { return !_hardwareError.load(std::memory_order_acquire); }
    bool recover() override;

    // Tiny ISR-context completion hooks.
    void setOnTxDone(PhyIsrCallback cb) override { _onTxDone = cb; }
    void setOnRxDone(PhyIsrCallback cb) override { _onRxDone = cb; }

private:
    enum class Mode : uint8_t { Idle, Rx, Tx };

#if defined(ARDUINO)
    static void onDio1();              // single DIO1 ISR: TX-done or RX-done (mode-routed)
    static Sx1280NativePhy* s_self;    // single radio instance per build

    void resetRadio();
    bool waitBusy();
    void spiCommand(uint8_t opcode, const uint8_t* params, uint8_t len);
    void writeRegister(uint16_t addr, const uint8_t* data, uint8_t len);
    void readRegister(uint16_t addr, uint8_t* data, uint8_t len);
    void writeBuffer(uint8_t offset, const uint8_t* data, uint8_t len);
    void readBuffer(uint8_t offset, uint8_t* data, uint8_t len);
    void setRfSwitch(Mode mode);
#endif

    PhyConfig _cfg{};
    std::atomic<Mode>     _mode{Mode::Idle};
    std::atomic<uint32_t> _lastIrqUs{0};  // HW arrival timestamp latched in the ISR
    std::atomic<bool>     _rxReady{false};
    std::atomic<bool>     _hardwareError{false};
    PhyIsrCallback _onTxDone = nullptr;
    PhyIsrCallback _onRxDone = nullptr;
};

} // namespace xlrs
