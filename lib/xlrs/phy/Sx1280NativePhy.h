#pragma once
#include "phy/IRadioPhy.h"
#include "phy/Sx1280Regs.h"
#include "util/RingBuffer.h"
#include <atomic>
#include <pico/types.h>

namespace xlrs {

enum class PhyDiagPhase : uint8_t {
    None,
    SpiGpio,
    Reset,
    Standby,
    Regulator,
    PacketType,
    TxParams,
    BufferBase,
    ModParams,
    SyncWord,
    RxGain,
    PllTune,
    AutoFs,
    DioIrq,
    ClearIrq,
    AttachIrq,
    StartRx,
    StartTx,
    ReadRx,
    SetPower,
    Reconfigure,
    Recover
};

enum class PhyDiagStatus : uint8_t {
    Begin,
    Complete,
    Fail
};

struct PhyDiagEvent {
    uint32_t timestampUs;
    PhyDiagPhase phase;
    PhyDiagStatus status;
    uint8_t opcode;
    uint32_t spiTimeouts;
};

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

    // Fault diagnostics (not on IRadioPhy — concrete extras for bring-up/telemetry). A sticky
    // healthy()=false tells you the radio is wedged; these tell you WHY: how many BUSY-line
    // timeouts and CRC drops have accumulated, and the opcode of the last SPI op that timed out.
    uint32_t spiTimeouts()    const { return _spiTimeouts.load(std::memory_order_relaxed); }
    uint32_t crcErrors()      const { return _crcErrors.load(std::memory_order_relaxed); }
    uint8_t  lastFailOpcode() const { return _lastFailOpcode.load(std::memory_order_relaxed); }
    PhyDiagPhase lastDiagPhase() const {
        return (PhyDiagPhase)_lastDiagPhase.load(std::memory_order_relaxed);
    }
    PhyDiagStatus lastDiagStatus() const {
        return (PhyDiagStatus)_lastDiagStatus.load(std::memory_order_relaxed);
    }
    uint8_t lastStartedOpcode() const {
        return _lastStartedOpcode.load(std::memory_order_relaxed);
    }
    uint8_t lastCompletedOpcode() const {
        return _lastCompletedOpcode.load(std::memory_order_relaxed);
    }
    size_t diagDrops() const { return _diagEvents.dropped(); }
    bool popDiagEvent(PhyDiagEvent& out) { return _diagEvents.pop(out); }
    static const char* diagPhaseName(PhyDiagPhase phase);
    static const char* diagStatusName(PhyDiagStatus status);

private:
    enum class Mode : uint8_t { Idle, Rx, Tx };

#if defined(XLRS_PICO_SDK)
    static void onDio1(uint gpio, uint32_t events); // single DIO1 ISR: TX-done or RX-done
    static Sx1280NativePhy* s_self;    // single radio instance per build

    void resetRadio();
    bool waitBusy();
    void recordHardwareFault(uint8_t opcode);
    void recordDiag(PhyDiagPhase phase, PhyDiagStatus status, uint8_t opcode = 0);
    void beginDiag(PhyDiagPhase phase, uint8_t opcode = 0);
    void completeDiag(PhyDiagPhase phase, uint8_t opcode = 0);
    void spiCommand(uint8_t opcode, const uint8_t* params, uint8_t len);
    bool readCommand(uint8_t opcode, uint8_t* out, uint8_t len); // GET-status read (opcode+dummy+len)
    void clearIrqStatus();
    void writeRegister(uint16_t addr, const uint8_t* data, uint8_t len);
    void readRegister(uint16_t addr, uint8_t* data, uint8_t len);
    void writeBuffer(uint8_t offset, const uint8_t* data, uint8_t len);
    void readBuffer(uint8_t offset, uint8_t* data, uint8_t len);
    void setRfSwitch(Mode mode);
    // Single source of truth for the modulation/packet register blocks (used by
    // init/reconfigure/setSyncWord/startTx) so tuning lives in one place.
    void buildModParams(uint8_t out[3]) const;
    void buildPacketParams(uint8_t out[7], uint8_t payloadLen) const;
#endif

    PhyConfig _cfg{};
    std::atomic<Mode>     _mode{Mode::Idle};
    std::atomic<uint32_t> _lastIrqUs{0};  // HW arrival timestamp latched in the ISR
    std::atomic<bool>     _rxReady{false};
    std::atomic<bool>     _hardwareError{false};
    std::atomic<uint32_t> _spiTimeouts{0};    // BUSY-line timeouts seen
    std::atomic<uint32_t> _crcErrors{0};      // RX frames dropped on CRC mismatch
    std::atomic<uint8_t>  _lastFailOpcode{0}; // opcode of the last SPI op that timed out
    std::atomic<uint8_t>  _lastDiagPhase{(uint8_t)PhyDiagPhase::None};
    std::atomic<uint8_t>  _lastDiagStatus{(uint8_t)PhyDiagStatus::Complete};
    std::atomic<uint8_t>  _lastStartedOpcode{0};
    std::atomic<uint8_t>  _lastCompletedOpcode{0};
    bool                  _diagTraceEnabled = false;
    SpscRing<PhyDiagEvent, 64> _diagEvents{};
    PhyIsrCallback _onTxDone = nullptr;
    PhyIsrCallback _onRxDone = nullptr;
};

} // namespace xlrs
