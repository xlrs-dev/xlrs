#include "phy/Sx1280NativePhy.h"

#if defined(XLRS_PICO_SDK)
#include "hal/Time.h"

#include <cstring>

#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <pico/stdlib.h>

// Default pin configurations matching hardware build flags
#ifndef SX128X_SPI_SCK
#define SX128X_SPI_SCK 18
#endif
#ifndef SX128X_SPI_MOSI
#define SX128X_SPI_MOSI 19
#endif
#ifndef SX128X_SPI_MISO
#define SX128X_SPI_MISO 16
#endif
#ifndef SX128X_SPI_CS
#define SX128X_SPI_CS 17
#endif
#ifndef SX128X_SPI_BUSY
#define SX128X_SPI_BUSY 20
#endif
#ifndef SX128X_SPI_DIO1
#define SX128X_SPI_DIO1 21
#endif
#ifndef SX128X_SPI_RST
#define SX128X_SPI_RST 22
#endif
#ifndef SX128X_RXEN
#define SX128X_RXEN 14
#endif
#ifndef SX128X_TXEN
#define SX128X_TXEN 15
#endif
#ifndef SX128X_SPI_BAUD
#define SX128X_SPI_BAUD 8000000
#endif
#ifndef SX128X_REGULATOR_MODE
#define SX128X_REGULATOR_MODE SX1280_REGULATOR_LDO
#endif

namespace xlrs {

Sx1280NativePhy* Sx1280NativePhy::s_self = nullptr;

namespace {
static spi_inst_t* kRadioSpi = spi0;

// BUSY-line wait bound. The SX1280 holds BUSY for at most a few ms after any command
// (datasheet §10.3); 10 ms is a generous ceiling that still bounds a wedged-chip stall to a
// handful of slots instead of the previous 100 ms (tens of slots at the faster rates).
static constexpr uint32_t BUSY_TIMEOUT_US = 10000;
static constexpr uint8_t SX1280_TIMEOUT_BASE_15_625_US = 0x00;
static constexpr uint16_t SX1280_TIMEOUT_SINGLE_MODE = 0x0000;

static uint8_t spiTransfer(uint8_t value) {
    uint8_t rx = 0;
    spi_write_read_blocking(kRadioSpi, &value, &rx, 1);
    return rx;
}

// frf = freqMHz * 2^18 / F_XTAL. Computed in 64-bit integer math (no soft-float multiply per
// hop on the FPU-less RP2040). FHSS channel centres are integer MHz (channels_2g4.h), so the
// conversion is exact.
static uint32_t freqToReg(float freqMHz) {
    uint32_t mhz = (uint32_t)(freqMHz + 0.5f);
    return (uint32_t)(((uint64_t)mhz * SX1280_FREQ_DIV) / SX1280_XTAL_MHZ);
}

// SetTxParams power byte: Pout = -18 + power, valid register range 0..31 (-18..+13 dBm).
// Clamp so an out-of-range power request can never write an invalid value (datasheet §11.7.4).
static uint8_t txPowerReg(int8_t dbm) {
    int p = (int)dbm + 18;
    if (p < 0) p = 0;
    if (p > 31) p = 31;
    return (uint8_t)p;
}
} // namespace

const char* Sx1280NativePhy::diagPhaseName(PhyDiagPhase phase) {
    switch (phase) {
        case PhyDiagPhase::None: return "None";
        case PhyDiagPhase::SpiGpio: return "SpiGpio";
        case PhyDiagPhase::Reset: return "Reset";
        case PhyDiagPhase::Standby: return "Standby";
        case PhyDiagPhase::Regulator: return "Regulator";
        case PhyDiagPhase::PacketType: return "PacketType";
        case PhyDiagPhase::TxParams: return "TxParams";
        case PhyDiagPhase::BufferBase: return "BufferBase";
        case PhyDiagPhase::ModParams: return "ModParams";
        case PhyDiagPhase::SyncWord: return "SyncWord";
        case PhyDiagPhase::RxGain: return "RxGain";
        case PhyDiagPhase::LoraTune: return "LoraTune";
        case PhyDiagPhase::AutoFs: return "AutoFs";
        case PhyDiagPhase::DioIrq: return "DioIrq";
        case PhyDiagPhase::ClearIrq: return "ClearIrq";
        case PhyDiagPhase::AttachIrq: return "AttachIrq";
        case PhyDiagPhase::StartRx: return "StartRx";
        case PhyDiagPhase::StartTx: return "StartTx";
        case PhyDiagPhase::ReadRx: return "ReadRx";
        case PhyDiagPhase::SetPower: return "SetPower";
        case PhyDiagPhase::Reconfigure: return "Reconfigure";
        case PhyDiagPhase::Recover: return "Recover";
    }
    return "Unknown";
}

const char* Sx1280NativePhy::diagStatusName(PhyDiagStatus status) {
    switch (status) {
        case PhyDiagStatus::Begin: return "begin";
        case PhyDiagStatus::Complete: return "ok";
        case PhyDiagStatus::Fail: return "fail";
    }
    return "unknown";
}

void Sx1280NativePhy::recordDiag(PhyDiagPhase phase, PhyDiagStatus status, uint8_t opcode) {
    _lastDiagPhase.store((uint8_t)phase, std::memory_order_release);
    _lastDiagStatus.store((uint8_t)status, std::memory_order_release);
    if (status == PhyDiagStatus::Begin && opcode != 0) {
        _lastStartedOpcode.store(opcode, std::memory_order_relaxed);
    } else if (status == PhyDiagStatus::Complete && opcode != 0) {
        _lastCompletedOpcode.store(opcode, std::memory_order_relaxed);
    }

    if (_diagTraceEnabled || status == PhyDiagStatus::Fail) {
        PhyDiagEvent event{};
        event.timestampUs = hal::nowUs();
        event.phase = phase;
        event.status = status;
        event.opcode = opcode;
        event.spiTimeouts = _spiTimeouts.load(std::memory_order_relaxed);
        _diagEvents.push(event);
    }
}

void Sx1280NativePhy::beginDiag(PhyDiagPhase phase, uint8_t opcode) {
    recordDiag(phase, PhyDiagStatus::Begin, opcode);
}

void Sx1280NativePhy::completeDiag(PhyDiagPhase phase, uint8_t opcode) {
    recordDiag(phase, PhyDiagStatus::Complete, opcode);
}

bool Sx1280NativePhy::waitBusy(bool recordTimeout) {
    uint32_t start = hal::nowUs();
    while (gpio_get(SX128X_SPI_BUSY)) {
        if (hal::nowUs() - start > BUSY_TIMEOUT_US) {
            if (recordTimeout) {
                _spiTimeouts.fetch_add(1, std::memory_order_relaxed);
            }
            return false;
        }
        hal::sleepUs(1);
    }
    return true;
}

bool Sx1280NativePhy::trySetStandby() {
    if (!waitBusy(false)) return false;
    uint8_t stdbyParam = SX1280_STDBY_RC;
    gpio_put(SX128X_SPI_CS, false);
    hal::sleepUs(2);
    spiTransfer(SX1280_OP_SET_STANDBY);
    spiTransfer(stdbyParam);
    gpio_put(SX128X_SPI_CS, true);
    hal::sleepUs(2);
    return waitBusy(false);
}

void Sx1280NativePhy::recordHardwareFault(uint8_t opcode) {
    _hardwareError.store(true, std::memory_order_release);
    _lastFailOpcode.store(opcode, std::memory_order_relaxed);
    recordDiag(lastDiagPhase(), PhyDiagStatus::Fail, opcode);
}

void Sx1280NativePhy::spiCommand(uint8_t opcode, const uint8_t* params, uint8_t len, bool waitPostBusy) {
    _lastStartedOpcode.store(opcode, std::memory_order_relaxed);
    if (!waitBusy()) {
        recordHardwareFault(opcode);
        return;
    }
    gpio_put(SX128X_SPI_CS, false);
    hal::sleepUs(2);
    spiTransfer(opcode);
    for (uint8_t i = 0; i < len; i++) {
        spiTransfer(params[i]);
    }
    gpio_put(SX128X_SPI_CS, true);
    hal::sleepUs(2);
    if (waitPostBusy && !waitBusy()) {
        recordHardwareFault(opcode);
        return;
    }
    _lastCompletedOpcode.store(opcode, std::memory_order_relaxed);
}

// GET-status reads share one shape: opcode, one dummy status byte, then `len` payload bytes.
// Any BUSY timeout aborts the whole read (returns false) so a caller never interprets bytes
// clocked out of a wedged chip.
bool Sx1280NativePhy::readCommand(uint8_t opcode, uint8_t* out, uint8_t len) {
    _lastStartedOpcode.store(opcode, std::memory_order_relaxed);
    if (!waitBusy()) {
        recordHardwareFault(opcode);
        return false;
    }
    gpio_put(SX128X_SPI_CS, false);
    hal::sleepUs(2);
    spiTransfer(opcode);
    spiTransfer(0x00); // dummy status byte
    for (uint8_t i = 0; i < len; i++) {
        out[i] = spiTransfer(0x00);
    }
    gpio_put(SX128X_SPI_CS, true);
    hal::sleepUs(2);
    if (!waitBusy()) {
        recordHardwareFault(opcode);
        return false;
    }
    _lastCompletedOpcode.store(opcode, std::memory_order_relaxed);
    return true;
}

void Sx1280NativePhy::clearIrqStatus() {
    uint8_t clearParams[2] = { 0xFF, 0xFF };
    spiCommand(SX1280_OP_CLEAR_IRQ_STATUS, clearParams, 2);
}

void Sx1280NativePhy::buildModParams(uint8_t out[3]) const {
    if (_cfg.modulation == Modulation::Lora) {
        out[0] = (uint8_t)(_cfg.sf << 4);
        if (_cfg.bwKHz > 1200.0f)      out[1] = SX1280_LORA_BW_1600;
        else if (_cfg.bwKHz > 600.0f)  out[1] = SX1280_LORA_BW_800;
        else if (_cfg.bwKHz > 300.0f)  out[1] = SX1280_LORA_BW_400;
        else                           out[1] = SX1280_LORA_BW_200;
        out[2] = (uint8_t)(_cfg.cr - 4);
    } else { // FLRC
        out[0] = (_cfg.flrcBitrateKbps < 800) ? SX1280_FLRC_BR_0_650 : SX1280_FLRC_BR_1_300;
        out[1] = (uint8_t)((_cfg.cr - 2) * 2);
        out[2] = SX1280_FLRC_BT_0_5;
    }
}

void Sx1280NativePhy::buildPacketParams(uint8_t out[7], uint8_t payloadLen) const {
    if (_cfg.modulation == Modulation::Lora) {
        out[0] = SX1280_LORA_PREAMBLE_12;
        out[1] = SX1280_LORA_HEADER_IMPLICIT;
        out[2] = payloadLen;
        out[3] = SX1280_LORA_CRC_ON;
        out[4] = SX1280_LORA_IQ_STD;
        out[5] = 0x00;
        out[6] = 0x00;
    } else { // FLRC — sync-match/CRC selectors aligned with SWDR005
        out[0] = SX1280_FLRC_PREAMBLE_16_BITS;
        out[1] = SX1280_FLRC_SYNC_WORD_4_BYTES;
        out[2] = SX1280_FLRC_SYNC_MATCH_1;
        out[3] = SX1280_FLRC_PACKET_FIXED;
        out[4] = payloadLen;
        out[5] = SX1280_FLRC_CRC_2_BYTE;
        out[6] = SX1280_FLRC_WHITENING_OFF;
    }
}

void Sx1280NativePhy::applyLoraModulationTuning() {
    if (_cfg.modulation != Modulation::Lora) return;

    uint8_t sfConfig = SX1280_LORA_SF_CONFIG_SF9_12;
    if (_cfg.sf <= 6) {
        sfConfig = SX1280_LORA_SF_CONFIG_SF5_6;
    } else if (_cfg.sf <= 8) {
        sfConfig = SX1280_LORA_SF_CONFIG_SF7_8;
    }

    writeRegister(SX1280_REG_LORA_SF_CONFIG, &sfConfig, 1);
    if (_hardwareError.load(std::memory_order_acquire)) return;

    uint8_t freqErrorComp = 0;
    readRegister(SX1280_REG_FREQ_ERROR_COMP, &freqErrorComp, 1);
    if (_hardwareError.load(std::memory_order_acquire)) return;

    freqErrorComp |= SX1280_FREQ_ERROR_COMP_ON;
    writeRegister(SX1280_REG_FREQ_ERROR_COMP, &freqErrorComp, 1);
}

void Sx1280NativePhy::writeRegister(uint16_t addr, const uint8_t* data, uint8_t len) {
    _lastStartedOpcode.store(SX1280_OP_WRITE_REGISTER, std::memory_order_relaxed);
    if (!waitBusy()) {
        recordHardwareFault(SX1280_OP_WRITE_REGISTER);
        return;
    }
    gpio_put(SX128X_SPI_CS, false);
    hal::sleepUs(2);
    spiTransfer(SX1280_OP_WRITE_REGISTER);
    spiTransfer((addr >> 8) & 0xFF);
    spiTransfer(addr & 0xFF);
    for (uint8_t i = 0; i < len; i++) {
        spiTransfer(data[i]);
    }
    gpio_put(SX128X_SPI_CS, true);
    hal::sleepUs(2);
    if (!waitBusy()) {
        recordHardwareFault(SX1280_OP_WRITE_REGISTER);
        return;
    }
    _lastCompletedOpcode.store(SX1280_OP_WRITE_REGISTER, std::memory_order_relaxed);
}

void Sx1280NativePhy::readRegister(uint16_t addr, uint8_t* data, uint8_t len) {
    _lastStartedOpcode.store(SX1280_OP_READ_REGISTER, std::memory_order_relaxed);
    if (!waitBusy()) {
        recordHardwareFault(SX1280_OP_READ_REGISTER);
        return;
    }
    gpio_put(SX128X_SPI_CS, false);
    hal::sleepUs(2);
    spiTransfer(SX1280_OP_READ_REGISTER);
    spiTransfer((addr >> 8) & 0xFF);
    spiTransfer(addr & 0xFF);
    spiTransfer(0x00); // dummy status byte
    for (uint8_t i = 0; i < len; i++) {
        data[i] = spiTransfer(0x00);
    }
    gpio_put(SX128X_SPI_CS, true);
    hal::sleepUs(2);
    if (!waitBusy()) {
        recordHardwareFault(SX1280_OP_READ_REGISTER);
        return;
    }
    _lastCompletedOpcode.store(SX1280_OP_READ_REGISTER, std::memory_order_relaxed);
}

void Sx1280NativePhy::writeBuffer(uint8_t offset, const uint8_t* data, uint8_t len) {
    _lastStartedOpcode.store(SX1280_OP_WRITE_BUFFER, std::memory_order_relaxed);
    if (!waitBusy()) {
        recordHardwareFault(SX1280_OP_WRITE_BUFFER);
        return;
    }
    gpio_put(SX128X_SPI_CS, false);
    hal::sleepUs(2);
    spiTransfer(SX1280_OP_WRITE_BUFFER);
    spiTransfer(offset);
    for (uint8_t i = 0; i < len; i++) {
        spiTransfer(data[i]);
    }
    gpio_put(SX128X_SPI_CS, true);
    hal::sleepUs(2);
    if (!waitBusy()) {
        recordHardwareFault(SX1280_OP_WRITE_BUFFER);
        return;
    }
    _lastCompletedOpcode.store(SX1280_OP_WRITE_BUFFER, std::memory_order_relaxed);
}

void Sx1280NativePhy::readBuffer(uint8_t offset, uint8_t* data, uint8_t len) {
    _lastStartedOpcode.store(SX1280_OP_READ_BUFFER, std::memory_order_relaxed);
    if (!waitBusy()) {
        recordHardwareFault(SX1280_OP_READ_BUFFER);
        return;
    }
    gpio_put(SX128X_SPI_CS, false);
    hal::sleepUs(2);
    spiTransfer(SX1280_OP_READ_BUFFER);
    spiTransfer(offset);
    spiTransfer(0x00); // dummy status byte
    for (uint8_t i = 0; i < len; i++) {
        data[i] = spiTransfer(0x00);
    }
    gpio_put(SX128X_SPI_CS, true);
    hal::sleepUs(2);
    if (!waitBusy()) {
        recordHardwareFault(SX1280_OP_READ_BUFFER);
        return;
    }
    _lastCompletedOpcode.store(SX1280_OP_READ_BUFFER, std::memory_order_relaxed);
}

void Sx1280NativePhy::resetRadio() {
    gpio_init(SX128X_SPI_RST);
    gpio_set_dir(SX128X_SPI_RST, GPIO_OUT);
    gpio_put(SX128X_SPI_RST, false);
    hal::sleepMs(1);
    gpio_put(SX128X_SPI_RST, true);
    // Keep RST driven high (RadioLib does not tri-state after reset).

    // Match RadioLib reset(): retry SetStandby until the chip responds or 3 s elapse.
    uint32_t startMs = hal::nowMs();
    while ((uint32_t)(hal::nowMs() - startMs) < 3000) {
        if (trySetStandby()) return;
        hal::sleepMs(10);
    }
    _spiTimeouts.fetch_add(1, std::memory_order_relaxed);
    recordHardwareFault(SX1280_OP_SET_STANDBY);
}

void Sx1280NativePhy::setRfSwitch(Mode mode) {
    if (mode == Mode::Rx) {
        gpio_put(SX128X_RXEN, true);
        gpio_put(SX128X_TXEN, false);
    } else if (mode == Mode::Tx) {
        gpio_put(SX128X_RXEN, false);
        gpio_put(SX128X_TXEN, true);
    } else {
        gpio_put(SX128X_RXEN, false);
        gpio_put(SX128X_TXEN, false);
    }
}

void Sx1280NativePhy::onDio1(uint gpio, uint32_t events) {
    (void)gpio;
    (void)events;
    if (!s_self) return;
    s_self->_lastIrqUs.store(hal::nowUs(), std::memory_order_release);
    Mode mode = s_self->_mode.load(std::memory_order_acquire);
    if (mode == Mode::Rx) {
        s_self->_rxReady.store(true, std::memory_order_release);
        if (s_self->_onRxDone) s_self->_onRxDone();
    } else if (mode == Mode::Tx) {
        s_self->_txInProgress.store(false, std::memory_order_release);
        if (s_self->_onTxDone) s_self->_onTxDone();
    }
}

bool Sx1280NativePhy::init(const PhyConfig& cfg) {
    _cfg = cfg;
    s_self = this;
    _hardwareError.store(false, std::memory_order_release);
    _txInProgress.store(false, std::memory_order_release);
    _spiTimeouts.store(0, std::memory_order_relaxed);
    _lastFailOpcode.store(0, std::memory_order_relaxed);
    _diagTraceEnabled = true;

    beginDiag(PhyDiagPhase::SpiGpio);
    spi_init(kRadioSpi, SX128X_SPI_BAUD);
    spi_set_format(kRadioSpi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(SX128X_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SX128X_SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SX128X_SPI_MISO, GPIO_FUNC_SPI);

    gpio_init(SX128X_SPI_CS);
    gpio_set_dir(SX128X_SPI_CS, GPIO_OUT);
    gpio_put(SX128X_SPI_CS, true);
    gpio_init(SX128X_SPI_BUSY);
    gpio_set_dir(SX128X_SPI_BUSY, GPIO_IN);
    gpio_disable_pulls(SX128X_SPI_BUSY);
    gpio_init(SX128X_SPI_DIO1);
    gpio_set_dir(SX128X_SPI_DIO1, GPIO_IN);
    gpio_init(SX128X_RXEN);
    gpio_set_dir(SX128X_RXEN, GPIO_OUT);
    gpio_init(SX128X_TXEN);
    gpio_set_dir(SX128X_TXEN, GPIO_OUT);
    setRfSwitch(Mode::Idle);
    completeDiag(PhyDiagPhase::SpiGpio);

    beginDiag(PhyDiagPhase::Reset);
    resetRadio();
    if (_hardwareError.load(std::memory_order_acquire)) return false;
    completeDiag(PhyDiagPhase::Reset);

    // 1. Set Standby RC
    beginDiag(PhyDiagPhase::Standby, SX1280_OP_SET_STANDBY);
    uint8_t stdbyParam = SX1280_STDBY_RC;
    spiCommand(SX1280_OP_SET_STANDBY, &stdbyParam, 1);
    if (_hardwareError.load(std::memory_order_acquire)) return false;
    completeDiag(PhyDiagPhase::Standby, SX1280_OP_SET_STANDBY);

    // 2. Set regulator mode. Default to LDO; DC-DC must be opted in only on boards with the
    // required inductor, otherwise FS/Rx/Tx can brown out while STDBY_RC still appears healthy.
    beginDiag(PhyDiagPhase::Regulator, SX1280_OP_SET_REGULATOR_MODE);
    uint8_t regModeParam = SX128X_REGULATOR_MODE;
    spiCommand(SX1280_OP_SET_REGULATOR_MODE, &regModeParam, 1);
    if (_hardwareError.load(std::memory_order_acquire)) return false;
    completeDiag(PhyDiagPhase::Regulator, SX1280_OP_SET_REGULATOR_MODE);

    // 3. Set Packet Type
    beginDiag(PhyDiagPhase::PacketType, SX1280_OP_SET_PACKET_TYPE);
    uint8_t packetType = (cfg.modulation == Modulation::Lora) ? SX1280_PACKET_LORA : SX1280_PACKET_FLRC;
    spiCommand(SX1280_OP_SET_PACKET_TYPE, &packetType, 1);
    if (_hardwareError.load(std::memory_order_acquire)) return false;
    completeDiag(PhyDiagPhase::PacketType, SX1280_OP_SET_PACKET_TYPE);

    // 4. Set TX Params
    beginDiag(PhyDiagPhase::TxParams, SX1280_OP_SET_TX_PARAMS);
    uint8_t txParams[2] = { txPowerReg(cfg.powerDbm), SX1280_RAMP_20_US };
    spiCommand(SX1280_OP_SET_TX_PARAMS, txParams, 2);
    if (_hardwareError.load(std::memory_order_acquire)) return false;
    completeDiag(PhyDiagPhase::TxParams, SX1280_OP_SET_TX_PARAMS);

    // 5. Set Buffer Base Address
    beginDiag(PhyDiagPhase::BufferBase, SX1280_OP_SET_BUFFER_BASE_ADDR);
    uint8_t baseAddr[2] = {0x00, 0x00};
    spiCommand(SX1280_OP_SET_BUFFER_BASE_ADDR, baseAddr, 2);
    if (_hardwareError.load(std::memory_order_acquire)) return false;
    completeDiag(PhyDiagPhase::BufferBase, SX1280_OP_SET_BUFFER_BASE_ADDR);

    // 6. Set modulation parameters
    beginDiag(PhyDiagPhase::ModParams, SX1280_OP_SET_MODULATION_PARAMS);
    uint8_t modParams[3];
    buildModParams(modParams);
    spiCommand(SX1280_OP_SET_MODULATION_PARAMS, modParams, 3);
    if (_hardwareError.load(std::memory_order_acquire)) return false;
    completeDiag(PhyDiagPhase::ModParams, SX1280_OP_SET_MODULATION_PARAMS);

    beginDiag(PhyDiagPhase::LoraTune, SX1280_OP_WRITE_REGISTER);
    applyLoraModulationTuning();
    if (_hardwareError.load(std::memory_order_acquire)) return false;
    completeDiag(PhyDiagPhase::LoraTune, SX1280_OP_WRITE_REGISTER);

    // 7. Set sync word (also sets packet parameters)
    beginDiag(PhyDiagPhase::SyncWord);
    setSyncWord(cfg.syncWord);
    if (_hardwareError.load(std::memory_order_acquire)) return false;
    completeDiag(PhyDiagPhase::SyncWord, _lastCompletedOpcode.load(std::memory_order_relaxed));

    // 7b. High-sensitivity front-end tuning.
    // RxGain field [7:6] = 3 selects high-sensitivity mode (datasheet Table 13-1). Read/modify/
    // write per the §13 note so the register's other live bits are preserved rather than clobbered.
    beginDiag(PhyDiagPhase::RxGain, SX1280_OP_READ_REGISTER);
    uint8_t rxGain = 0;
    readRegister(SX1280_REG_RX_GAIN, &rxGain, 1);
    if (_hardwareError.load(std::memory_order_acquire)) return false;
    rxGain = (uint8_t)((rxGain & SX1280_RX_GAIN_KEEP_MASK) | SX1280_RX_GAIN_HIGH_SENS);
    writeRegister(SX1280_REG_RX_GAIN, &rxGain, 1);
    if (_hardwareError.load(std::memory_order_acquire)) return false;
    completeDiag(PhyDiagPhase::RxGain, SX1280_OP_WRITE_REGISTER);

    // 7c. Enable AutoFS via the documented command (SetAutoFS, opcode 0x9E; 0x01 = enable).
    // Keeps the radio in FS between RX and TX instead of dropping to STDBY_RC, for faster
    // half-duplex turnaround. (The previous writeRegister(0x08F4) was not the documented path.)
    beginDiag(PhyDiagPhase::AutoFs, SX1280_OP_SET_AUTO_FS);
    uint8_t autoFs = 0x01;
    spiCommand(SX1280_OP_SET_AUTO_FS, &autoFs, 1);
    if (_hardwareError.load(std::memory_order_acquire)) return false;
    completeDiag(PhyDiagPhase::AutoFs, SX1280_OP_SET_AUTO_FS);

    // 8. Set DIO IRQ Params: route the IRQ mask to DIO1 (IrqMask + Dio1Mask), nothing to DIO2/3.
    beginDiag(PhyDiagPhase::DioIrq, SX1280_OP_SET_DIO_IRQ_PARAMS);
    uint8_t dioParams[8] = {
        (uint8_t)(SX1280_IRQ_MASK >> 8), (uint8_t)(SX1280_IRQ_MASK & 0xFF), // IrqMask
        (uint8_t)(SX1280_IRQ_MASK >> 8), (uint8_t)(SX1280_IRQ_MASK & 0xFF), // Dio1Mask
        0x00, 0x00,                                                          // Dio2Mask
        0x00, 0x00                                                           // Dio3Mask
    };
    spiCommand(SX1280_OP_SET_DIO_IRQ_PARAMS, dioParams, 8);
    if (_hardwareError.load(std::memory_order_acquire)) return false;
    completeDiag(PhyDiagPhase::DioIrq, SX1280_OP_SET_DIO_IRQ_PARAMS);

    // 9. Clear any pending interrupts
    beginDiag(PhyDiagPhase::ClearIrq, SX1280_OP_CLEAR_IRQ_STATUS);
    clearIrqStatus();

    if (_hardwareError.load(std::memory_order_acquire)) return false;
    completeDiag(PhyDiagPhase::ClearIrq, SX1280_OP_CLEAR_IRQ_STATUS);

    // 10. Attach interrupt (RX/TX armed by the scheduler on the first tick).
    beginDiag(PhyDiagPhase::AttachIrq);
    gpio_set_irq_enabled_with_callback(SX128X_SPI_DIO1, GPIO_IRQ_EDGE_RISE, true, onDio1);
    completeDiag(PhyDiagPhase::AttachIrq);

    _diagTraceEnabled = false;
    return !_hardwareError.load(std::memory_order_acquire);
}

void Sx1280NativePhy::startRx(float freqMHz) {
    if (_hardwareError.load(std::memory_order_acquire) ||
        _txInProgress.load(std::memory_order_acquire)) {
        return;
    }
    beginDiag(PhyDiagPhase::StartRx, SX1280_OP_SET_RX);
    _mode.store(Mode::Rx, std::memory_order_release);
    _rxReady.store(false, std::memory_order_release);

    // 1. Standby
    uint8_t stdbyParam = SX1280_STDBY_RC;
    spiCommand(SX1280_OP_SET_STANDBY, &stdbyParam, 1);

    // 2. Set RF Switch to RX
    setRfSwitch(Mode::Rx);

    // 3. Set Frequency
    uint32_t frf = freqToReg(freqMHz);
    uint8_t freqParams[3] = {
        (uint8_t)((frf >> 16) & 0xFF),
        (uint8_t)((frf >> 8) & 0xFF),
        (uint8_t)(frf & 0xFF)
    };
    spiCommand(SX1280_OP_SET_RF_FREQUENCY, freqParams, 3);

    // 4. Restore fixed air length — a prior startTx() may have shortened packet params.
    uint8_t packetParams[7];
    buildPacketParams(packetParams, _cfg.payloadLen);
    spiCommand(SX1280_OP_SET_PACKET_PARAMS, packetParams, 7);

    // 5. Clear Interrupts
    clearIrqStatus();

    // 6. Issue SetRx in single mode. The scheduler re-arms RX for every receive slot, and
    // Semtech errata 16.1 warns that continuous RX (0xFFFF) can wedge BUSY high under high
    // co-channel packet rates.
    uint8_t rxParams[3] = {
        SX1280_TIMEOUT_BASE_15_625_US,
        (uint8_t)(SX1280_TIMEOUT_SINGLE_MODE >> 8),
        (uint8_t)SX1280_TIMEOUT_SINGLE_MODE
    };
    // Skip post-BUSY wait — the chip may hold BUSY through RX entry; DIO1 marks completion.
    spiCommand(SX1280_OP_SET_RX, rxParams, 3, false);
    if (!_hardwareError.load(std::memory_order_acquire)) {
        completeDiag(PhyDiagPhase::StartRx, SX1280_OP_SET_RX);
    }
}

void Sx1280NativePhy::startTx(float freqMHz, const uint8_t* data, uint8_t len) {
    if (_hardwareError.load(std::memory_order_acquire) ||
        _txInProgress.load(std::memory_order_acquire)) {
        return;
    }
    beginDiag(PhyDiagPhase::StartTx, SX1280_OP_SET_TX);
    _mode.store(Mode::Tx, std::memory_order_release);

    // 1. Standby
    uint8_t stdbyParam = SX1280_STDBY_RC;
    spiCommand(SX1280_OP_SET_STANDBY, &stdbyParam, 1);

    // 2. Set RF Switch to TX
    setRfSwitch(Mode::Tx);

    // 3. Set Frequency
    uint32_t frf = freqToReg(freqMHz);
    uint8_t freqParams[3] = {
        (uint8_t)((frf >> 16) & 0xFF),
        (uint8_t)((frf >> 8) & 0xFF),
        (uint8_t)(frf & 0xFF)
    };
    spiCommand(SX1280_OP_SET_RF_FREQUENCY, freqParams, 3);

    // 4. Pad to the configured fixed air length (FLRC fixed-packet mode on both ends).
    const uint8_t airLen = _cfg.payloadLen;
    uint8_t padded[32] = {};
    if (len > airLen) len = airLen;
    if (data && len > 0) {
        memcpy(padded, data, len);
    }
    writeBuffer(0x00, padded, airLen);

    // 5. Packet params always use the nominal OTA length, not the logical frame size.
    uint8_t packetParams[7];
    buildPacketParams(packetParams, airLen);
    spiCommand(SX1280_OP_SET_PACKET_PARAMS, packetParams, 7);

    // 6. Clear Interrupts
    clearIrqStatus();

    // 7. Issue SetTx (0x00, 0x00, 0x00). Return once accepted — TX_DONE arrives on DIO1.
    _txInProgress.store(true, std::memory_order_release);
    uint8_t txParams[3] = { 0x00, 0x00, 0x00 };
    spiCommand(SX1280_OP_SET_TX, txParams, 3, false);
    if (_hardwareError.load(std::memory_order_acquire)) {
        _txInProgress.store(false, std::memory_order_release);
    } else {
        completeDiag(PhyDiagPhase::StartTx, SX1280_OP_SET_TX);
    }
}

bool Sx1280NativePhy::readRx(RxPacket& out) {
    if (!_rxReady.load(std::memory_order_acquire)) return false;
    beginDiag(PhyDiagPhase::ReadRx, SX1280_OP_GET_IRQ_STATUS);
    _rxReady.store(false, std::memory_order_release);

    // 1. Get IRQ Status. (Any SPI fault in these reads returns false — consistent contract.)
    uint8_t irqData[2] = {0, 0};
    if (!readCommand(SX1280_OP_GET_IRQ_STATUS, irqData, 2)) return false;
    uint16_t irq = ((uint16_t)irqData[0] << 8) | irqData[1];

    if (irq & SX1280_IRQ_CRC_ERROR) {
        _crcErrors.fetch_add(1, std::memory_order_relaxed);
        clearIrqStatus();
        return false;
    }

    // 2. Get Rx Buffer Status to find starting offset and length.
    uint8_t rxBufStatus[2] = {0, 0};
    if (!readCommand(SX1280_OP_GET_RX_BUFFER_STATUS, rxBufStatus, 2)) return false;

    uint8_t rxLen = rxBufStatus[0];
    uint8_t rxOffset = rxBufStatus[1];

    // In LoRa implicit header mode, rxBufStatus[0] might be 0, so fall back to configured payloadLen.
    if (_cfg.modulation == Modulation::Lora || rxLen == 0) {
        rxLen = _cfg.payloadLen;
    }
    if (rxLen > sizeof(out.data)) {
        rxLen = sizeof(out.data);
    }
    out.len = rxLen;

    // 3. Read packet data from the FIFO buffer.
    readBuffer(rxOffset, out.data, rxLen);

    // 4. Get Packet Status (RSSI and SNR). Conversion is integer-only (no soft-float on M0+).
    uint8_t packetStatus[5] = {0};
    if (!readCommand(SX1280_OP_GET_PACKET_STATUS, packetStatus, 5)) return false;

    if (_cfg.modulation == Modulation::Lora) {
        uint8_t rssiSync = packetStatus[0];
        uint8_t snrRaw   = packetStatus[1];
        int16_t snrQuarterDb = (snrRaw < 128) ? (int16_t)snrRaw : (int16_t)snrRaw - 256; // signed ¼ dB
        int16_t snrDb   = (int16_t)(snrQuarterDb / 4);
        int16_t rssiDbm = (int16_t)(-(int16_t)rssiSync / 2);  // -0.5 dB/LSB
        // Negative SNR ⇒ packet sits below the noise floor; correct RSSI per datasheet §11.7.6.
        out.rssiDbm = (snrQuarterDb <= 0) ? (int16_t)(rssiDbm - snrDb) : rssiDbm;
        out.snr = (int8_t)snrDb;
    } else { // FLRC
        out.rssiDbm = (int16_t)(-(int16_t)packetStatus[1] / 2);
        out.snr = 0;
    }

    out.timestampUs = _lastIrqUs.load(std::memory_order_acquire);

    clearIrqStatus();
    if (!_hardwareError.load(std::memory_order_acquire)) {
        completeDiag(PhyDiagPhase::ReadRx, SX1280_OP_CLEAR_IRQ_STATUS);
    }
    return !_hardwareError.load(std::memory_order_acquire);
}

void Sx1280NativePhy::setOutputPowerDbm(int8_t dbm) {
    if (_hardwareError.load(std::memory_order_acquire) ||
        _txInProgress.load(std::memory_order_acquire)) {
        return;
    }
    beginDiag(PhyDiagPhase::SetPower, SX1280_OP_SET_TX_PARAMS);
    _cfg.powerDbm = dbm;
    uint8_t txParams[2] = { txPowerReg(dbm), SX1280_RAMP_20_US };
    spiCommand(SX1280_OP_SET_TX_PARAMS, txParams, 2);
    if (!_hardwareError.load(std::memory_order_acquire)) {
        completeDiag(PhyDiagPhase::SetPower, SX1280_OP_SET_TX_PARAMS);
    }
}

bool Sx1280NativePhy::setSyncWord(uint16_t uidDerived) {
    if (_hardwareError.load(std::memory_order_acquire) ||
        _txInProgress.load(std::memory_order_acquire)) {
        return false;
    }

    // Register writes require standby; scheduler may call this while init left us in RX.
    const Mode prevMode = _mode.load(std::memory_order_acquire);
    if (prevMode != Mode::Idle) {
        uint8_t stdbyParam = SX1280_STDBY_RC;
        spiCommand(SX1280_OP_SET_STANDBY, &stdbyParam, 1);
        if (_hardwareError.load(std::memory_order_acquire)) return false;
    }

    if (_cfg.modulation == Modulation::Flrc) {
        // Datasheet §16.4: with a FLRC sync word at CR 1/2, sync words whose top 16 bits are
        // 0x8C38 or 0x630E resemble the FLRC preamble and cause elevated PER. Those top 16 bits
        // are uidDerived (see byte order below), so remap just those two values. The same
        // uidDerived is derived on TX and RX, so this stays binding-consistent. (CR 3/4 adds
        // LSB constraints; all XLRS FLRC rates use CR 1/2.)
        uint16_t flrcSync = uidDerived;
        if (flrcSync == 0x8C38 || flrcSync == 0x630E) flrcSync ^= 0x0001;
        uint8_t sync[4] = {
            (uint8_t)(flrcSync >> 8),     // SyncWord1[31:24] @ 0x09CF
            (uint8_t)(flrcSync),          // SyncWord1[23:16] @ 0x09D0
            (uint8_t)(~(flrcSync >> 8)),  // SyncWord1[15:8]  @ 0x09D1
            (uint8_t)(~flrcSync)          // SyncWord1[7:0]   @ 0x09D2
        };
        // FLRC requires 4-byte write to 0x09CF (Table 14-42)
        writeRegister(SX1280_REG_FLRC_SYNC_WORD, sync, 4);

        // Re-apply modulation + packet params (syncWordLen = 4, syncWordMatch = 1) via the
        // shared builders so the FLRC tuning lives in exactly one place.
        uint8_t modParams[3];
        buildModParams(modParams);
        spiCommand(SX1280_OP_SET_MODULATION_PARAMS, modParams, 3);

        uint8_t packetParams[7];
        buildPacketParams(packetParams, _cfg.payloadLen);
        spiCommand(SX1280_OP_SET_PACKET_PARAMS, packetParams, 7);
    } else {
        uint8_t syncWord = uidDerived & 0xFF;
        uint8_t controlBits = 0x44;
        uint8_t data[2] = {
            (uint8_t)((syncWord & 0xF0) | ((controlBits & 0xF0) >> 4)),
            (uint8_t)(((syncWord & 0x0F) << 4) | (controlBits & 0x0F))
        };
        writeRegister(SX1280_REG_LORA_SYNC_WORD, data, 2);

        uint8_t packetParams[7];
        buildPacketParams(packetParams, _cfg.payloadLen);
        spiCommand(SX1280_OP_SET_PACKET_PARAMS, packetParams, 7);
    }

    if (prevMode == Mode::Rx && !_hardwareError.load(std::memory_order_acquire)) {
        startRx(_cfg.freqMHz);
    }
    return !_hardwareError.load(std::memory_order_acquire);
}

void Sx1280NativePhy::reconfigure(const PhyConfig& cfg) {
    beginDiag(PhyDiagPhase::Reconfigure);
    _cfg = cfg;
    
    // Set standby first to be safe
    uint8_t stdbyParam = SX1280_STDBY_RC;
    spiCommand(SX1280_OP_SET_STANDBY, &stdbyParam, 1);

    // Set Packet Type
    uint8_t packetType = (cfg.modulation == Modulation::Lora) ? SX1280_PACKET_LORA : SX1280_PACKET_FLRC;
    spiCommand(SX1280_OP_SET_PACKET_TYPE, &packetType, 1);

    // Set modulation parameters
    uint8_t modParams[3];
    buildModParams(modParams);
    spiCommand(SX1280_OP_SET_MODULATION_PARAMS, modParams, 3);
    applyLoraModulationTuning();

    // Set sync word and packet parameters
    setSyncWord(cfg.syncWord);

    // Clear any pending interrupts
    clearIrqStatus();

    // Restart Rx at the new frequency
    startRx(cfg.freqMHz);
    if (!_hardwareError.load(std::memory_order_acquire)) {
        completeDiag(PhyDiagPhase::Reconfigure);
    }
}

uint32_t Sx1280NativePhy::txLatencyUs() const {
    return 55;
}

bool Sx1280NativePhy::txInProgress() const {
    return _txInProgress.load(std::memory_order_acquire);
}

bool Sx1280NativePhy::recover() {
    beginDiag(PhyDiagPhase::Recover);
    PhyConfig cfg = _cfg;
    gpio_set_irq_enabled(SX128X_SPI_DIO1, GPIO_IRQ_EDGE_RISE, false);
    _rxReady.store(false, std::memory_order_release);
    _txInProgress.store(false, std::memory_order_release);
    _mode.store(Mode::Idle, std::memory_order_release);
    const bool ok = init(cfg);
    if (ok) {
        completeDiag(PhyDiagPhase::Recover);
    }
    return ok;
}

} // namespace xlrs

#else

namespace xlrs {
bool Sx1280NativePhy::init(const PhyConfig& cfg) { return false; }
void Sx1280NativePhy::startRx(float freqMHz) {}
void Sx1280NativePhy::startTx(float freqMHz, const uint8_t* data, uint8_t len) {}
bool Sx1280NativePhy::readRx(RxPacket& out) { return false; }
void Sx1280NativePhy::setOutputPowerDbm(int8_t dbm) {}
bool Sx1280NativePhy::setSyncWord(uint16_t uidDerived) { (void)uidDerived; return true; }
void Sx1280NativePhy::reconfigure(const PhyConfig& cfg) {}
uint32_t Sx1280NativePhy::txLatencyUs() const { return 0; }
bool Sx1280NativePhy::txInProgress() const { return false; }
bool Sx1280NativePhy::recover() { return false; }
}

#endif
