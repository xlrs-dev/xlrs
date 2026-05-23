#include "phy/Sx1280NativePhy.h"

#if defined(XLRS_PICO_SDK)
#include "hal/Time.h"

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

namespace xlrs {

Sx1280NativePhy* Sx1280NativePhy::s_self = nullptr;

namespace {
static spi_inst_t* kRadioSpi = spi0;

// BUSY-line wait bound. The SX1280 holds BUSY for at most a few ms after any command
// (datasheet §10.3); 10 ms is a generous ceiling that still bounds a wedged-chip stall to a
// handful of slots instead of the previous 100 ms (tens of slots at the faster rates).
static constexpr uint32_t BUSY_TIMEOUT_US = 10000;

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

bool Sx1280NativePhy::waitBusy() {
    uint32_t start = hal::nowUs();
    while (gpio_get(SX128X_SPI_BUSY)) {
        if (hal::nowUs() - start > BUSY_TIMEOUT_US) {
            _spiTimeouts.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        hal::sleepUs(1);
    }
    return true;
}

void Sx1280NativePhy::spiCommand(uint8_t opcode, const uint8_t* params, uint8_t len) {
    if (!waitBusy()) {
        _hardwareError.store(true, std::memory_order_release);
        _lastFailOpcode.store(opcode, std::memory_order_relaxed);
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
    if (!waitBusy()) {
        _hardwareError.store(true, std::memory_order_release);
        _lastFailOpcode.store(opcode, std::memory_order_relaxed);
    }
}

// GET-status reads share one shape: opcode, one dummy status byte, then `len` payload bytes.
// Any BUSY timeout aborts the whole read (returns false) so a caller never interprets bytes
// clocked out of a wedged chip.
bool Sx1280NativePhy::readCommand(uint8_t opcode, uint8_t* out, uint8_t len) {
    if (!waitBusy()) {
        _hardwareError.store(true, std::memory_order_release);
        _lastFailOpcode.store(opcode, std::memory_order_relaxed);
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
        _hardwareError.store(true, std::memory_order_release);
        _lastFailOpcode.store(opcode, std::memory_order_relaxed);
        return false;
    }
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

void Sx1280NativePhy::writeRegister(uint16_t addr, const uint8_t* data, uint8_t len) {
    if (!waitBusy()) {
        _hardwareError.store(true, std::memory_order_release);
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
        _hardwareError.store(true, std::memory_order_release);
    }
}

void Sx1280NativePhy::readRegister(uint16_t addr, uint8_t* data, uint8_t len) {
    if (!waitBusy()) {
        _hardwareError.store(true, std::memory_order_release);
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
        _hardwareError.store(true, std::memory_order_release);
    }
}

void Sx1280NativePhy::writeBuffer(uint8_t offset, const uint8_t* data, uint8_t len) {
    if (!waitBusy()) {
        _hardwareError.store(true, std::memory_order_release);
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
        _hardwareError.store(true, std::memory_order_release);
    }
}

void Sx1280NativePhy::readBuffer(uint8_t offset, uint8_t* data, uint8_t len) {
    if (!waitBusy()) {
        _hardwareError.store(true, std::memory_order_release);
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
        _hardwareError.store(true, std::memory_order_release);
    }
}

void Sx1280NativePhy::resetRadio() {
    gpio_init(SX128X_SPI_RST);
    gpio_set_dir(SX128X_SPI_RST, GPIO_OUT);
    gpio_put(SX128X_SPI_RST, false);
    hal::sleepMs(10);
    gpio_put(SX128X_SPI_RST, true);
    hal::sleepMs(10);
    gpio_set_dir(SX128X_SPI_RST, GPIO_IN);
    if (!waitBusy()) {
        _hardwareError.store(true, std::memory_order_release);
    }
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
        if (s_self->_onTxDone) s_self->_onTxDone();
    }
}

bool Sx1280NativePhy::init(const PhyConfig& cfg) {
    _cfg = cfg;
    s_self = this;
    _hardwareError.store(false, std::memory_order_release); // reset state

    spi_init(kRadioSpi, 10000000);
    gpio_set_function(SX128X_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SX128X_SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SX128X_SPI_MISO, GPIO_FUNC_SPI);

    gpio_init(SX128X_SPI_CS);
    gpio_set_dir(SX128X_SPI_CS, GPIO_OUT);
    gpio_put(SX128X_SPI_CS, true);
    gpio_init(SX128X_SPI_BUSY);
    gpio_set_dir(SX128X_SPI_BUSY, GPIO_IN);
    gpio_init(SX128X_SPI_DIO1);
    gpio_set_dir(SX128X_SPI_DIO1, GPIO_IN);
    gpio_init(SX128X_RXEN);
    gpio_set_dir(SX128X_RXEN, GPIO_OUT);
    gpio_init(SX128X_TXEN);
    gpio_set_dir(SX128X_TXEN, GPIO_OUT);
    setRfSwitch(Mode::Idle);

    resetRadio();
    if (_hardwareError.load(std::memory_order_acquire)) return false;

    // 1. Set Standby RC
    uint8_t stdbyParam = SX1280_STDBY_RC;
    spiCommand(SX1280_OP_SET_STANDBY, &stdbyParam, 1);

    // 2. Set Regulator Mode to DC-DC
    uint8_t regModeParam = SX1280_REGULATOR_DCDC;
    spiCommand(SX1280_OP_SET_REGULATOR_MODE, &regModeParam, 1);

    // 3. Set Packet Type
    uint8_t packetType = (cfg.modulation == Modulation::Lora) ? SX1280_PACKET_LORA : SX1280_PACKET_FLRC;
    spiCommand(SX1280_OP_SET_PACKET_TYPE, &packetType, 1);

    // 4. Set TX Params
    uint8_t txParams[2] = { txPowerReg(cfg.powerDbm), SX1280_RAMP_20_US };
    spiCommand(SX1280_OP_SET_TX_PARAMS, txParams, 2);

    // 5. Set Buffer Base Address
    uint8_t baseAddr[2] = {0x00, 0x00};
    spiCommand(SX1280_OP_SET_BUFFER_BASE_ADDR, baseAddr, 2);

    // 6. Set modulation parameters
    uint8_t modParams[3];
    buildModParams(modParams);
    spiCommand(SX1280_OP_SET_MODULATION_PARAMS, modParams, 3);

    // 7. Set sync word (also sets packet parameters)
    setSyncWord(cfg.syncWord);

    // 7b. High-sensitivity front-end tuning.
    // RxGain field [7:6] = 3 selects high-sensitivity mode (datasheet Table 13-1). Read/modify/
    // write per the §13 note so the register's other live bits are preserved rather than clobbered.
    uint8_t rxGain = 0;
    readRegister(SX1280_REG_RX_GAIN, &rxGain, 1);
    rxGain = (uint8_t)((rxGain & SX1280_RX_GAIN_KEEP_MASK) | SX1280_RX_GAIN_HIGH_SENS);
    writeRegister(SX1280_REG_RX_GAIN, &rxGain, 1);

    uint8_t pllTune = 0x08;
    writeRegister(SX1280_REG_PLL_TUNE, &pllTune, 1); // vendor PLL tuning (undocumented register)

    // 7c. Enable AutoFS via the documented command (SetAutoFS, opcode 0x9E; 0x01 = enable).
    // Keeps the radio in FS between RX and TX instead of dropping to STDBY_RC, for faster
    // half-duplex turnaround. (The previous writeRegister(0x08F4) was not the documented path.)
    uint8_t autoFs = 0x01;
    spiCommand(SX1280_OP_SET_AUTO_FS, &autoFs, 1);

    // 8. Set DIO IRQ Params: route the IRQ mask to DIO1 (IrqMask + Dio1Mask), nothing to DIO2/3.
    uint8_t dioParams[8] = {
        (uint8_t)(SX1280_IRQ_MASK >> 8), (uint8_t)(SX1280_IRQ_MASK & 0xFF), // IrqMask
        (uint8_t)(SX1280_IRQ_MASK >> 8), (uint8_t)(SX1280_IRQ_MASK & 0xFF), // Dio1Mask
        0x00, 0x00,                                                          // Dio2Mask
        0x00, 0x00                                                           // Dio3Mask
    };
    spiCommand(SX1280_OP_SET_DIO_IRQ_PARAMS, dioParams, 8);

    // 9. Clear any pending interrupts
    clearIrqStatus();

    if (_hardwareError.load(std::memory_order_acquire)) return false;

    // 10. Attach interrupt
    gpio_set_irq_enabled_with_callback(SX128X_SPI_DIO1, GPIO_IRQ_EDGE_RISE, true, onDio1);

    startRx(cfg.freqMHz);
    return !_hardwareError.load(std::memory_order_acquire);
}

void Sx1280NativePhy::startRx(float freqMHz) {
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

    // 4. Clear Interrupts
    clearIrqStatus();

    // 5. Issue SetRx with continuous timeout (0x00, 0xFF, 0xFF)
    uint8_t rxParams[3] = { 0x00, 0xFF, 0xFF };
    spiCommand(SX1280_OP_SET_RX, rxParams, 3);
}

void Sx1280NativePhy::startTx(float freqMHz, const uint8_t* data, uint8_t len) {
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

    // 4. Write data to Buffer at offset 0
    writeBuffer(0x00, data, len);

    // 5. Update Packet Params with the actual payload length
    uint8_t packetParams[7];
    buildPacketParams(packetParams, len);
    spiCommand(SX1280_OP_SET_PACKET_PARAMS, packetParams, 7);

    // 6. Clear Interrupts
    clearIrqStatus();

    // 7. Issue SetTx (0x00, 0x00, 0x00)
    uint8_t txParams[3] = { 0x00, 0x00, 0x00 };
    spiCommand(SX1280_OP_SET_TX, txParams, 3);
}

bool Sx1280NativePhy::readRx(RxPacket& out) {
    if (!_rxReady.load(std::memory_order_acquire)) return false;
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
    return !_hardwareError.load(std::memory_order_acquire);
}

void Sx1280NativePhy::setOutputPowerDbm(int8_t dbm) {
    _cfg.powerDbm = dbm;
    uint8_t txParams[2] = { txPowerReg(dbm), SX1280_RAMP_20_US };
    spiCommand(SX1280_OP_SET_TX_PARAMS, txParams, 2);
}

void Sx1280NativePhy::setSyncWord(uint16_t uidDerived) {
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
}

void Sx1280NativePhy::reconfigure(const PhyConfig& cfg) {
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

    // Set sync word and packet parameters
    setSyncWord(cfg.syncWord);

    // Clear any pending interrupts
    clearIrqStatus();

    // Restart Rx at the new frequency
    startRx(cfg.freqMHz);
}

uint32_t Sx1280NativePhy::txLatencyUs() const {
    return 55;
}

bool Sx1280NativePhy::recover() {
    PhyConfig cfg = _cfg;
    gpio_set_irq_enabled(SX128X_SPI_DIO1, GPIO_IRQ_EDGE_RISE, false);
    _rxReady.store(false, std::memory_order_release);
    _mode.store(Mode::Idle, std::memory_order_release);
    return init(cfg);
}

} // namespace xlrs

#else

namespace xlrs {
bool Sx1280NativePhy::init(const PhyConfig& cfg) { return false; }
void Sx1280NativePhy::startRx(float freqMHz) {}
void Sx1280NativePhy::startTx(float freqMHz, const uint8_t* data, uint8_t len) {}
bool Sx1280NativePhy::readRx(RxPacket& out) { return false; }
void Sx1280NativePhy::setOutputPowerDbm(int8_t dbm) {}
void Sx1280NativePhy::setSyncWord(uint16_t uidDerived) {}
void Sx1280NativePhy::reconfigure(const PhyConfig& cfg) {}
uint32_t Sx1280NativePhy::txLatencyUs() const { return 0; }
bool Sx1280NativePhy::recover() { return false; }
}

#endif
