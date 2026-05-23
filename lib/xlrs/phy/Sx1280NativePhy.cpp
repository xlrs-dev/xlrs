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

static uint8_t spiTransfer(uint8_t value) {
    uint8_t rx = 0;
    spi_write_read_blocking(kRadioSpi, &value, &rx, 1);
    return rx;
}
} // namespace

bool Sx1280NativePhy::waitBusy() {
    uint32_t start = hal::nowUs();
    while (gpio_get(SX128X_SPI_BUSY)) {
        if (hal::nowUs() - start > 100000) { // 100ms timeout
            return false;
        }
        hal::sleepUs(1);
    }
    return true;
}

void Sx1280NativePhy::spiCommand(uint8_t opcode, const uint8_t* params, uint8_t len) {
    if (!waitBusy()) {
        _hardwareError.store(true, std::memory_order_release);
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
    uint8_t txParams[2] = {
        (uint8_t)(cfg.powerDbm + 18),
        0xE0 // 24 us ramp time
    };
    spiCommand(SX1280_OP_SET_TX_PARAMS, txParams, 2);

    // 5. Set Buffer Base Address
    uint8_t baseAddr[2] = {0x00, 0x00};
    spiCommand(SX1280_OP_SET_BUFFER_BASE_ADDR, baseAddr, 2);

    // 6. Set modulation parameters
    uint8_t modParams[3];
    if (cfg.modulation == Modulation::Lora) {
        modParams[0] = cfg.sf << 4;
        uint8_t bwByte = 0x18;
        if (cfg.bwKHz > 1200.0f) bwByte = 0x0A;
        else if (cfg.bwKHz > 600.0f) bwByte = 0x18;
        else if (cfg.bwKHz > 300.0f) bwByte = 0x26;
        else bwByte = 0x34;
        modParams[1] = bwByte;
        modParams[2] = cfg.cr - 4;
    } else { // FLRC
        uint8_t brBwByte = 0x45; // default 1.3 Mbps
        if (cfg.flrcBitrateKbps < 800) {
            brBwByte = 0x8A; // 650 kbps
          }
        modParams[0] = brBwByte;
        modParams[1] = (cfg.cr - 2) * 2;
        modParams[2] = 0x20; // BT 0.5
    }
    spiCommand(SX1280_OP_SET_MODULATION_PARAMS, modParams, 3);

    // 7. Set sync word (also sets packet parameters)
    setSyncWord(cfg.syncWord);

    // 7b. Apply professional Semtech high-sensitivity calibrations & AutoFS optimization
    uint8_t lnaTrim = 0xC0;
    writeRegister(0x0891, &lnaTrim, 1); // Maximize front-end LNA gain under weak signals

    uint8_t pllTune = 0x08;
    writeRegister(0x0930, &pllTune, 1); // Fine-tune PLL phase noise and speed up lock acquisition

    uint8_t autoFs = 0x01;
    writeRegister(0x08F4, &autoFs, 1);  // Enable AutoFS (Frequency Synthesis turnaround mode)

    // 8. Set DIO IRQ Params
    uint8_t dioParams[8] = { 0x40, 0x43, 0x40, 0x43, 0x00, 0x00, 0x00, 0x00 };
    spiCommand(SX1280_OP_SET_DIO_IRQ_PARAMS, dioParams, 8);

    // 9. Clear any pending interrupts
    uint8_t clearParams[2] = { 0xFF, 0xFF };
    spiCommand(SX1280_OP_CLEAR_IRQ_STATUS, clearParams, 2);

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
    uint32_t frf = (uint32_t)(freqMHz * 5041.23076923f);
    uint8_t freqParams[3] = {
        (uint8_t)((frf >> 16) & 0xFF),
        (uint8_t)((frf >> 8) & 0xFF),
        (uint8_t)(frf & 0xFF)
    };
    spiCommand(SX1280_OP_SET_RF_FREQUENCY, freqParams, 3);

    // 4. Clear Interrupts
    uint8_t clearParams[2] = { 0xFF, 0xFF };
    spiCommand(SX1280_OP_CLEAR_IRQ_STATUS, clearParams, 2);

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
    uint32_t frf = (uint32_t)(freqMHz * 5041.23076923f);
    uint8_t freqParams[3] = {
        (uint8_t)((frf >> 16) & 0xFF),
        (uint8_t)((frf >> 8) & 0xFF),
        (uint8_t)(frf & 0xFF)
    };
    spiCommand(SX1280_OP_SET_RF_FREQUENCY, freqParams, 3);

    // 4. Write data to Buffer at offset 0
    writeBuffer(0x00, data, len);

    // 5. Update Packet Params with the actual payload length
    if (_cfg.modulation == Modulation::Lora) {
        uint8_t packetParams[7] = {
            0x16, // 12 symbols preamble
            0x80, // implicit header type
            len,  // updated length
            0x20, // CRC ON
            0x40, // standard IQ
            0x00,
            0x00
        };
        spiCommand(SX1280_OP_SET_PACKET_PARAMS, packetParams, 7);
    } else { // FLRC
        uint8_t packetParams[7] = {
            SX1280_FLRC_PREAMBLE_16_BITS,
            SX1280_FLRC_SYNC_WORD_4_BYTES,
            SX1280_FLRC_SYNC_MATCH_1, // Aligned with SWDR005: 0x10 instead of 0x04
            SX1280_FLRC_PACKET_FIXED,
            len,  // updated length
            SX1280_FLRC_CRC_2_BYTE,   // Aligned with SWDR005: 0x10 instead of 0x20
            SX1280_FLRC_WHITENING_OFF
        };
        spiCommand(SX1280_OP_SET_PACKET_PARAMS, packetParams, 7);
    }

    // 6. Clear Interrupts
    uint8_t clearParams[2] = { 0xFF, 0xFF };
    spiCommand(SX1280_OP_CLEAR_IRQ_STATUS, clearParams, 2);

    // 7. Issue SetTx (0x00, 0x00, 0x00)
    uint8_t txParams[3] = { 0x00, 0x00, 0x00 };
    spiCommand(SX1280_OP_SET_TX, txParams, 3);
}

bool Sx1280NativePhy::readRx(RxPacket& out) {
    if (!_rxReady.load(std::memory_order_acquire)) return false;
    _rxReady.store(false, std::memory_order_release);

    // 1. Get IRQ Status
    uint8_t irqData[2] = {0, 0};
    if (!waitBusy()) {
        _hardwareError.store(true, std::memory_order_release);
        return false;
    }
    gpio_put(SX128X_SPI_CS, false);
    hal::sleepUs(2);
    spiTransfer(SX1280_OP_GET_IRQ_STATUS);
    spiTransfer(0x00); // dummy status
    irqData[0] = spiTransfer(0x00);
    irqData[1] = spiTransfer(0x00);
    gpio_put(SX128X_SPI_CS, true);
    hal::sleepUs(2);
    if (!waitBusy()) {
        _hardwareError.store(true, std::memory_order_release);
    }
    uint16_t irq = ((uint16_t)irqData[0] << 8) | irqData[1];

    // Check for CRC Error (0x0040)
    if (irq & 0x0040) {
        // Clear IRQ status
        uint8_t clearParams[2] = { 0xFF, 0xFF };
        spiCommand(SX1280_OP_CLEAR_IRQ_STATUS, clearParams, 2);
        return false;
    }

    // 2. Get Rx Buffer Status to find starting offset and length
    uint8_t rxBufStatus[2] = {0, 0};
    if (!waitBusy()) {
        _hardwareError.store(true, std::memory_order_release);
        return false;
    }
    gpio_put(SX128X_SPI_CS, false);
    hal::sleepUs(2);
    spiTransfer(SX1280_OP_GET_RX_BUFFER_STATUS);
    spiTransfer(0x00); // dummy status
    rxBufStatus[0] = spiTransfer(0x00); // payload length
    rxBufStatus[1] = spiTransfer(0x00); // start buffer pointer offset
    gpio_put(SX128X_SPI_CS, true);
    hal::sleepUs(2);
    if (!waitBusy()) {
        _hardwareError.store(true, std::memory_order_release);
    }

    uint8_t rxLen = rxBufStatus[0];
    uint8_t rxOffset = rxBufStatus[1];
    
    // In LoRa implicit header mode, rxBufStatus[0] might be 0, so we fall back to configured payloadLen
    if (_cfg.modulation == Modulation::Lora || rxLen == 0) {
        rxLen = _cfg.payloadLen;
    }
    if (rxLen > sizeof(out.data)) {
        rxLen = sizeof(out.data);
    }
    out.len = rxLen;

    // 3. Read packet data from FIFO buffer
    readBuffer(rxOffset, out.data, rxLen);

    // 4. Get Packet Status (RSSI and SNR)
    uint8_t packetStatus[5] = {0};
    if (!waitBusy()) {
        _hardwareError.store(true, std::memory_order_release);
        return false;
    }
    gpio_put(SX128X_SPI_CS, false);
    hal::sleepUs(2);
    spiTransfer(SX1280_OP_GET_PACKET_STATUS);
    spiTransfer(0x00); // dummy status
    for (uint8_t i = 0; i < 5; i++) {
        packetStatus[i] = spiTransfer(0x00);
    }
    gpio_put(SX128X_SPI_CS, true);
    hal::sleepUs(2);
    if (!waitBusy()) {
        _hardwareError.store(true, std::memory_order_release);
    }

    if (_cfg.modulation == Modulation::Lora) {
        uint8_t rssiSync = packetStatus[0];
        uint8_t snrRaw = packetStatus[1];
        
        float snrVal = (snrRaw < 128) ? (snrRaw / 4.0f) : ((snrRaw - 256) / 4.0f);
        float rssiVal = -0.5f * rssiSync;
        if (snrVal <= 0.0f) {
            out.rssiDbm = (int16_t)(rssiVal - snrVal);
        } else {
            out.rssiDbm = (int16_t)(rssiVal);
        }
        out.snr = (int8_t)snrVal;
    } else { // FLRC
        uint8_t rssiSync = packetStatus[1];
        out.rssiDbm = (int16_t)(-0.5f * rssiSync);
        out.snr = 0;
    }

    out.timestampUs = _lastIrqUs.load(std::memory_order_acquire);

    // Clear IRQ status
    uint8_t clearParams[2] = { 0xFF, 0xFF };
    spiCommand(SX1280_OP_CLEAR_IRQ_STATUS, clearParams, 2);

    return !_hardwareError.load(std::memory_order_acquire);
}

void Sx1280NativePhy::setOutputPowerDbm(int8_t dbm) {
    _cfg.powerDbm = dbm;
    uint8_t txParams[2] = {
        (uint8_t)(dbm + 18),
        0xE0 // 24 us ramp time
    };
    spiCommand(SX1280_OP_SET_TX_PARAMS, txParams, 2);
}

void Sx1280NativePhy::setSyncWord(uint16_t uidDerived) {
    if (_cfg.modulation == Modulation::Flrc) {
        uint8_t sync[4] = {
            (uint8_t)(uidDerived >> 8),
            (uint8_t)(uidDerived),
            (uint8_t)(~(uidDerived >> 8)),
            (uint8_t)(~uidDerived)
        };
        // FLRC requires 4-byte write to 0x09CF
        writeRegister(SX1280_REG_FLRC_SYNC_WORD, sync, 4);

        // Update packet params for FLRC with syncWordLen = 4, syncWordMatch = 1
        uint8_t brBwByte = (_cfg.flrcBitrateKbps < 800) ? 0x8A : 0x45;
        uint8_t modParams[3] = {
            brBwByte,
            (uint8_t)((_cfg.cr - 2) * 2),
            0x20 // BT 0.5
        };
        spiCommand(SX1280_OP_SET_MODULATION_PARAMS, modParams, 3);

        uint8_t packetParams[7] = {
            SX1280_FLRC_PREAMBLE_16_BITS,
            SX1280_FLRC_SYNC_WORD_4_BYTES,
            SX1280_FLRC_SYNC_MATCH_1, // Aligned with SWDR005: 0x10 instead of 0x04
            SX1280_FLRC_PACKET_FIXED,
            _cfg.payloadLen,
            SX1280_FLRC_CRC_2_BYTE,   // Aligned with SWDR005: 0x10 instead of 0x20
            SX1280_FLRC_WHITENING_OFF
        };
        spiCommand(SX1280_OP_SET_PACKET_PARAMS, packetParams, 7);
    } else {
        uint8_t syncWord = uidDerived & 0xFF;
        uint8_t controlBits = 0x44;
        uint8_t data[2] = {
            (uint8_t)((syncWord & 0xF0) | ((controlBits & 0xF0) >> 4)),
            (uint8_t)(((syncWord & 0x0F) << 4) | (controlBits & 0x0F))
        };
        writeRegister(SX1280_REG_LORA_SYNC_WORD, data, 2);

        // Update packet params for LoRa
        uint8_t packetParams[7] = {
            0x16, // 12 symbols preamble
            0x80, // implicit header type
            _cfg.payloadLen,
            0x20, // CRC ON
            0x40, // standard IQ
            0x00,
            0x00
        };
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
    if (cfg.modulation == Modulation::Lora) {
        modParams[0] = cfg.sf << 4;
        uint8_t bwByte = 0x18;
        if (cfg.bwKHz > 1200.0f) bwByte = 0x0A;
        else if (cfg.bwKHz > 600.0f) bwByte = 0x18;
        else if (cfg.bwKHz > 300.0f) bwByte = 0x26;
        else bwByte = 0x34;
        modParams[1] = bwByte;
        modParams[2] = cfg.cr - 4;
    } else { // FLRC
        uint8_t brBwByte = 0x45; // default 1.3 Mbps
        if (cfg.flrcBitrateKbps < 800) {
            brBwByte = 0x8A; // 650 kbps
        }
        modParams[0] = brBwByte;
        modParams[1] = (cfg.cr - 2) * 2;
        modParams[2] = 0x20; // BT 0.5
    }
    spiCommand(SX1280_OP_SET_MODULATION_PARAMS, modParams, 3);

    // Set sync word and packet parameters
    setSyncWord(cfg.syncWord);
    
    // Clear any pending interrupts
    uint8_t clearParams[2] = { 0xFF, 0xFF };
    spiCommand(SX1280_OP_CLEAR_IRQ_STATUS, clearParams, 2);
    
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
