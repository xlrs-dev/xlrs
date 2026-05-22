// RadioLibPhy implementation backed by RadioLib's SX1280 driver.
// Entire body is ARDUINO-guarded so the native (host) test build skips it.
#include "phy/RadioLibPhy.h"

#if defined(ARDUINO)
#include <SPI.h>

// Pin map comes from build flags (the xlrs_tx / xlrs_rx envs set these); defaults match
// the standard hardware wiring so the board works out of the box.
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

RadioLibPhy* RadioLibPhy::s_self = nullptr;

// Tiny ISR: latch the hardware timestamp (load-bearing for the PFD) and route to the
// matching done-callback. No SPI/RadioLib calls here — readRx() drains in the RF task.
void RadioLibPhy::onDio1() {
    if (!s_self) return;
    s_self->_lastIrqUs.store(micros(), std::memory_order_release);
    Mode mode = s_self->_mode.load(std::memory_order_acquire);
    if (mode == Mode::Rx) {
        s_self->_rxReady.store(true, std::memory_order_release);
        if (s_self->_onRxDone) s_self->_onRxDone();
    } else if (mode == Mode::Tx) {
        if (s_self->_onTxDone) s_self->_onTxDone();
    }
}

bool RadioLibPhy::init(const PhyConfig& cfg) {
    _cfg = cfg;
    s_self = this;
    _healthy.store(false, std::memory_order_release);

    SPI.setSCK(SX128X_SPI_SCK);
    SPI.setTX(SX128X_SPI_MOSI);
    SPI.setRX(SX128X_SPI_MISO);
    SPI.setCS(SX128X_SPI_CS);
    SPI.begin();

    static SPISettings spi(8000000, MSBFIRST, SPI_MODE0);
    _module = new Module(SX128X_SPI_CS, SX128X_SPI_DIO1, SX128X_SPI_RST, SX128X_SPI_BUSY, SPI, spi);
    if (!_module) return false;
    _radio = new SX1280(_module);
    if (!_radio) return false;
    _radio->setRfSwitchPins(SX128X_RXEN, SX128X_TXEN);

    int16_t st;
    if (cfg.modulation == Modulation::Flrc) {
        // beginFLRC(freq, br, cr, power, preambleBits, shaping)
        st = _radio->beginFLRC(cfg.freqMHz, cfg.flrcBitrateKbps, cfg.cr, cfg.powerDbm, 16,
                               RADIOLIB_SHAPING_0_5);
    } else {
        // begin(freq, bw, sf, cr, syncWord, power, preambleLen)
        st = _radio->begin(cfg.freqMHz, cfg.bwKHz, cfg.sf, cfg.cr,
                           RADIOLIB_SX128X_SYNC_WORD_PRIVATE, cfg.powerDbm, 12);
    }
    if (st != RADIOLIB_ERR_NONE) return false;

    setSyncWord(cfg.syncWord);
    _radio->setDio1Action(onDio1);
    _healthy.store(true, std::memory_order_release);
    startRx(cfg.freqMHz);
    return healthy();
}

void RadioLibPhy::startRx(float freqMHz) {
    if (!_radio) return;
    _mode.store(Mode::Rx, std::memory_order_release);
    _rxReady.store(false, std::memory_order_release);
    int16_t st = _radio->setFrequency(freqMHz);                 // atomic hop: freq set with the arm
    if (st != RADIOLIB_ERR_NONE) {
        _healthy.store(false, std::memory_order_release);
        return;
    }
    st = _radio->startReceive();
    if (st != RADIOLIB_ERR_NONE) {
        _healthy.store(false, std::memory_order_release);
    }
}

void RadioLibPhy::startTx(float freqMHz, const uint8_t* data, uint8_t len) {
    if (!_radio) return;
    _mode.store(Mode::Tx, std::memory_order_release);
    int16_t st = _radio->setFrequency(freqMHz);
    if (st != RADIOLIB_ERR_NONE) {
        _healthy.store(false, std::memory_order_release);
        return;
    }
    st = _radio->startTransmit(const_cast<uint8_t*>(data), len);   // non-blocking
    if (st != RADIOLIB_ERR_NONE) {
        _healthy.store(false, std::memory_order_release);
    }
}

bool RadioLibPhy::readRx(RxPacket& out) {
    if (!_radio || !_rxReady.load(std::memory_order_acquire)) return false;
    _rxReady.store(false, std::memory_order_release);
    size_t n = _radio->getPacketLength();
    if (n > sizeof(out.data)) n = sizeof(out.data);
    if (_radio->readData(out.data, n) != RADIOLIB_ERR_NONE) {
        _healthy.store(false, std::memory_order_release);
        return false;
    }
    out.len         = (uint8_t)n;
    out.rssiDbm     = (int16_t)_radio->getRSSI();
    out.snr         = (int8_t)_radio->getSNR();        // 0 in FLRC (RadioLib returns 0)
    out.timestampUs = _lastIrqUs.load(std::memory_order_acquire); // PFD subtracts airtime → packet start
    return true;
}

void RadioLibPhy::setOutputPowerDbm(int8_t dbm) {
    if (!_radio) return;
    if (_radio->setOutputPower(dbm) != RADIOLIB_ERR_NONE) {
        _healthy.store(false, std::memory_order_release);
    }
}

void RadioLibPhy::setSyncWord(uint16_t uidDerived) {
    if (!_radio) return;
    if (_cfg.modulation == Modulation::Flrc) {
        // FLRC mode requires exactly 4 bytes sync word.
        // We use bitwise NOT padding to map the 16-bit word deterministically.
        uint8_t sync[4] = {
            (uint8_t)(uidDerived >> 8),
            (uint8_t)(uidDerived),
            (uint8_t)(~(uidDerived >> 8)),
            (uint8_t)(~uidDerived)
        };
        if (_radio->setSyncWord(sync, 4) != RADIOLIB_ERR_NONE) {
            _healthy.store(false, std::memory_order_release);
        }
    } else {
        // LoRa mode uses a 1-byte private/public sync word.
        if (_radio->setSyncWord(uidDerived & 0xFF) != RADIOLIB_ERR_NONE) {
            _healthy.store(false, std::memory_order_release);
        }
    }
}

void RadioLibPhy::reconfigure(const PhyConfig& cfg) {
    if (!_radio) {
        _healthy.store(false, std::memory_order_release);
        return;
    }
    _cfg = cfg;
    _healthy.store(false, std::memory_order_release);
    
    // In RadioLib, we reconfigure by calling beginFLRC or begin on the existing _radio.
    int16_t st;
    if (cfg.modulation == Modulation::Flrc) {
        st = _radio->beginFLRC(cfg.freqMHz, cfg.flrcBitrateKbps, cfg.cr, cfg.powerDbm, 16,
                               RADIOLIB_SHAPING_0_5);
    } else {
        st = _radio->begin(cfg.freqMHz, cfg.bwKHz, cfg.sf, cfg.cr,
                           RADIOLIB_SX128X_SYNC_WORD_PRIVATE, cfg.powerDbm, 12);
    }
    if (st == RADIOLIB_ERR_NONE) {
        _healthy.store(true, std::memory_order_release);
        setSyncWord(cfg.syncWord);
        startRx(cfg.freqMHz);
    }
}

uint32_t RadioLibPhy::txLatencyUs() const {
    return 85;
}

bool RadioLibPhy::recover() {
    if (!_radio) {
        return init(_cfg);
    }
    reconfigure(_cfg);
    if (healthy()) {
        _radio->setDio1Action(onDio1);
    }
    return healthy();
}

} // namespace xlrs
#endif // ARDUINO
