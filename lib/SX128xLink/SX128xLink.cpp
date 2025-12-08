// SX128x LoRa transport helper using RadioLib.
#include "SX128xLink.h"

volatile bool SX128xLink::packetReceived = false;

SX128xLink::SX128xLink()
    : ready(false),
      module(nullptr),
      radio(nullptr) {}

void SX128xLink::dio1ISR() {
    packetReceived = true;
}

bool SX128xLink::begin() {
    if (ready) {
        return true;
    }

    // Configure SPI pins for RP2040 hardware SPI0 by default.
    SPI.setSCK(SX128X_SPI_SCK);
    SPI.setTX(SX128X_SPI_MOSI);
    SPI.setRX(SX128X_SPI_MISO);
    SPI.setCS(SX128X_SPI_CS);
    SPI.begin();

    module = new Module(SX128X_SPI_CS, SX128X_SPI_DIO1, SX128X_SPI_RST, SX128X_SPI_BUSY);
    if (!module) {
        return false;
    }

    radio = new SX1280(module);
    if (!radio) {
        return false;
    }

    // Drive external RF switch (Ebyte TX_EN/RX_EN).
    radio->setRfSwitchPins(SX128X_RXEN, SX128X_TXEN);

    radio->setDio1Action(SX128xLink::dio1ISR);

    int16_t state = radio->beginFLRC(
        SX128X_FREQ_MHZ,
        SX128X_FLRC_BR_KBPS,
        SX128X_FLRC_CR,
        SX128X_OUTPUT_POWER_DBM,
        SX128X_FLRC_PREAMBLE_BITS,
        SX128X_FLRC_SHAPING
    );
    if (state != RADIOLIB_ERR_NONE) {
        return false;
    }

    // Continuous RX after init.
    startReceive();
    ready = true;
    return true;
}

void SX128xLink::startReceive() {
    if (!radio) {
        return;
    }
    packetReceived = false;
    radio->startReceive(RADIOLIB_SX128X_RX_TIMEOUT_NONE);
}

bool SX128xLink::send(const uint8_t* data, size_t len) {
    if (!ready || !radio || !data || len == 0) {
        return false;
    }

    uint8_t buffer[64];
    if (len > sizeof(buffer)) {
        return false;
    }
    memcpy(buffer, data, len);

    int16_t state = radio->transmit(buffer, len);
    // Return to RX immediately after TX.
    startReceive();
    return state == RADIOLIB_ERR_NONE;
}

bool SX128xLink::receive(SX128xPacket& packet) {
    if (!ready || !radio || !packetReceived) {
        return false;
    }

    packetReceived = false;
    // Respect reported packet length to avoid stale bytes in buffer.
    size_t len = radio->getPacketLength();
    if (len > sizeof(packet.data)) {
        len = sizeof(packet.data);
    }

    int16_t state = radio->readData(packet.data, len);
    if (state != RADIOLIB_ERR_NONE) {
        startReceive();
        return false;
    }

    packet.length = len;
    packet.rssi = radio->getRSSI();
    packet.snr = radio->getSNR();

    // Continue listening.
    startReceive();
    return true;
}


