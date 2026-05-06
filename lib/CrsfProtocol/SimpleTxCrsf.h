#pragma once

#include <Arduino.h>

/**
 * Bit-identical CRSF packing from kkbin505 Arduino-Transmitter-for-ELRS SimpleTX/
 * crsf.cpp + crsf.h (GPL). Used only for handset → ELRS TX raw frames.

 * UART baud must match the ELRS TX module handset setting (often 400000; SimpleTX Arduino uses 420000).
 */

namespace simpletx_crsf {

constexpr uint8_t SIMPLETX_SYNTH_ADDR = 0xEE;
constexpr uint8_t TYPE_CHANNELS = 0x16;
constexpr uint8_t TYPE_SETTINGS_WRITE = 0x2D;
constexpr uint8_t ADDR_RADIO = 0xEA;
constexpr unsigned SIMPLETX_CHANNELS_DEFAULT_CENTER_US = 1500;

[[nodiscard]] uint8_t crc8(const uint8_t *ptr, uint8_t len);

/** 26-byte RC_CHANNELS frame; channels[*] masked to 11 bits like SimpleTX. */
void prepareDataPacket(uint8_t packet[26], const int16_t channels16[16]);

/** Build 26-byte frame from 8 logical channels (µs); ch8–15 = default center µs */
void prepareDataPacketFrom8Us(uint8_t packet[26], const uint16_t channels_us8[8], int16_t default_aux_us = 1500);

/** 8-byte ELRS SETTINGS_WRITE (SimpleTX crsfPrepareCmdPacket) */
void prepareCmdPacket(uint8_t packetCmd[8], uint8_t command, uint8_t value);

}  // namespace simpletx_crsf
