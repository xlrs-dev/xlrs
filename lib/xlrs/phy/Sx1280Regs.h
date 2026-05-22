#pragma once
#include <stdint.h>

namespace xlrs {

// ============================================================
// SX1280 SPI Command Opcodes
// ============================================================
enum Sx1280Opcode : uint8_t {
    SX1280_OP_SET_STANDBY            = 0x80,
    SX1280_OP_SET_RX                 = 0x82,
    SX1280_OP_SET_TX                 = 0x83,
    SX1280_OP_SET_RF_FREQUENCY       = 0x86,
    SX1280_OP_SET_PACKET_TYPE        = 0x8A,
    SX1280_OP_SET_MODULATION_PARAMS  = 0x8B,
    SX1280_OP_SET_PACKET_PARAMS      = 0x8C,
    SX1280_OP_SET_DIO_IRQ_PARAMS      = 0x8D,
    SX1280_OP_SET_TX_PARAMS           = 0x8E,
    SX1280_OP_SET_BUFFER_BASE_ADDR   = 0x8F,
    SX1280_OP_SET_REGULATOR_MODE     = 0x96,
    SX1280_OP_CLEAR_IRQ_STATUS       = 0x97,
    
    SX1280_OP_GET_STATUS             = 0xC0,
    SX1280_OP_GET_IRQ_STATUS         = 0x15,
    SX1280_OP_GET_RX_BUFFER_STATUS   = 0x17,
    SX1280_OP_GET_PACKET_STATUS     = 0x1D,
    
    SX1280_OP_WRITE_REGISTER         = 0x18,
    SX1280_OP_READ_REGISTER          = 0x19,
    SX1280_OP_WRITE_BUFFER           = 0x1A,
    SX1280_OP_READ_BUFFER            = 0x1B
};

// ============================================================
// SX1280 Register Map
// ============================================================
enum Sx1280Register : uint16_t {
    SX1280_REG_LORA_SYNC_WORD        = 0x0944,
    SX1280_REG_FLRC_SYNC_WORD        = 0x09CF
};

// ============================================================
// SX1280 Packet Types (Modulations)
// ============================================================
enum Sx1280PacketType : uint8_t {
    SX1280_PACKET_LORA               = 0x01,
    SX1280_PACKET_FLRC               = 0x03
};

// ============================================================
// SX1280 Standby & Power Configuration
// ============================================================
enum Sx1280StandbyConfig : uint8_t {
    SX1280_STDBY_RC                  = 0x00,
    SX1280_STDBY_XOSC                = 0x01
};

enum Sx1280RegulatorMode : uint8_t {
    SX1280_REGULATOR_LDO             = 0x00,
    SX1280_REGULATOR_DCDC            = 0x01
};

// ============================================================
// SX1280 FLRC Packet Parameters (aligned with SWDR005)
// ============================================================
enum Sx1280FlrcPreamble : uint8_t {
    SX1280_FLRC_PREAMBLE_16_BITS     = 0x30
};

enum Sx1280FlrcSyncWordLength : uint8_t {
    SX1280_FLRC_SYNC_WORD_4_BYTES    = 0x04
};

enum Sx1280FlrcSyncMatch : uint8_t {
    SX1280_FLRC_SYNC_MATCH_OFF       = 0x00,
    SX1280_FLRC_SYNC_MATCH_1         = 0x10,
    SX1280_FLRC_SYNC_MATCH_2         = 0x20,
    SX1280_FLRC_SYNC_MATCH_1_2       = 0x30
};

enum Sx1280FlrcPacketType : uint8_t {
    SX1280_FLRC_PACKET_FIXED         = 0x00,
    SX1280_FLRC_PACKET_VARIABLE      = 0x20
};

enum Sx1280FlrcCrc : uint8_t {
    SX1280_FLRC_CRC_OFF              = 0x00,
    SX1280_FLRC_CRC_2_BYTE           = 0x10,
    SX1280_FLRC_CRC_3_BYTE           = 0x20,
    SX1280_FLRC_CRC_4_BYTE           = 0x30
};


enum Sx1280FlrcWhitening : uint8_t {
    SX1280_FLRC_WHITENING_ON         = 0x00,
    SX1280_FLRC_WHITENING_OFF        = 0x08
};

} // namespace xlrs
