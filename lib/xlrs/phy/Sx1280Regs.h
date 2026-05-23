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
    SX1280_OP_SET_AUTO_FS            = 0x9E,
    
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
    SX1280_REG_FLRC_SYNC_WORD        = 0x09CF,
    SX1280_REG_RX_GAIN               = 0x0891, // [7:6]=3 → high-sensitivity (Table 13-1)
    SX1280_REG_PLL_TUNE              = 0x0930  // vendor PLL tuning (undocumented in datasheet)
};

// ------------------------------------------------------------
// RF frequency synthesis: frf = freqMHz * 2^18 / F_XTAL (datasheet §4.3).
// ------------------------------------------------------------
static constexpr uint32_t SX1280_FREQ_DIV  = 1u << 18; // 262144
static constexpr uint32_t SX1280_XTAL_MHZ  = 52;       // crystal reference (MHz)

// ------------------------------------------------------------
// IRQ status / mask bits (datasheet Table 11-73).
// ------------------------------------------------------------
enum Sx1280Irq : uint16_t {
    SX1280_IRQ_TX_DONE       = 0x0001,
    SX1280_IRQ_RX_DONE       = 0x0002,
    SX1280_IRQ_CRC_ERROR     = 0x0040,
    SX1280_IRQ_RXTX_TIMEOUT  = 0x4000
};

// DIO/IRQ mask programmed via SetDioIrqParams: route TX/RX-done, CRC error and timeout to DIO1.
static constexpr uint16_t SX1280_IRQ_MASK =
    SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE | SX1280_IRQ_CRC_ERROR | SX1280_IRQ_RXTX_TIMEOUT;

// PA ramp time selector for SetTxParams (Table 11-49).
static constexpr uint8_t SX1280_RAMP_20_US = 0xE0;

// RxGain register fields (SX1280_REG_RX_GAIN).
static constexpr uint8_t SX1280_RX_GAIN_HIGH_SENS = 0xC0; // [7:6]=3
static constexpr uint8_t SX1280_RX_GAIN_KEEP_MASK = 0x3F; // preserve the other live bits

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

// FLRC bitrate/bandwidth selector for SetModulationParams (Table 14-31).
enum Sx1280FlrcBitrate : uint8_t {
    SX1280_FLRC_BR_1_300             = 0x45, // 1.3 Mbps / BW 1.2 MHz
    SX1280_FLRC_BR_0_650             = 0x86  // 0.65 Mbps / BW 0.6 MHz
};

// FLRC Gaussian filter BT (modParam[2]).
static constexpr uint8_t SX1280_FLRC_BT_0_5 = 0x20;

// ============================================================
// SX1280 LoRa Modulation & Packet Parameters
// ============================================================
// LoRa bandwidth selector for SetModulationParams (modParam[1], Table 13-44).
enum Sx1280LoraBandwidth : uint8_t {
    SX1280_LORA_BW_1600              = 0x0A,
    SX1280_LORA_BW_800               = 0x18,
    SX1280_LORA_BW_400               = 0x26,
    SX1280_LORA_BW_200               = 0x34
};

// LoRa packet parameters for SetPacketParams (Table 13-49).
static constexpr uint8_t SX1280_LORA_PREAMBLE_12     = 0x16; // 12-symbol preamble
static constexpr uint8_t SX1280_LORA_HEADER_IMPLICIT = 0x80; // implicit (fixed-length) header
static constexpr uint8_t SX1280_LORA_CRC_ON          = 0x20;
static constexpr uint8_t SX1280_LORA_IQ_STD          = 0x40; // standard (non-inverted) IQ

} // namespace xlrs
