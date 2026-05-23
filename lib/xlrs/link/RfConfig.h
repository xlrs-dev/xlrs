#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define RF_CONFIG_EEPROM_BASE 0
#define RF_CONFIG_MAGIC 0x52464347UL  // "RFCG"
#define RF_CONFIG_VERSION 1

namespace xlrs {

enum class RfRegion : uint8_t { US = 0, EU = 1 };

struct __attribute__((packed)) RfConfigData {
    uint32_t magic;         // Magic word to validate data
    uint8_t  version;       // Schema version
    uint8_t  region;        // 0 = US, 1 = EU
    uint8_t  defaultRate;   // default startup rate index
    int8_t   maxPowerDbm;   // dynamic power cap
    uint8_t  failsafeMode;  // 0 = NoPulses, 1 = Hold
    uint8_t  dynamicPower;  // 0 = disabled, 1 = enabled
    uint8_t  reserved[2];   // alignment padding
    uint16_t checksum;      // CRC16 over the config payload
};

class RfConfig {
public:
    static void setDefaults(RfConfigData& cfg) {
        cfg.magic = RF_CONFIG_MAGIC;
        cfg.version = RF_CONFIG_VERSION;
        cfg.region = (uint8_t)RfRegion::US;
        cfg.defaultRate = 0; // F1000
        cfg.maxPowerDbm = 10; // default 10dBm
        cfg.failsafeMode = 0; // NoPulses
        cfg.dynamicPower = 1; // enabled
        cfg.reserved[0] = 0;
        cfg.reserved[1] = 0;
        cfg.checksum = calculateChecksum(cfg);
    }

    static bool validate(RfConfigData& cfg) {
        if (cfg.magic != RF_CONFIG_MAGIC) return false;
        if (cfg.version != RF_CONFIG_VERSION) return false;
        if (cfg.region > 1) cfg.region = 0;
        if (cfg.defaultRate >= 5) cfg.defaultRate = 0; // clamp to valid rates
        if (cfg.maxPowerDbm < -18 || cfg.maxPowerDbm > 13) {
            // Capped to SX1280 max of 12.5dBm/13dBm
            cfg.maxPowerDbm = 10;
        }
        // Force EU cap if region is EU
        if (cfg.region == (uint8_t)RfRegion::EU && cfg.maxPowerDbm > 10) {
            cfg.maxPowerDbm = 10;
        }
        if (cfg.failsafeMode > 1) cfg.failsafeMode = 0;
        if (cfg.dynamicPower > 1) cfg.dynamicPower = 1;
        return cfg.checksum == calculateChecksum(cfg);
    }

    static bool load(RfConfigData& cfg);
    static bool save(const RfConfigData& cfg);

private:
    static uint16_t calculateChecksum(const RfConfigData& cfg) {
        const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&cfg);
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < sizeof(RfConfigData) - 2; ++i) {
            crc ^= (uint16_t)ptr[i] << 8;
            for (uint8_t bit = 0; bit < 8; ++bit) {
                crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
            }
        }
        return crc;
    }
};

} // namespace xlrs
