#include "link/RfConfig.h"

#if defined(ARDUINO) || defined(PICO_BOARD) || defined(ARDUINO_ARCH_RP2040)
#include <EEPROM.h>
#endif

namespace xlrs {

bool RfConfig::load(RfConfigData& cfg) {
#if defined(ARDUINO) || defined(PICO_BOARD) || defined(ARDUINO_ARCH_RP2040)
    uint8_t* ptr = reinterpret_cast<uint8_t*>(&cfg);
    for (size_t i = 0; i < sizeof(RfConfigData); ++i) {
        ptr[i] = EEPROM.read(RF_CONFIG_EEPROM_BASE + i);
    }
    return validate(cfg);
#else
    // Simulation fallback
    setDefaults(cfg);
    return true;
#endif
}

bool RfConfig::save(const RfConfigData& cfg) {
#if defined(ARDUINO) || defined(PICO_BOARD) || defined(ARDUINO_ARCH_RP2040)
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&cfg);
    for (size_t i = 0; i < sizeof(RfConfigData); ++i) {
        EEPROM.write(RF_CONFIG_EEPROM_BASE + i, ptr[i]);
    }
    return EEPROM.commit();
#else
    // Simulation fallback
    return true;
#endif
}

} // namespace xlrs
