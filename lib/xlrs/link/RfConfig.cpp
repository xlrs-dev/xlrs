#include "link/RfConfig.h"
#include "hal/FlashStore.h"

namespace xlrs {

bool RfConfig::load(RfConfigData& cfg) {
    uint8_t* ptr = reinterpret_cast<uint8_t*>(&cfg);
    for (size_t i = 0; i < sizeof(RfConfigData); ++i) {
        ptr[i] = hal::FlashStore::read(RF_CONFIG_EEPROM_BASE + i);
    }
    return validate(cfg);
}

bool RfConfig::save(const RfConfigData& cfg) {
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&cfg);
    bool ok = true;
    for (size_t i = 0; i < sizeof(RfConfigData); ++i) {
        ok = hal::FlashStore::write(RF_CONFIG_EEPROM_BASE + i, ptr[i]) && ok;
    }
    return ok && hal::FlashStore::commit();
}

} // namespace xlrs
