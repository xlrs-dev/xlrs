#include "WiFiConfig.h"

WiFiConfigManager::WiFiConfigManager() {
}

bool WiFiConfigManager::begin() {
    EEPROM.begin(EEPROM_SIZE);
    return true;
}

bool WiFiConfigManager::loadConfig(WiFiConfig& config) {
    // Read SSID
    for (int i = 0; i < SSID_MAX_LEN; i++) {
        config.ssid[i] = EEPROM.read(SSID_ADDR + i);
    }
    config.ssid[SSID_MAX_LEN - 1] = '\0';
    
    // Read password
    for (int i = 0; i < PASSWORD_MAX_LEN; i++) {
        config.password[i] = EEPROM.read(PASSWORD_ADDR + i);
    }
    config.password[PASSWORD_MAX_LEN - 1] = '\0';
    
    // Check if config is valid (non-empty SSID)
    return strlen(config.ssid) > 0;
}

bool WiFiConfigManager::saveConfig(const WiFiConfig& config) {
    // Write SSID
    for (int i = 0; i < SSID_MAX_LEN; i++) {
        EEPROM.write(SSID_ADDR + i, config.ssid[i]);
    }
    
    // Write password
    for (int i = 0; i < PASSWORD_MAX_LEN; i++) {
        EEPROM.write(PASSWORD_ADDR + i, config.password[i]);
    }
    
    return EEPROM.commit();
}

void WiFiConfigManager::setDefaultConfig(const char* ssid, const char* password) {
    WiFiConfig config;
    if (!loadConfig(config) || strlen(config.ssid) == 0) {
        // No config exists, set defaults
        strncpy(config.ssid, ssid, SSID_MAX_LEN - 1);
        strncpy(config.password, password, PASSWORD_MAX_LEN - 1);
        config.ssid[SSID_MAX_LEN - 1] = '\0';
        config.password[PASSWORD_MAX_LEN - 1] = '\0';
        saveConfig(config);
    }
}

