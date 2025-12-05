#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <Arduino.h>
#include <EEPROM.h>

#define EEPROM_SIZE 256
#define SSID_ADDR 0
#define PASSWORD_ADDR 64
#define SSID_MAX_LEN 32
#define PASSWORD_MAX_LEN 32

struct WiFiConfig {
    char ssid[SSID_MAX_LEN];
    char password[PASSWORD_MAX_LEN];
};

class WiFiConfigManager {
public:
    WiFiConfigManager();
    bool begin();
    bool loadConfig(WiFiConfig& config);
    bool saveConfig(const WiFiConfig& config);
    void setDefaultConfig(const char* ssid, const char* password);
};

#endif // WIFI_CONFIG_H

