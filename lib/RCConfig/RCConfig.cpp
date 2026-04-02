#include "RCConfig.h"
#include <EEPROM.h>
#include <string.h>

#ifndef RC_CONFIG_EEPROM_SIZE_TOTAL
#define RC_CONFIG_EEPROM_SIZE_TOTAL 512
#endif

static const uint32_t crc32_table[16] = {
    0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC,
    0x76DC4190, 0x6B6B51F4, 0x4DB26158, 0x5005713C,
    0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C,
    0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C
};

static uint32_t crc32_byte(uint32_t crc, uint8_t byte) {
    uint8_t tbl = crc ^ byte;
    crc = (crc >> 4) ^ crc32_table[tbl & 0x0F];
    crc = (crc >> 4) ^ crc32_table[(tbl >> 4) & 0x0F];
    return crc;
}

void rc_config_set_defaults(rc_config_data_t* config) {
    if (!config) return;
    memset(config, 0, sizeof(rc_config_data_t));
    for (int i = 0; i < RC_NUM_AXES; i++) {
        config->channel_function[i] = (uint8_t)i;
        config->invert[i] = (i == 0 || i == 3) ? 1 : 0;  // Aileron and Rudder inverted by default (current behavior)
        config->calib_min[i] = RC_CALIB_DEFAULT_MIN;
        config->calib_max[i] = RC_CALIB_DEFAULT_MAX;
        config->calib_center[i] = RC_CALIB_DEFAULT_CENTER;
        config->deadzone[i] = (i == 3) ? 0.0f : 0.05f;   // no deadzone on throttle
        config->rate[i] = 1.0f;   // Full range (1000-2000); use 0.7 for reduced throw (1150-1850)
        config->expo[i] = 0.0f;
    }
    for (int i = 0; i < RC_NUM_CHANNELS; i++) {
        config->cutoff_min[i] = RC_CHANNEL_MIN;
        config->cutoff_max[i] = RC_CHANNEL_MAX;
        config->channel_trim[i] = 0;
    }
    config->high_pass_filter = 0;
    config->stick_low_pass = 2;
}

bool rc_config_validate(rc_config_data_t* config) {
    if (!config) return false;
    for (int i = 0; i < RC_NUM_AXES; i++) {
        if (config->channel_function[i] >= RC_NUM_AXES)
            config->channel_function[i] = (uint8_t)i;
        if (config->invert[i] > 1) config->invert[i] = 0;
        if (config->calib_max[i] <= config->calib_min[i] + 10) {
            config->calib_min[i] = RC_CALIB_DEFAULT_MIN;
            config->calib_max[i] = RC_CALIB_DEFAULT_MAX;
        }
        if (config->calib_center[i] == 0 || config->calib_center[i] >= config->calib_max[i])
            config->calib_center[i] = RC_CALIB_DEFAULT_CENTER;
        if (config->deadzone[i] < 0.0f || config->deadzone[i] > 0.5f)
            config->deadzone[i] = 0.05f;
        if (config->rate[i] < 0.3f || config->rate[i] > 1.0f)
            config->rate[i] = 1.0f;
        if (config->expo[i] < 0.0f || config->expo[i] > 1.0f)
            config->expo[i] = 0.0f;
    }
    for (int i = 0; i < RC_NUM_CHANNELS; i++) {
        if (config->cutoff_min[i] < RC_CHANNEL_MIN) config->cutoff_min[i] = RC_CHANNEL_MIN;
        if (config->cutoff_max[i] > RC_CHANNEL_MAX) config->cutoff_max[i] = RC_CHANNEL_MAX;
        if (config->cutoff_min[i] > config->cutoff_max[i]) {
            config->cutoff_min[i] = RC_CHANNEL_MIN;
            config->cutoff_max[i] = RC_CHANNEL_MAX;
        }
        if (config->channel_trim[i] < -RC_CHANNEL_TRIM_MAX_ABS)
            config->channel_trim[i] = -RC_CHANNEL_TRIM_MAX_ABS;
        if (config->channel_trim[i] > RC_CHANNEL_TRIM_MAX_ABS)
            config->channel_trim[i] = RC_CHANNEL_TRIM_MAX_ABS;
    }
    if (config->high_pass_filter > 1) config->high_pass_filter = 0;
    if (config->stick_low_pass > 3) config->stick_low_pass = 2;
    return true;
}

uint32_t rc_config_crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++)
        crc = crc32_byte(crc, data[i]);
    return crc ^ 0xFFFFFFFF;
}

bool rc_config_load(rc_config_data_t* config) {
    if (!config) return false;
    EEPROM.begin(RC_CONFIG_EEPROM_SIZE_TOTAL);
    uint32_t magic = 0;
    for (int i = 0; i < 4; i++)
        magic |= (uint32_t)EEPROM.read(RC_CONFIG_EEPROM_BASE + i) << (24 - i * 8);
    if (magic != RC_CONFIG_MAGIC) return false;
    uint16_t version = (uint16_t)EEPROM.read(RC_CONFIG_EEPROM_BASE + 4)
        | ((uint16_t)EEPROM.read(RC_CONFIG_EEPROM_BASE + 5) << 8);
    if (version != RC_CONFIG_SCHEMA_VERSION) return false;
    uint8_t buf[RC_CONFIG_PAYLOAD_SIZE];
    for (size_t i = 0; i < RC_CONFIG_PAYLOAD_SIZE; i++)
        buf[i] = EEPROM.read(RC_CONFIG_EEPROM_BASE + RC_CONFIG_HEADER_SIZE + (int)i);
    uint32_t stored_crc = 0;
    for (int i = 0; i < 4; i++)
        stored_crc |= (uint32_t)EEPROM.read(RC_CONFIG_EEPROM_BASE + RC_CONFIG_HEADER_SIZE + (int)RC_CONFIG_PAYLOAD_SIZE + i) << (24 - i * 8);
    uint32_t computed = rc_config_crc32(buf, RC_CONFIG_PAYLOAD_SIZE);
    if (stored_crc != computed) return false;
    memcpy(config, buf, RC_CONFIG_PAYLOAD_SIZE);
    rc_config_validate(config);
    return true;
}

bool rc_config_save(const rc_config_data_t* config) {
    if (!config) return false;
    EEPROM.begin(RC_CONFIG_EEPROM_SIZE_TOTAL);
    // magic
    EEPROM.write(RC_CONFIG_EEPROM_BASE + 0, (RC_CONFIG_MAGIC >> 24) & 0xFF);
    EEPROM.write(RC_CONFIG_EEPROM_BASE + 1, (RC_CONFIG_MAGIC >> 16) & 0xFF);
    EEPROM.write(RC_CONFIG_EEPROM_BASE + 2, (RC_CONFIG_MAGIC >> 8) & 0xFF);
    EEPROM.write(RC_CONFIG_EEPROM_BASE + 3, RC_CONFIG_MAGIC & 0xFF);
    // version
    EEPROM.write(RC_CONFIG_EEPROM_BASE + 4, RC_CONFIG_SCHEMA_VERSION & 0xFF);
    EEPROM.write(RC_CONFIG_EEPROM_BASE + 5, (RC_CONFIG_SCHEMA_VERSION >> 8) & 0xFF);
    // payload
    const uint8_t* p = (const uint8_t*)config;
    for (size_t i = 0; i < RC_CONFIG_PAYLOAD_SIZE; i++)
        EEPROM.write(RC_CONFIG_EEPROM_BASE + RC_CONFIG_HEADER_SIZE + (int)i, p[i]);
    uint32_t crc = rc_config_crc32(p, RC_CONFIG_PAYLOAD_SIZE);
    int off = RC_CONFIG_HEADER_SIZE + (int)RC_CONFIG_PAYLOAD_SIZE;
    EEPROM.write(RC_CONFIG_EEPROM_BASE + off + 0, (crc >> 24) & 0xFF);
    EEPROM.write(RC_CONFIG_EEPROM_BASE + off + 1, (crc >> 16) & 0xFF);
    EEPROM.write(RC_CONFIG_EEPROM_BASE + off + 2, (crc >> 8) & 0xFF);
    EEPROM.write(RC_CONFIG_EEPROM_BASE + off + 3, crc & 0xFF);
    return EEPROM.commit();
}
