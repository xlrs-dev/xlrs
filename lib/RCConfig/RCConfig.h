#ifndef RC_CONFIG_H
#define RC_CONFIG_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Schema version for protocol and EEPROM
#define RC_CONFIG_SCHEMA_VERSION 4

// EEPROM layout: avoid Security (120-168) and legacy cal block (150-175)
#define RC_CONFIG_EEPROM_BASE   200
#define RC_CONFIG_MAGIC         0x43434658UL  // "XCFC"

// Physical axes (ADC indices) and logical channels (AERT)
#define RC_NUM_AXES     4
#define RC_NUM_CHANNELS 8

// Default ADC range: 16-bit scaled (ADS1115 or internal scaled to 0–32767), or 12-bit raw (0–4095) when RC_STICK_ADC_12BIT
#if defined(RC_STICK_ADC_12BIT) && defined(INTERNAL_ADC)
#define RC_CALIB_DEFAULT_MIN    365
#define RC_CALIB_DEFAULT_MAX    2927
#define RC_CALIB_DEFAULT_CENTER 1650
#else
#define RC_CALIB_DEFAULT_MIN    2917
#define RC_CALIB_DEFAULT_MAX    23420
#define RC_CALIB_DEFAULT_CENTER 13199
#endif

// Channel value bounds (µs)
#define RC_CHANNEL_MIN  1000
#define RC_CHANNEL_MAX  2000
#define RC_CHANNEL_MID  1500

// Subtrim per output channel (µs); applied after stick mapping, before cutoff clamp
#define RC_CHANNEL_TRIM_MAX_ABS  250

typedef struct {
    // Axis -> logical channel mapping: channel_function[i] = which function (0=A,1=E,2=R,3=T) gets physical axis i
    // So channel_function[0]=0 means physical axis 0 is Aileron. Default: 0,1,2,3 (A,E,R,T order).
    uint8_t channel_function[RC_NUM_AXES];
    uint8_t invert[RC_NUM_AXES];       // 0 or 1 per axis
    uint16_t calib_min[RC_NUM_AXES];
    uint16_t calib_max[RC_NUM_AXES];
    uint16_t calib_center[RC_NUM_AXES];
    float deadzone[RC_NUM_AXES];       // 0.0..0.2 typical, 0 = disabled
    float rate[RC_NUM_AXES];           // 0.5..1.0 (dual-rate)
    float expo[RC_NUM_AXES];           // 0.0..0.5 (expo curve)
    uint16_t cutoff_min[RC_NUM_CHANNELS];
    uint16_t cutoff_max[RC_NUM_CHANNELS];
    int16_t channel_trim[RC_NUM_CHANNELS];  // µs bias per CRSF channel (A,E,R,T + toggles)
    uint8_t high_pass_filter;   // 0 = off, 1 = on (1st-order HPF on sticks for slow center drift)
    uint8_t stick_low_pass;    // 0-3 IIR LPF strength on internal ADC sticks (broadband noise); Core1 + single-core path
} rc_config_data_t;

// Stored layout: magic(4) + version(2) + payload + crc32(4). Payload = rc_config_data_t.
#define RC_CONFIG_PAYLOAD_SIZE  (sizeof(rc_config_data_t))
#define RC_CONFIG_HEADER_SIZE   (4 + 2)   // magic + version
#define RC_CONFIG_CRC_SIZE      4
#define RC_CONFIG_EEPROM_SIZE   (RC_CONFIG_HEADER_SIZE + RC_CONFIG_PAYLOAD_SIZE + RC_CONFIG_CRC_SIZE)

#ifdef __cplusplus
extern "C" {
#endif

// Fill config with defaults (matches current rc_crsf_main behavior)
void rc_config_set_defaults(rc_config_data_t* config);

// Validate and clamp fields; return true if valid
bool rc_config_validate(rc_config_data_t* config);

// Compute CRC32 of payload only (used for EEPROM)
uint32_t rc_config_crc32(const uint8_t* data, size_t len);

// Load from EEPROM. Returns true if valid block found and loaded.
bool rc_config_load(rc_config_data_t* config);

// Save to EEPROM. Call EEPROM.begin() before if needed. Returns true on success.
bool rc_config_save(const rc_config_data_t* config);

#ifdef __cplusplus
}
#endif

#endif // RC_CONFIG_H
