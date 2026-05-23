// Small app-level payloads carried by Link::queueTelemetry/getTelemetry.
#pragma once

#include <stdint.h>
#include <string.h>

#include "crsf_protocol.h"
#include "link/RfConfig.h"
#include "link/Uid.h"

namespace xlrs {

static constexpr uint8_t APP_TELEMETRY_MAX_LEN = 128;

struct AppTelemetryMessage {
    uint8_t len;
    uint8_t data[APP_TELEMETRY_MAX_LEN];
};

enum class AppTelemetryType : uint8_t {
    RxConfig = 1,
    BindUid = 2,
    CrsfFrame = 3,
    Reboot = 4,
    StartBind = 5,
};

inline bool appTelemetryHasPrefix(const uint8_t* payload, size_t len, AppTelemetryType type) {
    return payload && len >= 4 &&
           payload[0] == 'X' && payload[1] == 'L' && payload[2] == 1 &&
           payload[3] == (uint8_t)type;
}

inline bool makeRxConfigMessage(const RfConfigData& cfg, AppTelemetryMessage& out) {
    out.len = 9;
    out.data[0] = 'X';
    out.data[1] = 'L';
    out.data[2] = 1;
    out.data[3] = (uint8_t)AppTelemetryType::RxConfig;
    out.data[4] = cfg.region;
    out.data[5] = cfg.defaultRate;
    out.data[6] = (uint8_t)cfg.maxPowerDbm;
    out.data[7] = cfg.failsafeMode;
    out.data[8] = cfg.dynamicPower;
    return true;
}

inline bool parseRxConfigMessage(const uint8_t* payload, size_t len, RfConfigData& cfg) {
    if (!appTelemetryHasPrefix(payload, len, AppTelemetryType::RxConfig) || len < 9) return false;
    cfg.region = payload[4];
    cfg.defaultRate = payload[5];
    cfg.maxPowerDbm = (int8_t)payload[6];
    cfg.failsafeMode = payload[7];
    cfg.dynamicPower = payload[8];
    RfConfig::refreshChecksum(cfg);
    return true;
}

inline bool makeBindUidMessage(const uint8_t uid[LINK_UID_SIZE], AppTelemetryMessage& out) {
    if (!uid) return false;
    out.len = 4 + LINK_UID_SIZE;
    out.data[0] = 'X';
    out.data[1] = 'L';
    out.data[2] = 1;
    out.data[3] = (uint8_t)AppTelemetryType::BindUid;
    memcpy(&out.data[4], uid, LINK_UID_SIZE);
    return true;
}

inline bool parseBindUidMessage(const uint8_t* payload, size_t len, uint8_t uid[LINK_UID_SIZE]) {
    if (!uid || !appTelemetryHasPrefix(payload, len, AppTelemetryType::BindUid) ||
        len < 4 + LINK_UID_SIZE) {
        return false;
    }
    memcpy(uid, &payload[4], LINK_UID_SIZE);
    return true;
}

inline bool makeStartBindMessage(const uint8_t uid[LINK_UID_SIZE], AppTelemetryMessage& out) {
    if (!uid) return false;
    out.len = 4 + LINK_UID_SIZE;
    out.data[0] = 'X';
    out.data[1] = 'L';
    out.data[2] = 1;
    out.data[3] = (uint8_t)AppTelemetryType::StartBind;
    memcpy(&out.data[4], uid, LINK_UID_SIZE);
    return true;
}

inline bool parseStartBindMessage(const uint8_t* payload, size_t len, uint8_t uid[LINK_UID_SIZE]) {
    if (!uid || !appTelemetryHasPrefix(payload, len, AppTelemetryType::StartBind) ||
        len < 4 + LINK_UID_SIZE) {
        return false;
    }
    memcpy(uid, &payload[4], LINK_UID_SIZE);
    return true;
}

inline bool makeCrsfFrameMessage(const uint8_t* frame, uint8_t frameLen, AppTelemetryMessage& out) {
    if (!frame || frameLen == 0 || frameLen + 5 > APP_TELEMETRY_MAX_LEN) return false;
    out.len = frameLen + 5;
    out.data[0] = 'X';
    out.data[1] = 'L';
    out.data[2] = 1;
    out.data[3] = (uint8_t)AppTelemetryType::CrsfFrame;
    out.data[4] = frameLen;
    memcpy(&out.data[5], frame, frameLen);
    return true;
}

inline bool parseCrsfFrameMessage(const uint8_t* payload, size_t len, const uint8_t*& frame,
                                  uint8_t& frameLen) {
    if (!appTelemetryHasPrefix(payload, len, AppTelemetryType::CrsfFrame) || len < 5) return false;
    frameLen = payload[4];
    if (frameLen == 0 || (size_t)frameLen + 5 > len) return false;
    frame = &payload[5];
    return true;
}

inline bool makeRebootMessage(AppTelemetryMessage& out) {
    out.len = 4;
    out.data[0] = 'X';
    out.data[1] = 'L';
    out.data[2] = 1;
    out.data[3] = (uint8_t)AppTelemetryType::Reboot;
    return true;
}

} // namespace xlrs
