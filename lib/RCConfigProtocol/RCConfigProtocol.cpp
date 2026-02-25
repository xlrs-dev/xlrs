#include "RCConfigProtocol.h"
#include "../crc8/crc8.h"
#include <string.h>

static Crc8 s_crc8(0xD5);
static const uint8_t MAX_BINDING_PHRASE_LEN = 32;

RCConfigProtocol::RCConfigProtocol()
    : stream(nullptr)
    , rxState(0)
    , rxLen(0)
    , rxPos(0)
    , rxTimeout(0)
    , getConfig(nullptr)
    , setConfigDraft(nullptr)
    , applyConfig(nullptr)
    , saveConfig(nullptr)
    , calStart(nullptr)
    , calSample(nullptr)
    , calFinish(nullptr)
    , getState(nullptr)
    , setBindPhraseRx(nullptr)
    , setBindPhraseTx(nullptr)
    , getLinkStatus(nullptr)
    , enterPairingMode(nullptr)
    , streamingState(false)
{
    memset(deviceName, 0, sizeof(deviceName));
    memset(deviceVersion, 0, sizeof(deviceVersion));
    strncpy(deviceName, "RC-CRSF", sizeof(deviceName) - 1);
    strncpy(deviceVersion, "1.0", sizeof(deviceVersion) - 1);
}

void RCConfigProtocol::setDeviceInfo(const char* name, const char* version) {
    memset(deviceName, 0, sizeof(deviceName));
    memset(deviceVersion, 0, sizeof(deviceVersion));
    if (name && name[0] != '\0') {
        strncpy(deviceName, name, sizeof(deviceName) - 1);
    } else {
        strncpy(deviceName, "RC-CRSF", sizeof(deviceName) - 1);
    }
    if (version && version[0] != '\0') {
        strncpy(deviceVersion, version, sizeof(deviceVersion) - 1);
    } else {
        strncpy(deviceVersion, "1.0", sizeof(deviceVersion) - 1);
    }
}

void RCConfigProtocol::poll() {
    if (!stream) return;
    if (rxState != 0 && (millis() - rxTimeout) > RX_TIMEOUT_MS)
        resetRx();
    while (stream->available()) {
        uint8_t b = (uint8_t)stream->read();
        processRxByte(b);
    }
}

void RCConfigProtocol::processRxByte(uint8_t byte) {
    rxTimeout = millis();
    switch (rxState) {
    case 0:
        if (byte == RC_PROTO_SYNC_BYTE) rxState = 1;
        break;
    case 1:
        if (byte <= RC_PROTO_MAX_PAYLOAD) {
            rxLen = byte;
            rxPos = 0;
            rxState = 2;
        } else
            rxState = 0;
        break;
    case 2: {
        if (rxPos < sizeof(rxBuf)) rxBuf[rxPos++] = byte;
        if (rxPos >= rxLen) rxState = 3;
        break;
    }
    case 3: {
        uint8_t calcCrc = s_crc8.calc((uint8_t*)rxBuf, rxLen);
        if (calcCrc != byte) {
            if (getConfig && stream) {
                sendResponse(rxBuf[0], rxBuf[1], RC_STATUS_ERR_CRC, nullptr, 0);
            }
            rxState = 0;
            return;
        }
        uint8_t cmd = rxBuf[0];
        uint8_t seq = rxBuf[1];
        const uint8_t* payload = rxLen > 2 ? rxBuf + 2 : nullptr;
        uint8_t payloadLen = rxLen > 2 ? (uint8_t)(rxLen - 2) : 0;

        switch (cmd) {
        case RC_CMD_GET_DEVICE_INFO: {
            uint8_t resp[RC_PROTO_DEVICE_NAME_MAX + RC_PROTO_DEVICE_VER_MAX + 2];
            uint8_t n = 0;
            size_t nameLen = strlen(deviceName);
            size_t verLen = strlen(deviceVersion);
            memcpy(resp + n, deviceName, nameLen);
            n += (uint8_t)nameLen;
            resp[n++] = 0;
            memcpy(resp + n, deviceVersion, verLen);
            n += (uint8_t)verLen;
            resp[n++] = 0;
            sendResponse(cmd, seq, RC_STATUS_OK, resp, n);
            break;
        }
        case RC_CMD_GET_CONFIG:
            if (getConfig) {
                const rc_config_data_t* c = getConfig();
                sendResponse(cmd, seq, RC_STATUS_OK, (const uint8_t*)c, RC_CONFIG_PAYLOAD_SIZE);
            } else {
                sendResponse(cmd, seq, RC_STATUS_ERR_INVALID_CMD, nullptr, 0);
            }
            break;
        case RC_CMD_SET_CONFIG_DRAFT:
            if (setConfigDraft && payloadLen >= RC_CONFIG_PAYLOAD_SIZE) {
                rc_config_data_t draft;
                memcpy(&draft, payload, RC_CONFIG_PAYLOAD_SIZE);
                rc_config_validate(&draft);
                setConfigDraft(&draft);
                sendResponse(cmd, seq, RC_STATUS_OK, nullptr, 0);
            } else {
                sendResponse(cmd, seq, RC_STATUS_ERR_PAYLOAD, nullptr, 0);
            }
            break;
        case RC_CMD_APPLY_CONFIG:
            if (applyConfig) {
                applyConfig();
                sendResponse(cmd, seq, RC_STATUS_OK, nullptr, 0);
            } else {
                sendResponse(cmd, seq, RC_STATUS_ERR_INVALID_CMD, nullptr, 0);
            }
            break;
        case RC_CMD_SAVE_CONFIG:
            if (saveConfig) {
                bool ok = saveConfig();
                sendResponse(cmd, seq, ok ? RC_STATUS_OK : RC_STATUS_ERR_SAVE, nullptr, 0);
            } else {
                sendResponse(cmd, seq, RC_STATUS_ERR_INVALID_CMD, nullptr, 0);
            }
            break;
        case RC_CMD_START_CALIBRATION:
            if (calStart) {
                calStart();
                sendResponse(cmd, seq, RC_STATUS_OK, nullptr, 0);
            } else {
                sendResponse(cmd, seq, RC_STATUS_ERR_INVALID_CMD, nullptr, 0);
            }
            break;
        case RC_CMD_CALIBRATION_SAMPLE:
            if (calSample) {
                calSample();
                sendResponse(cmd, seq, RC_STATUS_OK, nullptr, 0);
            } else {
                sendResponse(cmd, seq, RC_STATUS_ERR_INVALID_CMD, nullptr, 0);
            }
            break;
        case RC_CMD_FINISH_CALIBRATION:
            if (calFinish) {
                calFinish();
                sendResponse(cmd, seq, RC_STATUS_OK, nullptr, 0);
            } else {
                sendResponse(cmd, seq, RC_STATUS_ERR_INVALID_CMD, nullptr, 0);
            }
            break;
        case RC_CMD_STREAM_STATE_START:
            streamingState = true;
            sendResponse(cmd, seq, RC_STATUS_OK, nullptr, 0);
            break;
        case RC_CMD_STREAM_STATE_STOP:
            streamingState = false;
            sendResponse(cmd, seq, RC_STATUS_OK, nullptr, 0);
            break;
        case RC_CMD_SET_BINDING_PHRASE_RX: {
            if (!setBindPhraseRx || !payload || payloadLen == 0 || payloadLen > MAX_BINDING_PHRASE_LEN) {
                sendResponse(cmd, seq, RC_STATUS_ERR_PAYLOAD, nullptr, 0);
                break;
            }
            char phrase[MAX_BINDING_PHRASE_LEN + 1];
            memcpy(phrase, payload, payloadLen);
            phrase[payloadLen] = '\0';
            bool ok = setBindPhraseRx(phrase, payloadLen);
            sendResponse(cmd, seq, ok ? RC_STATUS_OK : RC_STATUS_ERR_FORWARD, nullptr, 0);
            break;
        }
        case RC_CMD_SET_BINDING_PHRASE_TX: {
            if (!setBindPhraseTx || !payload || payloadLen == 0 || payloadLen > MAX_BINDING_PHRASE_LEN) {
                sendResponse(cmd, seq, RC_STATUS_ERR_PAYLOAD, nullptr, 0);
                break;
            }
            char phrase[MAX_BINDING_PHRASE_LEN + 1];
            memcpy(phrase, payload, payloadLen);
            phrase[payloadLen] = '\0';
            bool ok = setBindPhraseTx(phrase, payloadLen);
            sendResponse(cmd, seq, ok ? RC_STATUS_OK : RC_STATUS_ERR_FORWARD, nullptr, 0);
            break;
        }
        case RC_CMD_GET_LINK_STATUS: {
            if (!getLinkStatus) {
                sendResponse(cmd, seq, RC_STATUS_ERR_INVALID_CMD, nullptr, 0);
                break;
            }
            uint8_t resp[3] = {0, 0, 0xFF};
            getLinkStatus(&resp[0], &resp[1], &resp[2]);
            sendResponse(cmd, seq, RC_STATUS_OK, resp, sizeof(resp));
            break;
        }
        case RC_CMD_ENTER_PAIRING_MODE: {
            if (!enterPairingMode) {
                sendResponse(cmd, seq, RC_STATUS_ERR_INVALID_CMD, nullptr, 0);
                break;
            }
            bool ok = enterPairingMode();
            sendResponse(cmd, seq, ok ? RC_STATUS_OK : RC_STATUS_ERR_FORWARD, nullptr, 0);
            break;
        }
        default:
            sendResponse(cmd, seq, RC_STATUS_ERR_INVALID_CMD, nullptr, 0);
            break;
        }
        rxState = 0;
        break;
    }
    }
}

void RCConfigProtocol::resetRx() {
    rxState = 0;
    rxPos = 0;
    rxLen = 0;
}

uint8_t RCConfigProtocol::crc8(const uint8_t* data, uint8_t len) {
    uint8_t buf[RC_PROTO_MAX_PAYLOAD + 2];
    if (len <= sizeof(buf))
        memcpy(buf, data, len);
    return s_crc8.calc(buf, len);
}

bool RCConfigProtocol::sendFrame(uint8_t cmd, uint8_t seq, const uint8_t* payload, uint8_t payloadLen) {
    if (!stream) return false;
    if (2 + payloadLen > RC_PROTO_MAX_PAYLOAD) return false;
    uint8_t buf[RC_PROTO_MAX_PAYLOAD];
    buf[0] = cmd;
    buf[1] = seq;
    if (payloadLen && payload)
        memcpy(buf + 2, payload, payloadLen);
    uint8_t crc = s_crc8.calc(buf, (uint8_t)(2 + payloadLen));
    stream->write(RC_PROTO_SYNC_BYTE);
    stream->write((uint8_t)(2 + payloadLen));
    stream->write(buf, 2 + payloadLen);
    stream->write(crc);
    stream->flush();  // Ensure USB CDC sends immediately (avoids buffering)
    return true;
}

bool RCConfigProtocol::sendResponse(uint8_t reqCmd, uint8_t seq, RCProtoStatus status, const uint8_t* data, uint8_t dataLen) {
    uint8_t payload[RC_PROTO_MAX_PAYLOAD];
    uint8_t n = 0;
    payload[n++] = status;
    payload[n++] = RC_CONFIG_SCHEMA_VERSION & 0xFF;
    payload[n++] = (RC_CONFIG_SCHEMA_VERSION >> 8) & 0xFF;
    if (data && dataLen && (n + dataLen <= sizeof(payload))) {
        memcpy(payload + n, data, dataLen);
        n += dataLen;
    }
    return sendFrame((uint8_t)(RC_PROTO_RESP_BIT | reqCmd), seq, payload, n);
}

bool RCConfigProtocol::sendStateFrame(const int16_t* adc4, const uint16_t* ch8, const uint8_t* toggles4) {
    if (!stream || !adc4 || !ch8 || !toggles4) return false;
    uint8_t payload[4*2 + 8*2 + 4];
    uint8_t* p = payload;
    for (int i = 0; i < 4; i++) {
        uint16_t v = (uint16_t)adc4[i];
        *p++ = v & 0xFF;
        *p++ = (v >> 8) & 0xFF;
    }
    for (int i = 0; i < 8; i++) {
        uint16_t v = ch8[i];
        *p++ = v & 0xFF;
        *p++ = (v >> 8) & 0xFF;
    }
    for (int i = 0; i < 4; i++)
        *p++ = toggles4[i] ? 1 : 0;
    return sendFrame(RC_CMD_STATE_FRAME, 0, payload, (uint8_t)(4*2 + 8*2 + 4));
}
