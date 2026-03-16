#ifndef RC_CONFIG_PROTOCOL_H
#define RC_CONFIG_PROTOCOL_H

#include <Arduino.h>
#include "../RCConfig/RCConfig.h"

#define RC_PROTO_SYNC_BYTE  0xA5
#define RC_PROTO_MAX_PAYLOAD 128
#define RC_PROTO_RESP_BIT   0x80
#define RC_PROTO_DEVICE_NAME_MAX 16
#define RC_PROTO_DEVICE_VER_MAX 16

enum RCProtoCmd : uint8_t {
    RC_CMD_GET_DEVICE_INFO   = 0x01,
    RC_CMD_GET_CONFIG        = 0x02,
    RC_CMD_SET_CONFIG_DRAFT  = 0x03,
    RC_CMD_APPLY_CONFIG      = 0x04,
    RC_CMD_SAVE_CONFIG       = 0x05,
    RC_CMD_START_CALIBRATION = 0x10,
    RC_CMD_CALIBRATION_SAMPLE= 0x11,
    RC_CMD_FINISH_CALIBRATION= 0x12,
    RC_CMD_STREAM_STATE_START= 0x20,
    RC_CMD_STREAM_STATE_STOP = 0x21,
    RC_CMD_STATE_FRAME       = 0x30,
    RC_CMD_SET_BINDING_PHRASE_RX = 0x40,
    RC_CMD_SET_BINDING_PHRASE_TX = 0x41,
    RC_CMD_GET_LINK_STATUS   = 0x42,
    RC_CMD_ENTER_PAIRING_MODE = 0x43,
    RC_CMD_RE_DETECT_TX      = 0x44,
    RC_CMD_GET_ELRS_BINDING_PHRASE = 0x45,
    RC_CMD_ENTER_USB_UART_PROXY   = 0x46,
};

enum RCProtoStatus : uint8_t {
    RC_STATUS_OK = 0,
    RC_STATUS_ERR_INVALID_CMD = 1,
    RC_STATUS_ERR_PAYLOAD = 2,
    RC_STATUS_ERR_CRC = 3,
    RC_STATUS_ERR_BUSY = 4,
    RC_STATUS_ERR_SAVE = 5,
    RC_STATUS_ERR_FORWARD = 6,
};

class RCConfigProtocol {
public:
    RCConfigProtocol();

    void setStream(Stream* s) { stream = s; }
    Stream* getStream() const { return stream; }

    void poll();

    void setConfigGetter(const rc_config_data_t* (*fn)()) { getConfig = fn; }
    void setConfigDraftSetter(void (*fn)(const rc_config_data_t*)) { setConfigDraft = fn; }
    void setApplyCallback(void (*fn)()) { applyConfig = fn; }
    void setSaveCallback(bool (*fn)()) { saveConfig = fn; }
    void setCalibrationStart(void (*fn)()) { calStart = fn; }
    void setCalibrationSample(void (*fn)()) { calSample = fn; }
    void setCalibrationFinish(void (*fn)()) { calFinish = fn; }
    void setStateGetter(void (*fn)(int16_t* adc4, uint16_t* ch8, uint8_t* toggles4)) { getState = fn; }
    void setBindingPhraseRx(bool (*fn)(const char* phrase, uint8_t len)) { setBindPhraseRx = fn; }
    void setBindingPhraseTx(bool (*fn)(const char* phrase, uint8_t len)) { setBindPhraseTx = fn; }
    void setLinkStatusGetter(void (*fn)(uint8_t* txConnected, uint8_t* txPaired, uint8_t* txState)) { getLinkStatus = fn; }
    void setLinkStatusGetterEx(void (*fn)(uint8_t* buf, uint8_t bufLen)) { getLinkStatusEx = fn; }
    void setEnterPairingMode(bool (*fn)()) { enterPairingMode = fn; }
    void setReDetectTx(void (*fn)()) { reDetectTx = fn; }
    void setGetElrsBindingPhrase(bool (*fn)(char* phrase, uint8_t maxLen)) { getElrsBindingPhrase = fn; }
    void setEnterUsbUartProxy(void (*fn)()) { enterUsbUartProxy = fn; }
    void setDeviceInfo(const char* name, const char* version);

    bool isStreamingState() const { return streamingState; }
    void setStreamingState(bool on) { streamingState = on; }

    bool sendResponse(uint8_t reqCmd, uint8_t seq, RCProtoStatus status, const uint8_t* data = nullptr, uint8_t dataLen = 0);
    bool sendStateFrame(const int16_t* adc4, const uint16_t* ch8, const uint8_t* toggles4);

private:
    Stream* stream;
    uint8_t rxBuf[RC_PROTO_MAX_PAYLOAD];
    uint8_t rxState;
    uint8_t rxLen;
    uint8_t rxPos;
    unsigned long rxTimeout;
    static const unsigned long RX_TIMEOUT_MS = 100;

    const rc_config_data_t* (*getConfig)();
    void (*setConfigDraft)(const rc_config_data_t*);
    void (*applyConfig)();
    bool (*saveConfig)();
    void (*calStart)();
    void (*calSample)();
    void (*calFinish)();
    void (*getState)(int16_t* adc4, uint16_t* ch8, uint8_t* toggles4);
    bool (*setBindPhraseRx)(const char* phrase, uint8_t len);
    bool (*setBindPhraseTx)(const char* phrase, uint8_t len);
    void (*getLinkStatus)(uint8_t* txConnected, uint8_t* txPaired, uint8_t* txState);
    void (*getLinkStatusEx)(uint8_t* buf, uint8_t bufLen);
    bool (*enterPairingMode)();
    void (*reDetectTx)();
    bool (*getElrsBindingPhrase)(char* phrase, uint8_t maxLen);
    void (*enterUsbUartProxy)();
    char deviceName[RC_PROTO_DEVICE_NAME_MAX];
    char deviceVersion[RC_PROTO_DEVICE_VER_MAX];

    bool streamingState;

    void processRxByte(uint8_t byte);
    void resetRx();
    uint8_t crc8(const uint8_t* data, uint8_t len);
    bool sendFrame(uint8_t cmd, uint8_t seq, const uint8_t* payload, uint8_t payloadLen);
};

#endif
