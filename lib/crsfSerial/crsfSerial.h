#pragma once

#include <stdint.h>
#include <stddef.h>
#include <../crc8/crc8.h>
#include "hal/SerialPort.h"
#include "crsf_protocol.h"

enum eFailsafeAction { fsaNoPulses, fsaHold };

class CrsfSerial
{
public:
    // Packet timeout where buffer is flushed if no data is received in this time
    static const unsigned int CRSF_PACKET_TIMEOUT_MS = 100;
    static const unsigned int CRSF_FAILSAFE_STAGE1_MS = 300;

    CrsfSerial(xlrs::hal::SerialPort &port, uint32_t baud = CRSF_BAUDRATE);
    void begin(uint32_t baud = 0);
    void loop();
    void write(uint8_t b);
    void write(const uint8_t *buf, size_t len);
    void queuePacket(uint8_t addr, uint8_t type, const void *payload, uint8_t len);
    void queueExtendedPacket(uint8_t addr, uint8_t type, uint8_t destination, uint8_t origin,
                             const void *payload, uint8_t len);

    uint32_t getBaud() const { return _baud; };
    // Return current channel value (1-based) in us
    int getChannel(unsigned int ch) const { return _channels[ch - 1]; }
    const crsfLinkStatistics_t *getLinkStatistics() const { return &_linkStatistics; }
    const crsf_sensor_gps_t *getGpsSensor() const { return &_gpsSensor; }
    const crsf_sensor_battery_t *getBatterySensor() const { return &_batterySensor; }
    // Get battery voltage in V (decoded from big-endian)
    float getBatteryVoltage() const { return _batteryVoltage; }
    // Get battery current in A (decoded from big-endian)
    float getBatteryCurrent() const { return _batteryCurrent; }
    // Get battery capacity used in mAh (decoded from big-endian)
    uint32_t getBatteryCapacity() const { return _batteryCapacity; }
    // Get battery remaining in %
    uint8_t getBatteryRemaining() const { return _batteryRemaining; }
    bool isLinkUp() const { return _linkIsUp; }
    bool getPassthroughMode() const { return _passthroughBaud != 0; }
    void setPassthroughMode(bool val, uint32_t passthroughBaud = 0);

    // Event Handlers
    void (*onLinkUp)();
    void (*onLinkDown)();
    // OobData is any byte which is not CRSF, including passthrough
    void (*onOobData)(uint8_t b);
    // CRSF Packet Callbacks
    void (*onPacketChannels)();
    void (*onPacketLinkStatistics)(crsfLinkStatistics_t *ls);
    void (*onPacketGps)(crsf_sensor_gps_t *gpsSensor);
    void (*onPacketBattery)(float voltage, float current, uint32_t capacity, uint8_t remaining);
    // DEVICE_INFO (0x29) from 0xEE (TX) or 0xEC (RX): serial[4], null-term name, sourceAddr (0xEE=TX, 0xEC=RX)
    void (*onDeviceInfo)(const uint8_t *serial4, const char *name, uint8_t sourceAddr);
    // PARAMETER_SETTINGS_ENTRY (0x2B) from 0xEE: fieldId, type (0x0A=STRING, 0x0D=COMMAND), label, value
    void (*onParameterEntry)(uint8_t fieldId, uint8_t paramType, uint8_t chunksRemaining, const char *label, const uint8_t *value, uint8_t valueLen);
    void (*onDevicePing)(uint8_t destination, uint8_t origin);
    void (*onParameterRead)(uint8_t parameterNumber, uint8_t chunkNumber, uint8_t destination, uint8_t origin);
    void (*onParameterWrite)(uint8_t parameterNumber, const uint8_t *value, uint8_t valueLen, uint8_t destination, uint8_t origin);
    void (*onPacketRaw)(const uint8_t *frame, uint8_t frameLen);

private:
    xlrs::hal::SerialPort &_port;
    uint8_t _rxBuf[CRSF_MAX_PACKET_SIZE];
    uint8_t _rxBufPos;
    Crc8 _crc;
    crsfLinkStatistics_t _linkStatistics;
    crsf_sensor_gps_t _gpsSensor;
    crsf_sensor_battery_t _batterySensor;
    float _batteryVoltage;
    float _batteryCurrent;
    uint32_t _batteryCapacity;
    uint8_t _batteryRemaining;
    uint32_t _baud;
    uint32_t _lastReceive;
    uint32_t _lastChannelsPacket;
    bool _linkIsUp;
    uint32_t _passthroughBaud;
    int _channels[CRSF_NUM_CHANNELS];

    void handleSerialIn();
    void handleByteReceived();
    void shiftRxBuffer(uint8_t cnt);
    void processPacketIn(uint8_t len);
    void checkPacketTimeout();
    void checkLinkDown();
    uint8_t standardPayloadLen(const crsf_header_t *p) const;
    uint8_t extendedPayloadLen(const crsf_header_t *p) const;

    // Packet Handlers
    void packetChannelsPacked(const crsf_header_t *p);
    void packetLinkStatistics(const crsf_header_t *p);
    void packetGps(const crsf_header_t *p);
    void packetBattery(const crsf_header_t *p);
    void packetDeviceInfo(const crsf_header_t *p);
    void packetParameterEntry(const crsf_header_t *p);
    void packetExtendedHeader(const crsf_header_t *p);
};
