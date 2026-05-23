#include "crsfSerial.h"
#include "app/CrsfChannels.h"
#include "hal/Time.h"
#include <cstring>

// static void hexdump(void *p, size_t len)
// {
//     char *data = (char *)p;
//     while (len > 0)
//     {
//         uint8_t linepos = 0;
//         char* linestart = data;
//         // Binary part
//         while (len > 0 && linepos < 16)
//         {
//             if (*data < 0x0f)
//             Serial.write('0');
//             Serial.print(*data, HEX);
//             Serial.write(' ');
//             ++data;
//             ++linepos;
//             --len;
//         }

//         // Spacer to align last line
//         for (uint8_t i = linepos; i < 16; ++i)
//             Serial.print("   ");

//         // ASCII part
//         for (uint8_t i = 0; i < linepos; ++i)
//             Serial.write((linestart[i] < ' ') ? '.' : linestart[i]);
//         Serial.println();
//     }
// }

CrsfSerial::CrsfSerial(xlrs::hal::SerialPort &port, uint32_t baud) :
    _port(port), _rxBufPos(0), _crc(0xd5),
    _batteryVoltage(0), _batteryCurrent(0), _batteryCapacity(0), _batteryRemaining(0),
    _baud(baud), _lastReceive(0), _lastChannelsPacket(0), _linkIsUp(false),
    _passthroughBaud(0)
{
    memset(_rxBuf, 0, sizeof(_rxBuf));
    memset(&_linkStatistics, 0, sizeof(_linkStatistics));
    memset(&_gpsSensor, 0, sizeof(_gpsSensor));
    memset(&_batterySensor, 0, sizeof(_batterySensor));
    for (unsigned int i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        _channels[i] = 1500;
    }

    onLinkUp = nullptr;
    onLinkDown = nullptr;
    onOobData = nullptr;
    onPacketChannels = nullptr;
    onPacketLinkStatistics = nullptr;
    onPacketGps = nullptr;
    onPacketBattery = nullptr;
    onDeviceInfo = nullptr;
    onParameterEntry = nullptr;
    onDevicePing = nullptr;
    onParameterRead = nullptr;
    onParameterWrite = nullptr;
}

void CrsfSerial::begin(uint32_t baud)
{
    if (baud != 0)
        _port.begin(baud);
    else
        _port.begin(_baud);
}

// Call from main loop to update
void CrsfSerial::loop()
{
    handleSerialIn();
}

void CrsfSerial::handleSerialIn()
{
    while (_port.available())
    {
        int value = _port.read();
        if (value < 0) break;
        uint8_t b = (uint8_t)value;
        _lastReceive = xlrs::hal::nowMs();

        if (getPassthroughMode())
        {
            if (onOobData)
                onOobData(b);
            continue;
        }

        _rxBuf[_rxBufPos++] = b;
        handleByteReceived();

        if (_rxBufPos == (sizeof(_rxBuf)/sizeof(_rxBuf[0])))
        {
            // Packet buffer filled and no valid packet found, dump the whole thing
            _rxBufPos = 0;
        }
    }

    checkPacketTimeout();
    checkLinkDown();
}

void CrsfSerial::handleByteReceived()
{
    bool reprocess;
    do
    {
        reprocess = false;
        if (_rxBufPos > 0 && !isCrsfFrameAddress(_rxBuf[0]))
        {
            shiftRxBuffer(1);
            reprocess = true;
        }
        else if (_rxBufPos > 1)
        {
            uint8_t len = _rxBuf[1];
            // Sanity check the declared length isn't outside Type + X{1,CRSF_MAX_PAYLOAD_LEN} + CRC
            // assumes there never will be a CRSF message that just has a type and no data (X)
            if (len < 3 || len > (CRSF_MAX_PAYLOAD_LEN + 2))
            {
                shiftRxBuffer(1);
                reprocess = true;
            }

            else if (_rxBufPos >= (len + 2))
            {
                uint8_t inCrc = _rxBuf[2 + len - 1];
                uint8_t crc = _crc.calc(&_rxBuf[2], len - 1);
                if (crc == inCrc)
                {
                    processPacketIn(len);
                    shiftRxBuffer(len + 2);
                    reprocess = true;
                }
                else
                {
                    shiftRxBuffer(1);
                    reprocess = true;
                }
            }  // if complete packet
        } // if pos > 1
    } while (reprocess);
}

void CrsfSerial::checkPacketTimeout()
{
    if (_rxBufPos > 0 && xlrs::hal::nowMs() - _lastReceive > CRSF_PACKET_TIMEOUT_MS)
    {
        if (onOobData) {
            for (uint8_t i = 0; i < _rxBufPos; ++i) {
                onOobData(_rxBuf[i]);
            }
        }
        _rxBufPos = 0;
    }
}

void CrsfSerial::checkLinkDown()
{
    if (_linkIsUp && xlrs::hal::nowMs() - _lastChannelsPacket > CRSF_FAILSAFE_STAGE1_MS)
    {
        if (onLinkDown)
            onLinkDown();
        _linkIsUp = false;
    }
}

void CrsfSerial::processPacketIn(uint8_t len)
{
    const crsf_header_t *hdr = (crsf_header_t *)_rxBuf;
    if (hdr->type >= CRSF_FRAMETYPE_DEVICE_PING && hdr->type <= CRSF_FRAMETYPE_COMMAND)
    {
        packetExtendedHeader(hdr);
        return;
    }

    if ((hdr->device_addr == CRSF_ADDRESS_CRSF_TRANSMITTER || hdr->device_addr == CRSF_ADDRESS_CRSF_RECEIVER) && hdr->type == CRSF_FRAMETYPE_DEVICE_INFO)
    {
        packetDeviceInfo(hdr);
        return;
    }
    if (hdr->device_addr == CRSF_ADDRESS_CRSF_TRANSMITTER)
    {
        if (hdr->type == CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY)
            packetParameterEntry(hdr);
        else if (hdr->type == CRSF_FRAMETYPE_LINK_STATISTICS)
            packetLinkStatistics(hdr);
        else if (hdr->type == CRSF_FRAMETYPE_BATTERY_SENSOR)
            packetBattery(hdr);
        return;
    }
    if (hdr->device_addr == CRSF_ADDRESS_FLIGHT_CONTROLLER)
    {
        switch (hdr->type)
        {
        case CRSF_FRAMETYPE_GPS:
            packetGps(hdr);
            break;
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            packetChannelsPacked(hdr);
            break;
        case CRSF_FRAMETYPE_LINK_STATISTICS:
            packetLinkStatistics(hdr);
            break;
        case CRSF_FRAMETYPE_BATTERY_SENSOR:
            packetBattery(hdr);
            break;
        }
    } // CRSF_ADDRESS_FLIGHT_CONTROLLER
}

void CrsfSerial::packetExtendedHeader(const crsf_header_t *p)
{
    uint8_t payloadLen = p->frame_size >= CRSF_FRAME_LENGTH_EXT_TYPE_CRC
        ? (uint8_t)(p->frame_size - CRSF_FRAME_LENGTH_EXT_TYPE_CRC)
        : 0;
    if (p->frame_size < CRSF_FRAME_LENGTH_EXT_TYPE_CRC || !p->data) return;

    uint8_t destination = p->data[0];
    uint8_t origin = p->data[1];
    const uint8_t *payload = &p->data[2];

    switch (p->type)
    {
    case CRSF_FRAMETYPE_DEVICE_PING:
        if (payloadLen == 0 && onDevicePing)
            onDevicePing(destination, origin);
        break;
    case CRSF_FRAMETYPE_DEVICE_INFO:
        packetDeviceInfo(p);
        break;
    case CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY:
        packetParameterEntry(p);
        break;
    case CRSF_FRAMETYPE_PARAMETER_READ:
        if (payloadLen >= 2 && onParameterRead)
            onParameterRead(payload[0], payload[1], destination, origin);
        break;
    case CRSF_FRAMETYPE_PARAMETER_WRITE:
        if (payloadLen >= 1 && onParameterWrite)
            onParameterWrite(payload[0], &payload[1], (uint8_t)(payloadLen - 1), destination, origin);
        break;
    default:
        break;
    }
}

// Shift the bytes in the RxBuf down by cnt bytes
void CrsfSerial::shiftRxBuffer(uint8_t cnt)
{
    // If removing the whole thing, just set pos to 0
    if (cnt >= _rxBufPos)
    {
        _rxBufPos = 0;
        return;
    }

    if (cnt == 1 && onOobData)
        onOobData(_rxBuf[0]);

    // Otherwise do the slow shift down
    uint8_t *src = &_rxBuf[cnt];
    uint8_t *dst = &_rxBuf[0];
    _rxBufPos -= cnt;
    uint8_t left = _rxBufPos;
    while (left--)
        *dst++ = *src++;
}

void CrsfSerial::packetChannelsPacked(const crsf_header_t *p)
{
    const crsf_channels_t *crsfChannels = (const crsf_channels_t *)&p->data;
    uint16_t rcChannels[CRSF_NUM_CHANNELS];
    xlrs::crsfChannelsToRcUs(*crsfChannels, rcChannels);
    for (unsigned int i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        _channels[i] = rcChannels[i];
    }

    if (!_linkIsUp && onLinkUp)
        onLinkUp();
    _linkIsUp = true;
    _lastChannelsPacket = xlrs::hal::nowMs();

    if (onPacketChannels)
        onPacketChannels();
}

void CrsfSerial::packetLinkStatistics(const crsf_header_t *p)
{
    const crsfLinkStatistics_t *link = (crsfLinkStatistics_t *)p->data;
    memcpy(&_linkStatistics, link, sizeof(_linkStatistics));

    if (onPacketLinkStatistics)
        onPacketLinkStatistics(&_linkStatistics);
}

void CrsfSerial::packetGps(const crsf_header_t *p)
{
    const crsf_sensor_gps_t *gps = (crsf_sensor_gps_t *)p->data;
    _gpsSensor.latitude = be32toh(gps->latitude);
    _gpsSensor.longitude = be32toh(gps->longitude);
    _gpsSensor.groundspeed = be16toh(gps->groundspeed);
    _gpsSensor.heading = be16toh(gps->heading);
    _gpsSensor.altitude = be16toh(gps->altitude);
    _gpsSensor.satellites = gps->satellites;

    if (onPacketGps)
        onPacketGps(&_gpsSensor);
}

// DEVICE_INFO payload: 1 version, 1 param_count, 2 sw, 2 hw, 4 serial, null-term name
void CrsfSerial::packetDeviceInfo(const crsf_header_t *p)
{
    uint8_t payloadOffset = (p->type >= CRSF_FRAMETYPE_DEVICE_PING) ? 2 : 0;
    uint8_t payloadLen = p->frame_size >= (uint8_t)(CRSF_FRAME_LENGTH_TYPE_CRC + payloadOffset)
        ? (uint8_t)(p->frame_size - CRSF_FRAME_LENGTH_TYPE_CRC - payloadOffset)
        : 0;
    if (payloadLen < 15 || !p->data) return;

    const uint8_t *payload = &p->data[payloadOffset];
    const char *name = (const char *)payload;
    size_t nameLen = 0;
    while (nameLen < payloadLen && name[nameLen] != '\0') nameLen++;
    if (nameLen >= payloadLen || payloadLen < nameLen + 1 + 14) return;
    const uint8_t *serial4 = (const uint8_t *)&payload[nameLen + 1];
    if (onDeviceInfo)
        onDeviceInfo(serial4, name, p->device_addr);  // source: 0xEE=TX, 0xEC=RX
}

// PARAMETER_SETTINGS_ENTRY: [fieldId, chunksRemaining, parent, type/hidden, label\0, value...]
void CrsfSerial::packetParameterEntry(const crsf_header_t *p)
{
    uint8_t payloadOffset = (p->type >= CRSF_FRAMETYPE_DEVICE_PING) ? 2 : 0;
    uint8_t payloadLen = p->frame_size >= (uint8_t)(CRSF_FRAME_LENGTH_TYPE_CRC + payloadOffset)
        ? (uint8_t)(p->frame_size - CRSF_FRAME_LENGTH_TYPE_CRC - payloadOffset)
        : 0;
    if (payloadLen < 5 || !p->data) return;

    const uint8_t *payload = &p->data[payloadOffset];
    uint8_t fieldId = payload[0];
    uint8_t chunksRemaining = payload[1];
    uint8_t paramType = payload[3] & 0x7F;
    const char *label = (const char *)&payload[4];
    size_t labelMax = payloadLen - 4;
    size_t labelLen = 0;
    while (labelLen < labelMax && label[labelLen] != '\0') labelLen++;
    if (labelLen >= labelMax) return;
    const uint8_t *value = (const uint8_t *)&label[labelLen + 1];
    uint8_t valueLen = payloadLen - 4 - (uint8_t)labelLen - 1;
    if (onParameterEntry)
        onParameterEntry(fieldId, paramType, chunksRemaining, label, value, valueLen);
}

void CrsfSerial::packetBattery(const crsf_header_t *p)
{
    // Battery sensor data is 8 bytes, big-endian
    // Format: voltage (16-bit), current (16-bit), capacity (24-bit), remaining (8-bit)
    const uint8_t *data = p->data;
    
    // Voltage: V * 10, big-endian
    uint16_t voltage_raw = (data[0] << 8) | data[1];
    _batteryVoltage = voltage_raw / 10.0f;
    
    // Current: A * 10, big-endian
    uint16_t current_raw = (data[2] << 8) | data[3];
    _batteryCurrent = current_raw / 10.0f;
    
    // Capacity: mAh, 24-bit big-endian
    _batteryCapacity = (data[4] << 16) | (data[5] << 8) | data[6];
    
    // Remaining: %
    _batteryRemaining = data[7];
    
    // Store raw struct too
    memcpy(&_batterySensor, data, sizeof(_batterySensor));

    if (onPacketBattery)
        onPacketBattery(_batteryVoltage, _batteryCurrent, _batteryCapacity, _batteryRemaining);
}

void CrsfSerial::write(uint8_t b)
{
    _port.write(b);
}

void CrsfSerial::write(const uint8_t *buf, size_t len)
{
    _port.write(buf, len);
}

void CrsfSerial::queuePacket(uint8_t addr, uint8_t type, const void *payload, uint8_t len)
{
    if (getPassthroughMode())
        return;
    if (len > CRSF_MAX_PAYLOAD_LEN)
        return;

    uint8_t buf[CRSF_MAX_PACKET_SIZE];
    buf[0] = addr;
    buf[1] = len + 2; // type + payload + crc
    buf[2] = type;
    memcpy(&buf[3], payload, len);
    buf[len+3] = _crc.calc(&buf[2], len + 1);

    write(buf, len + 4);
}

void CrsfSerial::queueExtendedPacket(uint8_t addr, uint8_t type, uint8_t destination,
                                     uint8_t origin, const void *payload, uint8_t len)
{
    if (getPassthroughMode())
        return;
    if (len > (CRSF_MAX_PAYLOAD_LEN - 2))
        return;

    uint8_t buf[CRSF_MAX_PACKET_SIZE];
    buf[0] = addr;
    buf[1] = len + CRSF_FRAME_LENGTH_EXT_TYPE_CRC; // type + dest + origin + payload + crc
    buf[2] = type;
    buf[3] = destination;
    buf[4] = origin;
    if (len > 0 && payload) {
        memcpy(&buf[5], payload, len);
    }
    buf[len + 5] = _crc.calc(&buf[2], len + 3);

    write(buf, len + 6);
}

/**
 * @brief   Enter passthrough mode (serial sent directly to shiftybyte),
 *          optionally changing the baud rate used during passthrough mode
 * @param val
 *          True to start passthrough mode, false to resume processing CRSF
 * @param passthroughBaud
 *          New baud rate for passthrough mode, or 0 to not change baud
 *          Not used if disabling passthough
*/
void CrsfSerial::setPassthroughMode(bool val, uint32_t passthroughBaud)
{
    if (val)
    {
        // If not requesting any baud change
        if (passthroughBaud == 0)
        {
            // Enter passthrough mode if not yet
            if (_passthroughBaud == 0)
                _passthroughBaud = _baud;
            return;
        }

        _passthroughBaud = passthroughBaud;
    }
    else
    {
        // Not in passthrough, can't leave it any harder than we already are
        if (_passthroughBaud == 0)
            return;

        // Leaving passthrough, but going back to same baud, just disable
        if (_passthroughBaud == _baud)
        {
            _passthroughBaud = 0;
            return;
        }

        _passthroughBaud = 0;
    }

    // Can only get here if baud is changing, close and reopen the port
    _port.end(); // assumes flush()
    begin(_passthroughBaud);
}
