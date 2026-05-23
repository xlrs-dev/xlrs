#include "crsfSerial.h"
#include "hal/Time.h"
#include <cstring>

namespace {
static int32_t mapRange(int32_t value, int32_t inMin, int32_t inMax, int32_t outMin, int32_t outMax) {
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
} // namespace

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
    _port(port), _crc(0xd5), _baud(baud),
    _lastReceive(0), _lastChannelsPacket(0), _linkIsUp(false),
    _passthroughBaud(0), _batteryVoltage(0), _batteryCurrent(0),
    _batteryCapacity(0), _batteryRemaining(0)
{}

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
        if (_rxBufPos > 1)
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
    // If we haven't received data in a long time, flush the buffer a byte at a time (to trigger shiftyByte)
    if (_rxBufPos > 0 && xlrs::hal::nowMs() - _lastReceive > CRSF_PACKET_TIMEOUT_MS)
        while (_rxBufPos)
            shiftRxBuffer(1);
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
    crsf_channels_t *ch = (crsf_channels_t *)&p->data;
    _channels[0] = ch->ch0;
    _channels[1] = ch->ch1;
    _channels[2] = ch->ch2;
    _channels[3] = ch->ch3;
    _channels[4] = ch->ch4;
    _channels[5] = ch->ch5;
    _channels[6] = ch->ch6;
    _channels[7] = ch->ch7;
    _channels[8] = ch->ch8;
    _channels[9] = ch->ch9;
    _channels[10] = ch->ch10;
    _channels[11] = ch->ch11;
    _channels[12] = ch->ch12;
    _channels[13] = ch->ch13;
    _channels[14] = ch->ch14;
    _channels[15] = ch->ch15;

    for (unsigned int i=0; i<CRSF_NUM_CHANNELS; ++i)
        _channels[i] = mapRange(_channels[i], CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, 1000, 2000);

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
    uint8_t payloadLen = p->frame_size >= 2 ? (uint8_t)(p->frame_size - 2) : 0;
    if (payloadLen < 10 || !p->data) return;
    const uint8_t *serial4 = &p->data[6];
    const char *name = (const char *)&p->data[10];
    if (name - (const char *)p->data >= (int)payloadLen) return;
    if (onDeviceInfo)
        onDeviceInfo(serial4, name, p->device_addr);  // source: 0xEE=TX, 0xEC=RX
}

// PARAMETER_SETTINGS_ENTRY: [fieldId, chunksRemaining, parent, type/hidden, label\0, value...]
void CrsfSerial::packetParameterEntry(const crsf_header_t *p)
{
    uint8_t payloadLen = p->frame_size >= 2 ? (uint8_t)(p->frame_size - 2) : 0;
    if (payloadLen < 5 || !p->data) return;
    uint8_t fieldId = p->data[0];
    uint8_t chunksRemaining = p->data[1];
    uint8_t paramType = p->data[3] & 0x7F;
    const char *label = (const char *)&p->data[4];
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
