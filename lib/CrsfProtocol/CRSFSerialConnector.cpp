#include "CRSFSerialConnector.h"
#include "SimpleTxCrsf.h"

#include "../../include/debug_ndjson.h"

// True when bytes [2],[3] look like CRSF parent + field type on the FIRST chunk only.
// Continuation chunks are opaque — ASCII from option strings must not parse as fake types (e.g. 'D'=0x44).
static bool crsfx_parameterEntryHasTypedFrontMatter(const uint8_t *p, uint8_t len)
{
    if (len < 6)
        return false;
    const uint8_t parent = p[2];
    const uint8_t t = (uint8_t)(p[3] & 0x7Fu); // CRSF_FIELD_TYPE_MASK hides flags in bits 7-6 anyway
    if (parent > 220u)
        return false;
    if (t > 15u && t != (uint8_t)CRSF_OUT_OF_RANGE)
        return false;
    return true;
}

#if defined(CRSF_CORE1_CHANNELS)
mutex_t CRSFSerialConnector::s_queue_mutex;
uint8_t CRSFSerialConnector::s_queue_len[CORE1_QUEUE_SIZE];
uint8_t CRSFSerialConnector::s_queue_data[CORE1_QUEUE_SIZE][64];
volatile unsigned int CRSFSerialConnector::s_queue_head = 0;
volatile unsigned int CRSFSerialConnector::s_queue_tail = 0;
#endif

static uint8_t s_debug_rx_logs = 0;
static uint8_t s_debug_tx_logs = 0;
static uint8_t s_debug_pkt_logs = 0;
static bool s_verbose_mode = false;

CRSFSerialConnector::CRSFSerialConnector(HardwareSerial &port, uint32_t baud)
    : _port(port), _baud(baud)
{
    crsfRouter.addConnector(this);
}

void CRSFSerialConnector::setVerboseMode(bool enabled)
{
    s_verbose_mode = enabled;
}

void CRSFSerialConnector::begin(uint32_t baud)
{
    if (baud != 0)
        _port.begin(baud);
    else
        _port.begin(_baud);
}

void CRSFSerialConnector::resetRxParser()
{
    _parser.Reset();
}

void CRSFSerialConnector::queueCrsfRawTx(const uint8_t *bytes, uint8_t len)
{
    if (!bytes || len == 0) return;
#if defined(CRSF_CORE1_CHANNELS)
    pushToQueue(bytes, len);
#else
    _port.write(bytes, len);
#endif
}

void CRSFSerialConnector::queueSimpleTxRcChannelsPacked8(const uint16_t channels_us[8])
{
    uint8_t pkt[CRSFSerialConnector::CRSF_PACKET_RC_CHANNELS_FULL];
    simpletx_crsf::prepareDataPacketFrom8Us(pkt, channels_us,
                                            (int16_t)simpletx_crsf::SIMPLETX_CHANNELS_DEFAULT_CENTER_US);
    queueCrsfRawTx(pkt, CRSFSerialConnector::CRSF_PACKET_RC_CHANNELS_FULL);
}

#if defined(CRSF_CORE1_CHANNELS)
void CRSFSerialConnector::initTxMutex()
{
    mutex_init(&s_queue_mutex);
}

void CRSFSerialConnector::flushOutboundQueue()
{
    uint8_t buf[64];
    uint8_t len;
    while (popFromQueue(buf, &len))
        _port.write(buf, len);
}

void CRSFSerialConnector::pushToQueue(const uint8_t* data, uint8_t len)
{
    if (len > 64) return;
    mutex_enter_blocking(&s_queue_mutex);
    unsigned int next = (s_queue_tail + 1) % CORE1_QUEUE_SIZE;
    if (next != s_queue_head) {
        s_queue_len[s_queue_tail] = len;
        memcpy(s_queue_data[s_queue_tail], data, len);
        s_queue_tail = next;
    }
    mutex_exit(&s_queue_mutex);
}

bool CRSFSerialConnector::popFromQueue(uint8_t* out, uint8_t* outLen)
{
    mutex_enter_blocking(&s_queue_mutex);
    bool has = (s_queue_head != s_queue_tail);
    if (has) {
        *outLen = s_queue_len[s_queue_head];
        memcpy(out, s_queue_data[s_queue_head], *outLen);
        s_queue_head = (s_queue_head + 1) % CORE1_QUEUE_SIZE;
    }
    mutex_exit(&s_queue_mutex);
    return has;
}

#endif

#if defined(CRSF_CORE1_CHANNELS)

void CRSFSerialConnector::core1_drainAndSendChannels(const uint16_t channels[8], bool sendChannels)
{
    uint8_t buf[64];
    uint8_t len;
    while (popFromQueue(buf, &len)) {
        _port.write(buf, len);
    }
    if (!sendChannels) return;

    uint8_t pkt26[CRSFSerialConnector::CRSF_PACKET_RC_CHANNELS_FULL];
    simpletx_crsf::prepareDataPacketFrom8Us(pkt26, channels,
                                              (int16_t)simpletx_crsf::SIMPLETX_CHANNELS_DEFAULT_CENTER_US);
    _port.write(pkt26, sizeof(pkt26));
}
#endif

void CRSFSerialConnector::loop()
{
    while (_port.available())
    {
        uint8_t b = _port.read();
        _lastReceive = millis();

        // #region agent log
        if (s_verbose_mode || s_debug_rx_logs < 32)
        {
            char dataJson[96];
            snprintf(dataJson, sizeof(dataJson), "{\"byte\":%u}", b);
            debug_log_ndjson("CRSFSerialConnector.cpp:27", "uart_rx_byte", "repro1", "H1", dataJson);
            Serial.printf("[CRSF RX RAW] %02X\n", b);
            if (!s_verbose_mode) s_debug_rx_logs++;
        }
        // #endregion

        _parser.processByte(this, b, [this](const crsf_header_t *msg) {
            handlePacket(msg);
        });
    }
    checkPacketTimeout();
    checkLinkDown();
}

void CRSFSerialConnector::forwardMessage(const crsf_header_t *message)
{
    const uint8_t len = message->frame_size + 2;
    _port.write((const uint8_t *)message, len);
}

void CRSFSerialConnector::queuePacket(uint8_t addr, uint8_t type, const void *payload, uint8_t len)
{
    uint8_t buf[CRSF_MAX_PACKET_LEN];
    buf[0] = addr;
    buf[1] = len + 2;
    buf[2] = type;
    if (payload && len > 0)
        memcpy(&buf[3], payload, len);
    uint8_t crc = crsfRouter.crsf_crc.calc(&buf[2], len + 1);
    buf[3 + len] = crc;

    // #region agent log
    if (s_verbose_mode || s_debug_tx_logs < 8)
    {
        char hex[96];
        char dataJson[196];
        debug_bytes_hex(buf, 4 + len, hex, sizeof(hex));
        snprintf(dataJson, sizeof(dataJson), "{\"kind\":\"simple\",\"addr\":%u,\"type\":%u,\"len\":%u,\"frame\":\"%s\"}", addr, type, len, hex);
        debug_log_ndjson("CRSFSerialConnector.cpp:57", "uart_tx_frame", "repro1", "H3", dataJson);
        Serial.printf("[CRSF TX RAW] %s\n", hex);
        if (!s_verbose_mode) s_debug_tx_logs++;
    }
    // #endregion

#if defined(CRSF_CORE1_CHANNELS)
    pushToQueue(buf, 4 + len);
#else
    _port.write(buf, 4 + len);
#endif
}

void CRSFSerialConnector::queueExtendedPacket(crsf_addr_e dest, crsf_addr_e origin, uint8_t type, const void *payload, uint8_t len)
{
    uint8_t buf[CRSF_MAX_PACKET_LEN];
    crsf_ext_header_t *ext = (crsf_ext_header_t *)buf;
    // Handset → external TX module Lua/param frames match SimpleTX / serial captures:
    // e.g. EE 06 2C EE EA [field][chunk][crc] — first byte follows destination (0xEE), not CRSF_SYNC (0xC8).
    // FC telemetry inbound often uses C8; CRSFParser already accepts EE for crossfire transmitter.
    ext->device_addr = dest;
    ext->frame_size = CRSF_EXT_FRAME_SIZE(len);
    ext->type = (crsf_frame_type_e)type;
    ext->dest_addr = dest;
    ext->orig_addr = origin;
    if (payload && len > 0)
        memcpy(ext->payload, payload, len);
    ext->payload[len] = crsfRouter.crsf_crc.calc(&buf[2], len + 3);

    // #region agent log
    if (s_verbose_mode || s_debug_tx_logs < 8)
    {
        char hex[96];
        char dataJson[224];
        debug_bytes_hex(buf, 2 + ext->frame_size, hex, sizeof(hex));
        snprintf(
            dataJson,
            sizeof(dataJson),
            "{\"kind\":\"extended\",\"dest\":%u,\"origin\":%u,\"type\":%u,\"payloadLen\":%u,\"frame\":\"%s\"}",
            (unsigned)dest,
            (unsigned)origin,
            type,
            len,
            hex);
        debug_log_ndjson("CRSFSerialConnector.cpp:80", "uart_tx_frame", "repro1", "H3", dataJson);
        Serial.printf("[CRSF TX RAW] %s\n", hex);
        if (!s_verbose_mode) s_debug_tx_logs++;
    }
    // #endregion

#if defined(CRSF_CORE1_CHANNELS)
    pushToQueue(buf, 2 + ext->frame_size);
#else
    _port.write(buf, 2 + ext->frame_size);
#endif
}

void CRSFSerialConnector::handlePacket(const crsf_header_t *msg)
{
    const uint8_t *data = msg->payload;
    uint8_t payloadLen = msg->frame_size >= 2 ? (uint8_t)(msg->frame_size - 2) : 0;
    const crsf_ext_header_t *ext = (const crsf_ext_header_t *)msg;

    // #region agent log
    if (s_verbose_mode || s_debug_pkt_logs < 12)
    {
        char dataJson[160];
        snprintf(
            dataJson,
            sizeof(dataJson),
            "{\"type\":%u,\"frameSize\":%u,\"payloadLen\":%u,\"deviceAddr\":%u,\"dest\":%u,\"orig\":%u}",
            (unsigned)msg->type,
            (unsigned)msg->frame_size,
            (unsigned)payloadLen,
            (unsigned)msg->sync_byte,
            (unsigned)ext->dest_addr,
            (unsigned)ext->orig_addr);
        debug_log_ndjson("CRSFSerialConnector.cpp:108", "decoded_packet", "repro1", "H2", dataJson);
        if (s_verbose_mode)
            Serial.printf("[CRSF PKT] type=0x%02X frameSize=%u dest=0x%02X orig=0x%02X\n",
                          (unsigned)msg->type, (unsigned)msg->frame_size,
                          (unsigned)ext->dest_addr, (unsigned)ext->orig_addr);
        if (!s_verbose_mode) s_debug_pkt_logs++;
    }
    // #endregion

    if (msg->type >= CRSF_FRAMETYPE_DEVICE_PING && onExtendedFrame)
        onExtendedFrame(ext->orig_addr, (uint8_t)msg->type);

    switch (msg->type)
    {
    case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
        if (!_linkIsUp && onLinkUp)
            onLinkUp();
        _linkIsUp = true;
        _lastChannelsPacket = millis();
        if (onPacketChannels)
            onPacketChannels();
        break;

    case CRSF_FRAMETYPE_LINK_STATISTICS:
        if (payloadLen >= sizeof(crsfLinkStatistics_t))
        {
            _linkStatsPacketCount++;
            memcpy(&_linkStatistics, data, sizeof(crsfLinkStatistics_t));
            if (!_linkIsUp && onLinkUp)
                onLinkUp();
            _linkIsUp = true;
            _lastChannelsPacket = millis();
            if (onPacketLinkStatistics)
                onPacketLinkStatistics(&_linkStatistics);
        }
        break;

    case CRSF_FRAMETYPE_BATTERY_SENSOR:
        if (payloadLen >= 8)
        {
            uint16_t v = (data[0] << 8) | data[1];
            uint16_t c = (data[2] << 8) | data[3];
            _batteryVoltage = v / 10.0f;
            _batteryCurrent = c / 10.0f;
            _batteryCapacity = (data[4] << 16) | (data[5] << 8) | data[6];
            _batteryRemaining = data[7];
            if (!_linkIsUp && onLinkUp)
                onLinkUp();
            _linkIsUp = true;
            _lastChannelsPacket = millis();
            if (onPacketBattery)
                onPacketBattery(_batteryVoltage, _batteryCurrent, _batteryCapacity, _batteryRemaining);
        }
        break;

    case CRSF_FRAMETYPE_DEVICE_INFO:
    {
        // Extended frame: actual payload starts at ext->payload (offset 5), not msg->payload (offset 3).
        const uint8_t *extPayload = ext->payload;
        const uint8_t extPayloadLen = ext->frame_size >= 4 ? (uint8_t)(ext->frame_size - 4) : 0;
        if (extPayloadLen >= 10)
        {
            const char *name = (const char *)extPayload;
            size_t nameLen = strnlen(name, extPayloadLen) + 1;
            if (nameLen < extPayloadLen && extPayloadLen - nameLen >= sizeof(deviceInformationPacket_t))
            {
                const deviceInformationPacket_t *dev = (const deviceInformationPacket_t *)(extPayload + nameLen);
                const uint8_t *serial4 = (const uint8_t *)&dev->serialNo;
                if (onDeviceInfo)
                    onDeviceInfo(serial4, name, ext->orig_addr);
            }
            else if (extPayloadLen >= 10)
            {
                const uint8_t *serial4 = &extPayload[6];
                const char *nameAt10 = (const char *)&extPayload[10];
                if (nameAt10 - (const char *)extPayload < (int)extPayloadLen && onDeviceInfo)
                    onDeviceInfo(serial4, nameAt10, ext->orig_addr);
            }
        }
        break;
    }

    case CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY:
    {
        // Extended frame: actual payload starts at ext->payload (offset 5), not msg->payload (offset 3).
        const uint8_t *extPayload = ext->payload;
        const uint8_t extPayloadLen = ext->frame_size >= 4 ? (uint8_t)(ext->frame_size - 4) : 0;
        if (ext->orig_addr == CRSF_ADDRESS_CRSF_TRANSMITTER && extPayloadLen >= 2 && onParameterEntry)
        {
            const uint8_t fieldId = extPayload[0];
            const uint8_t chunksRemaining = extPayload[1];
            const bool typedFirstChunk = crsfx_parameterEntryHasTypedFrontMatter(extPayload, extPayloadLen);
            // First chunk: FieldId, ChunksRemain, parent (1), type (1), NUL-terminated label, then value blob.
            // Continuation chunks: FieldId + ChunksRemain only, then opaque bytes — see CRSFEndpoint::sendParameter.
            size_t labelMax = extPayloadLen > 4 ? (size_t)(extPayloadLen - 4) : 0;
            const char *label = labelMax ? (const char *)&extPayload[4] : "";
            uint8_t paramType = extPayloadLen > 3 ? (uint8_t)(extPayload[3] & 0x7F) : 0;
            size_t labelLen = 0;
            if (typedFirstChunk && labelMax > 0) {
                while (labelLen < labelMax && label[labelLen] != '\0')
                    labelLen++;
            }
            if (typedFirstChunk && extPayloadLen >= 6 && labelMax > 0 && labelLen < labelMax) {
                const uint8_t *value = (const uint8_t *)&label[labelLen + 1];
                uint8_t valueLen = extPayloadLen - 4 - (uint8_t)labelLen - 1;
                onParameterEntry(fieldId, paramType, chunksRemaining, label, value, valueLen);
            } else {
                uint8_t contLen = (uint8_t)(extPayloadLen > 2 ? extPayloadLen - 2 : 0);
                onParameterEntry(fieldId, 0, chunksRemaining, "", contLen ? &extPayload[2] : nullptr, contLen);
            }
        }
        break;
    }

    default:
        break;
    }
}

void CRSFSerialConnector::checkPacketTimeout()
{
    if (_lastReceive > 0 && millis() - _lastReceive > CRSF_PACKET_TIMEOUT_MS)
        _parser.Reset();
}

void CRSFSerialConnector::checkLinkDown()
{
    if (_linkIsUp && millis() - _lastChannelsPacket > CRSF_FAILSAFE_STAGE1_MS)
    {
        if (onLinkDown)
            onLinkDown();
        _linkIsUp = false;
    }
}
