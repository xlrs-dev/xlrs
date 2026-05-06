#pragma once

#include <Arduino.h>
#if defined(CRSF_CORE1_CHANNELS)
#include "pico/mutex.h"
#endif

#include "CRSFConnector.h"
#include "CRSFParser.h"
#include "CRSFRouter.h"
#include "crsf_protocol.h"

/**
 * Serial transport connector for CRSF over HardwareSerial.
 * Feeds bytes into CRSFParser and forwards outbound frames to the UART.
 * Exposes the same callback interface as CrsfSerial for RC app compatibility.
 */
class CRSFSerialConnector : public CRSFConnector
{
public:
    static const unsigned int CRSF_PACKET_TIMEOUT_MS = 100;
    static const unsigned int CRSF_FAILSAFE_STAGE1_MS = 300;
#if defined(RC_TX_MODULE_UART_BAUD)
    static const uint32_t CRSF_BAUDRATE = RC_TX_MODULE_UART_BAUD;
#else
    static const uint32_t CRSF_BAUDRATE = 420000;
#endif
/** Min spacing between handset RC_CHANNELS frames (µs). Default to 4 ms to match upstream SimpleTX 250 Hz cadence. */
#if !defined(RC_CRSF_SIMPLETX_FRAME_US)
#define RC_CRSF_SIMPLETX_FRAME_US 4000u
#endif
#if !defined(RC_SIMPLETX_BOOT_CHANNEL_PACKETS)
#define RC_SIMPLETX_BOOT_CHANNEL_PACKETS 500u
#endif
    static const uint16_t SIMPLETX_FRAME_SPACING_US = (uint16_t)RC_CRSF_SIMPLETX_FRAME_US;
    static const uint16_t SIMPLETX_BOOT_CHANNEL_PACKETS = (uint16_t)RC_SIMPLETX_BOOT_CHANNEL_PACKETS;

    static const uint8_t CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE = 22;
    static const uint8_t CRSF_MAX_PAYLOAD_LEN = CRSF_PAYLOAD_SIZE_MAX;
    /** Full CRSF RC_CHANNELS_PACKED frame (SimpleTX CRSF_PACKET_SIZE = 26), for boot warmup / tooling */
    static const uint8_t CRSF_PACKET_RC_CHANNELS_FULL = 26;

    explicit CRSFSerialConnector(HardwareSerial &port, uint32_t baud = CRSF_BAUDRATE);

    void begin(uint32_t baud = 0);
    /** Clear parser state (call after Serial2.end/begin or baud changes). */
    void resetRxParser();
    void loop();
#if defined(CRSF_CORE1_CHANNELS)
    void initTxMutex();  // Call from setup() before Core 1 starts
    /** Pop and transmit all queued outbound frames immediately (call from Core 0 during setup when Core 1 is not draining yet). */
    void flushOutboundQueue();
#endif
    /**
     * Transmit a complete pre-built CRSF frame (e.g. SimpleTX 26-byte RC or 8-byte cmd).
     * With CRSF_CORE1_CHANNELS: enqueued for Core 1. Otherwise: immediate write.
     */
    void queueCrsfRawTx(const uint8_t *bytes, uint8_t len);
    /** Calls queueCrsfRawTx with SimpleTX RC_CHANNELS 26-byte packing (GPL SimpleTxCrsf). */
    void queueSimpleTxRcChannelsPacked8(const uint16_t channels_us[8]);

    void forwardMessage(const crsf_header_t *message) override;

    void queuePacket(uint8_t addr, uint8_t type, const void *payload, uint8_t len);
    void queueExtendedPacketRaw(crsf_addr_e sync, crsf_addr_e dest, crsf_addr_e origin, uint8_t type, const void *payload, uint8_t len);
    void queueExtendedPacket(crsf_addr_e dest, crsf_addr_e origin, uint8_t type, const void *payload, uint8_t len);
#if defined(CRSF_CORE1_CHANNELS)
    // Core 1 calls this: drains pending packets from Core 0, then optionally sends channel packet. Core 1 is sole Serial2 writer.
    void core1_drainAndSendChannels(const uint16_t channels[8], bool sendChannels = true);
#endif

    const crsfLinkStatistics_t *getLinkStatistics() const { return &_linkStatistics; }
    uint32_t getLinkStatsPacketCount() const { return _linkStatsPacketCount; }
    void resetLinkStatsPacketCount() { _linkStatsPacketCount = 0; }
    float getBatteryVoltage() const { return _batteryVoltage; }
    float getBatteryCurrent() const { return _batteryCurrent; }
    uint32_t getBatteryCapacity() const { return _batteryCapacity; }
    uint8_t getBatteryRemaining() const { return _batteryRemaining; }
    bool isLinkUp() const { return _linkIsUp; }

    void setVerboseMode(bool enabled);

    void (*onLinkUp)();
    void (*onLinkDown)();
    void (*onOobData)(uint8_t b);
    void (*onPacketChannels)();
    void (*onPacketLinkStatistics)(crsfLinkStatistics_t *ls);
    void (*onPacketBattery)(float voltage, float current, uint32_t capacity, uint8_t remaining);
    void (*onExtendedFrame)(uint8_t sourceAddr, uint8_t type);
    void (*onDeviceInfo)(const uint8_t *serial4, const char *name, uint8_t sourceAddr);
    void (*onParameterEntry)(uint8_t fieldId, uint8_t paramType, uint8_t chunksRemaining, const char *label, const uint8_t *value, uint8_t valueLen);

private:
    HardwareSerial &_port;
    uint32_t _baud;
    CRSFParser _parser;

    crsfLinkStatistics_t _linkStatistics{};
    uint32_t _linkStatsPacketCount = 0;
    float _batteryVoltage = 0;
    float _batteryCurrent = 0;
    uint32_t _batteryCapacity = 0;
    uint8_t _batteryRemaining = 0;
    uint32_t _lastReceive = 0;
    uint32_t _lastChannelsPacket = 0;
    bool _linkIsUp = false;

    void handlePacket(const crsf_header_t *msg);
    void checkPacketTimeout();
    void checkLinkDown();
#if defined(CRSF_CORE1_CHANNELS)
    static mutex_t s_queue_mutex;
    static const unsigned int CORE1_QUEUE_SIZE = 48;
    static uint8_t s_queue_len[CORE1_QUEUE_SIZE];
    static uint8_t s_queue_data[CORE1_QUEUE_SIZE][64];
    static volatile unsigned int s_queue_head;
    static volatile unsigned int s_queue_tail;
    void pushToQueue(const uint8_t* data, uint8_t len);
    bool popFromQueue(uint8_t* out, uint8_t* outLen);
#endif
};
