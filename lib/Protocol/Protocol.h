#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>

// Forward declaration
class Security;

#define NUM_CHANNELS 8
#define CHANNEL_DATA_SIZE (NUM_CHANNELS * 2)  // 16 bytes (2 bytes per channel)
#define SEQUENCE_SIZE 2  // 2 bytes for sequence number
#define TIMESTAMP_SIZE 4 // 4 bytes for TX timestamp (ms)
#define HMAC_SIZE 4  // 4 bytes for HMAC
#define DEVICE_ID_SIZE 8  // 8 bytes for device ID
#define FRAME_SIZE (DEVICE_ID_SIZE + CHANNEL_DATA_SIZE + SEQUENCE_SIZE + HMAC_SIZE)  // 30 bytes total (8 + 16 + 2 + 4)

// Channel value range: 1000-2000 microseconds (standard RC)
#define CHANNEL_MIN 1000
#define CHANNEL_MAX 2000
#define CHANNEL_CENTER 1500

// Discovery protocol
#define DISCOVERY_PORT 8889
#define DISCOVERY_MAGIC "FPV_RX_IP:"

// Connection states
enum ConnectionState {
    STATE_DISCONNECTED = 0,
    STATE_PAIRING = 1,
    STATE_CONNECTING = 2,
    STATE_CONNECTED = 3,
    STATE_LOST = 4
};

// Radio message types
enum RadioMsgType : uint8_t {
    MSG_CHANNELS = 0x01,
    MSG_BATTERY  = 0x02,
    MSG_PAIRING  = 0x03,
    MSG_PAIRING_ACK = 0x04,
    MSG_SYNC     = 0x05,
    MSG_SYNC_ACK = 0x06,
    MSG_CONNECTION_STATUS = 0x07
};

class Protocol {
public:
    // Encode channel values into binary frame with security, device ID, and encryption
    // frame must be at least FRAME_SIZE bytes
    // deviceId: This device's ID (TX device ID for TX side, RX device ID for RX side)
    static void encodeFrame(uint16_t channels[NUM_CHANNELS], uint16_t sequence, Security* security, const uint8_t* deviceId, uint8_t* frame);
    
    // Decode binary frame into channel values and verify security, device ID, and decrypt
    // Returns true if frame is valid, device ID matches, and security checks pass
    // expectedDeviceId: The device ID we expect to receive from (TX ID for RX, RX ID for TX)
    static bool decodeFrame(const uint8_t* frame, uint16_t channels[NUM_CHANNELS], uint16_t* sequence, Security* security, uint16_t* lastSequence, const uint8_t* expectedDeviceId);
    
    // Validate frame structure (basic checksum)
    static bool validateFrame(const uint8_t* frame);
    
    // Create discovery packet (simple text format)
    static void createDiscoveryPacket(const IPAddress& ip, char* buffer, size_t bufferSize);
    
    // Parse discovery packet
    static bool parseDiscoveryPacket(const char* packet, IPAddress& ip);
    
    // Sync packet functions
    static void createSyncPacket(uint16_t syncSeq, uint32_t timestamp, Security* security, const uint8_t* deviceId, uint8_t* packet, size_t* packetLen);
    static bool parseSyncPacket(const uint8_t* packet, size_t packetLen, uint16_t* syncSeq, uint32_t* timestamp, Security* security, uint8_t* sourceDeviceId);
    static void createSyncAck(uint16_t syncSeq, uint32_t timestamp, Security* security, const uint8_t* deviceId, uint8_t* packet, size_t* packetLen);
    static bool parseSyncAck(const uint8_t* packet, size_t packetLen, uint16_t* syncSeq, uint32_t* timestamp, Security* security, uint8_t* sourceDeviceId);
};

#endif // PROTOCOL_H

