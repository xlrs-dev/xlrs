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
#define FRAME_SIZE (CHANNEL_DATA_SIZE + SEQUENCE_SIZE + TIMESTAMP_SIZE + HMAC_SIZE)  // 26 bytes total

// Channel value range: 1000-2000 microseconds (standard RC)
#define CHANNEL_MIN 1000
#define CHANNEL_MAX 2000
#define CHANNEL_CENTER 1500

// Discovery protocol
#define DISCOVERY_PORT 8889
#define DISCOVERY_MAGIC "FPV_RX_IP:"

class Protocol {
public:
    // Encode channel values into binary frame with security
    // frame must be at least FRAME_SIZE bytes
    static void encodeFrame(uint16_t channels[NUM_CHANNELS], uint16_t sequence, uint32_t timestampMs, Security* security, uint8_t* frame);
    
    // Decode binary frame into channel values and verify security
    // Returns true if frame is valid and security checks pass
    static bool decodeFrame(const uint8_t* frame, uint16_t channels[NUM_CHANNELS], uint16_t* sequence, uint32_t* timestampMs, Security* security, uint16_t* lastSequence);
    
    // Validate frame structure (basic checksum)
    static bool validateFrame(const uint8_t* frame);
    
    // Create discovery packet (simple text format)
    static void createDiscoveryPacket(const IPAddress& ip, char* buffer, size_t bufferSize);
    
    // Parse discovery packet
    static bool parseDiscoveryPacket(const char* packet, IPAddress& ip);
};

#endif // PROTOCOL_H

