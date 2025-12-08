#include "Protocol.h"
#include "Security.h"
#include <string.h>

void Protocol::encodeFrame(uint16_t channels[NUM_CHANNELS], uint16_t sequence, uint32_t timestampMs, Security* security, uint8_t* frame) {
    // Encode channel data (first 16 bytes)
    for (int i = 0; i < NUM_CHANNELS; i++) {
        // Clamp channel values to valid range
        uint16_t value = channels[i];
        if (value < CHANNEL_MIN) value = CHANNEL_MIN;
        if (value > CHANNEL_MAX) value = CHANNEL_MAX;
        
        // Store as big-endian (high byte first)
        frame[i * 2] = (value >> 8) & 0xFF;
        frame[i * 2 + 1] = value & 0xFF;
    }
    
    // Encode sequence number (bytes 16-17)
    frame[CHANNEL_DATA_SIZE] = (sequence >> 8) & 0xFF;
    frame[CHANNEL_DATA_SIZE + 1] = sequence & 0xFF;
    
    // Encode TX timestamp in milliseconds (bytes 18-21), big-endian
    frame[CHANNEL_DATA_SIZE + SEQUENCE_SIZE + 0] = (timestampMs >> 24) & 0xFF;
    frame[CHANNEL_DATA_SIZE + SEQUENCE_SIZE + 1] = (timestampMs >> 16) & 0xFF;
    frame[CHANNEL_DATA_SIZE + SEQUENCE_SIZE + 2] = (timestampMs >> 8) & 0xFF;
    frame[CHANNEL_DATA_SIZE + SEQUENCE_SIZE + 3] = timestampMs & 0xFF;
    
    // Calculate HMAC for channel data + sequence + timestamp (bytes 0-21)
    if (security) {
        uint8_t hmac[HMAC_SIZE];
        security->calculateHMAC(frame, CHANNEL_DATA_SIZE + SEQUENCE_SIZE + TIMESTAMP_SIZE, hmac);
        memcpy(frame + CHANNEL_DATA_SIZE + SEQUENCE_SIZE + TIMESTAMP_SIZE, hmac, HMAC_SIZE);
    } else {
        // No security - fill with zeros (for testing)
        memset(frame + CHANNEL_DATA_SIZE + SEQUENCE_SIZE + TIMESTAMP_SIZE, 0, HMAC_SIZE);
    }
}

bool Protocol::decodeFrame(const uint8_t* frame, uint16_t channels[NUM_CHANNELS], uint16_t* sequence, uint32_t* timestampMs, Security* security, uint16_t* lastSequence) {
    if (!validateFrame(frame)) {
        return false;
    }
    
    // Decode channel data
    for (int i = 0; i < NUM_CHANNELS; i++) {
        // Read as big-endian
        channels[i] = (frame[i * 2] << 8) | frame[i * 2 + 1];
        
        // Validate range
        if (channels[i] < CHANNEL_MIN || channels[i] > CHANNEL_MAX) {
            return false;
        }
    }
    
    // Decode sequence number
    if (sequence) {
        *sequence = (frame[CHANNEL_DATA_SIZE] << 8) | frame[CHANNEL_DATA_SIZE + 1];
    }
    
    // Decode timestamp (ms)
    if (timestampMs) {
        *timestampMs =
            (static_cast<uint32_t>(frame[CHANNEL_DATA_SIZE + SEQUENCE_SIZE + 0]) << 24) |
            (static_cast<uint32_t>(frame[CHANNEL_DATA_SIZE + SEQUENCE_SIZE + 1]) << 16) |
            (static_cast<uint32_t>(frame[CHANNEL_DATA_SIZE + SEQUENCE_SIZE + 2]) << 8) |
            static_cast<uint32_t>(frame[CHANNEL_DATA_SIZE + SEQUENCE_SIZE + 3]);
    }
    
    // Verify HMAC if security is enabled
    if (security) {
        uint8_t receivedHmac[HMAC_SIZE];
        memcpy(receivedHmac, frame + CHANNEL_DATA_SIZE + SEQUENCE_SIZE + TIMESTAMP_SIZE, HMAC_SIZE);
        
        // Verify HMAC
        if (!security->verifyHMAC(frame, CHANNEL_DATA_SIZE + SEQUENCE_SIZE + TIMESTAMP_SIZE, receivedHmac)) {
            return false;  // HMAC verification failed
        }
        
        // Check sequence number (prevent replay attacks)
        if (lastSequence && sequence) {
            uint16_t seq = *sequence;
            uint16_t lastSeq = *lastSequence;
            
            // Allow sequence to wrap around, but reject if it's too old (within last 1000 packets)
            if (seq != 0 && seq < lastSeq && (lastSeq - seq) > 1000) {
                return false;  // Sequence too old (replay attack)
            }
            
            // Update last sequence
            if (seq > lastSeq || (lastSeq > 60000 && seq < 1000)) {
                *lastSequence = seq;
            }
        }
    }
    
    return true;
}

bool Protocol::validateFrame(const uint8_t* frame) {
    if (!frame) return false;
    
    // Simple validation: check that frame is not all zeros or all 0xFF
    bool allZero = true;
    bool allFF = true;
    
    for (int i = 0; i < CHANNEL_DATA_SIZE; i++) {
        if (frame[i] != 0) allZero = false;
        if (frame[i] != 0xFF) allFF = false;
    }
    
    return !allZero && !allFF;
}

void Protocol::createDiscoveryPacket(const IPAddress& ip, char* buffer, size_t bufferSize) {
    if (!buffer || bufferSize < 32) return;
    
    snprintf(buffer, bufferSize, "%s%d.%d.%d.%d", 
             DISCOVERY_MAGIC, 
             ip[0], ip[1], ip[2], ip[3]);
}

bool Protocol::parseDiscoveryPacket(const char* packet, IPAddress& ip) {
    if (!packet) return false;
    
    // Check for magic string
    if (strncmp(packet, DISCOVERY_MAGIC, strlen(DISCOVERY_MAGIC)) != 0) {
        return false;
    }
    
    // Parse IP address
    const char* ipStr = packet + strlen(DISCOVERY_MAGIC);
    uint8_t octets[4];
    int parsed = sscanf(ipStr, "%hhu.%hhu.%hhu.%hhu", 
                        &octets[0], &octets[1], &octets[2], &octets[3]);
    
    if (parsed == 4) {
        ip = IPAddress(octets[0], octets[1], octets[2], octets[3]);
        return true;
    }
    
    return false;
}

