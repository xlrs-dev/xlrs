#include "Protocol.h"
#include "Security.h"
#include <string.h>

void Protocol::encodeFrame(uint16_t channels[NUM_CHANNELS], uint16_t sequence, Security* security, const uint8_t* deviceId, uint8_t* frame) {
    if (!frame || !deviceId) return;
    
    // Encode device ID (first 8 bytes)
    memcpy(frame, deviceId, DEVICE_ID_SIZE);
    
    // Encode channel data (bytes 8-23)
    uint8_t plaintext[CHANNEL_DATA_SIZE];
    for (int i = 0; i < NUM_CHANNELS; i++) {
        // Clamp channel values to valid range
        uint16_t value = channels[i];
        if (value < CHANNEL_MIN) value = CHANNEL_MIN;
        if (value > CHANNEL_MAX) value = CHANNEL_MAX;
        
        // Store as big-endian (high byte first)
        plaintext[i * 2] = (value >> 8) & 0xFF;
        plaintext[i * 2 + 1] = value & 0xFF;
    }
    
    // Encrypt channel data
    uint8_t encrypted[CHANNEL_DATA_SIZE];
    if (security && security->hasPairingKey()) {
        if (!security->encrypt(plaintext, CHANNEL_DATA_SIZE, encrypted, sequence, deviceId)) {
            // Encryption failed, use plaintext (should not happen)
            memcpy(encrypted, plaintext, CHANNEL_DATA_SIZE);
        }
    } else {
        // No security - use plaintext (for testing only)
        memcpy(encrypted, plaintext, CHANNEL_DATA_SIZE);
    }
    memcpy(frame + DEVICE_ID_SIZE, encrypted, CHANNEL_DATA_SIZE);
    
    // Encode sequence number (bytes 24-25)
    frame[DEVICE_ID_SIZE + CHANNEL_DATA_SIZE] = (sequence >> 8) & 0xFF;
    frame[DEVICE_ID_SIZE + CHANNEL_DATA_SIZE + 1] = sequence & 0xFF;
    
    // Calculate HMAC for device ID + encrypted data + sequence
    if (security && security->hasPairingKey()) {
        uint8_t hmac[HMAC_SIZE];
        security->calculateHMAC(frame, DEVICE_ID_SIZE + CHANNEL_DATA_SIZE + SEQUENCE_SIZE, hmac);
        memcpy(frame + DEVICE_ID_SIZE + CHANNEL_DATA_SIZE + SEQUENCE_SIZE, hmac, HMAC_SIZE);
    } else {
        // No security - fill with zeros (for testing)
        memset(frame + DEVICE_ID_SIZE + CHANNEL_DATA_SIZE + SEQUENCE_SIZE, 0, HMAC_SIZE);
    }
}

bool Protocol::decodeFrame(const uint8_t* frame, uint16_t channels[NUM_CHANNELS], uint16_t* sequence, Security* security, uint16_t* lastSequence, const uint8_t* expectedDeviceId) {
    if (!frame || !expectedDeviceId) {
        return false;
    }
    
    // CRITICAL: Verify device ID matches expected (1-to-1 enforcement)
    if (memcmp(frame, expectedDeviceId, DEVICE_ID_SIZE) != 0) {
        return false;  // Device ID mismatch - reject packet
    }
    
    // Verify HMAC if security is enabled
    if (security && security->hasPairingKey()) {
        uint8_t receivedHmac[HMAC_SIZE];
        memcpy(receivedHmac, frame + DEVICE_ID_SIZE + CHANNEL_DATA_SIZE + SEQUENCE_SIZE, HMAC_SIZE);
        
        // Verify HMAC (over device ID + encrypted data + sequence)
        if (!security->verifyHMAC(frame, DEVICE_ID_SIZE + CHANNEL_DATA_SIZE + SEQUENCE_SIZE, receivedHmac)) {
            return false;  // HMAC verification failed
        }
    }
    
    // Decode sequence number (before decryption, needed for decryption key)
    uint16_t seq = 0;
    if (sequence) {
        seq = (frame[DEVICE_ID_SIZE + CHANNEL_DATA_SIZE] << 8) | frame[DEVICE_ID_SIZE + CHANNEL_DATA_SIZE + 1];
        *sequence = seq;
    }
    
    // Decrypt channel data
    uint8_t encrypted[CHANNEL_DATA_SIZE];
    memcpy(encrypted, frame + DEVICE_ID_SIZE, CHANNEL_DATA_SIZE);
    
    uint8_t plaintext[CHANNEL_DATA_SIZE];
    if (security && security->hasPairingKey()) {
        if (!security->decrypt(encrypted, CHANNEL_DATA_SIZE, plaintext, seq, expectedDeviceId)) {
            return false;  // Decryption failed
        }
    } else {
        // No security - use plaintext (for testing only)
        memcpy(plaintext, encrypted, CHANNEL_DATA_SIZE);
    }
    
    // Decode channel data
    for (int i = 0; i < NUM_CHANNELS; i++) {
        // Read as big-endian
        channels[i] = (plaintext[i * 2] << 8) | plaintext[i * 2 + 1];
        
        // Validate range
        if (channels[i] < CHANNEL_MIN || channels[i] > CHANNEL_MAX) {
            return false;
        }
    }
    
    // Check sequence number (prevent replay attacks)
    if (security && lastSequence && sequence) {
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

// Sync packet format: [MSG_TYPE: 1][DEVICE_ID: 8][SYNC_SEQ: 2][TIMESTAMP: 4][HMAC: 4] = 19 bytes
void Protocol::createSyncPacket(uint16_t syncSeq, uint32_t timestamp, Security* security, const uint8_t* deviceId, uint8_t* packet, size_t* packetLen) {
    if (!packet || !deviceId || !packetLen) return;
    
    size_t offset = 0;
    packet[offset++] = MSG_SYNC;
    memcpy(packet + offset, deviceId, DEVICE_ID_SIZE);
    offset += DEVICE_ID_SIZE;
    packet[offset++] = (syncSeq >> 8) & 0xFF;
    packet[offset++] = syncSeq & 0xFF;
    packet[offset++] = (timestamp >> 24) & 0xFF;
    packet[offset++] = (timestamp >> 16) & 0xFF;
    packet[offset++] = (timestamp >> 8) & 0xFF;
    packet[offset++] = timestamp & 0xFF;
    
    // Calculate HMAC
    if (security && security->hasPairingKey()) {
        uint8_t hmac[HMAC_SIZE];
        security->calculateHMAC(packet, offset, hmac);
        memcpy(packet + offset, hmac, HMAC_SIZE);
        offset += HMAC_SIZE;
    } else {
        memset(packet + offset, 0, HMAC_SIZE);
        offset += HMAC_SIZE;
    }
    
    *packetLen = offset;
}

bool Protocol::parseSyncPacket(const uint8_t* packet, size_t packetLen, uint16_t* syncSeq, uint32_t* timestamp, Security* security, uint8_t* sourceDeviceId) {
    if (!packet || packetLen < 1 + DEVICE_ID_SIZE + 2 + 4 + HMAC_SIZE) return false;
    
    if (packet[0] != MSG_SYNC) return false;
    
    size_t offset = 1;
    if (sourceDeviceId) {
        memcpy(sourceDeviceId, packet + offset, DEVICE_ID_SIZE);
    }
    offset += DEVICE_ID_SIZE;
    
    if (syncSeq) {
        *syncSeq = (packet[offset] << 8) | packet[offset + 1];
    }
    offset += 2;
    
    if (timestamp) {
        *timestamp = ((uint32_t)packet[offset] << 24) | ((uint32_t)packet[offset + 1] << 16) | 
                     ((uint32_t)packet[offset + 2] << 8) | packet[offset + 3];
    }
    offset += 4;
    
    // Verify HMAC
    if (security && security->hasPairingKey()) {
        uint8_t receivedHmac[HMAC_SIZE];
        memcpy(receivedHmac, packet + offset, HMAC_SIZE);
        if (!security->verifyHMAC(packet, offset, receivedHmac)) {
            return false;
        }
    }
    
    return true;
}

void Protocol::createSyncAck(uint16_t syncSeq, uint32_t timestamp, Security* security, const uint8_t* deviceId, uint8_t* packet, size_t* packetLen) {
    if (!packet || !deviceId || !packetLen) return;
    
    size_t offset = 0;
    packet[offset++] = MSG_SYNC_ACK;
    memcpy(packet + offset, deviceId, DEVICE_ID_SIZE);
    offset += DEVICE_ID_SIZE;
    packet[offset++] = (syncSeq >> 8) & 0xFF;
    packet[offset++] = syncSeq & 0xFF;
    packet[offset++] = (timestamp >> 24) & 0xFF;
    packet[offset++] = (timestamp >> 16) & 0xFF;
    packet[offset++] = (timestamp >> 8) & 0xFF;
    packet[offset++] = timestamp & 0xFF;
    
    // Calculate HMAC
    if (security && security->hasPairingKey()) {
        uint8_t hmac[HMAC_SIZE];
        security->calculateHMAC(packet, offset, hmac);
        memcpy(packet + offset, hmac, HMAC_SIZE);
        offset += HMAC_SIZE;
    } else {
        memset(packet + offset, 0, HMAC_SIZE);
        offset += HMAC_SIZE;
    }
    
    *packetLen = offset;
}

bool Protocol::parseSyncAck(const uint8_t* packet, size_t packetLen, uint16_t* syncSeq, uint32_t* timestamp, Security* security, uint8_t* sourceDeviceId) {
    if (!packet || packetLen < 1 + DEVICE_ID_SIZE + 2 + 4 + HMAC_SIZE) return false;
    
    if (packet[0] != MSG_SYNC_ACK) return false;
    
    size_t offset = 1;
    if (sourceDeviceId) {
        memcpy(sourceDeviceId, packet + offset, DEVICE_ID_SIZE);
    }
    offset += DEVICE_ID_SIZE;
    
    if (syncSeq) {
        *syncSeq = (packet[offset] << 8) | packet[offset + 1];
    }
    offset += 2;
    
    if (timestamp) {
        *timestamp = ((uint32_t)packet[offset] << 24) | ((uint32_t)packet[offset + 1] << 16) | 
                     ((uint32_t)packet[offset + 2] << 8) | packet[offset + 3];
    }
    offset += 4;
    
    // Verify HMAC
    if (security && security->hasPairingKey()) {
        uint8_t receivedHmac[HMAC_SIZE];
        memcpy(receivedHmac, packet + offset, HMAC_SIZE);
        if (!security->verifyHMAC(packet, offset, receivedHmac)) {
            return false;
        }
    }
    
    return true;
}

