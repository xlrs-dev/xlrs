#include "CRSF.h"

// CRC8 calculation for CRSF (polynomial 0xD5)
uint8_t CRSF::crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

CRSF::CRSF(HardwareSerial* ser, uint8_t tx, uint8_t rx) 
    : serial(ser), txPin(tx), rxPin(rx) {
}

void CRSF::begin() {
    // Initialize serial port for CRSF
    serial->begin(CRSF_BAUD_RATE, SERIAL_8N1, rxPin, txPin);
}

void CRSF::sendRCChannels(uint16_t channels[], uint8_t numChannels) {
    // CRSF RC Channels packet format:
    // Address (1 byte): 0xEE
    // Length (1 byte): payload length (type + data + CRC)
    // Type (1 byte): 0x16 for RC channels
    // Payload (22 bytes): 16 channels × 11 bits = 22 bytes
    // CRC (1 byte): CRC8 of address + length + type + payload
    
    if (numChannels > CRSF_MAX_CHANNELS) {
        numChannels = CRSF_MAX_CHANNELS;
    }
    
    // Build payload: 16 channels as 11-bit values
    uint8_t payload[22] = {0};
    
    // Pack channels into 11-bit values (CRSF format)
    // CRSF channels are packed sequentially: channel 0 bits 0-10, channel 1 bits 11-21, etc.
    // Total: 16 channels × 11 bits = 176 bits = 22 bytes
    uint32_t bitBuffer = 0;
    uint8_t bitCount = 0;
    uint8_t payloadIndex = 0;
    
    for (uint8_t i = 0; i < CRSF_MAX_CHANNELS; i++) {
        uint16_t channelValue;
        
        if (i < numChannels) {
            // Clamp channel value
            uint16_t usValue = channels[i];
            if (usValue < 1000) usValue = 1000;
            if (usValue > 2000) usValue = 2000;
            
            // Convert from microseconds (1000-2000) to CRSF 11-bit value (0-2047)
            // CRSF formula: crsfValue = (us - 988) * 1.5
            channelValue = ((usValue - 988) * 1.5);
            if (channelValue > 2047) channelValue = 2047;
        } else {
            // Fill remaining channels with center value (1500us)
            channelValue = ((1500 - 988) * 1.5);
        }
        
        // Add 11 bits to buffer
        bitBuffer |= ((uint32_t)channelValue) << bitCount;
        bitCount += 11;
        
        // Extract complete bytes
        while (bitCount >= 8) {
            payload[payloadIndex++] = bitBuffer & 0xFF;
            bitBuffer >>= 8;
            bitCount -= 8;
        }
    }
    
    // Handle remaining bits (should be exactly 0 bits left for 16 channels)
    if (bitCount > 0 && payloadIndex < 22) {
        payload[payloadIndex] = bitBuffer & 0xFF;
    }
    
    // Build frame
    uint8_t frame[26];  // Address + Length + Type + Payload + CRC
    uint8_t frameIndex = 0;
    
    // Address
    frame[frameIndex++] = CRSF_ADDRESS;
    
    // Length (type + payload + CRC = 1 + 22 + 1 = 24)
    frame[frameIndex++] = 24;
    
    // Type
    frame[frameIndex++] = CRSF_TYPE_RC_CHANNELS;
    
    // Payload
    for (uint8_t i = 0; i < 22; i++) {
        frame[frameIndex++] = payload[i];
    }
    
    // Calculate CRC (CRC is calculated on: length + type + payload, NOT including address)
    uint8_t crc = crc8(&frame[1], frameIndex - 1);  // Start from length byte
    frame[frameIndex++] = crc;
    
    // Send frame
    serial->write(frame, frameIndex);
}

void CRSF::sendChannels(uint16_t channels[], uint8_t numChannels) {
    sendRCChannels(channels, numChannels);
}

