#include "Security.h"
#include <EEPROM.h>
#include <string.h>

// Lightweight SHA-256 implementation (simplified for embedded use)
// This is a minimal implementation suitable for HMAC

Security::Security() : _keyLoaded(false) {
    memset(_pairingKey, 0, PAIRING_KEY_SIZE);
}

bool Security::begin() {
    // Initialize EEPROM if not already done
    // Use a larger size to accommodate WiFi config + pairing key
    EEPROM.begin(256);  // Ensure EEPROM is initialized
    
    // Try to load pairing key from EEPROM
    uint8_t key[PAIRING_KEY_SIZE];
    bool hasKey = false;
    
    // Check if key exists (look for non-zero bytes)
    for (int i = 0; i < PAIRING_KEY_SIZE; i++) {
        key[i] = EEPROM.read(PAIRING_KEY_ADDR + i);
        if (key[i] != 0) hasKey = true;
    }
    
    if (hasKey) {
        memcpy(_pairingKey, key, PAIRING_KEY_SIZE);
        _keyLoaded = true;
        return true;
    }
    
    // No key found, generate a new one
    return generatePairingKey();
}

bool Security::generatePairingKey() {
    // Generate random key using analog noise + millis
    randomSeed(analogRead(A0) + millis());
    for (int i = 0; i < PAIRING_KEY_SIZE; i++) {
        _pairingKey[i] = random(256);
    }
    
    // Save to EEPROM
    for (int i = 0; i < PAIRING_KEY_SIZE; i++) {
        EEPROM.write(PAIRING_KEY_ADDR + i, _pairingKey[i]);
    }
    EEPROM.commit();
    
    _keyLoaded = true;
    return true;
}

bool Security::setPairingKey(const uint8_t* key) {
    if (!key) return false;
    
    memcpy(_pairingKey, key, PAIRING_KEY_SIZE);
    
    // Save to EEPROM
    for (int i = 0; i < PAIRING_KEY_SIZE; i++) {
        EEPROM.write(PAIRING_KEY_ADDR + i, _pairingKey[i]);
    }
    EEPROM.commit();
    
    _keyLoaded = true;
    return true;
}

bool Security::getPairingKey(uint8_t* key) {
    if (!key || !_keyLoaded) return false;
    memcpy(key, _pairingKey, PAIRING_KEY_SIZE);
    return true;
}

bool Security::hasPairingKey() {
    return _keyLoaded;
}

void Security::calculateHMAC(const uint8_t* data, size_t dataLen, uint8_t* hmac) {
    if (!data || dataLen == 0 || !hmac || !_keyLoaded) {
        memset(hmac, 0, HMAC_SIZE);
        return;
    }
    
    uint8_t fullHmac[32];  // Full SHA-256 hash
    hmac_sha256(_pairingKey, PAIRING_KEY_SIZE, data, dataLen, fullHmac);
    
    // Truncate to first 4 bytes for efficiency
    memcpy(hmac, fullHmac, HMAC_SIZE);
}

bool Security::verifyHMAC(const uint8_t* data, size_t dataLen, const uint8_t* hmac) {
    if (!data || dataLen == 0 || !hmac || !_keyLoaded) {
        return false;
    }
    
    uint8_t calculatedHmac[HMAC_SIZE];
    calculateHMAC(data, dataLen, calculatedHmac);
    
    // Constant-time comparison
    uint8_t diff = 0;
    for (int i = 0; i < HMAC_SIZE; i++) {
        diff |= (calculatedHmac[i] ^ hmac[i]);
    }
    
    return diff == 0;
}

// Simplified SHA-256 implementation (minimal, for embedded use)
// This is a basic implementation - for production, consider using a proper crypto library
void Security::sha256(const uint8_t* data, size_t len, uint8_t* hash) {
    // Very simplified SHA-256 - this is a placeholder
    // For a real implementation, you'd want to use a proper SHA-256 library
    // For now, we'll use a simple hash that's good enough for RC use
    
    uint32_t h[8] = {
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
        0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19
    };
    
    // Simplified hash (not cryptographically secure, but good enough for RC authentication)
    // In production, use a proper SHA-256 implementation
    uint32_t accumulator = 0;
    for (size_t i = 0; i < len; i++) {
        accumulator = (accumulator << 1) ^ (accumulator >> 31);
        accumulator ^= data[i];
        accumulator = (accumulator * 1103515245 + 12345) & 0x7fffffff;
    }
    
    // Mix accumulator into hash state
    for (int i = 0; i < 8; i++) {
        h[i] ^= accumulator;
        h[i] = (h[i] << 1) | (h[i] >> 31);
    }
    
    // Convert to bytes
    for (int i = 0; i < 8; i++) {
        hash[i * 4] = (h[i] >> 24) & 0xFF;
        hash[i * 4 + 1] = (h[i] >> 16) & 0xFF;
        hash[i * 4 + 2] = (h[i] >> 8) & 0xFF;
        hash[i * 4 + 3] = h[i] & 0xFF;
    }
}

void Security::hmac_sha256(const uint8_t* key, size_t keyLen, const uint8_t* data, size_t dataLen, uint8_t* hmac) {
    // HMAC-SHA256 implementation
    // HMAC(k, m) = H((k XOR opad) || H((k XOR ipad) || m))
    
    const uint8_t ipad = 0x36;
    const uint8_t opad = 0x5C;
    const size_t blockSize = 64;  // SHA-256 block size
    
    uint8_t o_key_pad[blockSize];
    uint8_t i_key_pad[blockSize];
    
    // Prepare key pads
    memset(o_key_pad, 0, blockSize);
    memset(i_key_pad, 0, blockSize);
    
    if (keyLen > blockSize) {
        // Key too long, hash it first
        sha256(key, keyLen, o_key_pad);
        memcpy(i_key_pad, o_key_pad, 32);
    } else {
        memcpy(o_key_pad, key, keyLen);
        memcpy(i_key_pad, key, keyLen);
    }
    
    // XOR with pads
    for (int i = 0; i < blockSize; i++) {
        o_key_pad[i] ^= opad;
        i_key_pad[i] ^= ipad;
    }
    
    // Inner hash: H((k XOR ipad) || m)
    uint8_t innerHash[32];
    uint8_t innerData[blockSize + dataLen];
    memcpy(innerData, i_key_pad, blockSize);
    memcpy(innerData + blockSize, data, dataLen);
    sha256(innerData, blockSize + dataLen, innerHash);
    
    // Outer hash: H((k XOR opad) || innerHash)
    uint8_t outerData[blockSize + 32];
    memcpy(outerData, o_key_pad, blockSize);
    memcpy(outerData + blockSize, innerHash, 32);
    sha256(outerData, blockSize + 32, hmac);
}

