#ifndef SECURITY_H
#define SECURITY_H

#include <Arduino.h>
#include <stdint.h>

#define PAIRING_KEY_SIZE 16
#define HMAC_SIZE 4  // Truncated HMAC-SHA256 (first 4 bytes)
#define SEQUENCE_SIZE 2

// EEPROM address for pairing key (after WiFi config)
#define PAIRING_KEY_ADDR 128

class Security {
public:
    Security();
    
    // Initialize security (load pairing key from EEPROM or generate new one)
    bool begin();
    
    // Generate or set pairing key
    bool generatePairingKey();
    bool setPairingKey(const uint8_t* key);
    bool getPairingKey(uint8_t* key);
    
    // Calculate HMAC for data (truncated to 4 bytes)
    void calculateHMAC(const uint8_t* data, size_t dataLen, uint8_t* hmac);
    
    // Verify HMAC
    bool verifyHMAC(const uint8_t* data, size_t dataLen, const uint8_t* hmac);
    
    // Check if pairing key exists
    bool hasPairingKey();
    
private:
    uint8_t _pairingKey[PAIRING_KEY_SIZE];
    bool _keyLoaded;
    
    // Simple HMAC-SHA256 implementation (lightweight)
    void sha256(const uint8_t* data, size_t len, uint8_t* hash);
    void hmac_sha256(const uint8_t* key, size_t keyLen, const uint8_t* data, size_t dataLen, uint8_t* hmac);
};

#endif // SECURITY_H

