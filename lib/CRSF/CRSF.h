#ifndef CRSF_H
#define CRSF_H

#include <Arduino.h>

#define CRSF_ADDRESS 0xEE
#define CRSF_TYPE_RC_CHANNELS 0x16
#define CRSF_MAX_CHANNELS 16
#define CRSF_BAUD_RATE 420000  // CRSF standard baud rate

class CRSF {
private:
    HardwareSerial* serial;
    uint8_t txPin;
    uint8_t rxPin;
    
    // Calculate CRC8 for CRSF frame
    uint8_t crc8(const uint8_t* data, uint8_t len);
    
public:
    CRSF(HardwareSerial* ser, uint8_t tx, uint8_t rx);
    void begin();
    void sendChannels(uint16_t channels[], uint8_t numChannels);
    void sendRCChannels(uint16_t channels[], uint8_t numChannels = 8);
};

#endif // CRSF_H



