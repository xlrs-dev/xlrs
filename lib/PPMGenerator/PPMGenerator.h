#ifndef PPM_GENERATOR_H
#define PPM_GENERATOR_H

#include <Arduino.h>

// RP2040 hardware timer includes
#ifdef ARDUINO_ARCH_RP2040
#include "hardware/timer.h"
#include "hardware/irq.h"
#endif

#define NUM_CHANNELS 8
#define PPM_FRAME_LENGTH 20000  // 20ms frame period in microseconds
#define PPM_PULSE_LENGTH 400    // 400us sync pulse
#define CHANNEL_MIN 1000
#define CHANNEL_MAX 2000
#define CHANNEL_CENTER 1500

// PPM state machine states
enum PPMState {
    PPM_SYNC_LOW,      // Sync pulse LOW period
    PPM_CHANNEL_HIGH,  // Channel pulse HIGH period
    PPM_CHANNEL_LOW    // Channel gap LOW period
};

class PPMGenerator {
private:
    uint8_t ppmPin;
    volatile uint16_t channels[NUM_CHANNELS];
    volatile uint8_t currentChannel;
    volatile PPMState state;
    volatile bool running;
    bool inverted;  // If true, invert PPM signal polarity
    
    // Static instance for interrupt handler
    static PPMGenerator* instance;
    
    // Timer callback function
    static int64_t timerCallback(alarm_id_t id, void* user_data);
    void handleTimerInterrupt();
    
public:
    PPMGenerator(uint8_t pin, bool invert = false);
    void begin();
    void setInverted(bool invert) { inverted = invert; }
    void setChannel(uint8_t channel, uint16_t value);
    void setChannels(uint16_t values[NUM_CHANNELS]);
    void stop();
    void start();
    uint16_t getChannel(uint8_t channel);
    bool isRunning() { return running; }
};

#endif // PPM_GENERATOR_H

