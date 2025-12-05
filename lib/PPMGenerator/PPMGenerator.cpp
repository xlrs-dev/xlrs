#include "PPMGenerator.h"

// Static instance for interrupt handler
PPMGenerator* PPMGenerator::instance = nullptr;

PPMGenerator::PPMGenerator(uint8_t pin, bool invert) 
    : ppmPin(pin), currentChannel(0), state(PPM_SYNC_LOW), running(false), inverted(invert) {
    // Initialize channels to center position
    for (int i = 0; i < NUM_CHANNELS; i++) {
        channels[i] = CHANNEL_CENTER;
    }
    instance = this;
}

void PPMGenerator::begin() {
    pinMode(ppmPin, OUTPUT);
    digitalWrite(ppmPin, LOW);
    
    // Reset state machine
    currentChannel = 0;
    state = PPM_SYNC_LOW;
    running = false;
}

int64_t PPMGenerator::timerCallback(alarm_id_t id, void* user_data) {
    if (instance != nullptr) {
        instance->handleTimerInterrupt();
    }
    return 0;  // Return 0 means don't reschedule
}

void PPMGenerator::handleTimerInterrupt() {
    if (!running) {
        return;
    }
    
    int64_t nextDelay = 0;
    
    switch (state) {
        case PPM_SYNC_LOW:
            // Start of frame: sync pulse LOW period (400us)
            // Pin should already be LOW from previous frame end
            digitalWrite(ppmPin, inverted ? HIGH : LOW);
            nextDelay = PPM_PULSE_LENGTH;
            state = PPM_CHANNEL_HIGH;
            currentChannel = 0;
            break;
            
        case PPM_CHANNEL_HIGH:
            // Channel pulse HIGH period (or LOW if inverted)
            digitalWrite(ppmPin, inverted ? LOW : HIGH);
            nextDelay = channels[currentChannel];
            state = PPM_CHANNEL_LOW;
            break;
            
        case PPM_CHANNEL_LOW:
            // Channel gap LOW period (400us) (or HIGH if inverted)
            digitalWrite(ppmPin, inverted ? HIGH : LOW);
            nextDelay = PPM_PULSE_LENGTH;
            currentChannel++;
            
            if (currentChannel < NUM_CHANNELS) {
                // More channels to go
                state = PPM_CHANNEL_HIGH;
            } else {
                // All channels done, calculate frame end delay
                // Total time so far: sync pulse (400us) + sum of (channel + gap) for each channel
                // We're currently in the gap after the last channel, so we've used:
                // sync (400us) + sum(channel[i] + gap[i]) for i=0..NUM_CHANNELS-1 + current gap (400us)
                uint32_t totalTime = PPM_PULSE_LENGTH; // sync pulse
                for (int i = 0; i < NUM_CHANNELS; i++) {
                    totalTime += channels[i] + PPM_PULSE_LENGTH; // channel pulse + gap
                }
                
                if (totalTime < PPM_FRAME_LENGTH) {
                    nextDelay = PPM_FRAME_LENGTH - totalTime;
                } else {
                    // Frame already exceeded timing, start next frame immediately
                    nextDelay = 1;  // Minimum 1us delay
                }
                
                // Start next frame
                state = PPM_SYNC_LOW;
            }
            break;
    }
    
    // Schedule next interrupt
    if (nextDelay > 0) {
        add_alarm_in_us(nextDelay, timerCallback, nullptr, true);
    } else {
        // If delay is 0, schedule immediately (next microsecond)
        add_alarm_in_us(1, timerCallback, nullptr, true);
    }
}

void PPMGenerator::setChannel(uint8_t channel, uint16_t value) {
    if (channel < NUM_CHANNELS) {
        // Clamp value
        if (value < CHANNEL_MIN) value = CHANNEL_MIN;
        if (value > CHANNEL_MAX) value = CHANNEL_MAX;
        
        // Atomic update (channels array is volatile)
        channels[channel] = value;
    }
}

void PPMGenerator::setChannels(uint16_t values[NUM_CHANNELS]) {
    // Atomic update (channels array is volatile)
    for (int i = 0; i < NUM_CHANNELS; i++) {
        // Clamp values
        if (values[i] < CHANNEL_MIN) channels[i] = CHANNEL_MIN;
        else if (values[i] > CHANNEL_MAX) channels[i] = CHANNEL_MAX;
        else channels[i] = values[i];
    }
}

void PPMGenerator::start() {
    if (running) {
        return;  // Already running
    }
    
    running = true;
    state = PPM_SYNC_LOW;
    currentChannel = 0;
    digitalWrite(ppmPin, LOW);
    
    // Start the timer chain
    add_alarm_in_us(1, timerCallback, nullptr, true);
}

void PPMGenerator::stop() {
    running = false;
    digitalWrite(ppmPin, LOW);
    // Note: We can't easily cancel alarms, but the interrupt handler checks running flag
}

uint16_t PPMGenerator::getChannel(uint8_t channel) {
    if (channel < NUM_CHANNELS) {
        return channels[channel];
    }
    return CHANNEL_CENTER;
}

