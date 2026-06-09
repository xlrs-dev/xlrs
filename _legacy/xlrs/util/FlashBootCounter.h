// FlashBootCounter — wear-leveled monotonic reboot counter persisted in flash (M8).
#pragma once
#include <stdint.h>

namespace xlrs {

class FlashBootCounter {
public:
    // Read the current boot counter from flash without modifying/incrementing it.
    static uint32_t read();

    // Increment the boot counter in flash, and return the new value.
    static uint32_t increment();

    // Host-native only: reset the simulated memory-based boot counter for test isolation.
    static void resetSim();
};

} // namespace xlrs
