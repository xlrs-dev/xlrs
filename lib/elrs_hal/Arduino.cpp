#include "Arduino.h"
#include "ArduinoCompat.h"

#if defined(XLRS_PICO_SDK) || defined(PICO_BOARD)
#include <pico/time.h>

unsigned long millis() {
    return to_ms_since_boot(get_absolute_time());
}

#endif
