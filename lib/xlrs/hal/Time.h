#pragma once

#include <stdint.h>

#if defined(XLRS_PICO_SDK)
#include <pico/time.h>

namespace xlrs::hal {
inline uint32_t nowUs() { return (uint32_t)time_us_64(); }
inline uint32_t nowMs() { return to_ms_since_boot(get_absolute_time()); }
inline void sleepMs(uint32_t ms) { sleep_ms(ms); }
inline void sleepUs(uint32_t us) { sleep_us(us); }
} // namespace xlrs::hal

#else
namespace xlrs::hal {
uint32_t nowUs();
uint32_t nowMs();
void sleepMs(uint32_t ms);
void sleepUs(uint32_t us);
} // namespace xlrs::hal
#endif
