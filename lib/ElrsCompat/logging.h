#pragma once

/**
 * Minimal ELRS logging.h compatibility for RP2040/Arduino.
 * Provides DBGLN/DBGVLN as no-ops (or Serial when DEBUG_CRSF is defined).
 * Source: ExpressLRS build-generated (stub)
 */

#include <Arduino.h>

#ifdef DEBUG_CRSF
#define DBGLN(...) do { Serial.printf(__VA_ARGS__); Serial.println(); } while(0)
#define DBGVLN(...) do { Serial.printf(__VA_ARGS__); Serial.println(); } while(0)
#else
#define DBGLN(...) ((void)0)
#define DBGVLN(...) ((void)0)
#endif
