#pragma once

/**
 * Minimal ELRS targets.h compatibility for RP2040/Arduino.
 * Provides only the symbols needed by lib/CrsfProtocol.
 * Source: ExpressLRS src/include/targets.h (simplified)
 */

#include "elrs_globals.h"

#define OTA_VERSION_ID 4

#undef ICACHE_RAM_ATTR
#define ICACHE_RAM_ATTR

#ifndef bit
#define bit(b) (1UL << (b))
#endif
