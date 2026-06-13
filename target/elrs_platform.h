#pragma once

#include "xlrs_rp2040.h"

#ifndef ICACHE_RAM_ATTR
#define ICACHE_RAM_ATTR
#endif

#ifndef WORD_ALIGNED_ATTR
#define WORD_ALIGNED_ATTR __attribute__((aligned(4)))
#endif

#ifndef WORD_PADDED
#define WORD_PADDED(size) (((size) + 3) & ~3)
#endif

#ifndef LATEST_VERSION
#define LATEST_VERSION "3.5.6"
#endif

#ifndef LATEST_COMMIT
#define LATEST_COMMIT "ee188b4e"
#endif

#ifndef FLASH_DISCRIM
#define FLASH_DISCRIM 0
#endif
