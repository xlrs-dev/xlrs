#pragma once

#include <stdio.h>

#ifndef DBGLN
#define DBGLN(...) printf(__VA_ARGS__); printf("\n")
#endif
