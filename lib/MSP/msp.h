#pragma once

#include "targets.h"

/**
 * Minimal MSP types for CRSF Router.
 * Source: ExpressLRS src/lib/MSP/msp.h (simplified)
 */

#define MSP_PORT_INBUF_SIZE 64

typedef struct
{
    uint8_t flags;
    uint16_t function;
    uint16_t payloadSize;
    uint8_t payload[MSP_PORT_INBUF_SIZE];
} mspPacket_t;
