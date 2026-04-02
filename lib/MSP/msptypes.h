#pragma once

/**
 * MSP type definitions for CRSF encapsulation.
 * Source: ExpressLRS src/lib/MSP/msptypes.h
 */

#define MSP_ELRS_FUNC 0x4578

// CRSF encapsulated MSP defines - sized for CRSF frame (max payload 62, minus header)
#define ENCAPSULATED_MSP_HEADER_CRC_LEN 4
#define ENCAPSULATED_MSP_MAX_PAYLOAD_SIZE 58
#define ENCAPSULATED_MSP_MAX_FRAME_LEN (ENCAPSULATED_MSP_HEADER_CRC_LEN + ENCAPSULATED_MSP_MAX_PAYLOAD_SIZE)
