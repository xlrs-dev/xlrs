#include "CRSFParser.h"

#include "CRSFRouter.h"
#include "../../include/debug_ndjson.h"

static uint8_t s_debug_parser_logs = 0;
static const uint8_t PARSER_LOG_LIMIT = 64;  // enough to see all errors during a discovery session

void CRSFParser::processBytes(CRSFConnector *origin, const uint8_t *inputBytes, const uint16_t size, const std::function<void(const crsf_header_t *)>& foundMessage)
{
    for (uint16_t i = 0; i < size; ++i)
    {
        processByte(origin, inputBytes[i], foundMessage);
    }
}

bool CRSFParser::processByte(CRSFConnector *origin, const uint8_t inputByte, const std::function<void(const crsf_header_t *)>& foundMessage)
{
    switch(telemetry_state) {
        case TELEMETRY_IDLE:
            // Telemetry from Betaflight/iNav starts with CRSF_SYNC_BYTE (CRSF_ADDRESS_FLIGHT_CONTROLLER).
            // Direct CRSF device-to-device traffic can also start with the addressed device id.
            if (inputByte == CRSF_SYNC_BYTE ||
                inputByte == CRSF_ADDRESS_RADIO_TRANSMITTER ||
                inputByte == CRSF_ADDRESS_CRSF_RECEIVER ||
                inputByte == CRSF_ADDRESS_CRSF_TRANSMITTER)
            {
                inBufferIndex = 0;
                telemetry_state = RECEIVING_LENGTH;
                CRSFinBuffer[0] = inputByte;
            }
            else
            {
                // #region agent log
                if (s_debug_parser_logs < PARSER_LOG_LIMIT)
                {
                    char dataJson[96];
                    snprintf(dataJson, sizeof(dataJson), "{\"reason\":\"unexpected_start\",\"byte\":%u}", inputByte);
                    debug_log_ndjson("CRSFParser.cpp:35", "parser_reject", "repro1", "H2", dataJson);
                    Serial.printf("[CRSF PARSE REJECT] bad_sync=0x%02X\n", inputByte);
                    s_debug_parser_logs++;
                }
                // #endregion
                return false;
            }
            break;

        case RECEIVING_LENGTH:
            if (inputByte < (CRSF_MIN_PACKET_LEN - CRSF_FRAME_NOT_COUNTED_BYTES) ||
                inputByte > (CRSF_MAX_PACKET_LEN - CRSF_FRAME_NOT_COUNTED_BYTES))
            {
                telemetry_state = TELEMETRY_IDLE;
                // #region agent log
                if (s_debug_parser_logs < PARSER_LOG_LIMIT)
                {
                    char dataJson[96];
                    snprintf(dataJson, sizeof(dataJson), "{\"reason\":\"invalid_length\",\"len\":%u}", inputByte);
                    debug_log_ndjson("CRSFParser.cpp:50", "parser_reject", "repro1", "H2", dataJson);
                    Serial.printf("[CRSF PARSE REJECT] invalid_len=%u\n", inputByte);
                    s_debug_parser_logs++;
                }
                // #endregion
                return false;
            }
            telemetry_state = RECEIVING_DATA;
            CRSFinBuffer[CRSF_TELEMETRY_LENGTH_INDEX] = inputByte;
            break;

        case RECEIVING_DATA:
            CRSFinBuffer[inBufferIndex + CRSF_FRAME_NOT_COUNTED_BYTES] = inputByte;
            inBufferIndex++;
            if (CRSFinBuffer[CRSF_TELEMETRY_LENGTH_INDEX] == inBufferIndex)
            {
                // exclude first bytes (sync byte + length), skip last byte (submitted crc)
                const uint8_t crc = crsfRouter.crsf_crc.calc(CRSFinBuffer + CRSF_FRAME_NOT_COUNTED_BYTES, CRSFinBuffer[CRSF_TELEMETRY_LENGTH_INDEX] - CRSF_TELEMETRY_CRC_LENGTH);
                telemetry_state = TELEMETRY_IDLE;

                if (inputByte == crc)
                {
                    const crsf_header_t *header = (crsf_header_t *) CRSFinBuffer;
                    crsfRouter.processMessage(origin, header);
                    // We have found a packet
                    if (foundMessage) foundMessage(header);
                    return true;
                }
                // #region agent log
                if (s_debug_parser_logs < PARSER_LOG_LIMIT)
                {
                    char dataJson[128];
                    snprintf(dataJson, sizeof(dataJson), "{\"reason\":\"crc_mismatch\",\"expected\":%u,\"actual\":%u,\"type\":%u}", crc, inputByte, CRSFinBuffer[CRSF_TELEMETRY_TYPE_INDEX]);
                    debug_log_ndjson("CRSFParser.cpp:79", "parser_reject", "repro1", "H2", dataJson);
                    Serial.printf("[CRSF PARSE REJECT] crc_mismatch type=0x%02X expected=0x%02X got=0x%02X\n",
                                  CRSFinBuffer[CRSF_TELEMETRY_TYPE_INDEX], crc, inputByte);
                    s_debug_parser_logs++;
                }
                // #endregion
                return false;
            }
            break;
    }

    return true;
}