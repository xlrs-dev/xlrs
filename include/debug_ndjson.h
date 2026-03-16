#pragma once

#include <Arduino.h>
#include <cstdio>
#include <cstring>

inline void debug_log_ndjson(const char *location, const char *message, const char *runId, const char *hypothesisId, const char *dataJson)
{
    FILE *f = fopen("/Users/ishan/dev/quoppo/dsi/rp2040-ppm-tx-rx/.cursor/debug-016ea4.log", "a");
    if (!f)
        return;
    fprintf(
        f,
        "{\"sessionId\":\"016ea4\",\"runId\":\"%s\",\"hypothesisId\":\"%s\",\"location\":\"%s\",\"message\":\"%s\",\"data\":%s,\"timestamp\":%lu}\n",
        runId,
        hypothesisId,
        location,
        message,
        dataJson,
        (unsigned long)millis());
    fclose(f);
}

inline void debug_bytes_hex(const uint8_t *data, size_t len, char *out, size_t outSize)
{
    if (outSize == 0)
        return;
    out[0] = '\0';
    const size_t maxBytes = len > 24 ? 24 : len;
    size_t pos = 0;
    for (size_t i = 0; i < maxBytes && pos + 4 < outSize; ++i)
    {
        int wrote = snprintf(out + pos, outSize - pos, "%02X%s", data[i], (i + 1 < maxBytes) ? " " : "");
        if (wrote <= 0)
            break;
        pos += (size_t)wrote;
    }
}
