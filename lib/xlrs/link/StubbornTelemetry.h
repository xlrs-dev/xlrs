#pragma once
#include <stdint.h>
#include <string.h>

namespace xlrs {

// A single telemetry chunk sent over the air inside an OTA16 frame.
// Frame format: [header:1][seq:1][length:1][data:13] = 16 bytes.
struct TelemetryChunk {
    uint8_t seq;        // sequence number
    uint8_t length;     // data length in this chunk (0..13)
    uint8_t data[13];   // fragment payload
};

class StubbornSender {
public:
    StubbornSender() = default;

    bool queuePayload(const uint8_t* payload, size_t len) {
        if (!idle()) return false;
        if (len > sizeof(_buf)) {
            len = sizeof(_buf); // truncate to buffer limit
        }
        memcpy(_buf, payload, len);
        _len = len;
        _offset = 0;
        return true;
    }

    bool getNextChunk(TelemetryChunk& out) {
        if (_offset >= _len) {
            return false; // nothing to send
        }
        size_t remaining = _len - _offset;
        uint8_t chunkSize = remaining > 13 ? 13 : (uint8_t)remaining;

        out.seq = _currentSeq;
        out.length = chunkSize;
        if (_offset + chunkSize >= _len) {
            out.length |= 0x80; // Set EOP (End of Payload) flag
        }
        memcpy(out.data, _buf + _offset, chunkSize);
        return true;
    }

    void receiveAck(uint8_t ackSeq) {
        if (ackSeq == 0 && _currentSeq != 0 && _offset < _len) {
            _offset = 0;
            _currentSeq = 0;
            _staleAckCount = 0;
            _lastStaleAck = 0;
            return;
        }

        uint8_t expectedNext = (uint8_t)(_currentSeq + 1);
        if (ackSeq == expectedNext && _offset < _len) {
            _staleAckCount = 0;
            size_t remaining = _len - _offset;
            size_t chunkSize = remaining > 13 ? 13 : remaining;
            _offset += chunkSize;
            _currentSeq = expectedNext;

            if (_offset >= _len) {
                _len = 0;
                _offset = 0;
            }
        } else if (_offset < _len && ackSeq != _currentSeq) {
            if (ackSeq != _lastStaleAck) {
                _lastStaleAck = ackSeq;
                _staleAckCount = 0;
            }
            if (++_staleAckCount >= 3) {
                const size_t rewindOffset = (size_t)ackSeq * 13u;
                if (ackSeq < _currentSeq && rewindOffset < _len) {
                    _offset = rewindOffset;
                    _currentSeq = ackSeq;
                } else {
                    _len = 0;
                    _offset = 0;
                    _currentSeq = ackSeq;
                }
                _staleAckCount = 0;
            }
        }
    }

    bool idle() const { return _len == 0; }

private:
    uint8_t _buf[128] = {0};
    size_t  _len = 0;
    size_t  _offset = 0;
    uint8_t _currentSeq = 0;
    uint8_t _lastStaleAck = 0;
    uint8_t _staleAckCount = 0;
};

class StubbornReceiver {
public:
    StubbornReceiver() = default;

    bool processChunk(const TelemetryChunk& in, uint8_t& outAckSeq) {
        uint8_t realLen = in.length & 0x7F;
        bool endOfPayload = (in.length & 0x80) != 0;

        if (in.seq == _expectedSeq) {
            size_t space = sizeof(_buf) - _len;
            size_t copyLen = realLen > space ? space : realLen;
            if (copyLen > 0) {
                memcpy(_buf + _len, in.data, copyLen);
                _len += copyLen;
            }
            _expectedSeq = (uint8_t)(_expectedSeq + 1);
            outAckSeq = _expectedSeq;

            if (endOfPayload) {
                _ready = true;
            }
            return true;
        } else {
            outAckSeq = _expectedSeq;
            return false;
        }
    }

    bool getPayload(uint8_t* outBuf, size_t& outLen) {
        if (!_ready) return false;
        memcpy(outBuf, _buf, _len);
        outLen = _len;
        
        _len = 0;
        _ready = false;
        return true;
    }

    bool ready() const { return _ready; }

private:
    uint8_t _buf[128] = {0};
    size_t  _len = 0;
    uint8_t _expectedSeq = 0;
    bool    _ready = false;
};

} // namespace xlrs
