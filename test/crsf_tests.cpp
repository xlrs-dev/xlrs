#include <unity.h>

#include <deque>
#include <vector>
#include <cstring>

#include "app/CrsfChannels.h"
#include "crc8/crc8.h"
#include "crsfSerial.h"
#include "hal/SerialPort.h"
#include "hal/Time.h"

namespace xlrs::hal {
static std::deque<uint8_t> s_serialIn;
static std::vector<uint8_t> s_serialOut;
static uint32_t s_nowMs = 0;
static uint32_t s_lastBaud = 0;

void resetHostSerial() {
    s_serialIn.clear();
    s_serialOut.clear();
    s_nowMs = 0;
    s_lastBaud = 0;
}

void pushHostSerial(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        s_serialIn.push_back(data[i]);
    }
}

const std::vector<uint8_t>& hostSerialOut() {
    return s_serialOut;
}

void advanceHostTimeMs(uint32_t ms) {
    s_nowMs += ms;
}

uint32_t lastHostBaud() {
    return s_lastBaud;
}

void SerialPort::begin(uint32_t baud) { s_lastBaud = baud; }
void SerialPort::end() {}
int SerialPort::available() const { return s_serialIn.empty() ? 0 : 1; }
int SerialPort::read() {
    if (s_serialIn.empty()) return -1;
    uint8_t b = s_serialIn.front();
    s_serialIn.pop_front();
    return b;
}
size_t SerialPort::write(uint8_t b) {
    s_serialOut.push_back(b);
    return 1;
}
size_t SerialPort::write(const uint8_t* data, size_t len) {
    if (!data) return 0;
    for (size_t i = 0; i < len; ++i) {
        s_serialOut.push_back(data[i]);
    }
    return len;
}

uint32_t nowUs() { return s_nowMs * 1000; }
uint32_t nowMs() { return s_nowMs; }
void sleepMs(uint32_t ms) { s_nowMs += ms; }
void sleepUs(uint32_t us) { s_nowMs += (us + 999) / 1000; }
} // namespace xlrs::hal

using namespace xlrs;

static uint8_t calcCrsfCrc(const uint8_t* data, uint8_t len) {
    Crc8 crc(0xD5);
    uint8_t tmp[CRSF_MAX_PACKET_SIZE] = {};
    memcpy(tmp, data, len);
    return crc.calc(tmp, len);
}

static std::vector<uint8_t> makeCrsfFrame(uint8_t addr, uint8_t type,
                                          const uint8_t* payload = nullptr,
                                          uint8_t payloadLen = 0) {
    std::vector<uint8_t> frame;
    frame.push_back(addr);
    frame.push_back(payloadLen + 2);
    frame.push_back(type);
    for (uint8_t i = 0; i < payloadLen; ++i) {
        frame.push_back(payload[i]);
    }
    frame.push_back(calcCrsfCrc(&frame[2], payloadLen + 1));
    return frame;
}

static std::vector<uint8_t> makeCrsfExtendedFrame(uint8_t addr, uint8_t type,
                                                  uint8_t destination, uint8_t origin,
                                                  const uint8_t* payload = nullptr,
                                                  uint8_t payloadLen = 0) {
    std::vector<uint8_t> frame;
    frame.push_back(addr);
    frame.push_back(payloadLen + CRSF_FRAME_LENGTH_EXT_TYPE_CRC);
    frame.push_back(type);
    frame.push_back(destination);
    frame.push_back(origin);
    for (uint8_t i = 0; i < payloadLen; ++i) {
        frame.push_back(payload[i]);
    }
    frame.push_back(calcCrsfCrc(&frame[2], payloadLen + 3));
    return frame;
}

static int s_channelsCallbacks = 0;
static int s_linkUpCallbacks = 0;
static int s_linkDownCallbacks = 0;
static int s_oobCallbacks = 0;
static int s_rawCallbacks = 0;
static int s_devicePingCallbacks = 0;
static int s_parameterReadCallbacks = 0;
static int s_parameterWriteCallbacks = 0;
static int s_parameterEntryCallbacks = 0;
static int s_deviceInfoCallbacks = 0;
static uint8_t s_destination = 0;
static uint8_t s_origin = 0;
static uint8_t s_parameterNumber = 0;
static uint8_t s_chunkNumber = 0;
static uint8_t s_value[16] = {};
static uint8_t s_valueLen = 0;
static char s_name[24] = {};
static uint8_t s_sourceAddr = 0;
static uint8_t s_paramType = 0;
static uint8_t s_chunksRemaining = 0;
static char s_label[24] = {};

static void resetCallbacks() {
    s_channelsCallbacks = 0;
    s_linkUpCallbacks = 0;
    s_linkDownCallbacks = 0;
    s_oobCallbacks = 0;
    s_rawCallbacks = 0;
    s_devicePingCallbacks = 0;
    s_parameterReadCallbacks = 0;
    s_parameterWriteCallbacks = 0;
    s_parameterEntryCallbacks = 0;
    s_deviceInfoCallbacks = 0;
    s_destination = 0;
    s_origin = 0;
    s_parameterNumber = 0;
    s_chunkNumber = 0;
    memset(s_value, 0, sizeof(s_value));
    s_valueLen = 0;
    memset(s_name, 0, sizeof(s_name));
    s_sourceAddr = 0;
    s_paramType = 0;
    s_chunksRemaining = 0;
    memset(s_label, 0, sizeof(s_label));
}

static CrsfSerial makeCrsf(xlrs::hal::SerialPort& port) {
    CrsfSerial crsf(port, CRSF_BAUDRATE);
    crsf.onPacketChannels = []() { ++s_channelsCallbacks; };
    crsf.onLinkUp = []() { ++s_linkUpCallbacks; };
    crsf.onLinkDown = []() { ++s_linkDownCallbacks; };
    crsf.onOobData = [](uint8_t) { ++s_oobCallbacks; };
    crsf.onPacketRaw = [](const uint8_t*, uint8_t) { ++s_rawCallbacks; };
    crsf.onDevicePing = [](uint8_t destination, uint8_t origin) {
        ++s_devicePingCallbacks;
        s_destination = destination;
        s_origin = origin;
    };
    crsf.onParameterRead = [](uint8_t parameterNumber, uint8_t chunkNumber,
                              uint8_t destination, uint8_t origin) {
        ++s_parameterReadCallbacks;
        s_parameterNumber = parameterNumber;
        s_chunkNumber = chunkNumber;
        s_destination = destination;
        s_origin = origin;
    };
    crsf.onParameterWrite = [](uint8_t parameterNumber, const uint8_t* value, uint8_t valueLen,
                               uint8_t destination, uint8_t origin) {
        ++s_parameterWriteCallbacks;
        s_parameterNumber = parameterNumber;
        s_valueLen = valueLen;
        memcpy(s_value, value, valueLen > sizeof(s_value) ? sizeof(s_value) : valueLen);
        s_destination = destination;
        s_origin = origin;
    };
    crsf.onParameterEntry = [](uint8_t fieldId, uint8_t paramType, uint8_t chunksRemaining,
                               const char* label, const uint8_t* value, uint8_t valueLen) {
        ++s_parameterEntryCallbacks;
        s_parameterNumber = fieldId;
        s_paramType = paramType;
        s_chunksRemaining = chunksRemaining;
        strncpy(s_label, label, sizeof(s_label) - 1);
        s_valueLen = valueLen;
        memcpy(s_value, value, valueLen > sizeof(s_value) ? sizeof(s_value) : valueLen);
    };
    crsf.onDeviceInfo = [](const uint8_t*, const char* name, uint8_t sourceAddr) {
        ++s_deviceInfoCallbacks;
        strncpy(s_name, name, sizeof(s_name) - 1);
        s_sourceAddr = sourceAddr;
    };
    return crsf;
}

static void feed(CrsfSerial& crsf, const std::vector<uint8_t>& frame) {
    xlrs::hal::pushHostSerial(frame.data(), frame.size());
    crsf.loop();
}

static void test_crsf_begin_sets_baud() {
    xlrs::hal::SerialPort port;
    CrsfSerial crsf(port, CRSF_BAUDRATE);
    crsf.begin();
    TEST_ASSERT_EQUAL_UINT32(CRSF_BAUDRATE, xlrs::hal::lastHostBaud());
}

static void test_crsf_rc_channels_frame_sets_link_up() {
    xlrs::hal::SerialPort port;
    CrsfSerial crsf = makeCrsf(port);

    crsf_channels_t channels{};
    for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        setCrsfChannelByIndex(channels, i, rcUsToCrsfChannel((uint16_t)(1000 + i * 25)));
    }
    auto frame = makeCrsfFrame(CRSF_ADDRESS_FLIGHT_CONTROLLER,
                               CRSF_FRAMETYPE_RC_CHANNELS_PACKED,
                               reinterpret_cast<const uint8_t*>(&channels),
                               CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE);
    feed(crsf, frame);

    TEST_ASSERT_EQUAL_INT(1, s_channelsCallbacks);
    TEST_ASSERT_EQUAL_INT(1, s_linkUpCallbacks);
    TEST_ASSERT_TRUE(crsf.isLinkUp());
    TEST_ASSERT_EQUAL_INT(1000, crsf.getChannel(1));
    TEST_ASSERT_EQUAL_INT(1175, crsf.getChannel(8));
    TEST_ASSERT_EQUAL_INT(1375, crsf.getChannel(16));

    xlrs::hal::advanceHostTimeMs(CrsfSerial::CRSF_FAILSAFE_STAGE1_MS + 1);
    crsf.loop();
    TEST_ASSERT_EQUAL_INT(1, s_linkDownCallbacks);
    TEST_ASSERT_FALSE(crsf.isLinkUp());
}

static void test_crsf_parser_discards_noise_and_bad_crc() {
    xlrs::hal::SerialPort port;
    CrsfSerial crsf = makeCrsf(port);

    uint8_t badAddr[] = {0x41, 0x42, 0x43};
    xlrs::hal::pushHostSerial(badAddr, sizeof(badAddr));
    crsf.loop();
    TEST_ASSERT_EQUAL_INT(0, s_oobCallbacks);
    TEST_ASSERT_EQUAL_INT(0, s_rawCallbacks);

    uint8_t payload[2] = {7, 0};
    auto badFrame = makeCrsfExtendedFrame(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_PARAMETER_READ,
                                          CRSF_ADDRESS_CRSF_TRANSMITTER,
                                          CRSF_ADDRESS_RADIO_TRANSMITTER,
                                          payload, sizeof(payload));
    badFrame.back() ^= 0x55;
    feed(crsf, badFrame);
    TEST_ASSERT_EQUAL_INT(0, s_parameterReadCallbacks);
    TEST_ASSERT_EQUAL_INT(0, s_rawCallbacks);
}

static void test_crsf_packet_timeout_flushes_partial_frame() {
    xlrs::hal::SerialPort port;
    CrsfSerial crsf = makeCrsf(port);

    uint8_t partial[] = {CRSF_SYNC_BYTE, 5, CRSF_FRAMETYPE_LINK_STATISTICS};
    xlrs::hal::pushHostSerial(partial, sizeof(partial));
    crsf.loop();
    TEST_ASSERT_EQUAL_INT(0, s_oobCallbacks);

    xlrs::hal::advanceHostTimeMs(CrsfSerial::CRSF_PACKET_TIMEOUT_MS + 1);
    crsf.loop();
    TEST_ASSERT_EQUAL_INT(3, s_oobCallbacks);
}

static void test_crsf_extended_device_ping_and_parameters() {
    xlrs::hal::SerialPort port;
    CrsfSerial crsf = makeCrsf(port);

    auto ping = makeCrsfExtendedFrame(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_DEVICE_PING,
                                      CRSF_ADDRESS_CRSF_TRANSMITTER,
                                      CRSF_ADDRESS_RADIO_TRANSMITTER);
    feed(crsf, ping);
    TEST_ASSERT_EQUAL_INT(1, s_devicePingCallbacks);
    TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_CRSF_TRANSMITTER, s_destination);
    TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_RADIO_TRANSMITTER, s_origin);

    uint8_t readPayload[] = {6, 0};
    auto read = makeCrsfExtendedFrame(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_PARAMETER_READ,
                                      CRSF_ADDRESS_CRSF_TRANSMITTER,
                                      CRSF_ADDRESS_RADIO_TRANSMITTER,
                                      readPayload, sizeof(readPayload));
    feed(crsf, read);
    TEST_ASSERT_EQUAL_INT(1, s_parameterReadCallbacks);
    TEST_ASSERT_EQUAL_UINT8(6, s_parameterNumber);
    TEST_ASSERT_EQUAL_UINT8(0, s_chunkNumber);

    uint8_t writePayload[] = {6, CRSF_COMMAND_START};
    auto write = makeCrsfExtendedFrame(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_PARAMETER_WRITE,
                                       CRSF_ADDRESS_CRSF_TRANSMITTER,
                                       CRSF_ADDRESS_RADIO_TRANSMITTER,
                                       writePayload, sizeof(writePayload));
    feed(crsf, write);
    TEST_ASSERT_EQUAL_INT(1, s_parameterWriteCallbacks);
    TEST_ASSERT_EQUAL_UINT8(6, s_parameterNumber);
    TEST_ASSERT_EQUAL_UINT8(1, s_valueLen);
    TEST_ASSERT_EQUAL_UINT8(CRSF_COMMAND_START, s_value[0]);
}

static void test_crsf_device_info_and_parameter_entry_parsing() {
    xlrs::hal::SerialPort port;
    CrsfSerial crsf = makeCrsf(port);

    uint8_t infoPayload[32] = {};
    uint8_t infoLen = 0;
    const char* name = "XLRS TX";
    memcpy(&infoPayload[infoLen], name, strlen(name) + 1);
    infoLen += strlen(name) + 1;
    const uint8_t fixedInfo[] = {
        0, 0, 0, 0, // serial
        0, 0, 0, 1, // hw
        0, 0, 0, 1, // fw
        7, 1        // parameter count/version
    };
    memcpy(&infoPayload[infoLen], fixedInfo, sizeof(fixedInfo));
    infoLen += sizeof(fixedInfo);
    auto info = makeCrsfExtendedFrame(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_DEVICE_INFO,
                                      CRSF_ADDRESS_RADIO_TRANSMITTER,
                                      CRSF_ADDRESS_CRSF_TRANSMITTER,
                                      infoPayload, infoLen);
    feed(crsf, info);
    TEST_ASSERT_EQUAL_INT(1, s_deviceInfoCallbacks);
    TEST_ASSERT_EQUAL_STRING("XLRS TX", s_name);
    TEST_ASSERT_EQUAL_UINT8(CRSF_SYNC_BYTE, s_sourceAddr);

    uint8_t entryPayload[40] = {};
    uint8_t entryLen = 0;
    entryPayload[entryLen++] = 6;
    entryPayload[entryLen++] = 0;
    entryPayload[entryLen++] = 0;
    entryPayload[entryLen++] = CRSF_PARAM_COMMAND;
    const char* label = "Bind RX";
    memcpy(&entryPayload[entryLen], label, strlen(label) + 1);
    entryLen += strlen(label) + 1;
    entryPayload[entryLen++] = CRSF_COMMAND_PROGRESS;
    entryPayload[entryLen++] = 10;
    auto entry = makeCrsfExtendedFrame(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY,
                                       CRSF_ADDRESS_RADIO_TRANSMITTER,
                                       CRSF_ADDRESS_CRSF_TRANSMITTER,
                                       entryPayload, entryLen);
    feed(crsf, entry);
    TEST_ASSERT_EQUAL_INT(1, s_parameterEntryCallbacks);
    TEST_ASSERT_EQUAL_UINT8(6, s_parameterNumber);
    TEST_ASSERT_EQUAL_UINT8(CRSF_PARAM_COMMAND, s_paramType);
    TEST_ASSERT_EQUAL_UINT8(0, s_chunksRemaining);
    TEST_ASSERT_EQUAL_STRING("Bind RX", s_label);
    TEST_ASSERT_EQUAL_UINT8(CRSF_COMMAND_PROGRESS, s_value[0]);
}

static void test_crsf_queue_packet_and_extended_packet() {
    xlrs::hal::SerialPort port;
    CrsfSerial crsf(port, CRSF_BAUDRATE);

    uint8_t stats[CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE] = {1,2,3,4,5,6,7,8,9,10};
    crsf.queuePacket(CRSF_ADDRESS_RADIO_TRANSMITTER,
                     CRSF_FRAMETYPE_LINK_STATISTICS,
                     stats, sizeof(stats));
    const auto& out = xlrs::hal::hostSerialOut();
    TEST_ASSERT_EQUAL_UINT8(14, out.size());
    TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_RADIO_TRANSMITTER, out[0]);
    TEST_ASSERT_EQUAL_UINT8(sizeof(stats) + 2, out[1]);
    TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_LINK_STATISTICS, out[2]);
    TEST_ASSERT_EQUAL_UINT8(calcCrsfCrc(&out[2], out[1] - 1), out.back());

    xlrs::hal::resetHostSerial();
    uint8_t payload[] = {6, 0};
    crsf.queueExtendedPacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY,
                             CRSF_ADDRESS_RADIO_TRANSMITTER,
                             CRSF_ADDRESS_CRSF_TRANSMITTER,
                             payload, sizeof(payload));
    const auto& ext = xlrs::hal::hostSerialOut();
    TEST_ASSERT_EQUAL_UINT8(8, ext.size());
    TEST_ASSERT_EQUAL_UINT8(CRSF_SYNC_BYTE, ext[0]);
    TEST_ASSERT_EQUAL_UINT8(sizeof(payload) + CRSF_FRAME_LENGTH_EXT_TYPE_CRC, ext[1]);
    TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY, ext[2]);
    TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_RADIO_TRANSMITTER, ext[3]);
    TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_CRSF_TRANSMITTER, ext[4]);
    TEST_ASSERT_EQUAL_UINT8(calcCrsfCrc(&ext[2], ext[1] - 1), ext.back());
}

void setUp() {
    xlrs::hal::resetHostSerial();
    resetCallbacks();
}

void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_crsf_begin_sets_baud);
    RUN_TEST(test_crsf_rc_channels_frame_sets_link_up);
    RUN_TEST(test_crsf_parser_discards_noise_and_bad_crc);
    RUN_TEST(test_crsf_packet_timeout_flushes_partial_frame);
    RUN_TEST(test_crsf_extended_device_ping_and_parameters);
    RUN_TEST(test_crsf_device_info_and_parameter_entry_parsing);
    RUN_TEST(test_crsf_queue_packet_and_extended_packet);
    return UNITY_END();
}
