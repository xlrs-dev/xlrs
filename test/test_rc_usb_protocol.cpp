#include <unity.h>

#include "rc_handset/usb/protocol.h"

using namespace rc_handset::usb;

static void test_hello_parses_and_echoes_seq() {
    ParseResult result = parseLine("rc.v1 hello seq=42\n");
    TEST_ASSERT_TRUE(result.ok);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandType::Hello),
                            static_cast<uint8_t>(result.command.type));
    TEST_ASSERT_TRUE(result.command.hasSeq);
    TEST_ASSERT_EQUAL_UINT32(42, result.command.seq);
    TEST_ASSERT_EQUAL_STRING("rc.v1 ok seq=42 version=usb%20rc.v1\n",
                             formatOk(result.command, {{"version", "usb rc.v1"}}).c_str());
}

static void test_config_field_parsing_decodes_value() {
    ParseResult result = parseLine("rc.v1 set_config seq=7 field=binding_phrase value=Bench%20Rig");
    TEST_ASSERT_TRUE(result.ok);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandType::SetConfig),
                            static_cast<uint8_t>(result.command.type));
    TEST_ASSERT_EQUAL_STRING("binding_phrase", result.command.field.c_str());
    TEST_ASSERT_EQUAL_STRING("Bench Rig", result.command.value.c_str());
    TEST_ASSERT_EQUAL_UINT32(7, result.command.seq);
}

static void test_get_config_accepts_field_selector() {
    ParseResult result = parseLine("rc.v1 get_config field=rate");
    TEST_ASSERT_TRUE(result.ok);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandType::GetConfig),
                            static_cast<uint8_t>(result.command.type));
    TEST_ASSERT_EQUAL_STRING("rate", result.command.field.c_str());
}

static void test_calibration_commands_parse() {
    ParseResult start = parseLine("rc.v1 cal_start seq=1 field=aileron");
    ParseResult sample = parseLine("rc.v1 cal_sample seq=2 field=aileron value=2048");
    ParseResult finish = parseLine("rc.v1 cal_finish seq=3 field=aileron");
    TEST_ASSERT_TRUE(start.ok);
    TEST_ASSERT_TRUE(sample.ok);
    TEST_ASSERT_TRUE(finish.ok);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandType::CalStart),
                            static_cast<uint8_t>(start.command.type));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandType::CalSample),
                            static_cast<uint8_t>(sample.command.type));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandType::CalFinish),
                            static_cast<uint8_t>(finish.command.type));
    TEST_ASSERT_EQUAL_STRING("2048", sample.command.value.c_str());
}

static void test_binding_commands_parse() {
    ParseResult get = parseLine("rc.v1 binding_get");
    ParseResult set = parseLine("rc.v1 binding_set target=tx phrase=Field%20Radio");
    ParseResult clear = parseLine("rc.v1 binding_clear");
    ParseResult verify = parseLine("rc.v1 binding_verify value=Field%20Radio");
    TEST_ASSERT_TRUE(get.ok);
    TEST_ASSERT_TRUE(set.ok);
    TEST_ASSERT_TRUE(clear.ok);
    TEST_ASSERT_TRUE(verify.ok);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandType::BindingGet),
                            static_cast<uint8_t>(get.command.type));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandType::BindingSet),
                            static_cast<uint8_t>(set.command.type));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandType::BindingClear),
                            static_cast<uint8_t>(clear.command.type));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandType::BindingVerify),
                            static_cast<uint8_t>(verify.command.type));
    TEST_ASSERT_EQUAL_STRING("Field Radio", set.command.value.c_str());
    TEST_ASSERT_EQUAL_STRING("tx", set.command.target.c_str());
}

static void test_webui_utility_commands_parse() {
    ParseResult txHello = parseLine("rc.v1 tx_hello seq=3");
    ParseResult apply = parseLine("rc.v1 apply");
    ParseResult save = parseLine("rc.v1 save");
    ParseResult defaults = parseLine("rc.v1 reset_defaults target=rc_config");
    TEST_ASSERT_TRUE(txHello.ok);
    TEST_ASSERT_TRUE(apply.ok);
    TEST_ASSERT_TRUE(save.ok);
    TEST_ASSERT_TRUE(defaults.ok);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandType::TxHello),
                            static_cast<uint8_t>(txHello.command.type));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandType::Apply),
                            static_cast<uint8_t>(apply.command.type));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandType::Save),
                            static_cast<uint8_t>(save.command.type));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandType::ResetDefaults),
                            static_cast<uint8_t>(defaults.command.type));
    TEST_ASSERT_EQUAL_STRING("rc_config", defaults.command.target.c_str());
}

static void test_state_stream_and_special_mode_parse() {
    ParseResult state = parseLine("rc.v1 state seq=9");
    ParseResult stream = parseLine("rc.v1 stream_state value=on");
    ParseResult special = parseLine("rc.v1 special_mode mode=bootloader");
    TEST_ASSERT_TRUE(state.ok);
    TEST_ASSERT_TRUE(stream.ok);
    TEST_ASSERT_TRUE(special.ok);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandType::State),
                            static_cast<uint8_t>(state.command.type));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandType::StreamState),
                            static_cast<uint8_t>(stream.command.type));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandType::SpecialMode),
                            static_cast<uint8_t>(special.command.type));
    TEST_ASSERT_EQUAL_STRING("bootloader", special.command.mode.c_str());
}

static void test_error_handling_for_malformed_lines() {
    ParseResult badProtocol = parseLine("rc.v2 hello");
    ParseResult badArgument = parseLine("rc.v1 hello seq");
    ParseResult badPercent = parseLine("rc.v1 set_config field=name value=Bad%XZ");
    ParseResult missing = parseLine("rc.v1 set_config field=name");
    TEST_ASSERT_FALSE(badProtocol.ok);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ParseError::BadProtocol),
                            static_cast<uint8_t>(badProtocol.error));
    TEST_ASSERT_FALSE(badArgument.ok);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ParseError::BadArgument),
                            static_cast<uint8_t>(badArgument.error));
    TEST_ASSERT_FALSE(badPercent.ok);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ParseError::BadPercentEncoding),
                            static_cast<uint8_t>(badPercent.error));
    TEST_ASSERT_FALSE(missing.ok);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ParseError::MissingRequiredArgument),
                            static_cast<uint8_t>(missing.error));
}

static void test_unknown_command_is_reported() {
    ParseResult result = parseLine("rc.v1 reboot");
    TEST_ASSERT_FALSE(result.ok);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ParseError::UnknownCommand),
                            static_cast<uint8_t>(result.error));
}

static void test_percent_encoding_round_trip() {
    const std::string encoded = percentEncode("space and %= symbols");
    TEST_ASSERT_EQUAL_STRING("space%20and%20%25%3D%20symbols", encoded.c_str());
    std::string decoded;
    TEST_ASSERT_TRUE(percentDecode(encoded, decoded));
    TEST_ASSERT_EQUAL_STRING("space and %= symbols", decoded.c_str());
}

static void test_error_response_encodes_message_and_echoes_seq() {
    TEST_ASSERT_EQUAL_STRING("rc.v1 err seq=77 code=bad_request message=needs%20field%3Dvalue\n",
                             formatErr(true, 77, "bad_request", "needs field=value").c_str());
}

void run_rc_usb_protocol_tests() {
    RUN_TEST(test_hello_parses_and_echoes_seq);
    RUN_TEST(test_config_field_parsing_decodes_value);
    RUN_TEST(test_get_config_accepts_field_selector);
    RUN_TEST(test_calibration_commands_parse);
    RUN_TEST(test_binding_commands_parse);
    RUN_TEST(test_webui_utility_commands_parse);
    RUN_TEST(test_state_stream_and_special_mode_parse);
    RUN_TEST(test_error_handling_for_malformed_lines);
    RUN_TEST(test_unknown_command_is_reported);
    RUN_TEST(test_percent_encoding_round_trip);
    RUN_TEST(test_error_response_encodes_message_and_echoes_seq);
}
