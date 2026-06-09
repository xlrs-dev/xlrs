#pragma once

#include <stdint.h>

#include <string>
#include <vector>

namespace rc_handset {
namespace usb {

constexpr const char* kProtocolName = "rc.v1";

enum class CommandType : uint8_t {
    Hello,
    TxHello,
    GetConfig,
    SetConfig,
    Apply,
    Save,
    ResetDefaults,
    State,
    StreamState,
    CalStart,
    CalSample,
    CalFinish,
    BindingGet,
    BindingSet,
    BindingClear,
    BindingVerify,
    SpecialMode,
};

enum class ParseError : uint8_t {
    None,
    EmptyLine,
    BadProtocol,
    MissingCommand,
    UnknownCommand,
    BadArgument,
    BadPercentEncoding,
    MissingRequiredArgument,
};

struct Argument {
    std::string key;
    std::string value;
};

struct ParsedCommand {
    CommandType type = CommandType::Hello;
    bool hasSeq = false;
    uint32_t seq = 0;
    std::string field;
    std::string value;
    std::string target;
    std::string mode;
    std::vector<Argument> arguments;
};

struct ParseResult {
    bool ok = false;
    ParsedCommand command{};
    ParseError error = ParseError::None;
    std::string errorMessage;
};

struct ResponseField {
    std::string key;
    std::string value;
};

std::string percentEncode(const std::string& value);
bool percentDecode(const std::string& encoded, std::string& decoded);

ParseResult parseLine(const std::string& line);

std::string formatOk(const ParsedCommand& command,
                     const std::vector<ResponseField>& fields = {});
std::string formatOk(bool hasSeq, uint32_t seq,
                     const std::vector<ResponseField>& fields = {});
std::string formatErr(const ParsedCommand& command, const std::string& code,
                      const std::string& message);
std::string formatErr(bool hasSeq, uint32_t seq, const std::string& code,
                      const std::string& message);

const char* commandTypeName(CommandType type);
const char* parseErrorName(ParseError error);

} // namespace usb
} // namespace rc_handset
