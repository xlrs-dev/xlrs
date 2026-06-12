#include "rc_handset/usb/protocol.h"

#include <ctype.h>
#include <stddef.h>
#include <stdlib.h>

#include <sstream>

namespace rc_handset {
namespace usb {

namespace {

bool isUnreserved(char c) {
    return isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_' ||
           c == '.' || c == '~';
}

int hexValue(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

std::string trimLineEnding(const std::string& line) {
    size_t end = line.size();
    while (end > 0 && (line[end - 1] == '\r' || line[end - 1] == '\n')) --end;
    return line.substr(0, end);
}

bool parseUint32(const std::string& value, uint32_t& out) {
    if (value.empty()) return false;
    uint32_t parsed = 0;
    for (char c : value) {
        if (!isdigit(static_cast<unsigned char>(c))) return false;
        const uint32_t digit = static_cast<uint32_t>(c - '0');
        if (parsed > (UINT32_MAX - digit) / 10u) return false;
        parsed = parsed * 10u + digit;
    }
    out = parsed;
    return true;
}

bool commandTypeFromName(const std::string& name, CommandType& out) {
    if (name == "hello") out = CommandType::Hello;
    else if (name == "tx_hello") out = CommandType::TxHello;
    else if (name == "get_config") out = CommandType::GetConfig;
    else if (name == "set_config") out = CommandType::SetConfig;
    else if (name == "apply") out = CommandType::Apply;
    else if (name == "save") out = CommandType::Save;
    else if (name == "reset_defaults") out = CommandType::ResetDefaults;
    else if (name == "state") out = CommandType::State;
    else if (name == "stream_state") out = CommandType::StreamState;
    else if (name == "cal_start") out = CommandType::CalStart;
    else if (name == "cal_sample") out = CommandType::CalSample;
    else if (name == "cal_finish") out = CommandType::CalFinish;
    else if (name == "binding_get") out = CommandType::BindingGet;
    else if (name == "binding_set") out = CommandType::BindingSet;
    else if (name == "binding_clear") out = CommandType::BindingClear;
    else if (name == "binding_verify") out = CommandType::BindingVerify;
    else if (name == "binding_bind") out = CommandType::BindingBind;
    else if (name == "special_mode") out = CommandType::SpecialMode;
    else return false;
    return true;
}

const Argument* findArgument(const ParsedCommand& command, const char* key) {
    for (const Argument& arg : command.arguments) {
        if (arg.key == key) return &arg;
    }
    return nullptr;
}

ParseResult parseError(ParseError error, const std::string& message) {
    ParseResult result{};
    result.ok = false;
    result.error = error;
    result.errorMessage = message;
    return result;
}

bool hydrateCommonFields(ParsedCommand& command, ParseResult& result) {
    if (const Argument* seq = findArgument(command, "seq")) {
        command.hasSeq = true;
        if (!parseUint32(seq->value, command.seq)) {
            result = parseError(ParseError::BadArgument, "invalid seq");
            return false;
        }
    }
    if (const Argument* field = findArgument(command, "field")) command.field = field->value;
    if (const Argument* value = findArgument(command, "value")) command.value = value->value;
    if (const Argument* phrase = findArgument(command, "phrase")) command.value = phrase->value;
    if (const Argument* target = findArgument(command, "target")) command.target = target->value;
    if (const Argument* mode = findArgument(command, "mode")) command.mode = mode->value;
    return true;
}

bool validateRequiredFields(const ParsedCommand& command, ParseResult& result) {
    switch (command.type) {
    case CommandType::SetConfig:
        if (command.field.empty() || command.value.empty()) {
            result = parseError(ParseError::MissingRequiredArgument,
                                "set_config requires field and value");
            return false;
        }
        break;
    case CommandType::BindingSet:
    case CommandType::BindingVerify:
        if (command.value.empty()) {
            result = parseError(ParseError::MissingRequiredArgument,
                                "binding command requires value");
            return false;
        }
        break;
    case CommandType::SpecialMode:
        if (command.mode.empty() && command.value.empty()) {
            result = parseError(ParseError::MissingRequiredArgument,
                                "special_mode requires mode or value");
            return false;
        }
        break;
    default:
        break;
    }
    return true;
}

std::string appendResponseFields(bool hasSeq, uint32_t seq,
                                 const std::vector<ResponseField>& fields,
                                 const char* status) {
    std::string out = std::string(kProtocolName) + " " + status;
    if (hasSeq) out += " seq=" + std::to_string(seq);
    for (const ResponseField& field : fields) {
        out += " " + field.key + "=" + percentEncode(field.value);
    }
    out += "\n";
    return out;
}

} // namespace

std::string percentEncode(const std::string& value) {
    static constexpr char kHex[] = "0123456789ABCDEF";
    std::string out;
    for (char c : value) {
        const unsigned char uc = static_cast<unsigned char>(c);
        if (isUnreserved(c)) {
            out += c;
        } else {
            out += '%';
            out += kHex[(uc >> 4) & 0x0F];
            out += kHex[uc & 0x0F];
        }
    }
    return out;
}

bool percentDecode(const std::string& encoded, std::string& decoded) {
    decoded.clear();
    for (size_t i = 0; i < encoded.size(); ++i) {
        if (encoded[i] != '%') {
            decoded += encoded[i];
            continue;
        }
        if (i + 2 >= encoded.size()) return false;
        const int hi = hexValue(encoded[i + 1]);
        const int lo = hexValue(encoded[i + 2]);
        if (hi < 0 || lo < 0) return false;
        decoded += static_cast<char>((hi << 4) | lo);
        i += 2;
    }
    return true;
}

ParseResult parseLine(const std::string& line) {
    const std::string cleaned = trimLineEnding(line);
    if (cleaned.empty()) return parseError(ParseError::EmptyLine, "empty line");

    std::istringstream stream(cleaned);
    std::string protocol;
    if (!(stream >> protocol) || protocol != kProtocolName) {
        return parseError(ParseError::BadProtocol, "expected rc.v1");
    }

    std::string commandName;
    if (!(stream >> commandName)) {
        return parseError(ParseError::MissingCommand, "missing command");
    }

    ParsedCommand command{};
    if (!commandTypeFromName(commandName, command.type)) {
        return parseError(ParseError::UnknownCommand, "unknown command");
    }

    std::string token;
    while (stream >> token) {
        const size_t equals = token.find('=');
        if (equals == std::string::npos || equals == 0) {
            return parseError(ParseError::BadArgument, "expected key=value");
        }
        std::string decoded;
        if (!percentDecode(token.substr(equals + 1), decoded)) {
            return parseError(ParseError::BadPercentEncoding, "invalid percent encoding");
        }
        command.arguments.push_back({token.substr(0, equals), decoded});
    }

    ParseResult result{};
    result.ok = true;
    result.command = command;
    if (!hydrateCommonFields(result.command, result)) return result;
    if (!validateRequiredFields(result.command, result)) return result;
    return result;
}

std::string formatOk(const ParsedCommand& command,
                     const std::vector<ResponseField>& fields) {
    return formatOk(command.hasSeq, command.seq, fields);
}

std::string formatOk(bool hasSeq, uint32_t seq,
                     const std::vector<ResponseField>& fields) {
    return appendResponseFields(hasSeq, seq, fields, "ok");
}

std::string formatErr(const ParsedCommand& command, const std::string& code,
                      const std::string& message) {
    return formatErr(command.hasSeq, command.seq, code, message);
}

std::string formatErr(bool hasSeq, uint32_t seq, const std::string& code,
                      const std::string& message) {
    return appendResponseFields(hasSeq, seq,
                                {{"code", code}, {"message", message}}, "err");
}

const char* commandTypeName(CommandType type) {
    switch (type) {
    case CommandType::Hello: return "hello";
    case CommandType::TxHello: return "tx_hello";
    case CommandType::GetConfig: return "get_config";
    case CommandType::SetConfig: return "set_config";
    case CommandType::Apply: return "apply";
    case CommandType::Save: return "save";
    case CommandType::ResetDefaults: return "reset_defaults";
    case CommandType::State: return "state";
    case CommandType::StreamState: return "stream_state";
    case CommandType::CalStart: return "cal_start";
    case CommandType::CalSample: return "cal_sample";
    case CommandType::CalFinish: return "cal_finish";
    case CommandType::BindingGet: return "binding_get";
    case CommandType::BindingSet: return "binding_set";
    case CommandType::BindingClear: return "binding_clear";
    case CommandType::BindingVerify: return "binding_verify";
    case CommandType::BindingBind: return "binding_bind";
    case CommandType::SpecialMode: return "special_mode";
    }
    return "unknown";
}

const char* parseErrorName(ParseError error) {
    switch (error) {
    case ParseError::None: return "none";
    case ParseError::EmptyLine: return "empty_line";
    case ParseError::BadProtocol: return "bad_protocol";
    case ParseError::MissingCommand: return "missing_command";
    case ParseError::UnknownCommand: return "unknown_command";
    case ParseError::BadArgument: return "bad_argument";
    case ParseError::BadPercentEncoding: return "bad_percent_encoding";
    case ParseError::MissingRequiredArgument: return "missing_required_argument";
    }
    return "unknown";
}

} // namespace usb
} // namespace rc_handset
