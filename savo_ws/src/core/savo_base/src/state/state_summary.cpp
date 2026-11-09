#include "savo_base/state/state_summary.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <utility>

#include <rapidjson/reader.h>
#include <rapidjson/memorystream.h>
#include <rapidjson/writer.h>

namespace savo_base::summary
{
namespace
{
// A reusable std::string sink avoids a second full summary-buffer copy.
struct StringSink
{
  using Ch = char;
  std::string & value;
  void Put(char c) {value.push_back(c);}
  void Flush() {}
};
using Writer = rapidjson::Writer<StringSink, rapidjson::UTF8<>, rapidjson::UTF8<>,
    rapidjson::CrtAllocator, rapidjson::kWriteNanAndInfFlag>;

void text(Writer & writer, const std::string & value)
{
  writer.String(value.data(), static_cast<rapidjson::SizeType>(value.size()));
}
void raw(Writer & writer, const std::string & value)
{
  // RawValue's type is only used for writer bookkeeping, not validation.
  writer.RawValue(value.data(), value.size(), rapidjson::kObjectType);
}
void field(Writer & w, const char * key, double value) {w.Key(key); w.Double(value);}
void field(Writer & w, const char * key, std::uint64_t value) {w.Key(key); w.Uint64(value);}
void field(Writer & w, const char * key, bool value) {w.Key(key); w.Bool(value);}
void field(Writer & w, const char * key, const std::string & value) {w.Key(key); text(w, value);}

// Non-ASCII uppercase mappings relevant to the ASCII status vocabulary:
// dotless i, long s and the two ST ligatures. Do not use a process locale.
std::string status_upper(const std::string & value)
{
  std::string result;
  for (std::size_t i = 0; i < value.size(); ++i) {
    const auto c = static_cast<unsigned char>(value[i]);
    if (i + 1 < value.size() && c == 0xc4 &&
      static_cast<unsigned char>(value[i + 1]) == 0xb1)
    {
      result += 'I'; ++i;
    } else if (i + 1 < value.size() && c == 0xc5 &&
      static_cast<unsigned char>(value[i + 1]) == 0xbf)
    {
      result += 'S'; ++i;
    } else if (i + 2 < value.size() && c == 0xef &&
      static_cast<unsigned char>(value[i + 1]) == 0xac &&
      (static_cast<unsigned char>(value[i + 2]) == 0x85 ||
      static_cast<unsigned char>(value[i + 2]) == 0x86))
    {
      result += "ST"; i += 2;
    } else {
      result += c >= 'a' && c <= 'z' ? static_cast<char>(c - 'a' + 'A') : value[i];
    }
  }
  return result;
}

// Validate and normalize a payload in one streaming pass. Raw numeric tokens
// preserve arbitrary-size Python integers and NaN/Infinity. No nested DOM is
// retained or rebuilt on summary ticks. Duplicate keys retain JSON's last-value
// semantics, including when the last status_level is not a string.
struct InputHandler : rapidjson::BaseReaderHandler<rapidjson::UTF8<>, InputHandler>
{
  explicit InputHandler(Writer & output) : writer(output) {}
  Writer & writer;
  unsigned depth{0};
  bool root_object{false};
  bool status_key{false};
  std::string status;
  const std::vector<std::string> * number_tokens{nullptr};
  std::size_t number_index{0};
  void value() {if (depth == 1 && status_key) {status.clear();}}
  bool Null() {value(); return writer.Null();}
  bool Bool(bool v) {value(); return writer.Bool(v);}
  bool RawNumber(const char * s, rapidjson::SizeType n, bool)
  {
    value();
    const std::string token = number_tokens ? number_tokens->at(number_index++) : std::string(s, n);
    // RapidJSON also accepts Inf and -NaN; Python json.loads does not.
    if (token == "Inf" || token == "-Inf" || token == "-NaN") {return false;}
    return writer.RawValue(token.data(), token.size(), rapidjson::kNumberType);
  }
  bool String(const char * s, rapidjson::SizeType n, bool copy)
  {
    value();
    if (depth == 1 && status_key && root_object) {status = status_upper(std::string(s, n));}
    return writer.String(s, n, copy);
  }
  bool Key(const char * s, rapidjson::SizeType n, bool copy)
  {
    if (depth == 1) {status_key = std::string(s, n) == "status_level";}
    return writer.Key(s, n, copy);
  }
  bool StartObject()
  {
    value();
    if (depth == 0) {root_object = true;}
    ++depth;
    return writer.StartObject();
  }
  bool EndObject(rapidjson::SizeType n) {--depth; return writer.EndObject(n);}
  bool StartArray() {value(); ++depth; return writer.StartArray();}
  bool EndArray(rapidjson::SizeType n) {--depth; return writer.EndArray(n);}
};

// Iterative JSON number grammar: bounded stack even for enormous exponents.
bool valid_number(const std::string & token)
{
  std::size_t i = 0;
  const auto digit = [&](std::size_t n) {
      return n < token.size() && token[n] >= '0' && token[n] <= '9';
    };
  if (i < token.size() && token[i] == '-') {++i;}
  if (!digit(i)) {return false;}
  if (token[i] == '0') {++i;} else {while (digit(i)) {++i;}}
  if (i < token.size() && token[i] == '.') {
    ++i;
    if (!digit(i)) {return false;}
    while (digit(i)) {++i;}
  }
  if (i < token.size() && (token[i] == 'e' || token[i] == 'E')) {
    ++i;
    if (i < token.size() && (token[i] == '-' || token[i] == '+')) {++i;}
    if (!digit(i)) {return false;}
    while (digit(i)) {++i;}
  }
  return i == token.size();
}

// RapidJSON 1.1 checks double range even with NumbersAsStrings. Only on that
// error, validate numeric grammar separately and let the same JSON parser see
// harmless zero placeholders. The SAX handler emits the original numeric tokens.
// This preserves Python's large integers without slowing the usual input path.
bool large_number_input(
  const std::string & input, std::string & normalized, std::string & status, bool & object)
{
  std::string substituted;
  std::vector<std::string> tokens;
  bool quoted = false, escaped = false;
  for (std::size_t i = 0; i < input.size(); ++i) {
    const char c = input[i];
    if (quoted) {
      substituted += c;
      if (escaped) {escaped = false;} else if (c == '\\') {escaped = true;}
      else if (c == '"') {quoted = false;}
    } else if (c == '"') {quoted = true; substituted += c;
    } else if (c == '-' || (c >= '0' && c <= '9') || c == 'N' || c == 'I') {
      const auto begin = i;
      while (i < input.size() && input[i] != ',' && input[i] != ']' && input[i] != '}' &&
        input[i] != ' ' && input[i] != '\t' && input[i] != '\r' && input[i] != '\n') {++i;}
      const auto token = input.substr(begin, i - begin);
      if (token != "NaN" && token != "Infinity" && token != "-Infinity" &&
        !valid_number(token)) {return false;}
      // CPython 3.12's default integer-string conversion limit.
      if (token.find_first_of(".eE") == std::string::npos &&
        token.size() - (token[0] == '-' ? 1 : 0) > 4300) {return false;}
      tokens.push_back(token);
      substituted += '0';
      --i;
    } else {substituted += c;}
  }
  normalized.clear();
  StringSink sink{normalized};
  Writer writer(sink);
  InputHandler handler(writer);
  handler.number_tokens = &tokens;
  rapidjson::MemoryStream stream(substituted.data(), substituted.size());
  rapidjson::Reader reader;
  if (!reader.Parse<rapidjson::kParseNumbersAsStringsFlag>(stream, handler) ||
    stream.Tell() != substituted.size()) {return false;}
  status = std::move(handler.status);
  object = handler.root_object;
  return true;
}

// Formatting only: cached JSON is already validated. Avoid reparsing even in
// pretty mode; string contents (including escaped quotes) remain untouched.
void pretty(const std::string & compact, std::string & output)
{
  output.clear();
  unsigned depth = 0;
  bool quoted = false, escaped = false;
  auto newline = [&]() {output += '\n'; output.append(depth * 2, ' ');};
  for (std::size_t i = 0; i < compact.size(); ++i) {
    const char c = compact[i];
    if (quoted) {
      output += c;
      if (escaped) {escaped = false;} else if (c == '\\') {escaped = true;}
      else if (c == '"') {quoted = false;}
    } else if (c == '"') {quoted = true; output += c;
    } else if (c == '{' || c == '[') {
      output += c; ++depth;
      if (i + 1 < compact.size() && compact[i + 1] != '}' && compact[i + 1] != ']') {newline();}
    } else if (c == '}' || c == ']') {
      --depth;
      if (i > 0 && compact[i - 1] != '{' && compact[i - 1] != '[') {newline();}
      output += c;
    } else if (c == ',') {output += c; newline();
    } else if (c == ':') {output += ": ";
    } else {output += c;}
  }
}

void metadata(Writer & w, const Sample & sample, double now, double timeout)
{
  field(w, "seen", sample.count > 0);
  field(w, "count", sample.count);
  field(w, "stale", sample.stale(now, timeout));
  field(w, "age_s", sample.age(now));
  field(w, "stale_timeout_s", timeout);
}
}  // namespace

void Parameters::clamp()
{
  publish_hz = std::max(0.5, publish_hz);
  heartbeat_hz = std::max(0.2, heartbeat_hz);
  cmd_stale_s = std::max(0.05, cmd_stale_s);
  safety_stale_s = std::max(0.05, safety_stale_s);
  watchdog_stale_s = std::max(0.10, watchdog_stale_s);
  base_state_stale_s = std::max(0.10, base_state_stale_s);
  motor_board_status_stale_s = std::max(0.10, motor_board_status_stale_s);
}

double Sample::age(double now) const
{
  return time <= 0.0 ? std::numeric_limits<double>::infinity() : std::max(0.0, now - time);
}

void JsonSample::receive_json(const std::string & input, double now)
{
  const bool repeated = count > 0 && input == previous_raw;
  receive(now);
  if (repeated) {
    if (!previous_valid) {++parse_error_count;}
    return;
  }
  previous_raw = input;
  previous_valid = false;
  std::string normalized;
  normalized.reserve(input.size());
  StringSink sink{normalized};
  Writer writer(sink);
  InputHandler handler(writer);
  rapidjson::MemoryStream stream(input.data(), input.size());
  rapidjson::Reader reader;
  constexpr unsigned flags = rapidjson::kParseNumbersAsStringsFlag | rapidjson::kParseNanAndInfFlag;
  const auto result = reader.Parse<flags>(stream, handler);
  // MemoryStream exposes embedded NUL as EOF, unlike Python: require full consumption.
  bool object = handler.root_object;
  if (!result || stream.Tell() != input.size()) {
    if (result.Code() != rapidjson::kParseErrorNumberTooBig ||
      !large_number_input(input, normalized, handler.status, object))
    {
      ++parse_error_count; return;
    }
  }
  data = object ? std::move(normalized) : "{\"_raw_value\":" + normalized + "}";
  status = object ? std::move(handler.status) : "";
  previous_valid = true;
}

StateSummary::StateSummary(Parameters parameters, double started)
: params_(std::move(parameters)), started_(started)
{
  params_.clamp();
  StringSink sink{topics_json_};
  Writer w(sink);
  w.StartObject(); w.Key("inputs"); w.StartObject();
  const auto input = [&](const char * key, bool enabled, const std::string & topic) {
      w.Key(key); if (enabled) {text(w, topic);} else {w.Null();}
    };
  input("cmd_topic", params_.subscribe_cmd, params_.cmd_topic);
  input("safety_stop_topic", params_.subscribe_safety_stop, params_.safety_stop_topic);
  input("slowdown_topic", params_.subscribe_slowdown, params_.slowdown_topic);
  input("watchdog_topic", params_.subscribe_watchdog, params_.watchdog_topic);
  input("base_state_topic", params_.subscribe_base_state, params_.base_state_topic);
  input("motor_board_status_topic", params_.subscribe_motor_board_status, params_.motor_board_status_topic);
  w.EndObject(); w.Key("outputs"); w.StartObject();
  field(w, "summary_topic", params_.summary_topic);
  field(w, "heartbeat_topic", params_.heartbeat_topic);
  w.EndObject(); w.EndObject();
  compact_buffer_.reserve(4096);
}

void StateSummary::command(double vx, double vy, double wz, double now)
{
  vx_ = vx; vy_ = vy; wz_ = wz; cmd_.receive(now);
}
void StateSummary::safety_stop(bool value, double now) {stop_value_ = value; stop_.receive(now);}
void StateSummary::slowdown(double value, double now) {slowdown_value_ = value; slowdown_.receive(now);}

std::string StateSummary::status(double now) const
{
  if (stop_.count > 0 && stop_value_) {return "SAFETY_STOP";}
  const auto & wd = json_[0];
  if (!wd.stale(now, params_.watchdog_stale_s) &&
    (wd.status == "ERROR" || wd.status == "STALE" || wd.status == "TRIPPED" || wd.status == "FAULT"))
  {
    return wd.status;
  }
  const auto & bs = json_[1];
  if (!bs.stale(now, params_.base_state_stale_s) &&
    (bs.status == "ERROR" || bs.status == "FAULT" || bs.status == "STALE" ||
    bs.status == "WARN" || bs.status == "WARNING"))
  {
    return bs.status == "WARNING" ? "WARN" : bs.status;
  }
  if (params_.subscribe_cmd && cmd_.stale(now, params_.cmd_stale_s)) {return "CMD_STALE";}
  return "OK";
}

void StateSummary::serialize(double now, std::string & output)
{
  auto & buffer = params_.pretty_json ? compact_buffer_ : output;
  buffer.clear();
  StringSink sink{buffer};
  Writer w(sink);
  w.StartObject();
  field(w, "node", std::string("base_state_publisher_node"));
  field(w, "robot_name", params_.robot_name);
  field(w, "status_level", status(now));
  field(w, "timestamp_monotonic_s", now);
  field(w, "uptime_s", std::max(0.0, now - started_));
  field(w, "summary_pub_count", summary_count_ + 1);
  w.Key("topics"); raw(w, topics_json_);
  w.Key("command_path"); w.StartObject();
  w.Key("cmd_vel_safe"); w.StartObject();
  metadata(w, cmd_, now, params_.cmd_stale_s);
  field(w, "vx", vx_); field(w, "vy", vy_); field(w, "wz", wz_); w.EndObject();
  w.Key("safety_stop"); w.StartObject();
  metadata(w, stop_, now, params_.safety_stale_s); field(w, "value", stop_value_); w.EndObject();
  w.Key("slowdown_factor"); w.StartObject();
  metadata(w, slowdown_, now, params_.safety_stale_s); field(w, "value", slowdown_value_); w.EndObject();
  w.EndObject();
  w.Key("driver_state_topics"); w.StartObject();
  const char * keys[] = {"watchdog_state", "base_state", "motor_board_status"};
  const double timeouts[] = {params_.watchdog_stale_s, params_.base_state_stale_s,
    params_.motor_board_status_stale_s};
  for (std::size_t i = 0; i < json_.size(); ++i) {
    w.Key(keys[i]); w.StartObject(); metadata(w, json_[i], now, timeouts[i]);
    field(w, "parse_error_count", json_[i].parse_error_count);
    w.Key("data"); raw(w, json_[i].data); w.EndObject();
  }
  w.EndObject();
  w.Key("diagnostics"); w.StartObject(); w.Key("notes"); w.StartArray();
  w.String("This node aggregates status for dashboards and logging.");
  w.String("Motor execution remains in base_driver_node.");
  w.String("Missing inputs are tolerated during staged bringup.");
  w.EndArray(); w.EndObject(); w.EndObject();
  if (params_.pretty_json) {pretty(compact_buffer_, output);}
}
}  // namespace savo_base::summary
