#include "savo_power/power_health.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>

namespace savo_power
{
namespace
{

std::string lowercase(std::string value)
{
  std::transform(
    value.begin(),
    value.end(),
    value.begin(),
    [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });

  return value;
}

bool contains_token(
  const std::string & text,
  const std::string & token)
{
  return text.find(token) != std::string::npos;
}

std::string reason_for_state(PowerState state)
{
  switch (state) {
    case PowerState::OK:
    case PowerState::FULL:
    case PowerState::CHARGING:
      return "power_ok";

    case PowerState::LOW:
      return "power_low";

    case PowerState::CRITICAL:
      return "power_critical";

    case PowerState::ERROR:
      return "power_error";

    case PowerState::STALE:
      return "power_stale";

    case PowerState::UNKNOWN:
      return "power_unknown";
  }

  return "power_unknown";
}

}  // namespace

PowerHealth::PowerHealth(PowerHealthConfig config)
: config_(config)
{
}

const PowerHealthConfig & PowerHealth::config() const
{
  return config_;
}

void PowerHealth::set_config(const PowerHealthConfig & config)
{
  config_ = config;
}

PowerHealthResult PowerHealth::evaluate(
  const PowerHealthInput & input) const
{
  PowerHealthResult result;

  result.missing = !input.seen;
  result.stale = false;

  if (result.missing) {
    result.state = PowerState::STALE;
    result.level = config_.missing_status_is_error ?
      PowerHealthLevel::ERROR :
      PowerHealthLevel::UNKNOWN;
    result.reason = "missing_power_status";
    result.text = make_health_text(result, input);
    return result;
  }

  result.stale = input.age_s > config_.stale_timeout_s;

  if (result.stale) {
    result.state = PowerState::STALE;
    result.level = config_.stale_status_is_error ?
      PowerHealthLevel::ERROR :
      PowerHealthLevel::WARN;
    result.reason = "stale_power_status";
    result.text = make_health_text(result, input);
    return result;
  }

  result.state = input.state;
  result.level = level_for_state(input.state);
  result.reason = reason_for_state(input.state);
  result.text = make_health_text(result, input);

  return result;
}

PowerHealthResult PowerHealth::evaluate_status_text(
  bool seen,
  double age_s,
  const std::string & status_text) const
{
  PowerHealthInput input;
  input.seen = seen;
  input.age_s = age_s;
  input.status_text = status_text;
  input.state = state_from_status_text(status_text);

  return evaluate(input);
}

PowerState PowerHealth::state_from_status_text(
  const std::string & status_text)
{
  const std::string lower = lowercase(status_text);

  if (
    contains_token(lower, "overall=critical") ||
    contains_token(lower, "state=critical"))
  {
    return PowerState::CRITICAL;
  }

  if (
    contains_token(lower, "overall=error") ||
    contains_token(lower, "state=error"))
  {
    return PowerState::ERROR;
  }

  if (
    contains_token(lower, "overall=stale") ||
    contains_token(lower, "state=stale"))
  {
    return PowerState::STALE;
  }

  if (
    contains_token(lower, "overall=low") ||
    contains_token(lower, "state=low"))
  {
    return PowerState::LOW;
  }

  if (
    contains_token(lower, "overall=unknown") ||
    contains_token(lower, "state=unknown"))
  {
    return PowerState::UNKNOWN;
  }

  if (
    contains_token(lower, "overall=charging") ||
    contains_token(lower, "state=charging"))
  {
    return PowerState::CHARGING;
  }

  if (
    contains_token(lower, "overall=full") ||
    contains_token(lower, "state=full"))
  {
    return PowerState::FULL;
  }

  if (
    contains_token(lower, "overall=ok") ||
    contains_token(lower, "state=ok"))
  {
    return PowerState::OK;
  }

  return PowerState::UNKNOWN;
}

PowerHealthLevel PowerHealth::level_for_state(
  PowerState state) const
{
  if (state == PowerState::UNKNOWN && !config_.unknown_state_is_error) {
    return PowerHealthLevel::UNKNOWN;
  }

  return health_level_from_power_state(state);
}

std::string PowerHealth::make_health_text(
  const PowerHealthResult & result,
  const PowerHealthInput & input) const
{
  std::ostringstream out;

  out << "level=" << std::string(to_string(result.level))
      << " state=" << std::string(to_string(result.state))
      << " reason=" << result.reason;

  if (result.stale) {
    out << " age_s=" << static_cast<int>(input.age_s);
  }

  if (!input.status_text.empty() && !result.missing && !result.stale) {
    out << " status=\"" << input.status_text << "\"";
  }

  return out.str();
}

}  // namespace savo_power
