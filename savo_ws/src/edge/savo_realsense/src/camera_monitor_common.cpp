#include "savo_realsense/camera_monitor_common.hpp"

#include <algorithm>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>

#include "diagnostic_msgs/msg/key_value.hpp"

namespace savo_realsense
{

bool StreamStatus::ok() const
{
  return seen && !stale && rate_quality != "BELOW_MINIMUM";
}

std::string classify_rate_quality(
  const double rate_hz,
  const double minimum_hz,
  const double good_hz,
  const double excellent_hz)
{
  if (!std::isfinite(rate_hz) || rate_hz < minimum_hz) {
    return "BELOW_MINIMUM";
  }
  if (rate_hz < good_hz) {
    return "MINIMUM";
  }
  if (rate_hz < excellent_hz) {
    return "GOOD";
  }
  return "EXCELLENT";
}

RateTracker::RateTracker(std::size_t window_size)
: window_size_(std::max<std::size_t>(1, window_size))
{
}

void RateTracker::tick(const TimePoint now)
{
  if (seen_) {
    const double dt = std::chrono::duration<double>(now - last_time_).count();
    if (dt > 0.0) {
      intervals_.push_back(dt);
      while (intervals_.size() > window_size_) {
        intervals_.pop_front();
      }
    }
  }

  last_time_ = now;
  seen_ = true;
}

bool RateTracker::seen() const
{
  return seen_;
}

double RateTracker::rate_hz() const
{
  if (intervals_.empty()) {
    return 0.0;
  }

  const double total = std::accumulate(intervals_.begin(), intervals_.end(), 0.0);
  if (total <= 0.0) {
    return 0.0;
  }

  return static_cast<double>(intervals_.size()) / total;
}

double RateTracker::last_age_s(const TimePoint now) const
{
  if (!seen_) {
    return std::numeric_limits<double>::infinity();
  }

  const double age = std::chrono::duration<double>(now - last_time_).count();
  return age < 0.0 ? 0.0 : age;
}

StreamStatus build_stream_status(
  const std::string & topic,
  const RateTracker & tracker,
  const RateTracker::TimePoint now,
  const double expected_hz,
  const double stale_timeout_s,
  const double minimum_hz,
  const double good_hz,
  const double excellent_hz)
{
  const double last_age = tracker.last_age_s(now);

  StreamStatus status;
  status.topic = topic;
  status.seen = tracker.seen();
  status.stale = !status.seen || last_age > stale_timeout_s;
  status.rate_hz = tracker.rate_hz();
  status.expected_hz = expected_hz;
  status.last_age_s = last_age;
  status.minimum_hz = minimum_hz;
  status.good_hz = good_hz;
  status.excellent_hz = excellent_hz;
  status.rate_quality = classify_rate_quality(
    status.rate_hz, minimum_hz, good_hz, excellent_hz);
  return status;
}

std::string bool_text(bool value)
{
  return value ? "true" : "false";
}

std::string number_text(double value)
{
  if (!std::isfinite(value)) {
    return "inf";
  }

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << value;
  return stream.str();
}

diagnostic_msgs::msg::DiagnosticStatus make_stream_diagnostic(
  const std::string & name,
  const StreamStatus & status)
{
  diagnostic_msgs::msg::DiagnosticStatus diagnostic;
  diagnostic.name = name;
  diagnostic.hardware_id = "realsense_d435";

  if (status.ok()) {
    diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostic.message = "stream " + status.rate_quality;
  } else if (!status.seen) {
    diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diagnostic.message = "stream not seen";
  } else if (status.stale) {
    diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diagnostic.message = "stream stale";
  } else {
    diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diagnostic.message = "stream BELOW_MINIMUM";
  }

  auto add_value = [&](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = value;
      diagnostic.values.push_back(item);
    };

  add_value("topic", status.topic);
  add_value("seen", bool_text(status.seen));
  add_value("stale", bool_text(status.stale));
  add_value("rate_hz", number_text(status.rate_hz));
  add_value("expected_hz", number_text(status.expected_hz));
  add_value("last_age_s", number_text(status.last_age_s));
  add_value("rate_quality", status.rate_quality);
  add_value("minimum_hz", number_text(status.minimum_hz));
  add_value("good_hz", number_text(status.good_hz));
  add_value("excellent_hz", number_text(status.excellent_hz));

  return diagnostic;
}

diagnostic_msgs::msg::DiagnosticArray make_diagnostic_array(
  const std::vector<diagnostic_msgs::msg::DiagnosticStatus> & statuses,
  const rclcpp::Time & stamp)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;
  array.status = statuses;
  return array;
}

}  // namespace savo_realsense
