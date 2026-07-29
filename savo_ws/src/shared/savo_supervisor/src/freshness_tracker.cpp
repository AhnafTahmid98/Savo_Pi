// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_supervisor/freshness_tracker.hpp"

#include <cmath>
#include <limits>
#include <string>

namespace savo_supervisor
{
namespace
{

constexpr int64_t kNanosecondsPerSecond = 1000000000LL;

bool valid_header_stamp(
  const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec >= 0 &&
         stamp.nanosec < static_cast<uint32_t>(
    kNanosecondsPerSecond);
}

int64_t header_stamp_to_nanoseconds(
  const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) *
         kNanosecondsPerSecond +
         static_cast<int64_t>(stamp.nanosec);
}

}  // namespace

void FreshnessTracker::mark_disabled()
{
  disabled_ = true;
}

void FreshnessTracker::observe_message(
  const rclcpp::Time & receive_time,
  const std::optional<builtin_interfaces::msg::Time> & header_stamp,
  bool malformed,
  const std::string & detail)
{
  if (disabled_) {
    return;
  }

  const bool had_received = received_;

  const bool was_faulted =
    malformed_ ||
    timestamp_fault_ ||
    time_regression_ ||
    stale_active_;

  received_ = true;
  malformed_ = malformed;
  stale_active_ = false;
  timestamp_fault_ = false;
  time_regression_ = false;
  detail_.clear();

  if (malformed_) {
    ++malformed_count_;
    detail_ = detail.empty() ?
      "malformed payload" :
      detail;
  }

  if (had_received) {
    if (receive_time.get_clock_type() !=
      last_receive_time_.get_clock_type())
    {
      time_regression_ = true;
      detail_ = "receive clock type changed";
    } else {
      if (
        receive_time.nanoseconds() <
        last_receive_time_.nanoseconds())
      {
        time_regression_ = true;
        detail_ = "receive time regressed";
      }
    }
  }

  /*
   * Always accept the latest receive time as the new baseline.
   * This allows recovery after a simulated-time or ROS-time reset.
   */
  last_receive_time_ = receive_time;

  if (!malformed_ && header_stamp.has_value()) {
    const auto & stamp = header_stamp.value();

    if (!valid_header_stamp(stamp)) {
      timestamp_fault_ = true;
      detail_ = "invalid message timestamp";
    } else {
      const int64_t stamp_ns =
        header_stamp_to_nanoseconds(stamp);

      if (
        last_header_stamp_ns_ >= 0 &&
        stamp_ns < last_header_stamp_ns_)
      {
        timestamp_fault_ = true;
        detail_ = "message timestamp regressed";
      }

      /*
       * Use the new timestamp as the baseline even after regression.
       * The next monotonic timestamp can therefore recover.
       */
      last_header_stamp_ns_ = stamp_ns;
      last_header_stamp_ = stamp;
    }
  }

  const bool now_faulted =
    malformed_ ||
    timestamp_fault_ ||
    time_regression_;

  /*
   * Initial acquisition is not a recovery.
   * Recovery is counted only after a previously received stream
   * returns from malformed, timestamp, clock, or stale failure.
   */
  if (had_received && was_faulted && !now_faulted) {
    ++recovery_count_;
  }
}

FreshnessSnapshot FreshnessTracker::snapshot(
  const rclcpp::Time & now,
  double timeout_s) const
{
  FreshnessSnapshot result;

  result.disabled = disabled_;
  result.received = received_;
  result.malformed = malformed_;
  result.timestamp_fault = timestamp_fault_;
  result.time_regression = time_regression_;
  result.timeout_s = timeout_s;
  result.malformed_count = malformed_count_;
  result.recovery_count = recovery_count_;
  result.last_receive_time = last_receive_time_;
  result.last_header_stamp = last_header_stamp_;

  if (disabled_) {
    result.valid = false;
    result.detail = "disabled";
    return result;
  }

  if (!std::isfinite(timeout_s) || timeout_s <= 0.0) {
    result.valid = false;
    result.detail = "invalid timeout";
    return result;
  }

  if (!received_) {
    result.stale = true;
    result.valid = false;
    result.age_s =
      std::numeric_limits<double>::infinity();
    result.detail = "never received";
    return result;
  }

  if (now.get_clock_type() !=
    last_receive_time_.get_clock_type())
  {
    time_regression_ = true;
    result.time_regression = true;
    result.valid = false;
    result.detail = "clock type mismatch";
    return result;
  }

  if (
    now.nanoseconds() <
    last_receive_time_.nanoseconds())
  {
    time_regression_ = true;
    result.time_regression = true;
    result.valid = false;
    result.age_s = 0.0;
    result.detail = "current time regressed";
    return result;
  }

  const int64_t age_ns =
    now.nanoseconds() -
    last_receive_time_.nanoseconds();

  result.age_s =
    static_cast<double>(age_ns) /
    static_cast<double>(kNanosecondsPerSecond);

  /*
   * A message is still fresh exactly at the timeout boundary.
   */
  result.stale = result.age_s > timeout_s;

  if (result.stale) {
    stale_active_ = true;
  }

  if (malformed_) {
    result.valid = false;
    result.detail = detail_.empty() ?
      "malformed payload" :
      detail_;
  } else if (timestamp_fault_) {
    result.valid = false;
    result.detail = detail_.empty() ?
      "timestamp fault" :
      detail_;
  } else if (time_regression_) {
    result.valid = false;
    result.detail = detail_.empty() ?
      "time regression" :
      detail_;
  } else if (result.stale) {
    result.valid = false;
    result.detail = "message stale";
  } else {
    result.valid = true;
    result.detail = "fresh";
  }

  result.recovery_count = recovery_count_;
  return result;
}

bool FreshnessTracker::received() const
{
  return received_;
}

std::size_t FreshnessTracker::malformed_count() const
{
  return malformed_count_;
}

std::size_t FreshnessTracker::recovery_count() const
{
  return recovery_count_;
}

}  // namespace savo_supervisor
