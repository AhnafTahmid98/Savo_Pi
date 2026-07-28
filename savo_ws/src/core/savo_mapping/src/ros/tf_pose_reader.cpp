#include "savo_mapping/tf_pose_reader.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"

namespace savo_mapping
{
namespace
{

constexpr double kMinimumQuaternionNorm = 1.0e-12;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 6.28318530717958647692;
constexpr auto kLookupPollInterval =
  std::chrono::milliseconds(1);

bool frame_is_blank(const std::string & frame)
{
  return frame.empty() ||
         std::all_of(
    frame.begin(),
    frame.end(),
    [](const unsigned char character) {
      return std::isspace(character) != 0;
    });
}

TfPoseSnapshot failure_snapshot(
  const TfPoseReaderOptions & options,
  const std::string & reason,
  const std::string & detail = {})
{
  TfPoseSnapshot snapshot;
  snapshot.target_frame = options.target_frame;
  snapshot.source_frame = options.source_frame;
  snapshot.reason = reason;
  snapshot.detail = detail;
  return snapshot;
}

double elapsed_seconds(
  const std::chrono::steady_clock::time_point started_at)
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now() -
    started_at).count();
}

}  // namespace

std::string validate_tf_pose_reader_options(
  const TfPoseReaderOptions & options)
{
  if (frame_is_blank(options.target_frame)) {
    return "tf_pose_target_frame_empty";
  }

  if (frame_is_blank(options.source_frame)) {
    return "tf_pose_source_frame_empty";
  }

  if (
    !std::isfinite(options.lookup_timeout_sec) ||
    options.lookup_timeout_sec <= 0.0)
  {
    return "tf_pose_lookup_timeout_invalid";
  }

  if (
    !std::isfinite(options.stale_timeout_sec) ||
    options.stale_timeout_sec < 0.0)
  {
    return "tf_pose_stale_timeout_invalid";
  }

  return {};
}

TfPoseReader::TfPoseReader(
  rclcpp::Clock::SharedPtr clock,
  tf2_ros::Buffer::SharedPtr buffer)
: TfPoseReader(
    std::move(clock),
    std::move(buffer),
    TfPoseReaderOptions{})
{
}

TfPoseReader::TfPoseReader(
  rclcpp::Clock::SharedPtr clock,
  tf2_ros::Buffer::SharedPtr buffer,
  TfPoseReaderOptions options)
: clock_(std::move(clock)),
  buffer_(std::move(buffer)),
  options_(std::move(options))
{
  if (!clock_ || !buffer_) {
    throw std::invalid_argument(
            "tf_pose_options_invalid");
  }

  const std::string validation =
    validate_tf_pose_reader_options(options_);

  if (!validation.empty()) {
    throw std::invalid_argument(validation);
  }
}

const TfPoseReaderOptions &
TfPoseReader::options() const noexcept
{
  return options_;
}

TfPoseSnapshot TfPoseReader::read() const
{
  const auto started_at =
    std::chrono::steady_clock::now();

  const auto timeout =
    std::chrono::duration<double>(
    options_.lookup_timeout_sec);

  const auto maximum_timeout =
    std::chrono::duration<double>(
    std::chrono::steady_clock::duration::max());

  const auto deadline =
    timeout >= maximum_timeout ?
    std::chrono::steady_clock::time_point::max() :
    started_at +
    std::chrono::duration_cast<
    std::chrono::steady_clock::duration>(timeout);

  std::string last_reason{
    "tf_pose_transform_unavailable"};

  std::string last_detail;

  while (true) {
    try {
      const auto transform =
        buffer_->lookupTransform(
        options_.target_frame,
        options_.source_frame,
        tf2::TimePointZero,
        tf2::Duration::zero());

      rclcpp::Time current_time;

      try {
        current_time = clock_->now();
      } catch (const std::exception & exception) {
        auto snapshot = failure_snapshot(
          options_,
          "tf_pose_clock_invalid",
          exception.what());

        snapshot.lookup_duration_sec =
          elapsed_seconds(started_at);

        return snapshot;
      }

      auto snapshot = evaluate_transform(
        transform,
        current_time,
        options_.stale_timeout_sec);

      snapshot.target_frame =
        options_.target_frame;

      snapshot.source_frame =
        options_.source_frame;

      snapshot.lookup_duration_sec =
        elapsed_seconds(started_at);

      return snapshot;
    } catch (const tf2::TimeoutException & exception) {
      auto snapshot = failure_snapshot(
        options_,
        "tf_pose_lookup_timeout",
        exception.what());

      snapshot.lookup_duration_sec =
        elapsed_seconds(started_at);

      return snapshot;
    } catch (const tf2::ExtrapolationException & exception) {
      last_reason = "tf_pose_extrapolation_error";
      last_detail = exception.what();
    } catch (const tf2::ConnectivityException & exception) {
      last_reason = "tf_pose_connectivity_error";
      last_detail = exception.what();
    } catch (const tf2::InvalidArgumentException & exception) {
      auto snapshot = failure_snapshot(
        options_,
        "tf_pose_invalid_argument",
        exception.what());

      snapshot.lookup_duration_sec =
        elapsed_seconds(started_at);

      return snapshot;
    } catch (const tf2::LookupException & exception) {
      last_reason = "tf_pose_transform_unavailable";
      last_detail = exception.what();
    } catch (const tf2::TransformException & exception) {
      auto snapshot = failure_snapshot(
        options_,
        "tf_pose_transform_unavailable",
        exception.what());

      snapshot.lookup_duration_sec =
        elapsed_seconds(started_at);

      return snapshot;
    } catch (const std::exception & exception) {
      auto snapshot = failure_snapshot(
        options_,
        "tf_pose_transform_unavailable",
        exception.what());

      snapshot.lookup_duration_sec =
        elapsed_seconds(started_at);

      return snapshot;
    }

    const auto now = std::chrono::steady_clock::now();

    if (now >= deadline) {
      auto snapshot = failure_snapshot(
        options_,
        last_reason,
        last_detail);

      snapshot.lookup_duration_sec =
        std::chrono::duration<double>(
        now - started_at).count();

      return snapshot;
    }

    const auto remaining = deadline - now;

    std::this_thread::sleep_for(
      std::min(
        remaining,
        std::chrono::duration_cast<
          std::chrono::steady_clock::duration>(
          kLookupPollInterval)));
  }
}

TfPoseSnapshot TfPoseReader::evaluate_transform(
  const geometry_msgs::msg::TransformStamped & transform,
  const rclcpp::Time & now,
  const double stale_timeout_sec)
{
  TfPoseSnapshot snapshot;
  snapshot.target_frame = transform.header.frame_id;
  snapshot.source_frame = transform.child_frame_id;
  snapshot.transform_stamp = rclcpp::Time(
    0,
    0,
    now.get_clock_type());

  if (
    !std::isfinite(stale_timeout_sec) ||
    stale_timeout_sec < 0.0)
  {
    snapshot.reason =
      "tf_pose_stale_timeout_invalid";

    return snapshot;
  }

  const auto & translation =
    transform.transform.translation;

  if (
    !std::isfinite(translation.x) ||
    !std::isfinite(translation.y) ||
    !std::isfinite(translation.z))
  {
    snapshot.reason =
      "tf_pose_translation_invalid";

    return snapshot;
  }

  const auto & quaternion =
    transform.transform.rotation;

  if (
    !std::isfinite(quaternion.x) ||
    !std::isfinite(quaternion.y) ||
    !std::isfinite(quaternion.z) ||
    !std::isfinite(quaternion.w))
  {
    snapshot.reason =
      "tf_pose_quaternion_invalid";

    return snapshot;
  }

  const double quaternion_norm =
    std::hypot(
    std::hypot(quaternion.x, quaternion.y),
    std::hypot(quaternion.z, quaternion.w));

  if (
    !std::isfinite(quaternion_norm) ||
    quaternion_norm <= kMinimumQuaternionNorm)
  {
    snapshot.reason =
      "tf_pose_quaternion_invalid";

    return snapshot;
  }

  const double normalized_x =
    quaternion.x / quaternion_norm;

  const double normalized_y =
    quaternion.y / quaternion_norm;

  const double normalized_z =
    quaternion.z / quaternion_norm;

  const double normalized_w =
    quaternion.w / quaternion_norm;

  const double yaw =
    std::atan2(
    2.0 *
    ((normalized_w * normalized_z) +
    (normalized_x * normalized_y)),
    1.0 -
    (2.0 *
    ((normalized_y * normalized_y) +
    (normalized_z * normalized_z))));

  const double normalized_yaw =
    normalize_yaw(yaw);

  if (
    !std::isfinite(yaw) ||
    !std::isfinite(normalized_yaw))
  {
    snapshot.reason =
      "tf_pose_yaw_invalid";

    return snapshot;
  }

  if (
    transform.header.stamp.sec < 0 ||
    transform.header.stamp.nanosec >=
    1000000000U)
  {
    snapshot.reason =
      "tf_pose_timestamp_invalid";

    return snapshot;
  }

  rclcpp::Time transform_time;

  try {
    transform_time = rclcpp::Time(
      transform.header.stamp,
      now.get_clock_type());
  } catch (const std::exception & exception) {
    snapshot.reason =
      "tf_pose_timestamp_invalid";

    snapshot.detail = exception.what();
    return snapshot;
  }

  snapshot.transform_stamp = transform_time;

  if (now.nanoseconds() < 0) {
    snapshot.reason =
      "tf_pose_clock_invalid";

    return snapshot;
  }

  double age_sec = 0.0;

  if (
    transform_time.nanoseconds() != 0 &&
    transform_time > now)
  {
    snapshot.reason =
      "tf_pose_timestamp_invalid";

    return snapshot;
  }

  if (transform_time.nanoseconds() != 0) {
    age_sec = (now - transform_time).seconds();
  }

  if (
    !std::isfinite(age_sec) ||
    age_sec < 0.0)
  {
    snapshot.reason =
      "tf_pose_clock_invalid";

    return snapshot;
  }

  snapshot.x_m = translation.x;
  snapshot.y_m = translation.y;
  snapshot.z_m = translation.z;
  snapshot.quaternion_x = normalized_x;
  snapshot.quaternion_y = normalized_y;
  snapshot.quaternion_z = normalized_z;
  snapshot.quaternion_w = normalized_w;
  snapshot.yaw_rad = normalized_yaw;
  snapshot.age_sec = age_sec;

  if (
    stale_timeout_sec > 0.0 &&
    age_sec > stale_timeout_sec)
  {
    snapshot.reason =
      "tf_pose_transform_stale";

    return snapshot;
  }

  snapshot.valid = true;
  snapshot.fresh = true;
  snapshot.reason = "tf_pose_ready";
  return snapshot;
}

double TfPoseReader::normalize_yaw(
  const double yaw_rad) noexcept
{
  if (!std::isfinite(yaw_rad)) {
    return std::numeric_limits<double>::
           quiet_NaN();
  }

  double normalized =
    std::fmod(yaw_rad + kPi, kTwoPi);

  if (normalized < 0.0) {
    normalized += kTwoPi;
  }

  return normalized - kPi;
}

}  // namespace savo_mapping
