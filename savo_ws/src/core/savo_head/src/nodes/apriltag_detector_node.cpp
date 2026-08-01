// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <functional>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag36h11.h>
#include <opencv2/core/mat.hpp>

#include "cv_bridge/cv_bridge.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "savo_msgs/msg/april_tag_observation.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "savo_head/core/apriltag_detector_config.hpp"

namespace savo_head
{
namespace
{

using Observation = savo_msgs::msg::AprilTagObservation;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

constexpr double kNanosecondsPerSecond = 1.0e9;

[[nodiscard]] std::chrono::nanoseconds seconds_to_period(const double seconds)
{
  const auto safe_seconds = std::max(0.001, seconds);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(safe_seconds));
}

[[nodiscard]] diagnostic_msgs::msg::KeyValue make_key_value(
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

[[nodiscard]] diagnostic_msgs::msg::KeyValue make_key_value(
  const std::string & key,
  const std::uint64_t value)
{
  return make_key_value(key, std::to_string(value));
}

[[nodiscard]] diagnostic_msgs::msg::KeyValue make_key_value(
  const std::string & key,
  const std::int64_t value)
{
  return make_key_value(key, std::to_string(value));
}

[[nodiscard]] diagnostic_msgs::msg::KeyValue make_key_value(
  const std::string & key,
  const double value)
{
  std::ostringstream stream;
  stream << value;
  return make_key_value(key, stream.str());
}

[[nodiscard]] diagnostic_msgs::msg::KeyValue make_key_value(
  const std::string & key,
  const bool value)
{
  return make_key_value(key, std::string(value ? "true" : "false"));
}

[[nodiscard]] std::string join_errors(const std::vector<std::string> & errors)
{
  std::ostringstream stream;
  for (std::size_t index = 0; index < errors.size(); ++index) {
    if (index != 0U) {
      stream << ',';
    }
    stream << errors[index];
  }
  return stream.str();
}

[[nodiscard]] bool finite_detection(const apriltag_detection_t & detection)
{
  if (detection.id < 0 || detection.hamming < 0 ||
    !std::isfinite(detection.decision_margin) ||
    !std::isfinite(detection.c[0]) ||
    !std::isfinite(detection.c[1]))
  {
    return false;
  }

  for (const auto & corner : detection.p) {
    if (!std::isfinite(corner[0]) || !std::isfinite(corner[1])) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] bool calibrated_camera_info(
  const sensor_msgs::msg::CameraInfo & camera_info)
{
  const double fx = camera_info.k[0];
  const double fy = camera_info.k[4];
  const double cx = camera_info.k[2];
  const double cy = camera_info.k[5];

  return std::isfinite(fx) && std::isfinite(fy) &&
         std::isfinite(cx) && std::isfinite(cy) &&
         fx > 0.0 && fy > 0.0;
}

[[nodiscard]] double absolute_stamp_difference_seconds(
  const builtin_interfaces::msg::Time & lhs,
  const builtin_interfaces::msg::Time & rhs)
{
  const auto lhs_ns = rclcpp::Time(lhs).nanoseconds();
  const auto rhs_ns = rclcpp::Time(rhs).nanoseconds();
  const auto difference_ns =
    lhs_ns >= rhs_ns ? lhs_ns - rhs_ns : rhs_ns - lhs_ns;
  return static_cast<double>(difference_ns) / kNanosecondsPerSecond;
}

[[nodiscard]] bool valid_image_buffer(const sensor_msgs::msg::Image & image)
{
  if (image.width == 0U || image.height == 0U || image.step == 0U) {
    return false;
  }

  const auto height = static_cast<std::size_t>(image.height);
  const auto step = static_cast<std::size_t>(image.step);
  if (step > std::numeric_limits<std::size_t>::max() / height) {
    return false;
  }
  return image.data.size() >= step * height;
}

}  // namespace

class AprilTagDetectorNode final : public rclcpp::Node
{
public:
  AprilTagDetectorNode()
  : Node("apriltag_detector_node")
  {
    config_ = load_configuration();
    const auto validation_errors = validate_apriltag_detector_config(config_);
    if (!validation_errors.empty()) {
      throw std::runtime_error(
              "invalid AprilTag detector configuration: " +
              join_errors(validation_errors));
    }

    initialize_detector();

    observation_publisher_ = create_publisher<Observation>(
      config_.observation_topic,
      rclcpp::QoS(20).reliable());

    diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      config_.diagnostics_topic,
      rclcpp::QoS(10).reliable());

    camera_info_subscription_ =
      create_subscription<sensor_msgs::msg::CameraInfo>(
      config_.camera_info_topic,
      rclcpp::SensorDataQoS().keep_last(5),
      std::bind(
        &AprilTagDetectorNode::on_camera_info,
        this,
        std::placeholders::_1));

    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      config_.image_topic,
      rclcpp::SensorDataQoS().keep_last(1),
      std::bind(
        &AprilTagDetectorNode::on_image,
        this,
        std::placeholders::_1));

    diagnostics_timer_ = create_wall_timer(
      seconds_to_period(1.0 / config_.diagnostics_rate_hz),
      std::bind(&AprilTagDetectorNode::publish_diagnostics, this));

    worker_thread_ = std::thread(&AprilTagDetectorNode::worker_loop, this);

    RCLCPP_INFO(
      get_logger(),
      "AprilTag detector ready enabled=%s family=%s tag_size=%.3f image=%s "
      "camera_info=%s observations=%s frame=%s",
      config_.enabled ? "true" : "false",
      config_.family.c_str(),
      config_.tag_size_m,
      config_.image_topic.c_str(),
      config_.camera_info_topic.c_str(),
      config_.observation_topic.c_str(),
      config_.camera_optical_frame.c_str());
  }

  ~AprilTagDetectorNode() override
  {
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      stopping_ = true;
      pending_image_.reset();
    }
    worker_condition_.notify_all();
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }

    destroy_detector();
  }

private:
  [[nodiscard]] AprilTagDetectorConfig load_configuration()
  {
    AprilTagDetectorConfig config;

    config.enabled = declare_parameter<bool>(
      "apriltag_enabled", config.enabled);
    config.family = declare_parameter<std::string>(
      "apriltag_family", config.family);
    config.tag_size_m = declare_parameter<double>(
      "tag_size_m", config.tag_size_m);
    config.pose_estimation_enabled = declare_parameter<bool>(
      "pose_estimation_enabled", config.pose_estimation_enabled);

    config.image_topic = declare_parameter<std::string>(
      "image_topic", config.image_topic);
    config.camera_info_topic = declare_parameter<std::string>(
      "camera_info_topic", config.camera_info_topic);
    config.observation_topic = declare_parameter<std::string>(
      "observation_topic", config.observation_topic);
    config.diagnostics_topic = declare_parameter<std::string>(
      "diagnostics_topic", config.diagnostics_topic);
    config.camera_optical_frame = declare_parameter<std::string>(
      "camera_optical_frame", config.camera_optical_frame);
    config.detector_name = declare_parameter<std::string>(
      "detector_name", config.detector_name);

    config.allowed_tag_ids = declare_parameter<std::vector<std::int64_t>>(
      "allowed_tag_ids", config.allowed_tag_ids);
    config.maximum_hamming_distance = static_cast<int>(
      declare_parameter<std::int64_t>(
        "maximum_hamming_distance",
        config.maximum_hamming_distance));
    config.minimum_decision_margin = declare_parameter<double>(
      "minimum_decision_margin", config.minimum_decision_margin);
    config.full_quality_decision_margin = declare_parameter<double>(
      "full_quality_decision_margin", config.full_quality_decision_margin);
    config.maximum_detection_distance_m = declare_parameter<double>(
      "maximum_detection_distance_m", config.maximum_detection_distance_m);

    config.detector_threads = static_cast<int>(
      declare_parameter<std::int64_t>(
        "detector_threads", config.detector_threads));
    config.quad_decimate = declare_parameter<double>(
      "quad_decimate", config.quad_decimate);
    config.quad_sigma = declare_parameter<double>(
      "quad_sigma", config.quad_sigma);
    config.refine_edges = declare_parameter<bool>(
      "refine_edges", config.refine_edges);
    config.decode_sharpening = declare_parameter<double>(
      "decode_sharpening", config.decode_sharpening);

    config.camera_info_max_age_s = declare_parameter<double>(
      "camera_info_max_age_s", config.camera_info_max_age_s);
    config.require_matching_camera_info_stamp = declare_parameter<bool>(
      "require_matching_camera_info_stamp",
      config.require_matching_camera_info_stamp);
    config.maximum_processing_latency_ms = declare_parameter<double>(
      "maximum_processing_latency_ms",
      config.maximum_processing_latency_ms);
    config.camera_stale_timeout_s = declare_parameter<double>(
      "camera_stale_timeout_s", config.camera_stale_timeout_s);
    config.diagnostics_rate_hz = declare_parameter<double>(
      "diagnostics_rate_hz", config.diagnostics_rate_hz);

    config.position_variance_m2 = declare_parameter<double>(
      "position_variance_m2", config.position_variance_m2);
    config.orientation_variance_rad2 = declare_parameter<double>(
      "orientation_variance_rad2", config.orientation_variance_rad2);

    return config;
  }

  void initialize_detector()
  {
    if (!config_.enabled) {
      return;
    }

    tag_family_ = tag36h11_create();
    detector_ = apriltag_detector_create();
    if (tag_family_ == nullptr || detector_ == nullptr) {
      destroy_detector();
      throw std::runtime_error("apriltag_detector_initialization_failed");
    }

    apriltag_detector_add_family_bits(
      detector_, tag_family_, config_.maximum_hamming_distance);
    detector_->nthreads = config_.detector_threads;
    detector_->quad_decimate = static_cast<float>(config_.quad_decimate);
    detector_->quad_sigma = static_cast<float>(config_.quad_sigma);
    detector_->refine_edges = config_.refine_edges;
    detector_->decode_sharpening = config_.decode_sharpening;
    detector_->debug = false;
  }

  void destroy_detector()
  {
    if (detector_ != nullptr) {
      apriltag_detector_destroy(detector_);
      detector_ = nullptr;
    }
    if (tag_family_ != nullptr) {
      tag36h11_destroy(tag_family_);
      tag_family_ = nullptr;
    }
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    if (!config_.enabled) {
      return;
    }

    frames_received_.fetch_add(1U, std::memory_order_relaxed);
    last_image_receive_ns_.store(now().nanoseconds(), std::memory_order_relaxed);

    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      if (pending_image_ != nullptr) {
        frames_replaced_.fetch_add(1U, std::memory_order_relaxed);
      }
      pending_image_ = message;
    }
    worker_condition_.notify_one();
  }

  void on_camera_info(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr message)
  {
    const bool basic_valid = calibrated_camera_info(*message) &&
      (message->header.frame_id.empty() ||
      message->header.frame_id == config_.camera_optical_frame);

    {
      std::lock_guard<std::mutex> lock(camera_info_mutex_);
      latest_camera_info_ = message;
    }
    latest_camera_info_basic_valid_.store(
      basic_valid, std::memory_order_relaxed);
    camera_info_received_.fetch_add(1U, std::memory_order_relaxed);
  }

  void worker_loop()
  {
    while (true) {
      sensor_msgs::msg::Image::ConstSharedPtr image;
      {
        std::unique_lock<std::mutex> lock(worker_mutex_);
        worker_condition_.wait(
          lock,
          [this]() {return stopping_ || pending_image_ != nullptr;});

        if (stopping_) {
          return;
        }

        image = std::move(pending_image_);
        pending_image_.reset();
      }

      if (image != nullptr) {
        try {
          process_image(image);
        } catch (const std::exception & exception) {
          detector_failures_.fetch_add(1U, std::memory_order_relaxed);
          set_last_issue(
            std::string("apriltag_processing_exception: ") + exception.what());
        } catch (...) {
          detector_failures_.fetch_add(1U, std::memory_order_relaxed);
          set_last_issue("apriltag_processing_unknown_exception");
        }
      }
    }
  }

  void process_image(const sensor_msgs::msg::Image::ConstSharedPtr & image)
  {
    const auto start = std::chrono::steady_clock::now();

    if (!valid_image_buffer(*image)) {
      malformed_images_.fetch_add(1U, std::memory_order_relaxed);
      set_last_issue("malformed_camera_image");
      finish_frame(start, 0);
      return;
    }

    if (!image->header.frame_id.empty() &&
      image->header.frame_id != config_.camera_optical_frame)
    {
      image_frame_mismatches_.fetch_add(1U, std::memory_order_relaxed);
      set_last_issue("camera_image_frame_mismatch");
      finish_frame(start, 0);
      return;
    }

    cv_bridge::CvImageConstPtr gray_image;
    try {
      gray_image = cv_bridge::toCvShare(
        image, sensor_msgs::image_encodings::MONO8);
    } catch (const cv_bridge::Exception & exception) {
      image_conversion_failures_.fetch_add(1U, std::memory_order_relaxed);
      set_last_issue(std::string("image_conversion_failed: ") + exception.what());
      finish_frame(start, 0);
      return;
    }

    if (gray_image == nullptr || gray_image->image.empty() ||
      gray_image->image.cols <= 0 || gray_image->image.rows <= 0 ||
      gray_image->image.data == nullptr ||
      gray_image->image.step > static_cast<std::size_t>(std::numeric_limits<int>::max()))
    {
      malformed_images_.fetch_add(1U, std::memory_order_relaxed);
      set_last_issue("invalid_mono8_image");
      finish_frame(start, 0);
      return;
    }

    image_u8_t image_header{
      static_cast<std::int32_t>(gray_image->image.cols),
      static_cast<std::int32_t>(gray_image->image.rows),
      static_cast<std::int32_t>(gray_image->image.step),
      gray_image->image.data};

    using DetectionsPtr = std::unique_ptr<
      zarray_t, decltype(&apriltag_detections_destroy)>;
    DetectionsPtr detections(
      apriltag_detector_detect(detector_, &image_header),
      &apriltag_detections_destroy);
    if (detections == nullptr) {
      detector_failures_.fetch_add(1U, std::memory_order_relaxed);
      set_last_issue("apriltag_detector_returned_null");
      finish_frame(start, 0);
      return;
    }

    const int detection_count = zarray_size(detections.get());
    detections_seen_.fetch_add(
      static_cast<std::uint64_t>(std::max(0, detection_count)),
      std::memory_order_relaxed);

    const auto camera_info = camera_info_snapshot();
    for (int index = 0; index < detection_count; ++index) {
      apriltag_detection_t * detection = nullptr;
      zarray_get(detections.get(), index, &detection);
      if (detection == nullptr || !finite_detection(*detection)) {
        malformed_detections_.fetch_add(1U, std::memory_order_relaxed);
        continue;
      }

      if (detection->family != tag_family_ ||
        detection->hamming > config_.maximum_hamming_distance ||
        static_cast<double>(detection->decision_margin) <
        config_.minimum_decision_margin ||
        !is_allowed_apriltag_id(config_, detection->id))
      {
        filtered_detections_.fetch_add(1U, std::memory_order_relaxed);
        continue;
      }

      Observation observation;
      observation.header.stamp = image->header.stamp;
      observation.header.frame_id = config_.camera_optical_frame;
      observation.detector_name = config_.detector_name;
      observation.family = config_.family;
      observation.tag_id = detection->id;
      observation.observation_sequence =
        observation_sequence_.fetch_add(1U, std::memory_order_relaxed) + 1U;
      observation.image_width = image->width;
      observation.image_height = image->height;

      for (std::size_t corner = 0; corner < 4U; ++corner) {
        observation.image_corners_xy[corner * 2U] =
          static_cast<float>(detection->p[corner][0]);
        observation.image_corners_xy[corner * 2U + 1U] =
          static_cast<float>(detection->p[corner][1]);
      }

      observation.decision_margin = detection->decision_margin;
      observation.detection_quality = static_cast<float>(
        normalized_detection_quality(
          detection->decision_margin,
          config_.full_quality_decision_margin));
      observation.hamming_distance = static_cast<std::uint8_t>(
        std::clamp(detection->hamming, 0, 255));

      observation.pose_valid = estimate_pose(
        *detection, *image, camera_info, observation);
      observation.tag_size_m =
        observation.pose_valid ? config_.tag_size_m : 0.0;

      observation_publisher_->publish(observation);
      observations_published_.fetch_add(1U, std::memory_order_relaxed);
    }

    clear_last_issue();
    finish_frame(start, detection_count);
  }

  [[nodiscard]] sensor_msgs::msg::CameraInfo::ConstSharedPtr
  camera_info_snapshot() const
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    return latest_camera_info_;
  }

  [[nodiscard]] bool estimate_pose(
    const apriltag_detection_t & detection,
    const sensor_msgs::msg::Image & image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
    Observation & observation)
  {
    if (!config_.pose_estimation_enabled) {
      pose_unavailable_.fetch_add(1U, std::memory_order_relaxed);
      return false;
    }

    if (camera_info == nullptr) {
      missing_camera_info_.fetch_add(1U, std::memory_order_relaxed);
      pose_unavailable_.fetch_add(1U, std::memory_order_relaxed);
      return false;
    }

    if (!camera_info_usable(*camera_info, image)) {
      invalid_camera_info_.fetch_add(1U, std::memory_order_relaxed);
      pose_unavailable_.fetch_add(1U, std::memory_order_relaxed);
      return false;
    }

    apriltag_detection_info_t detection_info;
    detection_info.det = const_cast<apriltag_detection_t *>(&detection);
    detection_info.tagsize = config_.tag_size_m;
    detection_info.fx = camera_info->k[0];
    detection_info.fy = camera_info->k[4];
    detection_info.cx = camera_info->k[2];
    detection_info.cy = camera_info->k[5];

    apriltag_pose_t estimated_pose{};
    const double pose_error = estimate_tag_pose(&detection_info, &estimated_pose);

    const bool matrices_available =
      estimated_pose.R != nullptr && estimated_pose.t != nullptr;
    if (!matrices_available || !std::isfinite(pose_error)) {
      destroy_pose_matrices(estimated_pose);
      pose_estimation_failures_.fetch_add(1U, std::memory_order_relaxed);
      pose_unavailable_.fetch_add(1U, std::memory_order_relaxed);
      return false;
    }

    const std::array<double, 3> translation{
      MATD_EL(estimated_pose.t, 0, 0),
      MATD_EL(estimated_pose.t, 1, 0),
      MATD_EL(estimated_pose.t, 2, 0)};

    tf2::Matrix3x3 rotation(
      MATD_EL(estimated_pose.R, 0, 0),
      MATD_EL(estimated_pose.R, 0, 1),
      MATD_EL(estimated_pose.R, 0, 2),
      MATD_EL(estimated_pose.R, 1, 0),
      MATD_EL(estimated_pose.R, 1, 1),
      MATD_EL(estimated_pose.R, 1, 2),
      MATD_EL(estimated_pose.R, 2, 0),
      MATD_EL(estimated_pose.R, 2, 1),
      MATD_EL(estimated_pose.R, 2, 2));

    destroy_pose_matrices(estimated_pose);

    if (!std::all_of(
        translation.begin(), translation.end(),
        [](const double value) {return std::isfinite(value);}))
    {
      pose_estimation_failures_.fetch_add(1U, std::memory_order_relaxed);
      pose_unavailable_.fetch_add(1U, std::memory_order_relaxed);
      return false;
    }

    const double distance_m = std::sqrt(
      translation[0] * translation[0] +
      translation[1] * translation[1] +
      translation[2] * translation[2]);
    if (!std::isfinite(distance_m) || distance_m <= 0.0 ||
      distance_m > config_.maximum_detection_distance_m)
    {
      pose_distance_rejections_.fetch_add(1U, std::memory_order_relaxed);
      pose_unavailable_.fetch_add(1U, std::memory_order_relaxed);
      return false;
    }

    tf2::Quaternion orientation;
    rotation.getRotation(orientation);
    if (!std::isfinite(orientation.x()) || !std::isfinite(orientation.y()) ||
      !std::isfinite(orientation.z()) || !std::isfinite(orientation.w()) ||
      orientation.length2() <= std::numeric_limits<double>::epsilon())
    {
      pose_estimation_failures_.fetch_add(1U, std::memory_order_relaxed);
      pose_unavailable_.fetch_add(1U, std::memory_order_relaxed);
      return false;
    }
    orientation.normalize();

    observation.pose.pose.position.x = translation[0];
    observation.pose.pose.position.y = translation[1];
    observation.pose.pose.position.z = translation[2];
    observation.pose.pose.orientation.x = orientation.x();
    observation.pose.pose.orientation.y = orientation.y();
    observation.pose.pose.orientation.z = orientation.z();
    observation.pose.pose.orientation.w = orientation.w();

    observation.pose.covariance.fill(0.0);
    observation.pose.covariance[0] = config_.position_variance_m2;
    observation.pose.covariance[7] = config_.position_variance_m2;
    observation.pose.covariance[14] = config_.position_variance_m2;
    observation.pose.covariance[21] = config_.orientation_variance_rad2;
    observation.pose.covariance[28] = config_.orientation_variance_rad2;
    observation.pose.covariance[35] = config_.orientation_variance_rad2;
    observation.pose_error = static_cast<float>(pose_error);

    pose_successes_.fetch_add(1U, std::memory_order_relaxed);
    return true;
  }

  [[nodiscard]] bool camera_info_usable(
    const sensor_msgs::msg::CameraInfo & camera_info,
    const sensor_msgs::msg::Image & image) const
  {
    if (!calibrated_camera_info(camera_info)) {
      return false;
    }

    if ((camera_info.width != 0U && camera_info.width != image.width) ||
      (camera_info.height != 0U && camera_info.height != image.height))
    {
      return false;
    }

    if (!camera_info.header.frame_id.empty() &&
      camera_info.header.frame_id != config_.camera_optical_frame)
    {
      return false;
    }

    const auto image_stamp_ns = rclcpp::Time(image.header.stamp).nanoseconds();
    const auto info_stamp_ns = rclcpp::Time(camera_info.header.stamp).nanoseconds();

    if (config_.require_matching_camera_info_stamp &&
      image_stamp_ns != info_stamp_ns)
    {
      return false;
    }

    if (image_stamp_ns != 0 && info_stamp_ns != 0 &&
      absolute_stamp_difference_seconds(
        image.header.stamp, camera_info.header.stamp) >
      config_.camera_info_max_age_s)
    {
      return false;
    }

    return true;
  }

  static void destroy_matrix(matd_t *& matrix)
  {
    if (matrix == nullptr) {
      return;
    }


    std::free(matrix);
    matrix = nullptr;
  }

  static void destroy_pose_matrices(apriltag_pose_t & pose)
  {
    destroy_matrix(pose.R);
    destroy_matrix(pose.t);
  }

  void finish_frame(
    const std::chrono::steady_clock::time_point & start,
    const int detection_count)
  {
    const auto elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();

    frames_processed_.fetch_add(1U, std::memory_order_relaxed);
    last_detection_count_.store(detection_count, std::memory_order_relaxed);
    last_processing_latency_ms_.store(elapsed, std::memory_order_relaxed);
    if (elapsed > config_.maximum_processing_latency_ms) {
      latency_exceeded_.fetch_add(1U, std::memory_order_relaxed);
    }
  }

  void set_last_issue(const std::string & issue)
  {
    std::lock_guard<std::mutex> lock(issue_mutex_);
    last_issue_ = issue;
  }

  void clear_last_issue()
  {
    std::lock_guard<std::mutex> lock(issue_mutex_);
    last_issue_.clear();
  }

  [[nodiscard]] std::string last_issue_snapshot() const
  {
    std::lock_guard<std::mutex> lock(issue_mutex_);
    return last_issue_;
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    DiagnosticStatus status;
    status.name = "savo_head/apriltag_detector";
    status.hardware_id = "pi_camera_v2_noir";

    const auto frames_received =
      frames_received_.load(std::memory_order_relaxed);
    const auto last_receive_ns =
      last_image_receive_ns_.load(std::memory_order_relaxed);
    const auto processing_latency =
      last_processing_latency_ms_.load(std::memory_order_relaxed);
    const auto issue = last_issue_snapshot();

    if (!config_.enabled) {
      status.level = DiagnosticStatus::OK;
      status.message = "disabled_by_configuration";
    } else if (frames_received == 0U) {
      status.level = DiagnosticStatus::WARN;
      status.message = "waiting_for_camera_image";
    } else {
      const double image_age_s = static_cast<double>(
        std::max<std::int64_t>(0, now().nanoseconds() - last_receive_ns)) /
        kNanosecondsPerSecond;

      if (image_age_s > config_.camera_stale_timeout_s) {
        status.level = DiagnosticStatus::ERROR;
        status.message = "camera_stream_stale";
      } else if (!issue.empty()) {
        status.level = DiagnosticStatus::WARN;
        status.message = issue;
      } else if (processing_latency > config_.maximum_processing_latency_ms) {
        status.level = DiagnosticStatus::WARN;
        status.message = "processing_latency_exceeded";
      } else if (config_.pose_estimation_enabled &&
        camera_info_received_.load(std::memory_order_relaxed) == 0U)
      {
        status.level = DiagnosticStatus::WARN;
        status.message = "id_detection_ready_pose_waiting_for_camera_info";
      } else if (config_.pose_estimation_enabled &&
        !latest_camera_info_basic_valid_.load(std::memory_order_relaxed))
      {
        status.level = DiagnosticStatus::WARN;
        status.message = "id_detection_ready_camera_info_invalid_for_pose";
      } else if (last_detection_count_.load(std::memory_order_relaxed) == 0) {
        status.level = DiagnosticStatus::OK;
        status.message = "running_no_tag_visible";
      } else {
        status.level = DiagnosticStatus::OK;
        status.message = "running";
      }
    }

    status.values.reserve(28U);
    status.values.push_back(make_key_value("enabled", config_.enabled));
    status.values.push_back(make_key_value("family", config_.family));
    status.values.push_back(make_key_value("tag_size_m", config_.tag_size_m));
    status.values.push_back(make_key_value(
      "pose_estimation_enabled", config_.pose_estimation_enabled));
    status.values.push_back(make_key_value(
      "camera_optical_frame", config_.camera_optical_frame));
    status.values.push_back(make_key_value(
      "frames_received", frames_received));
    status.values.push_back(make_key_value(
      "frames_replaced", frames_replaced_.load(std::memory_order_relaxed)));
    status.values.push_back(make_key_value(
      "frames_processed", frames_processed_.load(std::memory_order_relaxed)));
    status.values.push_back(make_key_value(
      "malformed_images", malformed_images_.load(std::memory_order_relaxed)));
    status.values.push_back(make_key_value(
      "image_conversion_failures",
      image_conversion_failures_.load(std::memory_order_relaxed)));
    status.values.push_back(make_key_value(
      "image_frame_mismatches",
      image_frame_mismatches_.load(std::memory_order_relaxed)));
    status.values.push_back(make_key_value(
      "detector_failures", detector_failures_.load(std::memory_order_relaxed)));
    status.values.push_back(make_key_value(
      "detections_seen", detections_seen_.load(std::memory_order_relaxed)));
    status.values.push_back(make_key_value(
      "filtered_detections",
      filtered_detections_.load(std::memory_order_relaxed)));
    status.values.push_back(make_key_value(
      "malformed_detections",
      malformed_detections_.load(std::memory_order_relaxed)));
    status.values.push_back(make_key_value(
      "observations_published",
      observations_published_.load(std::memory_order_relaxed)));
    status.values.push_back(make_key_value(
      "camera_info_received",
      camera_info_received_.load(std::memory_order_relaxed)));
    status.values.push_back(make_key_value(
      "missing_camera_info",
      missing_camera_info_.load(std::memory_order_relaxed)));
    status.values.push_back(make_key_value(
      "invalid_camera_info",
      invalid_camera_info_.load(std::memory_order_relaxed)));
    status.values.push_back(make_key_value(
      "pose_successes", pose_successes_.load(std::memory_order_relaxed)));
    status.values.push_back(make_key_value(
      "pose_unavailable", pose_unavailable_.load(std::memory_order_relaxed)));
    status.values.push_back(make_key_value(
      "pose_estimation_failures",
      pose_estimation_failures_.load(std::memory_order_relaxed)));
    status.values.push_back(make_key_value(
      "pose_distance_rejections",
      pose_distance_rejections_.load(std::memory_order_relaxed)));
    status.values.push_back(make_key_value(
      "last_detection_count",
      static_cast<std::int64_t>(
          last_detection_count_.load(std::memory_order_relaxed))));
    status.values.push_back(make_key_value(
      "last_processing_latency_ms", processing_latency));
    status.values.push_back(make_key_value(
      "latency_exceeded",
      latency_exceeded_.load(std::memory_order_relaxed)));
    status.values.push_back(make_key_value(
      "maximum_processing_latency_ms",
      config_.maximum_processing_latency_ms));
    status.values.push_back(make_key_value(
      "maximum_detection_distance_m",
      config_.maximum_detection_distance_m));
    status.values.push_back(make_key_value("last_issue", issue));

    array.status.push_back(std::move(status));
    diagnostics_publisher_->publish(array);
  }

  AprilTagDetectorConfig config_{};
  apriltag_detector_t * detector_{nullptr};
  apriltag_family_t * tag_family_{nullptr};

  rclcpp::Publisher<Observation>::SharedPtr observation_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
    camera_info_subscription_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  mutable std::mutex camera_info_mutex_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info_;

  std::mutex worker_mutex_;
  std::condition_variable worker_condition_;
  sensor_msgs::msg::Image::ConstSharedPtr pending_image_;
  bool stopping_{false};
  std::thread worker_thread_;

  mutable std::mutex issue_mutex_;
  std::string last_issue_;

  std::atomic<std::uint64_t> observation_sequence_{0U};
  std::atomic<std::uint64_t> frames_received_{0U};
  std::atomic<std::uint64_t> frames_replaced_{0U};
  std::atomic<std::uint64_t> frames_processed_{0U};
  std::atomic<std::uint64_t> malformed_images_{0U};
  std::atomic<std::uint64_t> image_conversion_failures_{0U};
  std::atomic<std::uint64_t> image_frame_mismatches_{0U};
  std::atomic<std::uint64_t> detector_failures_{0U};
  std::atomic<std::uint64_t> detections_seen_{0U};
  std::atomic<std::uint64_t> filtered_detections_{0U};
  std::atomic<std::uint64_t> malformed_detections_{0U};
  std::atomic<std::uint64_t> observations_published_{0U};
  std::atomic<std::uint64_t> camera_info_received_{0U};
  std::atomic<bool> latest_camera_info_basic_valid_{false};
  std::atomic<std::uint64_t> missing_camera_info_{0U};
  std::atomic<std::uint64_t> invalid_camera_info_{0U};
  std::atomic<std::uint64_t> pose_successes_{0U};
  std::atomic<std::uint64_t> pose_unavailable_{0U};
  std::atomic<std::uint64_t> pose_estimation_failures_{0U};
  std::atomic<std::uint64_t> pose_distance_rejections_{0U};
  std::atomic<std::uint64_t> latency_exceeded_{0U};
  std::atomic<std::int64_t> last_image_receive_ns_{0};
  std::atomic<int> last_detection_count_{0};
  std::atomic<double> last_processing_latency_ms_{0.0};
};

}  // namespace savo_head

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<savo_head::AprilTagDetectorNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("apriltag_detector_node"),
      "AprilTag detector startup/runtime failure: %s",
      exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
