#include "savo_mapping/coverage_grid.hpp"
#include "savo_mapping/coverage_planner.hpp"
#include "savo_mapping/qos_profiles.hpp"
#include "savo_mapping/slam_contract.hpp"
#include "savo_mapping/tf_pose_reader.hpp"
#include "savo_mapping/topic_names.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <functional>
#include <future>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>

namespace savo_mapping
{
namespace
{

constexpr double kMinimumQuaternionNorm = 1.0e-12;
constexpr double kPlanarOrientationTolerance = 1.0e-12;
constexpr double kMinimumTickPeriodSec = 0.01;
constexpr double kMaximumTickPeriodSec = 1.0;
constexpr double kMaximumTfLookupTimeoutSec = 2.0;
constexpr std::int64_t kMaximumWaypointLimit = 100000;
constexpr std::size_t kMaximumMapCellCount = 1048576;
constexpr long double kMaximumInflationCellChecks = 250000000.0L;

bool text_is_blank(const std::string & value)
{
  return value.empty() ||
         std::all_of(
    value.begin(),
    value.end(),
    [](const unsigned char character) {
      return std::isspace(character) != 0;
    });
}

std::string json_escape(const std::string & input)
{
  std::ostringstream output;
  for (const char character : input) {
    switch (character) {
      case '\\':
        output << "\\\\";
        break;
      case '"':
        output << "\\\"";
        break;
      case '\n':
        output << "\\n";
        break;
      case '\r':
        output << "\\r";
        break;
      case '\t':
        output << "\\t";
        break;
      default:
        output << character;
        break;
    }
  }
  return output.str();
}

const char * bool_text(const bool value)
{
  return value ? "true" : "false";
}

coverage::Connectivity parse_connectivity(
  const std::string & value)
{
  if (value == "four") {
    return coverage::Connectivity::Four;
  }
  if (value == "eight") {
    return coverage::Connectivity::Eight;
  }
  throw std::invalid_argument(
          "coverage_node_connectivity_invalid");
}

coverage::SweepAxis parse_sweep_axis(
  const std::string & value)
{
  if (value == "automatic") {
    return coverage::SweepAxis::Automatic;
  }
  if (value == "rows") {
    return coverage::SweepAxis::Rows;
  }
  if (value == "columns") {
    return coverage::SweepAxis::Columns;
  }
  throw std::invalid_argument(
          "coverage_node_sweep_axis_invalid");
}

bool grids_are_equivalent(
  const coverage::CoverageGrid & left,
  const coverage::CoverageGrid & right)
{
  const auto & left_metadata = left.metadata();
  const auto & right_metadata = right.metadata();
  return
    left_metadata.width == right_metadata.width &&
    left_metadata.height == right_metadata.height &&
    left_metadata.resolution_m == right_metadata.resolution_m &&
    left_metadata.origin_x_m == right_metadata.origin_x_m &&
    left_metadata.origin_y_m == right_metadata.origin_y_m &&
    left_metadata.origin_yaw_rad == right_metadata.origin_yaw_rad &&
    left.occupancy() == right.occupancy();
}

struct MapConversionResult
{
  std::shared_ptr<coverage::CoverageGrid> grid;
  std::string reason;
  std::string detail;
};

MapConversionResult map_failure(
  std::string reason,
  std::string detail)
{
  return {
    nullptr,
    std::move(reason),
    std::move(detail)
  };
}

struct PendingMapInput
{
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr message;
  std::uint64_t input_generation{0};
  std::chrono::steady_clock::time_point received_at;
};

struct MapWorkerResult
{
  std::uint64_t input_generation{0};
  std::chrono::steady_clock::time_point received_at;
  bool changed{false};
  MapConversionResult conversion;
};

struct PlanWorkerResult
{
  std::uint64_t input_generation{0};
  std::uint64_t map_sequence{0};
  TfPoseSnapshot pose;
  bool plan_attempted{false};
  coverage::CoveragePlanResult plan_result;
  std::optional<nav_msgs::msg::Path> path;
  std::string exception;
};

using WorkerResult = std::variant<MapWorkerResult, PlanWorkerResult>;

nav_msgs::msg::Path make_path(
  const coverage::CoveragePlan & plan,
  const std::string & map_frame,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = map_frame;
  path.header.stamp = stamp;
  path.poses.reserve(plan.waypoints.size());

  for (const auto & waypoint : plan.waypoints) {
    if (!std::isfinite(waypoint.position.x_m) ||
      !std::isfinite(waypoint.position.y_m) ||
      !std::isfinite(waypoint.yaw_rad))
    {
      throw std::runtime_error(
              "coverage_node_waypoint_invalid");
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = waypoint.position.x_m;
    pose.pose.position.y = waypoint.position.y_m;
    pose.pose.position.z = 0.0;

    const double half_yaw = waypoint.yaw_rad * 0.5;
    pose.pose.orientation.z = std::sin(half_yaw);
    pose.pose.orientation.w = std::cos(half_yaw);
    const double orientation_norm =
      std::hypot(
      pose.pose.orientation.z,
      pose.pose.orientation.w);
    if (!std::isfinite(orientation_norm) ||
      std::abs(orientation_norm - 1.0) >
      kMinimumQuaternionNorm)
    {
      throw std::runtime_error(
              "coverage_node_waypoint_orientation_invalid");
    }
    path.poses.push_back(std::move(pose));
  }

  if (path.poses.empty()) {
    throw std::runtime_error(
            "coverage_node_plan_empty");
  }
  return path;
}

}  // namespace

class CoverageMapperNode final : public rclcpp::Node
{
public:
  CoverageMapperNode()
  : Node("coverage_mapper_node")
  {
    declare_and_validate_parameters();
    create_interfaces();

    set_state(
      enabled_ ? "waiting_for_map" : "disabled",
      enabled_ ?
      "coverage_node_waiting_for_map" :
      "coverage_node_disabled");

    RCLCPP_INFO(
      get_logger(),
      "coverage mapper ready: enabled=%s auto_plan=%s",
      bool_text(enabled_),
      bool_text(auto_plan_));
  }

  ~CoverageMapperNode() override
  {
    if (worker_future_.valid()) {
      worker_future_.wait();
    }
  }

private:
  void declare_and_validate_parameters()
  {
    enabled_ = declare_parameter<bool>(
      "enabled", true);
    auto_plan_ = declare_parameter<bool>(
      "auto_plan", false);
    plan_once_ = declare_parameter<bool>(
      "plan_once", true);
    replan_on_map_update_ = declare_parameter<bool>(
      "replan_on_map_update", false);

    map_topic_ = declare_parameter<std::string>(
      "map_topic", std::string{topics::MAP});
    map_frame_ = declare_parameter<std::string>(
      "map_frame", std::string{slam::MAP_FRAME});
    base_frame_ = declare_parameter<std::string>(
      "base_frame", std::string{slam::BASE_FRAME});
    path_topic_ = declare_parameter<std::string>(
      "path_topic", "/savo_mapping/coverage/path");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/savo_mapping/coverage/status");
    state_topic_ = declare_parameter<std::string>(
      "state_topic", "/savo_mapping/coverage/state");

    tick_period_sec_ = declare_parameter<double>(
      "tick_period_sec", 0.10);
    map_stale_timeout_sec_ = declare_parameter<double>(
      "map_stale_timeout_sec", 5.0);
    tf_lookup_timeout_sec_ = declare_parameter<double>(
      "tf_lookup_timeout_sec", 0.20);
    tf_stale_timeout_sec_ = declare_parameter<double>(
      "tf_stale_timeout_sec", 1.0);

    const auto free_threshold =
      declare_parameter<std::int64_t>(
      "free_threshold", 0);
    const auto occupied_threshold =
      declare_parameter<std::int64_t>(
      "occupied_threshold", 65);
    grid_options_.allow_unknown =
      declare_parameter<bool>(
      "allow_unknown", false);
    grid_options_.inflation_radius_m =
      declare_parameter<double>(
      "inflation_radius_m", 0.0);

    const auto connectivity =
      declare_parameter<std::string>(
      "connectivity", "four");
    const auto sweep_axis =
      declare_parameter<std::string>(
      "sweep_axis", "automatic");
    planner_options_.track_spacing_m =
      declare_parameter<double>(
      "track_spacing_m", 0.5);
    planner_options_.minimum_segment_length_m =
      declare_parameter<double>(
      "minimum_segment_length_m", 0.0);
    const auto maximum_waypoints =
      declare_parameter<std::int64_t>(
      "maximum_waypoints", 10000);

    const std::array<std::pair<const char *, const std::string *>, 6>
    text_parameters{{
      {"map_topic", &map_topic_},
      {"map_frame", &map_frame_},
      {"base_frame", &base_frame_},
      {"path_topic", &path_topic_},
      {"status_topic", &status_topic_},
      {"state_topic", &state_topic_},
    }};
    for (const auto & [name, value] : text_parameters) {
      if (text_is_blank(*value)) {
        throw std::invalid_argument(
                std::string{"coverage_node_parameter_empty:"} +
                name);
      }
    }

    if (!std::isfinite(tick_period_sec_) ||
      tick_period_sec_ < kMinimumTickPeriodSec ||
      tick_period_sec_ > kMaximumTickPeriodSec)
    {
      throw std::invalid_argument(
              "coverage_node_tick_period_invalid");
    }
    if (!std::isfinite(map_stale_timeout_sec_) ||
      map_stale_timeout_sec_ < 0.0)
    {
      throw std::invalid_argument(
              "coverage_node_map_stale_timeout_invalid");
    }
    if (!std::isfinite(tf_lookup_timeout_sec_) ||
      tf_lookup_timeout_sec_ <= 0.0 ||
      tf_lookup_timeout_sec_ >
      kMaximumTfLookupTimeoutSec)
    {
      throw std::invalid_argument(
              "coverage_node_tf_lookup_timeout_invalid");
    }
    if (!std::isfinite(tf_stale_timeout_sec_) ||
      tf_stale_timeout_sec_ < 0.0)
    {
      throw std::invalid_argument(
              "coverage_node_tf_stale_timeout_invalid");
    }
    if (free_threshold < 0 || free_threshold > 100 ||
      occupied_threshold < 0 || occupied_threshold > 100 ||
      free_threshold >= occupied_threshold)
    {
      throw std::invalid_argument(
              "coverage_node_occupancy_thresholds_invalid");
    }
    if (maximum_waypoints <= 0 ||
      maximum_waypoints > kMaximumWaypointLimit)
    {
      throw std::invalid_argument(
              "coverage_node_maximum_waypoints_invalid");
    }

    grid_options_.free_threshold =
      static_cast<std::int8_t>(free_threshold);
    grid_options_.occupied_threshold =
      static_cast<std::int8_t>(occupied_threshold);
    planner_options_.connectivity =
      parse_connectivity(connectivity);
    planner_options_.sweep_axis =
      parse_sweep_axis(sweep_axis);
    planner_options_.maximum_waypoint_count =
      static_cast<std::size_t>(maximum_waypoints);

    const auto grid_options_error =
      coverage::validate_coverage_grid_options(
      grid_options_);
    if (!grid_options_error.empty()) {
      throw std::invalid_argument(grid_options_error);
    }

    const auto planner_options_error =
      coverage::validate_coverage_planner_options(
      planner_options_);
    if (!planner_options_error.empty()) {
      throw std::invalid_argument(planner_options_error);
    }

    planner_ = std::make_unique<coverage::CoveragePlanner>(
      planner_options_);
  }

  void create_interfaces()
  {
    path_publisher_ =
      create_publisher<nav_msgs::msg::Path>(
      path_topic_, qos::state_qos());
    status_publisher_ =
      create_publisher<std_msgs::msg::String>(
      status_topic_, qos::status_qos());
    state_publisher_ =
      create_publisher<std_msgs::msg::String>(
      state_topic_, qos::state_qos());

    map_subscription_ =
      create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_,
      qos::map_qos(),
      std::bind(
        &CoverageMapperNode::handle_map,
        this,
        std::placeholders::_1));

    tf_buffer_ =
      std::make_shared<tf2_ros::Buffer>(
      get_clock());
    tf_listener_ =
      std::make_unique<tf2_ros::TransformListener>(
      *tf_buffer_);
    tf_pose_reader_ =
      std::make_unique<TfPoseReader>(
      get_clock(),
      tf_buffer_,
      TfPoseReaderOptions{
        map_frame_,
        base_frame_,
        tf_lookup_timeout_sec_,
        tf_stale_timeout_sec_
      });

    const auto tick_period =
      std::chrono::duration_cast<
      std::chrono::nanoseconds>(
      std::chrono::duration<double>(
        tick_period_sec_));
    tick_timer_ =
      create_wall_timer(
      tick_period,
      std::bind(
        &CoverageMapperNode::planning_tick,
        this));
  }

  MapConversionResult convert_map(
    const nav_msgs::msg::OccupancyGrid & message) const
  {
    if (message.header.frame_id != map_frame_) {
      return map_failure(
        "coverage_node_map_frame_mismatch",
        message.header.frame_id.empty() ?
        "map_frame_empty" :
        message.header.frame_id);
    }

    const auto width =
      static_cast<std::size_t>(message.info.width);
    const auto height =
      static_cast<std::size_t>(message.info.height);
    if (height != 0 &&
      width >
      std::numeric_limits<std::size_t>::max() /
      height)
    {
      return map_failure(
        "coverage_node_map_invalid",
        "coverage_grid_dimensions_invalid");
    }
    const auto expected_size = width * height;
    if (message.data.size() != expected_size) {
      return map_failure(
        "coverage_node_map_invalid",
        "coverage_grid_data_size_mismatch");
    }
    if (expected_size > kMaximumMapCellCount) {
      return map_failure(
        "coverage_node_map_invalid",
        "coverage_node_map_work_limit_exceeded");
    }
    if (std::isfinite(message.info.resolution) &&
      message.info.resolution > 0.0 &&
      grid_options_.inflation_radius_m > 0.0)
    {
      const long double radius_cells =
        std::ceil(
        static_cast<long double>(
          grid_options_.inflation_radius_m) /
        static_cast<long double>(
          message.info.resolution));
      const long double kernel_width =
        (2.0L * radius_cells) + 1.0L;
      const long double inflation_checks =
        static_cast<long double>(expected_size) *
        kernel_width * kernel_width;
      if (!std::isfinite(inflation_checks) ||
        inflation_checks > kMaximumInflationCellChecks)
      {
        return map_failure(
          "coverage_node_map_invalid",
          "coverage_node_map_work_limit_exceeded");
      }
    }

    const auto & origin = message.info.origin;
    const auto & orientation = origin.orientation;
    if (!std::isfinite(origin.position.x) ||
      !std::isfinite(origin.position.y) ||
      !std::isfinite(origin.position.z) ||
      !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) ||
      !std::isfinite(orientation.w))
    {
      return map_failure(
        "coverage_node_map_invalid",
        "coverage_grid_origin_invalid");
    }

    const double quaternion_norm =
      std::hypot(
      std::hypot(orientation.x, orientation.y),
      std::hypot(orientation.z, orientation.w));
    if (!std::isfinite(quaternion_norm) ||
      quaternion_norm <= kMinimumQuaternionNorm)
    {
      return map_failure(
        "coverage_node_map_invalid",
        "coverage_grid_origin_invalid");
    }

    const double qx = orientation.x / quaternion_norm;
    const double qy = orientation.y / quaternion_norm;
    const double qz = orientation.z / quaternion_norm;
    const double qw = orientation.w / quaternion_norm;
    const double roll = std::atan2(
      2.0 * ((qw * qx) + (qy * qz)),
      1.0 - (2.0 * ((qx * qx) + (qy * qy))));
    const double pitch_term = std::clamp(
      2.0 * ((qw * qy) - (qz * qx)),
      -1.0,
      1.0);
    const double pitch = std::asin(pitch_term);
    const double yaw = std::atan2(
      2.0 * ((qw * qz) + (qx * qy)),
      1.0 - (2.0 * ((qy * qy) + (qz * qz))));
    if (!std::isfinite(roll) ||
      !std::isfinite(pitch) ||
      !std::isfinite(yaw) ||
      std::abs(roll) > kPlanarOrientationTolerance ||
      std::abs(pitch) > kPlanarOrientationTolerance)
    {
      return map_failure(
        "coverage_node_map_invalid",
        "coverage_grid_origin_orientation_unsupported");
    }

    try {
      auto grid =
        std::make_shared<coverage::CoverageGrid>(
        coverage::CoverageGridMetadata{
          width,
          height,
          static_cast<double>(
            message.info.resolution),
          origin.position.x,
          origin.position.y,
          yaw,
          message.data.size()
        },
        message.data,
        grid_options_);
      return {
        std::move(grid),
        {},
        {}
      };
    } catch (const std::exception & error) {
      return map_failure(
        "coverage_node_map_invalid",
        error.what());
    }
  }

  void handle_map(
    const nav_msgs::msg::OccupancyGrid::SharedPtr message)
  {
    pending_map_input_ = PendingMapInput{
      message,
      ++map_input_generation_,
      std::chrono::steady_clock::now()
    };
    publish_status();
  }

  void start_pending_map_work()
  {
    if (worker_future_.valid() ||
      !pending_map_input_.has_value())
    {
      return;
    }

    auto input = std::move(*pending_map_input_);
    pending_map_input_.reset();
    const auto previous_grid = valid_grid_;
    worker_future_ = std::async(
      std::launch::async,
      [this, input = std::move(input), previous_grid]() mutable
      -> WorkerResult {
        auto conversion = convert_map(*input.message);
        const bool changed =
        conversion.grid &&
        (!previous_grid ||
        !grids_are_equivalent(
          *previous_grid,
          *conversion.grid));
        MapWorkerResult result;
        result.input_generation =
        input.input_generation;
        result.received_at = input.received_at;
        result.changed = changed;
        result.conversion = std::move(conversion);
        return result;
      });
  }

  void apply_map_worker_result(
    MapWorkerResult result)
  {
    if (result.input_generation !=
      map_input_generation_)
    {
      return;
    }
    validated_map_input_generation_ =
      result.input_generation;

    auto & converted = result.conversion;
    if (!converted.grid) {
      deferred_plan_result_.reset();
      latest_map_input_valid_ = false;
      last_map_reason_ = converted.reason;
      last_map_error_ = converted.detail;
      if (enabled_) {
        set_state(
          "map_invalid",
          converted.reason,
          converted.detail);
      } else {
        set_state(
          "disabled",
          "coverage_node_disabled");
      }
      return;
    }

    const bool previous_input_was_valid =
      latest_map_input_valid_;
    valid_grid_ = std::move(converted.grid);
    latest_map_input_valid_ = true;
    last_map_reason_.clear();
    last_map_error_.clear();
    map_received_at_ = result.received_at;
    if (result.changed) {
      deferred_plan_result_.reset();
      ++map_sequence_;
    }

    update_state_after_valid_map(
      result.changed,
      !previous_input_was_valid);
  }

  void update_state_after_valid_map(
    const bool changed,
    const bool restored_after_invalid_input)
  {
    if (!enabled_) {
      set_state(
        "disabled",
        "coverage_node_disabled");
      return;
    }
    if (!auto_plan_) {
      set_state(
        "ready",
        "coverage_node_auto_plan_disabled");
      return;
    }
    if (successful_plan_published_ &&
      (plan_once_ || !replan_on_map_update_))
    {
      set_state(
        "plan_ready",
        "coverage_node_plan_ready",
        "coverage_plan_ready");
      return;
    }
    if (!replan_on_map_update_ &&
      last_attempted_map_sequence_.has_value() &&
      !last_attempt_state_.empty())
    {
      set_state(
        last_attempt_state_,
        last_attempt_reason_,
        last_attempt_detail_);
      return;
    }
    if (!changed && !restored_after_invalid_input) {
      publish_status();
      return;
    }
    if (!changed &&
      last_attempted_map_sequence_.has_value() &&
      *last_attempted_map_sequence_ == map_sequence_ &&
      !last_attempt_state_.empty())
    {
      set_state(
        last_attempt_state_,
        last_attempt_reason_,
        last_attempt_detail_);
      return;
    }

    last_pose_ = TfPoseSnapshot{};
    set_state(
      "waiting_for_pose",
      "coverage_node_waiting_for_pose");
  }

  bool map_is_fresh() const
  {
    if (!latest_map_input_valid_ ||
      !valid_grid_ ||
      !map_received_at_.has_value())
    {
      return false;
    }
    if (map_stale_timeout_sec_ == 0.0) {
      return true;
    }
    const double age_sec =
      std::chrono::duration<double>(
      std::chrono::steady_clock::now() -
      *map_received_at_).count();
    return std::isfinite(age_sec) &&
           age_sec <= map_stale_timeout_sec_;
  }

  double map_age_sec() const
  {
    if (!map_received_at_.has_value()) {
      return 0.0;
    }
    return std::max(
      0.0,
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() -
        *map_received_at_).count());
  }

  bool should_attempt_current_map() const
  {
    if (!last_attempted_map_sequence_.has_value()) {
      return true;
    }
    if (*last_attempted_map_sequence_ == map_sequence_) {
      return false;
    }
    if (!replan_on_map_update_) {
      return false;
    }
    if (successful_plan_published_ && plan_once_) {
      return false;
    }
    return true;
  }

  void consume_worker_result()
  {
    if (!worker_future_.valid() ||
      worker_future_.wait_for(
        std::chrono::seconds(0)) !=
      std::future_status::ready)
    {
      return;
    }

    try {
      auto result = worker_future_.get();
      if (std::holds_alternative<
          MapWorkerResult>(result))
      {
        apply_map_worker_result(
          std::get<MapWorkerResult>(
            std::move(result)));
      } else {
        auto plan_result =
          std::get<PlanWorkerResult>(
          std::move(result));
        if (plan_result.map_sequence ==
          map_sequence_ &&
          (plan_result.input_generation !=
          map_input_generation_ ||
          pending_map_input_.has_value()))
        {
          deferred_plan_result_ =
            std::move(plan_result);
        } else {
          apply_plan_worker_result(
            std::move(plan_result),
            false);
        }
      }
    } catch (const std::exception & error) {
      terminal_ = true;
      set_state(
        "plan_failed",
        "coverage_node_worker_failed",
        error.what());
    } catch (...) {
      terminal_ = true;
      set_state(
        "plan_failed",
        "coverage_node_worker_failed",
        "coverage_node_worker_unknown_failure");
    }
  }

  void start_planning_work()
  {
    if (worker_future_.valid() ||
      !valid_grid_)
    {
      return;
    }

    const auto grid = valid_grid_;
    const auto input_generation =
      map_input_generation_;
    const auto map_sequence = map_sequence_;
    const auto reader = tf_pose_reader_.get();
    const auto planner = planner_.get();
    const auto clock = get_clock();
    const auto map_frame = map_frame_;

    set_state(
      "waiting_for_pose",
      "coverage_node_waiting_for_pose");
    worker_future_ = std::async(
      std::launch::async,
      [grid, input_generation, map_sequence,
      reader, planner, clock, map_frame]()
      -> WorkerResult {
        PlanWorkerResult result;
        result.input_generation = input_generation;
        result.map_sequence = map_sequence;
        try {
          result.pose = reader->read();
          if (!result.pose.valid ||
          !result.pose.fresh)
          {
            return result;
          }

          result.plan_attempted = true;
          result.plan_result =
          planner->plan_from_world(
            *grid,
            coverage::WorldPoint{
            result.pose.x_m,
            result.pose.y_m
            });
          if (result.plan_result.valid) {
            result.path = make_path(
              result.plan_result.plan,
              map_frame,
              clock->now());
          }
        } catch (const std::exception & error) {
          result.exception = error.what();
        } catch (...) {
          result.exception =
          "coverage_node_worker_unknown_failure";
        }
        return result;
      });
  }

  void apply_plan_worker_result(
    PlanWorkerResult result,
    const bool allow_semantic_rebase)
  {
    if ((!allow_semantic_rebase &&
      result.input_generation !=
      map_input_generation_) ||
      (allow_semantic_rebase &&
      validated_map_input_generation_ <
      result.input_generation) ||
      result.map_sequence != map_sequence_ ||
      pending_map_input_.has_value() ||
      !enabled_ ||
      !auto_plan_ ||
      !latest_map_input_valid_ ||
      !map_is_fresh() ||
      !should_attempt_current_map())
    {
      return;
    }

    last_pose_ = std::move(result.pose);
    if (!last_pose_.valid || !last_pose_.fresh) {
      handle_pose_failure();
      return;
    }
    if (!pose_is_currently_fresh()) {
      last_pose_.fresh = false;
      last_pose_.reason =
        "tf_pose_transform_stale";
      handle_pose_failure();
      return;
    }

    set_state(
      "planning",
      "coverage_node_planning");
    if (!result.exception.empty()) {
      last_attempted_map_sequence_ =
        map_sequence_;
      record_plan_failure(
        "coverage_node_plan_failed",
        result.exception);
      return;
    }
    if (!result.plan_attempted) {
      set_state(
        "pose_invalid",
        "coverage_node_pose_invalid");
      return;
    }

    last_attempted_map_sequence_ =
      map_sequence_;
    if (!result.plan_result.valid) {
      handle_plan_failure(
        result.plan_result.reason);
      return;
    }
    if (!result.path.has_value()) {
      record_plan_failure(
        "coverage_node_plan_failed",
        "coverage_node_plan_path_missing");
      return;
    }
    publish_plan(
      result.plan_result.plan,
      std::move(*result.path));
  }

  void planning_tick()
  {
    consume_worker_result();
    if (worker_future_.valid()) {
      publish_status();
      return;
    }
    if (pending_map_input_.has_value()) {
      start_pending_map_work();
      return;
    }
    if (deferred_plan_result_.has_value()) {
      auto result =
        std::move(*deferred_plan_result_);
      deferred_plan_result_.reset();
      apply_plan_worker_result(
        std::move(result),
        true);
      return;
    }

    if (!enabled_) {
      set_state(
        "disabled",
        "coverage_node_disabled");
      return;
    }
    if (!valid_grid_) {
      if (last_map_error_.empty()) {
        set_state(
          "waiting_for_map",
          "coverage_node_waiting_for_map");
      } else {
        set_state(
          "map_invalid",
          last_map_reason_.empty() ?
          "coverage_node_map_invalid" :
          last_map_reason_,
          last_map_error_);
      }
      return;
    }
    if (!latest_map_input_valid_) {
      set_state(
        "map_invalid",
        last_map_reason_.empty() ?
        "coverage_node_map_invalid" :
        last_map_reason_,
        last_map_error_);
      return;
    }
    if (!map_is_fresh()) {
      set_state(
        "map_stale",
        "coverage_node_map_stale");
      return;
    }
    if (!auto_plan_) {
      set_state(
        "ready",
        "coverage_node_auto_plan_disabled");
      return;
    }
    if (!should_attempt_current_map()) {
      if (successful_plan_published_) {
        set_state(
          "plan_ready",
          "coverage_node_plan_ready");
      } else {
        publish_status();
      }
      return;
    }

    start_planning_work();
  }

  void handle_pose_failure()
  {
    const auto & reason = last_pose_.reason;
    if (reason == "tf_pose_transform_stale") {
      set_state(
        "pose_stale",
        "coverage_node_pose_stale",
        reason);
      return;
    }
    if (reason == "tf_pose_transform_unavailable" ||
      reason == "tf_pose_lookup_timeout" ||
      reason == "tf_pose_extrapolation_error" ||
      reason == "tf_pose_connectivity_error")
    {
      set_state(
        "waiting_for_pose",
        "coverage_node_waiting_for_pose",
        reason);
      return;
    }
    set_state(
      "pose_invalid",
      "coverage_node_pose_invalid",
      reason);
  }

  void handle_plan_failure(
    const std::string & planner_reason)
  {
    if (planner_reason ==
      "coverage_planner_start_out_of_bounds")
    {
      record_plan_failure(
        "coverage_node_start_out_of_bounds",
        planner_reason);
      return;
    }
    if (planner_reason ==
      "coverage_planner_start_blocked")
    {
      record_plan_failure(
        "coverage_node_start_blocked",
        planner_reason);
      return;
    }
    record_plan_failure(
      "coverage_node_plan_failed",
      planner_reason);
  }

  void record_plan_failure(
    const std::string & reason,
    const std::string & detail)
  {
    if (!successful_plan_published_) {
      reset_plan_metrics();
    }
    terminal_ = !replan_on_map_update_;
    last_attempt_state_ = "plan_failed";
    last_attempt_reason_ = reason;
    last_attempt_detail_ = detail;
    set_state(
      last_attempt_state_,
      last_attempt_reason_,
      last_attempt_detail_);
  }

  void publish_plan(
    const coverage::CoveragePlan & plan,
    nav_msgs::msg::Path path)
  {
    path_publisher_->publish(path);
    successful_plan_published_ = true;
    last_attempted_map_sequence_ = map_sequence_;
    last_attempt_state_.clear();
    last_attempt_reason_.clear();
    last_attempt_detail_.clear();
    ++plan_sequence_;
    waypoint_count_ = plan.waypoints.size();
    reachable_cell_count_ =
      plan.reachable_cell_count;
    covered_cell_count_ =
      plan.covered_cell_count;
    coverage_ratio_ =
      plan.estimated_coverage_ratio;
    estimated_path_length_m_ =
      plan.estimated_path_length_m;
    terminal_ =
      plan_once_ || !replan_on_map_update_;

    set_state(
      "plan_ready",
      "coverage_node_plan_ready",
      "coverage_plan_ready");
  }

  void reset_plan_metrics()
  {
    waypoint_count_ = 0;
    reachable_cell_count_ = 0;
    covered_cell_count_ = 0;
    coverage_ratio_ = 0.0;
    estimated_path_length_m_ = 0.0;
  }

  void set_state(
    std::string state,
    std::string reason,
    std::string detail = {})
  {
    const bool state_changed =
      state != current_state_;
    current_state_ = std::move(state);
    current_reason_ = std::move(reason);
    current_detail_ = std::move(detail);

    if (state_changed) {
      std_msgs::msg::String message;
      message.data = current_state_;
      state_publisher_->publish(message);
    }
    publish_status();
  }

  void publish_status()
  {
    std::ostringstream output;
    output.precision(17);
    output
      << "{\"enabled\":" << bool_text(enabled_)
      << ",\"auto_plan\":" << bool_text(auto_plan_)
      << ",\"plan_once\":" << bool_text(plan_once_)
      << ",\"replan_on_map_update\":"
      << bool_text(replan_on_map_update_)
      << ",\"state\":\""
      << json_escape(current_state_)
      << "\",\"reason\":\""
      << json_escape(current_reason_)
      << "\",\"detail\":\""
      << json_escape(current_detail_)
      << "\",\"map_valid\":"
      << bool_text(
      latest_map_input_valid_ &&
      static_cast<bool>(valid_grid_))
      << ",\"map_fresh\":"
      << bool_text(map_is_fresh())
      << ",\"map_age_sec\":"
      << map_age_sec()
      << ",\"pose_valid\":"
      << bool_text(last_pose_.valid)
      << ",\"pose_fresh\":"
      << bool_text(pose_is_currently_fresh())
      << ",\"waypoint_count\":"
      << waypoint_count_
      << ",\"reachable_cell_count\":"
      << reachable_cell_count_
      << ",\"covered_cell_count\":"
      << covered_cell_count_
      << ",\"coverage_ratio\":"
      << coverage_ratio_
      << ",\"estimated_path_length_m\":"
      << estimated_path_length_m_
      << ",\"map_sequence\":"
      << map_sequence_
      << ",\"plan_sequence\":"
      << plan_sequence_
      << ",\"terminal\":"
      << bool_text(terminal_)
      << "}";

    std_msgs::msg::String message;
    message.data = output.str();
    status_publisher_->publish(message);
  }

  bool pose_is_currently_fresh() const
  {
    if (!last_pose_.valid || !last_pose_.fresh) {
      return false;
    }
    if (tf_stale_timeout_sec_ == 0.0 ||
      last_pose_.transform_stamp.nanoseconds() == 0)
    {
      return true;
    }
    try {
      const double age_sec =
        (now() - last_pose_.transform_stamp).seconds();
      return std::isfinite(age_sec) &&
             age_sec >= 0.0 &&
             age_sec <= tf_stale_timeout_sec_;
    } catch (const std::exception &) {
      return false;
    }
  }

  bool enabled_{true};
  bool auto_plan_{false};
  bool plan_once_{true};
  bool replan_on_map_update_{false};

  std::string map_topic_;
  std::string map_frame_;
  std::string base_frame_;
  std::string path_topic_;
  std::string status_topic_;
  std::string state_topic_;

  double tick_period_sec_{0.10};
  double map_stale_timeout_sec_{5.0};
  double tf_lookup_timeout_sec_{0.20};
  double tf_stale_timeout_sec_{1.0};

  coverage::CoverageGridOptions grid_options_;
  coverage::CoveragePlannerOptions planner_options_;
  std::unique_ptr<coverage::CoveragePlanner> planner_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener>
  tf_listener_;
  std::unique_ptr<TfPoseReader> tf_pose_reader_;

  std::uint64_t map_input_generation_{0};
  std::uint64_t validated_map_input_generation_{0};
  std::optional<PendingMapInput> pending_map_input_;
  std::optional<PlanWorkerResult> deferred_plan_result_;
  std::future<WorkerResult> worker_future_;

  std::shared_ptr<coverage::CoverageGrid> valid_grid_;
  bool latest_map_input_valid_{false};
  std::string last_map_reason_;
  std::string last_map_error_;
  std::optional<std::chrono::steady_clock::time_point>
  map_received_at_;
  std::uint64_t map_sequence_{0};
  std::optional<std::uint64_t>
  last_attempted_map_sequence_;
  std::string last_attempt_state_;
  std::string last_attempt_reason_;
  std::string last_attempt_detail_;

  TfPoseSnapshot last_pose_;
  bool successful_plan_published_{false};
  bool terminal_{false};
  std::uint64_t plan_sequence_{0};
  std::size_t waypoint_count_{0};
  std::size_t reachable_cell_count_{0};
  std::size_t covered_cell_count_{0};
  double coverage_ratio_{0.0};
  double estimated_path_length_m_{0.0};

  std::string current_state_;
  std::string current_reason_;
  std::string current_detail_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
    path_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    state_publisher_;
  rclcpp::Subscription<
    nav_msgs::msg::OccupancyGrid>::SharedPtr
    map_subscription_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace savo_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<
        savo_mapping::CoverageMapperNode>());
  } catch (const std::exception & error) {
    std::cerr
      << "coverage_mapper_node failed: "
      << error.what()
      << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
