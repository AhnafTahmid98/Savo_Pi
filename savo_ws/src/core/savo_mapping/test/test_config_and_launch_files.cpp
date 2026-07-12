#include "savo_mapping/exploration_goal_handoff.hpp"
#include "savo_mapping/topic_names.hpp"
#include <gtest/gtest.h>
#include <rclcpp/parameter_map.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef SAVO_MAPPING_SOURCE_DIR
#error "SAVO_MAPPING_SOURCE_DIR must be defined by CMake"
#endif

namespace
{

namespace fs = std::filesystem;

const fs::path package_root{SAVO_MAPPING_SOURCE_DIR};
const fs::path config_root = package_root / "config";

std::string read_text_file(const fs::path & path)
{
  std::ifstream input(path);

  if (!input.is_open()) {
    throw std::runtime_error("failed to open file: " + path.string());
  }

  std::ostringstream content;
  content << input.rdbuf();
  return content.str();
}

void expect_contains(
  const std::string & content,
  const std::string & expected)
{
  EXPECT_NE(content.find(expected), std::string::npos)
    << "missing expected text: " << expected;
}

void expect_common_ros_parameter_structure(const std::string & content)
{
  expect_contains(content, "/**:");
  expect_contains(content, "ros__parameters:");

  EXPECT_EQ(content.find('\t'), std::string::npos)
    << "YAML files must use spaces, not tab characters";

  EXPECT_EQ(content.find('\r'), std::string::npos)
    << "YAML files must use Unix LF line endings";
}

}  // namespace

TEST(ConfigContracts, RequiredConfigFilesExistAndAreNotEmpty)
{
  const std::vector<std::string> filenames{
    "topics.yaml",
    "frames.yaml",
    "qos.yaml",
    "mapping_common.yaml",
    "map_session.yaml",
    "map_quality.yaml"
  };

  for (const auto & filename : filenames) {
    const auto path = config_root / filename;

    EXPECT_TRUE(fs::exists(path))
      << "missing config file: " << path;

    EXPECT_TRUE(fs::is_regular_file(path))
      << "config path is not a regular file: " << path;

    if (fs::exists(path) && fs::is_regular_file(path)) {
      EXPECT_GT(fs::file_size(path), 0u)
        << "config file is empty: " << path;
    }
  }
}

TEST(ConfigContracts, TopicConfigContainsOfficialInterfaces)
{
  const auto content = read_text_file(config_root / "topics.yaml");

  expect_common_ros_parameter_structure(content);

  expect_contains(content, "scan: \"/scan\"");
  expect_contains(content, "map: \"/map\"");
  expect_contains(content, "odom_filtered: \"/odometry/filtered\"");
  expect_contains(content, "status: \"/savo_mapping/status\"");
  expect_contains(content, "readiness: \"/savo_mapping/readiness\"");
  expect_contains(content, "map_saved: \"/savo_mapping/map_saved\"");
  expect_contains(content, "head_confirmations: \"/savo_head/semantic_confirmations\"");
  expect_contains(content, "status: \"/savo_nav/status\"");
  expect_contains(
    content,
    "selected_goal: \"/savo_mapping/exploration/selected_goal\"");

  expect_contains(
    content,
    "state: \"/savo_mapping/exploration_goal/state\"");

  expect_contains(
    content,
    "status: \"/savo_mapping/exploration_goal/status\"");

  expect_contains(
    content,
    "feedback: \"/savo_mapping/exploration_goal/feedback\"");

  expect_contains(
    content,
    "exploration_goal_cancel: "
    "\"/savo_mapping/exploration_goal/cancel\"");

  expect_contains(
    content,
    "exploration_navigate_to_pose: "
    "\"/savo_nav/exploration/navigate_to_pose\"");
  expect_contains(content, "stop: \"/safety/stop\"");
}

TEST(ConfigContracts, FrameConfigContainsCoreTfContract)
{
  const auto content = read_text_file(config_root / "frames.yaml");

  expect_common_ros_parameter_structure(content);

  expect_contains(content, "map: \"map\"");
  expect_contains(content, "odom: \"odom\"");
  expect_contains(content, "base: \"base_link\"");
  expect_contains(content, "lidar: \"laser_frame\"");
  expect_contains(content, "require_map_to_odom: true");
  expect_contains(content, "require_odom_to_base: true");
  expect_contains(content, "require_base_to_lidar: true");
  expect_contains(
    content,
    "require_fresh_map_to_odom: false");

  expect_contains(
    content,
    "require_fresh_odom_to_base: true");

  expect_contains(
    content,
    "require_fresh_base_to_lidar: false");
}

TEST(ConfigContracts, QosConfigMatchesCoreProfiles)
{
  const auto content = read_text_file(config_root / "qos.yaml");

  expect_common_ros_parameter_structure(content);

  expect_contains(content, "status:");
  expect_contains(content, "state:");
  expect_contains(content, "command:");
  expect_contains(content, "scan:");
  expect_contains(content, "pointcloud:");
  expect_contains(content, "map:");
  expect_contains(content, "event:");

  expect_contains(content, "reliability: \"best_effort\"");
  expect_contains(content, "reliability: \"reliable\"");
  expect_contains(content, "durability: \"transient_local\"");
  expect_contains(content, "durability: \"volatile\"");
}

TEST(ConfigContracts, CommonConfigDefaultsToSafeOwnership)
{
  const auto content =
    read_text_file(config_root / "mapping_common.yaml");

  expect_common_ros_parameter_structure(content);

  expect_contains(content, "default_mode: \"monitor_only\"");
  expect_contains(content, "default_exploration_mode: \"idle\"");
  expect_contains(content, "allow_direct_motor_control: false");
  expect_contains(content, "allow_internal_package_calls: false");
  expect_contains(content, "navigation_handoff:");
  expect_contains(content, "enabled: false");
}

TEST(ConfigContracts, SessionConfigUsesPortableRuntimeStorage)
{
  const auto content =
    read_text_file(config_root / "map_session.yaml");

  expect_common_ros_parameter_structure(content);

  expect_contains(content, "root_directory: \"maps/saved\"");
  expect_contains(content, "path_base: \"repository_root\"");
  expect_contains(content, "allow_overwrite: false");
  expect_contains(content, "map_yaml_filename: \"map.yaml\"");
  expect_contains(content, "map_image_filename: \"map.pgm\"");
  expect_contains(content, "quality_report_filename: \"quality_report.yaml\"");
  expect_contains(content, "semantic_landmarks_filename: \"semantic_landmarks.yaml\"");
}

TEST(ConfigContracts, QualityConfigContainsNavigationGate)
{
  const auto content =
    read_text_file(config_root / "map_quality.yaml");

  expect_common_ros_parameter_structure(content);

  expect_contains(content, "minimum_quality_score: 0.70");
  expect_contains(content, "navigation_handoff_score: 0.80");
  expect_contains(content, "reject_non_finite_values: true");
  expect_contains(content, "reject_empty_map: true");
  expect_contains(content, "require_saved_map_files: true");
  expect_contains(content, "require_complete_session: true");
}

TEST(ConfigContracts, FilesParseWithRosParameterParser)
{
  const std::vector<std::string> filenames{
    "topics.yaml",
    "frames.yaml",
    "qos.yaml",
    "mapping_common.yaml",
    "map_session.yaml",
    "map_quality.yaml"
  };

  for (const auto & filename : filenames) {
    const auto path = config_root / filename;

    try {
      const auto parameter_map =
        rclcpp::parameter_map_from_yaml_file(path.string());

      EXPECT_FALSE(parameter_map.empty())
        << "ROS parameter map is empty: " << path;
    } catch (const std::exception & exception) {
      FAIL()
        << "ROS parameter parser rejected " << path
        << ": " << exception.what();
    }
  }
}

TEST(ConfigContracts, TopicYamlCoversEveryCppTopicContract)
{
  const auto content = read_text_file(config_root / "topics.yaml");

  const std::vector<std::string_view> topics{
    savo_mapping::topics::SCAN,
    savo_mapping::topics::MAP,
    savo_mapping::topics::MAP_METADATA,
    savo_mapping::topics::TF,
    savo_mapping::topics::TF_STATIC,
    savo_mapping::topics::ODOM,
    savo_mapping::topics::ODOM_FILTERED,
    savo_mapping::topics::WHEEL_ODOM,

    savo_mapping::topics::REALSENSE_STATUS,
    savo_mapping::topics::DEPTH_MIN_FRONT,
    savo_mapping::topics::DEPTH_POINTS,
    savo_mapping::topics::HEAD_SEMANTIC_CONFIRMATIONS,

    savo_mapping::topics::STATUS,
    savo_mapping::topics::READINESS,
    savo_mapping::topics::MODE,
    savo_mapping::topics::WORKFLOW_PHASE,
    savo_mapping::topics::SESSION_STATE,
    savo_mapping::topics::MAP_QUALITY,
    savo_mapping::topics::MAP_SAVED,
    savo_mapping::topics::EXPLORATION_STATUS,
    savo_mapping::topics::SEMANTIC_EVENTS,
    savo_mapping::topics::DASHBOARD,

    savo_mapping::topics::MODE_CMD,
    savo_mapping::topics::START_SESSION_CMD,
    savo_mapping::topics::STOP_SESSION_CMD,
    savo_mapping::topics::SAVE_MAP_CMD,
    savo_mapping::topics::CANCEL_SESSION_CMD,
    savo_mapping::topics::SCAN360_CMD,

    savo_mapping::topics::EXPLORATION_SELECTED_GOAL,
    savo_mapping::topics::EXPLORATION_GOAL_STATE,
    savo_mapping::topics::EXPLORATION_GOAL_STATUS,
    savo_mapping::topics::EXPLORATION_GOAL_FEEDBACK,
    savo_mapping::topics::NAV_STATUS,

    savo_mapping::topics::SAFETY_STOP,
    savo_mapping::topics::SAFETY_SLOWDOWN_FACTOR,
    savo_mapping::topics::CONTROL_MODE_STATE
  };

  for (const auto topic : topics) {
    expect_contains(
      content,
      "\"" + std::string{topic} + "\"");
  }
}


TEST(ConfigContracts, ExplorationHandoffBoundaryIsExplicitAndSafe)
{
  const auto content =
    read_text_file(config_root / "topics.yaml");

  expect_contains(
    content,
    "\"" +
    std::string{
      savo_mapping::exploration::
      kGoalCancelService} +
    "\"");

  expect_contains(
    content,
    "\"" +
    std::string{
      savo_mapping::exploration::
      kExplorationActionName} +
    "\"");

  // savo_mapping must not expose a generic direct Nav2 goal contract.
  EXPECT_EQ(
    content.find("\"/goal_pose\""),
    std::string::npos);

  EXPECT_EQ(
    content.find("\"/navigate_to_pose\""),
    std::string::npos);

  // Movement command ownership belongs outside savo_mapping.
  EXPECT_EQ(
    content.find("\"/cmd_vel\""),
    std::string::npos);

  EXPECT_EQ(
    content.find("\"/cmd_vel_nav\""),
    std::string::npos);

  EXPECT_EQ(
    content.find("\"/cmd_vel_safe\""),
    std::string::npos);
}

TEST(ConfigContracts, MonitorOnlyProfileExistsAndParses)
{
  const auto profile_path =
    config_root / "profiles" / "monitor_only.yaml";

  ASSERT_TRUE(fs::exists(profile_path))
    << "missing monitor-only profile: " << profile_path;

  ASSERT_TRUE(fs::is_regular_file(profile_path));
  ASSERT_GT(fs::file_size(profile_path), 0u);

  const auto content = read_text_file(profile_path);

  expect_contains(content, "/mapping_supervisor_node:");
  expect_contains(content, "ros__parameters:");
  expect_contains(content, "default_mode: \"monitor_only\"");
  expect_contains(content, "default_exploration_mode: \"idle\"");
  expect_contains(content, "require_scan: true");
  expect_contains(content, "require_map: true");
  expect_contains(content, "require_odom: true");
  expect_contains(content, "require_tf: true");
  expect_contains(content, "allow_direct_motor_control: false");
  expect_contains(content, "allow_internal_package_calls: false");
  expect_contains(content, "enabled: false");

  try {
    const auto parameter_map =
      rclcpp::parameter_map_from_yaml_file(
      profile_path.string());

    EXPECT_FALSE(parameter_map.empty());
  } catch (const std::exception & exception) {
    FAIL()
      << "ROS parameter parser rejected "
      << profile_path
      << ": "
      << exception.what();
  }
}

TEST(LaunchContracts, MonitorOnlyLaunchExistsAndIsNotEmpty)
{
  const auto launch_path =
    package_root / "launch" / "monitor_only.launch.xml";

  ASSERT_TRUE(fs::exists(launch_path))
    << "missing monitor-only launch: " << launch_path;

  ASSERT_TRUE(fs::is_regular_file(launch_path));
  ASSERT_GT(fs::file_size(launch_path), 0u);
}

TEST(LaunchContracts, MonitorOnlyLaunchStartsOnlySupervisor)
{
  const auto content = read_text_file(
    package_root / "launch" / "monitor_only.launch.xml");

  expect_contains(content, "<?xml version=\"1.0\"");
  expect_contains(content, "<launch>");
  expect_contains(content, "pkg=\"savo_mapping\"");
  expect_contains(content, "exec=\"mapping_supervisor_node\"");
  expect_contains(content, "name=\"mapping_supervisor_node\"");
  expect_contains(content, "output=\"screen\"");
  expect_contains(content, "</launch>");

  EXPECT_EQ(
    content.find("slam_toolbox"),
    std::string::npos);

  EXPECT_EQ(
    content.find("savo_nav"),
    std::string::npos);

  EXPECT_EQ(
    content.find("savo_control"),
    std::string::npos);

  EXPECT_EQ(
    content.find("cmd_vel"),
    std::string::npos);
}

TEST(LaunchContracts, MonitorOnlyLaunchLoadsRequiredConfiguration)
{
  const auto content = read_text_file(
    package_root / "launch" / "monitor_only.launch.xml");

  expect_contains(
    content,
    "$(find-pkg-share savo_mapping)/config/topics.yaml");

  expect_contains(
    content,
    "$(find-pkg-share savo_mapping)/config/frames.yaml");

  expect_contains(
    content,
    "$(find-pkg-share savo_mapping)/config/mapping_common.yaml");

  expect_contains(
    content,
    "$(find-pkg-share savo_mapping)/config/map_session.yaml");

  expect_contains(
    content,
    "$(find-pkg-share savo_mapping)/config/profiles/monitor_only.yaml");

  expect_contains(
    content,
    "name=\"use_sim_time\"");

  expect_contains(
    content,
    "value=\"$(var use_sim_time)\"");
}

TEST(LaunchContracts, MonitorOnlyLaunchContainsNoPythonRuntime)
{
  const auto content = read_text_file(
    package_root / "launch" / "monitor_only.launch.xml");

  EXPECT_EQ(content.find(".launch.py"), std::string::npos);
  EXPECT_EQ(content.find("python"), std::string::npos);
  EXPECT_EQ(content.find("rclpy"), std::string::npos);
}

TEST(ConfigContracts, SlamToolboxMappingConfigExistsAndParses)
{
  const auto path =
    config_root / "slam_toolbox_mapping.yaml";

  ASSERT_TRUE(fs::exists(path))
    << "missing slam_toolbox mapping config: " << path;

  ASSERT_TRUE(fs::is_regular_file(path));
  ASSERT_GT(fs::file_size(path), 0u);

  try {
    const auto parameter_map =
      rclcpp::parameter_map_from_yaml_file(path.string());

    EXPECT_FALSE(parameter_map.empty());
  } catch (const std::exception & exception) {
    FAIL()
      << "ROS parameter parser rejected "
      << path
      << ": "
      << exception.what();
  }
}

TEST(ConfigContracts, SlamToolboxMappingUsesRobotSavoFrames)
{
  const auto content =
    read_text_file(config_root / "slam_toolbox_mapping.yaml");

  expect_contains(content, "slam_toolbox:");
  expect_contains(content, "mode: \"mapping\"");
  expect_contains(content, "map_frame: \"map\"");
  expect_contains(content, "odom_frame: \"odom\"");
  expect_contains(content, "base_frame: \"base_link\"");
  expect_contains(content, "scan_topic: \"/scan\"");
}

TEST(ConfigContracts, SlamToolboxUsesSafeAsyncSettings)
{
  const auto content =
    read_text_file(config_root / "slam_toolbox_mapping.yaml");

  expect_contains(content, "scan_queue_size: 1");
  expect_contains(content, "throttle_scans: 1");
  expect_contains(content, "map_update_interval: 2.0");
  expect_contains(content, "transform_publish_period: 0.05");
  expect_contains(content, "resolution: 0.05");
  expect_contains(content, "enable_interactive_mode: false");
  expect_contains(content, "do_loop_closing: true");
}

TEST(ConfigContracts, SlamToolboxUsesRobustCeresSolver)
{
  const auto content =
    read_text_file(config_root / "slam_toolbox_mapping.yaml");

  expect_contains(
    content,
    "solver_plugin: \"solver_plugins::CeresSolver\"");

  expect_contains(
    content,
    "ceres_linear_solver: \"SPARSE_NORMAL_CHOLESKY\"");

  expect_contains(
    content,
    "ceres_preconditioner: \"SCHUR_JACOBI\"");

  expect_contains(
    content,
    "ceres_trust_strategy: \"LEVENBERG_MARQUARDT\"");

  expect_contains(
    content,
    "ceres_loss_function: \"HuberLoss\"");
}

TEST(ConfigContracts, FreshMappingDoesNotLoadExistingPoseGraph)
{
  const auto content =
    read_text_file(config_root / "slam_toolbox_mapping.yaml");

  EXPECT_EQ(content.find("map_file_name:"), std::string::npos);
  EXPECT_EQ(content.find("map_start_pose:"), std::string::npos);
  EXPECT_EQ(content.find("map_start_at_dock:"), std::string::npos);
}


TEST(ConfigContracts, ManualMappingProfileExistsAndParses)
{
  const auto profile_path =
    config_root / "profiles" / "manual_mapping.yaml";

  ASSERT_TRUE(fs::exists(profile_path))
    << "missing manual-mapping profile: " << profile_path;

  ASSERT_TRUE(fs::is_regular_file(profile_path));
  ASSERT_GT(fs::file_size(profile_path), 0u);

  const auto content = read_text_file(profile_path);

  expect_contains(content, "/mapping_supervisor_node:");
  expect_contains(content, "ros__parameters:");
  expect_contains(content, "default_mode: \"manual\"");
  expect_contains(content, "default_exploration_mode: \"idle\"");

  expect_contains(content, "require_scan: true");
  expect_contains(content, "require_map: true");
  expect_contains(content, "require_odom: true");
  expect_contains(content, "require_tf: true");

  expect_contains(
    content,
    "allow_direct_motor_control: false");

  expect_contains(
    content,
    "allow_internal_package_calls: false");

  expect_contains(content, "enabled: false");

  try {
    const auto parameter_map =
      rclcpp::parameter_map_from_yaml_file(
      profile_path.string());

    EXPECT_FALSE(parameter_map.empty());
  } catch (const std::exception & exception) {
    FAIL()
      << "ROS parameter parser rejected "
      << profile_path
      << ": "
      << exception.what();
  }
}

TEST(LaunchContracts, ManualMappingLaunchExistsAndIsNotEmpty)
{
  const auto launch_path =
    package_root / "launch" / "manual_mapping.launch.xml";

  ASSERT_TRUE(fs::exists(launch_path))
    << "missing manual-mapping launch: " << launch_path;

  ASSERT_TRUE(fs::is_regular_file(launch_path));
  ASSERT_GT(fs::file_size(launch_path), 0u);
}

TEST(LaunchContracts, ManualMappingLaunchUsesOfficialLifecycleLaunch)
{
  const auto content = read_text_file(
    package_root / "launch" / "manual_mapping.launch.xml");

  expect_contains(
    content,
    "$(find-pkg-share slam_toolbox)/launch/online_async_launch.py");

  expect_contains(
    content,
    "name=\"slam_params_file\"");

  expect_contains(
    content,
    "$(find-pkg-share savo_mapping)/config/slam_toolbox_mapping.yaml");

  expect_contains(
    content,
    "name=\"autostart\"");

  expect_contains(
    content,
    "value=\"$(var autostart)\"");

  expect_contains(
    content,
    "name=\"use_lifecycle_manager\"");

  expect_contains(
    content,
    "value=\"false\"");
}

TEST(LaunchContracts, ManualMappingLaunchStartsSupervisor)
{
  const auto content = read_text_file(
    package_root / "launch" / "manual_mapping.launch.xml");

  expect_contains(content, "pkg=\"savo_mapping\"");
  expect_contains(content, "exec=\"mapping_supervisor_node\"");
  expect_contains(content, "name=\"mapping_supervisor_node\"");

  expect_contains(
    content,
    "$(find-pkg-share savo_mapping)/config/topics.yaml");

  expect_contains(
    content,
    "$(find-pkg-share savo_mapping)/config/frames.yaml");

  expect_contains(
    content,
    "$(find-pkg-share savo_mapping)/config/mapping_common.yaml");

  expect_contains(
    content,
    "$(find-pkg-share savo_mapping)/config/map_session.yaml");

  expect_contains(
    content,
    "$(find-pkg-share savo_mapping)/config/profiles/manual_mapping.yaml");
}

TEST(LaunchContracts, ManualMappingLaunchPreservesPackageOwnership)
{
  const auto content = read_text_file(
    package_root / "launch" / "manual_mapping.launch.xml");

  EXPECT_EQ(content.find("savo_lidar"), std::string::npos);
  EXPECT_EQ(content.find("savo_localization"), std::string::npos);
  EXPECT_EQ(content.find("savo_control"), std::string::npos);
  EXPECT_EQ(content.find("savo_base"), std::string::npos);
  EXPECT_EQ(content.find("savo_nav"), std::string::npos);

  EXPECT_EQ(content.find("cmd_vel"), std::string::npos);
  EXPECT_EQ(content.find("goal_pose"), std::string::npos);
}

TEST(LaunchContracts, ManualMappingLaunchSupportsRealAndSimTime)
{
  const auto content = read_text_file(
    package_root / "launch" / "manual_mapping.launch.xml");

  expect_contains(
    content,
    "name=\"use_sim_time\"");

  expect_contains(
    content,
    "default=\"false\"");

  expect_contains(
    content,
    "value=\"$(var use_sim_time)\"");
}

TEST(ConfigContracts, SimulatedSlamProfileExistsAndParses)
{
  const auto profile_path =
    config_root / "profiles" / "simulated_slam.yaml";

  ASSERT_TRUE(fs::exists(profile_path));
  ASSERT_TRUE(fs::is_regular_file(profile_path));
  ASSERT_GT(fs::file_size(profile_path), 0u);

  const auto content = read_text_file(profile_path);

  expect_contains(
    content,
    "/simulated_slam_input_node:");

  expect_contains(
    content,
    "publish_rate_hz: 10.0");

  expect_contains(
    content,
    "sample_count: 360");

  expect_contains(
    content,
    "scan: \"/scan\"");

  expect_contains(
    content,
    "odom: \"/odometry/filtered\"");

  expect_contains(
    content,
    "lidar: \"laser_frame\"");

  try {
    const auto parameter_map =
      rclcpp::parameter_map_from_yaml_file(
      profile_path.string());

    EXPECT_FALSE(parameter_map.empty());
  } catch (const std::exception & exception) {
    FAIL()
      << "ROS parameter parser rejected "
      << profile_path
      << ": "
      << exception.what();
  }
}

TEST(LaunchContracts, SimulatedManualMappingLaunchExists)
{
  const auto launch_path =
    package_root /
    "launch" /
    "simulated_manual_mapping.launch.xml";

  ASSERT_TRUE(fs::exists(launch_path));
  ASSERT_TRUE(fs::is_regular_file(launch_path));
  ASSERT_GT(fs::file_size(launch_path), 0u);
}

TEST(LaunchContracts, SimulatedLaunchReusesManualMapping)
{
  const auto content = read_text_file(
    package_root /
    "launch" /
    "simulated_manual_mapping.launch.xml");

  expect_contains(
    content,
    "$(find-pkg-share savo_mapping)/launch/manual_mapping.launch.xml");

  expect_contains(
    content,
    "exec=\"simulated_slam_input_node\"");

  expect_contains(
    content,
    "$(find-pkg-share savo_mapping)/config/profiles/simulated_slam.yaml");

  expect_contains(
    content,
    "name=\"use_sim_time\"");

  expect_contains(
    content,
    "value=\"false\"");
}

TEST(LaunchContracts, SimulationIsNotInProductionManualLaunch)
{
  const auto production_content = read_text_file(
    package_root /
    "launch" /
    "manual_mapping.launch.xml");

  EXPECT_EQ(
    production_content.find(
      "simulated_slam_input_node"),
    std::string::npos);

  EXPECT_EQ(
    production_content.find(
      "simulated_slam.yaml"),
    std::string::npos);
}

TEST(LaunchContracts, SimulationDoesNotProvideFakeMap)
{
  const auto content = read_text_file(
    package_root /
    "launch" /
    "simulated_manual_mapping.launch.xml");

  EXPECT_EQ(
    content.find("first_map_validator_node"),
    std::string::npos);

  EXPECT_EQ(
    content.find("nav_msgs/msg/OccupancyGrid"),
    std::string::npos);
}

TEST(ConfigContracts, SlamLifecycleHealthConfigParses)
{
  const auto config_path =
    config_root / "slam_lifecycle_health.yaml";

  ASSERT_TRUE(fs::exists(config_path));
  ASSERT_TRUE(fs::is_regular_file(config_path));
  ASSERT_GT(fs::file_size(config_path), 0u);

  const auto content = read_text_file(config_path);

  expect_contains(
    content,
    "/slam_lifecycle_health_bridge_node:");

  expect_contains(
    content,
    "get_state_service: \"/slam_toolbox/get_state\"");

  expect_contains(
    content,
    "lifecycle_state: \"/savo_mapping/slam_lifecycle_state\"");

  expect_contains(
    content,
    "health: \"/savo_mapping/slam_health\"");

  expect_contains(
    content,
    "manual_workflow_state: \"/savo_mapping/manual_workflow_state\"");

  try {
    const auto parameter_map =
      rclcpp::parameter_map_from_yaml_file(
      config_path.string());

    EXPECT_FALSE(parameter_map.empty());
  } catch (const std::exception & exception) {
    FAIL()
      << "ROS parameter parser rejected "
      << config_path
      << ": "
      << exception.what();
  }
}

TEST(LaunchContracts, ManualMappingStartsLifecycleHealthBridge)
{
  const auto content = read_text_file(
    package_root /
    "launch" /
    "manual_mapping.launch.xml");

  expect_contains(
    content,
    "exec=\"slam_lifecycle_health_bridge_node\"");

  expect_contains(
    content,
    "name=\"slam_lifecycle_health_bridge_node\"");

  expect_contains(
    content,
    "$(find-pkg-share savo_mapping)/config/slam_lifecycle_health.yaml");
}

TEST(OwnershipContracts, LifecycleBridgeIsReadOnly)
{
  const auto source = read_text_file(
    package_root /
    "src" /
    "nodes" /
    "slam_lifecycle_health_bridge_node.cpp");

  expect_contains(
    source,
    "lifecycle_msgs/srv/get_state.hpp");

  expect_contains(
    source,
    "create_client<GetState>");

  EXPECT_EQ(
    source.find(
      "lifecycle_msgs/srv/change_state.hpp"),
    std::string::npos);

  EXPECT_EQ(
    source.find(
      "/slam_toolbox/change_state"),
    std::string::npos);

  EXPECT_EQ(
    source.find(
      "create_client<lifecycle_msgs::srv::ChangeState>"),
    std::string::npos);
}

TEST(OwnershipContracts, LifecycleBridgeHasNoMovementInterface)
{
  const auto source = read_text_file(
    package_root /
    "src" /
    "nodes" /
    "slam_lifecycle_health_bridge_node.cpp");

  EXPECT_EQ(
    source.find("cmd_vel"),
    std::string::npos);

  EXPECT_EQ(
    source.find("goal_pose"),
    std::string::npos);
}

TEST(ConfigContracts, MapSessionManagerConfigParses)
{
  const auto config_path =
    config_root / "map_session_manager.yaml";

  ASSERT_TRUE(fs::exists(config_path));
  ASSERT_TRUE(fs::is_regular_file(config_path));
  ASSERT_GT(fs::file_size(config_path), 0u);

  const auto content =
    read_text_file(config_path);

  expect_contains(
    content,
    "/map_session_manager_node:");

  expect_contains(
    content,
    "allow_overwrite: false");

  expect_contains(
    content,
    "require_mapping_ready: true");

  expect_contains(
    content,
    "require_slam_active: true");

  expect_contains(
    content,
    "require_structurally_valid_map: true");

  expect_contains(
    content,
    "trigger_save: \"/savo_mapping/map_session/save\"");

  expect_contains(
    content,
    "slam_save_map: \"/slam_toolbox/save_map\"");

  expect_contains(
    content,
    "slam_serialize_map: \"/slam_toolbox/serialize_map\"");

  try {
    const auto parameter_map =
      rclcpp::parameter_map_from_yaml_file(
      config_path.string());

    EXPECT_FALSE(parameter_map.empty());
  } catch (const std::exception & exception) {
    FAIL()
      << "ROS parameter parser rejected "
      << config_path
      << ": "
      << exception.what();
  }
}

TEST(LaunchContracts, MapSessionManagerLaunchExists)
{
  const auto launch_path =
    package_root /
    "launch" /
    "map_session_manager.launch.xml";

  ASSERT_TRUE(fs::exists(launch_path));
  ASSERT_TRUE(fs::is_regular_file(launch_path));
  ASSERT_GT(fs::file_size(launch_path), 0u);
}

TEST(LaunchContracts, ManualMappingIncludesSessionManager)
{
  const auto content =
    read_text_file(
    package_root /
    "launch" /
    "manual_mapping.launch.xml");

  expect_contains(
    content,
    "$(find-pkg-share savo_mapping)/launch/map_session_manager.launch.xml");

  expect_contains(
    content,
    "name=\"map_id\"");

  expect_contains(
    content,
    "name=\"map_output_root\"");

  expect_contains(
    content,
    "name=\"allow_map_overwrite\"");
}

TEST(LaunchContracts, SimulationForwardsSessionArguments)
{
  const auto content =
    read_text_file(
    package_root /
    "launch" /
    "simulated_manual_mapping.launch.xml");

  expect_contains(
    content,
    "value=\"$(var map_id)\"");

  expect_contains(
    content,
    "value=\"$(var map_output_root)\"");

  expect_contains(
    content,
    "value=\"$(var allow_map_overwrite)\"");
}

TEST(OwnershipContracts, SessionManagerOwnsOnlySaving)
{
  const auto source =
    read_text_file(
    package_root /
    "src" /
    "nodes" /
    "map_session_manager_node.cpp");

  expect_contains(
    source,
    "slam_toolbox/srv/save_map.hpp");

  expect_contains(
    source,
    "slam_toolbox/srv/serialize_pose_graph.hpp");

  EXPECT_EQ(
    source.find("cmd_vel"),
    std::string::npos);

  EXPECT_EQ(
    source.find("goal_pose"),
    std::string::npos);

  EXPECT_EQ(
    source.find("change_state"),
    std::string::npos);

  EXPECT_EQ(
    source.find("pause_new_measurements"),
    std::string::npos);

  EXPECT_EQ(
    source.find("/slam_toolbox/reset"),
    std::string::npos);
}

TEST(ConfigContracts, LifecycleBridgeUsesMapSessionState)
{
  const auto content =
    read_text_file(
    config_root /
    "slam_lifecycle_health.yaml");

  expect_contains(
    content,
    "session_state: \"/savo_mapping/map_session/state\"");
}


TEST(LaunchContracts, ContinuedMappingUsesSavedMapPreflight)
{
  const auto content =
    read_text_file(
    package_root /
    "launch" /
    "continued_mapping.launch.py");

  expect_contains(
    content,
    "saved_map_verifier_node");

  expect_contains(
    content,
    "map_file_name");

  expect_contains(
    content,
    "map_start_at_dock");

  expect_contains(
    content,
    "map_start_pose");

  expect_contains(
    content,
    "OnProcessExit");

  expect_contains(
    content,
    "manual_mapping.launch.xml");
}

TEST(OwnershipContracts, SavedMapVerifierHasNoRobotControl)
{
  const auto source =
    read_text_file(
    package_root /
    "src" /
    "nodes" /
    "saved_map_verifier_node.cpp");

  EXPECT_EQ(
    source.find("cmd_vel"),
    std::string::npos);

  EXPECT_EQ(
    source.find("goal_pose"),
    std::string::npos);

  EXPECT_EQ(
    source.find("change_state"),
    std::string::npos);

  EXPECT_EQ(
    source.find("deserialize_map"),
    std::string::npos);
}
