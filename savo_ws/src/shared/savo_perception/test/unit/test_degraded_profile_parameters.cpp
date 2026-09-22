#include "savo_perception/range_sensor_parameters.hpp"

#include <gtest/gtest.h>
#include <rclcpp/parameter_map.hpp>

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

#ifndef SAVO_PERCEPTION_DEGRADED_PROFILE_PATH
#error "SAVO_PERCEPTION_DEGRADED_PROFILE_PATH must be defined by CMake"
#endif

namespace
{

namespace fs = std::filesystem;

const rclcpp::Parameter & required_sensors_parameter(
  const rclcpp::ParameterMap & parameter_map,
  const std::string & expected_node_name)
{
  const auto node = std::find_if(
    parameter_map.begin(), parameter_map.end(),
    [&expected_node_name](const auto & entry) {
      return entry.first == expected_node_name ||
             entry.first == "/" + expected_node_name;
    });
  if (node == parameter_map.end()) {
    throw std::runtime_error(
            "profile missing node parameters: " + expected_node_name);
  }

  const auto parameter = std::find_if(
    node->second.begin(), node->second.end(),
    [](const rclcpp::Parameter & candidate) {
      return candidate.get_name() == "required_sensors";
    });
  if (parameter == node->second.end()) {
    throw std::runtime_error(
            "profile missing required_sensors: " + expected_node_name);
  }
  return *parameter;
}

}  // namespace

TEST(DegradedProfileParameters, RosParserLoadsTypedEmptyRequiredSensorPolicy)
{
  const fs::path profile =
    fs::path{SAVO_PERCEPTION_DEGRADED_PROFILE_PATH};

  const auto parameter_map =
    rclcpp::parameter_map_from_yaml_file(profile.string());

  for (const std::string node_name : {
      "safety_stop_node",
      "safety_stop_node_py",
      "range_health_node",
      "range_health_node_py"})
  {
    const auto & parameter =
      required_sensors_parameter(parameter_map, node_name);
    ASSERT_EQ(
      parameter.get_type(),
      rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
    EXPECT_EQ(
      parameter.as_string_array(),
      std::vector<std::string>{savo_perception::kNoRequiredSensorSentinel});
    EXPECT_TRUE(
      savo_perception::normalize_required_sensor_names(
        parameter.as_string_array()).empty());
  }
}
