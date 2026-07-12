#include "savo_mapping/saved_map_contract.hpp"

#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <iostream>
#include <memory>
#include <string>

namespace savo_mapping
{

class SavedMapVerifierNode final
  : public rclcpp::Node
{
public:
  SavedMapVerifierNode()
  : Node("saved_map_verifier_node")
  {
    declare_parameter<std::string>(
      "source.session_directory",
      "");

    declare_parameter<std::string>(
      "source.map_id",
      "");

    declare_parameter<std::string>(
      "source.expected_frame",
      "map");
  }

  int verify()
  {
    const auto result =
      session::verify_saved_map_session(
      get_parameter(
        "source.session_directory")
      .as_string(),
      get_parameter(
        "source.map_id")
      .as_string(),
      get_parameter(
        "source.expected_frame")
      .as_string());

    if (!result.valid) {
      RCLCPP_ERROR(
        get_logger(),
        "SAVED MAP VERIFICATION FAILED: %s",
        result.reason.c_str());

      return 1;
    }

    RCLCPP_INFO(
      get_logger(),
      "SAVED MAP VERIFICATION PASSED");

    RCLCPP_INFO(
      get_logger(),
      "map_id=%s frame=%s schema=%d",
      result.map_id.c_str(),
      result.frame_id.c_str(),
      result.schema_version);

    RCLCPP_INFO(
      get_logger(),
      "grid_yaml=%juB grid_image=%juB "
      "posegraph=%juB data=%juB",
      result.grid_yaml_bytes,
      result.grid_image_bytes,
      result.posegraph_bytes,
      result.posegraph_data_bytes);

    RCLCPP_INFO(
      get_logger(),
      "posegraph_base=%s",
      result.paths.artifact_base
      .c_str());

    return 0;
  }
};

}  // namespace savo_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    const auto node =
      std::make_shared<
        savo_mapping::SavedMapVerifierNode>();

    const int result = node->verify();

    rclcpp::shutdown();
    return result;
  } catch (const std::exception & exception) {
    std::cerr
      << "saved_map_verifier_node failed: "
      << exception.what()
      << '\n';

    rclcpp::shutdown();
    return 1;
  }
}
