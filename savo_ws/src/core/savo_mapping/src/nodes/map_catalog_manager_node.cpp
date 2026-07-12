#include "savo_mapping/production_map_release.hpp"
#include "savo_mapping/saved_map_contract.hpp"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

namespace savo_mapping
{

namespace
{

std::filesystem::path expand_home_path(
  const std::string & value)
{
  if (value != "~" &&
      value.rfind("~/", 0U) != 0U)
  {
    return std::filesystem::path{value};
  }

  const char * home =
    std::getenv("HOME");

  if (home == nullptr ||
      std::string{home}.empty())
  {
    throw std::runtime_error(
            "home_directory_not_available");
  }

  if (value == "~") {
    return std::filesystem::path{home};
  }

  return
    std::filesystem::path{home} /
    value.substr(2U);
}

}  // namespace

class MapCatalogManagerNode final
  : public rclcpp::Node
{
public:
  MapCatalogManagerNode()
  : Node("map_catalog_manager_node")
  {
    declare_parameter<std::string>(
      "source.root",
      "~/Savo_Pi/runtime/maps");

    declare_parameter<std::string>(
      "source.map_id",
      "");

    declare_parameter<std::string>(
      "source.expected_frame",
      "map");

    declare_parameter<std::string>(
      "production.root",
      "~/Savo_Pi/runtime/maps/production");

    declare_parameter<std::string>(
      "release.id",
      "");

    declare_parameter<bool>(
      "release.make_read_only",
      true);

    const auto state_qos =
      rclcpp::QoS(
      rclcpp::KeepLast(1))
      .reliable()
      .transient_local();

    state_publisher_ =
      create_publisher<
        std_msgs::msg::String>(
        release::kReleaseStateTopic,
        state_qos);

    result_publisher_ =
      create_publisher<
        std_msgs::msg::String>(
        release::kReleaseResultTopic,
        state_qos);

    catalog_publisher_ =
      create_publisher<
        std_msgs::msg::String>(
        release::kMapCatalogTopic,
        state_qos);

    active_publisher_ =
      create_publisher<
        std_msgs::msg::String>(
        release::kActiveMapTopic,
        state_qos);

    create_service_ =
      create_service<
        std_srvs::srv::Trigger>(
        release::kCreateReleaseService,
        std::bind(
          &MapCatalogManagerNode::
          handle_create,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    verify_service_ =
      create_service<
        std_srvs::srv::Trigger>(
        release::kVerifyReleaseService,
        std::bind(
          &MapCatalogManagerNode::
          handle_verify,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    promote_service_ =
      create_service<
        std_srvs::srv::Trigger>(
        release::kPromoteReleaseService,
        std::bind(
          &MapCatalogManagerNode::
          handle_promote,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    deactivate_service_ =
      create_service<
        std_srvs::srv::Trigger>(
        release::kDeactivateReleaseService,
        std::bind(
          &MapCatalogManagerNode::
          handle_deactivate,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    publish_state("idle");
    publish_catalog();
    publish_active();

    RCLCPP_INFO(
      get_logger(),
      "production map catalog manager ready");

    RCLCPP_INFO(
      get_logger(),
      "manager owns release creation and "
      "active-map metadata only; it does not "
      "control Nav2, slam_toolbox, or movement");
  }

private:
  std::filesystem::path source_root() const
  {
    return expand_home_path(
      get_parameter(
        "source.root")
      .as_string());
  }

  std::filesystem::path production_root() const
  {
    return expand_home_path(
      get_parameter(
        "production.root")
      .as_string());
  }

  std::string source_map_id() const
  {
    return get_parameter(
      "source.map_id")
      .as_string();
  }

  std::string release_id() const
  {
    return get_parameter(
      "release.id")
      .as_string();
  }

  void publish_string(
    const rclcpp::Publisher<
      std_msgs::msg::String>::SharedPtr &
      publisher,
    const std::string & value)
  {
    std_msgs::msg::String message;
    message.data = value;

    publisher->publish(message);
  }

  void publish_state(
    const std::string & state)
  {
    publish_string(
      state_publisher_,
      state);
  }

  void publish_result(
    const std::string & result)
  {
    publish_string(
      result_publisher_,
      result);
  }

  void publish_catalog()
  {
    publish_string(
      catalog_publisher_,
      release::catalog_to_json(
        production_root()));
  }

  void publish_active()
  {
    publish_string(
      active_publisher_,
      release::active_map_to_json(
        release::read_active_map(
          production_root())));
  }

  session::SavedMapVerification
  verify_source() const
  {
    const std::string map_id =
      source_map_id();

    return
      session::verify_saved_map_session(
      source_root() / map_id,
      map_id,
      get_parameter(
        "source.expected_frame")
      .as_string());
  }

  void handle_create(
    const std::shared_ptr<
      std_srvs::srv::Trigger::Request>,
    std::shared_ptr<
      std_srvs::srv::Trigger::Response>
      response)
  {
    publish_state("creating");

    try {
      const auto created =
        release::create_release(
        verify_source(),
        production_root(),
        release_id(),
        get_parameter(
          "release.make_read_only")
        .as_bool());

      response->success =
        created.valid;

      response->message =
        created.valid ?
        "release_created:" +
        created.release_directory.string() :
        "release_creation_failed:" +
        created.reason;

      publish_result(
        release::release_record_to_json(
          created));

      publish_catalog();

      publish_state(
        created.valid ?
        "created" :
        "error");
    } catch (const std::exception & exception) {
      response->success = false;

      response->message =
        std::string{
        "release_creation_error:"} +
        exception.what();

      publish_state("error");
    }
  }

  void handle_verify(
    const std::shared_ptr<
      std_srvs::srv::Trigger::Request>,
    std::shared_ptr<
      std_srvs::srv::Trigger::Response>
      response)
  {
    const auto verified =
      release::verify_release(
      production_root(),
      release_id());

    response->success =
      verified.valid;

    response->message =
      verified.reason;

    publish_result(
      release::release_record_to_json(
        verified));

    publish_state(
      verified.valid ?
      "verified" :
      "verification_failed");
  }

  void handle_promote(
    const std::shared_ptr<
      std_srvs::srv::Trigger::Request>,
    std::shared_ptr<
      std_srvs::srv::Trigger::Response>
      response)
  {
    publish_state("promoting");

    try {
      const auto active =
        release::promote_release(
        production_root(),
        release_id());

      response->success =
        active.active;

      response->message =
        active.active ?
        "active_map_promoted:" +
        active.release_id :
        "active_map_promotion_failed:" +
        active.reason;

      publish_string(
        active_publisher_,
        release::active_map_to_json(
          active));

      publish_state(
        active.active ?
        "active" :
        "error");
    } catch (const std::exception & exception) {
      response->success = false;

      response->message =
        std::string{
        "active_map_promotion_error:"} +
        exception.what();

      publish_state("error");
    }
  }

  void handle_deactivate(
    const std::shared_ptr<
      std_srvs::srv::Trigger::Request>,
    std::shared_ptr<
      std_srvs::srv::Trigger::Response>
      response)
  {
    try {
      const auto active =
        release::deactivate_active_map(
        production_root());

      response->success =
        !active.active;

      response->message =
        "active_map_deactivated";

      publish_string(
        active_publisher_,
        release::active_map_to_json(
          active));

      publish_state("deactivated");
    } catch (const std::exception & exception) {
      response->success = false;

      response->message =
        std::string{
        "active_map_deactivation_error:"} +
        exception.what();

      publish_state("error");
    }
  }

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    state_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    result_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    catalog_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    active_publisher_;

  rclcpp::Service<
    std_srvs::srv::Trigger>::SharedPtr
    create_service_;

  rclcpp::Service<
    std_srvs::srv::Trigger>::SharedPtr
    verify_service_;

  rclcpp::Service<
    std_srvs::srv::Trigger>::SharedPtr
    promote_service_;

  rclcpp::Service<
    std_srvs::srv::Trigger>::SharedPtr
    deactivate_service_;
};

}  // namespace savo_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<
      savo_mapping::
      MapCatalogManagerNode>());

  rclcpp::shutdown();
  return 0;
}
