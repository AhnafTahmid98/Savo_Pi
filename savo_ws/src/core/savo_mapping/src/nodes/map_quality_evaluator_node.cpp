#include "savo_mapping/saved_map_contract.hpp"
#include "savo_mapping/saved_map_quality.hpp"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <cstdint>
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

class MapQualityEvaluatorNode final
  : public rclcpp::Node
{
public:
  MapQualityEvaluatorNode()
  : Node("map_quality_evaluator_node")
  {
    declare_parameters();

    const auto state_qos =
      rclcpp::QoS(
      rclcpp::KeepLast(1))
      .reliable()
      .transient_local();

    state_publisher_ =
      create_publisher<
        std_msgs::msg::String>(
        quality::kQualityStateTopic,
        state_qos);

    evaluation_publisher_ =
      create_publisher<
        std_msgs::msg::String>(
        quality::kQualityEvaluationTopic,
        state_qos);

    handoff_publisher_ =
      create_publisher<
        std_msgs::msg::String>(
        quality::kNavigationHandoffTopic,
        state_qos);

    evaluate_service_ =
      create_service<
        std_srvs::srv::Trigger>(
        quality::kEvaluateService,
        std::bind(
          &MapQualityEvaluatorNode::
          handle_evaluate,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    approve_service_ =
      create_service<
        std_srvs::srv::Trigger>(
        quality::kApproveHandoffService,
        std::bind(
          &MapQualityEvaluatorNode::
          handle_approve,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    revoke_service_ =
      create_service<
        std_srvs::srv::Trigger>(
        quality::kRevokeHandoffService,
        std::bind(
          &MapQualityEvaluatorNode::
          handle_revoke,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    publish_state("idle");
    publish_initial_handoff();

    RCLCPP_INFO(
      get_logger(),
      "saved-map quality evaluator ready");

    RCLCPP_INFO(
      get_logger(),
      "quality evaluation does not control "
      "robot movement, Nav2, or slam_toolbox");
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>(
      "session.root",
      "~/Savo_Pi/runtime/maps");

    declare_parameter<std::string>(
      "session.map_id",
      "");

    declare_parameter<std::string>(
      "session.expected_frame",
      "map");

    declare_parameter<std::int64_t>(
      "policy.min_width_cells",
      20);

    declare_parameter<std::int64_t>(
      "policy.min_height_cells",
      20);

    declare_parameter<std::int64_t>(
      "policy.min_known_cells",
      500);

    declare_parameter<std::int64_t>(
      "policy.min_free_cells",
      400);

    declare_parameter<std::int64_t>(
      "policy.min_occupied_cells",
      10);

    declare_parameter<double>(
      "policy.min_resolution_m",
      0.02);

    declare_parameter<double>(
      "policy.max_resolution_m",
      0.10);

    declare_parameter<double>(
      "policy.min_known_ratio",
      0.20);

    declare_parameter<double>(
      "policy."
      "min_largest_free_component_ratio",
      0.85);
  }

  quality::QualityPolicy policy() const
  {
    quality::QualityPolicy policy;

    const auto to_size =
      [](const std::int64_t value)
      {
        return static_cast<std::size_t>(
          std::max<std::int64_t>(
            value,
            0));
      };

    policy.min_width_cells =
      to_size(
      get_parameter(
        "policy.min_width_cells")
      .as_int());

    policy.min_height_cells =
      to_size(
      get_parameter(
        "policy.min_height_cells")
      .as_int());

    policy.min_known_cells =
      to_size(
      get_parameter(
        "policy.min_known_cells")
      .as_int());

    policy.min_free_cells =
      to_size(
      get_parameter(
        "policy.min_free_cells")
      .as_int());

    policy.min_occupied_cells =
      to_size(
      get_parameter(
        "policy.min_occupied_cells")
      .as_int());

    policy.min_resolution_m =
      get_parameter(
      "policy.min_resolution_m")
      .as_double();

    policy.max_resolution_m =
      get_parameter(
      "policy.max_resolution_m")
      .as_double();

    policy.min_known_ratio =
      get_parameter(
      "policy.min_known_ratio")
      .as_double();

    policy
    .min_largest_free_component_ratio =
      get_parameter(
      "policy."
      "min_largest_free_component_ratio")
      .as_double();

    return policy;
  }

  session::SavedMapVerification
  verify_session() const
  {
    const std::filesystem::path root =
      expand_home_path(
      get_parameter(
        "session.root")
      .as_string());

    const std::string map_id =
      get_parameter(
        "session.map_id")
      .as_string();

    return
      session::verify_saved_map_session(
      root / map_id,
      map_id,
      get_parameter(
        "session.expected_frame")
      .as_string());
  }

  void publish_state(
    const std::string & state)
  {
    std_msgs::msg::String message;
    message.data = state;

    state_publisher_->publish(message);
  }

  void publish_evaluation(
    const quality::QualityEvaluation &
      evaluation)
  {
    std_msgs::msg::String message;

    message.data =
      quality::quality_evaluation_to_json(
      evaluation);

    evaluation_publisher_->publish(message);
  }

  void publish_handoff(
    const quality::NavigationHandoff &
      handoff)
  {
    std_msgs::msg::String message;

    message.data =
      quality::navigation_handoff_to_json(
      handoff);

    handoff_publisher_->publish(message);
  }

  void publish_initial_handoff()
  {
    const auto verification =
      verify_session();

    if (!verification.valid) {
      quality::NavigationHandoff handoff;

      handoff.map_id =
        get_parameter(
        "session.map_id")
        .as_string();

      handoff.reason =
        verification.reason;

      publish_handoff(handoff);
      return;
    }

    publish_handoff(
      quality::read_navigation_handoff(
        verification));
  }

  void handle_evaluate(
    const std::shared_ptr<
      std_srvs::srv::Trigger::Request>,
    std::shared_ptr<
      std_srvs::srv::Trigger::Response>
      response)
  {
    publish_state("evaluating");

    try {
      const auto verification =
        verify_session();

      if (!verification.valid) {
        response->success = false;
        response->message =
          "saved_map_verification_failed:" +
          verification.reason;

        publish_state("error");
        return;
      }

      const auto evaluation =
        quality::evaluate_saved_map_session(
        verification,
        policy());

      quality::persist_quality_evaluation(
        verification,
        evaluation);

      publish_evaluation(evaluation);

      publish_handoff(
        quality::read_navigation_handoff(
          verification));

      response->success =
        evaluation.passed;

      response->message =
        evaluation.reason + ":" +
        evaluation.report_path.string();

      publish_state(
        evaluation.passed ?
        "passed_approval_required" :
        "failed");

      RCLCPP_INFO(
        get_logger(),
        "quality evaluation completed: "
        "passed=%s known_ratio=%.3f "
        "free_connectivity=%.3f",
        evaluation.passed ?
        "true" : "false",
        evaluation.metrics.known_ratio,
        evaluation.metrics
        .largest_free_component_ratio);
    } catch (const std::exception & exception) {
      response->success = false;
      response->message =
        std::string{
        "quality_evaluation_error:"} +
        exception.what();

      publish_state("error");

      RCLCPP_ERROR(
        get_logger(),
        "%s",
        response->message.c_str());
    }
  }

  void handle_approve(
    const std::shared_ptr<
      std_srvs::srv::Trigger::Request>,
    std::shared_ptr<
      std_srvs::srv::Trigger::Response>
      response)
  {
    try {
      const auto verification =
        verify_session();

      if (!verification.valid) {
        response->success = false;
        response->message =
          "saved_map_verification_failed:" +
          verification.reason;

        publish_state("error");
        return;
      }

      const auto handoff =
        quality::set_navigation_handoff(
        verification,
        true,
        "operator_approved");

      publish_handoff(handoff);
      publish_state("approved");

      response->success = true;
      response->message =
        "navigation_handoff_ready:" +
        verification.map_id;
    } catch (const std::exception & exception) {
      response->success = false;
      response->message =
        std::string{
        "navigation_handoff_rejected:"} +
        exception.what();

      publish_state("approval_rejected");
    }
  }

  void handle_revoke(
    const std::shared_ptr<
      std_srvs::srv::Trigger::Request>,
    std::shared_ptr<
      std_srvs::srv::Trigger::Response>
      response)
  {
    try {
      const auto verification =
        verify_session();

      if (!verification.valid) {
        response->success = false;
        response->message =
          "saved_map_verification_failed:" +
          verification.reason;

        publish_state("error");
        return;
      }

      const auto handoff =
        quality::set_navigation_handoff(
        verification,
        false,
        "operator_revoked");

      publish_handoff(handoff);
      publish_state("revoked");

      response->success = true;
      response->message =
        "navigation_handoff_revoked:" +
        verification.map_id;
    } catch (const std::exception & exception) {
      response->success = false;
      response->message =
        std::string{
        "navigation_handoff_revoke_error:"} +
        exception.what();

      publish_state("error");
    }
  }

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    state_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    evaluation_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    handoff_publisher_;

  rclcpp::Service<
    std_srvs::srv::Trigger>::SharedPtr
    evaluate_service_;

  rclcpp::Service<
    std_srvs::srv::Trigger>::SharedPtr
    approve_service_;

  rclcpp::Service<
    std_srvs::srv::Trigger>::SharedPtr
    revoke_service_;
};

}  // namespace savo_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<
      savo_mapping::
      MapQualityEvaluatorNode>());

  rclcpp::shutdown();
  return 0;
}
