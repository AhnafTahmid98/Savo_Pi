#include "savo_mapping/manual_workflow_state.hpp"
#include "savo_mapping/parameter_utils.hpp"
#include "savo_mapping/qos_profiles.hpp"
#include "savo_mapping/slam_lifecycle_topics.hpp"

#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>

namespace savo_mapping
{

namespace
{

using SteadyClock = std::chrono::steady_clock;
using GetState = lifecycle_msgs::srv::GetState;

std::chrono::nanoseconds seconds_to_period(
  double seconds)
{
  return std::chrono::duration_cast<
    std::chrono::nanoseconds>(
    std::chrono::duration<double>(seconds));
}

std::string require_non_empty(
  const std::string & parameter_name,
  std::string value)
{
  if (value.empty()) {
    throw std::invalid_argument(
            "parameter '" + parameter_name +
            "' must not be empty");
  }

  return value;
}

std::string normalize_state_string(
  std::string value)
{
  std::transform(
    value.begin(),
    value.end(),
    value.begin(),
    [](unsigned char character) {
      return static_cast<char>(
        std::tolower(character));
    });

  return value;
}

}  // namespace

class SlamLifecycleHealthBridgeNode final
  : public rclcpp::Node
{
public:
  SlamLifecycleHealthBridgeNode()
  : Node("slam_lifecycle_health_bridge_node"),
    started_at_(SteadyClock::now())
  {
    declare_parameters();
    create_interfaces();

    query_timer_ = create_wall_timer(
      seconds_to_period(query_period_s_),
      std::bind(
        &SlamLifecycleHealthBridgeNode::on_timer,
        this));

    publish_state();

    RCLCPP_INFO(
      get_logger(),
      "monitoring lifecycle service %s",
      get_state_service_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "bridge is read-only and does not request "
      "lifecycle transitions");
  }

private:
  void declare_parameters()
  {
    query_period_s_ =
      params::require_positive_parameter(
      "lifecycle.query_period_s",
      params::declare_or_get<double>(
        *this,
        "lifecycle.query_period_s",
        0.50));

    response_timeout_s_ =
      params::require_positive_parameter(
      "lifecycle.response_timeout_s",
      params::declare_or_get<double>(
        *this,
        "lifecycle.response_timeout_s",
        1.50));

    startup_grace_s_ =
      params::require_positive_parameter(
      "lifecycle.startup_grace_s",
      params::declare_or_get<double>(
        *this,
        "lifecycle.startup_grace_s",
        5.0));

    get_state_service_ = require_non_empty(
      "lifecycle.get_state_service",
      params::declare_or_get<std::string>(
        *this,
        "lifecycle.get_state_service",
        std::string{
          slam_lifecycle_topics::
          GET_STATE_SERVICE}));

    readiness_topic_ = require_non_empty(
      "topics.readiness",
      params::declare_or_get<std::string>(
        *this,
        "topics.readiness",
        std::string{
          slam_lifecycle_topics::READINESS}));

    session_state_topic_ = require_non_empty(
      "topics.session_state",
      params::declare_or_get<std::string>(
        *this,
        "topics.session_state",
        std::string{
          slam_lifecycle_topics::SESSION_STATE}));

    lifecycle_state_topic_ = require_non_empty(
      "topics.lifecycle_state",
      params::declare_or_get<std::string>(
        *this,
        "topics.lifecycle_state",
        std::string{
          slam_lifecycle_topics::
          LIFECYCLE_STATE}));

    health_topic_ = require_non_empty(
      "topics.health",
      params::declare_or_get<std::string>(
        *this,
        "topics.health",
        std::string{
          slam_lifecycle_topics::HEALTH}));

    manual_workflow_state_topic_ =
      require_non_empty(
      "topics.manual_workflow_state",
      params::declare_or_get<std::string>(
        *this,
        "topics.manual_workflow_state",
        std::string{
          slam_lifecycle_topics::
          MANUAL_WORKFLOW_STATE}));
  }

  void create_interfaces()
  {
    get_state_client_ =
      create_client<GetState>(
      get_state_service_);

    lifecycle_state_publisher_ =
      create_publisher<std_msgs::msg::String>(
      lifecycle_state_topic_,
      qos::state_qos());

    health_publisher_ =
      create_publisher<std_msgs::msg::String>(
      health_topic_,
      qos::state_qos());

    manual_workflow_state_publisher_ =
      create_publisher<std_msgs::msg::String>(
      manual_workflow_state_topic_,
      qos::state_qos());

    readiness_subscription_ =
      create_subscription<std_msgs::msg::String>(
      readiness_topic_,
      qos::state_qos(),
      std::bind(
        &SlamLifecycleHealthBridgeNode::
        on_readiness,
        this,
        std::placeholders::_1));

    session_state_subscription_ =
      create_subscription<std_msgs::msg::String>(
      session_state_topic_,
      qos::state_qos(),
      std::bind(
        &SlamLifecycleHealthBridgeNode::
        on_session_state,
        this,
        std::placeholders::_1));
  }

  void on_readiness(
    const std_msgs::msg::String::ConstSharedPtr message)
  {
    latest_readiness_ = message->data;
    mapping_ready_ = message->data == "ready";

    publish_state();
  }

  void on_session_state(
    const std_msgs::msg::String::ConstSharedPtr message)
  {
    session_state_ =
      normalize_state_string(message->data);

    publish_state();
  }

  void on_timer()
  {
    service_available_ =
      get_state_client_->service_is_ready();

    const auto now = SteadyClock::now();

    if (request_pending_ &&
        pending_since_.has_value())
    {
      const double pending_age_s =
        std::chrono::duration<double>(
        now - *pending_since_).count();

      if (pending_age_s >
          response_timeout_s_)
      {
        request_pending_ = false;
        pending_since_.reset();
      }
    }

    if (service_available_ &&
        !request_pending_)
    {
      send_get_state_request();
    }

    publish_state();
  }

  void send_get_state_request()
  {
    request_pending_ = true;
    pending_since_ = SteadyClock::now();

    const std::uint64_t request_id =
      ++latest_request_id_;

    auto request =
      std::make_shared<GetState::Request>();

    get_state_client_->async_send_request(
      request,
      [this, request_id](
        rclcpp::Client<GetState>::SharedFuture future)
      {
        on_get_state_response(
          request_id,
          future);
      });
  }

  void on_get_state_response(
    std::uint64_t request_id,
    rclcpp::Client<GetState>::SharedFuture future)
  {
    if (request_id <
        latest_completed_request_id_)
    {
      return;
    }

    try {
      const auto response = future.get();

      if (!response) {
        return;
      }

      latest_completed_request_id_ =
        request_id;

      lifecycle_state_id_ =
        response->current_state.id;

      lifecycle_state_label_ =
        response->current_state.label.empty() ?
        "unknown" :
        response->current_state.label;

      response_received_ = true;
      last_response_at_ = SteadyClock::now();

      if (workflow::lifecycle_state_is_active(
          lifecycle_state_id_))
      {
        ever_active_ = true;
      }
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        get_logger(),
        "get_state response failed: %s",
        exception.what());
    }

    request_pending_ = false;
    pending_since_.reset();

    publish_state();
  }

  workflow::SlamLifecycleObservation
  build_observation() const
  {
    const auto now = SteadyClock::now();

    workflow::SlamLifecycleObservation observation;

    observation.service_available =
      service_available_;

    observation.response_received =
      response_received_;

    observation.ever_active =
      ever_active_;

    observation.state_id =
      lifecycle_state_id_;

    const double startup_age_s =
      std::chrono::duration<double>(
      now - started_at_).count();

    observation.startup_grace_expired =
      startup_age_s > startup_grace_s_;

    if (last_response_at_.has_value()) {
      const double response_age_s =
        std::chrono::duration<double>(
        now - *last_response_at_).count();

      observation.response_fresh =
        response_age_s <= response_timeout_s_;
    }

    return observation;
  }

  std::string determine_health_reason(
    const workflow::SlamLifecycleObservation &
    observation) const
  {
    if (!observation.response_received) {
      return observation.service_available ?
             "waiting_for_response" :
             "service_unavailable";
    }

    if (!observation.response_fresh) {
      return "lifecycle_response_stale";
    }

    if (observation.state_id ==
        workflow::lifecycle_state_id::
        ERROR_PROCESSING)
    {
      return "lifecycle_error_processing";
    }

    if (observation.state_id ==
        workflow::lifecycle_state_id::
        FINALIZED)
    {
      return "lifecycle_finalized";
    }

    return lifecycle_state_label_;
  }

  void publish_state()
  {
    const auto observation =
      build_observation();

    const auto workflow_state =
      workflow::evaluate_manual_workflow_state(
      observation,
      mapping_ready_,
      session_state_);

    const bool lifecycle_healthy =
      observation.response_fresh &&
      workflow::lifecycle_state_is_active(
        observation.state_id);

    std_msgs::msg::String lifecycle_message;
    lifecycle_message.data =
      lifecycle_state_label_;

    lifecycle_state_publisher_->publish(
      lifecycle_message);

    std_msgs::msg::String workflow_message;
    workflow_message.data =
      std::string{
      workflow::to_string(workflow_state)};

    manual_workflow_state_publisher_->publish(
      workflow_message);

    std::ostringstream health_stream;

    health_stream
      << std::boolalpha
      << "{"
      << "\"service_available\":"
      << observation.service_available
      << ",\"response_received\":"
      << observation.response_received
      << ",\"response_fresh\":"
      << observation.response_fresh
      << ",\"healthy\":"
      << lifecycle_healthy
      << ",\"state_id\":"
      << static_cast<unsigned int>(
        observation.state_id)
      << ",\"state\":\""
      << lifecycle_state_label_
      << "\",\"reason\":\""
      << determine_health_reason(observation)
      << "\",\"mapping_ready\":"
      << mapping_ready_
      << ",\"readiness\":\""
      << latest_readiness_
      << "\",\"session_state\":\""
      << session_state_
      << "\",\"manual_workflow_state\":\""
      << workflow_message.data
      << "\"}";

    std_msgs::msg::String health_message;
    health_message.data = health_stream.str();

    health_publisher_->publish(health_message);
  }

  double query_period_s_{0.50};
  double response_timeout_s_{1.50};
  double startup_grace_s_{5.0};

  std::string get_state_service_;
  std::string readiness_topic_;
  std::string session_state_topic_;
  std::string lifecycle_state_topic_;
  std::string health_topic_;
  std::string manual_workflow_state_topic_;

  bool service_available_{false};
  bool response_received_{false};
  bool request_pending_{false};
  bool mapping_ready_{false};
  bool ever_active_{false};

  std::uint8_t lifecycle_state_id_{
    workflow::lifecycle_state_id::UNKNOWN};

  std::string lifecycle_state_label_{"unknown"};
  std::string latest_readiness_{
    "not_ready: unavailable"};
  std::string session_state_{"idle"};

  std::uint64_t latest_request_id_{0};
  std::uint64_t latest_completed_request_id_{0};

  const SteadyClock::time_point started_at_;

  std::optional<SteadyClock::time_point>
    last_response_at_;

  std::optional<SteadyClock::time_point>
    pending_since_;

  rclcpp::Client<GetState>::SharedPtr
    get_state_client_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    lifecycle_state_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    health_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    manual_workflow_state_publisher_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    readiness_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    session_state_subscription_;

  rclcpp::TimerBase::SharedPtr query_timer_;
};

}  // namespace savo_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<
        savo_mapping::
        SlamLifecycleHealthBridgeNode>());
  } catch (const std::exception & exception) {
    std::cerr
      << "slam_lifecycle_health_bridge_node "
      << "failed: "
      << exception.what()
      << '\n';

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
