#include "savo_mapping/exploration_mode.hpp"
#include "savo_mapping/mapping_mode.hpp"
#include "savo_mapping/qos_profiles.hpp"
#include "savo_mapping/session_state.hpp"
#include "savo_mapping/topic_names.hpp"
#include "savo_mapping/workflow_authority.hpp"
#include "savo_mapping/workflow_phase.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cctype>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace savo_mapping
{
namespace
{

std::string normalize(std::string_view input)
{
  const auto first = std::find_if_not(
    input.begin(), input.end(), [](unsigned char value) {
      return std::isspace(value) != 0;
    });

  const auto last = std::find_if_not(
    input.rbegin(), input.rend(), [](unsigned char value) {
      return std::isspace(value) != 0;
    }).base();

  if (first >= last) {
    return {};
  }

  std::string output{first, last};

  std::transform(
    output.begin(),
    output.end(),
    output.begin(),
    [](unsigned char value) {
      return value == '-' ?
             '_' :
             static_cast<char>(std::tolower(value));
    });

  return output;
}

bool handoff_state_is_active(std::string_view state)
{
  return state == "waiting_for_savo_nav" ||
         state == "sending" ||
         state == "accepted" ||
         state == "executing" ||
         state == "canceling";
}

bool handoff_state_is_known(std::string_view state)
{
  return state == "idle" ||
         handoff_state_is_active(state) ||
         state == "succeeded" ||
         state == "rejected" ||
         state == "aborted" ||
         state == "canceled" ||
         state == "timed_out" ||
         state == "error";
}

}  // namespace

class MappingModeManagerNode final : public rclcpp::Node
{
public:
  MappingModeManagerNode()
  : Node("mapping_mode_manager_node")
  {
    configure();
    create_interfaces();
    publish_state();

    RCLCPP_INFO(
      get_logger(),
      "mapping mode authority started; movement execution=false "
      "navigation execution=false");
  }

private:
  using StringPublisher =
    rclcpp::Publisher<std_msgs::msg::String>;

  void configure()
  {
    const auto strategy_text =
      declare_parameter<std::string>(
      "authority.default_autonomous_strategy",
      "frontier");

    const auto strategy =
      exploration_mode_from_string(
      strategy_text);

    if (!strategy.has_value() ||
      (strategy.value() != ExplorationMode::Frontier &&
      strategy.value() != ExplorationMode::Scan360 &&
      strategy.value() != ExplorationMode::Coverage))
    {
      throw std::invalid_argument(
              "default autonomous strategy must be frontier, "
              "scan360, or coverage");
    }

    default_autonomous_strategy_ =
      strategy.value();

    const auto initial_mode_text =
      declare_parameter<std::string>(
      "authority.initial_mode",
      "monitor_only");

    const auto initial_mode =
      mapping_mode_from_string(
      initial_mode_text);

    if (!initial_mode.has_value() ||
      (initial_mode.value() !=
      MappingMode::MonitorOnly &&
      initial_mode.value() !=
      MappingMode::Manual))
    {
      throw std::invalid_argument(
              "initial mode must be monitor_only or manual");
    }

    const auto initial_session_text =
      declare_parameter<std::string>(
      "authority.initial_session_state",
      "idle");

    const auto initial_session =
      session_state_from_string(
      initial_session_text);

    if (!initial_session.has_value() ||
      (initial_session.value() !=
      SessionState::Idle &&
      initial_session.value() !=
      SessionState::Active))
    {
      throw std::invalid_argument(
              "initial session state must be idle or active");
    }

    if (
      initial_mode.value() ==
      MappingMode::Manual &&
      initial_session.value() !=
      SessionState::Active)
    {
      throw std::invalid_argument(
              "manual initial mode requires active session");
    }

    state_.mode =
      initial_mode.value();

    state_.exploration_mode =
      ExplorationMode::Idle;

    state_.workflow_phase =
      workflow::workflow_phase_for(
      state_.mode,
      state_.exploration_mode);

    state_.session_state =
      initial_session.value();

    if (!workflow::
      is_workflow_authority_state_consistent(
        state_))
    {
      throw std::invalid_argument(
              "initial workflow authority state "
              "is inconsistent");
    }
  }

  void create_interfaces()
  {
    using std::placeholders::_1;

    mode_publisher_ =
      create_publisher<std_msgs::msg::String>(
      std::string{topics::MODE},
      qos::state_qos());

    exploration_mode_publisher_ =
      create_publisher<std_msgs::msg::String>(
      std::string{topics::EXPLORATION_MODE},
      qos::state_qos());

    workflow_phase_publisher_ =
      create_publisher<std_msgs::msg::String>(
      std::string{topics::WORKFLOW_PHASE},
      qos::state_qos());

    session_state_publisher_ =
      create_publisher<std_msgs::msg::String>(
      std::string{topics::SESSION_STATE},
      qos::state_qos());

    status_publisher_ =
      create_publisher<std_msgs::msg::String>(
      std::string{topics::MODE_MANAGER_STATUS},
      qos::status_qos());

    mode_command_subscription_ =
      subscribe_string(
      topics::MODE_CMD,
      std::bind(
        &MappingModeManagerNode::on_mode_command,
        this,
        _1),
      qos::command_qos());

    start_session_subscription_ =
      subscribe_string(
      topics::START_SESSION_CMD,
      std::bind(
        &MappingModeManagerNode::on_start_session,
        this,
        _1),
      qos::command_qos());

    stop_session_subscription_ =
      subscribe_string(
      topics::STOP_SESSION_CMD,
      std::bind(
        &MappingModeManagerNode::on_stop_session,
        this,
        _1),
      qos::command_qos());

    cancel_session_subscription_ =
      subscribe_string(
      topics::CANCEL_SESSION_CMD,
      std::bind(
        &MappingModeManagerNode::on_cancel_session,
        this,
        _1),
      qos::command_qos());

    readiness_subscription_ =
      subscribe_string(
      topics::READINESS,
      std::bind(
        &MappingModeManagerNode::on_readiness,
        this,
        _1),
      qos::state_qos());

    handoff_state_subscription_ =
      subscribe_string(
      topics::EXPLORATION_GOAL_STATE,
      std::bind(
        &MappingModeManagerNode::on_handoff_state,
        this,
        _1),
      qos::state_qos());

    safety_stop_subscription_ =
      create_subscription<std_msgs::msg::Bool>(
      std::string{topics::SAFETY_STOP},
      qos::status_qos(),
      std::bind(
        &MappingModeManagerNode::on_safety_stop,
        this,
        _1));
  }

  template<typename CallbackT>
  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
  subscribe_string(
    std::string_view topic,
    CallbackT callback,
    const rclcpp::QoS & profile)
  {
    return create_subscription<
      std_msgs::msg::String>(
      std::string{topic},
      profile,
      std::move(callback));
  }

  std::optional<
    workflow::ModeChangeRequest>
  parse_mode_request(
    std::string_view input) const
  {
    const std::string command =
      normalize(input);

    const auto separator =
      command.find(':');

    const auto mode =
      mapping_mode_from_string(
      command.substr(0, separator));

    if (!mode.has_value()) {
      return std::nullopt;
    }

    workflow::ModeChangeRequest request;
    request.target_mode = mode.value();
    request.autonomous_strategy =
      ExplorationMode::Idle;

    if (request.target_mode !=
      MappingMode::Autonomous)
    {
      return request;
    }

    if (separator == std::string::npos) {
      request.autonomous_strategy =
        default_autonomous_strategy_;

      return request;
    }

    const auto strategy =
      exploration_mode_from_string(
      command.substr(separator + 1));

    if (!strategy.has_value()) {
      return std::nullopt;
    }

    request.autonomous_strategy =
      strategy.value();

    return request;
  }

  workflow::ModeChangeRequest
  current_request() const
  {
    return workflow::ModeChangeRequest{
      state_.mode,
      state_.exploration_mode
    };
  }

  void on_mode_command(
    const std_msgs::msg::String::
    ConstSharedPtr message)
  {
    const auto request =
      parse_mode_request(message->data);

    if (!request.has_value()) {
      update_result(
        workflow::AuthorityDisposition::
        Rejected,
        "rejected: invalid_mode_command",
        false);

      return;
    }

    {
      std::lock_guard<std::mutex> lock(
        mutex_);

      const auto decision =
        workflow::evaluate_mode_change(
        state_,
        inputs_,
        request.value());

      last_disposition_ =
        decision.disposition;

      last_reason_ =
        decision.reason;

      cancel_active_exploration_ =
        decision.cancel_active_exploration;

      if (decision.disposition ==
        workflow::AuthorityDisposition::
        Accepted)
      {
        state_ = decision.next_state;
      }
    }

    publish_state();
  }

  void on_start_session(
    const std_msgs::msg::String::
    ConstSharedPtr)
  {
    {
      std::lock_guard<std::mutex> lock(
        mutex_);

      if (inputs_.exploration_goal_active) {
        reject(
          "rejected: exploration_goal_active");
      } else {
        switch (state_.session_state) {
          case SessionState::Active:
            accept(
              "accepted: session_already_active");
            break;

          case SessionState::Starting:
          case SessionState::Paused:
          case SessionState::Saving:
            reject(
              "rejected: "
              "session_transition_in_progress");
            break;

          default:
            state_.mode =
              MappingMode::MonitorOnly;

            state_.exploration_mode =
              ExplorationMode::Idle;

            state_.workflow_phase =
              WorkflowPhase::Idle;

            state_.session_state =
              SessionState::Active;

            accept(
              "accepted: session_started");
            break;
        }
      }
    }

    publish_state();
  }

  void on_stop_session(
    const std_msgs::msg::String::
    ConstSharedPtr)
  {
    end_session(
      SessionState::Idle,
      "accepted: session_stopped");
  }

  void on_cancel_session(
    const std_msgs::msg::String::
    ConstSharedPtr)
  {
    end_session(
      SessionState::Cancelled,
      "accepted: session_cancelled");
  }

  void end_session(
    SessionState final_state,
    std::string reason)
  {
    {
      std::lock_guard<std::mutex> lock(
        mutex_);

      if (inputs_.exploration_goal_active) {
        last_disposition_ =
          workflow::AuthorityDisposition::
          CancelRequired;

        last_reason_ =
          "cancel_required: "
          "active_navigation_goal";

        cancel_active_exploration_ = true;
      } else {
        state_.mode =
          MappingMode::MonitorOnly;

        state_.exploration_mode =
          ExplorationMode::Idle;

        state_.workflow_phase =
          WorkflowPhase::Idle;

        state_.session_state =
          final_state;

        accept(std::move(reason));
      }
    }

    publish_state();
  }

  void on_readiness(
    const std_msgs::msg::String::
    ConstSharedPtr message)
  {
    update_runtime(
      normalize(message->data) == "ready",
      std::nullopt);
  }

  void on_safety_stop(
    const std_msgs::msg::Bool::
    ConstSharedPtr message)
  {
    update_runtime(
      std::nullopt,
      message->data);
  }

  void update_runtime(
    std::optional<bool> mapping_ready,
    std::optional<bool> safety_stop)
  {
    {
      std::lock_guard<std::mutex> lock(
        mutex_);

      if (mapping_ready.has_value()) {
        inputs_.mapping_ready =
          mapping_ready.value();
      }

      if (safety_stop.has_value()) {
        inputs_.safety_stop_active =
          safety_stop.value();
      }

      const auto decision =
        workflow::evaluate_mode_change(
        state_,
        inputs_,
        current_request());

      last_disposition_ =
        decision.disposition;

      last_reason_ =
        decision.reason;

      cancel_active_exploration_ =
        decision.cancel_active_exploration;
    }

    publish_state();
  }

  void on_handoff_state(
    const std_msgs::msg::String::
    ConstSharedPtr message)
  {
    const std::string handoff_state =
      normalize(message->data);

    {
      std::lock_guard<std::mutex> lock(
        mutex_);

      if (!handoff_state_is_known(
          handoff_state))
      {
        reject(
          "rejected: invalid_handoff_state");
      } else {
        inputs_.navigation_handoff_ready =
          true;

        inputs_.exploration_goal_active =
          handoff_state_is_active(
          handoff_state);

        last_handoff_state_ =
          handoff_state;

        const auto decision =
          workflow::evaluate_mode_change(
          state_,
          inputs_,
          current_request());

        last_disposition_ =
          decision.disposition;

        last_reason_ =
          "handoff_state: " +
          handoff_state;

        cancel_active_exploration_ =
          decision.cancel_active_exploration;
      }
    }

    publish_state();
  }

  void accept(std::string reason)
  {
    last_disposition_ =
      workflow::AuthorityDisposition::
      Accepted;

    last_reason_ =
      std::move(reason);

    cancel_active_exploration_ =
      false;
  }

  void reject(std::string reason)
  {
    last_disposition_ =
      workflow::AuthorityDisposition::
      Rejected;

    last_reason_ =
      std::move(reason);

    cancel_active_exploration_ =
      false;
  }

  void update_result(
    workflow::AuthorityDisposition
    disposition,
    std::string reason,
    bool cancel_active_exploration)
  {
    {
      std::lock_guard<std::mutex> lock(
        mutex_);

      last_disposition_ =
        disposition;

      last_reason_ =
        std::move(reason);

      cancel_active_exploration_ =
        cancel_active_exploration;
    }

    publish_state();
  }

  void publish_state()
  {
    std::lock_guard<std::mutex> lock(
      mutex_);

    const auto decision =
      workflow::evaluate_mode_change(
      state_,
      inputs_,
      current_request());

    publish(
      mode_publisher_,
      to_string(state_.mode));

    publish(
      exploration_mode_publisher_,
      to_string(state_.exploration_mode));

    publish(
      workflow_phase_publisher_,
      to_string(state_.workflow_phase));

    publish(
      session_state_publisher_,
      to_string(state_.session_state));

    std::ostringstream status;

    status
      << std::boolalpha
      << "mode="
      << to_string(state_.mode)
      << " exploration="
      << to_string(
           state_.exploration_mode)
      << " phase="
      << to_string(
           state_.workflow_phase)
      << " session="
      << to_string(
           state_.session_state)
      << " disposition="
      << workflow::to_string(
           last_disposition_)
      << " mapping_ready="
      << inputs_.mapping_ready
      << " handoff_ready="
      << inputs_.
      navigation_handoff_ready
      << " safety_stop="
      << inputs_.safety_stop_active
      << " goal_active="
      << inputs_.exploration_goal_active
      << " cancel_active_exploration="
      << cancel_active_exploration_
      << " movement_authorized="
      << decision.movement_authorized
      << " frontier_enabled="
      << workflow::
      is_frontier_runtime_enabled(
        decision)
      << " handoff_state="
      << last_handoff_state_
      << " reason=\""
      << last_reason_
      << "\"";

    publish(
      status_publisher_,
      status.str());
  }

  static void publish(
    const StringPublisher::SharedPtr &
    publisher,
    std::string_view text)
  {
    std_msgs::msg::String message;
    message.data = text;
    publisher->publish(message);
  }

  ExplorationMode
    default_autonomous_strategy_{
    ExplorationMode::Frontier};

  std::mutex mutex_;

  workflow::WorkflowAuthorityState
    state_{};

  workflow::WorkflowAuthorityInputs
    inputs_{};

  workflow::AuthorityDisposition
    last_disposition_{
    workflow::AuthorityDisposition::
    Accepted};

  bool cancel_active_exploration_{
    false};

  std::string last_handoff_state_{
    "unavailable"};

  std::string last_reason_{
    "accepted: startup"};

  StringPublisher::SharedPtr
    mode_publisher_;

  StringPublisher::SharedPtr
    exploration_mode_publisher_;

  StringPublisher::SharedPtr
    workflow_phase_publisher_;

  StringPublisher::SharedPtr
    session_state_publisher_;

  StringPublisher::SharedPtr
    status_publisher_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    mode_command_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    start_session_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    stop_session_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    cancel_session_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    readiness_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    handoff_state_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::Bool>::SharedPtr
    safety_stop_subscription_;
};

}  // namespace savo_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<
        savo_mapping::
        MappingModeManagerNode>());
  } catch (
    const std::exception & exception)
  {
    std::cerr
      << "mapping_mode_manager_node "
      << "failed: "
      << exception.what()
      << '\n';

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
