// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "savo_msgs/msg/location_event.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"

#include "savo_mapping/mapping_mode.hpp"
#include "savo_mapping/qos_profiles.hpp"
#include "savo_mapping/session_state.hpp"
#include "savo_mapping/topic_names.hpp"

namespace savo_mapping
{
namespace
{

using Json = nlohmann::json;
using LocationEvent = savo_msgs::msg::LocationEvent;

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

[[nodiscard]] std::chrono::nanoseconds period_from_hz(const double hz)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / hz));
}

[[nodiscard]] std::string location_event_name(const std::uint8_t event_type)
{
  switch (event_type) {
    case LocationEvent::EVENT_CANDIDATE_REGISTERED:
      return "candidate_registered";
    case LocationEvent::EVENT_LOCATION_APPROVED:
      return "location_approved";
    case LocationEvent::EVENT_CANDIDATE_REJECTED:
      return "candidate_rejected";
    case LocationEvent::EVENT_LOCATION_UPDATED:
      return "location_updated";
    case LocationEvent::EVENT_LOCATION_ENABLED:
      return "location_enabled";
    case LocationEvent::EVENT_LOCATION_DISABLED:
      return "location_disabled";
    case LocationEvent::EVENT_LOCATION_RETIRED:
      return "location_retired";
    case LocationEvent::EVENT_IMPORT_COMPLETED:
      return "import_completed";
    case LocationEvent::EVENT_STORAGE_DEGRADED:
      return "storage_degraded";
    default:
      return "unknown";
  }
}

[[nodiscard]] bool valid_head_confirmation(
  const Json & payload,
  std::string & rejection_reason)
{
  if (!payload.is_object()) {
    rejection_reason = "head_confirmation_must_be_json_object";
    return false;
  }

  const auto state = payload.value("state", std::string{});
  if (state != "confirmed" && state != "rejected") {
    rejection_reason = "head_confirmation_state_invalid";
    return false;
  }

  const auto family = payload.value("family", std::string{});
  if (family.empty()) {
    rejection_reason = "head_confirmation_family_required";
    return false;
  }

  if (!payload.contains("tag_id") || !payload.at("tag_id").is_number_integer() ||
    payload.at("tag_id").get<std::int32_t>() < 0)
  {
    rejection_reason = "head_confirmation_tag_id_invalid";
    return false;
  }

  if (payload.contains("confidence")) {
    if (!payload.at("confidence").is_number()) {
      rejection_reason = "head_confirmation_confidence_invalid";
      return false;
    }

    const double confidence = payload.at("confidence").get<double>();
    if (!std::isfinite(confidence) || confidence < 0.0 || confidence > 1.0) {
      rejection_reason = "head_confirmation_confidence_out_of_range";
      return false;
    }
  }

  rejection_reason.clear();
  return true;
}

}  // namespace

class SemanticLandmarkBridgeNode final : public rclcpp::Node
{
public:
  SemanticLandmarkBridgeNode()
  : Node("semantic_landmark_bridge_node")
  {
    declare_parameters();
    validate_parameters();
    create_interfaces();
    publish_status("ready");

    RCLCPP_INFO(
      get_logger(),
      "semantic landmark bridge ready: head=%s locations=%s events=%s",
      head_confirmations_topic_.c_str(),
      location_events_topic_.c_str(),
      semantic_events_topic_.c_str());
  }

private:
  void declare_parameters()
  {
    head_confirmations_topic_ = declare_parameter<std::string>(
      "head_confirmations_topic",
      std::string{topics::HEAD_SEMANTIC_CONFIRMATIONS});
    location_events_topic_ = declare_parameter<std::string>(
      "location_events_topic", std::string{topics::LOCATION_EVENTS});
    mapping_mode_topic_ = declare_parameter<std::string>(
      "mapping_mode_topic", std::string{topics::MODE});
    session_state_topic_ = declare_parameter<std::string>(
      "session_state_topic", std::string{topics::SESSION_STATE});
    semantic_events_topic_ = declare_parameter<std::string>(
      "semantic_events_topic", std::string{topics::SEMANTIC_EVENTS});
    status_topic_ = declare_parameter<std::string>(
      "status_topic", std::string{topics::SEMANTIC_STATUS});
    heartbeat_topic_ = declare_parameter<std::string>(
      "heartbeat_topic", std::string{topics::SEMANTIC_HEARTBEAT});
    registration_action_ = declare_parameter<std::string>(
      "registration_action", "/savo_mapping/locations/register");

    publish_head_rejections_ = declare_parameter<bool>(
      "publish_head_rejections", false);
    require_active_mapping_session_ = declare_parameter<bool>(
      "require_active_mapping_session", true);
    deduplication_window_s_ = declare_parameter<double>(
      "deduplication_window_s", 3.0);
    status_publish_hz_ = declare_parameter<double>(
      "status_publish_hz", 1.0);
    heartbeat_publish_hz_ = declare_parameter<double>(
      "heartbeat_publish_hz", 2.0);
  }

  void validate_parameters()
  {
    if (head_confirmations_topic_.empty() || location_events_topic_.empty() ||
      mapping_mode_topic_.empty() || session_state_topic_.empty() ||
      semantic_events_topic_.empty() || status_topic_.empty() ||
      heartbeat_topic_.empty() || registration_action_.empty())
    {
      throw std::invalid_argument(
              "semantic landmark bridge topic/action names must not be empty");
    }

    if (!std::isfinite(deduplication_window_s_) ||
      deduplication_window_s_ < 0.0 ||
      !std::isfinite(status_publish_hz_) || status_publish_hz_ <= 0.0 ||
      !std::isfinite(heartbeat_publish_hz_) || heartbeat_publish_hz_ <= 0.0)
    {
      throw std::invalid_argument(
              "semantic landmark bridge timing parameters are invalid");
    }

    deduplication_window_ns_ = static_cast<std::int64_t>(
      deduplication_window_s_ *
      static_cast<double>(kNanosecondsPerSecond));
  }

  void create_interfaces()
  {
    semantic_event_publisher_ = create_publisher<std_msgs::msg::String>(
      semantic_events_topic_, qos::event_qos());
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      status_topic_, qos::state_qos());
    heartbeat_publisher_ = create_publisher<std_msgs::msg::UInt64>(
      heartbeat_topic_, qos::status_qos());

    head_confirmation_subscription_ =
      create_subscription<std_msgs::msg::String>(
      head_confirmations_topic_,
      qos::event_qos(),
      std::bind(
        &SemanticLandmarkBridgeNode::on_head_confirmation,
        this,
        std::placeholders::_1));

    location_event_subscription_ = create_subscription<LocationEvent>(
      location_events_topic_,
      rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile(),
      std::bind(
        &SemanticLandmarkBridgeNode::on_location_event,
        this,
        std::placeholders::_1));

    mapping_mode_subscription_ = create_subscription<std_msgs::msg::String>(
      mapping_mode_topic_,
      qos::state_qos(),
      std::bind(
        &SemanticLandmarkBridgeNode::on_mapping_mode,
        this,
        std::placeholders::_1));

    session_state_subscription_ = create_subscription<std_msgs::msg::String>(
      session_state_topic_,
      qos::state_qos(),
      std::bind(
        &SemanticLandmarkBridgeNode::on_session_state,
        this,
        std::placeholders::_1));

    status_timer_ = create_wall_timer(
      period_from_hz(status_publish_hz_),
      std::bind(
        &SemanticLandmarkBridgeNode::publish_periodic_status,
        this));
    heartbeat_timer_ = create_wall_timer(
      period_from_hz(heartbeat_publish_hz_),
      std::bind(
        &SemanticLandmarkBridgeNode::publish_heartbeat,
        this));
  }

  [[nodiscard]] bool active_mapping_session() const
  {
    const auto mode = mapping_mode_from_string(mapping_mode_);
    const auto session = session_state_from_string(session_state_);

    if (!mode.has_value() || !session.has_value()) {
      return false;
    }

    return mode.value() != MappingMode::MonitorOnly &&
           can_accept_mapping_data(session.value());
  }

  void on_mapping_mode(const std_msgs::msg::String::SharedPtr message)
  {
    mapping_mode_ = message->data;
    if (!mapping_mode_from_string(mapping_mode_).has_value()) {
      last_reason_ = "invalid_mapping_mode_state";
    }
  }

  void on_session_state(const std_msgs::msg::String::SharedPtr message)
  {
    session_state_ = message->data;
    if (!session_state_from_string(session_state_).has_value()) {
      last_reason_ = "invalid_mapping_session_state";
    }
  }

  [[nodiscard]] bool duplicate_confirmation(
    const std::string & family,
    const std::int32_t tag_id,
    const std::int64_t now_ns)
  {
    const std::string key = family + ":" + std::to_string(tag_id);
    const auto found = last_confirmation_ns_.find(key);

    if (found != last_confirmation_ns_.end()) {
      const std::int64_t age_ns = now_ns - found->second;
      if (age_ns >= 0 && age_ns <= deduplication_window_ns_) {
        return true;
      }
    }

    last_confirmation_ns_[key] = now_ns;
    return false;
  }

  void on_head_confirmation(const std_msgs::msg::String::SharedPtr message)
  {
    ++head_messages_received_;

    const Json payload = Json::parse(message->data, nullptr, false);
    std::string rejection_reason;
    if (payload.is_discarded() ||
      !valid_head_confirmation(payload, rejection_reason))
    {
      ++invalid_head_messages_;
      last_reason_ = payload.is_discarded() ?
        "head_confirmation_json_parse_failed" : rejection_reason;
      return;
    }

    const std::string state = payload.at("state").get<std::string>();
    if (state == "rejected" && !publish_head_rejections_) {
      ++suppressed_head_messages_;
      last_reason_ = "head_rejection_suppressed";
      return;
    }

    if (state == "confirmed" && require_active_mapping_session_ &&
      !active_mapping_session())
    {
      ++suppressed_head_messages_;
      last_reason_ = "head_confirmation_outside_active_mapping_session";
      return;
    }

    const std::string family = payload.at("family").get<std::string>();
    const std::int32_t tag_id = payload.at("tag_id").get<std::int32_t>();
    const std::int64_t now_ns = now().nanoseconds();

    if (state == "confirmed" &&
      duplicate_confirmation(family, tag_id, now_ns))
    {
      ++duplicate_head_messages_;
      last_reason_ = "duplicate_head_confirmation_suppressed";
      return;
    }

    Json event;
    event["schema_version"] = 1;
    event["event_sequence"] = ++event_sequence_;
    event["stamp_ns"] = now_ns;
    event["event_type"] = state == "confirmed" ?
      "apriltag_candidate_observed" : "apriltag_confirmation_rejected";
    event["source"] = "savo_mapping.semantic_landmark_bridge";
    event["mapping_mode"] = mapping_mode_;
    event["session_state"] = session_state_;
    event["operator_action_required"] = state == "confirmed";
    event["registration_action"] = registration_action_;
    event["evidence_authority"] = "savo_head";
    event["persistence_authority"] = "savo_locations";
    event["evidence_is_hint_only"] = true;
    event["head_confirmation"] = payload;

    publish_event(event, "head_semantic_event_published");
  }

  void on_location_event(const LocationEvent::SharedPtr message)
  {
    ++location_events_received_;

    Json event;
    event["schema_version"] = 1;
    event["event_sequence"] = ++event_sequence_;
    event["stamp_ns"] = now().nanoseconds();
    event["event_type"] = "location_registry_event";
    event["registry_event_type"] = location_event_name(message->event_type);
    event["registry_event_sequence"] = message->event_sequence;
    event["source"] = "savo_mapping.semantic_landmark_bridge";
    event["persistence_authority"] = "savo_locations";
    event["mapping_mode"] = mapping_mode_;
    event["session_state"] = session_state_;
    event["candidate_id"] = message->candidate_id;
    event["location_id"] = message->location_id;
    event["entity_revision"] = message->entity_revision;
    event["actor_id"] = message->actor_id;
    event["reason"] = message->reason;
    event["operator_action_required"] =
      message->event_type == LocationEvent::EVENT_CANDIDATE_REGISTERED;

    publish_event(event, "location_registry_event_published");
  }

  void publish_event(const Json & event, std::string reason)
  {
    std_msgs::msg::String message;
    message.data = event.dump();
    semantic_event_publisher_->publish(message);
    ++events_published_;
    last_reason_ = std::move(reason);
  }

  void publish_periodic_status()
  {
    publish_status(last_reason_);
  }

  void publish_status(const std::string & reason)
  {
    Json status;
    status["schema_version"] = 1;
    status["ready"] = true;
    status["lifecycle"] = "active";
    status["reason"] = reason;
    status["mapping_mode"] = mapping_mode_;
    status["session_state"] = session_state_;
    status["active_mapping_session"] = active_mapping_session();
    status["require_active_mapping_session"] = require_active_mapping_session_;
    status["head_messages_received"] = head_messages_received_;
    status["location_events_received"] = location_events_received_;
    status["events_published"] = events_published_;
    status["invalid_head_messages"] = invalid_head_messages_;
    status["suppressed_head_messages"] = suppressed_head_messages_;
    status["duplicate_head_messages"] = duplicate_head_messages_;
    status["registration_action"] = registration_action_;

    std_msgs::msg::String message;
    message.data = status.dump();
    status_publisher_->publish(message);
  }

  void publish_heartbeat()
  {
    std_msgs::msg::UInt64 message;
    message.data = ++heartbeat_sequence_;
    heartbeat_publisher_->publish(message);
  }

  std::string head_confirmations_topic_{};
  std::string location_events_topic_{};
  std::string mapping_mode_topic_{};
  std::string session_state_topic_{};
  std::string semantic_events_topic_{};
  std::string status_topic_{};
  std::string heartbeat_topic_{};
  std::string registration_action_{};

  bool publish_head_rejections_{false};
  bool require_active_mapping_session_{true};
  double deduplication_window_s_{3.0};
  double status_publish_hz_{1.0};
  double heartbeat_publish_hz_{2.0};
  std::int64_t deduplication_window_ns_{3 * kNanosecondsPerSecond};

  std::string mapping_mode_{"unknown"};
  std::string session_state_{"unknown"};
  std::string last_reason_{"starting"};
  std::unordered_map<std::string, std::int64_t> last_confirmation_ns_{};

  std::uint64_t event_sequence_{0U};
  std::uint64_t heartbeat_sequence_{0U};
  std::uint64_t head_messages_received_{0U};
  std::uint64_t location_events_received_{0U};
  std::uint64_t events_published_{0U};
  std::uint64_t invalid_head_messages_{0U};
  std::uint64_t suppressed_head_messages_{0U};
  std::uint64_t duplicate_head_messages_{0U};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr semantic_event_publisher_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_{};
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr heartbeat_publisher_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    head_confirmation_subscription_{};
  rclcpp::Subscription<LocationEvent>::SharedPtr location_event_subscription_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    mapping_mode_subscription_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    session_state_subscription_{};
  rclcpp::TimerBase::SharedPtr status_timer_{};
  rclcpp::TimerBase::SharedPtr heartbeat_timer_{};
};

}  // namespace savo_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<savo_mapping::SemanticLandmarkBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
