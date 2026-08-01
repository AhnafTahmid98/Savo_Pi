// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"

#include "savo_observer/observer_contract.hpp"
#include "savo_observer/telemetry_freshness.hpp"
#include "savo_observer/telemetry_snapshot.hpp"

namespace
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using std_msgs::msg::Bool;
using std_msgs::msg::String;
using std_msgs::msg::UInt64;

std::int64_t MonotonicMilliseconds()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string Lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(),
    [](const unsigned char character) {return static_cast<char>(std::tolower(character));});
  return value;
}

bool Failed(const std::string & detail)
{
  const auto normalized = Lower(detail);
  return normalized == "failed" || normalized == "fault" || normalized == "error" ||
         normalized == "blocked" || normalized.find("\"state\":\"failed\"") != std::string::npos ||
         normalized.find("\"state\":\"blocked\"") != std::string::npos ||
         normalized.find("\"healthy\":false") != std::string::npos;
}

bool Degraded(const std::string & detail)
{
  const auto normalized = Lower(detail);
  return normalized == "degraded" || normalized.find("degraded") != std::string::npos ||
         normalized.find("warning") != std::string::npos;
}

class ObserverTelemetryNode final : public rclcpp::Node
{
public:
  ObserverTelemetryNode()
  : Node("observer_telemetry_node"), freshness_(LoadPolicies())
  {
    const auto output_namespace = declare_parameter<std::string>(
      "output_namespace", "/savo_observer");
    const auto validation = savo_observer::ValidateOutputNamespace(output_namespace);
    if (!validation.empty()) {
      throw std::invalid_argument(validation);
    }
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 1.0);
    if (publish_rate_hz_ <= 0.0 || publish_rate_hz_ > 10.0) {
      throw std::invalid_argument("publish_rate_hz must be in (0, 10]");
    }

    const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
    const auto stream_qos = rclcpp::QoS(10).reliable();
    state_publisher_ = create_publisher<String>(output_namespace + "/state", state_qos);
    connected_publisher_ = create_publisher<Bool>(output_namespace + "/connected", state_qos);
    heartbeat_publisher_ = create_publisher<UInt64>(output_namespace + "/heartbeat", stream_qos);
    diagnostics_publisher_ = create_publisher<DiagnosticArray>(
      output_namespace + "/diagnostics", stream_qos);
    telemetry_publisher_ = create_publisher<String>(output_namespace + "/telemetry", state_qos);
    alerts_publisher_ = create_publisher<String>(output_namespace + "/alerts", state_qos);

    ConfigureSubscriptions();
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      [this]() {PublishSnapshot();});
    PublishSnapshot();
  }

private:
  std::vector<savo_observer::FreshnessPolicy> LoadPolicies()
  {
    const auto default_timeout = declare_parameter<std::int64_t>("freshness_timeout_ms", 5000);
    if (default_timeout <= 0) {
      throw std::invalid_argument("freshness_timeout_ms must be positive");
    }
    const std::vector<std::pair<std::string, bool>> defaults{
      {"core", true}, {"edge", false}, {"robot_mode", false}, {"safety", false},
      {"localization", false}, {"mapping", false}, {"navigation", false},
      {"power", false}, {"lidar", false}, {"speech", false}, {"bridge", false},
      {"active_map", false}};
    std::vector<savo_observer::FreshnessPolicy> policies;
    policies.reserve(defaults.size());
    for (const auto & item : defaults) {
      const bool enabled = declare_parameter<bool>("enable_" + item.first, true);
      const bool required = declare_parameter<bool>("require_" + item.first, item.second);
      const auto timeout = declare_parameter<std::int64_t>(
        item.first + "_freshness_timeout_ms", default_timeout);
      policies.push_back({item.first, enabled, required, timeout});
    }
    return policies;
  }

  void Subscribe(const std::string & key, const std::string & default_topic)
  {
    const auto topic = declare_parameter<std::string>(key + "_topic", default_topic);
    if (!savo_observer::IsAbsoluteTopic(topic) || savo_observer::IsProhibitedInterface(topic)) {
      throw std::invalid_argument("observer input topic is invalid or prohibited: " + topic);
    }
    string_subscriptions_.push_back(create_subscription<String>(
      topic, rclcpp::QoS(10).reliable(),
        [this, key](const String::SharedPtr message) {
          const bool accepted = freshness_.Observe(
          key, MonotonicMilliseconds(), message->data, Degraded(message->data),
          Failed(message->data));
          if (!accepted) {
            RCLCPP_WARN(get_logger(), "Rejected non-monotonic or unknown observation: %s",
            key.c_str());
          }
      }));
  }

  void ConfigureSubscriptions()
  {
    Subscribe("core", "/savo_bringup/core/state");
    Subscribe("edge", "/savo_bringup/edge/state");
    Subscribe("robot_mode", "/savo_control/mode_state");
    Subscribe("safety", "/savo_perception/safety_state");
    Subscribe("localization", "/savo_localization/health");
    Subscribe("mapping", "/savo_mapping/readiness");
    Subscribe("navigation", "/savo_nav/readiness");
    Subscribe("power", "/savo_power/status");
    Subscribe("lidar", "/savo_lidar/heartbeat");
    Subscribe("speech", "/savo_speech/state");
    Subscribe("bridge", "/savo_bridge/state");
    Subscribe("active_map", "/savo_mapping/active_map");
  }

  void PublishSnapshot()
  {
    const auto monotonic_ms = MonotonicMilliseconds();
    const auto snapshot = savo_observer::BuildSnapshot(
      ++sequence_, monotonic_ms, freshness_.Evaluate(monotonic_ms),
      freshness_.ClockReversalDetected());

    String state;
    state.data = std::string(savo_observer::ToString(snapshot.state));
    state_publisher_->publish(state);
    Bool connected;
    connected.data = snapshot.connected;
    connected_publisher_->publish(connected);
    UInt64 heartbeat;
    heartbeat.data = sequence_;
    heartbeat_publisher_->publish(heartbeat);
    String telemetry;
    telemetry.data = savo_observer::SerializeSnapshot(snapshot);
    telemetry_publisher_->publish(telemetry);
    String alerts;
    for (std::size_t index = 0; index < snapshot.alerts.size(); ++index) {
      if (index > 0U) {
        alerts.data += ';';
      }
      alerts.data += snapshot.alerts[index];
    }
    alerts_publisher_->publish(alerts);
    diagnostics_publisher_->publish(BuildDiagnostics(snapshot));
  }

  DiagnosticArray BuildDiagnostics(const savo_observer::TelemetrySnapshot & snapshot) const
  {
    DiagnosticArray array;
    array.header.stamp = now();
    DiagnosticStatus status;
    status.name = "savo_observer/connection";
    status.hardware_id = "external_observer";
    status.message = std::string(savo_observer::ToString(snapshot.state));
    status.level = snapshot.state == savo_observer::ObserverState::kConnected ?
      DiagnosticStatus::OK :
      (snapshot.state == savo_observer::ObserverState::kDegraded ?
      DiagnosticStatus::WARN : DiagnosticStatus::ERROR);
    for (const auto & dependency : snapshot.dependencies) {
      KeyValue value;
      value.key = dependency.name;
      value.value = std::string(savo_observer::ToString(dependency.state)) +
        ":" + dependency.detail;
      status.values.push_back(std::move(value));
    }
    array.status.push_back(std::move(status));
    return array;
  }

  savo_observer::TelemetryFreshness freshness_;
  double publish_rate_hz_{1.0};
  std::uint64_t sequence_{0};
  std::vector<rclcpp::Subscription<String>::SharedPtr> string_subscriptions_;
  rclcpp::Publisher<String>::SharedPtr state_publisher_;
  rclcpp::Publisher<Bool>::SharedPtr connected_publisher_;
  rclcpp::Publisher<UInt64>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<String>::SharedPtr telemetry_publisher_;
  rclcpp::Publisher<String>::SharedPtr alerts_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ObserverTelemetryNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("observer_telemetry_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
