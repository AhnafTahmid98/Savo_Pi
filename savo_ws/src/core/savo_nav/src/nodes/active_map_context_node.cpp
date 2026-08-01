// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "savo_msgs/srv/update_map_context.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"

#include "savo_nav/service_names.hpp"
#include "savo_nav/topic_names.hpp"

namespace savo_nav
{

class ActiveMapContextNode final : public rclcpp::Node
{
public:
  ActiveMapContextNode()
  : Node("active_map_context_node")
  {
    service_name_ = declare_parameter<std::string>(
      "supervisor_service",
      std::string(services::kUpdateSupervisorMapContext));
    actor_id_ = declare_parameter<std::string>("actor_id", "savo_nav");
    request_id_ = declare_parameter<std::string>(
      "request_id", "savo_nav-production-map-context");
    map_id_ = declare_parameter<std::string>("map_id", "");
    map_revision_ = declare_parameter<int>("map_revision", 0);
    map_release_id_ = declare_parameter<std::string>("map_release_id", "");
    approved_ = declare_parameter<bool>("approved", true);
    retry_period_ms_ = declare_parameter<int>("retry_period_ms", 1000);
    status_topic_ = declare_parameter<std::string>(
      "status_topic", std::string(topics::kMapContextStatus));
    heartbeat_topic_ = declare_parameter<std::string>(
      "heartbeat_topic", std::string(topics::kMapContextHeartbeat));

    if (
      service_name_.empty() || actor_id_.empty() || request_id_.empty() ||
      map_id_.empty() || map_revision_ <= 0 || map_release_id_.empty() ||
      retry_period_ms_ <= 0 || status_topic_.empty() || heartbeat_topic_.empty())
    {
      throw std::invalid_argument("invalid active-map context parameters");
    }

    auto retained_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    retained_qos.reliable();
    retained_qos.transient_local();

    status_publisher_ = create_publisher<std_msgs::msg::String>(
      status_topic_, retained_qos);
    heartbeat_publisher_ = create_publisher<std_msgs::msg::UInt64>(
      heartbeat_topic_, rclcpp::QoS(10).reliable());
    client_ = create_client<savo_msgs::srv::UpdateMapContext>(service_name_);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(retry_period_ms_),
      [this]() {Tick();});

    PublishStatus("starting");
  }

private:
  using UpdateMapContext = savo_msgs::srv::UpdateMapContext;

  void Tick()
  {
    std_msgs::msg::UInt64 heartbeat;
    heartbeat.data = ++heartbeat_sequence_;
    heartbeat_publisher_->publish(heartbeat);

    if (!client_->service_is_ready()) {
      synchronized_.store(false);
      service_was_ready_.store(false);
      PublishStatus("waiting_for_supervisor_service");
      return;
    }

    if (!service_was_ready_.exchange(true)) {
      synchronized_.store(false);
    }

    if (synchronized_.load() || request_in_flight_.exchange(true)) {
      return;
    }

    auto request = std::make_shared<UpdateMapContext::Request>();
    request->command = UpdateMapContext::Request::COMMAND_SET_SAVED_RELEASE;
    request->request_id = request_id_;
    request->actor_id = actor_id_;
    request->map_id = map_id_;
    request->map_revision = static_cast<std::uint32_t>(map_revision_);
    request->map_release_id = map_release_id_;
    request->approved = approved_;

    PublishStatus("synchronizing");

    client_->async_send_request(
      request,
      [this](rclcpp::Client<UpdateMapContext>::SharedFuture future)
      {
        request_in_flight_.store(false);

        try {
          const auto response = future.get();
          const bool matches =
          response->updated &&
          response->active_map_id == map_id_ &&
          response->active_map_revision ==
          static_cast<std::uint32_t>(map_revision_) &&
          response->active_map_release_id == map_release_id_;

          synchronized_.store(matches);

          if (matches) {
            PublishStatus("synchronized");
            return;
          }

          PublishStatus("rejected:" + response->reason);
        } catch (const std::exception & exception) {
          synchronized_.store(false);
          PublishStatus(
            std::string("service_exception:") + exception.what());
        }
      });
  }

  void PublishStatus(const std::string & state)
  {
    std_msgs::msg::String message;
    std::ostringstream stream;
    stream
      << "state=" << state
      << ";map_id=" << map_id_
      << ";map_revision=" << map_revision_
      << ";map_release_id=" << map_release_id_
      << ";synchronized=" << (synchronized_.load() ? "true" : "false");
    message.data = stream.str();
    status_publisher_->publish(message);
  }

  std::string service_name_;
  std::string actor_id_;
  std::string request_id_;
  std::string map_id_;
  std::string map_release_id_;
  std::string status_topic_;
  std::string heartbeat_topic_;
  int map_revision_{0};
  int retry_period_ms_{1000};
  bool approved_{true};

  std::atomic<bool> synchronized_{false};
  std::atomic<bool> request_in_flight_{false};
  std::atomic<bool> service_was_ready_{false};
  std::uint64_t heartbeat_sequence_{0U};

  rclcpp::Client<UpdateMapContext>::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr heartbeat_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace savo_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<savo_nav::ActiveMapContextNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("active_map_context_node"),
      "Fatal startup/runtime error: %s",
      exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
