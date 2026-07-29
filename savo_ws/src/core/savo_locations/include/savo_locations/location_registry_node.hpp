// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#ifndef SAVO_LOCATIONS__LOCATION_REGISTRY_NODE_HPP_
#define SAVO_LOCATIONS__LOCATION_REGISTRY_NODE_HPP_

#include <atomic>
#include <memory>
#include <shared_mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"

#include "savo_msgs/srv/get_location.hpp"
#include "savo_msgs/srv/list_locations.hpp"
#include "savo_msgs/srv/resolve_location.hpp"

#include "savo_locations/read_only_catalog_view.hpp"
#include "savo_locations/sqlite_repository.hpp"
#include "savo_locations/sqlite_store.hpp"


namespace savo_locations
{

class LocationRegistryNode final :
  public rclcpp::Node
{
public:
  explicit LocationRegistryNode(
    const rclcpp::NodeOptions & options =
      rclcpp::NodeOptions());

  [[nodiscard]]
  bool registry_ready() const;

private:
  using ResolveService =
    savo_msgs::srv::ResolveLocation;

  using GetService =
    savo_msgs::srv::GetLocation;

  using ListService =
    savo_msgs::srv::ListLocations;

  void initialize_storage();

  void publish_status();
  void publish_heartbeat();
  void publish_snapshot();

  void handle_resolve(
    const std::shared_ptr<
      ResolveService::Request> request,
    std::shared_ptr<
      ResolveService::Response> response);

  void handle_get(
    const std::shared_ptr<
      GetService::Request> request,
    std::shared_ptr<
      GetService::Response> response);

  void handle_list(
    const std::shared_ptr<
      ListService::Request> request,
    std::shared_ptr<
      ListService::Response> response);

  mutable std::shared_mutex state_mutex_;

  std::string database_path_;
  bool create_parent_directories_{false};
  bool auto_migrate_{true};
  bool publish_snapshot_enabled_{true};

  double status_publish_hz_{1.0};
  double heartbeat_publish_hz_{2.0};

  bool ready_{false};
  bool storage_healthy_{false};

  std::string state_{"starting"};
  std::string reason_{"startup pending"};

  BootstrapReport bootstrap_report_;
  ReadOnlyCatalogView catalog_view_;

  std::unique_ptr<SqliteStore> store_;
  std::unique_ptr<SqliteRepository> repository_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
      status_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::UInt64>::SharedPtr
      heartbeat_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
      snapshot_publisher_;

  rclcpp::Service<
    ResolveService>::SharedPtr
      resolve_service_;

  rclcpp::Service<
    GetService>::SharedPtr
      get_service_;

  rclcpp::Service<
    ListService>::SharedPtr
      list_service_;

  rclcpp::TimerBase::SharedPtr
    status_timer_;

  rclcpp::TimerBase::SharedPtr
    heartbeat_timer_;

  std::atomic<std::uint64_t>
    heartbeat_sequence_{0U};
};

}  // namespace savo_locations

#endif  // SAVO_LOCATIONS__LOCATION_REGISTRY_NODE_HPP_
