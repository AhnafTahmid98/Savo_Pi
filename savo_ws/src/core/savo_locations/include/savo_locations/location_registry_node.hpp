// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#ifndef SAVO_LOCATIONS__LOCATION_REGISTRY_NODE_HPP_
#define SAVO_LOCATIONS__LOCATION_REGISTRY_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"

#include "savo_msgs/msg/location_event.hpp"
#include "savo_msgs/srv/approve_location.hpp"
#include "savo_msgs/srv/get_location.hpp"
#include "savo_msgs/srv/list_locations.hpp"
#include "savo_msgs/srv/register_location_candidate.hpp"
#include "savo_msgs/srv/resolve_location.hpp"
#include "savo_msgs/srv/set_location_enabled.hpp"

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

  [[nodiscard]]
  bool registry_write_ready() const;

private:
  using ResolveService =
    savo_msgs::srv::ResolveLocation;

  using GetService =
    savo_msgs::srv::GetLocation;

  using ListService =
    savo_msgs::srv::ListLocations;

  using RegisterService =
    savo_msgs::srv::RegisterLocationCandidate;

  using ApproveService =
    savo_msgs::srv::ApproveLocation;

  using SetEnabledService =
    savo_msgs::srv::SetLocationEnabled;

  void initialize_storage();

  void publish_status();
  void publish_heartbeat();
  void publish_snapshot();

  void publish_committed_event(
    std::uint64_t event_sequence,
    std::uint8_t event_type,
    const std::string & candidate_id,
    const std::string & location_id,
    std::uint64_t entity_revision,
    const std::string & actor_id,
    const std::string & reason);

  [[nodiscard]]
  bool begin_mutation(
    const std::string & operation,
    CatalogSnapshot * snapshot,
    std::string * rejection_reason);

  void finish_mutation_rejected(
    const std::string & reason);

  void finish_mutation_degraded(
    const std::string & reason);

  void finish_mutation_committed(
    CatalogSnapshot snapshot,
    std::uint64_t event_sequence,
    const std::string & result);

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

  void handle_register_candidate(
    const std::shared_ptr<
      RegisterService::Request> request,
    std::shared_ptr<
      RegisterService::Response> response);

  void handle_approve_candidate(
    const std::shared_ptr<
      ApproveService::Request> request,
    std::shared_ptr<
      ApproveService::Response> response);

  void handle_set_enabled(
    const std::shared_ptr<
      SetEnabledService::Request> request,
    std::shared_ptr<
      SetEnabledService::Response> response);

  mutable std::shared_mutex state_mutex_;
  std::mutex mutation_mutex_;

  std::string database_path_;
  bool create_parent_directories_{false};
  bool auto_migrate_{true};
  bool publish_snapshot_enabled_{true};
  bool enable_write_services_{true};

  double status_publish_hz_{1.0};
  double heartbeat_publish_hz_{2.0};

  bool ready_{false};
  bool write_ready_{false};
  bool storage_healthy_{false};
  bool mutation_in_progress_{false};

  std::string state_{"starting"};
  std::string reason_{"startup pending"};
  std::string last_mutation_result_{"none"};

  std::uint64_t
    last_mutation_event_sequence_{0U};

  BootstrapReport bootstrap_report_;
  CatalogSnapshot catalog_snapshot_;
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

  rclcpp::Publisher<
    savo_msgs::msg::LocationEvent>::SharedPtr
      event_publisher_;

  rclcpp::Service<ResolveService>::SharedPtr
    resolve_service_;

  rclcpp::Service<GetService>::SharedPtr
    get_service_;

  rclcpp::Service<ListService>::SharedPtr
    list_service_;

  rclcpp::Service<RegisterService>::SharedPtr
    register_service_;

  rclcpp::Service<ApproveService>::SharedPtr
    approve_service_;

  rclcpp::Service<SetEnabledService>::SharedPtr
    set_enabled_service_;

  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  std::atomic<std::uint64_t>
    heartbeat_sequence_{0U};
};

}  // namespace savo_locations

#endif  // SAVO_LOCATIONS__LOCATION_REGISTRY_NODE_HPP_
