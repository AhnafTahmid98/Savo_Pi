// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_locations/location_registry_node.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "savo_locations/ros_conversions.hpp"
#include "savo_locations/service_names.hpp"
#include "savo_locations/sqlite_schema.hpp"
#include "savo_locations/topic_names.hpp"


namespace savo_locations
{
namespace
{

constexpr char kDefaultDatabasePath[] =
  "/var/lib/robot_savo/locations/locations.db";


std::string json_escape(
  const std::string & value)
{
  std::string escaped;
  escaped.reserve(value.size());

  for (const char character : value) {
    switch (character) {
      case '\\':
        escaped += "\\\\";
        break;

      case '"':
        escaped += "\\\"";
        break;

      case '\n':
        escaped += "\\n";
        break;

      case '\r':
        escaped += "\\r";
        break;

      case '\t':
        escaped += "\\t";
        break;

      default:
        escaped += character;
        break;
    }
  }

  return escaped;
}


std::chrono::nanoseconds period_from_hz(
  const double frequency_hz)
{
  if (
    !std::isfinite(frequency_hz) ||
    frequency_hz <= 0.0)
  {
    throw std::invalid_argument(
      "publish frequency must be positive");
  }

  return std::chrono::duration_cast<
    std::chrono::nanoseconds>(
      std::chrono::duration<double>(
        1.0 / frequency_hz));
}

}  // namespace


LocationRegistryNode::LocationRegistryNode(
  const rclcpp::NodeOptions & options)
: Node("savo_locations", options)
{
  database_path_ =
    declare_parameter<std::string>(
      "database_path",
      kDefaultDatabasePath);

  create_parent_directories_ =
    declare_parameter<bool>(
      "create_parent_directories",
      false);

  auto_migrate_ =
    declare_parameter<bool>(
      "auto_migrate",
      true);

  publish_snapshot_enabled_ =
    declare_parameter<bool>(
      "publish_snapshot",
      true);

  status_publish_hz_ =
    declare_parameter<double>(
      "status_publish_hz",
      1.0);

  heartbeat_publish_hz_ =
    declare_parameter<double>(
      "heartbeat_publish_hz",
      2.0);

  const auto latched_qos =
    rclcpp::QoS(
      rclcpp::KeepLast(1))
      .reliable()
      .transient_local();

  status_publisher_ =
    create_publisher<std_msgs::msg::String>(
      topics::kStatus,
      latched_qos);

  snapshot_publisher_ =
    create_publisher<std_msgs::msg::String>(
      topics::kSnapshot,
      latched_qos);

  heartbeat_publisher_ =
    create_publisher<std_msgs::msg::UInt64>(
      topics::kHeartbeat,
      rclcpp::QoS(
        rclcpp::KeepLast(10))
        .reliable()
        .durability_volatile());

  resolve_service_ =
    create_service<ResolveService>(
      services::kResolve,
      std::bind(
        &LocationRegistryNode::handle_resolve,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

  get_service_ =
    create_service<GetService>(
      services::kGet,
      std::bind(
        &LocationRegistryNode::handle_get,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

  list_service_ =
    create_service<ListService>(
      services::kList,
      std::bind(
        &LocationRegistryNode::handle_list,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

  initialize_storage();

  status_timer_ =
    create_wall_timer(
      period_from_hz(status_publish_hz_),
      std::bind(
        &LocationRegistryNode::publish_status,
        this));

  heartbeat_timer_ =
    create_wall_timer(
      period_from_hz(
        heartbeat_publish_hz_),
      std::bind(
        &LocationRegistryNode::
          publish_heartbeat,
        this));

  publish_status();
  publish_heartbeat();

  if (publish_snapshot_enabled_) {
    publish_snapshot();
  }
}


bool
LocationRegistryNode::registry_ready() const
{
  std::shared_lock<std::shared_mutex> lock{
    state_mutex_};

  return ready_;
}


void
LocationRegistryNode::initialize_storage()
{
  std::unique_lock<std::shared_mutex> lock{
    state_mutex_};

  ready_ = false;
  storage_healthy_ = false;
  state_ = "starting";
  reason_ = "opening SQLite registry";

  try {
    if (
      database_path_.empty())
    {
      throw std::runtime_error(
        "database_path is empty");
    }

    if (
      create_parent_directories_ &&
      database_path_ != ":memory:")
    {
      const std::filesystem::path path{
        database_path_};

      const auto parent =
        path.parent_path();

      if (!parent.empty()) {
        std::filesystem::create_directories(
          parent);
      }
    }

    store_ =
      std::make_unique<SqliteStore>(
        database_path_);

    const auto open_result =
      store_->open();

    if (!open_result.success) {
      throw std::runtime_error(
        "SQLite open failed: " +
        open_result.reason);
    }

    if (auto_migrate_) {
      SchemaStatus schema_status;

      const auto migration_result =
        store_->migrate(
          &schema_status);

      if (!migration_result.success) {
        throw std::runtime_error(
          "SQLite migration failed: " +
          migration_result.reason);
      }
    }
    else
    {
      std::uint32_t schema_version = 0U;

      const auto version_result =
        store_->schema_version(
          &schema_version);

      if (!version_result.success) {
        throw std::runtime_error(
          "schema query failed: " +
          version_result.reason);
      }

      if (
        schema_version !=
        kSupportedSqliteSchemaVersion)
      {
        throw std::runtime_error(
          "database schema is not current");
      }
    }

    repository_ =
      std::make_unique<SqliteRepository>(
        *store_);

    CatalogSnapshot snapshot;
    BootstrapReport report;

    const auto bootstrap_result =
      repository_->bootstrap(
        &snapshot,
        &report);

    if (!bootstrap_result.success) {
      throw std::runtime_error(
        "catalog bootstrap failed: " +
        bootstrap_result.reason);
    }

    catalog_view_.replace(
      std::move(snapshot));

    bootstrap_report_ = report;

    ready_ = true;
    storage_healthy_ = true;
    state_ = "ready";
    reason_ =
      "persistent read-only registry ready";

    RCLCPP_INFO(
      get_logger(),
      "savo_locations ready: locations=%zu, "
      "candidates=%zu, events=%llu",
      report.location_count,
      report.candidate_count,
      static_cast<unsigned long long>(
        report.event_count));
  }
  catch (
    const std::exception & exception)
  {
    ready_ = false;
    storage_healthy_ = false;
    state_ = "degraded";
    reason_ = exception.what();

    RCLCPP_ERROR(
      get_logger(),
      "savo_locations startup degraded: %s",
      exception.what());
  }
}


void
LocationRegistryNode::publish_status()
{
  std_msgs::msg::String message;
  std::ostringstream stream;

  {
    std::shared_lock<std::shared_mutex> lock{
      state_mutex_};

    stream
      << "{"
      << "\"component\":\"savo_locations\","
      << "\"mode\":\"read_only\","
      << "\"state\":\""
      << json_escape(state_)
      << "\","
      << "\"ready\":"
      << (ready_ ? "true" : "false")
      << ","
      << "\"storage_healthy\":"
      << (
        storage_healthy_ ?
        "true" :
        "false")
      << ","
      << "\"schema_version\":"
      << bootstrap_report_.schema_version
      << ","
      << "\"location_count\":"
      << bootstrap_report_.location_count
      << ","
      << "\"candidate_count\":"
      << bootstrap_report_.candidate_count
      << ","
      << "\"event_count\":"
      << bootstrap_report_.event_count
      << ","
      << "\"last_event_sequence\":"
      << bootstrap_report_
           .last_event_sequence
      << ","
      << "\"database_path\":\""
      << json_escape(database_path_)
      << "\","
      << "\"reason\":\""
      << json_escape(reason_)
      << "\""
      << "}";
  }

  message.data = stream.str();

  status_publisher_->publish(message);
}


void
LocationRegistryNode::publish_heartbeat()
{
  std_msgs::msg::UInt64 message;

  message.data =
    heartbeat_sequence_.fetch_add(
      1U) + 1U;

  heartbeat_publisher_->publish(message);
}


void
LocationRegistryNode::publish_snapshot()
{
  std_msgs::msg::String message;
  std::ostringstream stream;

  std::shared_lock<std::shared_mutex> lock{
    state_mutex_};

  stream
    << "{"
    << "\"schema_version\":1,"
    << "\"read_only\":true,"
    << "\"locations\":[";

  bool first = true;

  for (
    const auto & record :
    catalog_view_.locations())
  {
    if (!first) {
      stream << ",";
    }

    first = false;

    stream
      << "{"
      << "\"location_id\":\""
      << json_escape(
           record.location.location_id)
      << "\","
      << "\"display_name\":\""
      << json_escape(
           record.location.display_name)
      << "\","
      << "\"semantic_type\":\""
      << json_escape(
           record.location.semantic_type)
      << "\","
      << "\"map_id\":\""
      << json_escape(
           record.location.map.map_id)
      << "\","
      << "\"map_revision\":"
      << record.location.map.map_revision
      << ","
      << "\"state\":"
      << static_cast<unsigned int>(
           record.state)
      << ","
      << "\"enabled\":"
      << (
        record.enabled ?
        "true" :
        "false")
      << "}";
  }

  stream << "]}";

  message.data = stream.str();

  snapshot_publisher_->publish(message);
}


void LocationRegistryNode::handle_resolve(
  const std::shared_ptr<
    ResolveService::Request> request,
  std::shared_ptr<
    ResolveService::Response> response)
{
  std::shared_lock<std::shared_mutex> lock{
    state_mutex_};

  if (!ready_) {
    response->resolved = false;

    response->result_code =
      ResolveService::Response::
        RESULT_INTERNAL_ERROR;

    response->match_type =
      ResolveService::Response::
        MATCH_NONE;

    response->reason =
      "registry unavailable: " + reason_;

    return;
  }

  ReadResolveRequest view_request;

  view_request.query = request->query;

  view_request.enforce_map_context =
    request->enforce_map_context;

  view_request.map_id =
    request->map_id;

  view_request.map_revision =
    request->map_revision;

  const auto result =
    catalog_view_.resolve(view_request);

  response->reason = result.reason;

  response->normalized_query =
    result.normalized_query;

  response->match_type =
    static_cast<std::uint8_t>(
      result.match_type);

  response->ambiguous_location_ids =
    result.ambiguous_location_ids;

  switch (result.code) {
    case ReadResolveCode::kResolved:
      response->resolved = true;

      response->result_code =
        ResolveService::Response::
          RESULT_RESOLVED;

      response->location =
        to_ros_location_record(
          result.location);

      return;

    case ReadResolveCode::kInvalidQuery:
      response->result_code =
        ResolveService::Response::
          RESULT_INVALID_QUERY;
      break;

    case ReadResolveCode::kNotFound:
      response->result_code =
        ResolveService::Response::
          RESULT_NOT_FOUND;
      break;

    case ReadResolveCode::kAmbiguous:
      response->result_code =
        ResolveService::Response::
          RESULT_AMBIGUOUS;
      break;

    case ReadResolveCode::kDisabled:
      response->result_code =
        ResolveService::Response::
          RESULT_DISABLED;

      response->location =
        to_ros_location_record(
          result.location);
      break;

    case ReadResolveCode::kRetired:
      response->result_code =
        ResolveService::Response::
          RESULT_RETIRED;

      response->location =
        to_ros_location_record(
          result.location);
      break;

    case ReadResolveCode::kMapMismatch:
      response->result_code =
        ResolveService::Response::
          RESULT_MAP_MISMATCH;

      if (
        !result
          .location
          .location
          .location_id
          .empty())
      {
        response->location =
          to_ros_location_record(
            result.location);
      }
      break;
  }

  response->resolved = false;
}


void LocationRegistryNode::handle_get(
  const std::shared_ptr<
    GetService::Request> request,
  std::shared_ptr<
    GetService::Response> response)
{
  std::shared_lock<std::shared_mutex> lock{
    state_mutex_};

  if (!ready_) {
    response->found = false;

    response->result_code =
      GetService::Response::
        RESULT_INTERNAL_ERROR;

    response->reason =
      "registry unavailable: " + reason_;

    return;
  }

  const auto result =
    catalog_view_.get(
      request->location_id,
      request->include_disabled,
      request->include_retired);

  response->reason = result.reason;

  switch (result.code) {
    case ReadGetCode::kFound:
      response->found = true;

      response->result_code =
        GetService::Response::
          RESULT_FOUND;

      response->location =
        to_ros_location_record(
          result.location);

      return;

    case ReadGetCode::kInvalidId:
      response->result_code =
        GetService::Response::
          RESULT_INVALID_ID;
      break;

    case ReadGetCode::kNotFound:
      response->result_code =
        GetService::Response::
          RESULT_NOT_FOUND;
      break;

    case ReadGetCode::kDisabled:
      response->result_code =
        GetService::Response::
          RESULT_DISABLED;

      response->location =
        to_ros_location_record(
          result.location);
      break;

    case ReadGetCode::kRetired:
      response->result_code =
        GetService::Response::
          RESULT_RETIRED;

      response->location =
        to_ros_location_record(
          result.location);
      break;
  }

  response->found = false;
}


void LocationRegistryNode::handle_list(
  const std::shared_ptr<
    ListService::Request> request,
  std::shared_ptr<
    ListService::Response> response)
{
  std::shared_lock<std::shared_mutex> lock{
    state_mutex_};

  if (!ready_) {
    response->success = false;

    response->result_code =
      ListService::Response::
        RESULT_INTERNAL_ERROR;

    response->reason =
      "registry unavailable: " + reason_;

    return;
  }

  ReadListRequest view_request;

  view_request.map_id =
    request->map_id;

  view_request.map_revision =
    request->map_revision;

  view_request.enforce_map_context =
    request->enforce_map_context;

  view_request.semantic_type =
    request->semantic_type;

  view_request.state_filter =
    request->state_filter;

  view_request.enabled_only =
    request->enabled_only;

  const auto result =
    catalog_view_.list(view_request);

  if (!result.valid) {
    response->success = false;

    response->result_code =
      ListService::Response::
        RESULT_INVALID_FILTER;

    response->reason = result.reason;

    return;
  }

  response->success = true;

  response->result_code =
    ListService::Response::
      RESULT_OK;

  response->reason = result.reason;

  response->locations.reserve(
    result.locations.size());

  for (
    const auto & record :
    result.locations)
  {
    response->locations.push_back(
      to_ros_location_record(record));
  }
}

}  // namespace savo_locations
