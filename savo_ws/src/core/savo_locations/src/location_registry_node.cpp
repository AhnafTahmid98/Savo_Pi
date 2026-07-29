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

#include "savo_locations/location_catalog.hpp"
#include "savo_locations/normalization.hpp"
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


bool hydrate_catalog(
  const CatalogSnapshot & snapshot,
  InMemoryLocationCatalog * catalog,
  std::string * reason)
{
  if (catalog == nullptr) {
    if (reason != nullptr) {
      *reason = "catalog output is null";
    }

    return false;
  }

  for (const auto & location : snapshot.locations) {
    const auto result =
      catalog->insert_location(location);

    if (!result.success) {
      if (reason != nullptr) {
        *reason =
          "could not restore location " +
          location.location.location_id +
          ": " + result.reason;
      }

      return false;
    }
  }

  for (const auto & candidate : snapshot.candidates) {
    const auto result =
      catalog->restore_candidate_record(candidate);

    if (!result.success) {
      if (reason != nullptr) {
        *reason =
          "could not restore candidate " +
          candidate.candidate.candidate_id +
          ": " + result.reason;
      }

      return false;
    }
  }

  return true;
}


bool approval_has_approach_issue(
  const ApprovalResult & result)
{
  if (
    result.code ==
    ApprovalCode::kMissingApproachPose)
  {
    return true;
  }

  for (const auto & issue : result.validation_issues) {
    if (
      issue.field.find("approach_pose") !=
      std::string::npos)
    {
      return true;
    }
  }

  return false;
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

  enable_write_services_ =
    declare_parameter<bool>(
      "enable_write_services",
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
    rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable()
      .transient_local();

  status_publisher_ =
    create_publisher<std_msgs::msg::String>(
      std::string(
        ::savo_locations::topic_names::kStatus),
      latched_qos);

  snapshot_publisher_ =
    create_publisher<std_msgs::msg::String>(
      std::string(
        ::savo_locations::topic_names::kSnapshot),
      latched_qos);

  heartbeat_publisher_ =
    create_publisher<std_msgs::msg::UInt64>(
      std::string(
        ::savo_locations::topic_names::kHeartbeat),
      rclcpp::QoS(rclcpp::KeepLast(10))
        .reliable()
        .durability_volatile());

  event_publisher_ =
    create_publisher<
      savo_msgs::msg::LocationEvent>(
        std::string(
          ::savo_locations::topic_names::kEvents),
        rclcpp::QoS(rclcpp::KeepLast(100))
          .reliable()
          .durability_volatile());

  resolve_service_ =
    create_service<ResolveService>(
      std::string(
        ::savo_locations::service_names::kResolve),
      std::bind(
        &LocationRegistryNode::handle_resolve,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

  get_service_ =
    create_service<GetService>(
      std::string(
        ::savo_locations::service_names::kGet),
      std::bind(
        &LocationRegistryNode::handle_get,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

  list_service_ =
    create_service<ListService>(
      std::string(
        ::savo_locations::service_names::kList),
      std::bind(
        &LocationRegistryNode::handle_list,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

  register_service_ =
    create_service<RegisterService>(
      std::string(
        ::savo_locations::service_names::
          kRegisterCandidate),
      std::bind(
        &LocationRegistryNode::
          handle_register_candidate,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

  approve_service_ =
    create_service<ApproveService>(
      std::string(
        ::savo_locations::service_names::
          kApproveCandidate),
      std::bind(
        &LocationRegistryNode::
          handle_approve_candidate,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

  set_enabled_service_ =
    create_service<SetEnabledService>(
      std::string(
        ::savo_locations::service_names::kSetEnabled),
      std::bind(
        &LocationRegistryNode::handle_set_enabled,
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
      period_from_hz(heartbeat_publish_hz_),
      std::bind(
        &LocationRegistryNode::publish_heartbeat,
        this));

  publish_status();
  publish_heartbeat();

  if (publish_snapshot_enabled_) {
    publish_snapshot();
  }
}


bool LocationRegistryNode::registry_ready() const
{
  std::shared_lock<std::shared_mutex> lock{
    state_mutex_};

  return ready_;
}


bool
LocationRegistryNode::registry_write_ready() const
{
  std::shared_lock<std::shared_mutex> lock{
    state_mutex_};

  return write_ready_;
}


void LocationRegistryNode::initialize_storage()
{
  std::unique_lock<std::shared_mutex> lock{
    state_mutex_};

  ready_ = false;
  write_ready_ = false;
  storage_healthy_ = false;
  mutation_in_progress_ = false;
  state_ = "starting";
  reason_ = "opening SQLite registry";
  last_mutation_result_ = "none";

  try {
    if (database_path_.empty()) {
      throw std::runtime_error(
        "database_path is empty");
    }

    if (
      create_parent_directories_ &&
      database_path_ != ":memory:")
    {
      const std::filesystem::path path{
        database_path_};

      const auto parent = path.parent_path();

      if (!parent.empty()) {
        std::filesystem::create_directories(
          parent);
      }
    }

    store_ =
      std::make_unique<SqliteStore>(
        database_path_);

    const auto open_result = store_->open();

    if (!open_result.success) {
      throw std::runtime_error(
        "SQLite open failed: " +
        open_result.reason);
    }

    if (auto_migrate_) {
      SchemaStatus schema_status;

      const auto migration_result =
        store_->migrate(&schema_status);

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
        store_->schema_version(&schema_version);

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

    InMemoryLocationCatalog catalog;
    std::string hydration_reason;

    if (
      !hydrate_catalog(
        snapshot,
        &catalog,
        &hydration_reason))
    {
      throw std::runtime_error(
        "catalog hydration failed: " +
        hydration_reason);
    }

    catalog_snapshot_ = snapshot;
    catalog_view_.replace(snapshot);
    bootstrap_report_ = report;

    ready_ = true;
    storage_healthy_ = true;
    write_ready_ = enable_write_services_;
    state_ = "ready";
    reason_ =
      enable_write_services_ ?
      "persistent read/write registry ready" :
      "persistent read-only registry ready";

    RCLCPP_INFO(
      get_logger(),
      "savo_locations ready: locations=%zu, "
      "candidates=%zu, events=%llu, writes=%s",
      report.location_count,
      report.candidate_count,
      static_cast<unsigned long long>(
        report.event_count),
      write_ready_ ? "ready" : "disabled");
  }
  catch (const std::exception & exception) {
    ready_ = false;
    write_ready_ = false;
    storage_healthy_ = false;
    state_ = "degraded";
    reason_ = exception.what();

    RCLCPP_ERROR(
      get_logger(),
      "savo_locations startup degraded: %s",
      exception.what());
  }
}


void LocationRegistryNode::publish_status()
{
  std_msgs::msg::String message;
  std::ostringstream stream;

  {
    std::shared_lock<std::shared_mutex> lock{
      state_mutex_};

    stream
      << "{"
      << "\"component\":\"savo_locations\","
      << "\"mode\":\""
      << (
        enable_write_services_ ?
        "read_write" :
        "read_only")
      << "\","
      << "\"state\":\""
      << json_escape(state_)
      << "\","
      << "\"ready\":"
      << (ready_ ? "true" : "false")
      << ","
      << "\"read_ready\":"
      << (ready_ ? "true" : "false")
      << ","
      << "\"write_ready\":"
      << (write_ready_ ? "true" : "false")
      << ","
      << "\"storage_healthy\":"
      << (storage_healthy_ ? "true" : "false")
      << ","
      << "\"mutation_in_progress\":"
      << (
        mutation_in_progress_ ?
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
      << bootstrap_report_.last_event_sequence
      << ","
      << "\"last_mutation_event_sequence\":"
      << last_mutation_event_sequence_
      << ","
      << "\"last_mutation_result\":\""
      << json_escape(last_mutation_result_)
      << "\","
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


void LocationRegistryNode::publish_heartbeat()
{
  std_msgs::msg::UInt64 message;

  message.data =
    heartbeat_sequence_.fetch_add(1U) + 1U;

  heartbeat_publisher_->publish(message);
}


void LocationRegistryNode::publish_snapshot()
{
  std_msgs::msg::String message;
  std::ostringstream stream;

  std::shared_lock<std::shared_mutex> lock{
    state_mutex_};

  stream
    << "{"
    << "\"schema_version\":1,"
    << "\"read_only\":"
    << (write_ready_ ? "false" : "true")
    << ","
    << "\"locations\":[";

  bool first = true;

  for (const auto & record : catalog_view_.locations()) {
    if (!first) {
      stream << ",";
    }

    first = false;

    stream
      << "{"
      << "\"location_id\":\""
      << json_escape(record.location.location_id)
      << "\","
      << "\"display_name\":\""
      << json_escape(record.location.display_name)
      << "\","
      << "\"semantic_type\":\""
      << json_escape(record.location.semantic_type)
      << "\","
      << "\"map_id\":\""
      << json_escape(record.location.map.map_id)
      << "\","
      << "\"map_revision\":"
      << record.location.map.map_revision
      << ","
      << "\"state\":"
      << static_cast<unsigned int>(record.state)
      << ","
      << "\"enabled\":"
      << (record.enabled ? "true" : "false")
      << "}";
  }

  stream << "]}";

  message.data = stream.str();
  snapshot_publisher_->publish(message);
}


void LocationRegistryNode::publish_committed_event(
  const std::uint64_t event_sequence,
  const std::uint8_t event_type,
  const std::string & candidate_id,
  const std::string & location_id,
  const std::uint64_t entity_revision,
  const std::string & actor_id,
  const std::string & reason)
{
  savo_msgs::msg::LocationEvent message;

  message.event_sequence = event_sequence;
  message.stamp = now();
  message.event_type = event_type;
  message.candidate_id = candidate_id;
  message.location_id = location_id;
  message.entity_revision = entity_revision;
  message.actor_id = actor_id;
  message.reason = reason;

  event_publisher_->publish(message);
}


bool LocationRegistryNode::begin_mutation(
  const std::string & operation,
  CatalogSnapshot * snapshot,
  std::string * rejection_reason)
{
  if (snapshot == nullptr) {
    if (rejection_reason != nullptr) {
      *rejection_reason =
        "mutation snapshot output is null";
    }

    return false;
  }

  std::unique_lock<std::shared_mutex> lock{
    state_mutex_};

  if (!enable_write_services_) {
    if (rejection_reason != nullptr) {
      *rejection_reason =
        "write services are disabled";
    }

    return false;
  }

  if (
    !ready_ ||
    !write_ready_ ||
    !storage_healthy_ ||
    repository_ == nullptr)
  {
    if (rejection_reason != nullptr) {
      *rejection_reason =
        "write registry unavailable: " + reason_;
    }

    return false;
  }

  mutation_in_progress_ = true;
  last_mutation_result_ =
    operation + ":in_progress";

  *snapshot = catalog_snapshot_;
  return true;
}


void LocationRegistryNode::finish_mutation_rejected(
  const std::string & reason)
{
  std::unique_lock<std::shared_mutex> lock{
    state_mutex_};

  mutation_in_progress_ = false;
  last_mutation_result_ =
    "rejected:" + reason;
}


void LocationRegistryNode::finish_mutation_degraded(
  const std::string & reason)
{
  std::unique_lock<std::shared_mutex> lock{
    state_mutex_};

  mutation_in_progress_ = false;
  write_ready_ = false;
  storage_healthy_ = false;
  state_ = "degraded_write";
  reason_ = reason;
  last_mutation_result_ =
    "storage_failure:" + reason;
}


void LocationRegistryNode::finish_mutation_committed(
  CatalogSnapshot snapshot,
  const std::uint64_t event_sequence,
  const std::string & result)
{
  std::unique_lock<std::shared_mutex> lock{
    state_mutex_};

  catalog_snapshot_ = std::move(snapshot);
  catalog_view_.replace(catalog_snapshot_);

  bootstrap_report_.location_count =
    catalog_snapshot_.locations.size();

  bootstrap_report_.candidate_count =
    catalog_snapshot_.candidates.size();

  bootstrap_report_.event_count += 1U;
  bootstrap_report_.last_event_sequence =
    event_sequence;

  last_mutation_event_sequence_ =
    event_sequence;

  mutation_in_progress_ = false;
  write_ready_ = true;
  storage_healthy_ = true;
  state_ = "ready";
  reason_ = "persistent read/write registry ready";
  last_mutation_result_ = result;
}


void LocationRegistryNode::handle_resolve(
  const std::shared_ptr<ResolveService::Request> request,
  std::shared_ptr<ResolveService::Response> response)
{
  std::shared_lock<std::shared_mutex> lock{
    state_mutex_};

  if (!ready_) {
    response->resolved = false;
    response->result_code =
      ResolveService::Response::RESULT_INTERNAL_ERROR;
    response->match_type =
      ResolveService::Response::MATCH_NONE;
    response->reason =
      "registry unavailable: " + reason_;
    return;
  }

  ReadResolveRequest view_request;
  view_request.query = request->query;
  view_request.enforce_map_context =
    request->enforce_map_context;
  view_request.map_id = request->map_id;
  view_request.map_revision = request->map_revision;

  const auto result =
    catalog_view_.resolve(view_request);

  response->reason = result.reason;
  response->normalized_query =
    result.normalized_query;
  response->match_type =
    static_cast<std::uint8_t>(result.match_type);
  response->ambiguous_location_ids =
    result.ambiguous_location_ids;

  switch (result.code) {
    case ReadResolveCode::kResolved:
      response->resolved = true;
      response->result_code =
        ResolveService::Response::RESULT_RESOLVED;
      response->location =
        to_ros_location_record(result.location);
      return;

    case ReadResolveCode::kInvalidQuery:
      response->result_code =
        ResolveService::Response::RESULT_INVALID_QUERY;
      break;

    case ReadResolveCode::kNotFound:
      response->result_code =
        ResolveService::Response::RESULT_NOT_FOUND;
      break;

    case ReadResolveCode::kAmbiguous:
      response->result_code =
        ResolveService::Response::RESULT_AMBIGUOUS;
      break;

    case ReadResolveCode::kDisabled:
      response->result_code =
        ResolveService::Response::RESULT_DISABLED;
      response->location =
        to_ros_location_record(result.location);
      break;

    case ReadResolveCode::kRetired:
      response->result_code =
        ResolveService::Response::RESULT_RETIRED;
      response->location =
        to_ros_location_record(result.location);
      break;

    case ReadResolveCode::kMapMismatch:
      response->result_code =
        ResolveService::Response::RESULT_MAP_MISMATCH;

      if (!result.location.location.location_id.empty()) {
        response->location =
          to_ros_location_record(result.location);
      }
      break;
  }

  response->resolved = false;
}


void LocationRegistryNode::handle_get(
  const std::shared_ptr<GetService::Request> request,
  std::shared_ptr<GetService::Response> response)
{
  std::shared_lock<std::shared_mutex> lock{
    state_mutex_};

  if (!ready_) {
    response->found = false;
    response->result_code =
      GetService::Response::RESULT_INTERNAL_ERROR;
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
        GetService::Response::RESULT_FOUND;
      response->location =
        to_ros_location_record(result.location);
      return;

    case ReadGetCode::kInvalidId:
      response->result_code =
        GetService::Response::RESULT_INVALID_ID;
      break;

    case ReadGetCode::kNotFound:
      response->result_code =
        GetService::Response::RESULT_NOT_FOUND;
      break;

    case ReadGetCode::kDisabled:
      response->result_code =
        GetService::Response::RESULT_DISABLED;
      response->location =
        to_ros_location_record(result.location);
      break;

    case ReadGetCode::kRetired:
      response->result_code =
        GetService::Response::RESULT_RETIRED;
      response->location =
        to_ros_location_record(result.location);
      break;
  }

  response->found = false;
}


void LocationRegistryNode::handle_list(
  const std::shared_ptr<ListService::Request> request,
  std::shared_ptr<ListService::Response> response)
{
  std::shared_lock<std::shared_mutex> lock{
    state_mutex_};

  if (!ready_) {
    response->success = false;
    response->result_code =
      ListService::Response::RESULT_INTERNAL_ERROR;
    response->reason =
      "registry unavailable: " + reason_;
    return;
  }

  ReadListRequest view_request;
  view_request.map_id = request->map_id;
  view_request.map_revision = request->map_revision;
  view_request.enforce_map_context =
    request->enforce_map_context;
  view_request.semantic_type = request->semantic_type;
  view_request.state_filter = request->state_filter;
  view_request.enabled_only = request->enabled_only;

  const auto result =
    catalog_view_.list(view_request);

  if (!result.valid) {
    response->success = false;
    response->result_code =
      ListService::Response::RESULT_INVALID_FILTER;
    response->reason = result.reason;
    return;
  }

  response->success = true;
  response->result_code =
    ListService::Response::RESULT_OK;
  response->reason = result.reason;
  response->locations.reserve(result.locations.size());

  for (const auto & record : result.locations) {
    response->locations.push_back(
      to_ros_location_record(record));
  }
}


void LocationRegistryNode::handle_register_candidate(
  const std::shared_ptr<RegisterService::Request> request,
  std::shared_ptr<RegisterService::Response> response)
{
  std::lock_guard<std::mutex> serialized{
    mutation_mutex_};

  CatalogSnapshot current;
  std::string rejection;

  if (!begin_mutation(
      "register_candidate",
      &current,
      &rejection))
  {
    response->registered = false;
    response->result_code =
      RegisterService::Response::
        RESULT_STORAGE_UNAVAILABLE;
    response->reason = rejection;
    publish_status();
    return;
  }

  try {
    const auto actor_id =
      collapse_ascii_whitespace(
        request->actor_id);

    const auto & input = request->candidate;

    if (
      actor_id.empty() ||
      (
        input.state !=
          savo_msgs::msg::LocationCandidate::
            STATE_UNKNOWN &&
        input.state !=
          savo_msgs::msg::LocationCandidate::
            STATE_PENDING_REVIEW) ||
      input.candidate_revision > 1U ||
      !trim_ascii(input.review_reason).empty())
    {
      const std::string reason =
        "actor and new-candidate envelope are required";

      finish_mutation_rejected(reason);
      response->registered = false;
      response->result_code =
        RegisterService::Response::
          RESULT_INVALID_CANDIDATE;
      response->reason = reason;
      publish_status();
      return;
    }

    InMemoryLocationCatalog catalog;
    std::string hydration_reason;

    if (!hydrate_catalog(
        current,
        &catalog,
        &hydration_reason))
    {
      finish_mutation_degraded(
        "catalog hydration failed: " +
        hydration_reason);

      response->registered = false;
      response->result_code =
        RegisterService::Response::
          RESULT_INTERNAL_ERROR;
      response->reason = hydration_reason;
      publish_status();
      return;
    }

    const auto domain_result =
      catalog.register_candidate(
        from_ros_candidate(input));

    if (!domain_result.success) {
      finish_mutation_rejected(
        domain_result.reason);

      response->registered = false;
      response->reason = domain_result.reason;

      switch (domain_result.code) {
        case CandidateMutationCode::
            kCandidateIdConflict:
          response->result_code =
            RegisterService::Response::
              RESULT_DUPLICATE_CANDIDATE_ID;
          break;

        case CandidateMutationCode::kTagConflict:
          response->result_code =
            RegisterService::Response::
              RESULT_TAG_CONFLICT;
          break;

        case CandidateMutationCode::
            kInvalidCandidate:
          response->result_code =
            RegisterService::Response::
              RESULT_INVALID_CANDIDATE;
          break;

        default:
          response->result_code =
            RegisterService::Response::
              RESULT_INTERNAL_ERROR;
          break;
      }

      publish_status();
      return;
    }

    CatalogSnapshot post;
    post.locations = catalog.list_locations();
    post.candidates = catalog.list_candidates();

    const auto stored_candidate =
      domain_result.candidate.value();

    CandidateRegistrationCommit commit;
    commit.candidate_id =
      stored_candidate.candidate.candidate_id;
    commit.actor_id = actor_id;
    commit.reason = "candidate registered";
    commit.payload_json = "{}";
    commit.post_registration_snapshot = post;

    std::uint64_t event_sequence = 0U;

    const auto persistence =
      repository_->commit_candidate_registration(
        commit,
        &event_sequence);

    if (!persistence.success) {
      finish_mutation_degraded(
        "candidate registration persistence failed: " +
        persistence.reason);

      response->registered = false;
      response->reason = persistence.reason;

      switch (persistence.code) {
        case SnapshotCode::kIdentityConflict:
          response->result_code =
            RegisterService::Response::
              RESULT_DUPLICATE_CANDIDATE_ID;
          break;

        case SnapshotCode::kTagConflict:
          response->result_code =
            RegisterService::Response::
              RESULT_TAG_CONFLICT;
          break;

        case SnapshotCode::kValidationFailed:
          response->result_code =
            RegisterService::Response::
              RESULT_INVALID_CANDIDATE;
          break;

        default:
          response->result_code =
            RegisterService::Response::
              RESULT_STORAGE_UNAVAILABLE;
          break;
      }

      publish_status();
      return;
    }

    finish_mutation_committed(
      std::move(post),
      event_sequence,
      "candidate_registered");

    publish_committed_event(
      event_sequence,
      savo_msgs::msg::LocationEvent::
        EVENT_CANDIDATE_REGISTERED,
      stored_candidate.candidate.candidate_id,
      "",
      stored_candidate.candidate_revision,
      actor_id,
      "candidate registered");

    if (publish_snapshot_enabled_) {
      publish_snapshot();
    }

    publish_status();

    response->registered = true;
    response->result_code =
      RegisterService::Response::RESULT_REGISTERED;
    response->reason =
      "candidate registered; committed event_sequence=" +
      std::to_string(event_sequence);
    response->stored_candidate =
      to_ros_candidate_record(stored_candidate);
  }
  catch (const std::exception & exception) {
    finish_mutation_degraded(
      "candidate registration exception: " +
      std::string(exception.what()));

    response->registered = false;
    response->result_code =
      RegisterService::Response::RESULT_INTERNAL_ERROR;
    response->reason = exception.what();
    publish_status();
  }
}


void LocationRegistryNode::handle_approve_candidate(
  const std::shared_ptr<ApproveService::Request> request,
  std::shared_ptr<ApproveService::Response> response)
{
  std::lock_guard<std::mutex> serialized{
    mutation_mutex_};

  CatalogSnapshot current;
  std::string rejection;

  if (!begin_mutation(
      "approve_candidate",
      &current,
      &rejection))
  {
    response->approved = false;
    response->result_code =
      ApproveService::Response::
        RESULT_STORAGE_UNAVAILABLE;
    response->reason = rejection;
    publish_status();
    return;
  }

  try {
    const auto actor_id =
      collapse_ascii_whitespace(
        request->actor_id);

    if (
      actor_id.empty() ||
      trim_ascii(request->candidate_id).empty() ||
      request->expected_candidate_revision == 0U)
    {
      const std::string reason =
        "candidate ID, revision and actor are required";

      finish_mutation_rejected(reason);
      response->approved = false;
      response->result_code =
        ApproveService::Response::
          RESULT_INVALID_REQUEST;
      response->reason = reason;
      publish_status();
      return;
    }

    InMemoryLocationCatalog catalog;
    std::string hydration_reason;

    if (!hydrate_catalog(
        current,
        &catalog,
        &hydration_reason))
    {
      finish_mutation_degraded(
        "catalog hydration failed: " +
        hydration_reason);

      response->approved = false;
      response->result_code =
        ApproveService::Response::
          RESULT_INTERNAL_ERROR;
      response->reason = hydration_reason;
      publish_status();
      return;
    }

    const auto domain_request =
      from_ros_approval_request(*request);

    const auto domain_result =
      catalog.approve_candidate(domain_request);

    if (!domain_result.success) {
      finish_mutation_rejected(
        domain_result.reason);

      response->approved = false;
      response->reason = domain_result.reason;

      switch (domain_result.code) {
        case ApprovalCode::kCandidateNotFound:
          response->result_code =
            ApproveService::Response::
              RESULT_CANDIDATE_NOT_FOUND;
          break;

        case ApprovalCode::kCandidateNotPending:
          response->result_code =
            ApproveService::Response::
              RESULT_CANDIDATE_NOT_PENDING;
          break;

        case ApprovalCode::kStaleRevision:
          response->result_code =
            ApproveService::Response::
              RESULT_STALE_REVISION;
          break;

        case ApprovalCode::kLocationConflict:
          if (
            domain_result.location_mutation_code ==
            MutationCode::kLocationIdConflict)
          {
            response->result_code =
              ApproveService::Response::
                RESULT_LOCATION_ID_CONFLICT;
          }
          else
          {
            response->result_code =
              ApproveService::Response::
                RESULT_ALIAS_CONFLICT;
          }
          break;

        case ApprovalCode::kMissingApproachPose:
        case ApprovalCode::kInvalidLocation:
          response->result_code =
            approval_has_approach_issue(domain_result) ?
            ApproveService::Response::
              RESULT_INVALID_APPROACH_POSE :
            ApproveService::Response::
              RESULT_INVALID_REQUEST;
          break;

        default:
          response->result_code =
            ApproveService::Response::
              RESULT_INVALID_REQUEST;
          break;
      }

      publish_status();
      return;
    }

    const auto approved_candidate =
      domain_result.candidate.value();

    const auto approved_location =
      domain_result.location.value();

    CatalogSnapshot post;
    post.locations = catalog.list_locations();
    post.candidates = catalog.list_candidates();

    CandidateApprovalCommit commit;
    commit.candidate_id =
      approved_candidate.candidate.candidate_id;
    commit.expected_candidate_revision =
      request->expected_candidate_revision;
    commit.approved_location_id =
      approved_location.location.location_id;
    commit.actor_id = actor_id;
    commit.reason = "candidate approved";
    commit.payload_json = "{}";
    commit.post_approval_snapshot = post;

    std::uint64_t event_sequence = 0U;

    const auto persistence =
      repository_->commit_candidate_approval(
        commit,
        &event_sequence);

    if (!persistence.success) {
      finish_mutation_degraded(
        "candidate approval persistence failed: " +
        persistence.reason);

      response->approved = false;
      response->reason = persistence.reason;

      if (
        persistence.code ==
        SnapshotCode::kStaleRevision)
      {
        response->result_code =
          ApproveService::Response::
            RESULT_STALE_REVISION;
      }
      else
      {
        response->result_code =
          ApproveService::Response::
            RESULT_STORAGE_UNAVAILABLE;
      }

      publish_status();
      return;
    }

    finish_mutation_committed(
      std::move(post),
      event_sequence,
      "candidate_approved");

    publish_committed_event(
      event_sequence,
      savo_msgs::msg::LocationEvent::
        EVENT_LOCATION_APPROVED,
      approved_candidate.candidate.candidate_id,
      approved_location.location.location_id,
      approved_candidate.candidate_revision,
      actor_id,
      "candidate approved");

    if (publish_snapshot_enabled_) {
      publish_snapshot();
    }

    publish_status();

    response->approved = true;
    response->result_code =
      ApproveService::Response::RESULT_APPROVED;
    response->reason =
      "candidate approved; committed event_sequence=" +
      std::to_string(event_sequence);
    response->location =
      to_ros_location_record(approved_location);
  }
  catch (const std::exception & exception) {
    finish_mutation_degraded(
      "candidate approval exception: " +
      std::string(exception.what()));

    response->approved = false;
    response->result_code =
      ApproveService::Response::RESULT_INTERNAL_ERROR;
    response->reason = exception.what();
    publish_status();
  }
}


void LocationRegistryNode::handle_set_enabled(
  const std::shared_ptr<SetEnabledService::Request> request,
  std::shared_ptr<SetEnabledService::Response> response)
{
  std::lock_guard<std::mutex> serialized{
    mutation_mutex_};

  CatalogSnapshot current;
  std::string rejection;

  if (!begin_mutation(
      "set_location_enabled",
      &current,
      &rejection))
  {
    response->updated = false;
    response->result_code =
      SetEnabledService::Response::
        RESULT_STORAGE_UNAVAILABLE;
    response->reason = rejection;
    publish_status();
    return;
  }

  try {
    const auto actor_id =
      collapse_ascii_whitespace(
        request->actor_id);

    const auto mutation_reason =
      collapse_ascii_whitespace(
        request->reason);

    if (
      actor_id.empty() ||
      mutation_reason.empty() ||
      trim_ascii(request->location_id).empty() ||
      request->expected_record_revision == 0U)
    {
      const std::string reason =
        "location ID, revision, actor and reason are required";

      finish_mutation_rejected(reason);
      response->updated = false;
      response->result_code =
        SetEnabledService::Response::
          RESULT_INVALID_REQUEST;
      response->reason = reason;
      publish_status();
      return;
    }

    InMemoryLocationCatalog catalog;
    std::string hydration_reason;

    if (!hydrate_catalog(
        current,
        &catalog,
        &hydration_reason))
    {
      finish_mutation_degraded(
        "catalog hydration failed: " +
        hydration_reason);

      response->updated = false;
      response->result_code =
        SetEnabledService::Response::
          RESULT_INTERNAL_ERROR;
      response->reason = hydration_reason;
      publish_status();
      return;
    }

    const auto existing =
      catalog.get_location(request->location_id);

    if (
      existing.has_value() &&
      existing->enabled == request->enabled)
    {
      const std::string reason =
        "location already has the requested enablement state";

      finish_mutation_rejected(reason);
      response->updated = false;
      response->result_code =
        SetEnabledService::Response::
          RESULT_INVALID_REQUEST;
      response->reason = reason;
      response->location =
        to_ros_location_record(existing.value());
      publish_status();
      return;
    }

    const auto domain_result =
      catalog.set_location_enabled(
        request->location_id,
        request->expected_record_revision,
        request->enabled);

    if (!domain_result.success) {
      finish_mutation_rejected(
        domain_result.reason);

      response->updated = false;
      response->reason = domain_result.reason;

      switch (domain_result.code) {
        case MutationCode::kNotFound:
          response->result_code =
            SetEnabledService::Response::
              RESULT_NOT_FOUND;
          break;

        case MutationCode::kRetired:
          response->result_code =
            SetEnabledService::Response::
              RESULT_RETIRED;
          break;

        case MutationCode::kStaleRevision:
          response->result_code =
            SetEnabledService::Response::
              RESULT_STALE_REVISION;
          break;

        case MutationCode::kInvalidRecord:
          response->result_code =
            SetEnabledService::Response::
              RESULT_INVALID_REQUEST;
          break;

        default:
          response->result_code =
            SetEnabledService::Response::
              RESULT_INTERNAL_ERROR;
          break;
      }

      publish_status();
      return;
    }

    const auto updated_location =
      domain_result.record.value();

    CatalogSnapshot post;
    post.locations = catalog.list_locations();
    post.candidates = catalog.list_candidates();

    LocationEnabledCommit commit;
    commit.location_id =
      updated_location.location.location_id;
    commit.expected_record_revision =
      request->expected_record_revision;
    commit.enabled = request->enabled;
    commit.actor_id = actor_id;
    commit.reason = mutation_reason;
    commit.payload_json = "{}";
    commit.post_update_snapshot = post;

    std::uint64_t event_sequence = 0U;

    const auto persistence =
      repository_->commit_location_enabled(
        commit,
        &event_sequence);

    if (!persistence.success) {
      finish_mutation_degraded(
        "location enablement persistence failed: " +
        persistence.reason);

      response->updated = false;
      response->reason = persistence.reason;

      if (
        persistence.code ==
        SnapshotCode::kStaleRevision)
      {
        response->result_code =
          SetEnabledService::Response::
            RESULT_STALE_REVISION;
      }
      else
      {
        response->result_code =
          SetEnabledService::Response::
            RESULT_STORAGE_UNAVAILABLE;
      }

      publish_status();
      return;
    }

    finish_mutation_committed(
      std::move(post),
      event_sequence,
      request->enabled ?
        "location_enabled" :
        "location_disabled");

    publish_committed_event(
      event_sequence,
      request->enabled ?
        savo_msgs::msg::LocationEvent::
          EVENT_LOCATION_ENABLED :
        savo_msgs::msg::LocationEvent::
          EVENT_LOCATION_DISABLED,
      "",
      updated_location.location.location_id,
      updated_location.record_revision,
      actor_id,
      mutation_reason);

    if (publish_snapshot_enabled_) {
      publish_snapshot();
    }

    publish_status();

    response->updated = true;
    response->result_code =
      SetEnabledService::Response::RESULT_UPDATED;
    response->reason =
      "location enablement updated; committed event_sequence=" +
      std::to_string(event_sequence);
    response->location =
      to_ros_location_record(updated_location);
  }
  catch (const std::exception & exception) {
    finish_mutation_degraded(
      "location enablement exception: " +
      std::string(exception.what()));

    response->updated = false;
    response->result_code =
      SetEnabledService::Response::RESULT_INTERNAL_ERROR;
    response->reason = exception.what();
    publish_status();
  }
}

}  // namespace savo_locations
