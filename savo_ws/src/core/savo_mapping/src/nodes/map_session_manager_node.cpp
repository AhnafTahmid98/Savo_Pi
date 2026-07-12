#include "savo_mapping/map_session_policy.hpp"

#include <slam_toolbox/srv/save_map.hpp>
#include <slam_toolbox/srv/serialize_pose_graph.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

namespace savo_mapping
{

namespace
{

namespace fs = std::filesystem;

using SaveMap = slam_toolbox::srv::SaveMap;
using SerializePoseGraph =
  slam_toolbox::srv::SerializePoseGraph;
using Trigger = std_srvs::srv::Trigger;

std::string json_escape(
  std::string_view input)
{
  std::string output;
  output.reserve(input.size());

  for (const char character : input) {
    switch (character) {
      case '\\':
        output += "\\\\";
        break;

      case '"':
        output += "\\\"";
        break;

      case '\n':
        output += "\\n";
        break;

      case '\r':
        output += "\\r";
        break;

      case '\t':
        output += "\\t";
        break;

      default:
        output += character;
        break;
    }
  }

  return output;
}

std::chrono::milliseconds
seconds_to_milliseconds(
  double seconds)
{
  return std::chrono::duration_cast<
    std::chrono::milliseconds>(
    std::chrono::duration<double>(seconds));
}

std::int64_t unix_time_nanoseconds()
{
  return std::chrono::duration_cast<
    std::chrono::nanoseconds>(
    std::chrono::system_clock::now()
      .time_since_epoch()).count();
}

class OperationGuard
{
public:
  explicit OperationGuard(
    std::atomic_bool & flag)
  : flag_(flag)
  {
  }

  ~OperationGuard()
  {
    flag_.store(false);
  }

private:
  std::atomic_bool & flag_;
};

}  // namespace

class MapSessionManagerNode final
  : public rclcpp::Node
{
public:
  MapSessionManagerNode()
  : Node("map_session_manager_node")
  {
    declare_parameters();
    create_interfaces();

    publish_state("idle");

    RCLCPP_INFO(
      get_logger(),
      "map-session manager ready; output_root=%s",
      get_parameter(
        "session.output_root")
      .as_string().c_str());

    RCLCPP_INFO(
      get_logger(),
      "session manager owns save/serialize requests "
      "but does not control robot movement or "
      "slam_toolbox lifecycle");
  }

private:
  struct RuntimeInputs
  {
    bool mapping_ready{false};
    bool slam_active{false};
    bool map_structurally_valid{false};

    std::string readiness{
      "not_ready: unavailable"};

    std::string lifecycle_state{"unknown"};
  };

  void declare_parameters()
  {
    declare_parameter<std::string>(
      "session.map_id",
      "");

    declare_parameter<std::string>(
      "session.output_root",
      "~/Savo_Pi/runtime/maps");

    declare_parameter<bool>(
      "session.allow_overwrite",
      false);

    declare_parameter<bool>(
      "session.require_mapping_ready",
      true);

    declare_parameter<bool>(
      "session.require_slam_active",
      true);

    declare_parameter<bool>(
      "session.require_structurally_valid_map",
      true);

    declare_parameter<double>(
      "session.service_timeout_s",
      15.0);

    declare_parameter<std::string>(
      "services.trigger_save",
      "/savo_mapping/map_session/save");

    declare_parameter<std::string>(
      "services.slam_save_map",
      "/slam_toolbox/save_map");

    declare_parameter<std::string>(
      "services.slam_serialize_map",
      "/slam_toolbox/serialize_map");

    declare_parameter<std::string>(
      "topics.readiness",
      "/savo_mapping/readiness");

    declare_parameter<std::string>(
      "topics.slam_lifecycle_state",
      "/savo_mapping/slam_lifecycle_state");

    declare_parameter<std::string>(
      "topics.map_quality",
      "/savo_mapping/map_quality");

    declare_parameter<std::string>(
      "topics.session_state",
      "/savo_mapping/map_session/state");

    declare_parameter<std::string>(
      "topics.session_result",
      "/savo_mapping/map_session/result");

    declare_parameter<std::string>(
      "topics.session_manifest",
      "/savo_mapping/map_session/manifest");

    const double timeout =
      get_parameter(
        "session.service_timeout_s")
      .as_double();

    if (timeout <= 0.0) {
      throw std::invalid_argument(
              "session.service_timeout_s_must_be_positive");
    }
  }

  void create_interfaces()
  {
    rclcpp::QoS state_qos{
      rclcpp::KeepLast(1)};

    state_qos.reliable();
    state_qos.transient_local();

    state_publisher_ =
      create_publisher<std_msgs::msg::String>(
      get_parameter(
        "topics.session_state")
      .as_string(),
      state_qos);

    result_publisher_ =
      create_publisher<std_msgs::msg::String>(
      get_parameter(
        "topics.session_result")
      .as_string(),
      state_qos);

    manifest_publisher_ =
      create_publisher<std_msgs::msg::String>(
      get_parameter(
        "topics.session_manifest")
      .as_string(),
      state_qos);

    readiness_subscription_ =
      create_subscription<std_msgs::msg::String>(
      get_parameter(
        "topics.readiness")
      .as_string(),
      state_qos,
      [this](
        const std_msgs::msg::String::
        ConstSharedPtr message)
      {
        std::lock_guard<std::mutex> lock{
          inputs_mutex_};

        inputs_.readiness = message->data;
        inputs_.mapping_ready =
          message->data == "ready";
      });

    lifecycle_subscription_ =
      create_subscription<std_msgs::msg::String>(
      get_parameter(
        "topics.slam_lifecycle_state")
      .as_string(),
      state_qos,
      [this](
        const std_msgs::msg::String::
        ConstSharedPtr message)
      {
        std::lock_guard<std::mutex> lock{
          inputs_mutex_};

        inputs_.lifecycle_state =
          message->data;

        inputs_.slam_active =
          message->data == "active";
      });

    map_quality_subscription_ =
      create_subscription<std_msgs::msg::String>(
      get_parameter(
        "topics.map_quality")
      .as_string(),
      state_qos,
      [this](
        const std_msgs::msg::String::
        ConstSharedPtr message)
      {
        const std::string & data =
          message->data;

        const bool structurally_valid =
          data.find(
            "\"structurally_valid\":true") !=
          std::string::npos;

        std::lock_guard<std::mutex> lock{
          inputs_mutex_};

        inputs_.map_structurally_valid =
          structurally_valid;
      });

    service_callback_group_ =
      create_callback_group(
      rclcpp::CallbackGroupType::
      MutuallyExclusive);

    client_callback_group_ =
      create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

    save_map_client_ =
      create_client<SaveMap>(
      get_parameter(
        "services.slam_save_map")
      .as_string(),
      rclcpp::ServicesQoS{},
      client_callback_group_);

    serialize_map_client_ =
      create_client<SerializePoseGraph>(
      get_parameter(
        "services.slam_serialize_map")
      .as_string(),
      rclcpp::ServicesQoS{},
      client_callback_group_);

    save_service_ =
      create_service<Trigger>(
      get_parameter(
        "services.trigger_save")
      .as_string(),
      std::bind(
        &MapSessionManagerNode::on_save,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      rclcpp::ServicesQoS{},
      service_callback_group_);
  }

  RuntimeInputs input_snapshot()
  {
    std::lock_guard<std::mutex> lock{
      inputs_mutex_};

    return inputs_;
  }

  void publish_state(
    const std::string & state)
  {
    std_msgs::msg::String message;
    message.data = state;

    state_publisher_->publish(message);
  }

  void publish_result(
    bool success,
    const std::string & reason,
    const std::string & map_id,
    const fs::path & session_directory)
  {
    std::ostringstream stream;

    stream
      << std::boolalpha
      << "{"
      << "\"success\":" << success
      << ",\"reason\":\""
      << json_escape(reason)
      << "\",\"map_id\":\""
      << json_escape(map_id)
      << "\",\"session_directory\":\""
      << json_escape(
        session_directory.generic_string())
      << "\"}";

    std_msgs::msg::String message;
    message.data = stream.str();

    result_publisher_->publish(message);
  }

  void reject_request(
    const std::shared_ptr<
      Trigger::Response> & response,
    const std::string & reason,
    const std::string & map_id,
    const fs::path & session_directory)
  {
    publish_state("blocked");

    publish_result(
      false,
      reason,
      map_id,
      session_directory);

    response->success = false;
    response->message = reason;

    RCLCPP_WARN(
      get_logger(),
      "map-session save rejected: %s",
      reason.c_str());
  }

  void fail_operation(
    const std::shared_ptr<
      Trigger::Response> & response,
    const std::string & reason,
    const std::string & map_id,
    const fs::path & session_directory)
  {
    publish_state("error");

    publish_result(
      false,
      reason,
      map_id,
      session_directory);

    response->success = false;
    response->message = reason;

    RCLCPP_ERROR(
      get_logger(),
      "map-session save failed: %s",
      reason.c_str());
  }

  bool call_save_map(
    const fs::path & base,
    std::chrono::milliseconds timeout,
    std::string & failure_reason)
  {
    if (!save_map_client_->wait_for_service(
        timeout))
    {
      failure_reason =
        "slam_save_map_service_unavailable";

      return false;
    }

    auto request =
      std::make_shared<SaveMap::Request>();

    request->name.data =
      base.generic_string();

    auto future =
      save_map_client_->async_send_request(
      request);

    if (future.wait_for(timeout) !=
        std::future_status::ready)
    {
      failure_reason =
        "slam_save_map_service_timeout";

      return false;
    }

    const auto response = future.get();

    if (!response) {
      failure_reason =
        "slam_save_map_empty_response";

      return false;
    }

    if (response->result !=
        SaveMap::Response::RESULT_SUCCESS)
    {
      std::ostringstream stream;

      stream
        << "slam_save_map_failed_result_"
        << static_cast<unsigned int>(
          response->result);

      failure_reason = stream.str();
      return false;
    }

    return true;
  }

  bool call_serialize_map(
    const fs::path & base,
    std::chrono::milliseconds timeout,
    std::string & failure_reason)
  {
    if (!serialize_map_client_->
        wait_for_service(timeout))
    {
      failure_reason =
        "slam_serialize_map_service_unavailable";

      return false;
    }

    auto request =
      std::make_shared<
      SerializePoseGraph::Request>();

    request->filename =
      base.generic_string();

    auto future =
      serialize_map_client_->
      async_send_request(request);

    if (future.wait_for(timeout) !=
        std::future_status::ready)
    {
      failure_reason =
        "slam_serialize_map_service_timeout";

      return false;
    }

    const auto response = future.get();

    if (!response) {
      failure_reason =
        "slam_serialize_map_empty_response";

      return false;
    }

    if (response->result !=
        SerializePoseGraph::Response::
        RESULT_SUCCESS)
    {
      std::ostringstream stream;

      stream
        << "slam_serialize_map_failed_result_"
        << static_cast<unsigned int>(
          response->result);

      failure_reason = stream.str();
      return false;
    }

    return true;
  }

  fs::path select_grid_image(
    const session::MapArtifactPaths & paths)
  {
    if (fs::is_regular_file(
        paths.grid_pgm))
    {
      return paths.grid_pgm;
    }

    if (fs::is_regular_file(
        paths.grid_png))
    {
      return paths.grid_png;
    }

    return {};
  }

  void write_manifest(
    const fs::path & manifest_path,
    const std::string & map_id,
    const fs::path & final_directory,
    const fs::path & final_image)
  {
    const fs::path final_base =
      final_directory / map_id;

    const auto final_artifacts =
      session::build_artifact_paths(
      final_base);

    const fs::path temporary =
      manifest_path.string() + ".tmp";

    std::ofstream output{temporary};

    if (!output.is_open()) {
      throw std::runtime_error(
              "manifest_open_failed");
    }

    output
      << "schema_version: 1\n"
      << "map_id: \"" << map_id << "\"\n"
      << "frame_id: \"map\"\n"
      << "created_unix_ns: "
      << unix_time_nanoseconds() << "\n"
      << "session_directory: \""
      << final_directory.generic_string()
      << "\"\n"
      << "occupancy_grid:\n"
      << "  yaml: \""
      << final_artifacts.grid_yaml
        .generic_string()
      << "\"\n"
      << "  image: \""
      << (
        final_directory /
        final_image.filename())
        .generic_string()
      << "\"\n"
      << "pose_graph:\n"
      << "  posegraph: \""
      << final_artifacts.posegraph
        .generic_string()
      << "\"\n"
      << "  data: \""
      << final_artifacts.posegraph_data
        .generic_string()
      << "\"\n"
      << "map_quality:\n"
      << "  structurally_valid: true\n"
      << "  evaluated: false\n"
      << "navigation_handoff_ready: false\n"
      << "location_link:\n"
      << "  map_id: \"" << map_id << "\"\n"
      << "  registered: false\n";

    output.close();

    if (!output) {
      throw std::runtime_error(
              "manifest_write_failed");
    }

    fs::rename(temporary, manifest_path);
  }

  void publish_manifest(
    const std::string & map_id,
    const fs::path & final_directory)
  {
    const auto artifacts =
      session::build_artifact_paths(
      final_directory / map_id);

    fs::path image = artifacts.grid_pgm;

    if (!fs::is_regular_file(image)) {
      image = artifacts.grid_png;
    }

    std::ostringstream stream;

    stream
      << "{"
      << "\"schema_version\":1"
      << ",\"map_id\":\""
      << json_escape(map_id)
      << "\",\"frame_id\":\"map\""
      << ",\"session_directory\":\""
      << json_escape(
        final_directory.generic_string())
      << "\",\"grid_yaml\":\""
      << json_escape(
        artifacts.grid_yaml.generic_string())
      << "\",\"grid_image\":\""
      << json_escape(image.generic_string())
      << "\",\"posegraph\":\""
      << json_escape(
        artifacts.posegraph.generic_string())
      << "\",\"posegraph_data\":\""
      << json_escape(
        artifacts.posegraph_data.generic_string())
      << "\",\"structurally_valid\":true"
      << ",\"quality_evaluated\":false"
      << ",\"navigation_handoff_ready\":false"
      << ",\"location_registered\":false"
      << "}";

    std_msgs::msg::String message;
    message.data = stream.str();

    manifest_publisher_->publish(message);
  }

  void commit_staging_directory(
    const fs::path & staging_directory,
    const fs::path & final_directory,
    bool allow_overwrite,
    std::int64_t operation_id)
  {
    fs::path backup_directory;

    if (fs::exists(final_directory)) {
      if (!allow_overwrite) {
        throw std::runtime_error(
                "map_session_already_exists");
      }

      backup_directory =
        final_directory.parent_path() /
        (
          "." +
          final_directory.filename().string() +
          "_backup_" +
          std::to_string(operation_id));

      fs::rename(
        final_directory,
        backup_directory);
    }

    try {
      fs::rename(
        staging_directory,
        final_directory);
    } catch (...) {
      if (!backup_directory.empty() &&
          fs::exists(backup_directory) &&
          !fs::exists(final_directory))
      {
        std::error_code restore_error;

        fs::rename(
          backup_directory,
          final_directory,
          restore_error);
      }

      throw;
    }

    if (!backup_directory.empty()) {
      std::error_code cleanup_error;

      fs::remove_all(
        backup_directory,
        cleanup_error);
    }
  }

  void on_save(
    const std::shared_ptr<
      Trigger::Request>,
    std::shared_ptr<
      Trigger::Response> response)
  {
    if (operation_in_progress_.exchange(true)) {
      response->success = false;
      response->message =
        "map_session_operation_already_in_progress";

      return;
    }

    OperationGuard guard{
      operation_in_progress_};

    const std::string map_id =
      get_parameter(
        "session.map_id")
      .as_string();

    const std::string raw_output_root =
      get_parameter(
        "session.output_root")
      .as_string();

    const bool allow_overwrite =
      get_parameter(
        "session.allow_overwrite")
      .as_bool();

    const bool require_mapping_ready =
      get_parameter(
        "session.require_mapping_ready")
      .as_bool();

    const bool require_slam_active =
      get_parameter(
        "session.require_slam_active")
      .as_bool();

    const bool require_valid_map =
      get_parameter(
        "session.require_structurally_valid_map")
      .as_bool();

    const double service_timeout_s =
      get_parameter(
        "session.service_timeout_s")
      .as_double();

    fs::path output_root;
    session::MapSessionPaths final_paths;

    try {
      const std::string map_id_error =
        session::validate_map_id(map_id);

      if (!map_id_error.empty()) {
        reject_request(
          response,
          map_id_error,
          map_id,
          {});

        return;
      }

      output_root =
        session::expand_user_path(
        raw_output_root)
        .lexically_normal();

      const std::string root_error =
        session::validate_output_root(
        output_root);

      if (!root_error.empty()) {
        reject_request(
          response,
          root_error,
          map_id,
          {});

        return;
      }

      final_paths =
        session::build_session_paths(
        output_root,
        map_id);

      const RuntimeInputs inputs =
        input_snapshot();

      session::SaveReadiness readiness;

      readiness.mapping_ready =
        inputs.mapping_ready;

      readiness.slam_active =
        inputs.slam_active;

      readiness.map_structurally_valid =
        inputs.map_structurally_valid;

      const std::string readiness_error =
        session::validate_save_readiness(
        readiness,
        require_mapping_ready,
        require_slam_active,
        require_valid_map);

      if (!readiness_error.empty()) {
        reject_request(
          response,
          readiness_error,
          map_id,
          final_paths.session_directory);

        return;
      }

      if (!allow_overwrite &&
          session::session_target_exists(
            final_paths))
      {
        reject_request(
          response,
          "map_session_already_exists",
          map_id,
          final_paths.session_directory);

        return;
      }

      publish_state("validating");

      fs::create_directories(output_root);

      const std::int64_t operation_id =
        unix_time_nanoseconds();

      const fs::path staging_directory =
        output_root /
        (
          "." + map_id +
          "_staging_" +
          std::to_string(operation_id));

      fs::create_directory(
        staging_directory);

      const fs::path staging_base =
        staging_directory / map_id;

      const auto staging_artifacts =
        session::build_artifact_paths(
        staging_base);

      const auto timeout =
        seconds_to_milliseconds(
        service_timeout_s);

      std::string failure_reason;

      publish_state("saving_grid");

      if (!call_save_map(
          staging_base,
          timeout,
          failure_reason))
      {
        std::error_code cleanup_error;

        fs::remove_all(
          staging_directory,
          cleanup_error);

        fail_operation(
          response,
          failure_reason,
          map_id,
          final_paths.session_directory);

        return;
      }

      publish_state(
        "serializing_pose_graph");

      if (!call_serialize_map(
          staging_base,
          timeout,
          failure_reason))
      {
        std::error_code cleanup_error;

        fs::remove_all(
          staging_directory,
          cleanup_error);

        fail_operation(
          response,
          failure_reason,
          map_id,
          final_paths.session_directory);

        return;
      }

      const auto missing =
        session::missing_required_artifacts(
        staging_artifacts);

      if (!missing.empty()) {
        std::ostringstream reason;

        reason
          << "saved_artifact_missing:"
          << missing.front().filename().string();

        std::error_code cleanup_error;

        fs::remove_all(
          staging_directory,
          cleanup_error);

        fail_operation(
          response,
          reason.str(),
          map_id,
          final_paths.session_directory);

        return;
      }

      const fs::path staging_image =
        select_grid_image(
        staging_artifacts);

      publish_state("writing_manifest");

      write_manifest(
        staging_directory /
        "manifest.yaml",
        map_id,
        final_paths.session_directory,
        staging_image);

      commit_staging_directory(
        staging_directory,
        final_paths.session_directory,
        allow_overwrite,
        operation_id);

      publish_manifest(
        map_id,
        final_paths.session_directory);

      publish_state("completed");

      publish_result(
        true,
        "map_session_saved",
        map_id,
        final_paths.session_directory);

      response->success = true;

      response->message =
        "map_session_saved:" +
        final_paths.session_directory
        .generic_string();

      RCLCPP_INFO(
        get_logger(),
        "map session saved successfully: "
        "map_id=%s directory=%s",
        map_id.c_str(),
        final_paths.session_directory
        .c_str());
    } catch (const std::exception & exception) {
      fail_operation(
        response,
        exception.what(),
        map_id,
        final_paths.session_directory);
    }
  }

  std::atomic_bool operation_in_progress_{
    false};

  std::mutex inputs_mutex_;
  RuntimeInputs inputs_;

  rclcpp::CallbackGroup::SharedPtr
    service_callback_group_;

  rclcpp::CallbackGroup::SharedPtr
    client_callback_group_;

  rclcpp::Client<SaveMap>::SharedPtr
    save_map_client_;

  rclcpp::Client<
    SerializePoseGraph>::SharedPtr
    serialize_map_client_;

  rclcpp::Service<Trigger>::SharedPtr
    save_service_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    state_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    result_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    manifest_publisher_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    readiness_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    lifecycle_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    map_quality_subscription_;
};

}  // namespace savo_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    const auto node =
      std::make_shared<
        savo_mapping::MapSessionManagerNode>();

    rclcpp::executors::
    MultiThreadedExecutor executor{
      rclcpp::ExecutorOptions(),
      4};

    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & exception) {
    std::cerr
      << "map_session_manager_node failed: "
      << exception.what()
      << '\n';

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
