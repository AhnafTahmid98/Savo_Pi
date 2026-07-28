// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include "savo_bridge/command_server.hpp"

#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <charconv>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <string_view>

#include <nlohmann/json.hpp>

namespace
{

using Json = nlohmann::json;

struct FixtureOptions
{
  bool dry_run_fixture{false};
  std::string socket_path;
  std::string bridge_instance_id{"savo-bridge-fixture"};
  std::int64_t accept_timeout_ms{1000};
  std::int64_t read_timeout_ms{1000};
  std::int64_t write_timeout_ms{1000};
};

[[nodiscard]] bool is_safe_identifier(const std::string_view value)
{
  if (value.empty() || value.size() > 128U) {
    return false;
  }

  const auto is_alphanumeric = [](const char character) {
      return
        (character >= 'A' && character <= 'Z') ||
        (character >= 'a' && character <= 'z') ||
        (character >= '0' && character <= '9');
    };
  if (!is_alphanumeric(value.front())) {
    return false;
  }

  for (const char character : value) {
    if (!is_alphanumeric(character) &&
      character != '.' && character != '_' &&
      character != ':' && character != '-')
    {
      return false;
    }
  }
  return true;
}

[[nodiscard]] bool is_safe_socket_path(const std::string_view path)
{
  constexpr std::string_view production_directory{"/run/savo_bridge"};
  if (path.empty() || path.front() != '/') {
    return false;
  }
  const std::string normalized = std::filesystem::path{
    std::string(path)}.lexically_normal().string();
  const std::string production_prefix =
    std::string(production_directory) + "/";
  return normalized != production_directory &&
         normalized.rfind(production_prefix, 0U) != 0U;
}

[[nodiscard]] std::optional<std::int64_t> parse_positive_timeout(
  const std::string_view value)
{
  std::int64_t parsed{0};
  const auto conversion = std::from_chars(
    value.data(), value.data() + value.size(), parsed);
  if (conversion.ec != std::errc{} ||
    conversion.ptr != value.data() + value.size() ||
    parsed <= 0 ||
    parsed > std::numeric_limits<int>::max())
  {
    return std::nullopt;
  }
  return parsed;
}

[[nodiscard]] std::optional<FixtureOptions> parse_arguments(
  const int argument_count,
  char ** arguments)
{
  FixtureOptions options;
  std::set<std::string> supplied_options;

  for (int index = 1; index < argument_count; ++index) {
    const std::string option{arguments[index]};
    if (!supplied_options.insert(option).second) {
      return std::nullopt;
    }
    if (option == "--dry-run-fixture") {
      options.dry_run_fixture = true;
      continue;
    }
    if (index + 1 >= argument_count) {
      return std::nullopt;
    }

    const std::string value{arguments[++index]};
    if (option == "--socket-path") {
      options.socket_path = value;
    } else if (option == "--bridge-instance-id") {
      options.bridge_instance_id = value;
    } else if (option == "--accept-timeout-ms") {
      const auto timeout = parse_positive_timeout(value);
      if (!timeout.has_value()) {
        return std::nullopt;
      }
      options.accept_timeout_ms = timeout.value();
    } else if (option == "--read-timeout-ms") {
      const auto timeout = parse_positive_timeout(value);
      if (!timeout.has_value()) {
        return std::nullopt;
      }
      options.read_timeout_ms = timeout.value();
    } else if (option == "--write-timeout-ms") {
      const auto timeout = parse_positive_timeout(value);
      if (!timeout.has_value()) {
        return std::nullopt;
      }
      options.write_timeout_ms = timeout.value();
    } else {
      return std::nullopt;
    }
  }

  if (!options.dry_run_fixture ||
    !is_safe_socket_path(options.socket_path) ||
    !is_safe_identifier(options.bridge_instance_id))
  {
    return std::nullopt;
  }
  return options;
}

[[nodiscard]] bool path_is_absent(const std::string & path)
{
  struct stat metadata {};
  return ::lstat(path.c_str(), &metadata) != 0 && errno == ENOENT;
}

template<typename Value>
void set_optional(
  Json & summary,
  const std::string_view name,
  const std::optional<Value> & value)
{
  if (value.has_value()) {
    summary[std::string(name)] = value.value();
  } else {
    summary[std::string(name)] = nullptr;
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  const auto options = parse_arguments(argc, argv);
  if (!options.has_value()) {
    std::cerr << "invalid or unsafe fixture arguments\n";
    return 2;
  }

  savo_bridge::CommandServerConfig config;
  config.enabled = true;
  config.socket_path = options->socket_path;
  config.bridge_instance_id = options->bridge_instance_id;
  config.accept_timeout_ms = options->accept_timeout_ms;
  config.client_read_timeout_ms = options->read_timeout_ms;
  config.client_write_timeout_ms = options->write_timeout_ms;
  config.socket_mode = 0600U;
  config.max_request_bytes = 65536U;
  config.max_response_bytes = 65536U;
  config.allowed_peer_uids = {
    static_cast<std::uint32_t>(::geteuid())
  };

  savo_bridge::CommandServer server{config};
  const auto start_result = server.start();
  if (start_result.status != savo_bridge::CommandServerStatus::Started) {
    std::cerr << "fixture server startup failed: "
              << start_result.reason << '\n';
    return 3;
  }

  std::cout << "READY " << options->socket_path << '\n' << std::flush;

  const auto server_result = server.serve_one();
  const auto stop_result = server.stop();
  const bool socket_removed = path_is_absent(options->socket_path);
  const bool succeeded =
    server_result.status ==
    savo_bridge::CommandServerStatus::DryRunAcknowledged &&
    server_result.command_id.has_value() &&
    server_result.request_received &&
    server_result.response_sent &&
    !server_result.dispatch_attempted &&
    server_result.ros_publications == 0U &&
    stop_result.status == savo_bridge::CommandServerStatus::Stopped &&
    socket_removed;

  Json summary{
    {"fixture_status", succeeded ? "passed" : "failed"},
    {"server_status", savo_bridge::to_string(server_result.status)},
    {"reason", server_result.reason},
    {"request_received", server_result.request_received},
    {"response_sent", server_result.response_sent},
    {"bytes_received", server_result.bytes_received},
    {"bytes_sent", server_result.bytes_sent},
    {"dispatch_attempted", server_result.dispatch_attempted},
    {"ros_publications", server_result.ros_publications},
    {"socket_removed", socket_removed},
  };
  set_optional(summary, "command_id", server_result.command_id);
  set_optional(summary, "peer_pid", server_result.peer_pid);
  set_optional(summary, "peer_uid", server_result.peer_uid);
  set_optional(summary, "peer_gid", server_result.peer_gid);
  std::cout << summary.dump() << '\n' << std::flush;

  return succeeded ? 0 : 4;
}
