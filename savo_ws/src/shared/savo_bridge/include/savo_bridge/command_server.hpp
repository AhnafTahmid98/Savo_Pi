// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#ifndef SAVO_BRIDGE__COMMAND_SERVER_HPP_
#define SAVO_BRIDGE__COMMAND_SERVER_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "savo_bridge/command_protocol.hpp"

namespace savo_bridge
{

inline constexpr std::size_t DEFAULT_COMMAND_MAX_REQUEST_BYTES = 65536U;
inline constexpr std::size_t DEFAULT_COMMAND_MAX_RESPONSE_BYTES = 65536U;
inline constexpr std::int64_t DEFAULT_COMMAND_ACCEPT_TIMEOUT_MS = 250;
inline constexpr std::int64_t DEFAULT_COMMAND_READ_TIMEOUT_MS = 1000;
inline constexpr std::int64_t DEFAULT_COMMAND_WRITE_TIMEOUT_MS = 1000;
inline constexpr std::uint32_t DEFAULT_COMMAND_SOCKET_MODE = 0660U;

struct CommandServerConfig
{
  bool enabled{false};
  std::string socket_path{"/run/savo_bridge/command.sock"};
  std::size_t max_request_bytes{DEFAULT_COMMAND_MAX_REQUEST_BYTES};
  std::size_t max_response_bytes{DEFAULT_COMMAND_MAX_RESPONSE_BYTES};
  std::int64_t accept_timeout_ms{DEFAULT_COMMAND_ACCEPT_TIMEOUT_MS};
  std::int64_t client_read_timeout_ms{DEFAULT_COMMAND_READ_TIMEOUT_MS};
  std::int64_t client_write_timeout_ms{DEFAULT_COMMAND_WRITE_TIMEOUT_MS};
  std::uint32_t socket_mode{DEFAULT_COMMAND_SOCKET_MODE};
  std::optional<std::uint32_t> socket_gid;
  std::vector<std::uint32_t> allowed_peer_uids;
  std::string bridge_instance_id{"savo-bridge"};
  std::string execution_mode{"dry_run"};

  CommandServerConfig();
};

enum class CommandServerStatus
{
  Disabled,
  Started,
  Stopped,
  NotRunning,
  InvalidConfig,
  UnsafePath,
  SocketError,
  AcceptTimeout,
  AcceptError,
  PeerCredentialsUnavailable,
  UnauthorizedPeer,
  ReadTimeout,
  ReadError,
  EofBeforeNewline,
  EmptyFrame,
  RequestTooLarge,
  InvalidFrame,
  ProtocolError,
  DryRunAcknowledged,
  CommandAcknowledged,
  CommandRejected,
  ResponseTooLarge,
  WriteTimeout,
  WriteError,
};

[[nodiscard]] const char * to_string(
  CommandServerStatus status) noexcept;

struct CommandDispatchResult
{
  bool accepted{false};
  std::string state{"rejected"};
  std::string reason{"bridge_command_dispatch_rejected"};
  bool dispatch_attempted{false};
  std::size_t ros_publications{0U};
};

struct CommandServerResult
{
  CommandServerStatus status{CommandServerStatus::NotRunning};
  std::string reason{"command_server_not_running"};
  std::optional<std::string> command_id;
  std::optional<std::int32_t> peer_pid;
  std::optional<std::uint32_t> peer_uid;
  std::optional<std::uint32_t> peer_gid;
  bool request_received{false};
  bool response_sent{false};
  std::size_t bytes_received{0U};
  std::size_t bytes_sent{0U};
  bool dispatch_attempted{false};
  std::size_t ros_publications{0U};
};

class CommandServer
{
public:
  using Clock = std::function<std::int64_t()>;
  using Dispatcher = std::function<
    CommandDispatchResult(const ValidatedCommand &)>;

  explicit CommandServer(
    CommandServerConfig config,
    Clock clock = {},
    Dispatcher dispatcher = {});
  ~CommandServer();

  CommandServer(const CommandServer &) = delete;
  CommandServer & operator=(const CommandServer &) = delete;
  CommandServer(CommandServer &&) noexcept;
  CommandServer & operator=(CommandServer &&) noexcept;

  [[nodiscard]] CommandServerResult start();
  [[nodiscard]] CommandServerResult serve_one();
  [[nodiscard]] CommandServerResult stop() noexcept;
  [[nodiscard]] bool running() const noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace savo_bridge

#endif  // SAVO_BRIDGE__COMMAND_SERVER_HPP_
