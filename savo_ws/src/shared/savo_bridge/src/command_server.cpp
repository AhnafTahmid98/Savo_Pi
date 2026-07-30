// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include "savo_bridge/command_server.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <exception>
#include <filesystem>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>

#include <nlohmann/json.hpp>

#include "savo_bridge/command_protocol.hpp"

namespace savo_bridge
{
namespace
{

using Json = nlohmann::json;
using SteadyClock = std::chrono::steady_clock;

class FileDescriptor
{
public:
  FileDescriptor() = default;
  explicit FileDescriptor(const int descriptor) noexcept
  : descriptor_(descriptor)
  {
  }

  ~FileDescriptor()
  {
    reset();
  }

  FileDescriptor(const FileDescriptor &) = delete;
  FileDescriptor & operator=(const FileDescriptor &) = delete;

  FileDescriptor(FileDescriptor && other) noexcept
  : descriptor_(other.release())
  {
  }

  FileDescriptor & operator=(FileDescriptor && other) noexcept
  {
    if (this != &other) {
      reset(other.release());
    }
    return *this;
  }

  [[nodiscard]] int get() const noexcept
  {
    return descriptor_;
  }

  [[nodiscard]] bool valid() const noexcept
  {
    return descriptor_ >= 0;
  }

  [[nodiscard]] int release() noexcept
  {
    const int descriptor = descriptor_;
    descriptor_ = -1;
    return descriptor;
  }

  void reset(const int descriptor = -1) noexcept
  {
    if (descriptor_ >= 0) {
      (void)::close(descriptor_);
    }
    descriptor_ = descriptor;
  }

private:
  int descriptor_{-1};
};

class UmaskGuard
{
public:
  explicit UmaskGuard(const mode_t mask) noexcept
  : previous_(::umask(mask))
  {
  }

  ~UmaskGuard()
  {
    (void)::umask(previous_);
  }

  UmaskGuard(const UmaskGuard &) = delete;
  UmaskGuard & operator=(const UmaskGuard &) = delete;

private:
  mode_t previous_;
};

struct PeerCredentials
{
  std::int32_t pid;
  std::uint32_t uid;
  std::uint32_t gid;
};

struct FrameReadResult
{
  CommandServerStatus status{CommandServerStatus::ReadError};
  std::string reason{"command_server_read_error"};
  std::string frame;
  std::size_t bytes_received{0U};

  [[nodiscard]] bool succeeded() const noexcept
  {
    return status == CommandServerStatus::DryRunAcknowledged;
  }
};

[[nodiscard]] CommandServerResult make_result(
  const CommandServerStatus status,
  std::string reason)
{
  CommandServerResult result;
  result.status = status;
  result.reason = std::move(reason);
  return result;
}

[[nodiscard]] bool is_safe_identifier(
  const std::string_view value) noexcept
{
  if (value.empty() || value.size() > 128U) {
    return false;
  }

  const auto alphanumeric = [](const char character) {
      return
        (character >= 'A' && character <= 'Z') ||
        (character >= 'a' && character <= 'z') ||
        (character >= '0' && character <= '9');
    };

  if (!alphanumeric(value.front())) {
    return false;
  }

  for (const char character : value.substr(1U)) {
    if (!alphanumeric(character) &&
      character != '.' &&
      character != '_' &&
      character != ':' &&
      character != '-')
    {
      return false;
    }
  }

  return true;
}

[[nodiscard]] bool valid_utf8(
  const std::string_view value) noexcept
{
  std::size_t index = 0U;

  while (index < value.size()) {
    const auto first = static_cast<unsigned char>(value[index]);
    std::size_t continuation_count = 0U;
    std::uint32_t code_point = 0U;
    std::uint32_t minimum = 0U;

    if (first <= 0x7FU) {
      ++index;
      continue;
    }
    if ((first & 0xE0U) == 0xC0U) {
      continuation_count = 1U;
      code_point = first & 0x1FU;
      minimum = 0x80U;
    } else if ((first & 0xF0U) == 0xE0U) {
      continuation_count = 2U;
      code_point = first & 0x0FU;
      minimum = 0x800U;
    } else if ((first & 0xF8U) == 0xF0U) {
      continuation_count = 3U;
      code_point = first & 0x07U;
      minimum = 0x10000U;
    } else {
      return false;
    }

    if (index + continuation_count >= value.size()) {
      return false;
    }

    for (std::size_t offset = 1U;
      offset <= continuation_count;
      ++offset)
    {
      const auto next =
        static_cast<unsigned char>(value[index + offset]);
      if ((next & 0xC0U) != 0x80U) {
        return false;
      }
      code_point = (code_point << 6U) | (next & 0x3FU);
    }

    if (code_point < minimum ||
      code_point > 0x10FFFFU ||
      (code_point >= 0xD800U && code_point <= 0xDFFFU))
    {
      return false;
    }

    index += continuation_count + 1U;
  }

  return true;
}

[[nodiscard]] std::int64_t default_now_unix_ms() noexcept
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

[[nodiscard]] int remaining_timeout_ms(
  const SteadyClock::time_point deadline) noexcept
{
  const auto remaining = std::chrono::duration_cast<
    std::chrono::milliseconds>(deadline - SteadyClock::now()).count();

  if (remaining <= 0) {
    return 0;
  }

  return static_cast<int>(std::min<std::int64_t>(
      remaining,
      std::numeric_limits<int>::max()));
}

[[nodiscard]] int poll_until(
  const int descriptor,
  const std::int16_t events,
  const SteadyClock::time_point deadline) noexcept
{
  while (true) {
    pollfd descriptor_state{};
    descriptor_state.fd = descriptor;
    descriptor_state.events = events;

    const int timeout = remaining_timeout_ms(deadline);
    const int result = ::poll(&descriptor_state, 1, timeout);

    if (result > 0) {
      return 1;
    }
    if (result == 0) {
      return 0;
    }
    if (errno != EINTR) {
      return -1;
    }
    if (remaining_timeout_ms(deadline) == 0) {
      return 0;
    }
  }
}

[[nodiscard]] bool set_close_on_exec(
  const int descriptor) noexcept
{
  const int flags = ::fcntl(descriptor, F_GETFD);
  if (flags < 0) {
    return false;
  }
  return ::fcntl(descriptor, F_SETFD, flags | FD_CLOEXEC) == 0;
}

[[nodiscard]] bool set_nonblocking(
  const int descriptor) noexcept
{
  const int flags = ::fcntl(descriptor, F_GETFL);
  if (flags < 0) {
    return false;
  }
  return ::fcntl(descriptor, F_SETFL, flags | O_NONBLOCK) == 0;
}

[[nodiscard]] FileDescriptor create_unix_socket() noexcept
{
  int type = SOCK_STREAM;
#ifdef SOCK_CLOEXEC
  type |= SOCK_CLOEXEC;
#endif
  FileDescriptor descriptor{::socket(AF_UNIX, type, 0)};

  if (descriptor.valid() && !set_close_on_exec(descriptor.get())) {
    descriptor.reset();
  }

  return descriptor;
}

[[nodiscard]] std::optional<PeerCredentials> peer_credentials(
  const int descriptor) noexcept
{
#ifdef SO_PEERCRED
  ucred credentials{};
  socklen_t size = sizeof(credentials);
  if (::getsockopt(
      descriptor,
      SOL_SOCKET,
      SO_PEERCRED,
      &credentials,
      &size) != 0 ||
    size != sizeof(credentials))
  {
    return std::nullopt;
  }

  return PeerCredentials{
    static_cast<std::int32_t>(credentials.pid),
    static_cast<std::uint32_t>(credentials.uid),
    static_cast<std::uint32_t>(credentials.gid),
  };
#else
  (void)descriptor;
  return std::nullopt;
#endif
}

[[nodiscard]] bool peer_allowed(
  const std::vector<std::uint32_t> & allowed,
  const std::uint32_t uid) noexcept
{
  return std::find(allowed.begin(), allowed.end(), uid) != allowed.end();
}

[[nodiscard]] bool safe_parent_directory(
  const std::filesystem::path & socket_path) noexcept
{
  try {
    const auto parent = socket_path.parent_path();
    if (parent.empty() || !parent.is_absolute()) {
      return false;
    }

    std::filesystem::path current{"/"};
    for (const auto & component : parent.relative_path()) {
      if (component == "." || component == "..") {
        return false;
      }
      current /= component;

      struct stat metadata {};
      if (::lstat(current.c_str(), &metadata) != 0 ||
        S_ISLNK(metadata.st_mode) ||
        !S_ISDIR(metadata.st_mode))
      {
        return false;
      }
    }
    return true;
  } catch (...) {
    return false;
  }
}

[[nodiscard]] bool same_identity(
  const struct stat & first,
  const struct stat & second) noexcept
{
  return first.st_dev == second.st_dev && first.st_ino == second.st_ino;
}

void remove_if_identity(
  const std::string & path,
  const struct stat & expected) noexcept
{
  struct stat current {};
  if (::lstat(path.c_str(), &current) == 0 &&
    S_ISSOCK(current.st_mode) &&
    same_identity(expected, current))
  {
    (void)::unlink(path.c_str());
  }
}

enum class ExistingPathState
{
  Absent,
  StaleSocket,
  Unsafe,
};

[[nodiscard]] ExistingPathState inspect_existing_path(
  const std::string & path,
  struct stat & stale_identity) noexcept
{
  struct stat metadata {};
  if (::lstat(path.c_str(), &metadata) != 0) {
    return errno == ENOENT ?
           ExistingPathState::Absent :
           ExistingPathState::Unsafe;
  }

  if (!S_ISSOCK(metadata.st_mode)) {
    return ExistingPathState::Unsafe;
  }

  auto probe = create_unix_socket();
  if (!probe.valid() || !set_nonblocking(probe.get())) {
    return ExistingPathState::Unsafe;
  }

  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::memcpy(address.sun_path, path.c_str(), path.size() + 1U);

  if (::connect(
      probe.get(),
      reinterpret_cast<const sockaddr *>(&address),
      sizeof(address)) == 0)
  {
    return ExistingPathState::Unsafe;
  }

  if (errno != ECONNREFUSED) {
    return ExistingPathState::Unsafe;
  }

  struct stat after_probe {};
  if (::lstat(path.c_str(), &after_probe) != 0 ||
    !S_ISSOCK(after_probe.st_mode) ||
    !same_identity(metadata, after_probe))
  {
    return ExistingPathState::Unsafe;
  }

  stale_identity = after_probe;
  return ExistingPathState::StaleSocket;
}

[[nodiscard]] FrameReadResult read_frame(
  const int descriptor,
  const std::size_t maximum_bytes,
  const std::int64_t timeout_ms) noexcept
{
  FrameReadResult result;
  std::string bytes;
  bytes.reserve(std::min<std::size_t>(maximum_bytes + 2U, 4096U));

  const auto deadline = SteadyClock::now() +
    std::chrono::milliseconds(timeout_ms);

  while (true) {
    const int readiness = poll_until(descriptor, POLLIN, deadline);
    if (readiness == 0) {
      result.status = CommandServerStatus::ReadTimeout;
      result.reason = "command_server_client_read_timeout";
      result.bytes_received = bytes.size();
      return result;
    }
    if (readiness < 0) {
      result.status = CommandServerStatus::ReadError;
      result.reason = "command_server_client_read_failed";
      result.bytes_received = bytes.size();
      return result;
    }

    std::array<char, 4096U> chunk{};
    const std::size_t remaining = maximum_bytes + 2U -
      std::min(maximum_bytes + 2U, bytes.size());
    const std::size_t requested = std::min(chunk.size(), remaining);

    const ssize_t count = ::recv(descriptor, chunk.data(), requested, 0);
    if (count < 0) {
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      result.status = CommandServerStatus::ReadError;
      result.reason = "command_server_client_read_failed";
      result.bytes_received = bytes.size();
      return result;
    }
    if (count == 0) {
      result.status = CommandServerStatus::EofBeforeNewline;
      result.reason = "command_server_eof_before_newline";
      result.bytes_received = bytes.size();
      return result;
    }

    bytes.append(chunk.data(), static_cast<std::size_t>(count));
    result.bytes_received = bytes.size();

    const auto newline = bytes.find('\n');
    if (newline != std::string::npos) {
      if (newline > maximum_bytes) {
        result.status = CommandServerStatus::RequestTooLarge;
        result.reason = "command_server_request_too_large";
        return result;
      }
      if (newline + 1U != bytes.size()) {
        result.status = CommandServerStatus::InvalidFrame;
        result.reason = "command_server_multiple_requests_rejected";
        return result;
      }

      result.frame = bytes.substr(0U, newline);
      if (result.frame.empty()) {
        result.status = CommandServerStatus::EmptyFrame;
        result.reason = "command_server_empty_request";
        return result;
      }
      if (result.frame.find('\0') != std::string::npos) {
        result.status = CommandServerStatus::InvalidFrame;
        result.reason = "command_server_nul_byte_rejected";
        return result;
      }
      if (!valid_utf8(result.frame)) {
        result.status = CommandServerStatus::InvalidFrame;
        result.reason = "command_server_invalid_utf8";
        return result;
      }

      result.status = CommandServerStatus::DryRunAcknowledged;
      result.reason = "command_server_frame_received";
      return result;
    }

    if (bytes.size() > maximum_bytes) {
      result.status = CommandServerStatus::RequestTooLarge;
      result.reason = "command_server_request_too_large";
      return result;
    }
  }
}

struct WriteResult
{
  CommandServerStatus status{CommandServerStatus::WriteError};
  std::string reason{"command_server_response_write_failed"};
  std::size_t bytes_sent{0U};
  bool succeeded{false};
};

[[nodiscard]] WriteResult write_response(
  const int descriptor,
  const Json & response,
  const std::size_t maximum_bytes,
  const std::int64_t timeout_ms) noexcept
{
  WriteResult result;
  std::string encoded;

  try {
    encoded = response.dump();
  } catch (...) {
    result.reason = "command_server_response_serialization_failed";
    return result;
  }

  if (encoded.size() >= maximum_bytes) {
    result.status = CommandServerStatus::ResponseTooLarge;
    result.reason = "command_server_response_too_large";
    return result;
  }
  encoded.push_back('\n');

  const auto deadline = SteadyClock::now() +
    std::chrono::milliseconds(timeout_ms);

  while (result.bytes_sent < encoded.size()) {
    const int readiness = poll_until(descriptor, POLLOUT, deadline);
    if (readiness == 0) {
      result.status = CommandServerStatus::WriteTimeout;
      result.reason = "command_server_client_write_timeout";
      return result;
    }
    if (readiness < 0) {
      result.reason = "command_server_client_write_failed";
      return result;
    }

    int flags = 0;
#ifdef MSG_NOSIGNAL
    flags |= MSG_NOSIGNAL;
#endif
    const ssize_t count = ::send(
      descriptor,
      encoded.data() + result.bytes_sent,
      encoded.size() - result.bytes_sent,
      flags);

    if (count < 0) {
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      result.reason = "command_server_client_write_failed";
      return result;
    }
    if (count == 0) {
      result.reason = "command_server_client_write_closed";
      return result;
    }
    result.bytes_sent += static_cast<std::size_t>(count);
  }

  result.succeeded = true;
  return result;
}

[[nodiscard]] Json response_details(
  const bool dispatch_attempted = false,
  const std::size_t ros_publications = 0U)
{
  return Json{
    {"dispatch_attempted", dispatch_attempted},
    {"ros_publications", ros_publications},
  };
}

[[nodiscard]] Json error_response(
  const std::string & error_code,
  const std::string & message,
  const std::int64_t observed_at_unix_ms,
  const std::optional<std::string> & command_id = std::nullopt)
{
  Json response{
    {"protocol_version", COMMAND_PROTOCOL_VERSION},
    {"message_type", "command_error"},
    {"error_code", error_code},
    {"message", message},
    {"retryable", false},
    {"observed_at_unix_ms", observed_at_unix_ms},
    {"details", response_details()},
  };

  if (command_id.has_value()) {
    response["command_id"] = command_id.value();
  }
  return response;
}

[[nodiscard]] Json acknowledgement_response(
  const ValidatedCommand & command,
  const std::string & bridge_instance_id,
  const std::string & execution_mode,
  const CommandDispatchResult & dispatch,
  const std::int64_t received_at_unix_ms,
  const bool duplicate = false)
{
  return Json{
    {"protocol_version", COMMAND_PROTOCOL_VERSION},
    {"message_type", "command_acknowledgement"},
    {"command_id", command.command_id},
    {"accepted", dispatch.accepted},
    {"state", dispatch.state},
    {"reason", dispatch.reason},
    {"bridge_instance_id", bridge_instance_id},
    {"execution_mode", execution_mode},
    {"duplicate", duplicate},
    {"received_at_unix_ms", received_at_unix_ms},
    {"details", response_details(
        dispatch.dispatch_attempted,
        dispatch.ros_publications)},
  };
}

}  // namespace

CommandServerConfig::CommandServerConfig()
{
  allowed_peer_uids = {
    0U,
    static_cast<std::uint32_t>(::geteuid()),
  };
}

const char * to_string(const CommandServerStatus status) noexcept
{
  switch (status) {
    case CommandServerStatus::Disabled:
      return "disabled";
    case CommandServerStatus::Started:
      return "started";
    case CommandServerStatus::Stopped:
      return "stopped";
    case CommandServerStatus::NotRunning:
      return "not_running";
    case CommandServerStatus::InvalidConfig:
      return "invalid_config";
    case CommandServerStatus::UnsafePath:
      return "unsafe_path";
    case CommandServerStatus::SocketError:
      return "socket_error";
    case CommandServerStatus::AcceptTimeout:
      return "accept_timeout";
    case CommandServerStatus::AcceptError:
      return "accept_error";
    case CommandServerStatus::PeerCredentialsUnavailable:
      return "peer_credentials_unavailable";
    case CommandServerStatus::UnauthorizedPeer:
      return "unauthorized_peer";
    case CommandServerStatus::ReadTimeout:
      return "read_timeout";
    case CommandServerStatus::ReadError:
      return "read_error";
    case CommandServerStatus::EofBeforeNewline:
      return "eof_before_newline";
    case CommandServerStatus::EmptyFrame:
      return "empty_frame";
    case CommandServerStatus::RequestTooLarge:
      return "request_too_large";
    case CommandServerStatus::InvalidFrame:
      return "invalid_frame";
    case CommandServerStatus::ProtocolError:
      return "protocol_error";
    case CommandServerStatus::DryRunAcknowledged:
      return "dry_run_acknowledged";
    case CommandServerStatus::CommandAcknowledged:
      return "command_acknowledged";
    case CommandServerStatus::CommandRejected:
      return "command_rejected";
    case CommandServerStatus::ResponseTooLarge:
      return "response_too_large";
    case CommandServerStatus::WriteTimeout:
      return "write_timeout";
    case CommandServerStatus::WriteError:
      return "write_error";
  }
  return "unknown";
}

class CommandServer::Impl
{
private:
  struct CachedAcknowledgement
  {
    CommandServerStatus status{
      CommandServerStatus::CommandRejected};

    CommandDispatchResult dispatch;
    std::int64_t received_at_unix_ms{0};
  };

public:
  Impl(
    CommandServerConfig config,
    Clock clock,
    Dispatcher dispatcher)
  : config_(std::move(config)),
    clock_(std::move(clock)),
    dispatcher_(std::move(dispatcher))
  {
    if (!clock_) {
      clock_ = default_now_unix_ms;
    }
  }

  ~Impl()
  {
    (void)stop();
  }

  [[nodiscard]] CommandServerResult start()
  {
    if (!config_.enabled) {
      return make_result(
        CommandServerStatus::Disabled,
        "command_server_disabled");
    }
    if (listener_.valid()) {
      return make_result(
        CommandServerStatus::Started,
        "command_server_already_started");
    }

    if (
      config_.execution_mode != "dry_run" &&
      config_.execution_mode != "live")
    {
      return make_result(
        CommandServerStatus::InvalidConfig,
        "command_server_execution_mode_invalid");
    }

    if (
      config_.execution_mode == "live" &&
      !dispatcher_)
    {
      return make_result(
        CommandServerStatus::InvalidConfig,
        "command_server_live_dispatcher_missing");
    }

    const auto validation = validate_config();
    if (validation.has_value()) {
      return make_result(
        CommandServerStatus::InvalidConfig,
        validation.value());
    }

    const std::filesystem::path path{config_.socket_path};
    if (!safe_parent_directory(path)) {
      return make_result(
        CommandServerStatus::UnsafePath,
        "command_server_parent_directory_unsafe");
    }

    struct stat stale_identity {};
    const auto existing = inspect_existing_path(
      config_.socket_path,
      stale_identity);

    if (existing == ExistingPathState::Unsafe) {
      return make_result(
        CommandServerStatus::UnsafePath,
        "command_server_existing_path_unsafe_or_live");
    }
    if (existing == ExistingPathState::StaleSocket) {
      struct stat before_unlink {};
      if (::lstat(config_.socket_path.c_str(), &before_unlink) != 0 ||
        !same_identity(stale_identity, before_unlink) ||
        ::unlink(config_.socket_path.c_str()) != 0)
      {
        return make_result(
          CommandServerStatus::UnsafePath,
          "command_server_stale_socket_cleanup_failed");
      }
    }

    auto listener = create_unix_socket();
    if (!listener.valid() || !set_nonblocking(listener.get())) {
      return make_result(
        CommandServerStatus::SocketError,
        "command_server_socket_create_failed");
    }

    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::memcpy(
      address.sun_path,
      config_.socket_path.c_str(),
      config_.socket_path.size() + 1U);

    {
      const UmaskGuard restrictive_umask{0077};
      if (::bind(
          listener.get(),
          reinterpret_cast<const sockaddr *>(&address),
          sizeof(address)) != 0)
      {
        return make_result(
          CommandServerStatus::SocketError,
          "command_server_bind_failed");
      }
    }

    struct stat created {};
    if (::lstat(config_.socket_path.c_str(), &created) != 0 ||
      !S_ISSOCK(created.st_mode))
    {
      return make_result(
        CommandServerStatus::SocketError,
        "command_server_socket_identity_failed");
    }

    if (::chmod(
        config_.socket_path.c_str(),
        static_cast<mode_t>(config_.socket_mode)) != 0)
    {
      remove_if_identity(config_.socket_path, created);
      return make_result(
        CommandServerStatus::SocketError,
        "command_server_chmod_failed");
    }

    if (
      config_.socket_gid.has_value() &&
      ::chown(
        config_.socket_path.c_str(),
        static_cast<uid_t>(-1),
        static_cast<gid_t>(
          config_.socket_gid.value())) != 0)
    {
      remove_if_identity(config_.socket_path, created);
      return make_result(
        CommandServerStatus::SocketError,
        "command_server_chown_failed");
    }

    if (::listen(listener.get(), 4) != 0) {
      remove_if_identity(config_.socket_path, created);
      return make_result(
        CommandServerStatus::SocketError,
        "command_server_listen_failed");
    }

    socket_device_ = created.st_dev;
    socket_inode_ = created.st_ino;
    listener_ = std::move(listener);
    return make_result(
      CommandServerStatus::Started,
      "command_server_started");
  }

  [[nodiscard]] CommandServerResult serve_one()
  {
    if (!listener_.valid()) {
      return make_result(
        CommandServerStatus::NotRunning,
        "command_server_not_running");
    }

    const auto accept_deadline = SteadyClock::now() +
      std::chrono::milliseconds(config_.accept_timeout_ms);
    const int ready = poll_until(listener_.get(), POLLIN, accept_deadline);
    if (ready == 0) {
      return make_result(
        CommandServerStatus::AcceptTimeout,
        "command_server_accept_timeout");
    }
    if (ready < 0) {
      return make_result(
        CommandServerStatus::AcceptError,
        "command_server_accept_failed");
    }

    int accepted = -1;
    while (accepted < 0) {
#ifdef SOCK_CLOEXEC
      accepted = ::accept4(listener_.get(), nullptr, nullptr, SOCK_CLOEXEC);
#else
      accepted = ::accept(listener_.get(), nullptr, nullptr);
#endif
      if (accepted < 0 && errno == EINTR) {
        continue;
      }
      break;
    }

    FileDescriptor client{accepted};
    if (!client.valid()) {
      return make_result(
        CommandServerStatus::AcceptError,
        "command_server_accept_failed");
    }
    if (!set_close_on_exec(client.get()) ||
      !set_nonblocking(client.get()))
    {
      return make_result(
        CommandServerStatus::AcceptError,
        "command_server_client_descriptor_setup_failed");
    }

    auto result = make_result(
      CommandServerStatus::PeerCredentialsUnavailable,
      "command_server_peer_credentials_unavailable");

    const auto credentials = peer_credentials(client.get());
    if (!credentials.has_value()) {
      return result;
    }

    result.peer_pid = credentials->pid;
    result.peer_uid = credentials->uid;
    result.peer_gid = credentials->gid;

    if (!peer_allowed(config_.allowed_peer_uids, credentials->uid)) {
      result.status = CommandServerStatus::UnauthorizedPeer;
      result.reason = "command_server_peer_uid_not_allowed";
      return result;
    }

    const auto frame = read_frame(
      client.get(),
      config_.max_request_bytes,
      config_.client_read_timeout_ms);

    result.status = frame.status;
    result.reason = frame.reason;
    result.bytes_received = frame.bytes_received;
    result.request_received = frame.succeeded();

    const std::int64_t now = clock_();
    if (!frame.succeeded()) {
      const auto response = error_response(
        to_string(frame.status),
        frame.reason,
        now);
      return send_and_finish(client.get(), response, std::move(result));
    }

    const auto parsed = parse_command_request(frame.frame, now);
    if (!parsed.succeeded()) {
      result.status = CommandServerStatus::ProtocolError;
      result.reason = parsed.error.has_value() ?
        std::string(to_string(parsed.error->code)) :
        "invalid_protocol_request";

      const auto response = error_response(
        result.reason,
        "request failed protocol validation",
        now);
      return send_and_finish(client.get(), response, std::move(result));
    }

    result.command_id = parsed.command->command_id;

    const auto cached =
      completed_commands_.find(
      parsed.command->command_id);

    if (cached != completed_commands_.end()) {
      CommandDispatchResult replay_dispatch =
        cached->second.dispatch;

      result.status = cached->second.status;
      result.reason = replay_dispatch.reason;
      result.dispatch_attempted = false;
      result.ros_publications = 0U;
      result.duplicate = true;

      replay_dispatch.dispatch_attempted = false;
      replay_dispatch.ros_publications = 0U;

      const auto response = acknowledgement_response(
        parsed.command.value(),
        config_.bridge_instance_id,
        config_.execution_mode,
        replay_dispatch,
        cached->second.received_at_unix_ms,
        true);

      return send_and_finish(
        client.get(),
        response,
        std::move(result));
    }

    CommandDispatchResult dispatch;

    if (config_.execution_mode == "dry_run") {
      dispatch.accepted = true;
      dispatch.state = "dry_run";
      dispatch.reason = "bridge_command_dry_run_accepted";
      dispatch.dispatch_attempted = false;
      dispatch.ros_publications = 0U;

      result.status =
        CommandServerStatus::DryRunAcknowledged;
    } else {
      try {
        dispatch = dispatcher_(
          parsed.command.value());
      } catch (const std::exception &) {
        dispatch.accepted = false;
        dispatch.state = "rejected";
        dispatch.reason =
          "bridge_command_live_dispatch_internal_error";
        dispatch.dispatch_attempted = true;
        dispatch.ros_publications = 0U;
      } catch (...) {
        dispatch.accepted = false;
        dispatch.state = "rejected";
        dispatch.reason =
          "bridge_command_live_dispatch_internal_error";
        dispatch.dispatch_attempted = true;
        dispatch.ros_publications = 0U;
      }

      if (dispatch.accepted) {
        dispatch.state = "accepted";
        result.status =
          CommandServerStatus::CommandAcknowledged;
      } else {
        dispatch.state = "rejected";
        result.status =
          CommandServerStatus::CommandRejected;
      }

      if (dispatch.reason.empty()) {
        dispatch.reason = dispatch.accepted ?
          "bridge_command_accepted" :
          "bridge_command_rejected";
      }
    }

    result.reason = dispatch.reason;
    result.dispatch_attempted =
      dispatch.dispatch_attempted;
    result.ros_publications =
      dispatch.ros_publications;

    cache_acknowledgement(
      parsed.command->command_id,
      result.status,
      dispatch,
      now);

    const auto response = acknowledgement_response(
      parsed.command.value(),
      config_.bridge_instance_id,
      config_.execution_mode,
      dispatch,
      now);

    return send_and_finish(
      client.get(),
      response,
      std::move(result));
  }

  [[nodiscard]] CommandServerResult stop() noexcept
  {
    if (!listener_.valid()) {
      return make_result(
        CommandServerStatus::Stopped,
        "command_server_already_stopped");
    }

    listener_.reset();
    struct stat current {};
    if (::lstat(config_.socket_path.c_str(), &current) == 0 &&
      S_ISSOCK(current.st_mode) &&
      current.st_dev == socket_device_ &&
      current.st_ino == socket_inode_)
    {
      (void)::unlink(config_.socket_path.c_str());
    }

    socket_device_ = 0;
    socket_inode_ = 0;
    return make_result(
      CommandServerStatus::Stopped,
      "command_server_stopped");
  }

  [[nodiscard]] bool running() const noexcept
  {
    return listener_.valid();
  }

private:
  [[nodiscard]] std::optional<std::string> validate_config() const
  {
    if (config_.socket_path.empty()) {
      return "command_server_socket_path_empty";
    }
    if (config_.socket_path.find('\0') != std::string::npos) {
      return "command_server_socket_path_contains_nul";
    }
    if (config_.socket_path.front() != '/') {
      return "command_server_socket_path_must_be_absolute";
    }
    if (config_.socket_path.size() >= sizeof(sockaddr_un::sun_path)) {
      return "command_server_socket_path_too_long";
    }
    if (config_.max_request_bytes == 0U ||
      config_.max_response_bytes == 0U)
    {
      return "command_server_message_limit_must_be_positive";
    }
    if (config_.accept_timeout_ms <= 0 ||
      config_.client_read_timeout_ms <= 0 ||
      config_.client_write_timeout_ms <= 0 ||
      config_.accept_timeout_ms > std::numeric_limits<int>::max() ||
      config_.client_read_timeout_ms > std::numeric_limits<int>::max() ||
      config_.client_write_timeout_ms > std::numeric_limits<int>::max())
    {
      return "command_server_timeout_invalid";
    }
    if ((config_.socket_mode & ~0777U) != 0U ||
      (config_.socket_mode & 0111U) != 0U ||
      (config_.socket_mode & 0002U) != 0U)
    {
      return "command_server_socket_mode_unsafe";
    }
    if (config_.allowed_peer_uids.empty()) {
      return "command_server_allowed_peer_uids_empty";
    }
    if (!is_safe_identifier(config_.bridge_instance_id)) {
      return "command_server_bridge_instance_id_invalid";
    }
    if (config_.command_id_cache_capacity == 0U) {
      return "command_server_command_cache_capacity_invalid";
    }
    return std::nullopt;
  }

  void cache_acknowledgement(
    const std::string & command_id,
    const CommandServerStatus status,
    const CommandDispatchResult & dispatch,
    const std::int64_t received_at_unix_ms)
  {
    if (
      completed_commands_.find(command_id) !=
      completed_commands_.end())
    {
      return;
    }

    while (
      completed_command_order_.size() >=
      config_.command_id_cache_capacity)
    {
      completed_commands_.erase(
        completed_command_order_.front());

      completed_command_order_.pop_front();
    }

    completed_command_order_.push_back(command_id);

    completed_commands_.emplace(
      command_id,
      CachedAcknowledgement{
        status,
        dispatch,
        received_at_unix_ms});
  }

  [[nodiscard]] CommandServerResult send_and_finish(
    const int descriptor,
    const Json & response,
    CommandServerResult result) const noexcept
  {
    const auto written = write_response(
      descriptor,
      response,
      config_.max_response_bytes,
      config_.client_write_timeout_ms);

    result.bytes_sent = written.bytes_sent;
    result.response_sent = written.succeeded;
    if (!written.succeeded) {
      result.status = written.status;
      result.reason = written.reason;
    }
    return result;
  }

  CommandServerConfig config_;
  Clock clock_;
  Dispatcher dispatcher_;

  std::deque<std::string> completed_command_order_;

  std::unordered_map<
    std::string,
    CachedAcknowledgement> completed_commands_;

  FileDescriptor listener_;
  dev_t socket_device_{0};
  ino_t socket_inode_{0};
};

CommandServer::CommandServer(
  CommandServerConfig config,
  Clock clock,
  Dispatcher dispatcher)
: impl_(std::make_unique<Impl>(
    std::move(config),
    std::move(clock),
    std::move(dispatcher)))
{
}

CommandServer::~CommandServer() = default;

CommandServer::CommandServer(CommandServer &&) noexcept = default;

CommandServer & CommandServer::operator=(CommandServer &&) noexcept = default;

CommandServerResult CommandServer::start()
{
  return impl_->start();
}

CommandServerResult CommandServer::serve_one()
{
  return impl_->serve_one();
}

CommandServerResult CommandServer::stop() noexcept
{
  return impl_->stop();
}

bool CommandServer::running() const noexcept
{
  return impl_->running();
}

}  // namespace savo_bridge
