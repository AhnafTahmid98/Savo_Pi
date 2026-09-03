// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include <gtest/gtest.h>

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
#include <iterator>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "savo_bridge/command_server.hpp"

namespace
{

using Json = nlohmann::json;
using savo_bridge::CommandServer;
using savo_bridge::CommandServerConfig;
using savo_bridge::CommandServerResult;
using savo_bridge::CommandServerStatus;

inline constexpr std::int64_t NOW_UNIX_MS = 1800000000500;

inline constexpr const char * CANONICAL_STOP =
  R"json({
  "protocol_version": 1,
  "message_type": "command_request",
  "command_id": "server-stop-1",
  "command_type": "stop",
  "source": "savomind",
  "origin_agent": "safety_agent",
  "priority": "emergency",
  "issued_at_unix_ms": 1800000000000,
  "expires_at_unix_ms": 1800000001000,
  "payload": {"reason": "server_test", "scope": "all_movement"}
})json";

inline constexpr const char * CANONICAL_TELEOP =
  R"json({
  "command_id": "server-teleop-1",
  "command_type": "teleop_nudge",
  "source": "savomind",
  "priority": "normal",
  "issued_at_unix_ms": 1800000000000,
  "expires_at_unix_ms": 1800000001000,
  "payload": {
    "direction": "forward",
    "linear_x_mps": 0.12,
    "duration_ms": 600
  }
})json";

inline constexpr const char * CANONICAL_NAVIGATION =
  R"json({
  "command_id": "server-navigation-1",
  "command_type": "navigate_to_location",
  "source": "savomind",
  "priority": "normal",
  "issued_at_unix_ms": 1800000000000,
  "expires_at_unix_ms": 1800000001000,
  "payload": {"location_id": "A201", "map_id": "campus-main"}
})json";

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    std::array<char, 64U> pattern{};
    const std::string base{"/tmp/savo-bridge-command-server-XXXXXX"};
    std::copy(base.begin(), base.end(), pattern.begin());
    char * created = ::mkdtemp(pattern.data());
    if (created != nullptr) {
      path_ = created;
    }
  }

  ~TemporaryDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  TemporaryDirectory(const TemporaryDirectory &) = delete;
  TemporaryDirectory & operator=(const TemporaryDirectory &) = delete;

  [[nodiscard]] const std::filesystem::path & path() const noexcept
  {
    return path_;
  }

private:
  std::filesystem::path path_;
};

class FileDescriptor
{
public:
  explicit FileDescriptor(const int descriptor = -1) noexcept
  : descriptor_(descriptor)
  {
  }

  ~FileDescriptor()
  {
    if (descriptor_ >= 0) {
      (void)::close(descriptor_);
    }
  }

  FileDescriptor(const FileDescriptor &) = delete;
  FileDescriptor & operator=(const FileDescriptor &) = delete;

  FileDescriptor(FileDescriptor && other) noexcept
  : descriptor_(other.descriptor_)
  {
    other.descriptor_ = -1;
  }

  FileDescriptor & operator=(FileDescriptor && other) noexcept
  {
    if (this != &other) {
      if (descriptor_ >= 0) {
        (void)::close(descriptor_);
      }
      descriptor_ = other.descriptor_;
      other.descriptor_ = -1;
    }
    return *this;
  }

  [[nodiscard]] int get() const noexcept
  {
    return descriptor_;
  }

private:
  int descriptor_;
};

[[nodiscard]] CommandServerConfig config_for(
  const std::filesystem::path & path)
{
  CommandServerConfig config;
  config.enabled = true;
  config.socket_path = path.string();
  config.accept_timeout_ms = 100;
  config.client_read_timeout_ms = 100;
  config.client_write_timeout_ms = 100;
  config.bridge_instance_id = "test-bridge-1";
  return config;
}

struct ExchangeResult
{
  CommandServerResult server;
  std::string response;
};

[[nodiscard]] FileDescriptor connect_client(
  const std::filesystem::path & path)
{
  FileDescriptor client{::socket(AF_UNIX, SOCK_STREAM, 0)};
  if (client.get() < 0) {
    return client;
  }

  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  const std::string path_text = path.string();
  std::copy(path_text.begin(), path_text.end(), address.sun_path);
  address.sun_path[path_text.size()] = '\0';

  if (::connect(
      client.get(),
      reinterpret_cast<const sockaddr *>(&address),
      sizeof(address)) != 0)
  {
    return FileDescriptor{};
  }
  return client;
}

[[nodiscard]] std::string read_response(const int descriptor)
{
  std::string response;
  std::array<char, 4096U> buffer{};

  while (response.find('\n') == std::string::npos) {
    const ssize_t count = ::recv(
      descriptor,
      buffer.data(),
      buffer.size(),
      0);
    if (count <= 0) {
      break;
    }
    response.append(buffer.data(), static_cast<std::size_t>(count));
  }
  return response;
}

[[nodiscard]] ExchangeResult exchange(
  CommandServer & server,
  const std::filesystem::path & path,
  const std::string & request,
  const bool append_newline = true,
  const bool shutdown_write = false)
{
  auto future = std::async(
    std::launch::async,
    [&server]() {return server.serve_one();});

  auto client = connect_client(path);
  EXPECT_GE(client.get(), 0);

  std::string frame = request;
  if (append_newline) {
    frame.push_back('\n');
  }
  if (!frame.empty()) {
    EXPECT_EQ(
      ::send(client.get(), frame.data(), frame.size(), 0),
      static_cast<ssize_t>(frame.size()));
  }
  if (shutdown_write) {
    EXPECT_EQ(::shutdown(client.get(), SHUT_WR), 0);
  }

  const std::string response = read_response(client.get());
  return ExchangeResult{future.get(), response};
}

[[nodiscard]] Json response_document(const std::string & response)
{
  EXPECT_FALSE(response.empty());
  EXPECT_EQ(response.back(), '\n');
  EXPECT_EQ(response.find('\n'), response.size() - 1U);
  return Json::parse(response.substr(0U, response.size() - 1U));
}

[[nodiscard]] std::string compact_json(const char * document)
{
  return Json::parse(document).dump();
}

TEST(SavoBridgeCommandServer, DefaultsAreSafeAndDisabled)
{
  const CommandServerConfig config;
  EXPECT_FALSE(config.enabled);
  EXPECT_EQ(config.socket_path, "/run/savo_bridge/command.sock");
  EXPECT_EQ(config.max_request_bytes, 65536U);
  EXPECT_EQ(config.max_response_bytes, 65536U);
  EXPECT_EQ(config.accept_timeout_ms, 250);
  EXPECT_EQ(config.client_read_timeout_ms, 1000);
  EXPECT_EQ(config.client_write_timeout_ms, 1000);
  EXPECT_EQ(config.socket_mode, 0660U);
  EXPECT_NE(
    std::find(
      config.allowed_peer_uids.begin(),
      config.allowed_peer_uids.end(),
      static_cast<std::uint32_t>(::geteuid())),
    config.allowed_peer_uids.end());
}

TEST(SavoBridgeCommandServer, DisabledServerCreatesNoFilesystemEntry)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "disabled.sock";
  CommandServerConfig config;
  config.socket_path = path.string();
  CommandServer server{config};

  const auto result = server.start();
  EXPECT_EQ(result.status, CommandServerStatus::Disabled);
  EXPECT_FALSE(server.running());
  EXPECT_FALSE(std::filesystem::exists(path));
}

TEST(SavoBridgeCommandServer, DisabledServerDoesNotInspectInvalidParent)
{
  CommandServerConfig config;
  config.socket_path = "/path/that/does/not/exist/command.sock";
  CommandServer server{config};
  EXPECT_EQ(server.start().status, CommandServerStatus::Disabled);
}

TEST(SavoBridgeCommandServer, RelativeAndOverlongPathsAreRejected)
{
  auto relative = config_for("relative.sock");
  CommandServer relative_server{relative};
  EXPECT_EQ(relative_server.start().status, CommandServerStatus::InvalidConfig);

  auto overlong = config_for(
    "/tmp/" + std::string(sizeof(sockaddr_un::sun_path), 'x'));
  CommandServer overlong_server{overlong};
  EXPECT_EQ(overlong_server.start().status, CommandServerStatus::InvalidConfig);
}

TEST(SavoBridgeCommandServer, ExistingRegularFileIsRejectedAndPreserved)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "regular.sock";
  std::ofstream(path) << "preserve";
  CommandServer server{config_for(path)};

  EXPECT_EQ(server.start().status, CommandServerStatus::UnsafePath);
  EXPECT_TRUE(std::filesystem::is_regular_file(path));
}

TEST(SavoBridgeCommandServer, ExistingDirectoryIsRejectedAndPreserved)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "directory.sock";
  ASSERT_TRUE(std::filesystem::create_directory(path));
  CommandServer server{config_for(path)};

  EXPECT_EQ(server.start().status, CommandServerStatus::UnsafePath);
  EXPECT_TRUE(std::filesystem::is_directory(path));
}

TEST(SavoBridgeCommandServer, SymlinkPathIsRejectedAndPreserved)
{
  TemporaryDirectory temporary;
  const auto target = temporary.path() / "target";
  const auto path = temporary.path() / "link.sock";
  std::ofstream(target) << "target";
  std::filesystem::create_symlink(target, path);
  ASSERT_TRUE(std::filesystem::is_symlink(path));
  CommandServer server{config_for(path)};

  EXPECT_EQ(server.start().status, CommandServerStatus::UnsafePath);
  EXPECT_TRUE(std::filesystem::is_symlink(path));
}

TEST(SavoBridgeCommandServer, SymlinkParentIsRejected)
{
  TemporaryDirectory temporary;
  const auto real = temporary.path() / "real";
  const auto link = temporary.path() / "link";
  ASSERT_TRUE(std::filesystem::create_directory(real));
  std::filesystem::create_symlink(real, link);
  ASSERT_TRUE(std::filesystem::is_symlink(link));
  CommandServer server{config_for(link / "command.sock")};
  EXPECT_EQ(server.start().status, CommandServerStatus::UnsafePath);
}

TEST(SavoBridgeCommandServer, MissingPrivateParentIsCreatedIdempotently)
{
  TemporaryDirectory temporary;
  const auto parent = temporary.path() / "runtime";
  const auto path = parent / "command.sock";

  CommandServer first{config_for(path)};
  ASSERT_EQ(first.start().status, CommandServerStatus::Started);
  ASSERT_EQ(first.stop().status, CommandServerStatus::Stopped);

  struct stat metadata {};
  ASSERT_EQ(::lstat(parent.c_str(), &metadata), 0);
  EXPECT_TRUE(S_ISDIR(metadata.st_mode));
  EXPECT_EQ(metadata.st_uid, ::geteuid());
  EXPECT_EQ(metadata.st_mode & 0777U, 0700U);

  CommandServer second{config_for(path)};
  ASSERT_EQ(second.start().status, CommandServerStatus::Started);
  EXPECT_EQ(second.stop().status, CommandServerStatus::Stopped);
}

TEST(SavoBridgeCommandServer, MissingSharedParentUsesConfiguredGroup)
{
  TemporaryDirectory temporary;
  const auto parent = temporary.path() / "shared-runtime";
  const auto path = parent / "command.sock";
  auto config = config_for(path);
  config.socket_gid = static_cast<std::uint32_t>(::getegid());

  CommandServer server{config};
  ASSERT_EQ(server.start().status, CommandServerStatus::Started);

  struct stat metadata {};
  ASSERT_EQ(::lstat(parent.c_str(), &metadata), 0);
  EXPECT_EQ(metadata.st_uid, ::geteuid());
  EXPECT_EQ(metadata.st_gid, ::getegid());
  EXPECT_EQ(metadata.st_mode & 0777U, 0770U);
  EXPECT_EQ(server.stop().status, CommandServerStatus::Stopped);
}

TEST(SavoBridgeCommandServer, WorldWritableParentIsRejected)
{
  TemporaryDirectory temporary;
  const auto parent = temporary.path() / "world-writable";
  ASSERT_TRUE(std::filesystem::create_directory(parent));
  ASSERT_EQ(::chmod(parent.c_str(), 0777), 0);

  CommandServer server{config_for(parent / "command.sock")};
  EXPECT_EQ(server.start().status, CommandServerStatus::UnsafePath);
  EXPECT_FALSE(std::filesystem::exists(parent / "command.sock"));
}

TEST(SavoBridgeCommandServer, ConfiguredSharedParentMustBeGroupTraversable)
{
  TemporaryDirectory temporary;
  const auto parent = temporary.path() / "private-parent";
  ASSERT_TRUE(std::filesystem::create_directory(parent));
  ASSERT_EQ(::chmod(parent.c_str(), 0700), 0);
  auto config = config_for(parent / "command.sock");
  config.socket_gid = static_cast<std::uint32_t>(::getegid());

  CommandServer server{config};
  EXPECT_EQ(server.start().status, CommandServerStatus::UnsafePath);
  EXPECT_FALSE(std::filesystem::exists(parent / "command.sock"));
}

TEST(SavoBridgeCommandServer, NonWritableParentIsRejected)
{
  TemporaryDirectory temporary;
  const auto parent = temporary.path() / "read-only";
  ASSERT_TRUE(std::filesystem::create_directory(parent));
  ASSERT_EQ(::chmod(parent.c_str(), 0500), 0);

  CommandServer server{config_for(parent / "command.sock")};
  EXPECT_EQ(server.start().status, CommandServerStatus::UnsafePath);
  EXPECT_FALSE(std::filesystem::exists(parent / "command.sock"));
}

TEST(SavoBridgeCommandServer, UnsafeSocketModesAreRejected)
{
  TemporaryDirectory temporary;
  for (const std::uint32_t mode : {0662U, 0770U, 0670U}) {
    auto config = config_for(temporary.path() / std::to_string(mode));
    config.socket_mode = mode;
    CommandServer server{config};
    EXPECT_EQ(server.start().status, CommandServerStatus::InvalidConfig);
  }
}

TEST(SavoBridgeCommandServer, CreatesSocketWithExpectedPermissionsAndCleansUp)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "command.sock";
  CommandServer server{config_for(path)};
  ASSERT_EQ(server.start().status, CommandServerStatus::Started);
  ASSERT_TRUE(server.running());

  struct stat metadata {};
  ASSERT_EQ(::lstat(path.c_str(), &metadata), 0);
  EXPECT_TRUE(S_ISSOCK(metadata.st_mode));
  EXPECT_EQ(metadata.st_mode & 0777U, 0660U);

  EXPECT_EQ(server.stop().status, CommandServerStatus::Stopped);
  EXPECT_FALSE(std::filesystem::exists(path));
}

TEST(SavoBridgeCommandServer, CleanupDoesNotRemoveReplacedPath)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "command.sock";
  CommandServer server{config_for(path)};
  ASSERT_EQ(server.start().status, CommandServerStatus::Started);
  ASSERT_EQ(::unlink(path.c_str()), 0);
  std::ofstream(path) << "replacement";

  EXPECT_EQ(server.stop().status, CommandServerStatus::Stopped);
  EXPECT_TRUE(std::filesystem::is_regular_file(path));
}

TEST(SavoBridgeCommandServer, UnauthorizedPeerIsRejectedBeforeRead)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "command.sock";
  auto config = config_for(path);
  config.allowed_peer_uids = {
    static_cast<std::uint32_t>(::geteuid()) + 100000U,
  };
  CommandServer server{config};
  ASSERT_EQ(server.start().status, CommandServerStatus::Started);

  const auto outcome = exchange(server, path, "", false, false);
  EXPECT_EQ(outcome.server.status, CommandServerStatus::UnauthorizedPeer);
  EXPECT_FALSE(outcome.server.request_received);
  EXPECT_FALSE(outcome.server.response_sent);
}

TEST(SavoBridgeCommandServer, AuthorizedPeerCredentialsAreCaptured)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "command.sock";
  CommandServer server{config_for(path), []() {return NOW_UNIX_MS;}};
  ASSERT_EQ(server.start().status, CommandServerStatus::Started);

  const auto outcome = exchange(server, path, compact_json(CANONICAL_STOP));
  ASSERT_TRUE(outcome.server.peer_uid.has_value());
  EXPECT_EQ(outcome.server.peer_uid.value(), ::geteuid());
  EXPECT_TRUE(outcome.server.peer_pid.has_value());
  EXPECT_TRUE(outcome.server.peer_gid.has_value());
}

class CanonicalRequestTest : public testing::TestWithParam<std::pair<
      std::string, std::string>>
{
};

TEST_P(CanonicalRequestTest, ReceivesDryRunAcknowledgement)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "command.sock";
  CommandServer server{config_for(path), []() {return NOW_UNIX_MS;}};
  ASSERT_EQ(server.start().status, CommandServerStatus::Started);

  const auto outcome = exchange(server, path, GetParam().first);
  ASSERT_EQ(outcome.server.status, CommandServerStatus::DryRunAcknowledged)
    << outcome.server.reason << '\n' << outcome.response;
  ASSERT_TRUE(outcome.server.response_sent);
  EXPECT_FALSE(outcome.server.dispatch_attempted);
  EXPECT_EQ(outcome.server.ros_publications, 0U);

  const auto response = response_document(outcome.response);
  EXPECT_EQ(response.at("message_type"), "command_acknowledgement");
  EXPECT_EQ(response.at("command_id"), GetParam().second);
  EXPECT_EQ(response.at("state"), "dry_run");
  EXPECT_EQ(response.at("execution_mode"), "dry_run");
  EXPECT_EQ(response.at("duplicate"), false);
  EXPECT_EQ(response["details"]["dispatch_attempted"], false);
  EXPECT_EQ(response["details"]["ros_publications"], 0);
}

INSTANTIATE_TEST_SUITE_P(
  CanonicalCommands,
  CanonicalRequestTest,
  testing::Values(
    std::make_pair(compact_json(CANONICAL_STOP), "server-stop-1"),
    std::make_pair(compact_json(CANONICAL_TELEOP), "server-teleop-1"),
    std::make_pair(
      compact_json(CANONICAL_NAVIGATION),
      "server-navigation-1")));

TEST(SavoBridgeCommandServer, InvalidProtocolReturnsStructuredError)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "command.sock";
  CommandServer server{config_for(path), []() {return NOW_UNIX_MS;}};
  ASSERT_EQ(server.start().status, CommandServerStatus::Started);

  Json invalid = Json::parse(CANONICAL_TELEOP);
  invalid["protocol_version"] = 2;
  const auto outcome = exchange(server, path, invalid.dump());
  EXPECT_EQ(outcome.server.status, CommandServerStatus::ProtocolError);
  const auto response = response_document(outcome.response);
  EXPECT_EQ(response.at("message_type"), "command_error");
  EXPECT_EQ(response.at("error_code"), "unsupported_protocol_version");
  EXPECT_FALSE(response.contains("command_id"));
  EXPECT_EQ(response["details"]["dispatch_attempted"], false);
  EXPECT_EQ(response["details"]["ros_publications"], 0);
}

TEST(SavoBridgeCommandServer, ProtocolErrorDoesNotEchoUntrustedFieldName)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "command.sock";
  CommandServer server{config_for(path), []() {return NOW_UNIX_MS;}};
  ASSERT_EQ(server.start().status, CommandServerStatus::Started);

  Json invalid = Json::parse(CANONICAL_TELEOP);
  invalid["attacker_controlled_secret_name"] = true;
  const auto outcome = exchange(server, path, invalid.dump());
  const auto response = response_document(outcome.response);
  EXPECT_EQ(response.at("error_code"), "unknown_top_level_field");
  EXPECT_EQ(response.at("message"), "request failed protocol validation");
  EXPECT_EQ(
    outcome.response.find("attacker_controlled_secret_name"),
    std::string::npos);
}

TEST(SavoBridgeCommandServer, DuplicateKeyReturnsStructuredError)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "command.sock";
  CommandServer server{config_for(path), []() {return NOW_UNIX_MS;}};
  ASSERT_EQ(server.start().status, CommandServerStatus::Started);
  const std::string request =
    R"json({"command_id":"one","command_id":"two"})json";

  const auto outcome = exchange(server, path, request);
  const auto response = response_document(outcome.response);
  EXPECT_EQ(response.at("error_code"), "duplicate_object_key");
}

TEST(SavoBridgeCommandServer, ExpiredRequestReturnsStructuredError)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "command.sock";
  CommandServer server{config_for(path), []() {return 1800000001000;}};
  ASSERT_EQ(server.start().status, CommandServerStatus::Started);

  const auto outcome = exchange(server, path, compact_json(CANONICAL_STOP));
  const auto response = response_document(outcome.response);
  EXPECT_EQ(response.at("error_code"), "expired_command")
    << outcome.server.reason << '\n' << outcome.response;
}

TEST(SavoBridgeCommandServer, EmptyFrameIsRejected)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "command.sock";
  CommandServer server{config_for(path), []() {return NOW_UNIX_MS;}};
  ASSERT_EQ(server.start().status, CommandServerStatus::Started);
  const auto outcome = exchange(server, path, "");
  EXPECT_EQ(outcome.server.status, CommandServerStatus::EmptyFrame);
}

TEST(SavoBridgeCommandServer, EofBeforeNewlineIsRejected)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "command.sock";
  CommandServer server{config_for(path), []() {return NOW_UNIX_MS;}};
  ASSERT_EQ(server.start().status, CommandServerStatus::Started);
  const auto outcome = exchange(server, path, "{}", false, true);
  EXPECT_EQ(outcome.server.status, CommandServerStatus::EofBeforeNewline);
}

TEST(SavoBridgeCommandServer, NulContainingFrameIsRejected)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "command.sock";
  CommandServer server{config_for(path), []() {return NOW_UNIX_MS;}};
  ASSERT_EQ(server.start().status, CommandServerStatus::Started);
  const std::string request{"{\"x\":\0}", 8U};
  const auto outcome = exchange(server, path, request);
  EXPECT_EQ(outcome.server.status, CommandServerStatus::InvalidFrame);
}

TEST(SavoBridgeCommandServer, InvalidUtf8FrameIsRejected)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "command.sock";
  CommandServer server{config_for(path), []() {return NOW_UNIX_MS;}};
  ASSERT_EQ(server.start().status, CommandServerStatus::Started);
  const std::string request{"{\"x\":\"\xC0\x80\"}"};
  const auto outcome = exchange(server, path, request);
  EXPECT_EQ(outcome.server.status, CommandServerStatus::InvalidFrame);
}

TEST(SavoBridgeCommandServer, OversizedRequestIsRejected)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "command.sock";
  auto config = config_for(path);
  config.max_request_bytes = 32U;
  CommandServer server{config, []() {return NOW_UNIX_MS;}};
  ASSERT_EQ(server.start().status, CommandServerStatus::Started);
  const auto outcome = exchange(server, path, std::string(33U, 'x'));
  EXPECT_EQ(outcome.server.status, CommandServerStatus::RequestTooLarge);
}

TEST(SavoBridgeCommandServer, MultipleCommandsAreRejected)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "command.sock";
  CommandServer server{config_for(path), []() {return NOW_UNIX_MS;}};
  ASSERT_EQ(server.start().status, CommandServerStatus::Started);
  const auto outcome = exchange(
    server,
    path,
    std::string(CANONICAL_STOP) + "\n" + CANONICAL_STOP);
  EXPECT_EQ(outcome.server.status, CommandServerStatus::InvalidFrame);
}

TEST(SavoBridgeCommandServer, ReadTimeoutIsBounded)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "command.sock";
  auto config = config_for(path);
  config.client_read_timeout_ms = 30;
  CommandServer server{config};
  ASSERT_EQ(server.start().status, CommandServerStatus::Started);

  const auto beginning = std::chrono::steady_clock::now();
  const auto outcome = exchange(server, path, "", false, false);
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - beginning).count();
  EXPECT_EQ(outcome.server.status, CommandServerStatus::ReadTimeout);
  EXPECT_LT(elapsed, 500);
}

TEST(SavoBridgeCommandServer, SmallResponseLimitPreventsWrite)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "command.sock";
  auto config = config_for(path);
  config.max_response_bytes = 32U;
  CommandServer server{config, []() {return NOW_UNIX_MS;}};
  ASSERT_EQ(server.start().status, CommandServerStatus::Started);
  const auto outcome = exchange(server, path, CANONICAL_STOP);
  EXPECT_EQ(outcome.server.status, CommandServerStatus::ResponseTooLarge);
  EXPECT_FALSE(outcome.server.response_sent);
  EXPECT_EQ(outcome.server.bytes_sent, 0U);
}

TEST(SavoBridgeCommandServer, ClientDisconnectDuringResponseIsSafe)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "command.sock";
  CommandServer server{config_for(path), []() {return NOW_UNIX_MS;}};
  ASSERT_EQ(server.start().status, CommandServerStatus::Started);

  auto future = std::async(
    std::launch::async,
    [&server]() {return server.serve_one();});
  {
    auto client = connect_client(path);
    ASSERT_GE(client.get(), 0);
    const std::string frame = compact_json(CANONICAL_STOP) + "\n";
    ASSERT_EQ(
      ::send(client.get(), frame.data(), frame.size(), 0),
      static_cast<ssize_t>(frame.size()));
    linger reset{1, 0};
    ASSERT_EQ(
      ::setsockopt(
        client.get(), SOL_SOCKET, SO_LINGER, &reset, sizeof(reset)),
      0);
  }

  const auto result = future.get();
  EXPECT_TRUE(
    result.status == CommandServerStatus::WriteError ||
    result.status == CommandServerStatus::DryRunAcknowledged);
  EXPECT_FALSE(result.dispatch_attempted);
  EXPECT_EQ(result.ros_publications, 0U);
}

TEST(SavoBridgeCommandServer, OneServeCallHandlesAtMostOneClient)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "command.sock";
  auto config = config_for(path);
  config.accept_timeout_ms = 30;
  CommandServer server{config, []() {return NOW_UNIX_MS;}};
  ASSERT_EQ(server.start().status, CommandServerStatus::Started);

  const auto first = exchange(server, path, compact_json(CANONICAL_STOP));
  EXPECT_EQ(first.server.status, CommandServerStatus::DryRunAcknowledged)
    << first.server.reason << '\n' << first.response;
  const auto second = server.serve_one();
  EXPECT_EQ(second.status, CommandServerStatus::AcceptTimeout);
}

TEST(SavoBridgeCommandServer, LifecycleCreatesNoBackgroundThread)
{
  const auto task_count = []() {
      return static_cast<std::size_t>(std::distance(
          std::filesystem::directory_iterator("/proc/self/task"),
          std::filesystem::directory_iterator{}));
    };

  TemporaryDirectory temporary;
  const std::size_t before = task_count();
  CommandServer server{config_for(temporary.path() / "command.sock")};
  ASSERT_EQ(server.start().status, CommandServerStatus::Started);
  EXPECT_EQ(task_count(), before);
}


TEST(SavoBridgeCommandServer, DuplicateCommandIdReplaysWithoutRedispatch)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "command.sock";

  auto config = config_for(path);
  config.execution_mode = "live";
  config.command_id_cache_capacity = 8U;

  std::atomic<std::size_t> dispatch_count{0U};

  CommandServer server{
    config,
    []() {return NOW_UNIX_MS;},
    [&dispatch_count](
      const savo_bridge::ValidatedCommand &)
    {
      dispatch_count.fetch_add(1U);

      savo_bridge::CommandDispatchResult result;
      result.accepted = true;
      result.state = "accepted";
      result.reason = "test_live_dispatch_accepted";
      result.dispatch_attempted = true;
      result.ros_publications = 3U;
      result.result_json = R"json({"scope":"mapping","active":true})json";
      return result;
    }};

  ASSERT_EQ(
    server.start().status,
    CommandServerStatus::Started);

  const auto first =
    exchange(
    server,
    path,
    compact_json(CANONICAL_STOP));

  const auto second =
    exchange(
    server,
    path,
    compact_json(CANONICAL_STOP));

  ASSERT_FALSE(first.response.empty());
  ASSERT_FALSE(second.response.empty());

  const Json first_json = Json::parse(
    first.response.substr(
      0U,
      first.response.size() - 1U));

  const Json second_json = Json::parse(
    second.response.substr(
      0U,
      second.response.size() - 1U));

  EXPECT_EQ(dispatch_count.load(), 1U);

  EXPECT_EQ(
    first.server.status,
    CommandServerStatus::CommandAcknowledged);

  EXPECT_FALSE(first.server.duplicate);
  EXPECT_FALSE(first_json.at("duplicate").get<bool>());

  EXPECT_EQ(
    first_json.at("details").
    at("dispatch_attempted"),
    true);

  EXPECT_EQ(
    first_json.at("details").
    at("ros_publications"),
    3);
  EXPECT_EQ(first_json.at("details").at("result").at("scope"), "mapping");
  EXPECT_TRUE(first_json.at("details").at("result").at("active").get<bool>());

  EXPECT_EQ(
    second.server.status,
    CommandServerStatus::CommandAcknowledged);

  EXPECT_TRUE(second.server.duplicate);
  EXPECT_TRUE(second_json.at("duplicate").get<bool>());

  EXPECT_EQ(
    second_json.at("reason"),
    "test_live_dispatch_accepted");

  EXPECT_EQ(
    second_json.at("details").
    at("dispatch_attempted"),
    false);

  EXPECT_EQ(
    second_json.at("details").
    at("ros_publications"),
    0);
  EXPECT_EQ(
    second_json.at("details").at("result"),
    first_json.at("details").at("result"));

  EXPECT_FALSE(second.server.dispatch_attempted);
  EXPECT_EQ(second.server.ros_publications, 0U);
}

TEST(SavoBridgeCommandServer, TransportHasNoRosOrNodeIntegration)
{
  const std::filesystem::path package_root =
    std::filesystem::path{__FILE__}.parent_path().parent_path().parent_path();
  const std::vector<std::filesystem::path> files{
    package_root / "include/savo_bridge/command_server.hpp",
    package_root / "src/command_server.cpp",
  };
  const std::vector<std::string> forbidden{
    "rclcpp",
    "publisher",
    "geometry_msgs",
    "std_msgs",
    "diagnostic_msgs",
    "cmd_vel",
    "NavigateToPose",
  };

  for (const auto & file : files) {
    std::ifstream input(file);
    ASSERT_TRUE(input) << file;
    const std::string content{
      std::istreambuf_iterator<char>{input},
      std::istreambuf_iterator<char>{}};
    for (const auto & token : forbidden) {
      EXPECT_EQ(content.find(token), std::string::npos) << token;
    }
  }
}

}  // namespace
