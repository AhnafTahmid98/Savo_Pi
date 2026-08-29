// Copyright 2026 Ahnaf Tahmid
#include "savo_speech/transport/robot_playback_server.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

#include "savo_speech/audio/wav_reader.hpp"

#ifdef __linux__
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#endif

namespace savo_speech::transport
{
namespace
{
constexpr std::size_t kRequestHeaderSize{20U};
constexpr std::size_t kAckSize{16U};
constexpr std::array<std::uint8_t, 4U> kRequestMagic{'S', 'V', 'P', 'W'};
constexpr std::array<std::uint8_t, 4U> kAckMagic{'S', 'V', 'P', 'A'};
constexpr std::uint16_t kProtocolVersion{1U};

enum class AckStatus : std::uint16_t
{
  Accepted = 0U, InvalidHeader = 1U, PayloadTooLarge = 2U,
  Unauthorized = 3U, InvalidWav = 4U, InvalidFormat = 5U,
  PlaybackRejected = 6U, InternalError = 7U
};

std::uint16_t read_u16_be(const std::uint8_t * data)
{
  return static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(data[0]) << 8U) | data[1]);
}

std::uint32_t read_u32_be(const std::uint8_t * data)
{
  return (static_cast<std::uint32_t>(data[0]) << 24U) |
         (static_cast<std::uint32_t>(data[1]) << 16U) |
         (static_cast<std::uint32_t>(data[2]) << 8U) | data[3];
}

std::uint64_t read_u64_be(const std::uint8_t * data)
{
  std::uint64_t value{0U};
  for (std::size_t index = 0U; index < 8U; ++index) {
    value = (value << 8U) | data[index];
  }
  return value;
}

void write_u16_be(std::uint8_t * data, const std::uint16_t value)
{
  data[0] = static_cast<std::uint8_t>(value >> 8U);
  data[1] = static_cast<std::uint8_t>(value);
}

void write_u64_be(std::uint8_t * data, const std::uint64_t value)
{
  for (std::size_t index = 0U; index < 8U; ++index) {
    data[7U - index] = static_cast<std::uint8_t>(value >> (index * 8U));
  }
}

#ifdef __linux__
bool receive_exact(const int fd, std::span<std::uint8_t> output)
{
  std::size_t offset{0U};
  while (offset < output.size()) {
    const ssize_t count = ::recv(fd, output.data() + offset, output.size() - offset, 0);
    if (count == 0) {return false;}
    if (count < 0) {
      if (errno == EINTR) {continue;}
      return false;
    }
    offset += static_cast<std::size_t>(count);
  }
  return true;
}

void send_ack(const int fd, const AckStatus status, const std::uint64_t request_id)
{
  std::array<std::uint8_t, kAckSize> ack{};
  std::copy(kAckMagic.begin(), kAckMagic.end(), ack.begin());
  write_u16_be(ack.data() + 4U, kProtocolVersion);
  write_u16_be(ack.data() + 6U, static_cast<std::uint16_t>(status));
  write_u64_be(ack.data() + 8U, request_id);
  std::size_t offset{0U};
  while (offset < ack.size()) {
    const ssize_t count = ::send(
      fd, ack.data() + offset, ack.size() - offset, MSG_NOSIGNAL);
    if (count < 0 && errno == EINTR) {continue;}
    if (count <= 0) {return;}
    offset += static_cast<std::size_t>(count);
  }
}
#endif
}  // namespace

bool RobotPlaybackServerConfig::is_valid() const noexcept
{
  return !socket_path.empty() && socket_path.front() == '/' &&
         socket_path.size() < 108U && maximum_wav_bytes >= 44U &&
         maximum_wav_bytes <= 64U * 1024U * 1024U;
}

RobotPlaybackServer::RobotPlaybackServer(
  audio::AudioRuntime & audio_runtime, RobotPlaybackServerConfig config)
: audio_runtime_{audio_runtime}, config_{std::move(config)}
{
  if (!config_.is_valid()) {
    throw std::invalid_argument{"invalid robot playback server config"};
  }
}

RobotPlaybackServer::~RobotPlaybackServer() {stop();}

bool RobotPlaybackServer::start()
{
  if (thread_.joinable() || running_.load()) {return false;}
  if (!audio_runtime_.ready()) {
    throw std::runtime_error{"audio runtime must be running before robot playback server"};
  }
#ifndef __linux__
  throw std::runtime_error{"robot playback Unix socket requires Linux"};
#else
  {const std::scoped_lock lock{mutex_}; last_error_.clear();}
  const std::filesystem::path path{config_.socket_path};
  std::filesystem::create_directories(path.parent_path());
  struct stat existing {};
  if (::lstat(config_.socket_path.c_str(), &existing) == 0) {
    if (!S_ISSOCK(existing.st_mode)) {
      throw std::runtime_error{"playback socket path exists and is not a socket"};
    }
    if (::unlink(config_.socket_path.c_str()) != 0) {
      throw std::runtime_error{"unable to remove stale playback socket"};
    }
  }
  const int fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (fd < 0) {throw std::runtime_error{"unable to create playback socket"};}
  listener_fd_.store(fd);
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::memcpy(
    address.sun_path, config_.socket_path.c_str(),
    config_.socket_path.size() + 1U);
  // Mode 0666 lets the containerized SavoMind process connect without sharing
  // the host user's group. SO_PEERCRED below remains the authorization check
  // and validates the configured production peer UID (10001).
  if (::bind(
      fd, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) != 0 ||
    ::chmod(config_.socket_path.c_str(), 0666) != 0 || ::listen(fd, 4) != 0)
  {
    listener_fd_.store(-1);
    ::close(fd);
    static_cast<void>(::unlink(config_.socket_path.c_str()));
    throw std::runtime_error{"unable to bind or listen on playback socket"};
  }
  running_.store(true);
  thread_ = std::jthread([this]() {run();});
  return true;
#endif
}

void RobotPlaybackServer::stop() noexcept
{
  thread_.request_stop();
#ifdef __linux__
  const int fd = listener_fd_.exchange(-1);
  if (fd >= 0) {::shutdown(fd, SHUT_RDWR); ::close(fd);}
  const int client = active_client_fd_.exchange(-1);
  if (client >= 0) {::shutdown(client, SHUT_RDWR); ::close(client);}
#endif
  if (thread_.joinable()) {thread_.join();}
  running_.store(false);
}

bool RobotPlaybackServer::running() const noexcept {return running_.load();}

std::string RobotPlaybackServer::last_error() const
{
  const std::scoped_lock lock{mutex_};
  return last_error_;
}

void RobotPlaybackServer::set_error(std::string error)
{
  const std::scoped_lock lock{mutex_};
  last_error_ = std::move(error);
}

void RobotPlaybackServer::run() noexcept
{
#ifdef __linux__
  try {
    const int fd = listener_fd_.load();
    while (!thread_.get_stop_token().stop_requested()) {
      const int client = ::accept4(fd, nullptr, nullptr, SOCK_CLOEXEC);
      if (client < 0) {
        if (errno == EINTR) {continue;}
        if (thread_.get_stop_token().stop_requested()) {break;}
        throw std::runtime_error{"playback socket accept failed"};
      }
      active_client_fd_.store(client);
      timeval timeout{30, 0};
      static_cast<void>(::setsockopt(
        client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)));
      static_cast<void>(::setsockopt(
        client, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)));
      handle_client(client);
      if (active_client_fd_.exchange(-1) == client) {::close(client);}
    }
  } catch (const std::exception & exception) {
    set_error(exception.what());
  }
  const int fd = listener_fd_.exchange(-1);
  if (fd >= 0) {::close(fd);}
  static_cast<void>(::unlink(config_.socket_path.c_str()));
  running_.store(false);
#endif
}

void RobotPlaybackServer::handle_client(const int client_fd) noexcept
{
#ifdef __linux__
  std::uint64_t request_id{0U};
  try {
    if (config_.require_peer_uid) {
      ucred credentials{}; socklen_t size{sizeof(credentials)};
      if (::getsockopt(client_fd, SOL_SOCKET, SO_PEERCRED, &credentials, &size) != 0 ||
        static_cast<std::uint32_t>(credentials.uid) != config_.peer_uid)
      {send_ack(client_fd, AckStatus::Unauthorized, 0U); return;}
    }
    std::array<std::uint8_t, kRequestHeaderSize> header{};
    if (!receive_exact(client_fd, header)) {return;}
    request_id = read_u64_be(header.data() + 8U);
    if (!std::equal(kRequestMagic.begin(), kRequestMagic.end(), header.begin()) ||
      read_u16_be(header.data() + 4U) != kProtocolVersion ||
      read_u16_be(header.data() + 6U) != 0U || request_id == 0U)
    {send_ack(client_fd, AckStatus::InvalidHeader, request_id); return;}
    const std::size_t payload_size = read_u32_be(header.data() + 16U);
    if (payload_size < 44U || payload_size > config_.maximum_wav_bytes) {
      send_ack(client_fd, AckStatus::PayloadTooLarge, request_id); return;
    }
    std::vector<std::uint8_t> wav(payload_size);
    if (!receive_exact(client_fd, wav)) {return;}
    audio::WavReadLimits limits;
    limits.maximum_file_bytes = config_.maximum_wav_bytes;
    limits.maximum_audio_data_bytes = config_.maximum_wav_bytes - 44U;
    limits.maximum_channels = 1U;
    limits.minimum_sample_rate_hz = 16000U;
    limits.maximum_sample_rate_hz = 16000U;
    audio::AudioBuffer decoded;
    try {
      decoded = audio::WavReader::decode(wav, limits);
    } catch (const std::exception &) {
      send_ack(client_fd, AckStatus::InvalidWav, request_id);
      return;
    }
    if (decoded.format.sample_rate_hz != 16000U || decoded.format.channels != 1U ||
      decoded.format.sample_format != audio::PcmSampleFormat::Signed16LittleEndian)
    {send_ack(client_fd, AckStatus::InvalidFormat, request_id); return;}
    audio::PlaybackRequest request{request_id, std::move(decoded)};
    const auto result = audio_runtime_.enqueue_playback(std::move(request));
    send_ack(client_fd, result == audio::PlaybackEnqueueResult::Accepted ?
      AckStatus::Accepted : AckStatus::PlaybackRejected, request_id);
  } catch (const std::exception & exception) {
    set_error(exception.what()); send_ack(client_fd, AckStatus::InternalError, request_id);
  }
#else
  static_cast<void>(client_fd);
#endif
}

}  // namespace savo_speech::transport
