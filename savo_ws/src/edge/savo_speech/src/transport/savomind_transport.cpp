// Copyright 2026 Ahnaf Tahmid
#include "savo_speech/transport/savomind_transport.hpp"

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
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace savo_speech::transport
{
namespace
{

using Clock = std::chrono::steady_clock;

constexpr std::array<std::uint8_t, 8> REQUEST_MAGIC{
  'S', 'A', 'V', 'O', 'S', 'P', 'R', 'Q'};
constexpr std::array<std::uint8_t, 8> RESPONSE_MAGIC{
  'S', 'A', 'V', 'O', 'S', 'P', 'R', 'S'};
constexpr std::array<std::uint8_t, 8> PLAYBACK_ACK_MAGIC{
  'S', 'A', 'V', 'O', 'S', 'P', 'A', 'K'};
constexpr std::array<std::uint8_t, 8> PLAYBACK_ACK_RESPONSE_MAGIC{
  'S', 'A', 'V', 'O', 'S', 'P', 'A', 'R'};
constexpr std::size_t REQUEST_HEADER_SIZE = 8U + 4U + 4U + 4U + 8U;
constexpr std::size_t RESPONSE_HEADER_SIZE =
  8U + 4U + 4U + 4U + 4U + 4U + 4U + 4U + 4U + 8U;
constexpr std::size_t PLAYBACK_ACK_HEADER_SIZE =
  8U + 4U + 4U + 4U + 4U + 4U + 4U;
constexpr std::size_t PLAYBACK_ACK_RESPONSE_HEADER_SIZE =
  8U + 4U + 4U + 4U + 4U;
constexpr std::uint32_t FLAG_PLAYBACK_ACK_REQUIRED = 1U;

class FileDescriptor
{
public:
  explicit FileDescriptor(const int value = -1) noexcept
  : value_{value} {}
  ~FileDescriptor() {reset();}
  FileDescriptor(const FileDescriptor &) = delete;
  FileDescriptor & operator=(const FileDescriptor &) = delete;
  FileDescriptor(FileDescriptor && other) noexcept
  : value_{other.release()} {}
  FileDescriptor & operator=(FileDescriptor && other) noexcept
  {
    if (this != &other) {reset(other.release());}
    return *this;
  }
  [[nodiscard]] int get() const noexcept {return value_;}
  [[nodiscard]] bool valid() const noexcept {return value_ >= 0;}
  [[nodiscard]] int release() noexcept
  {
    const int value = value_;
    value_ = -1;
    return value;
  }
  void reset(const int value = -1) noexcept
  {
    if (value_ >= 0) {(void)::close(value_);}
    value_ = value;
  }

private:
  int value_{-1};
};

[[nodiscard]] std::uint16_t read_u16_le(
  const std::vector<std::uint8_t> & bytes,
  const std::size_t offset)
{
  return static_cast<std::uint16_t>(bytes[offset]) |
         static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(bytes[offset + 1U]) << 8U);
}

[[nodiscard]] std::uint32_t read_u32_le(
  const std::vector<std::uint8_t> & bytes,
  const std::size_t offset)
{
  return static_cast<std::uint32_t>(bytes[offset]) |
         (static_cast<std::uint32_t>(bytes[offset + 1U]) << 8U) |
         (static_cast<std::uint32_t>(bytes[offset + 2U]) << 16U) |
         (static_cast<std::uint32_t>(bytes[offset + 3U]) << 24U);
}

[[nodiscard]] bool has_tag(
  const std::vector<std::uint8_t> & bytes,
  const std::size_t offset,
  const char * tag)
{
  if (offset > bytes.size() || bytes.size() - offset < 4U) {return false;}
  return std::equal(
    tag, tag + 4,
    bytes.begin() + static_cast<std::ptrdiff_t>(offset));
}

[[nodiscard]] Validation validate_wav(
  const std::vector<std::uint8_t> & wav,
  const Limits & limits)
{
  if (wav.size() < 44U || !has_tag(wav, 0U, "RIFF") ||
    !has_tag(wav, 8U, "WAVE") || !has_tag(wav, 12U, "fmt "))
  {
    return {false, "invalid_wav_header"};
  }
  if (read_u16_le(wav, 20U) != 1U) {return {false, "tts_wav_not_pcm"};}
  if (read_u16_le(wav, 22U) != limits.required_channels) {
    return {false, "tts_wav_channel_mismatch"};
  }
  if (read_u32_le(wav, 24U) != limits.required_sample_rate_hz) {
    return {false, "tts_wav_sample_rate_mismatch"};
  }
  if (read_u16_le(wav, 34U) != limits.required_bits_per_sample) {
    return {false, "tts_wav_sample_width_mismatch"};
  }
  return {true, "accepted"};
}

void append_u32(std::vector<std::uint8_t> & output, const std::uint32_t value)
{
  output.push_back(static_cast<std::uint8_t>((value >> 24U) & 0xFFU));
  output.push_back(static_cast<std::uint8_t>((value >> 16U) & 0xFFU));
  output.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
  output.push_back(static_cast<std::uint8_t>(value & 0xFFU));
}

void append_u64(std::vector<std::uint8_t> & output, const std::uint64_t value)
{
  for (int shift = 56; shift >= 0; shift -= 8) {
    output.push_back(static_cast<std::uint8_t>(
        (value >> static_cast<unsigned int>(shift)) & 0xFFU));
  }
}

[[nodiscard]] std::uint32_t read_u32_be(
  const std::vector<std::uint8_t> & input,
  const std::size_t offset)
{
  return (static_cast<std::uint32_t>(input[offset]) << 24U) |
         (static_cast<std::uint32_t>(input[offset + 1U]) << 16U) |
         (static_cast<std::uint32_t>(input[offset + 2U]) << 8U) |
         static_cast<std::uint32_t>(input[offset + 3U]);
}

[[nodiscard]] std::uint64_t read_u64_be(
  const std::vector<std::uint8_t> & input,
  const std::size_t offset)
{
  std::uint64_t value = 0U;
  for (std::size_t index = 0U; index < 8U; ++index) {
    value = (value << 8U) | static_cast<std::uint64_t>(input[offset + index]);
  }
  return value;
}

[[nodiscard]] std::uint32_t checked_u32(
  const std::size_t value,
  const char * field)
{
  if (value > static_cast<std::size_t>(
      std::numeric_limits<std::uint32_t>::max()))
  {
    throw TransportError{std::string{field} + "_too_large"};
  }
  return static_cast<std::uint32_t>(value);
}

[[nodiscard]] int remaining_ms(const Clock::time_point deadline)
{
  const auto now = Clock::now();
  if (now >= deadline) {return 0;}
  const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
    deadline - now);
  const auto count = duration.count();
  return count > static_cast<std::int64_t>(std::numeric_limits<int>::max()) ?
         std::numeric_limits<int>::max() : static_cast<int>(count);
}

void wait_fd(
  const int fd,
  const std::int16_t events,
  const Clock::time_point deadline)
{
  pollfd descriptor{};
  descriptor.fd = fd;
  descriptor.events = events;
  while (true) {
    const int timeout = remaining_ms(deadline);
    if (timeout <= 0) {throw TransportError{"savomind_transport_timeout"};}
    const int result = ::poll(&descriptor, 1U, timeout);
    if (result > 0) {
      if ((descriptor.revents & events) != 0) {return;}
      if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        throw TransportError{"savomind_socket_closed"};
      }
      continue;
    }
    if (result == 0) {throw TransportError{"savomind_transport_timeout"};}
    if (errno == EINTR) {continue;}
    throw TransportError{
            "savomind_poll_failed:" + std::string{std::strerror(errno)}};
  }
}

void write_all(
  const int fd,
  const std::vector<std::uint8_t> & bytes,
  const Clock::time_point deadline)
{
  std::size_t offset = 0U;
  while (offset < bytes.size()) {
    wait_fd(fd, POLLOUT, deadline);
    const ssize_t count = ::send(
      fd,
      bytes.data() + static_cast<std::ptrdiff_t>(offset),
      bytes.size() - offset,
      MSG_NOSIGNAL);
    if (count > 0) {
      offset += static_cast<std::size_t>(count);
      continue;
    }
    if (count < 0 &&
      (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK))
    {
      continue;
    }
    throw TransportError{
            "savomind_write_failed:" + std::string{std::strerror(errno)}};
  }
}

[[nodiscard]] std::vector<std::uint8_t> read_exact(
  const int fd,
  const std::size_t size,
  const Clock::time_point deadline)
{
  std::vector<std::uint8_t> bytes(size);
  std::size_t offset = 0U;
  while (offset < size) {
    wait_fd(fd, POLLIN, deadline);
    const ssize_t count = ::recv(
      fd,
      bytes.data() + static_cast<std::ptrdiff_t>(offset),
      size - offset,
      0);
    if (count > 0) {
      offset += static_cast<std::size_t>(count);
      continue;
    }
    if (count == 0) {throw TransportError{"savomind_eof"};}
    if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {continue;}
    throw TransportError{
            "savomind_read_failed:" + std::string{std::strerror(errno)}};
  }
  return bytes;
}

[[nodiscard]] std::string bytes_to_string(
  const std::vector<std::uint8_t> & bytes,
  std::size_t & offset,
  const std::size_t length)
{
  if (offset > bytes.size() || length > bytes.size() - offset) {
    throw TransportError{"savomind_frame_truncated"};
  }
  const auto begin = bytes.begin() + static_cast<std::ptrdiff_t>(offset);
  const auto end = begin + static_cast<std::ptrdiff_t>(length);
  offset += length;
  return std::string(begin, end);
}

[[nodiscard]] std::vector<std::uint8_t> encode_request(const Request & request)
{
  std::vector<std::uint8_t> frame;
  frame.reserve(
    REQUEST_HEADER_SIZE + request.request_id.size() +
    request.session_id.size() + request.wav.size());
  frame.insert(frame.end(), REQUEST_MAGIC.begin(), REQUEST_MAGIC.end());
  append_u32(frame, SAVOMIND_SPEECH_PROTOCOL_VERSION);
  append_u32(frame, checked_u32(request.request_id.size(), "request_id"));
  append_u32(frame, checked_u32(request.session_id.size(), "session_id"));
  append_u64(frame, static_cast<std::uint64_t>(request.wav.size()));
  frame.insert(frame.end(), request.request_id.begin(), request.request_id.end());
  frame.insert(frame.end(), request.session_id.begin(), request.session_id.end());
  frame.insert(frame.end(), request.wav.begin(), request.wav.end());
  return frame;
}

[[nodiscard]] std::vector<std::uint8_t> encode_playback_ack(
  const PlaybackAckRequest & request)
{
  std::vector<std::uint8_t> frame;
  frame.reserve(
    PLAYBACK_ACK_HEADER_SIZE + request.request_id.size() +
    request.session_id.size() + request.playback_token.size() +
    request.reason.size());
  frame.insert(frame.end(), PLAYBACK_ACK_MAGIC.begin(), PLAYBACK_ACK_MAGIC.end());
  append_u32(frame, SAVOMIND_SPEECH_PROTOCOL_VERSION);
  append_u32(frame, request.playback_ok ? 0U : 1U);
  append_u32(frame, checked_u32(request.request_id.size(), "request_id"));
  append_u32(frame, checked_u32(request.session_id.size(), "session_id"));
  append_u32(frame, checked_u32(request.playback_token.size(), "playback_token"));
  append_u32(frame, checked_u32(request.reason.size(), "reason"));
  frame.insert(frame.end(), request.request_id.begin(), request.request_id.end());
  frame.insert(frame.end(), request.session_id.begin(), request.session_id.end());
  frame.insert(
    frame.end(), request.playback_token.begin(), request.playback_token.end());
  frame.insert(frame.end(), request.reason.begin(), request.reason.end());
  return frame;
}

[[nodiscard]] bool safe_socket_path(const std::string & socket_path)
{
  if (socket_path.empty() || socket_path.front() != '/' ||
    socket_path.size() >= sizeof(sockaddr_un::sun_path))
  {
    return false;
  }
  struct stat status{};
  if (::lstat(socket_path.c_str(), &status) != 0) {return false;}
  return S_ISSOCK(status.st_mode) && !S_ISLNK(status.st_mode);
}

[[nodiscard]] FileDescriptor connect_socket(const UnixSocketConfig & config)
{
  if (!safe_socket_path(config.socket_path)) {
    throw TransportError{"savomind_socket_missing_or_unsafe"};
  }
  FileDescriptor descriptor{
    ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0)};
  if (!descriptor.valid()) {
    throw TransportError{
            "savomind_socket_create_failed:" + std::string{std::strerror(errno)}};
  }
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::memcpy(
    address.sun_path, config.socket_path.c_str(), config.socket_path.size() + 1U);
  const int result = ::connect(
    descriptor.get(),
    reinterpret_cast<const sockaddr *>(&address),
    static_cast<socklen_t>(
      offsetof(sockaddr_un, sun_path) + config.socket_path.size() + 1U));
  if (result != 0) {
    if (errno != EINPROGRESS) {
      throw TransportError{
              "savomind_connect_failed:" + std::string{std::strerror(errno)}};
    }
    const auto deadline = Clock::now() + config.connect_timeout;
    wait_fd(descriptor.get(), POLLOUT, deadline);
    int socket_error = 0;
    socklen_t error_size = sizeof(socket_error);
    if (::getsockopt(
        descriptor.get(), SOL_SOCKET, SO_ERROR, &socket_error, &error_size) != 0 ||
      socket_error != 0)
    {
      throw TransportError{
              "savomind_connect_failed:" +
              std::string{std::strerror(socket_error == 0 ? errno : socket_error)}};
    }
  }
#ifdef SO_PEERCRED
  if (config.required_server_uid.has_value()) {
    ucred credentials{};
    socklen_t size = sizeof(credentials);
    if (::getsockopt(
        descriptor.get(), SOL_SOCKET, SO_PEERCRED, &credentials, &size) != 0)
    {
      throw TransportError{"savomind_peer_credentials_unavailable"};
    }
    if (credentials.uid != config.required_server_uid.value()) {
      throw TransportError{"savomind_server_uid_rejected"};
    }
  }
#else
  if (config.required_server_uid.has_value()) {
    throw TransportError{"savomind_peer_credentials_unsupported"};
  }
#endif
  return descriptor;
}

}  // namespace

bool UnixSocketConfig::is_valid() const noexcept
{
  return !socket_path.empty() && socket_path.front() == '/' &&
         socket_path.size() < sizeof(sockaddr_un::sun_path) &&
         connect_timeout >= std::chrono::milliseconds{1} &&
         connect_timeout <= std::chrono::minutes{1} &&
         io_timeout >= std::chrono::milliseconds{1} &&
         io_timeout <= std::chrono::minutes{5} &&
         limits.maximum_request_bytes >= 44U &&
         limits.maximum_response_bytes >= 44U &&
         limits.maximum_text_bytes >= 1U &&
         limits.maximum_text_bytes <= limits.maximum_response_bytes &&
         limits.required_sample_rate_hz > 0U && limits.required_channels > 0U &&
         limits.required_bits_per_sample == 16U;
}

Validation validate_request(const Request & request, const Limits & limits)
{
  if (request.request_id.empty() || request.session_id.empty()) {
    return {false, "request_identity_missing"};
  }
  if (request.request_id.size() > 128U || request.session_id.size() > 128U) {
    return {false, "request_identity_oversized"};
  }
  if (request.wav.empty()) {return {false, "request_audio_empty"};}
  if (request.wav.size() > limits.maximum_request_bytes) {
    return {false, "request_audio_oversized"};
  }
  return validate_wav(request.wav, limits);
}

Validation validate_response(
  const Request & request,
  const Response & response,
  const Limits & limits)
{
  if (response.request_id != request.request_id) {return {false, "wrong_request_id"};}
  if (!response.success) {
    return {
      false,
      response.reason.empty() ? "savomind_request_rejected" : response.reason};
  }
  if (response.transcript.size() > limits.maximum_text_bytes ||
    response.reply.size() > limits.maximum_text_bytes ||
    response.reason.size() > limits.maximum_text_bytes ||
    response.playback_token.size() > limits.maximum_text_bytes)
  {
    return {false, "response_text_oversized"};
  }
  const std::size_t text_size =
    response.transcript.size() + response.reply.size() + response.reason.size() +
    response.playback_token.size();
  if (text_size > limits.maximum_response_bytes ||
    response.tts_wav.size() > limits.maximum_response_bytes - text_size)
  {
    return {false, "response_oversized"};
  }
  if (response.transcript.empty()) {return {false, "transcript_missing"};}
  if (response.reply.empty()) {return {false, "reply_missing"};}
  if (response.tts_wav.empty()) {return {false, "tts_audio_missing"};}
  if (response.playback_ack_required && response.playback_token.empty()) {
    return {false, "playback_token_missing"};
  }
  return validate_wav(response.tts_wav, limits);
}

Validation validate_playback_ack_request(
  const PlaybackAckRequest & request,
  const Limits & limits)
{
  if (request.request_id.empty() || request.session_id.empty() ||
    request.playback_token.empty())
  {
    return {false, "playback_ack_identity_missing"};
  }
  if (request.request_id.size() > 128U || request.session_id.size() > 128U) {
    return {false, "playback_ack_identity_oversized"};
  }
  if (request.playback_token.size() > limits.maximum_text_bytes ||
    request.reason.size() > limits.maximum_text_bytes)
  {
    return {false, "playback_ack_text_oversized"};
  }
  const std::size_t total = request.request_id.size() + request.session_id.size() +
    request.playback_token.size() + request.reason.size();
  if (total > limits.maximum_request_bytes) {
    return {false, "playback_ack_oversized"};
  }
  return {true, "accepted"};
}

Validation validate_playback_ack_response(
  const PlaybackAckRequest & request,
  const PlaybackAckResponse & response,
  const Limits & limits)
{
  if (response.request_id != request.request_id) {
    return {false, "wrong_playback_ack_request_id"};
  }
  if (response.reason.size() > limits.maximum_text_bytes) {
    return {false, "playback_ack_reason_oversized"};
  }
  if (!response.success) {
    return {
      false,
      response.reason.empty() ? "playback_ack_rejected" : response.reason};
  }
  return {true, "accepted"};
}

class UnixSocketSavoMindTransport::Impl
{
public:
  explicit Impl(UnixSocketConfig config)
  : config_{std::move(config)}
  {
    if (!config_.is_valid()) {
      throw std::invalid_argument{"invalid SavoMind Unix-socket config"};
    }
  }

  Response exchange(const Request & request)
  {
    const Validation request_validation = validate_request(request, config_.limits);
    if (!request_validation.accepted) {throw TransportError{request_validation.reason};}

    begin_operation(request.request_id);
    FileDescriptor descriptor;
    try {
      descriptor = connect_socket(config_);
      set_active_fd(request.request_id, descriptor.get());
      const auto deadline = Clock::now() + config_.io_timeout;
      write_all(descriptor.get(), encode_request(request), deadline);
      const auto header = read_exact(descriptor.get(), RESPONSE_HEADER_SIZE, deadline);
      if (!std::equal(RESPONSE_MAGIC.begin(), RESPONSE_MAGIC.end(), header.begin())) {
        throw TransportError{"savomind_response_magic_invalid"};
      }
      const std::uint32_t version = read_u32_be(header, 8U);
      if (version != SAVOMIND_SPEECH_PROTOCOL_VERSION) {
        throw TransportError{"savomind_protocol_version_unsupported"};
      }
      const std::uint32_t status = read_u32_be(header, 12U);
      const std::uint32_t flags = read_u32_be(header, 16U);
      if ((flags & ~FLAG_PLAYBACK_ACK_REQUIRED) != 0U) {
        throw TransportError{"savomind_response_flags_invalid"};
      }
      const std::size_t request_id_size = read_u32_be(header, 20U);
      const std::size_t reason_size = read_u32_be(header, 24U);
      const std::size_t transcript_size = read_u32_be(header, 28U);
      const std::size_t reply_size = read_u32_be(header, 32U);
      const std::size_t token_size = read_u32_be(header, 36U);
      const std::uint64_t wav_size_u64 = read_u64_be(header, 40U);
      if (request_id_size > 128U ||
        reason_size > config_.limits.maximum_text_bytes ||
        transcript_size > config_.limits.maximum_text_bytes ||
        reply_size > config_.limits.maximum_text_bytes ||
        token_size > config_.limits.maximum_text_bytes ||
        wav_size_u64 > config_.limits.maximum_response_bytes)
      {
        throw TransportError{"savomind_response_length_invalid"};
      }
      const std::size_t wav_size = static_cast<std::size_t>(wav_size_u64);
      const std::size_t text_total = request_id_size + reason_size +
        transcript_size + reply_size + token_size;
      if (text_total > config_.limits.maximum_response_bytes ||
        wav_size > config_.limits.maximum_response_bytes - text_total)
      {
        throw TransportError{"savomind_response_oversized"};
      }
      auto body = read_exact(descriptor.get(), text_total + wav_size, deadline);
      std::size_t offset = 0U;
      Response response;
      response.success = status == 0U;
      response.playback_ack_required =
        (flags & FLAG_PLAYBACK_ACK_REQUIRED) != 0U;
      response.request_id = bytes_to_string(body, offset, request_id_size);
      response.reason = bytes_to_string(body, offset, reason_size);
      response.transcript = bytes_to_string(body, offset, transcript_size);
      response.reply = bytes_to_string(body, offset, reply_size);
      response.playback_token = bytes_to_string(body, offset, token_size);
      response.tts_wav.assign(
        body.begin() + static_cast<std::ptrdiff_t>(offset), body.end());
      const Validation response_validation =
        validate_response(request, response, config_.limits);
      if (!response_validation.accepted) {
        throw TransportError{response_validation.reason};
      }
      finish_success(false);
      return response;
    } catch (const std::exception & exception) {
      finish_failure(exception.what(), false);
      throw;
    }
  }

  PlaybackAckResponse acknowledge_playback(const PlaybackAckRequest & request)
  {
    const Validation request_validation =
      validate_playback_ack_request(request, config_.limits);
    if (!request_validation.accepted) {throw TransportError{request_validation.reason};}

    begin_operation(request.request_id);
    FileDescriptor descriptor;
    try {
      descriptor = connect_socket(config_);
      set_active_fd(request.request_id, descriptor.get());
      const auto deadline = Clock::now() + config_.io_timeout;
      write_all(descriptor.get(), encode_playback_ack(request), deadline);
      const auto header = read_exact(
        descriptor.get(), PLAYBACK_ACK_RESPONSE_HEADER_SIZE, deadline);
      if (!std::equal(
          PLAYBACK_ACK_RESPONSE_MAGIC.begin(),
          PLAYBACK_ACK_RESPONSE_MAGIC.end(),
          header.begin()))
      {
        throw TransportError{"playback_ack_response_magic_invalid"};
      }
      const std::uint32_t version = read_u32_be(header, 8U);
      if (version != SAVOMIND_SPEECH_PROTOCOL_VERSION) {
        throw TransportError{"savomind_protocol_version_unsupported"};
      }
      const std::uint32_t status = read_u32_be(header, 12U);
      const std::size_t request_id_size = read_u32_be(header, 16U);
      const std::size_t reason_size = read_u32_be(header, 20U);
      if (request_id_size > 128U ||
        reason_size > config_.limits.maximum_text_bytes ||
        request_id_size + reason_size > config_.limits.maximum_response_bytes)
      {
        throw TransportError{"playback_ack_response_length_invalid"};
      }
      auto body = read_exact(
        descriptor.get(), request_id_size + reason_size, deadline);
      std::size_t offset = 0U;
      PlaybackAckResponse response;
      response.success = status == 0U;
      response.request_id = bytes_to_string(body, offset, request_id_size);
      response.reason = bytes_to_string(body, offset, reason_size);
      const Validation response_validation =
        validate_playback_ack_response(request, response, config_.limits);
      if (!response_validation.accepted) {
        throw TransportError{response_validation.reason};
      }
      finish_success(true);
      return response;
    } catch (const std::exception & exception) {
      finish_failure(exception.what(), true);
      throw;
    }
  }

  void cancel(const std::string & request_id) noexcept
  {
    std::lock_guard<std::mutex> lock{mutex_};
    if (!exchange_active_ || active_fd_ < 0 || active_request_id_ != request_id) {
      return;
    }
    (void)::shutdown(active_fd_, SHUT_RDWR);
    ++cancellations_;
  }

  [[nodiscard]] bool healthy() const noexcept
  {
    std::lock_guard<std::mutex> lock{mutex_};
    return healthy_ ||
           (!operation_attempted_ && safe_socket_path(config_.socket_path));
  }

  [[nodiscard]] UnixSocketSnapshot snapshot() const
  {
    std::lock_guard<std::mutex> lock{mutex_};
    return UnixSocketSnapshot{
      exchange_active_, healthy_, successful_exchanges_, failed_exchanges_,
      successful_playback_acks_, failed_playback_acks_, cancellations_,
      active_request_id_, last_error_};
  }

private:
  void begin_operation(const std::string & request_id)
  {
    std::lock_guard<std::mutex> lock{mutex_};
    if (exchange_active_) {throw TransportError{"savomind_transport_busy"};}
    operation_attempted_ = true;
    exchange_active_ = true;
    active_request_id_ = request_id;
    active_fd_ = -1;
  }

  void set_active_fd(const std::string & request_id, const int fd)
  {
    std::lock_guard<std::mutex> lock{mutex_};
    if (!exchange_active_ || active_request_id_ != request_id) {
      throw TransportError{"savomind_operation_state_changed"};
    }
    active_fd_ = fd;
  }

  void finish_success(const bool playback_ack)
  {
    std::lock_guard<std::mutex> lock{mutex_};
    exchange_active_ = false;
    active_fd_ = -1;
    active_request_id_.clear();
    healthy_ = true;
    last_error_.clear();
    if (playback_ack) {
      ++successful_playback_acks_;
    } else {
      ++successful_exchanges_;
    }
  }

  void finish_failure(const std::string & error, const bool playback_ack)
  {
    std::lock_guard<std::mutex> lock{mutex_};
    exchange_active_ = false;
    active_fd_ = -1;
    active_request_id_.clear();
    healthy_ = false;
    last_error_ = error;
    if (playback_ack) {
      ++failed_playback_acks_;
    } else {
      ++failed_exchanges_;
    }
  }

  UnixSocketConfig config_;
  mutable std::mutex mutex_;
  bool exchange_active_{false};
  bool healthy_{false};
  bool operation_attempted_{false};
  int active_fd_{-1};
  std::uint64_t successful_exchanges_{0U};
  std::uint64_t failed_exchanges_{0U};
  std::uint64_t successful_playback_acks_{0U};
  std::uint64_t failed_playback_acks_{0U};
  std::uint64_t cancellations_{0U};
  std::string active_request_id_;
  std::string last_error_;
};

UnixSocketSavoMindTransport::UnixSocketSavoMindTransport(UnixSocketConfig config)
: impl_{std::make_unique<Impl>(std::move(config))}
{
}

UnixSocketSavoMindTransport::~UnixSocketSavoMindTransport() = default;

Response UnixSocketSavoMindTransport::exchange(const Request & request)
{
  return impl_->exchange(request);
}

PlaybackAckResponse UnixSocketSavoMindTransport::acknowledge_playback(
  const PlaybackAckRequest & request)
{
  return impl_->acknowledge_playback(request);
}

void UnixSocketSavoMindTransport::cancel(const std::string & request_id) noexcept
{
  impl_->cancel(request_id);
}

bool UnixSocketSavoMindTransport::healthy() const noexcept
{
  return impl_->healthy();
}

UnixSocketSnapshot UnixSocketSavoMindTransport::snapshot() const
{
  return impl_->snapshot();
}

}  // namespace savo_speech::transport
