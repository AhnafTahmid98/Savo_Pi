// Copyright 2026 Ahnaf Tahmid
#ifndef SAVO_SPEECH__TRANSPORT__SAVOMIND_TRANSPORT_HPP_
#define SAVO_SPEECH__TRANSPORT__SAVOMIND_TRANSPORT_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace savo_speech::transport
{

inline constexpr std::uint32_t SAVOMIND_SPEECH_PROTOCOL_VERSION = 2U;

struct Request
{
  std::string request_id;
  std::string session_id;
  std::vector<std::uint8_t> wav;
};

struct Response
{
  bool success{true};
  std::string request_id;
  std::string reason;
  std::string transcript;
  std::string reply;
  std::string playback_token;
  bool playback_ack_required{false};
  std::vector<std::uint8_t> tts_wav;
};

struct PlaybackAckRequest
{
  std::string request_id;
  std::string session_id;
  std::string playback_token;
  bool playback_ok{false};
  std::string reason;
};

struct PlaybackAckResponse
{
  bool success{false};
  std::string request_id;
  std::string reason;
};

struct Limits
{
  std::size_t maximum_request_bytes{25U * 1024U * 1024U};
  std::size_t maximum_response_bytes{25U * 1024U * 1024U};
  std::size_t maximum_text_bytes{8192U};
  std::uint32_t required_sample_rate_hz{16000U};
  std::uint16_t required_channels{1U};
  std::uint16_t required_bits_per_sample{16U};
};

struct Validation
{
  bool accepted{false};
  std::string reason;
};

class TransportError : public std::runtime_error
{
public:
  explicit TransportError(const std::string & message)
  : std::runtime_error{message}
  {
  }
};

class SavoMindTransport
{
public:
  virtual ~SavoMindTransport() = default;
  virtual Response exchange(const Request & request) = 0;
  virtual PlaybackAckResponse acknowledge_playback(
    const PlaybackAckRequest & request) = 0;
  virtual void cancel(const std::string & request_id) noexcept = 0;
  [[nodiscard]] virtual bool healthy() const noexcept = 0;
};

struct UnixSocketConfig
{
  std::string socket_path{"/run/savomind/speech.sock"};
  std::chrono::milliseconds connect_timeout{1500};
  std::chrono::milliseconds io_timeout{30000};
  std::optional<std::uint32_t> required_server_uid{};
  Limits limits{};

  [[nodiscard]] bool is_valid() const noexcept;
};

struct UnixSocketSnapshot
{
  bool exchange_active{false};
  bool healthy{false};
  std::uint64_t successful_exchanges{0U};
  std::uint64_t failed_exchanges{0U};
  std::uint64_t successful_playback_acks{0U};
  std::uint64_t failed_playback_acks{0U};
  std::uint64_t cancellations{0U};
  std::string active_request_id;
  std::string last_error;
};

// Bounded, one-frame-per-connection Unix-domain socket client.
//
// Protocol v2 returns an optional playback token with synthesized speech. When
// acknowledgement is required, physical playback completion is reported on a
// second authenticated connection before SavoMind may dispatch pending
// navigation. The wire contract is documented in
// docs/savomind_speech_transport_v2.md.
class UnixSocketSavoMindTransport final : public SavoMindTransport
{
public:
  explicit UnixSocketSavoMindTransport(UnixSocketConfig config);
  ~UnixSocketSavoMindTransport() override;

  UnixSocketSavoMindTransport(const UnixSocketSavoMindTransport &) = delete;
  UnixSocketSavoMindTransport & operator=(const UnixSocketSavoMindTransport &) = delete;
  UnixSocketSavoMindTransport(UnixSocketSavoMindTransport &&) = delete;
  UnixSocketSavoMindTransport & operator=(UnixSocketSavoMindTransport &&) = delete;

  Response exchange(const Request & request) override;
  PlaybackAckResponse acknowledge_playback(
    const PlaybackAckRequest & request) override;
  void cancel(const std::string & request_id) noexcept override;
  [[nodiscard]] bool healthy() const noexcept override;
  [[nodiscard]] UnixSocketSnapshot snapshot() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

[[nodiscard]] Validation validate_request(const Request & request, const Limits & limits);
[[nodiscard]] Validation validate_response(
  const Request & request,
  const Response & response,
  const Limits & limits);
[[nodiscard]] Validation validate_playback_ack_request(
  const PlaybackAckRequest & request,
  const Limits & limits);
[[nodiscard]] Validation validate_playback_ack_response(
  const PlaybackAckRequest & request,
  const PlaybackAckResponse & response,
  const Limits & limits);

}  // namespace savo_speech::transport

#endif  // SAVO_SPEECH__TRANSPORT__SAVOMIND_TRANSPORT_HPP_
