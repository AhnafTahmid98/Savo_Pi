// Copyright 2026 Ahnaf Tahmid
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <future>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "savo_speech/transport/savomind_transport.hpp"

namespace
{
using namespace std::chrono_literals;
using savo_speech::transport::PlaybackAckRequest;
using savo_speech::transport::Request;
using savo_speech::transport::UnixSocketConfig;
using savo_speech::transport::UnixSocketSavoMindTransport;

std::vector<std::uint8_t> wav()
{
  std::vector<std::uint8_t> bytes(46U, 0U);
  const std::array<std::uint8_t, 44> header{
    'R', 'I', 'F', 'F', 38, 0, 0, 0, 'W', 'A', 'V', 'E', 'f', 'm', 't', ' ', 16, 0, 0, 0,
    1, 0, 1, 0, 0x80, 0x3e, 0, 0, 0, 0x7d, 0, 0, 2, 0, 16, 0, 'd', 'a', 't', 'a', 2, 0, 0, 0};
  std::copy(header.begin(), header.end(), bytes.begin());
  return bytes;
}

void append_u32(std::vector<std::uint8_t> & out, const std::uint32_t value)
{
  out.push_back(static_cast<std::uint8_t>((value >> 24U) & 0xffU));
  out.push_back(static_cast<std::uint8_t>((value >> 16U) & 0xffU));
  out.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xffU));
  out.push_back(static_cast<std::uint8_t>(value & 0xffU));
}

void append_u64(std::vector<std::uint8_t> & out, const std::uint64_t value)
{
  for (int shift = 56; shift >= 0; shift -= 8) {
    out.push_back(static_cast<std::uint8_t>(
        (value >> static_cast<unsigned int>(shift)) & 0xffU));
  }
}

std::uint32_t read_u32(const std::uint8_t * data, const std::size_t offset)
{
  return (static_cast<std::uint32_t>(data[offset]) << 24U) |
         (static_cast<std::uint32_t>(data[offset + 1U]) << 16U) |
         (static_cast<std::uint32_t>(data[offset + 2U]) << 8U) |
         static_cast<std::uint32_t>(data[offset + 3U]);
}

std::uint64_t read_u64(const std::uint8_t * data, const std::size_t offset)
{
  std::uint64_t value = 0U;
  for (std::size_t index = 0U; index < 8U; ++index) {
    value = (value << 8U) | static_cast<std::uint64_t>(data[offset + index]);
  }
  return value;
}

std::vector<std::uint8_t> receive_all(const int fd, const std::size_t size)
{
  std::vector<std::uint8_t> body(size);
  std::size_t offset = 0U;
  while (offset < body.size()) {
    const auto count = ::recv(
      fd, body.data() + static_cast<std::ptrdiff_t>(offset),
      body.size() - offset, 0);
    EXPECT_GT(count, 0);
    if (count <= 0) {break;}
    offset += static_cast<std::size_t>(count);
  }
  return body;
}

void send_all(const int fd, const std::vector<std::uint8_t> & data)
{
  std::size_t offset = 0U;
  while (offset < data.size()) {
    const auto count = ::send(
      fd, data.data() + static_cast<std::ptrdiff_t>(offset),
      data.size() - offset, MSG_NOSIGNAL);
    if (count <= 0) {return;}
    offset += static_cast<std::size_t>(count);
  }
}

class FakeServer
{
public:
  explicit FakeServer(std::filesystem::path socket_path)
  : socket_path_{std::move(socket_path)}
  {
    listener_ = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    EXPECT_GE(listener_, 0);
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    const auto text = socket_path_.string();
    std::memcpy(address.sun_path, text.c_str(), text.size() + 1U);
    EXPECT_EQ(
      ::bind(
        listener_, reinterpret_cast<const sockaddr *>(&address),
        static_cast<socklen_t>(
          offsetof(sockaddr_un, sun_path) + text.size() + 1U)),
      0);
    EXPECT_EQ(::listen(listener_, 2), 0);
  }

  ~FakeServer()
  {
    if (thread_.joinable()) {thread_.join();}
    if (listener_ >= 0) {(void)::close(listener_);}
    std::error_code ignored;
    std::filesystem::remove(socket_path_, ignored);
  }

  void respond(
    std::string request_id,
    const bool wrong_id = false,
    const std::chrono::milliseconds delay = 0ms,
    const bool require_ack = true)
  {
    thread_ = std::thread([
          this, request_id = std::move(request_id), wrong_id, delay, require_ack]()
        {
          const int client = ::accept4(listener_, nullptr, nullptr, SOCK_CLOEXEC);
          ASSERT_GE(client, 0);
          const auto request_header = receive_all(client, 28U);
          ASSERT_EQ(request_header.size(), 28U);
          ASSERT_TRUE(std::equal(
          request_header.begin(), request_header.begin() + 8,
          std::array<std::uint8_t, 8>{'S', 'A', 'V', 'O', 'S', 'P', 'R', 'Q'}.begin()));
          const std::size_t body_size =
          static_cast<std::size_t>(read_u32(request_header.data(), 12U)) +
          static_cast<std::size_t>(read_u32(request_header.data(), 16U)) +
          static_cast<std::size_t>(read_u64(request_header.data(), 20U));
          (void)receive_all(client, body_size);
          if (delay.count() > 0) {std::this_thread::sleep_for(delay);}
          const std::string response_id = wrong_id ? "wrong" : request_id;
          const std::string transcript = "hello savo";
          const std::string reply = "Hello. How can I help?";
          const std::string reason = "speech_pipeline_completed";
          const std::string token = require_ack ? "playback-token" : "";
          const auto audio = wav();
          std::vector<std::uint8_t> response;
          response.insert(response.end(), {'S', 'A', 'V', 'O', 'S', 'P', 'R', 'S'});
          append_u32(response, 2U);
          append_u32(response, 0U);
          append_u32(response, require_ack ? 1U : 0U);
          append_u32(response, static_cast<std::uint32_t>(response_id.size()));
          append_u32(response, static_cast<std::uint32_t>(reason.size()));
          append_u32(response, static_cast<std::uint32_t>(transcript.size()));
          append_u32(response, static_cast<std::uint32_t>(reply.size()));
          append_u32(response, static_cast<std::uint32_t>(token.size()));
          append_u64(response, audio.size());
          response.insert(response.end(), response_id.begin(), response_id.end());
          response.insert(response.end(), reason.begin(), reason.end());
          response.insert(response.end(), transcript.begin(), transcript.end());
          response.insert(response.end(), reply.begin(), reply.end());
          response.insert(response.end(), token.begin(), token.end());
          response.insert(response.end(), audio.begin(), audio.end());
          send_all(client, response);
          (void)::close(client);

          if (!require_ack || wrong_id || delay.count() > 0) {return;}
          const int ack_client = ::accept4(listener_, nullptr, nullptr, SOCK_CLOEXEC);
          ASSERT_GE(ack_client, 0);
          const auto ack_header = receive_all(ack_client, 32U);
          ASSERT_TRUE(std::equal(
          ack_header.begin(), ack_header.begin() + 8,
          std::array<std::uint8_t, 8>{'S', 'A', 'V', 'O', 'S', 'P', 'A', 'K'}.begin()));
          const std::size_t ack_body_size =
          static_cast<std::size_t>(read_u32(ack_header.data(), 16U)) +
          static_cast<std::size_t>(read_u32(ack_header.data(), 20U)) +
          static_cast<std::size_t>(read_u32(ack_header.data(), 24U)) +
          static_cast<std::size_t>(read_u32(ack_header.data(), 28U));
          (void)receive_all(ack_client, ack_body_size);
          const std::string ack_reason = "navigation_dispatched";
          std::vector<std::uint8_t> ack_response;
          ack_response.insert(
          ack_response.end(), {'S', 'A', 'V', 'O', 'S', 'P', 'A', 'R'});
          append_u32(ack_response, 2U);
          append_u32(ack_response, 0U);
          append_u32(
          ack_response, static_cast<std::uint32_t>(request_id.size()));
          append_u32(
          ack_response, static_cast<std::uint32_t>(ack_reason.size()));
          ack_response.insert(
          ack_response.end(), request_id.begin(), request_id.end());
          ack_response.insert(
          ack_response.end(), ack_reason.begin(), ack_reason.end());
          send_all(ack_client, ack_response);
          (void)::close(ack_client);
      });
  }

private:
  std::filesystem::path socket_path_;
  int listener_{-1};
  std::thread thread_;
};

std::filesystem::path unique_socket()
{
  return std::filesystem::temp_directory_path() /
         ("savo-speech-test-" + std::to_string(::getpid()) + "-" +
         std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()) + ".sock");
}

TEST(UnixSocketSavoMindTransport, ExchangesAndAcknowledgesPhysicalPlayback)
{
  const auto path = unique_socket();
  FakeServer server{path};
  server.respond("request-1");
  UnixSocketConfig config;
  config.socket_path = path.string();
  config.connect_timeout = 1s;
  config.io_timeout = 2s;
  UnixSocketSavoMindTransport transport{config};
  Request request{"request-1", "session-1", wav()};
  const auto response = transport.exchange(request);
  EXPECT_EQ(response.request_id, request.request_id);
  EXPECT_EQ(response.transcript, "hello savo");
  EXPECT_TRUE(response.playback_ack_required);
  EXPECT_EQ(response.playback_token, "playback-token");
  EXPECT_FALSE(response.tts_wav.empty());
  PlaybackAckRequest ack;
  ack.request_id = request.request_id;
  ack.session_id = request.session_id;
  ack.playback_token = response.playback_token;
  ack.playback_ok = true;
  const auto ack_response = transport.acknowledge_playback(ack);
  EXPECT_TRUE(ack_response.success);
  EXPECT_EQ(ack_response.reason, "navigation_dispatched");
  EXPECT_TRUE(transport.healthy());
  const auto snapshot = transport.snapshot();
  EXPECT_EQ(snapshot.successful_exchanges, 1U);
  EXPECT_EQ(snapshot.successful_playback_acks, 1U);
}

TEST(UnixSocketSavoMindTransport, RejectsWrongCorrelation)
{
  const auto path = unique_socket();
  FakeServer server{path};
  server.respond("request-2", true, 0ms, false);
  UnixSocketConfig config;
  config.socket_path = path.string();
  UnixSocketSavoMindTransport transport{config};
  Request request{"request-2", "session-2", wav()};
  EXPECT_THROW(
    transport.exchange(request), savo_speech::transport::TransportError);
  EXPECT_FALSE(transport.healthy());
  EXPECT_EQ(transport.snapshot().failed_exchanges, 1U);
}

TEST(UnixSocketSavoMindTransport, CancellationInterruptsActiveExchange)
{
  const auto path = unique_socket();
  FakeServer server{path};
  server.respond("request-3", false, 2s, false);
  UnixSocketConfig config;
  config.socket_path = path.string();
  config.io_timeout = 5s;
  UnixSocketSavoMindTransport transport{config};
  Request request{"request-3", "session-3", wav()};
  auto future = std::async(std::launch::async, [&]() {
        EXPECT_THROW(
        transport.exchange(request), savo_speech::transport::TransportError);
    });
  for (int attempt = 0;
    attempt < 100 && !transport.snapshot().exchange_active; ++attempt)
  {
    std::this_thread::sleep_for(10ms);
  }
  transport.cancel(request.request_id);
  future.get();
  EXPECT_GE(transport.snapshot().cancellations, 1U);
}

}  // namespace
