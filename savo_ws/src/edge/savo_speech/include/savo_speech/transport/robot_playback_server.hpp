// Copyright 2026 Ahnaf Tahmid
#ifndef SAVO_SPEECH__TRANSPORT__ROBOT_PLAYBACK_SERVER_HPP_
#define SAVO_SPEECH__TRANSPORT__ROBOT_PLAYBACK_SERVER_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "savo_speech/audio/audio_runtime.hpp"

namespace savo_speech::transport
{

struct RobotPlaybackServerConfig
{
  std::string socket_path{"/run/savo_speech/playback.sock"};
  std::size_t maximum_wav_bytes{16U * 1024U * 1024U};
  bool require_peer_uid{false};
  std::uint32_t peer_uid{0U};

  [[nodiscard]] bool is_valid() const noexcept;
};

class RobotPlaybackServer final
{
public:
  RobotPlaybackServer(
    audio::AudioRuntime & audio_runtime,
    RobotPlaybackServerConfig config = RobotPlaybackServerConfig{});
  ~RobotPlaybackServer();

  RobotPlaybackServer(const RobotPlaybackServer &) = delete;
  RobotPlaybackServer & operator=(const RobotPlaybackServer &) = delete;

  [[nodiscard]] bool start();
  void stop() noexcept;
  [[nodiscard]] bool running() const noexcept;
  [[nodiscard]] std::string last_error() const;

private:
  void run() noexcept;
  void handle_client(int client_fd) noexcept;
  void set_error(std::string error);

  audio::AudioRuntime & audio_runtime_;
  RobotPlaybackServerConfig config_;
  mutable std::mutex mutex_;
  std::jthread thread_;
  std::atomic<int> listener_fd_{-1};
  std::atomic<int> active_client_fd_{-1};
  std::atomic<bool> running_{false};
  std::string last_error_{};
};

}  // namespace savo_speech::transport

#endif  // SAVO_SPEECH__TRANSPORT__ROBOT_PLAYBACK_SERVER_HPP_
