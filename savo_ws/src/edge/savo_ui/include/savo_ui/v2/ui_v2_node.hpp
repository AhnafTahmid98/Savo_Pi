// Copyright 2026 Ahnaf Tahmid
#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "savo_ui/platform/framebuffer_display.hpp"
#include "savo_ui/render/canvas.hpp"
#include "savo_ui/v2/navigation_renderer.hpp"
#include "savo_ui/v2/ui_v2_types.hpp"
#include "savo_ui/v2/voice_face_animation.hpp"
#include "savo_ui/v2/voice_face_renderer.hpp"

namespace savo_ui::v2
{

struct UiV2Config
{
  std::string framebuffer_device{"/dev/fb0"};
  std::string preview_output_dir{"/tmp/savo_ui_v2_preview"};
  std::string page_mode{"auto"};

  std::string speech_state_topic{"/savo_speech/state"};
  std::string speech_dashboard_topic{"/savo_speech/dashboard"};
  std::string playback_state_topic{"/savo_speech/playback/state"};

  std::string navigation_state_topic{"/savo_nav/navigation/state"};
  std::string navigation_status_topic{"/savo_nav/navigation/status"};
  std::string navigation_feedback_topic{"/savo_nav/navigation/feedback"};
  std::string navigation_result_topic{"/savo_nav/navigation/result"};
  std::string navigation_legacy_status_topic{"/savo_nav/status"};

  int screen_width{800};
  int screen_height{480};
  double loop_hz{30.0};
  double arrived_hold_seconds{4.0};
  bool enable_framebuffer{false};
  bool export_preview_frames{true};
};

class UiV2Node final : public rclcpp::Node
{
public:
  explicit UiV2Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declare_parameters();
  void load_parameters();
  void configure_display();
  void configure_subscriptions();
  void configure_timer();

  void on_speech_state(const std::string & text);
  void on_speech_dashboard(const std::string & text);
  void on_playback_state(const std::string & text);
  void on_navigation_state(const std::string & text);
  void on_navigation_status(const std::string & text);
  void on_navigation_feedback(const std::string & text);
  void on_navigation_result(const std::string & text);
  void on_navigation_legacy_status(const std::string & text);

  void loop();
  [[nodiscard]] Page choose_page() const;
  void render(Page page);
  void present();
  void export_preview_set();

  UiV2Config config_{};
  Canvas canvas_{};
  FramebufferDisplay display_{};
  std::unique_ptr<VoiceFaceRenderer> voice_renderer_{};
  std::unique_ptr<NavigationRenderer> navigation_renderer_{};
  VoiceFaceAnimation voice_animation_{};

  VoiceState voice_{};
  NavigationState navigation_{};

  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions_{};
  rclcpp::TimerBase::SharedPtr loop_timer_{};

  std::chrono::steady_clock::time_point last_loop_{};
  std::chrono::steady_clock::time_point navigation_terminal_time_{};
  double animation_time_seconds_{0.0};
  bool preview_exported_{false};
};

}  // namespace savo_ui::v2
