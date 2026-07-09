#pragma once

#include "savo_ui/platform/framebuffer_display.hpp"
#include "savo_ui/render/canvas.hpp"
#include "savo_ui/render/image_asset.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <string>
#include <vector>

namespace savo_ui
{

enum class UiScreen
{
  Intro,
  Home,
  Listening,
  Thinking,
  Speaking,
  VoiceIdle,
  Navigation,
  Map,
  Status,
  Diagnostics,
  Power,
  SafetyStop
};

std::string to_string(UiScreen screen);

struct UiNodeConfig
{
  std::string robot_name{"SAVO"};
  std::string framebuffer_device{"/dev/fb0"};
  std::string asset_root{""};
  std::string preview_output_dir{"/tmp/savo_ui_preview"};

  int screen_width{800};
  int screen_height{480};

  double loop_hz{10.0};
  double intro_seconds{4.0};
  double robot360_seconds_per_frame{0.75};

  int preview_animation_frames{24};

  bool enable_framebuffer{false};
  bool enable_touch{false};
  bool export_preview_frames{true};
};

class UiNode final : public rclcpp::Node
{
public:
  explicit UiNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~UiNode() override = default;

  UiNode(const UiNode &) = delete;
  UiNode & operator=(const UiNode &) = delete;

private:
  void declare_parameters();
  void load_parameters();
  void load_assets();
  bool load_robot360_frames();
  void configure_display();
  void configure_runtime();

  void configure_home_idle_sequence();
  void export_preview_frames();
  void export_home_animation_preview();

  void update_runtime(double dt_seconds);
  void render_current_screen();
  void render_intro();
  void render_intro_overlay(float progress);
  void render_home();
  void render_home_glow();
  void render_home_dashboard();
  void render_home_top_bar();
  void render_home_left_menu();
  void render_home_status_panel();

  void present_if_enabled();

  void transition_to(UiScreen next_screen);
  void loop_callback();

  static double clamp_double(double value, double min_value, double max_value);
  static int clamp_int(int value, int min_value, int max_value);
  static std::string bool_text(bool value);

  UiNodeConfig config_{};

  rclcpp::TimerBase::SharedPtr loop_timer_;

  UiScreen active_screen_{UiScreen::Intro};
  bool first_tick_logged_{false};

  ImageAsset boot_intro_;
  std::vector<ImageAsset> robot360_frames_;

  std::vector<std::size_t> home_idle_sequence_;

  Canvas canvas_;
  FramebufferDisplay framebuffer_;

  std::chrono::steady_clock::time_point start_time_{};
  std::chrono::steady_clock::time_point last_loop_time_{};

  double intro_elapsed_seconds_{0.0};
  double robot360_elapsed_seconds_{0.0};
  double home_glow_elapsed_seconds_{0.0};

  std::size_t home_idle_sequence_index_{0};
};

}  // namespace savo_ui
