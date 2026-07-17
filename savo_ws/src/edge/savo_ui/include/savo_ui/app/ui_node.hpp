#pragma once

#include "savo_ui/platform/framebuffer_display.hpp"
#include "savo_ui/render/canvas.hpp"
#include "savo_ui/render/image_asset.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <array>
#include <chrono>
#include <deque>
#include <optional>
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

enum class VoicePhase
{
  Idle,
  Listening,
  Thinking,
  Speaking,
  Error
};

enum class NavigationPhase
{
  Idle,
  Preparing,
  Navigating,
  Paused,
  Arrived,
  Error
};

enum class ScreenRequestSource
{
  Touch,
  Speech,
  SavoMind,
  Internal
};

enum class StatusView
{
  Overview,
  Sensors,
  AiLink,
  AlertsSystem
};

struct PowerUiSourceState
{
  bool seen{false};
  bool has_voltage{false};
  bool has_percent{false};

  double voltage_v{0.0};
  double percent{0.0};

  std::string state{"unknown"};
  std::string percent_label{"PERCENT"};

  std::chrono::steady_clock::time_point last_update{};
  std::deque<double> voltage_history{};
};

struct NavigationUiState
{
  bool live{false};
  std::string state{"WAITING"};
  std::string message{"Waiting for navigation"};
  std::string error_message{"Navigation unavailable"};
};


struct VoiceUiState
{
  bool live{false};

  std::string wake_word_state{"MISSING"};
  std::string microphone_state{"MISSING"};
  std::string speech_link_state{"MISSING"};

  std::string stt_route{"MISSING"};
  std::string tts_route{"MISSING"};
  std::string provider{"--"};
  std::string intent{"--"};

  std::string playback_state{"IDLE"};
  std::string speaker_state{"MISSING"};

  std::string transcript{"No transcript"};
  std::string reply{"No reply"};
  std::string error_message{"No voice error"};
  std::string error_detail{"--"};

  double input_level_percent{0.0};
  double elapsed_seconds{0.0};
  double playback_progress{0.0};

  std::array<double, 24> waveform{{
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  }};
};


struct ObstacleDistanceUiState
{
  std::string label;

  bool seen{false};
  bool valid{false};
  bool clear_out_of_range{false};

  double distance_m{0.0};
  std::string state{"MISSING"};
};

struct StatusUiState
{
  std::string robot_state{"WAITING"};
  std::string operating_mode{"UNKNOWN"};
  int alert_count{0};
  bool live{false};

  std::string safety_state{"MISSING"};
  std::string safety_gate{"--"};
  std::string slowdown{"--"};

  std::string control_state{"MISSING"};
  std::string base_state{"--"};
  std::string control_mode{"--"};
  std::string watchdog_state{"--"};

  std::string localization_state{"MISSING"};
  std::string ekf_state{"--"};
  std::string wheel_state{"--"};
  std::string imu_state{"--"};
  std::string vo_state{"--"};

  std::string perception_state{"MISSING"};
  std::string lidar_state{"--"};
  std::string tof_state{"--"};
  std::string ultrasonic_state{"--"};
  std::string depth_state{"--"};

  std::string navigation_state{"MISSING"};
  std::string nav_state{"--"};
  std::string mapping_state{"--"};
  std::string recovery_state{"--"};
  std::string navigation_goal{"None"};

  std::string connectivity_state{"MISSING"};
  std::string savomind_state{"--"};
  std::string speech_state{"--"};
  std::string link_state{"--"};

  // Detailed AI / Link state.
  std::string brain_service_state{"--"};
  std::string intent_engine_state{"--"};
  std::string llm_provider{"--"};
  std::string llm_model{"--"};

  std::string microphone_state{"--"};
  std::string stt_route_state{"--"};
  std::string tts_route_state{"--"};
  std::string playback_state{"--"};

  std::string core_edge_state{"--"};
  std::string ros_discovery_state{"--"};
  std::string internet_state{"--"};

  // Detailed Alerts / System state.
  std::string core_computer_state{"--"};
  std::string edge_computer_state{"--"};
  std::string critical_nodes_state{"--"};

  int ros_nodes_ready{0};
  int ros_nodes_total{0};
  int stale_publishers{0};

  double core_cpu_temp_c{0.0};
  double edge_cpu_temp_c{0.0};

  int core_memory_percent{0};
  int edge_memory_percent{0};
  int core_storage_percent{0};
  int edge_storage_percent{0};

  std::array<ObstacleDistanceUiState, 5> obstacle_distances{{
    {"LiDAR Front", false, false, false, 0.0, "MISSING"},
    {"ToF Left", false, false, false, 0.0, "MISSING"},
    {"ToF Right", false, false, false, 0.0, "MISSING"},
    {"Ultrasonic", false, false, false, 0.0, "MISSING"},
    {"Depth Front", false, false, false, 0.0, "MISSING"},
  }};

  std::string active_alert{"None"};
};

std::string to_string(UiScreen screen);

struct UiNodeConfig
{
  std::string robot_name{"SAVO"};
  std::string framebuffer_device{"/dev/fb0"};
  std::string touch_device{"/dev/input/event1"};
  std::string asset_root{""};
  std::string preview_output_dir{"/tmp/savo_ui_preview"};

  std::string power_core_topic{"/savo_power/core/ups"};
  std::string power_edge_topic{"/savo_power/edge/ups"};
  std::string power_base_topic{"/savo_power/base/battery"};
  std::string power_status_topic{"/savo_power/status"};

  int screen_width{800};
  int screen_height{480};

  double loop_hz{10.0};
  double intro_seconds{4.0};
  double robot360_seconds_per_frame{0.75};
  double power_stale_timeout_s{5.0};

  int preview_animation_frames{24};
  int power_history_samples{120};

  bool enable_framebuffer{false};
  bool enable_touch{false};
  bool export_preview_frames{true};
};

class UiNode final : public rclcpp::Node
{
public:
  explicit UiNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~UiNode() override;

  UiNode(const UiNode &) = delete;
  UiNode & operator=(const UiNode &) = delete;

private:
  void declare_parameters();
  void load_parameters();
  void load_assets();
  bool load_robot360_frames();
  bool load_page_shells();
  bool load_runtime_font_atlases();
  void configure_display();
  void configure_touch();
  void close_touch();
  void configure_runtime();
  void configure_power_subscriptions();

  void update_power_source(
    PowerUiSourceState & source,
    const std::string & text,
    const std::string & source_label,
    const std::string & percent_token,
    const std::string & percent_label);

  void update_power_status(const std::string & text);
  bool power_source_is_stale(const PowerUiSourceState & source) const;

  void configure_home_idle_sequence();
  void export_preview_frames();
  void export_home_animation_preview();

  void update_runtime(double dt_seconds);
  void poll_touch_input();
  void handle_touch_tap(int x, int y);
  bool screen_from_touch(int x, int y, UiScreen * screen) const;
  bool status_view_from_touch(int x, int y, StatusView * view) const;

  void request_status_view(
    StatusView view,
    ScreenRequestSource source);
  void queue_status_view_after_tts(StatusView view);

  void request_screen(UiScreen screen, ScreenRequestSource source);
  void queue_screen_after_tts(UiScreen screen);
  void handle_tts_finished();

  void render_current_screen();
  void render_intro();
  void render_intro_overlay(float progress);
  void render_home();
  void render_page_shell(
    const ImageAsset & shell,
    const std::string & page_name);
  void render_voice_page(VoicePhase phase);
  void seed_voice_preview_data(VoicePhase phase);

  void render_navigation_page(NavigationPhase phase);
  void seed_navigation_preview_data(NavigationPhase phase);

  void render_status_page();
  void seed_status_preview_data();
  void render_power_page();
  void seed_power_preview_data();
  void maybe_export_power_live_preview();
  void render_home_glow();
  void render_home_dashboard();
  void render_home_top_bar();
  void render_home_left_menu();
  void render_home_status_panel();
  void render_placeholder_screen(const std::string & title, const std::string & subtitle);

  void present_if_enabled();

  void transition_to(UiScreen next_screen);
  void loop_callback();

  static double clamp_double(double value, double min_value, double max_value);
  static int clamp_int(int value, int min_value, int max_value);
  static std::string bool_text(bool value);

  UiNodeConfig config_{};

  rclcpp::TimerBase::SharedPtr loop_timer_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    power_core_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    power_edge_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    power_base_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    power_status_subscription_;

  UiScreen active_screen_{UiScreen::Intro};
  VoicePhase voice_phase_{VoicePhase::Idle};

  // Framebuffer-safe Voice animation clocks.
  double voice_animation_time_seconds_{0.0};
  double voice_phase_elapsed_seconds_{0.0};
  VoicePhase rendered_voice_phase_{VoicePhase::Idle};

  NavigationPhase navigation_phase_{NavigationPhase::Idle};
  double navigation_animation_time_seconds_{0.0};
  double navigation_phase_elapsed_seconds_{0.0};
  NavigationPhase rendered_navigation_phase_{NavigationPhase::Idle};

  // Framebuffer-safe Status and Power presentation clocks.
  double status_animation_time_seconds_{0.0};
  double power_animation_time_seconds_{0.0};

  std::optional<UiScreen> pending_screen_;

  StatusView status_view_{StatusView::Overview};
  std::optional<StatusView> pending_status_view_;

  std::string voice_input_source_;
  std::string voice_transcript_;
  std::string voice_response_;
  std::string resolved_intent_;
  std::string navigation_destination_;

  VoiceUiState voice_ui_;
  NavigationUiState navigation_ui_;
  StatusUiState status_ui_;

  PowerUiSourceState core_power_;
  PowerUiSourceState edge_power_;
  PowerUiSourceState base_power_;

  bool power_status_seen_{false};
  std::string power_overall_state_{"unknown"};
  std::string power_health_state_{"UNKNOWN"};

  std::chrono::steady_clock::time_point power_status_last_update_{};
  std::chrono::steady_clock::time_point last_power_preview_export_{};

  bool first_tick_logged_{false};

  ImageAsset boot_intro_;
  ImageAsset voice_shell_;
  ImageAsset navigate_shell_;
  ImageAsset status_shell_;
  ImageAsset power_shell_;

  ImageAsset ui_font_small_;
  ImageAsset ui_font_medium_;
  ImageAsset ui_font_large_bold_;

  std::vector<ImageAsset> robot360_frames_;

  std::vector<std::size_t> home_idle_sequence_;

  Canvas canvas_;
  FramebufferDisplay framebuffer_;

  int touch_fd_{-1};
  bool touch_down_{false};
  int touch_x_{0};
  int touch_y_{0};
  int touch_down_x_{0};
  int touch_down_y_{0};

  std::chrono::steady_clock::time_point start_time_{};
  std::chrono::steady_clock::time_point last_loop_time_{};

  double intro_elapsed_seconds_{0.0};
  double robot360_elapsed_seconds_{0.0};
  double home_glow_elapsed_seconds_{0.0};

  std::size_t home_idle_sequence_index_{0};
};

}  // namespace savo_ui
