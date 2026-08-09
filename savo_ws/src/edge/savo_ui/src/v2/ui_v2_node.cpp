// Copyright 2026 Ahnaf Tahmid
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>

#include "savo_ui/v2/ui_v2_node.hpp"

namespace savo_ui::v2
{
namespace
{

std::string lowercase(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
  return value;
}

bool contains(const std::string & haystack, const std::string & needle)
{
  return lowercase(haystack).find(lowercase(needle)) != std::string::npos;
}

std::string field(const std::string & text, const std::string & key)
{
  const std::string token = key + "=";
  const auto begin = text.find(token);
  if (begin == std::string::npos) {
    return {};
  }
  const auto value_begin = begin + token.size();
  const auto end = text.find(';', value_begin);
  return text.substr(value_begin, end == std::string::npos ? std::string::npos : end - value_begin);
}

double numeric_field(const std::string & text, const std::string & key, const double fallback)
{
  const auto value = field(text, key);
  if (value.empty()) {
    return fallback;
  }
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    return fallback;
  }
}

unsigned int unsigned_field(
  const std::string & text,
  const std::string & key,
  const unsigned int fallback)
{
  const auto value = field(text, key);
  if (value.empty()) {
    return fallback;
  }
  try {
    return static_cast<unsigned int>(std::stoul(value));
  } catch (const std::exception &) {
    return fallback;
  }
}

VoicePhase classify_voice(const std::string & text)
{
  const std::string phase = lowercase(field(text, "phase"));
  const std::string source = phase.empty() ? lowercase(text) : phase;

  if (contains(source, "error") || contains(source, "failed")) {
    return VoicePhase::Error;
  }
  if (contains(source, "wake_word") || contains(source, "listen") || contains(source, "record") ||
    contains(source, "capture"))
  {
    return VoicePhase::Listening;
  }
  if (contains(source, "process") || contains(source, "think") || contains(source, "send") ||
    contains(source, "await") || contains(source, "recover") || contains(source, "cancel"))
  {
    return VoicePhase::Thinking;
  }
  if (contains(source, "speak") || contains(source, "play")) {
    return VoicePhase::Speaking;
  }
  return VoicePhase::Idle;
}

NavigationPhase classify_navigation(const std::string & text)
{
  const std::string raw_state = field(text, "state");
  const std::string state = lowercase(raw_state.empty() ? text : raw_state);

  if (contains(state, "error") || contains(state, "fail") || contains(state, "abort")) {
    return NavigationPhase::Error;
  }
  if (contains(state, "cancel") || contains(state, "pause")) {
    return NavigationPhase::Paused;
  }
  if (contains(state, "active") || contains(state, "navigat")) {
    return NavigationPhase::Navigating;
  }
  if (contains(state, "reserved") || contains(state, "forward") || contains(state, "prepar")) {
    return NavigationPhase::Preparing;
  }
  return NavigationPhase::Idle;
}

bool terminal_hold_active(
  const std::chrono::steady_clock::time_point terminal_time,
  const double hold_seconds)
{
  if (terminal_time.time_since_epoch().count() == 0 || hold_seconds <= 0.0) {
    return false;
  }
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now() - terminal_time).count() < hold_seconds;
}

}  // namespace

UiV2Node::UiV2Node(const rclcpp::NodeOptions & options)
: Node("savo_ui_v2_node", options)
{
  declare_parameters();
  load_parameters();

  canvas_.resize(config_.screen_width, config_.screen_height);
  voice_renderer_ = std::make_unique<VoiceFaceRenderer>(
    config_.screen_width, config_.screen_height);
  navigation_renderer_ = std::make_unique<NavigationRenderer>(
    config_.screen_width, config_.screen_height);

  configure_display();
  configure_subscriptions();
  configure_timer();

  last_loop_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(
    get_logger(),
    "Savo UI v2 ready | %dx%d %.1f Hz mode=%s framebuffer=%s",
    config_.screen_width,
    config_.screen_height,
    config_.loop_hz,
    config_.page_mode.c_str(),
    config_.enable_framebuffer ? "enabled" : "disabled");
}

void UiV2Node::declare_parameters()
{
  declare_parameter<std::string>("framebuffer_device", config_.framebuffer_device);
  declare_parameter<std::string>("preview_output_dir", config_.preview_output_dir);
  declare_parameter<std::string>("page_mode", config_.page_mode);
  declare_parameter<std::string>("speech_state_topic", config_.speech_state_topic);
  declare_parameter<std::string>("speech_dashboard_topic", config_.speech_dashboard_topic);
  declare_parameter<std::string>("playback_state_topic", config_.playback_state_topic);
  declare_parameter<std::string>("navigation_state_topic", config_.navigation_state_topic);
  declare_parameter<std::string>("navigation_status_topic", config_.navigation_status_topic);
  declare_parameter<std::string>("navigation_feedback_topic", config_.navigation_feedback_topic);
  declare_parameter<std::string>("navigation_result_topic", config_.navigation_result_topic);
  declare_parameter<std::string>(
    "navigation_legacy_status_topic", config_.navigation_legacy_status_topic);
  declare_parameter<int>("screen_width", config_.screen_width);
  declare_parameter<int>("screen_height", config_.screen_height);
  declare_parameter<double>("loop_hz", config_.loop_hz);
  declare_parameter<double>("arrived_hold_seconds", config_.arrived_hold_seconds);
  declare_parameter<bool>("enable_framebuffer", config_.enable_framebuffer);
  declare_parameter<bool>("export_preview_frames", config_.export_preview_frames);
}

void UiV2Node::load_parameters()
{
  config_.framebuffer_device = get_parameter("framebuffer_device").as_string();
  config_.preview_output_dir = get_parameter("preview_output_dir").as_string();
  config_.page_mode = lowercase(get_parameter("page_mode").as_string());
  config_.speech_state_topic = get_parameter("speech_state_topic").as_string();
  config_.speech_dashboard_topic = get_parameter("speech_dashboard_topic").as_string();
  config_.playback_state_topic = get_parameter("playback_state_topic").as_string();
  config_.navigation_state_topic = get_parameter("navigation_state_topic").as_string();
  config_.navigation_status_topic = get_parameter("navigation_status_topic").as_string();
  config_.navigation_feedback_topic = get_parameter("navigation_feedback_topic").as_string();
  config_.navigation_result_topic = get_parameter("navigation_result_topic").as_string();
  config_.navigation_legacy_status_topic =
    get_parameter("navigation_legacy_status_topic").as_string();
  config_.screen_width = get_parameter("screen_width").as_int();
  config_.screen_height = get_parameter("screen_height").as_int();
  config_.loop_hz = get_parameter("loop_hz").as_double();
  config_.arrived_hold_seconds = get_parameter("arrived_hold_seconds").as_double();
  config_.enable_framebuffer = get_parameter("enable_framebuffer").as_bool();
  config_.export_preview_frames = get_parameter("export_preview_frames").as_bool();

  if (config_.screen_width <= 0 || config_.screen_height <= 0 || config_.loop_hz <= 0.0) {
    throw std::runtime_error("invalid Savo UI v2 display configuration");
  }
  if (config_.page_mode != "auto" && config_.page_mode != "voice" &&
    config_.page_mode != "navigation")
  {
    throw std::runtime_error("page_mode must be auto, voice, or navigation");
  }
}

void UiV2Node::configure_display()
{
  if (!config_.enable_framebuffer) {
    return;
  }

  std::string error;
  if (!display_.open_device(
      config_.framebuffer_device,
      config_.screen_width,
      config_.screen_height,
      &error))
  {
    throw std::runtime_error("failed to open Savo UI v2 framebuffer: " + error);
  }
}

void UiV2Node::configure_subscriptions()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.best_effort();
  qos.durability_volatile();

  auto add = [this, &qos](const std::string & topic, auto callback) {
      subscriptions_.push_back(create_subscription<std_msgs::msg::String>(
        topic, qos,
          [callback](const std_msgs::msg::String::SharedPtr message) {
            callback(message->data);
        }));
    };

  add(config_.speech_state_topic, [this](const std::string & text) {on_speech_state(text);});
  add(
    config_.speech_dashboard_topic,
    [this](const std::string & text) {on_speech_dashboard(text);});
  add(config_.playback_state_topic, [this](const std::string & text) {on_playback_state(text);});
  add(
    config_.navigation_state_topic,
    [this](const std::string & text) {on_navigation_state(text);});
  add(
    config_.navigation_status_topic,
    [this](const std::string & text) {on_navigation_status(text);});
  add(
    config_.navigation_feedback_topic,
    [this](const std::string & text) {on_navigation_feedback(text);});
  add(
    config_.navigation_result_topic,
    [this](const std::string & text) {on_navigation_result(text);});
  add(config_.navigation_legacy_status_topic,
    [this](const std::string & text) {on_navigation_legacy_status(text);});
}

void UiV2Node::configure_timer()
{
  const auto period = std::chrono::duration<double>(1.0 / config_.loop_hz);
  loop_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&UiV2Node::loop, this));
}

void UiV2Node::on_speech_state(const std::string & text)
{
  voice_.speech_state = text;
  voice_.phase = classify_voice(text);
}

void UiV2Node::on_speech_dashboard(const std::string & text)
{
  const double rms = numeric_field(text, "audio_rms", 0.0);
  // Typical speech RMS is small. This maps 0.00..0.08 into a useful 0..1 display range.
  voice_.input_level = std::clamp(rms / 0.08, 0.0, 1.0);
}

void UiV2Node::on_playback_state(const std::string & text)
{
  voice_.playback_state = text;
  if (contains(text, "playing")) {
    voice_.phase = VoicePhase::Speaking;
  }
}

void UiV2Node::on_navigation_state(const std::string & text)
{
  navigation_.status = text;
  if (!terminal_hold_active(navigation_terminal_time_, config_.arrived_hold_seconds)) {
    navigation_.phase = classify_navigation(text);
  }
}

void UiV2Node::on_navigation_status(const std::string & text)
{
  navigation_.status = text;
  const auto goal = field(text, "goal_id");
  if (!goal.empty()) {
    navigation_.goal_id = goal;
  }
  if (!terminal_hold_active(navigation_terminal_time_, config_.arrived_hold_seconds)) {
    navigation_.phase = classify_navigation(text);
  }
}

void UiV2Node::on_navigation_feedback(const std::string & text)
{
  const auto goal = field(text, "goal_id");
  if (!goal.empty()) {
    navigation_.goal_id = goal;
  }
  navigation_.distance_remaining_m = numeric_field(
    text, "distance_remaining", navigation_.distance_remaining_m);
  navigation_.recoveries = unsigned_field(text, "recoveries", navigation_.recoveries);
  if (
    navigation_.phase == NavigationPhase::Idle ||
    navigation_.phase == NavigationPhase::Preparing)
  {
    navigation_.phase = NavigationPhase::Navigating;
  }
}

void UiV2Node::on_navigation_result(const std::string & text)
{
  navigation_.result = text;
  const auto goal = field(text, "goal_id");
  if (!goal.empty()) {
    navigation_.goal_id = goal;
  }

  if (contains(text, "succeeded")) {
    navigation_.phase = NavigationPhase::Arrived;
  } else if (contains(text, "canceled")) {
    navigation_.phase = NavigationPhase::Paused;
  } else {
    navigation_.phase = NavigationPhase::Error;
  }
  navigation_terminal_time_ = std::chrono::steady_clock::now();
}

void UiV2Node::on_navigation_legacy_status(const std::string & text)
{
  // Read-only fallback for deployments where only /savo_nav/status is present.
  if (navigation_.phase == NavigationPhase::Idle) {
    const auto goal = field(text, "goal_id");
    if (!goal.empty()) {
      navigation_.goal_id = goal;
    }
    const auto phase = classify_navigation(text);
    if (phase != NavigationPhase::Idle) {
      navigation_.phase = phase;
      navigation_.status = text;
    }
  }
}

Page UiV2Node::choose_page() const
{
  if (config_.page_mode == "voice") {
    return Page::Voice;
  }
  if (config_.page_mode == "navigation") {
    return Page::Navigation;
  }

  // Voice interaction has display priority while Savo is actively listening,
  // thinking or speaking. Otherwise an active navigation mission owns the page.
  if (voice_.phase != VoicePhase::Idle) {
    return Page::Voice;
  }

  if (navigation_.phase != NavigationPhase::Idle) {
    const bool terminal_phase =
      navigation_.phase == NavigationPhase::Arrived ||
      navigation_.phase == NavigationPhase::Error ||
      navigation_.phase == NavigationPhase::Paused;
    if (terminal_phase && navigation_terminal_time_.time_since_epoch().count() != 0 &&
      !terminal_hold_active(navigation_terminal_time_, config_.arrived_hold_seconds))
    {
      return Page::Voice;
    }
    return Page::Navigation;
  }

  return Page::Voice;
}

void UiV2Node::loop()
{
  const auto now = std::chrono::steady_clock::now();
  const double dt = std::clamp(std::chrono::duration<double>(now - last_loop_).count(), 0.0, 0.1);
  last_loop_ = now;
  animation_time_seconds_ += dt;

  voice_animation_.update(dt, voice_.phase, voice_.input_level);
  render(choose_page());
  present();

  if (config_.export_preview_frames && !preview_exported_) {
    export_preview_set();
    preview_exported_ = true;
  }
}

void UiV2Node::render(const Page page)
{
  if (page == Page::Voice) {
    voice_renderer_->render(canvas_, voice_, voice_animation_.frame());
  } else {
    navigation_renderer_->render(canvas_, navigation_, animation_time_seconds_);
  }
}

void UiV2Node::present()
{
  if (!config_.enable_framebuffer) {
    return;
  }
  std::string error;
  if (!display_.present_rgb_canvas(canvas_, &error)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "UI v2 framebuffer error: %s", error.c_str());
  }
}

void UiV2Node::export_preview_set()
{
  std::error_code error;
  std::filesystem::create_directories(config_.preview_output_dir, error);
  if (error) {
    RCLCPP_WARN(get_logger(), "cannot create UI v2 preview directory: %s", error.message().c_str());
    return;
  }

  VoiceState saved_voice = voice_;
  NavigationState saved_navigation = navigation_;

  const std::pair<VoicePhase, const char *> voice_cases[] = {
    {VoicePhase::Idle, "voice_idle.ppm"},
    {VoicePhase::Listening, "voice_listening.ppm"},
    {VoicePhase::Thinking, "voice_thinking.ppm"},
    {VoicePhase::Speaking, "voice_speaking.ppm"},
    {VoicePhase::Error, "voice_error.ppm"},
  };

  for (const auto & item : voice_cases) {
    VoiceFaceAnimation animation;
    animation.reset(item.first);
    for (int i = 0; i < 24; ++i) {
      animation.update(1.0 / 30.0, item.first, item.first == VoicePhase::Listening ? 0.70 : 0.0);
    }
    voice_.phase = item.first;
    voice_renderer_->render(canvas_, voice_, animation.frame());
    canvas_.write_ppm(config_.preview_output_dir + "/" + item.second);
  }

  const std::pair<NavigationPhase, const char *> nav_cases[] = {
    {NavigationPhase::Idle, "navigation_idle.ppm"},
    {NavigationPhase::Preparing, "navigation_preparing.ppm"},
    {NavigationPhase::Navigating, "navigation_active.ppm"},
    {NavigationPhase::Paused, "navigation_paused.ppm"},
    {NavigationPhase::Arrived, "navigation_arrived.ppm"},
    {NavigationPhase::Error, "navigation_error.ppm"},
  };

  navigation_.goal_id = "A201";
  navigation_.distance_remaining_m = 12.4;
  for (const auto & item : nav_cases) {
    navigation_.phase = item.first;
    navigation_renderer_->render(canvas_, navigation_, 1.15);
    canvas_.write_ppm(config_.preview_output_dir + "/" + item.second);
  }

  voice_ = saved_voice;
  navigation_ = saved_navigation;
  RCLCPP_INFO(
    get_logger(), "exported Savo UI v2 preview set to %s",
    config_.preview_output_dir.c_str());
}

}  // namespace savo_ui::v2
