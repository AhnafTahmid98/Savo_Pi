#include "savo_ui/app/ui_node.hpp"
#include "savo_ui/render/preview_writer.hpp"
#include "savo_ui/render/font.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <linux/input.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>



namespace
{

std::string current_local_time_text()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);

  std::tm local_tm{};
  localtime_r(&now_time, &local_tm);

  std::ostringstream out;
  out << std::put_time(&local_tm, "%H:%M");
  return out.str();
}

bool is_usable_ipv4(const std::string & ip)
{
  return !ip.empty() && ip != "127.0.0.1";
}

std::string get_ipv4_for_interface(const std::string & wanted_interface)
{
  ifaddrs * interfaces = nullptr;

  if (getifaddrs(&interfaces) != 0 || interfaces == nullptr) {
    return "--";
  }

  std::string result{"--"};

  for (const ifaddrs * ifa = interfaces; ifa != nullptr; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr == nullptr || ifa->ifa_name == nullptr) {
      continue;
    }

    if (wanted_interface != ifa->ifa_name) {
      continue;
    }

    if (ifa->ifa_addr->sa_family != AF_INET) {
      continue;
    }

    char buffer[INET_ADDRSTRLEN]{};
    const auto * addr = reinterpret_cast<const sockaddr_in *>(ifa->ifa_addr);

    if (inet_ntop(AF_INET, &addr->sin_addr, buffer, sizeof(buffer)) != nullptr) {
      const std::string ip{buffer};
      if (is_usable_ipv4(ip)) {
        result = ip;
        break;
      }
    }
  }

  freeifaddrs(interfaces);
  return result;
}

std::string current_ip_text()
{
  // UI runs on savo-edge. Prefer the validated direct robot Ethernet IP.
  const auto eth0 = get_ipv4_for_interface("eth0");

  if (eth0.rfind("192.168.50.", 0) == 0) {
    return eth0;
  }

  // PC dryrun or temporarily disconnected edge fallback.
  return "192.168.50.2";
}

void draw_live_top_bar_overlay(savo_ui::Canvas & canvas)
{
  const std::string core_ip{"192.168.50.1"};
  const std::string edge_ip = current_ip_text();
  const std::string time_text = current_local_time_text();

  constexpr int kScale = 1;

  // Fixed two-row IP area between the generated Wi-Fi icon and separator.
  constexpr int kLabelX = 620;
  constexpr int kIpX = 650;
  constexpr int kCoreY = 15;
  constexpr int kEdgeY = 29;

  // CORE row.
  savo_ui::Font::draw_text(
    canvas,
    kLabelX,
    kCoreY,
    "CORE",
    savo_ui::ColorRgb{0U, 220U, 255U},
    kScale,
    0.95F);

  savo_ui::Font::draw_text(
    canvas,
    kIpX,
    kCoreY,
    core_ip,
    savo_ui::ColorRgb{235U, 248U, 255U},
    kScale,
    0.95F);

  // EDGE row.
  savo_ui::Font::draw_text(
    canvas,
    kLabelX,
    kEdgeY,
    "EDGE",
    savo_ui::ColorRgb{0U, 220U, 255U},
    kScale,
    0.95F);

  savo_ui::Font::draw_text(
    canvas,
    kIpX,
    kEdgeY,
    edge_ip,
    savo_ui::ColorRgb{235U, 248U, 255U},
    kScale,
    0.95F);

  // Right-align time with clear space after the generated separator at x=740.
  constexpr int kTimeRight = 790;
  constexpr int kTimeY = 23;

  const int time_x =
    kTimeRight - savo_ui::Font::text_width(time_text, kScale);

  savo_ui::Font::draw_text(
    canvas,
    time_x,
    kTimeY,
    time_text,
    savo_ui::ColorRgb{235U, 248U, 255U},
    kScale,
    0.95F);
}


}  // namespace


namespace savo_ui
{
namespace
{

std::string join_path(const std::string & left, const std::string & right)
{
  return (std::filesystem::path(left) / right).string();
}

std::string trim_copy(const std::string & text)
{
  const auto first = text.find_first_not_of(" \t\r\n");

  if (first == std::string::npos) {
    return "";
  }

  const auto last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1U);
}

std::string lowercase_copy(std::string text)
{
  std::transform(
    text.begin(),
    text.end(),
    text.begin(),
    [](const unsigned char value) {
      return static_cast<char>(std::tolower(value));
    });

  return text;
}

std::string uppercase_copy(std::string text)
{
  std::transform(
    text.begin(),
    text.end(),
    text.begin(),
    [](const unsigned char value) {
      return static_cast<char>(std::toupper(value));
    });

  return text;
}

bool extract_number_before(
  const std::string & text,
  const std::string & marker,
  double * value)
{
  if (value == nullptr) {
    return false;
  }

  const auto marker_position = text.find(marker);

  if (marker_position == std::string::npos) {
    return false;
  }

  std::size_t end = marker_position;

  while (
    end > 0U &&
    std::isspace(static_cast<unsigned char>(text[end - 1U])) != 0)
  {
    --end;
  }

  std::size_t start = end;

  while (start > 0U) {
    const char character = text[start - 1U];

    if (
      std::isdigit(static_cast<unsigned char>(character)) == 0 &&
      character != '.' &&
      character != '-' &&
      character != '+')
    {
      break;
    }

    --start;
  }

  if (start == end) {
    return false;
  }

  try {
    *value = std::stod(text.substr(start, end - start));
    return std::isfinite(*value);
  } catch (const std::exception &) {
    return false;
  }
}

bool extract_number_after(
  const std::string & text,
  const std::string & marker,
  double * value)
{
  if (value == nullptr) {
    return false;
  }

  const auto marker_position = text.find(marker);

  if (marker_position == std::string::npos) {
    return false;
  }

  std::size_t start = marker_position + marker.size();

  while (
    start < text.size() &&
    std::isspace(static_cast<unsigned char>(text[start])) != 0)
  {
    ++start;
  }

  std::size_t end = start;

  while (end < text.size()) {
    const char character = text[end];

    if (
      std::isdigit(static_cast<unsigned char>(character)) == 0 &&
      character != '.' &&
      character != '-' &&
      character != '+')
    {
      break;
    }

    ++end;
  }

  if (start == end) {
    return false;
  }

  try {
    *value = std::stod(text.substr(start, end - start));
    return std::isfinite(*value);
  } catch (const std::exception &) {
    return false;
  }
}

std::string extract_value_after_key(
  const std::string & text,
  const std::string & key)
{
  const auto key_position = text.find(key);

  if (key_position == std::string::npos) {
    return "";
  }

  const std::size_t start = key_position + key.size();
  const auto end = text.find_first_of(" \t\r\n", start);

  return text.substr(
    start,
    end == std::string::npos ? std::string::npos : end - start);
}

}  // namespace

std::string to_string(const UiScreen screen)
{
  switch (screen) {
    case UiScreen::Intro:
      return "Intro";
    case UiScreen::Home:
      return "Home";
    case UiScreen::Listening:
      return "Listening";
    case UiScreen::Thinking:
      return "Thinking";
    case UiScreen::Speaking:
      return "Speaking";
    case UiScreen::VoiceIdle:
      return "VoiceIdle";
    case UiScreen::Navigation:
      return "Navigation";
    case UiScreen::Map:
      return "Map";
    case UiScreen::Status:
      return "Status";
    case UiScreen::Diagnostics:
      return "Diagnostics";
    case UiScreen::Power:
      return "Power";
    case UiScreen::SafetyStop:
      return "SafetyStop";
  }

  return "Unknown";
}

UiNode::UiNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("savo_ui_node", options)
{
  declare_parameters();
  load_parameters();
  load_assets();
  configure_home_idle_sequence();

  canvas_.resize(config_.screen_width, config_.screen_height);
  if (!canvas_.valid()) {
    throw std::runtime_error("failed to create UI canvas");
  }

  RCLCPP_INFO(
    get_logger(),
    "canvas created | size=%dx%d rgb_bytes=%zu",
    canvas_.width(),
    canvas_.height(),
    canvas_.pixels_rgb().size());

  start_time_ = std::chrono::steady_clock::now();
  last_loop_time_ = start_time_;

  render_current_screen();
  export_preview_frames();

  configure_display();
  configure_touch();
  configure_power_subscriptions();
  present_if_enabled();

  configure_runtime();

  RCLCPP_INFO(
    get_logger(),
    "savo_ui C++ started | robot=%s screen=%dx%d loop_hz=%.1f framebuffer=%s fb_device=%s touch=%s",
    config_.robot_name.c_str(),
    config_.screen_width,
    config_.screen_height,
    config_.loop_hz,
    bool_text(config_.enable_framebuffer).c_str(),
    config_.framebuffer_device.c_str(),
    bool_text(config_.enable_touch).c_str());

  RCLCPP_INFO(
    get_logger(),
    "initial screen=%s intro_seconds=%.2f home_idle_frame_s=%.2f",
    to_string(active_screen_).c_str(),
    config_.intro_seconds,
    config_.robot360_seconds_per_frame);
}

UiNode::~UiNode()
{
  close_touch();
}

void UiNode::declare_parameters()
{
  declare_parameter<std::string>("robot_name", config_.robot_name);
  declare_parameter<std::string>("framebuffer_device", config_.framebuffer_device);
  declare_parameter<std::string>("touch_device", config_.touch_device);
  declare_parameter<std::string>("asset_root", config_.asset_root);
  declare_parameter<std::string>("preview_output_dir", config_.preview_output_dir);

  declare_parameter<std::string>(
    "power_core_topic",
    config_.power_core_topic);
  declare_parameter<std::string>(
    "power_edge_topic",
    config_.power_edge_topic);
  declare_parameter<std::string>(
    "power_base_topic",
    config_.power_base_topic);
  declare_parameter<std::string>(
    "power_status_topic",
    config_.power_status_topic);

  declare_parameter<int>("screen_width", config_.screen_width);
  declare_parameter<int>("screen_height", config_.screen_height);

  declare_parameter<double>("loop_hz", config_.loop_hz);
  declare_parameter<double>("intro_seconds", config_.intro_seconds);
  declare_parameter<double>(
    "robot360_seconds_per_frame",
    config_.robot360_seconds_per_frame);

  declare_parameter<int>("preview_animation_frames", config_.preview_animation_frames);
  declare_parameter<int>("power_history_samples", config_.power_history_samples);

  declare_parameter<double>(
    "power_stale_timeout_s",
    config_.power_stale_timeout_s);

  declare_parameter<bool>("enable_framebuffer", config_.enable_framebuffer);
  declare_parameter<bool>("enable_touch", config_.enable_touch);
  declare_parameter<bool>("export_preview_frames", config_.export_preview_frames);
}

void UiNode::load_parameters()
{
  config_.robot_name = get_parameter("robot_name").as_string();
  config_.framebuffer_device = get_parameter("framebuffer_device").as_string();
  config_.touch_device = get_parameter("touch_device").as_string();
  config_.asset_root = get_parameter("asset_root").as_string();
  config_.preview_output_dir = get_parameter("preview_output_dir").as_string();

  config_.power_core_topic =
    get_parameter("power_core_topic").as_string();
  config_.power_edge_topic =
    get_parameter("power_edge_topic").as_string();
  config_.power_base_topic =
    get_parameter("power_base_topic").as_string();
  config_.power_status_topic =
    get_parameter("power_status_topic").as_string();

  config_.screen_width = clamp_int(get_parameter("screen_width").as_int(), 320, 3840);
  config_.screen_height = clamp_int(get_parameter("screen_height").as_int(), 240, 2160);

  config_.loop_hz = clamp_double(get_parameter("loop_hz").as_double(), 1.0, 60.0);
  config_.intro_seconds = clamp_double(get_parameter("intro_seconds").as_double(), 1.0, 20.0);
  config_.robot360_seconds_per_frame =
    clamp_double(get_parameter("robot360_seconds_per_frame").as_double(), 0.1, 5.0);

  config_.preview_animation_frames =
    clamp_int(get_parameter("preview_animation_frames").as_int(), 4, 120);

  config_.power_history_samples =
    clamp_int(get_parameter("power_history_samples").as_int(), 10, 600);

  config_.power_stale_timeout_s =
    clamp_double(
      get_parameter("power_stale_timeout_s").as_double(),
      1.0,
      60.0);

  config_.enable_framebuffer = get_parameter("enable_framebuffer").as_bool();
  config_.enable_touch = get_parameter("enable_touch").as_bool();
  config_.export_preview_frames = get_parameter("export_preview_frames").as_bool();

  if (config_.framebuffer_device.empty()) {
    throw std::runtime_error("framebuffer_device parameter cannot be empty");
  }

  if (config_.touch_device.empty()) {
    throw std::runtime_error("touch_device parameter cannot be empty");
  }

  if (config_.asset_root.empty()) {
    throw std::runtime_error("asset_root parameter cannot be empty");
  }

  if (config_.preview_output_dir.empty()) {
    throw std::runtime_error("preview_output_dir parameter cannot be empty");
  }

  if (
    config_.power_core_topic.empty() ||
    config_.power_edge_topic.empty() ||
    config_.power_base_topic.empty() ||
    config_.power_status_topic.empty())
  {
    throw std::runtime_error("power topic parameters cannot be empty");
  }
}

void UiNode::load_assets()
{
  const auto boot_intro_path = join_path(config_.asset_root, "images/boot_intro.ppm");

  std::string error_message;
  if (!boot_intro_.load_ppm(boot_intro_path, &error_message)) {
    throw std::runtime_error(error_message);
  }

  RCLCPP_INFO(
    get_logger(),
    "loaded boot intro image | path=%s size=%dx%d",
    boot_intro_path.c_str(),
    boot_intro_.width(),
    boot_intro_.height());

  if (!load_robot360_frames()) {
    throw std::runtime_error("failed to load robot360 frames");
  }

  RCLCPP_INFO(
    get_logger(),
    "loaded generated home frames | count=%zu",
    robot360_frames_.size());

  if (!load_page_shells()) {
    throw std::runtime_error("failed to load generated page shells");
  }

  RCLCPP_INFO(
    get_logger(),
    "loaded generated page shells | count=4");

  if (!load_runtime_font_atlases()) {
    throw std::runtime_error("failed to load runtime font atlases");
  }

  RCLCPP_INFO(
    get_logger(),
    "loaded runtime font atlases | count=3");
}

bool UiNode::load_robot360_frames()
{
  robot360_frames_.clear();

  // Runtime homepage frames are generated ahead of time by:
  //   scripts/generate_home_assets.py
  //
  // They are full 800x480 PPM frames, ready for direct framebuffer display.
  // This keeps the Pi runtime C++ only and avoids live PNG/Pillow rendering.
  const std::vector<std::string> frame_names{
    "home_idle_000.ppm",
    "home_idle_001.ppm",
    "home_idle_002.ppm",
    "home_idle_003.ppm",
    "home_idle_004.ppm"
  };

  for (const auto & frame_name : frame_names) {
    const auto frame_path = join_path(config_.asset_root, "images/generated/" + frame_name);

    ImageAsset frame;
    std::string error_message;

    if (!frame.load_ppm(frame_path, &error_message)) {
      RCLCPP_ERROR(get_logger(), "%s", error_message.c_str());
      return false;
    }

    if (frame.width() != config_.screen_width || frame.height() != config_.screen_height) {
      RCLCPP_ERROR(
        get_logger(),
        "generated home frame has wrong size | %s size=%dx%d expected=%dx%d",
        frame_name.c_str(),
        frame.width(),
        frame.height(),
        config_.screen_width,
        config_.screen_height);
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "loaded generated home frame | %s size=%dx%d",
      frame_name.c_str(),
      frame.width(),
      frame.height());

    robot360_frames_.push_back(std::move(frame));
  }

  return robot360_frames_.size() == frame_names.size();
}



bool UiNode::load_page_shells()
{
  auto load_shell =
    [this](
      const std::string & file_name,
      const std::string & page_name,
      ImageAsset * shell) -> bool
    {
      if (shell == nullptr) {
        RCLCPP_ERROR(
          get_logger(),
          "page shell destination is null | page=%s",
          page_name.c_str());
        return false;
      }

      const auto path =
        join_path(config_.asset_root, "images/generated/" + file_name);

      std::string error_message;
      if (!shell->load_ppm(path, &error_message)) {
        RCLCPP_ERROR(
          get_logger(),
          "failed to load page shell | page=%s error=%s",
          page_name.c_str(),
          error_message.c_str());
        return false;
      }

      if (
        shell->width() != config_.screen_width ||
        shell->height() != config_.screen_height)
      {
        RCLCPP_ERROR(
          get_logger(),
          "page shell has wrong size | page=%s size=%dx%d expected=%dx%d",
          page_name.c_str(),
          shell->width(),
          shell->height(),
          config_.screen_width,
          config_.screen_height);
        return false;
      }

      RCLCPP_INFO(
        get_logger(),
        "loaded generated page shell | page=%s file=%s size=%dx%d",
        page_name.c_str(),
        file_name.c_str(),
        shell->width(),
        shell->height());

      return true;
    };

  return
    load_shell("voice_shell.ppm", "Voice", &voice_shell_) &&
    load_shell("navigate_shell.ppm", "Navigate", &navigate_shell_) &&
    load_shell("status_shell.ppm", "Status", &status_shell_) &&
    load_shell("power_shell.ppm", "Power", &power_shell_);
}



bool UiNode::load_runtime_font_atlases()
{
  auto load_font =
    [this](
      const std::string & file_name,
      const int expected_width,
      const int expected_height,
      ImageAsset * destination) -> bool
    {
      if (destination == nullptr) {
        return false;
      }

      const std::string path =
        join_path(
        config_.asset_root,
        "images/generated/" + file_name);

      std::string error_message;

      if (!destination->load_ppm(path, &error_message)) {
        RCLCPP_ERROR(
          get_logger(),
          "failed to load runtime font atlas | file=%s error=%s",
          file_name.c_str(),
          error_message.c_str());
        return false;
      }

      if (
        destination->width() != expected_width ||
        destination->height() != expected_height)
      {
        RCLCPP_ERROR(
          get_logger(),
          "runtime font atlas has wrong size | file=%s "
          "size=%dx%d expected=%dx%d",
          file_name.c_str(),
          destination->width(),
          destination->height(),
          expected_width,
          expected_height);
        return false;
      }

      RCLCPP_INFO(
        get_logger(),
        "loaded runtime font atlas | file=%s size=%dx%d",
        file_name.c_str(),
        destination->width(),
        destination->height());

      return true;
    };

  return
    load_font(
      "ui_font_small.ppm",
      288,
      132,
      &ui_font_small_) &&
    load_font(
      "ui_font_medium.ppm",
      320,
      144,
      &ui_font_medium_) &&
    load_font(
      "ui_font_large_bold.ppm",
      512,
      216,
      &ui_font_large_bold_);
}


void UiNode::configure_home_idle_sequence()
{
  home_idle_sequence_.clear();

  if (robot360_frames_.empty()) {
    RCLCPP_WARN(get_logger(), "generated home frame count is zero");
    return;
  }

  for (std::size_t i = 0; i < robot360_frames_.size(); ++i) {
    home_idle_sequence_.push_back(i);
  }

  RCLCPP_INFO(
    get_logger(),
    "home idle generated-frame sequence configured | frames=%zu frame_seconds=%.2f",
    home_idle_sequence_.size(),
    config_.robot360_seconds_per_frame);
}


void UiNode::export_preview_frames()
{
  if (!config_.export_preview_frames) {
    RCLCPP_INFO(
      get_logger(),
      "preview frame export disabled by config");
    return;
  }

  try {
    std::filesystem::create_directories(
      config_.preview_output_dir);
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      get_logger(),
      "failed to create preview directory | dir=%s error=%s",
      config_.preview_output_dir.c_str(),
      error.what());
    return;
  }

  // ============================================================
  // Home previews
  // ============================================================
  for (
    std::size_t index = 0;
    index < robot360_frames_.size();
    ++index)
  {
    canvas_.clear(ColorRgb{2U, 8U, 22U});

    if (!canvas_.draw_image_fit(robot360_frames_[index])) {
      RCLCPP_WARN(
        get_logger(),
        "failed to draw preview Home frame %zu",
        index);
      continue;
    }

    draw_live_top_bar_overlay(canvas_);

    const std::string output_path =
      config_.preview_output_dir +
      "/home_live_" +
      std::to_string(static_cast<int>(index)) +
      ".ppm";

    if (!canvas_.write_ppm(output_path)) {
      RCLCPP_WARN(
        get_logger(),
        "failed to write Home preview | %s",
        output_path.c_str());
      continue;
    }

    RCLCPP_INFO(
      get_logger(),
      "saved Home preview | %s",
      output_path.c_str());
  }

  // ============================================================
  // Voice static previews
  // ============================================================
  const auto saved_voice_ui =
    voice_ui_;

  const auto saved_voice_phase =
    voice_phase_;

  const double saved_voice_animation_time =
    voice_animation_time_seconds_;

  const double saved_voice_phase_elapsed =
    voice_phase_elapsed_seconds_;

  const auto saved_rendered_voice_phase =
    rendered_voice_phase_;

  auto export_voice_phase =
    [this](
      const VoicePhase phase,
      const std::string & file_name)
    {
      seed_voice_preview_data(phase);
      render_voice_page(phase);

      const std::string output_path =
        config_.preview_output_dir +
        "/" +
        file_name;

      if (!canvas_.write_ppm(output_path)) {
        RCLCPP_WARN(
          get_logger(),
          "failed to write Voice state preview | %s",
          output_path.c_str());
        return;
      }

      RCLCPP_INFO(
        get_logger(),
        "saved Voice state preview | %s",
        output_path.c_str());
    };

  export_voice_phase(
    VoicePhase::Idle,
    "voice_preview.ppm");

  export_voice_phase(
    VoicePhase::Idle,
    "voice_idle.ppm");

  export_voice_phase(
    VoicePhase::Listening,
    "voice_listening.ppm");

  export_voice_phase(
    VoicePhase::Thinking,
    "voice_thinking.ppm");

  export_voice_phase(
    VoicePhase::Speaking,
    "voice_speaking.ppm");

  export_voice_phase(
    VoicePhase::Error,
    "voice_error.ppm");

  // ============================================================
  // Voice animated preview
  // ============================================================
  const std::string voice_animation_directory =
    config_.preview_output_dir +
    "/voice_animation";

  try {
    std::filesystem::create_directories(
      voice_animation_directory);
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      get_logger(),
      "failed to create Voice animation directory | %s",
      error.what());
  }

  const std::array<VoicePhase, 5> voice_phases{{
    VoicePhase::Idle,
    VoicePhase::Listening,
    VoicePhase::Thinking,
    VoicePhase::Speaking,
    VoicePhase::Error,
  }};

  constexpr int voice_frames_per_phase = 24;
  constexpr double voice_preview_fps = 12.0;

  int global_voice_frame = 0;

  for (const auto phase : voice_phases) {
    for (
      int frame = 0;
      frame < voice_frames_per_phase;
      ++frame)
    {
      seed_voice_preview_data(phase);

      voice_animation_time_seconds_ =
        static_cast<double>(global_voice_frame) /
        voice_preview_fps;

      voice_phase_elapsed_seconds_ =
        static_cast<double>(frame) /
        voice_preview_fps;

      rendered_voice_phase_ = phase;

      if (phase == VoicePhase::Speaking) {
        voice_ui_.playback_progress =
          static_cast<double>(frame) /
          static_cast<double>(
            voice_frames_per_phase - 1);
      }

      render_voice_page(phase);

      std::ostringstream frame_path;
      frame_path
        << voice_animation_directory
        << "/frame_"
        << std::setw(3)
        << std::setfill('0')
        << global_voice_frame
        << ".ppm";

      if (!canvas_.write_ppm(frame_path.str())) {
        RCLCPP_WARN(
          get_logger(),
          "failed to write Voice animation frame | %s",
          frame_path.str().c_str());
      }

      ++global_voice_frame;
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "saved Voice animation preview | dir=%s frames=%d",
    voice_animation_directory.c_str(),
    global_voice_frame);

  voice_ui_ =
    saved_voice_ui;

  voice_phase_ =
    saved_voice_phase;

  voice_animation_time_seconds_ =
    saved_voice_animation_time;

  voice_phase_elapsed_seconds_ =
    saved_voice_phase_elapsed;

  rendered_voice_phase_ =
    saved_rendered_voice_phase;

  // ============================================================
  // Navigation static previews
  // ============================================================
  const auto saved_navigation_ui =
    navigation_ui_;

  const auto saved_navigation_phase =
    navigation_phase_;

  const double saved_navigation_animation_time =
    navigation_animation_time_seconds_;

  const double saved_navigation_phase_elapsed =
    navigation_phase_elapsed_seconds_;

  const auto saved_rendered_navigation_phase =
    rendered_navigation_phase_;

  auto export_navigation_phase =
    [this](
      const NavigationPhase phase,
      const std::string & file_name)
    {
      seed_navigation_preview_data(phase);
      render_navigation_page(phase);

      const std::string output_path =
        config_.preview_output_dir +
        "/" +
        file_name;

      if (!canvas_.write_ppm(output_path)) {
        RCLCPP_WARN(
          get_logger(),
          "failed to write Navigation state preview | %s",
          output_path.c_str());
        return;
      }

      RCLCPP_INFO(
        get_logger(),
        "saved Navigation state preview | %s",
        output_path.c_str());
    };

  export_navigation_phase(
    NavigationPhase::Idle,
    "navigate_preview.ppm");

  export_navigation_phase(
    NavigationPhase::Idle,
    "navigation_idle.ppm");

  export_navigation_phase(
    NavigationPhase::Preparing,
    "navigation_preparing.ppm");

  export_navigation_phase(
    NavigationPhase::Navigating,
    "navigation_navigating.ppm");

  export_navigation_phase(
    NavigationPhase::Paused,
    "navigation_paused.ppm");

  export_navigation_phase(
    NavigationPhase::Arrived,
    "navigation_arrived.ppm");

  export_navigation_phase(
    NavigationPhase::Error,
    "navigation_error.ppm");

  // ============================================================
  // Navigation animations for all six states
  // ============================================================
  const std::string navigation_animation_root =
    config_.preview_output_dir +
    "/navigation_animation";

  struct NavigationAnimationSpec
  {
    NavigationPhase phase;
    const char * directory;
    double time_offset_seconds;
  };

  const std::array<NavigationAnimationSpec, 6>
  navigation_animation_specs{{
    {
      NavigationPhase::Idle,
      "idle",
      3.0
    },
    {
      NavigationPhase::Preparing,
      "preparing",
      0.0
    },
    {
      NavigationPhase::Navigating,
      "navigating",
      2.3
    },
    {
      NavigationPhase::Paused,
      "paused",
      0.0
    },
    {
      NavigationPhase::Arrived,
      "arrived",
      0.0
    },
    {
      NavigationPhase::Error,
      "error",
      0.0
    },
  }};

  constexpr int navigation_frames_per_phase = 30;
  constexpr double navigation_preview_fps = 15.0;

  int total_navigation_frames = 0;

  for (const auto & spec : navigation_animation_specs) {
    const std::string phase_directory =
      navigation_animation_root +
      "/" +
      spec.directory;

    try {
      std::filesystem::create_directories(
        phase_directory);
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        get_logger(),
        "failed to create Navigation animation directory | dir=%s error=%s",
        phase_directory.c_str(),
        error.what());
      continue;
    }

    for (
      int frame = 0;
      frame < navigation_frames_per_phase;
      ++frame)
    {
      seed_navigation_preview_data(spec.phase);

      const double frame_seconds =
        static_cast<double>(frame) /
        navigation_preview_fps;

      navigation_animation_time_seconds_ =
        spec.time_offset_seconds +
        frame_seconds;

      navigation_phase_elapsed_seconds_ =
        frame_seconds;

      rendered_navigation_phase_ =
        spec.phase;

      render_navigation_page(spec.phase);

      std::ostringstream frame_path;
      frame_path
        << phase_directory
        << "/frame_"
        << std::setw(3)
        << std::setfill('0')
        << frame
        << ".ppm";

      if (!canvas_.write_ppm(frame_path.str())) {
        RCLCPP_WARN(
          get_logger(),
          "failed to write Navigation animation frame | %s",
          frame_path.str().c_str());
      }

      ++total_navigation_frames;
    }

    RCLCPP_INFO(
      get_logger(),
      "saved Navigation phase animation | phase=%s dir=%s frames=%d",
      spec.directory,
      phase_directory.c_str(),
      navigation_frames_per_phase);
  }

  RCLCPP_INFO(
    get_logger(),
    "saved all Navigation animations | root=%s total_frames=%d",
    navigation_animation_root.c_str(),
    total_navigation_frames);

  navigation_ui_ =
    saved_navigation_ui;

  navigation_phase_ =
    saved_navigation_phase;

  navigation_animation_time_seconds_ =
    saved_navigation_animation_time;

  navigation_phase_elapsed_seconds_ =
    saved_navigation_phase_elapsed;

  rendered_navigation_phase_ =
    saved_rendered_navigation_phase;

  // ============================================================
  // Status previews
  // ============================================================
  const auto saved_status_ui =
    status_ui_;

  const auto saved_status_view =
    status_view_;

  const double saved_status_animation_time =
    status_animation_time_seconds_;

  seed_status_preview_data();

  auto export_status_view =
    [this](
      const StatusView view,
      const std::string & file_name)
    {
      status_view_ = view;
      render_status_page();

      const std::string output_path =
        config_.preview_output_dir +
        "/" +
        file_name;

      if (!canvas_.write_ppm(output_path)) {
        RCLCPP_WARN(
          get_logger(),
          "failed to write Status state preview | %s",
          output_path.c_str());
        return;
      }

      RCLCPP_INFO(
        get_logger(),
        "saved Status state preview | %s",
        output_path.c_str());
    };

  export_status_view(
    StatusView::Overview,
    "status_preview.ppm");

  export_status_view(
    StatusView::Overview,
    "status_overview.ppm");

  export_status_view(
    StatusView::Sensors,
    "status_sensors.ppm");

  export_status_view(
    StatusView::AiLink,
    "status_ai_link.ppm");

  export_status_view(
    StatusView::AlertsSystem,
    "status_alerts_system.ppm");

  // ============================================================
  // Status animations for all four internal views
  // ============================================================
  const std::string status_animation_root =
    config_.preview_output_dir +
    "/status_animation";

  struct StatusAnimationSpec
  {
    StatusView view;
    const char * directory;
  };

  const std::array<StatusAnimationSpec, 4>
  status_animation_specs{{
    {
      StatusView::Overview,
      "overview"
    },
    {
      StatusView::Sensors,
      "sensors"
    },
    {
      StatusView::AiLink,
      "ai_link"
    },
    {
      StatusView::AlertsSystem,
      "alerts_system"
    },
  }};

  constexpr int status_frames_per_view = 30;
  constexpr double status_preview_fps = 15.0;

  int total_status_animation_frames = 0;

  for (const auto & spec : status_animation_specs) {
    const std::string phase_directory =
      status_animation_root +
      "/" +
      spec.directory;

    try {
      std::filesystem::create_directories(
        phase_directory);
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        get_logger(),
        "failed to create Status animation directory | dir=%s error=%s",
        phase_directory.c_str(),
        error.what());
      continue;
    }

    for (
      int frame = 0;
      frame < status_frames_per_view;
      ++frame)
    {
      seed_status_preview_data();

      status_view_ =
        spec.view;

      status_animation_time_seconds_ =
        static_cast<double>(frame) /
        status_preview_fps;

      // Dry-run only: demonstrate the conditional alert pulse.
      if (spec.view == StatusView::AlertsSystem) {
        status_ui_.alert_count = 1;
        status_ui_.active_alert =
          "Depth front slowdown";
      }

      render_status_page();

      std::ostringstream frame_path;
      frame_path
        << phase_directory
        << "/frame_"
        << std::setw(3)
        << std::setfill('0')
        << frame
        << ".ppm";

      if (!canvas_.write_ppm(frame_path.str())) {
        RCLCPP_WARN(
          get_logger(),
          "failed to write Status animation frame | %s",
          frame_path.str().c_str());
      }

      ++total_status_animation_frames;
    }

    RCLCPP_INFO(
      get_logger(),
      "saved Status view animation | view=%s dir=%s frames=%d",
      spec.directory,
      phase_directory.c_str(),
      status_frames_per_view);
  }

  RCLCPP_INFO(
    get_logger(),
    "saved all Status animations | root=%s total_frames=%d",
    status_animation_root.c_str(),
    total_status_animation_frames);

  status_animation_time_seconds_ =
    saved_status_animation_time;

  status_ui_ =
    saved_status_ui;

  status_view_ =
    saved_status_view;

  // ============================================================
  // Power preview
  // ============================================================
  const auto saved_core_power =
    core_power_;

  const auto saved_edge_power =
    edge_power_;

  const auto saved_base_power =
    base_power_;

  const bool saved_power_status_seen =
    power_status_seen_;

  const auto saved_power_overall_state =
    power_overall_state_;

  const auto saved_power_health_state =
    power_health_state_;

  const auto saved_power_status_update =
    power_status_last_update_;

  const double saved_power_animation_time =
    power_animation_time_seconds_;

  seed_power_preview_data();
  render_power_page();

  const std::string power_preview_path =
    config_.preview_output_dir +
    "/power_preview.ppm";

  if (!canvas_.write_ppm(power_preview_path)) {
    RCLCPP_WARN(
      get_logger(),
      "failed to write Power dashboard preview | %s",
      power_preview_path.c_str());
  } else {
    RCLCPP_INFO(
      get_logger(),
      "saved Power dashboard preview | %s",
      power_preview_path.c_str());
  }

  // ============================================================
  // Power animated preview
  // ============================================================
  const std::string power_animation_directory =
    config_.preview_output_dir +
    "/power_animation";

  try {
    std::filesystem::create_directories(
      power_animation_directory);
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      get_logger(),
      "failed to create Power animation directory | dir=%s error=%s",
      power_animation_directory.c_str(),
      error.what());
  }

  constexpr int power_animation_frames = 45;
  constexpr double power_preview_fps = 15.0;

  seed_power_preview_data();

  // Dry-run only: demonstrate charging presentation.
  edge_power_.state = "charging";
  power_overall_state_ = "charging";

  for (
    int frame = 0;
    frame < power_animation_frames;
    ++frame)
  {
    power_animation_time_seconds_ =
      static_cast<double>(frame) /
      power_preview_fps;

    render_power_page();

    std::ostringstream frame_path;
    frame_path
      << power_animation_directory
      << "/frame_"
      << std::setw(3)
      << std::setfill('0')
      << frame
      << ".ppm";

    if (!canvas_.write_ppm(frame_path.str())) {
      RCLCPP_WARN(
        get_logger(),
        "failed to write Power animation frame | %s",
        frame_path.str().c_str());
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "saved Power animation preview | dir=%s frames=%d",
    power_animation_directory.c_str(),
    power_animation_frames);

  power_animation_time_seconds_ =
    saved_power_animation_time;

  core_power_ =
    saved_core_power;

  edge_power_ =
    saved_edge_power;

  base_power_ =
    saved_base_power;

  power_status_seen_ =
    saved_power_status_seen;

  power_overall_state_ =
    saved_power_overall_state;

  power_health_state_ =
    saved_power_health_state;

  power_status_last_update_ =
    saved_power_status_update;

  // Return the canvas to the actual runtime screen.
  render_current_screen();
}


void UiNode::configure_display()
{
  if (!config_.enable_framebuffer) {
    RCLCPP_INFO(get_logger(), "framebuffer disabled by config");
    return;
  }

  std::string error_message;
  if (!framebuffer_.open_device(
      config_.framebuffer_device,
      config_.screen_width,
      config_.screen_height,
      &error_message))
  {
    RCLCPP_ERROR(
      get_logger(),
      "failed to open framebuffer | device=%s error=%s",
      config_.framebuffer_device.c_str(),
      error_message.c_str());
    throw std::runtime_error(error_message);
  }

  RCLCPP_INFO(
    get_logger(),
    "framebuffer opened | device=%s size=%dx%d bpp=%d",
    framebuffer_.device_path().c_str(),
    framebuffer_.width(),
    framebuffer_.height(),
    framebuffer_.bits_per_pixel());
}

void UiNode::configure_touch()
{
  if (!config_.enable_touch) {
    RCLCPP_INFO(get_logger(), "touch disabled by config");
    return;
  }

  touch_fd_ = ::open(config_.touch_device.c_str(), O_RDONLY | O_NONBLOCK);
  if (touch_fd_ < 0) {
    RCLCPP_ERROR(
      get_logger(),
      "failed to open touch input | device=%s error=%s",
      config_.touch_device.c_str(),
      std::strerror(errno));
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "touch input opened | device=%s",
    config_.touch_device.c_str());
}

void UiNode::close_touch()
{
  if (touch_fd_ >= 0) {
    ::close(touch_fd_);
    touch_fd_ = -1;
  }
}

void UiNode::poll_touch_input()
{
  if (!config_.enable_touch || touch_fd_ < 0) {
    return;
  }

  input_event ev{};

  while (true) {
    const auto n = ::read(touch_fd_, &ev, sizeof(ev));

    if (n == static_cast<ssize_t>(sizeof(ev))) {
      if (ev.type == EV_ABS) {
        if (ev.code == ABS_X || ev.code == ABS_MT_POSITION_X) {
          touch_x_ = ev.value;
        } else if (ev.code == ABS_Y || ev.code == ABS_MT_POSITION_Y) {
          touch_y_ = ev.value;
        }
      } else if (ev.type == EV_KEY && ev.code == BTN_TOUCH) {
        if (ev.value == 1) {
          touch_down_ = true;
          touch_down_x_ = touch_x_;
          touch_down_y_ = touch_y_;

          RCLCPP_DEBUG(
            get_logger(),
            "touch down | x=%d y=%d",
            touch_down_x_,
            touch_down_y_);
        } else if (ev.value == 0 && touch_down_) {
          touch_down_ = false;

          const int dx = std::abs(touch_x_ - touch_down_x_);
          const int dy = std::abs(touch_y_ - touch_down_y_);

          // Treat as tap, not swipe.
          if (dx <= 35 && dy <= 35) {
            handle_touch_tap(touch_x_, touch_y_);
          }
        }
      }

      continue;
    }

    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      break;
    }

    if (n < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "touch read failed | error=%s",
        std::strerror(errno));
      break;
    }

    break;
  }
}

void UiNode::handle_touch_tap(const int x, const int y)
{
  UiScreen next_screen{UiScreen::Home};

  if (active_screen_ == UiScreen::Intro) {
    RCLCPP_INFO(
      get_logger(),
      "touch tap during intro | skipping to home x=%d y=%d",
      x,
      y);

    intro_elapsed_seconds_ = config_.intro_seconds;
    transition_to(UiScreen::Home);
    return;
  }

  // Status tabs have priority while Status is visible.
  if (active_screen_ == UiScreen::Status) {
    StatusView requested_view{StatusView::Overview};

    if (status_view_from_touch(x, y, &requested_view)) {
      request_status_view(
        requested_view,
        ScreenRequestSource::Touch);
      return;
    }
  }

  if (!screen_from_touch(x, y, &next_screen)) {
    RCLCPP_INFO(
      get_logger(),
      "touch tap outside active menu | x=%d y=%d",
      x,
      y);
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "touch menu tap | x=%d y=%d screen=%s",
    x,
    y,
    to_string(next_screen).c_str());

  // Pressing the left Navigate button opens the calm Idle state.
  if (next_screen == UiScreen::Navigation) {
    navigation_phase_ = NavigationPhase::Idle;
    navigation_ui_ = NavigationUiState{};

    request_screen(
      UiScreen::Navigation,
      ScreenRequestSource::Touch);
    return;
  }

  // Pressing the left Voice button always opens Voice Idle.
  if (next_screen == UiScreen::VoiceIdle) {
    voice_phase_ = VoicePhase::Idle;
    request_screen(
      UiScreen::VoiceIdle,
      ScreenRequestSource::Touch);
    return;
  }

  // Pressing the left Status button always opens Overview.
  if (next_screen == UiScreen::Status) {
    request_status_view(
      StatusView::Overview,
      ScreenRequestSource::Touch);
    return;
  }

  request_screen(next_screen, ScreenRequestSource::Touch);
}

bool UiNode::screen_from_touch(const int x, const int y, UiScreen * screen) const
{
  if (screen == nullptr) {
    return false;
  }

  // Left vertical menu hit zones.
  // Coordinates match the current 800x480 generated UI layout.
  const int menu_x0 = 10;
  const int menu_x1 = 96;

  if (x < menu_x0 || x > menu_x1) {
    return false;
  }

  struct Zone
  {
    int y0;
    int y1;
    UiScreen screen;
  };

  const std::vector<Zone> zones{
    {70, 132, UiScreen::Home},
    {133, 196, UiScreen::VoiceIdle},
    {197, 260, UiScreen::Navigation},
    {261, 324, UiScreen::Status},
    {325, 430, UiScreen::Power},
  };

  for (const auto & zone : zones) {
    if (y >= zone.y0 && y <= zone.y1) {
      *screen = zone.screen;
      return true;
    }
  }

  return false;
}

bool UiNode::status_view_from_touch(
  const int x,
  const int y,
  StatusView * view) const
{
  if (view == nullptr) {
    return false;
  }

  // Four tab hitboxes inside the Status content panel.
  if (y < 84 || y > 113) {
    return false;
  }

  struct StatusTabZone
  {
    int x0;
    int x1;
    StatusView view;
  };

  const std::array<StatusTabZone, 4> zones{{
    {138, 282, StatusView::Overview},
    {287, 431, StatusView::Sensors},
    {436, 580, StatusView::AiLink},
    {585, 730, StatusView::AlertsSystem},
  }};

  for (const auto & zone : zones) {
    if (x >= zone.x0 && x <= zone.x1) {
      *view = zone.view;
      return true;
    }
  }

  return false;
}


void UiNode::configure_power_subscriptions()
{
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  power_core_subscription_ =
    create_subscription<std_msgs::msg::String>(
    config_.power_core_topic,
    qos,
    [this](std_msgs::msg::String::ConstSharedPtr message) {
      update_power_source(
        core_power_,
        message->data,
        "Core UPS",
        "capacity ",
        "CAP");
    });

  power_edge_subscription_ =
    create_subscription<std_msgs::msg::String>(
    config_.power_edge_topic,
    qos,
    [this](std_msgs::msg::String::ConstSharedPtr message) {
      update_power_source(
        edge_power_,
        message->data,
        "Edge UPS",
        "capacity ",
        "CAP");
    });

  power_base_subscription_ =
    create_subscription<std_msgs::msg::String>(
    config_.power_base_topic,
    qos,
    [this](std_msgs::msg::String::ConstSharedPtr message) {
      update_power_source(
        base_power_,
        message->data,
        "Base battery",
        "SoC ",
        "SOC");
    });

  power_status_subscription_ =
    create_subscription<std_msgs::msg::String>(
    config_.power_status_topic,
    qos,
    [this](std_msgs::msg::String::ConstSharedPtr message) {
      update_power_status(message->data);
    });

  RCLCPP_INFO(
    get_logger(),
    "power UI subscriptions configured | core=%s edge=%s base=%s status=%s",
    config_.power_core_topic.c_str(),
    config_.power_edge_topic.c_str(),
    config_.power_base_topic.c_str(),
    config_.power_status_topic.c_str());
}

void UiNode::update_power_source(
  PowerUiSourceState & source,
  const std::string & text,
  const std::string & source_label,
  const std::string & percent_token,
  const std::string & percent_label)
{
  source.seen = true;
  source.last_update = std::chrono::steady_clock::now();
  source.percent_label = percent_label;

  const auto colon = text.find(':');

  if (
    colon != std::string::npos &&
    text.rfind(source_label, 0U) == 0U)
  {
    source.state = lowercase_copy(
      trim_copy(
        text.substr(
          source_label.size(),
          colon - source_label.size())));
  } else {
    source.state = "unknown";
  }

  double voltage = 0.0;
  source.has_voltage = extract_number_before(text, " V", &voltage);

  if (source.has_voltage) {
    source.voltage_v = voltage;
    source.voltage_history.push_back(voltage);

    while (
      source.voltage_history.size() >
      static_cast<std::size_t>(config_.power_history_samples))
    {
      source.voltage_history.pop_front();
    }
  }

  double percent = 0.0;
  source.has_percent =
    extract_number_after(text, percent_token, &percent);

  if (source.has_percent) {
    source.percent = std::clamp(percent, 0.0, 100.0);
  }
}

void UiNode::update_power_status(const std::string & text)
{
  const std::string overall =
    extract_value_after_key(text, "overall=");

  const std::string health =
    extract_value_after_key(text, "health=");

  if (!overall.empty()) {
    power_overall_state_ = lowercase_copy(overall);
  }

  if (!health.empty()) {
    power_health_state_ = uppercase_copy(health);
  }

  power_status_seen_ = true;
  power_status_last_update_ = std::chrono::steady_clock::now();
}

bool UiNode::power_source_is_stale(
  const PowerUiSourceState & source) const
{
  if (!source.seen) {
    return false;
  }

  const double age_seconds =
    std::chrono::duration<double>(
    std::chrono::steady_clock::now() -
    source.last_update).count();

  return age_seconds > config_.power_stale_timeout_s;
}


void UiNode::configure_runtime()
{
  const auto period = std::chrono::duration<double>(1.0 / config_.loop_hz);
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(period);

  loop_timer_ = create_wall_timer(
    period_ns,
    std::bind(&UiNode::loop_callback, this));
}

void UiNode::update_runtime(const double dt_seconds)
{
  // Cap unusually large frame intervals so animations do not jump after
  // debugging pauses or temporary scheduling delays.
  const double animation_dt =
    std::clamp(dt_seconds, 0.0, 0.25);

  voice_animation_time_seconds_ += animation_dt;
  voice_phase_elapsed_seconds_ += animation_dt;

  navigation_animation_time_seconds_ += animation_dt;
  navigation_phase_elapsed_seconds_ += animation_dt;

  status_animation_time_seconds_ += animation_dt;
  power_animation_time_seconds_ += animation_dt;

  poll_touch_input();

  home_glow_elapsed_seconds_ += dt_seconds;

  if (active_screen_ == UiScreen::Intro) {
    intro_elapsed_seconds_ += dt_seconds;

    if (intro_elapsed_seconds_ >= config_.intro_seconds) {
      transition_to(UiScreen::Home);
    }
  }

  if (active_screen_ == UiScreen::Home && !robot360_frames_.empty()) {
    robot360_elapsed_seconds_ += dt_seconds;

    if (robot360_elapsed_seconds_ >= config_.robot360_seconds_per_frame) {
      robot360_elapsed_seconds_ = 0.0;
      home_idle_sequence_index_ =
        (home_idle_sequence_index_ + 1U) % home_idle_sequence_.size();

      const auto frame_index = home_idle_sequence_[home_idle_sequence_index_];

      RCLCPP_DEBUG(
        get_logger(),
        "home generated idle frame | sequence_index=%zu frame_index=%zu",
        home_idle_sequence_index_,
        frame_index);
    }
  }
}

void UiNode::render_current_screen()
{
  switch (active_screen_) {
    case UiScreen::Intro:
      render_intro();
      break;

    case UiScreen::Home:
      render_home();
      break;

    case UiScreen::Listening:
      render_voice_page(VoicePhase::Listening);
      break;

    case UiScreen::Thinking:
      render_voice_page(VoicePhase::Thinking);
      break;

    case UiScreen::Speaking:
      render_voice_page(VoicePhase::Speaking);
      break;

    case UiScreen::VoiceIdle:
      render_voice_page(voice_phase_);
      break;

    case UiScreen::Navigation:
      render_navigation_page(navigation_phase_);
      break;

    case UiScreen::Status:
      render_status_page();
      break;

    case UiScreen::Power:
      render_power_page();
      break;

    default:
      render_home();
      break;
  }
}

void UiNode::render_intro()
{
  canvas_.clear(ColorRgb{0U, 0U, 0U});

  const bool ok = canvas_.draw_image_fit(boot_intro_);
  if (!ok) {
    RCLCPP_WARN(get_logger(), "failed to draw intro image");
  }

  const float progress =
    static_cast<float>(
      std::clamp(intro_elapsed_seconds_ / config_.intro_seconds, 0.0, 1.0));

  render_intro_overlay(progress);
}

void UiNode::render_intro_overlay(const float progress)
{
  const float phase =
    std::fmod(static_cast<float>(intro_elapsed_seconds_) * 0.42F, 1.0F);

  const double wave = 0.5 + 0.5 * std::sin(intro_elapsed_seconds_ * 2.2);

  const int center_x = config_.screen_width / 2;

  // Soft glow behind/around the robot. This should be subtle, not a line over the body.
  canvas_.draw_circle_ring(
    center_x,
    247,
    126 + static_cast<int>(wave * 5.0),
    3,
    ColorRgb{35U, 150U, 255U},
    static_cast<float>(0.07 + wave * 0.05));

  // Animate the lower floor ring area only. This matches the correct area you marked.
  canvas_.draw_circle_ring(
    center_x,
    310,
    118 + static_cast<int>(wave * 8.0),
    4,
    ColorRgb{45U, 190U, 255U},
    static_cast<float>(0.18 + wave * 0.10));

  canvas_.draw_circle_ring(
    center_x,
    310,
    146 + static_cast<int>(wave * 5.0),
    3,
    ColorRgb{0U, 95U, 220U},
    static_cast<float>(0.10 + wave * 0.06));

  // Small circular loading spinner only. No progress bar.
  canvas_.draw_spinner(
    center_x,
    388,
    11,
    phase,
    ColorRgb{110U, 220U, 255U});

  // Fade in
  if (progress < 0.12F) {
    const float fade_alpha = 1.0F - (progress / 0.12F);
    canvas_.blend_rect(
      0,
      0,
      config_.screen_width,
      config_.screen_height,
      ColorRgb{0U, 0U, 0U},
      fade_alpha);
  }

  // Fade out
  if (progress > 0.90F) {
    const float fade_alpha = (progress - 0.90F) / 0.10F;
    canvas_.blend_rect(
      0,
      0,
      config_.screen_width,
      config_.screen_height,
      ColorRgb{0U, 0U, 0U},
      fade_alpha);
  }
}

void UiNode::render_home()
{
  canvas_.clear(ColorRgb{2U, 8U, 22U});

  if (robot360_frames_.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "cannot render home: generated home frames missing");
    return;
  }

  if (home_idle_sequence_.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "cannot render home: idle sequence missing");
    return;
  }

  const auto frame_index = home_idle_sequence_[home_idle_sequence_index_];
  const auto & frame = robot360_frames_[frame_index];

  // Draw the pre-rendered 800x480 homepage frame directly.
  // No C++ dashboard overlay here; design is generated offline.
  const bool ok = canvas_.draw_image_fit(frame);

  if (!ok) {
    RCLCPP_WARN(get_logger(), "failed to draw generated home frame");
  }
  draw_live_top_bar_overlay(canvas_);

}


void UiNode::render_home_glow()
{
  const double wave = 0.5 + 0.5 * std::sin(home_glow_elapsed_seconds_ * 0.7);
  const int pulse_radius = 198 + static_cast<int>(wave * 10.0);
  const float soft_alpha = static_cast<float>(0.10 + wave * 0.05);
  const float sharp_alpha = static_cast<float>(0.16 + wave * 0.06);

  const int center_x = config_.screen_width / 2;
  const int center_y = 350;

  canvas_.draw_circle_ring(
    center_x,
    center_y,
    pulse_radius + 24,
    8,
    ColorRgb{0U, 95U, 190U},
    soft_alpha);

  canvas_.draw_circle_ring(
    center_x,
    center_y,
    pulse_radius,
    3,
    ColorRgb{70U, 210U, 255U},
    sharp_alpha);
}

void UiNode::render_home_dashboard()
{
  render_home_top_bar();
  render_home_left_menu();
  render_home_status_panel();

  Font::draw_text(canvas_, 126, 96, "SAVO", ColorRgb{190U, 230U, 255U}, 7, 0.95F);
  Font::draw_text(canvas_, 128, 152, "AUTONOMOUS GUIDE ROBOT", ColorRgb{230U, 240U, 250U}, 2, 0.80F);
  Font::draw_text(canvas_, 156, 198, "READY TO ASSIST", ColorRgb{70U, 235U, 255U}, 3, 0.92F);
}

void UiNode::render_home_top_bar()
{
  canvas_.blend_rect(0, 0, config_.screen_width, 50, ColorRgb{0U, 6U, 18U}, 0.82F);
  canvas_.blend_rect(0, 49, config_.screen_width, 1, ColorRgb{0U, 180U, 230U}, 0.60F);

  Font::draw_text(canvas_, 26, 17, "SAVO", ColorRgb{230U, 245U, 255U}, 3, 0.95F);
  Font::draw_text(canvas_, 126, 17, "HOME", ColorRgb{70U, 235U, 255U}, 3, 0.95F);

  Font::draw_text(canvas_, 610, 17, "10.0.0.15", ColorRgb{230U, 245U, 255U}, 2, 0.88F);
  Font::draw_text(canvas_, 724, 17, "09:42", ColorRgb{230U, 245U, 255U}, 2, 0.88F);
}

void UiNode::render_home_left_menu()
{
  const int x = 14;
  const int y = 70;
  const int w = 74;
  const int h = 380;

  canvas_.blend_rect(x, y, w, h, ColorRgb{0U, 12U, 30U}, 0.72F);

  canvas_.draw_circle_ring(x + w / 2, y + 42, 31, 2, ColorRgb{50U, 210U, 255U}, 0.50F);
  canvas_.blend_rect(x + 8, y + 8, w - 16, 68, ColorRgb{0U, 55U, 95U}, 0.42F);
  Font::draw_text(canvas_, x + 19, y + 90, "HOME", ColorRgb{240U, 250U, 255U}, 2, 0.90F);

  Font::draw_text(canvas_, x + 15, y + 150, "VOICE", ColorRgb{220U, 235U, 245U}, 2, 0.78F);
  Font::draw_text(canvas_, x + 7, y + 220, "NAV", ColorRgb{220U, 235U, 245U}, 2, 0.78F);
  Font::draw_text(canvas_, x + 9, y + 290, "STATUS", ColorRgb{220U, 235U, 245U}, 2, 0.78F);
  Font::draw_text(canvas_, x + 15, y + 360, "POWER", ColorRgb{220U, 235U, 245U}, 2, 0.78F);

  for (int yy : {145, 215, 285, 355}) {
    canvas_.blend_rect(x + 12, y + yy - 18, w - 24, 1, ColorRgb{30U, 120U, 160U}, 0.45F);
  }
}

void UiNode::render_home_status_panel()
{
  const int x = 604;
  const int y = 104;
  const int w = 176;
  const int h = 240;

  canvas_.blend_rect(x, y, w, h, ColorRgb{0U, 12U, 32U}, 0.74F);
  canvas_.draw_circle_ring(x + w / 2, y + h / 2, 132, 2, ColorRgb{30U, 180U, 230U}, 0.35F);

  Font::draw_text(canvas_, x + 34, y + 22, "SYSTEM STATUS", ColorRgb{70U, 235U, 255U}, 2, 0.95F);

  const int row_x = x + 18;
  int row_y = y + 58;

  auto status_row = [&](const std::string & label, const std::string & value) {
    canvas_.blend_rect(row_x, row_y, w - 36, 42, ColorRgb{0U, 22U, 48U}, 0.62F);
    Font::draw_text(canvas_, row_x + 12, row_y + 14, label, ColorRgb{240U, 250U, 255U}, 2, 0.84F);
    Font::draw_text(canvas_, row_x + 96, row_y + 14, value, ColorRgb{50U, 235U, 255U}, 2, 0.95F);
    row_y += 48;
  };

  status_row("MODE", "IDLE");
  status_row("VOICE", "READY");
  status_row("NAV", "STANDBY");
  status_row("BATTERY", "GOOD");
}


void UiNode::render_page_shell(
  const ImageAsset & shell,
  const std::string & page_name)
{
  canvas_.clear(ColorRgb{2U, 8U, 22U});

  if (!canvas_.draw_image_fit(shell)) {
    RCLCPP_WARN(
      get_logger(),
      "failed to draw generated page shell | page=%s",
      page_name.c_str());
    return;
  }

  // IP addresses and local time are deliberately runtime-owned.
  // They are not baked into the generated static shell.
  draw_live_top_bar_overlay(canvas_);
}




void UiNode::render_voice_page(const VoicePhase phase)
{
  if (phase != rendered_voice_phase_) {
    rendered_voice_phase_ = phase;
    voice_phase_elapsed_seconds_ = 0.0;
  }

  render_page_shell(voice_shell_, "Voice");

  const ColorRgb cyan{70U, 235U, 255U};
  const ColorRgb cyan_soft{145U, 210U, 230U};
  const ColorRgb white{235U, 248U, 255U};
  const ColorRgb good{75U, 235U, 175U};
  const ColorRgb warning{255U, 190U, 70U};
  const ColorRgb danger{255U, 90U, 95U};
  const ColorRgb muted{135U, 165U, 185U};

  auto draw_voice_text =
    [this](
      const int x,
      const int y,
      const std::string & value,
      const ColorRgb color,
      const bool medium,
      const float alpha = 1.0F)
    {
      if (medium) {
        Font::draw_atlas_text(
          canvas_,
          ui_font_medium_,
          20,
          24,
          16,
          2,
          x,
          y,
          value,
          color,
          alpha);
        return;
      }

      Font::draw_atlas_text(
        canvas_,
        ui_font_small_,
        18,
        22,
        16,
        2,
        x,
        y,
        value,
        color,
        alpha);
    };

  auto voice_text_width =
    [this](
      const std::string & value,
      const bool medium) -> int
    {
      if (medium) {
        return Font::atlas_text_width(
          ui_font_medium_,
          20,
          24,
          16,
          value);
      }

      return Font::atlas_text_width(
        ui_font_small_,
        18,
        22,
        16,
        value);
    };

  auto fit_text =
    [&](std::string value,
      const int maximum_width,
      const bool medium) -> std::string
    {
      if (voice_text_width(value, medium) <= maximum_width) {
        return value;
      }

      const std::string suffix = "...";

      while (
        value.size() > 3U &&
        voice_text_width(value + suffix, medium) > maximum_width)
      {
        value.pop_back();
      }

      return value + suffix;
    };

  auto centered_text =
    [&](const int center_x,
      const int y,
      const std::string & value,
      const ColorRgb color,
      const bool medium,
      const float alpha = 1.0F)
    {
      const int width =
        voice_text_width(value, medium);

      draw_voice_text(
        center_x - (width / 2),
        y,
        value,
        color,
        medium,
        alpha);
    };

  auto state_color =
    [&](const std::string & raw_state) -> ColorRgb
    {
      const std::string state =
        uppercase_copy(raw_state);

      if (
        state == "ERROR" ||
        state == "FAILED" ||
        state == "OFFLINE" ||
        state == "BLOCKED")
      {
        return danger;
      }

      if (
        state == "WARNING" ||
        state == "STALE" ||
        state == "DEGRADED")
      {
        return warning;
      }

      if (
        state == "READY" ||
        state == "ONLINE" ||
        state == "OK" ||
        state == "ACTIVE" ||
        state == "PLAYING")
      {
        return good;
      }

      if (
        state == "MISSING" ||
        state == "UNKNOWN" ||
        state == "--")
      {
        return muted;
      }

      return cyan;
    };

  std::string phase_title{"VOICE READY"};
  std::string phase_state{"IDLE"};
  ColorRgb phase_color = cyan;

  switch (phase) {
    case VoicePhase::Idle:
      phase_title = "VOICE READY";
      phase_state = "IDLE";
      phase_color = cyan;
      break;

    case VoicePhase::Listening:
      phase_title = "LISTENING";
      phase_state = "LIVE";
      phase_color = good;
      break;

    case VoicePhase::Thinking:
      phase_title = "THINKING";
      phase_state = "PROCESSING";
      phase_color = cyan;
      break;

    case VoicePhase::Speaking:
      phase_title = "SPEAKING";
      phase_state = "PLAYING";
      phase_color = good;
      break;

    case VoicePhase::Error:
      phase_title = "VOICE ERROR";
      phase_state = "ERROR";
      phase_color = danger;
      break;
  }

  // Decorative animation values. These do not represent robot state.
  const double idle_pulse =
    0.5 +
    0.5 *
    std::sin(
      voice_animation_time_seconds_ * 2.4);

  const int thinking_active_block =
    static_cast<int>(
      std::fmod(
        voice_animation_time_seconds_ * 4.0,
        5.0));

  const bool preview_animation =
    config_.export_preview_frames;

  // Remove the generated shell's duplicated inner heading.
  canvas_.blend_rect(
    132,
    82,
    618,
    356,
    ColorRgb{0U, 11U, 29U},
    1.0F);

  // ============================================================
  // Shared Voice header
  // ============================================================
  canvas_.blend_rect(
    132,
    88,
    618,
    34,
    ColorRgb{0U, 25U, 52U},
    0.94F);

  const float heading_alpha =
    phase == VoicePhase::Idle ?
    static_cast<float>(0.78 + (0.22 * idle_pulse)) :
    1.0F;

  draw_voice_text(
    148,
    96,
    phase_title,
    phase_color,
    true,
    heading_alpha);

  const int phase_state_width =
    voice_text_width(phase_state, false);

  draw_voice_text(
    734 - phase_state_width,
    99,
    phase_state,
    phase_color,
    false);

  canvas_.blend_rect(
    132,
    130,
    618,
    210,
    ColorRgb{0U, 16U, 36U},
    0.94F);

  // ============================================================
  // IDLE
  // ============================================================
  if (phase == VoicePhase::Idle) {
    centered_text(
      441,
      157,
      "HELLO, I AM SAVO",
      white,
      true);

    centered_text(
      441,
      191,
      "Wake me with any of these phrases",
      cyan_soft,
      false);

    struct WakePhrase
    {
      int x;
      const char * text;
    };

    const std::array<WakePhrase, 3> phrases{{
      {154, "HEY SAVO"},
      {339, "HI SAVO"},
      {524, "SAVO"},
    }};

    for (
      std::size_t index = 0;
      index < phrases.size();
      ++index)
    {
      const auto & phrase = phrases[index];

      const double card_pulse =
        0.5 +
        0.5 *
        std::sin(
          voice_animation_time_seconds_ * 2.0 +
          static_cast<double>(index) * 0.85);

      canvas_.blend_rect(
        phrase.x,
        222,
        160,
        42,
        ColorRgb{0U, 35U, 64U},
        static_cast<float>(
          0.78 +
          0.18 * card_pulse));

      canvas_.blend_rect(
        phrase.x,
        262,
        160,
        2,
        cyan,
        static_cast<float>(
          0.30 +
          0.65 * card_pulse));

      centered_text(
        phrase.x + 80,
        234,
        phrase.text,
        cyan,
        false,
        static_cast<float>(
          0.78 +
          0.22 * card_pulse));
    }

    centered_text(
      441,
      292,
      "I am ready when you are.",
      muted,
      false);
  }

  // ============================================================
  // LISTENING
  // ============================================================
  if (phase == VoicePhase::Listening) {
    centered_text(
      441,
      151,
      "SPEAK NOW",
      white,
      true);

    centered_text(
      441,
      181,
      "I am listening to your command",
      cyan_soft,
      false);

    constexpr int waveform_x = 164;
    constexpr int waveform_y = 292;
    constexpr int bar_width = 12;
    constexpr int bar_gap = 10;
    constexpr int maximum_height = 82;

    canvas_.blend_rect(
      waveform_x,
      waveform_y,
      546,
      1,
      ColorRgb{25U, 65U, 82U},
      0.85F);

    for (
      std::size_t index = 0;
      index < voice_ui_.waveform.size();
      ++index)
    {
      double normalized =
        std::clamp(
          voice_ui_.waveform[index],
          0.0,
          1.0);

      // Only dry-run preview adds synthetic movement. On the Pi,
      // waveform values must come from real savo_speech audio levels.
      if (
        preview_animation &&
        voice_ui_.live &&
        uppercase_copy(voice_ui_.microphone_state) != "MISSING")
      {
        const double movement =
          0.72 +
          0.28 *
          std::sin(
            voice_animation_time_seconds_ * 7.0 +
            static_cast<double>(index) * 0.58);

        normalized =
          std::clamp(
            normalized * movement,
            0.0,
            1.0);
      }

      const int height =
        std::max(
          4,
          static_cast<int>(
            std::lround(
              normalized *
              static_cast<double>(maximum_height))));

      const int x =
        waveform_x +
        static_cast<int>(index) *
        (bar_width + bar_gap);

      const double highlight =
        0.5 +
        0.5 *
        std::sin(
          voice_animation_time_seconds_ * 6.0 -
          static_cast<double>(index) * 0.34);

      canvas_.blend_rect(
        x,
        waveform_y - (height / 2),
        bar_width,
        height,
        phase_color,
        static_cast<float>(
          0.58 +
          0.40 * highlight));
    }
  }

  // ============================================================
  // THINKING
  // ============================================================
  if (phase == VoicePhase::Thinking) {
    draw_voice_text(
      152,
      148,
      "YOU SAID",
      cyan_soft,
      false);

    canvas_.blend_rect(
      150,
      173,
      582,
      58,
      ColorRgb{0U, 31U, 57U},
      0.94F);

    draw_voice_text(
      168,
      191,
      fit_text(
        voice_ui_.transcript,
        546,
        true),
      white,
      true);

    centered_text(
      441,
      255,
      "SAVOMIND IS PROCESSING",
      cyan,
      true);

    const int block_width = 28;
    const int block_gap = 12;
    const int total_width =
      (block_width * 5) +
      (block_gap * 4);

    const int start_x =
      441 - (total_width / 2);

    for (int index = 0; index < 5; ++index) {
      const int distance =
        (index - thinking_active_block + 5) % 5;

      const bool active = distance == 0;
      const bool trailing = distance == 4;

      const ColorRgb block_color =
        active || trailing ?
        cyan :
        ColorRgb{20U, 55U, 72U};

      const float block_alpha =
        active ?
        1.0F :
        trailing ?
        0.62F :
        0.34F;

      canvas_.blend_rect(
        start_x + index * (block_width + block_gap),
        294,
        block_width,
        active ? 9 : 7,
        block_color,
        block_alpha);
    }
  }

  // ============================================================
  // SPEAKING
  // ============================================================
  if (phase == VoicePhase::Speaking) {
    draw_voice_text(
      152,
      148,
      "SAVO SAYS",
      cyan_soft,
      false);

    canvas_.blend_rect(
      150,
      173,
      582,
      72,
      ColorRgb{0U, 31U, 57U},
      0.94F);

    draw_voice_text(
      168,
      195,
      fit_text(
        voice_ui_.reply,
        546,
        true),
      white,
      true);

    draw_voice_text(
      152,
      273,
      "PLAYBACK",
      cyan_soft,
      false);

    canvas_.blend_rect(
      246,
      281,
      466,
      8,
      ColorRgb{20U, 55U, 72U},
      0.94F);

    const int progress_width =
      static_cast<int>(
      std::lround(
        466.0 *
        std::clamp(
          voice_ui_.playback_progress,
          0.0,
          1.0)));

    canvas_.blend_rect(
      246,
      281,
      std::clamp(progress_width, 0, 466),
      8,
      good,
      0.98F);

    const std::string percentage =
      std::to_string(
      static_cast<int>(
        std::lround(
          std::clamp(
            voice_ui_.playback_progress,
            0.0,
            1.0) *
          100.0))) + "%";

    draw_voice_text(
      712 - voice_text_width(percentage, false),
      301,
      percentage,
      good,
      false);

    // Decorative speaker activity. Playback progress remains real data.
    const bool playback_active =
      uppercase_copy(voice_ui_.playback_state) == "PLAYING";

    for (int index = 0; index < 4; ++index) {
      const double wave =
        playback_active ?
        0.5 +
        0.5 *
        std::sin(
          voice_animation_time_seconds_ * 7.0 +
          static_cast<double>(index) * 0.80) :
        0.0;

      const int height =
        playback_active ?
        5 +
        static_cast<int>(
          std::lround(
            18.0 * wave)) :
        4;

      canvas_.blend_rect(
        168 + index * 18,
        316 - height,
        8,
        height,
        playback_active ? good : muted,
        playback_active ?
        static_cast<float>(
          0.60 +
          0.38 * wave) :
        0.35F);
    }
  }

  // ============================================================
  // ERROR
  // ============================================================
  if (phase == VoicePhase::Error) {
    // Pulse strongly only during the first 1.2 seconds, then stay stable.
    if (voice_phase_elapsed_seconds_ < 1.2) {
      const double error_pulse =
        0.5 +
        0.5 *
        std::sin(
          voice_phase_elapsed_seconds_ * 12.0);

      canvas_.blend_rect(
        132,
        130,
        618,
        3,
        danger,
        static_cast<float>(
          0.30 +
          0.65 * error_pulse));
    }

    centered_text(
      441,
      153,
      "I COULD NOT HEAR YOU",
      danger,
      true);

    canvas_.blend_rect(
      150,
      193,
      582,
      64,
      ColorRgb{50U, 14U, 28U},
      0.94F);

    centered_text(
      441,
      214,
      fit_text(
        voice_ui_.error_message,
        540,
        true),
      white,
      true);

    centered_text(
      441,
      282,
      fit_text(
        voice_ui_.error_detail,
        540,
        false),
      warning,
      false);

    centered_text(
      441,
      311,
      "Please try again.",
      muted,
      false);
  }

  // ============================================================
  // Shared phase details
  // ============================================================
  canvas_.blend_rect(
    132,
    348,
    618,
    58,
    ColorRgb{0U, 20U, 43U},
    0.94F);

  auto draw_metric =
    [&](const int x,
      const std::string & label,
      const std::string & value,
      const ColorRgb value_color)
    {
      draw_voice_text(
        x,
        357,
        label,
        muted,
        false);

      draw_voice_text(
        x,
        379,
        fit_text(
          value,
          170,
          false),
        value_color,
        false);
    };

  if (phase == VoicePhase::Idle) {
    draw_metric(
      150,
      "WAKE WORD",
      voice_ui_.wake_word_state,
      state_color(voice_ui_.wake_word_state));

    draw_metric(
      350,
      "MICROPHONE",
      voice_ui_.microphone_state,
      state_color(voice_ui_.microphone_state));

    draw_metric(
      550,
      "SPEECH LINK",
      voice_ui_.speech_link_state,
      state_color(voice_ui_.speech_link_state));
  }

  if (phase == VoicePhase::Listening) {
    std::ostringstream level;
    level
      << std::fixed
      << std::setprecision(0)
      << voice_ui_.input_level_percent
      << "%";

    std::ostringstream elapsed;
    elapsed
      << std::fixed
      << std::setprecision(1)
      << voice_ui_.elapsed_seconds
      << " s";

    draw_metric(
      150,
      "INPUT LEVEL",
      level.str(),
      good);

    draw_metric(
      350,
      "ELAPSED",
      elapsed.str(),
      white);

    draw_metric(
      550,
      "MICROPHONE",
      voice_ui_.microphone_state,
      state_color(voice_ui_.microphone_state));
  }

  if (phase == VoicePhase::Thinking) {
    draw_metric(
      150,
      "PROVIDER",
      voice_ui_.provider,
      white);

    draw_metric(
      350,
      "INTENT",
      voice_ui_.intent,
      cyan);

    draw_metric(
      550,
      "STT ROUTE",
      voice_ui_.stt_route,
      state_color(voice_ui_.stt_route));
  }

  if (phase == VoicePhase::Speaking) {
    draw_metric(
      150,
      "TTS ROUTE",
      voice_ui_.tts_route,
      state_color(voice_ui_.tts_route));

    draw_metric(
      350,
      "SPEAKER",
      voice_ui_.speaker_state,
      state_color(voice_ui_.speaker_state));

    draw_metric(
      550,
      "PLAYBACK",
      voice_ui_.playback_state,
      state_color(voice_ui_.playback_state));
  }

  if (phase == VoicePhase::Error) {
    draw_metric(
      150,
      "MICROPHONE",
      voice_ui_.microphone_state,
      state_color(voice_ui_.microphone_state));

    draw_metric(
      350,
      "SPEECH LINK",
      voice_ui_.speech_link_state,
      state_color(voice_ui_.speech_link_state));

    draw_metric(
      550,
      "ERROR",
      voice_ui_.error_detail,
      warning);
  }

  // ============================================================
  // Shared instruction strip
  // ============================================================
  canvas_.blend_rect(
    132,
    414,
    618,
    23,
    ColorRgb{0U, 23U, 45U},
    0.94F);

  std::string instruction{"Say \"Hey Savo\" to begin."};
  ColorRgb instruction_color = cyan_soft;

  switch (phase) {
    case VoicePhase::Idle:
      instruction = "Say \"Hey Savo\" to begin.";
      instruction_color = cyan_soft;
      break;

    case VoicePhase::Listening:
      instruction = "Listening for your command...";
      instruction_color = good;
      break;

    case VoicePhase::Thinking:
      instruction = "Processing your request...";
      instruction_color = cyan;
      break;

    case VoicePhase::Speaking:
      instruction = "Savo is speaking...";
      instruction_color = good;
      break;

    case VoicePhase::Error:
      instruction = "Voice request failed. Please try again.";
      instruction_color = warning;
      break;
  }

  draw_voice_text(
    148,
    416,
    instruction,
    instruction_color,
    false);
}


void UiNode::seed_voice_preview_data(const VoicePhase phase)
{
  voice_ui_ = VoiceUiState{};
  voice_phase_ = phase;

  voice_ui_.live = true;
  voice_ui_.wake_word_state = "READY";
  voice_ui_.microphone_state = "READY";
  voice_ui_.speech_link_state = "ONLINE";

  voice_ui_.stt_route = "ONLINE";
  voice_ui_.tts_route = "ONLINE";
  voice_ui_.provider = "CEREBRAS";
  voice_ui_.intent = "NAVIGATION";

  voice_ui_.playback_state = "PLAYING";
  voice_ui_.speaker_state = "ACTIVE";

  voice_ui_.transcript =
    "Take me to room A201";

  voice_ui_.reply =
    "I can take you to room A201.";

  voice_ui_.error_message =
    "I could not hear you clearly.";

  voice_ui_.error_detail =
    "NO SPEECH DETECTED";

  voice_ui_.input_level_percent = 72.0;
  voice_ui_.elapsed_seconds = 2.4;
  voice_ui_.playback_progress = 0.68;

  voice_ui_.waveform = {{
    0.12, 0.24, 0.36, 0.20, 0.52, 0.74,
    0.42, 0.68, 0.92, 0.58, 0.36, 0.80,
    0.64, 0.46, 0.88, 0.72, 0.40, 0.62,
    0.34, 0.56, 0.78, 0.44, 0.28, 0.16,
  }};

  if (phase == VoicePhase::Idle) {
    voice_ui_.playback_state = "IDLE";
    voice_ui_.speaker_state = "READY";
  }

  if (phase == VoicePhase::Thinking) {
    voice_ui_.playback_state = "IDLE";
    voice_ui_.speaker_state = "READY";
  }

  if (phase == VoicePhase::Error) {
    voice_ui_.live = false;
    voice_ui_.playback_state = "IDLE";
    voice_ui_.speaker_state = "READY";
  }
}

void UiNode::render_navigation_page(
  const NavigationPhase phase)
{
  if (phase != rendered_navigation_phase_) {
    rendered_navigation_phase_ = phase;
    navigation_phase_elapsed_seconds_ = 0.0;
  }

  render_page_shell(
    navigate_shell_,
    "Navigate");

  const ColorRgb cyan{70U, 235U, 255U};
  const ColorRgb cyan_soft{145U, 210U, 230U};
  const ColorRgb white{235U, 248U, 255U};
  const ColorRgb good{75U, 235U, 175U};
  const ColorRgb warning{255U, 190U, 70U};
  const ColorRgb danger{255U, 90U, 95U};
  const ColorRgb face_fill{0U, 34U, 57U};

  constexpr double pi =
    3.14159265358979323846;

  auto draw_navigation_text =
    [this](
      const int x,
      const int y,
      const std::string & value,
      const ColorRgb color,
      const bool medium,
      const float alpha = 1.0F)
    {
      if (medium) {
        Font::draw_atlas_text(
          canvas_,
          ui_font_medium_,
          20,
          24,
          16,
          2,
          x,
          y,
          value,
          color,
          alpha);
        return;
      }

      Font::draw_atlas_text(
        canvas_,
        ui_font_small_,
        18,
        22,
        16,
        2,
        x,
        y,
        value,
        color,
        alpha);
    };

  auto navigation_text_width =
    [this](
      const std::string & value,
      const bool medium) -> int
    {
      if (medium) {
        return Font::atlas_text_width(
          ui_font_medium_,
          20,
          24,
          16,
          value);
      }

      return Font::atlas_text_width(
        ui_font_small_,
        18,
        22,
        16,
        value);
    };

  auto centered_text =
    [&](const int center_x,
      const int y,
      const std::string & value,
      const ColorRgb color,
      const bool medium,
      const float alpha = 1.0F)
    {
      draw_navigation_text(
        center_x -
        navigation_text_width(value, medium) / 2,
        y,
        value,
        color,
        medium,
        alpha);
    };

  auto draw_filled_circle =
    [&](const int center_x,
      const int center_y,
      const int radius,
      const ColorRgb color,
      const float alpha)
    {
      const int radius_squared =
        radius * radius;

      for (int dy = -radius; dy <= radius; ++dy) {
        const int inside =
          radius_squared - dy * dy;

        if (inside < 0) {
          continue;
        }

        const int half_width =
          static_cast<int>(
          std::floor(
            std::sqrt(
              static_cast<double>(inside))));

        canvas_.blend_rect(
          center_x - half_width,
          center_y + dy,
          half_width * 2 + 1,
          1,
          color,
          alpha);
      }
    };

  auto draw_arc =
    [&](const int center_x,
      const int center_y,
      const int radius_x,
      const int radius_y,
      const double start_angle,
      const double end_angle,
      const ColorRgb color,
      const float alpha,
      const int thickness)
    {
      constexpr int segments = 72;

      for (int segment = 0; segment <= segments; ++segment) {
        const double progress =
          static_cast<double>(segment) /
          static_cast<double>(segments);

        const double angle =
          start_angle +
          (end_angle - start_angle) *
          progress;

        const int x =
          center_x +
          static_cast<int>(
            std::lround(
              std::cos(angle) *
              static_cast<double>(radius_x)));

        const int y =
          center_y +
          static_cast<int>(
            std::lround(
              std::sin(angle) *
              static_cast<double>(radius_y)));

        canvas_.blend_rect(
          x - thickness / 2,
          y - thickness / 2,
          thickness,
          thickness,
          color,
          alpha);
      }
    };

  const double pulse =
    0.5 +
    0.5 *
    std::sin(
      navigation_animation_time_seconds_ *
      2.2);

  int face_x_offset = 0;
  int face_y_offset = 0;

  if (phase == NavigationPhase::Idle) {
    // Calm breathing movement.
    face_y_offset =
      static_cast<int>(
      std::lround(
        std::sin(
          navigation_animation_time_seconds_ *
          1.7) *
        2.0));
  }

  if (phase == NavigationPhase::Preparing) {
    // Small searching movement while preparing the route.
    face_x_offset =
      static_cast<int>(
      std::lround(
        std::sin(
          navigation_animation_time_seconds_ *
          2.2) *
        3.0));

    face_y_offset =
      static_cast<int>(
      std::lround(
        std::sin(
          navigation_animation_time_seconds_ *
          1.8) *
        2.0));
  }

  if (phase == NavigationPhase::Navigating) {
    // Friendly walking-style bob.
    face_y_offset =
      static_cast<int>(
      std::lround(
        std::sin(
          navigation_animation_time_seconds_ *
          2.4) *
        4.0));
  }

  if (phase == NavigationPhase::Paused) {
    // Very slow breathing while waiting.
    face_y_offset =
      static_cast<int>(
      std::lround(
        std::sin(
          navigation_animation_time_seconds_ *
          1.2) *
        1.0));
  }

  if (phase == NavigationPhase::Arrived) {
    // Happy celebration bounce.
    face_y_offset =
      -static_cast<int>(
      std::lround(
        std::abs(
          std::sin(
            navigation_animation_time_seconds_ *
            4.0)) *
        6.0));
  }

  if (
    phase == NavigationPhase::Error &&
    navigation_phase_elapsed_seconds_ < 1.2)
  {
    // A brief concerned shake, then the face remains stable.
    const double fade =
      1.0 -
      navigation_phase_elapsed_seconds_ /
      1.2;

    face_x_offset =
      static_cast<int>(
      std::lround(
        std::sin(
          navigation_phase_elapsed_seconds_ *
          18.0) *
        4.0 *
        fade));
  }

  std::string title{"NAVIGATION"};
  std::string state = navigation_ui_.state;
  std::string message = navigation_ui_.message;
  ColorRgb phase_color = cyan;

  switch (phase) {
    case NavigationPhase::Idle:
      title = "NAVIGATION READY";
      phase_color = cyan;
      break;

    case NavigationPhase::Preparing:
      title = "GETTING READY";
      phase_color = cyan;
      break;

    case NavigationPhase::Navigating:
      title = "NAVIGATING";
      phase_color = good;
      break;

    case NavigationPhase::Paused:
      title = "NAVIGATION PAUSED";
      phase_color = warning;
      break;

    case NavigationPhase::Arrived:
      title = "ARRIVED";
      phase_color = good;
      break;

    case NavigationPhase::Error:
      title = "NAVIGATION ERROR";
      phase_color = danger;
      break;
  }

  // Remove duplicated inner shell heading.
  canvas_.blend_rect(
    132,
    82,
    618,
    356,
    ColorRgb{0U, 11U, 29U},
    1.0F);

  // Header.
  canvas_.blend_rect(
    132,
    88,
    618,
    34,
    ColorRgb{0U, 25U, 52U},
    0.94F);

  draw_navigation_text(
    148,
    96,
    title,
    phase_color,
    true);

  draw_navigation_text(
    734 -
    navigation_text_width(state, false),
    99,
    state,
    phase_color,
    false);

  // Main face surface.
  canvas_.blend_rect(
    132,
    130,
    618,
    276,
    ColorRgb{0U, 16U, 36U},
    0.94F);

  const int face_center_x =
    441 + face_x_offset;

  const int face_center_y =
    257 + face_y_offset;

  const float glow_alpha =
    static_cast<float>(
      0.18 +
      0.24 * pulse);

  canvas_.draw_circle_ring(
    face_center_x,
    face_center_y,
    116,
    3,
    phase_color,
    glow_alpha);

  if (phase == NavigationPhase::Arrived) {
    const int celebration_radius =
      122 +
      static_cast<int>(
      std::lround(
        8.0 * pulse));

    canvas_.draw_circle_ring(
      face_center_x,
      face_center_y,
      celebration_radius,
      2,
      good,
      static_cast<float>(
        0.18 +
        0.30 * (1.0 - pulse)));
  }

  canvas_.draw_circle_ring(
    face_center_x,
    face_center_y,
    108,
    2,
    phase_color,
    0.68F);

  draw_filled_circle(
    face_center_x,
    face_center_y,
    101,
    face_fill,
    0.98F);

  canvas_.draw_circle_ring(
    face_center_x,
    face_center_y,
    101,
    2,
    cyan_soft,
    0.55F);

  const double blink_position =
    std::fmod(
      navigation_animation_time_seconds_,
      phase == NavigationPhase::Navigating ?
      3.2 :
      4.2);

  const bool blinking =
    blink_position >
    (
      phase == NavigationPhase::Navigating ?
      3.02 :
      4.02
    );

  int eye_offset_x = 0;

  if (phase == NavigationPhase::Preparing) {
    eye_offset_x =
      static_cast<int>(
      std::lround(
        std::sin(
          navigation_animation_time_seconds_ *
          3.2) *
        6.0));
  }

  const int left_eye_x =
    face_center_x - 38 + eye_offset_x;

  const int right_eye_x =
    face_center_x + 38 + eye_offset_x;

  const int eye_y =
    face_center_y - 28;

  // Eyes.
  if (phase == NavigationPhase::Arrived) {
    draw_arc(
      left_eye_x,
      eye_y,
      17,
      10,
      1.15 * pi,
      1.85 * pi,
      good,
      1.0F,
      4);

    draw_arc(
      right_eye_x,
      eye_y,
      17,
      10,
      1.15 * pi,
      1.85 * pi,
      good,
      1.0F,
      4);
  } else if (
    (
      phase == NavigationPhase::Paused &&
      pulse > 0.32
    ) ||
    blinking)
  {
    canvas_.blend_rect(
      left_eye_x - 15,
      eye_y,
      30,
      4,
      phase_color,
      0.98F);

    canvas_.blend_rect(
      right_eye_x - 15,
      eye_y,
      30,
      4,
      phase_color,
      0.98F);
  } else {
    draw_filled_circle(
      left_eye_x,
      eye_y,
      12,
      white,
      0.98F);

    draw_filled_circle(
      right_eye_x,
      eye_y,
      12,
      white,
      0.98F);

    draw_filled_circle(
      left_eye_x + 2,
      eye_y + 1,
      6,
      phase_color,
      0.98F);

    draw_filled_circle(
      right_eye_x + 2,
      eye_y + 1,
      6,
      phase_color,
      0.98F);
  }

  // Cheeks.
  if (
    phase == NavigationPhase::Navigating ||
    phase == NavigationPhase::Arrived)
  {
    draw_filled_circle(
      face_center_x - 69,
      face_center_y + 18,
      9,
      cyan,
      0.20F);

    draw_filled_circle(
      face_center_x + 69,
      face_center_y + 18,
      9,
      cyan,
      0.20F);
  }

  // Mouth.
  if (phase == NavigationPhase::Error) {
    draw_arc(
      face_center_x,
      face_center_y + 60,
      38,
      21,
      1.15 * pi,
      1.85 * pi,
      danger,
      0.98F,
      4);
  } else if (phase == NavigationPhase::Paused) {
    canvas_.blend_rect(
      face_center_x - 27,
      face_center_y + 48,
      54,
      4,
      warning,
      0.98F);
  } else {
    const int smile_height =
      20 +
      static_cast<int>(
      std::lround(
        phase == NavigationPhase::Navigating ?
        3.0 *
        std::sin(
          navigation_animation_time_seconds_ *
          2.8) :
        0.0));

    draw_arc(
      face_center_x,
      face_center_y + 27,
      phase == NavigationPhase::Arrived ?
      51 :
      43,
      phase == NavigationPhase::Arrived ?
      32 :
      smile_height,
      0.15 * pi,
      0.85 * pi,
      phase_color,
      1.0F,
      4);
  }

  // Minimal message only—no map, route or camera data.
  centered_text(
    441,
    375,
    message,
    phase_color,
    true);

  // Bottom instruction strip.
  canvas_.blend_rect(
    132,
    414,
    618,
    23,
    ColorRgb{0U, 23U, 45U},
    0.94F);

  std::string instruction{
    "Ready for a destination."
  };

  switch (phase) {
    case NavigationPhase::Idle:
      instruction =
        "Ready for a destination.";
      break;

    case NavigationPhase::Preparing:
      instruction =
        "Preparing navigation...";
      break;

    case NavigationPhase::Navigating:
      instruction =
        "Savo is guiding you.";
      break;

    case NavigationPhase::Paused:
      instruction =
        "Please wait a moment.";
      break;

    case NavigationPhase::Arrived:
      instruction =
        "Destination reached.";
      break;

    case NavigationPhase::Error:
      instruction =
        navigation_ui_.error_message;
      break;
  }

  draw_navigation_text(
    148,
    416,
    instruction,
    phase == NavigationPhase::Error ?
    danger :
    cyan_soft,
    false);
}


void UiNode::seed_navigation_preview_data(
  const NavigationPhase phase)
{
  navigation_ui_ = NavigationUiState{};
  navigation_phase_ = phase;
  navigation_ui_.live = true;

  switch (phase) {
    case NavigationPhase::Idle:
      navigation_ui_.state = "IDLE";
      navigation_ui_.message =
        "Where would you like to go?";
      break;

    case NavigationPhase::Preparing:
      navigation_ui_.state = "PREPARING";
      navigation_ui_.message =
        "Getting ready...";
      break;

    case NavigationPhase::Navigating:
      navigation_ui_.state = "NAVIGATING";
      navigation_ui_.message =
        "Taking you there...";
      break;

    case NavigationPhase::Paused:
      navigation_ui_.state = "PAUSED";
      navigation_ui_.message =
        "Please wait...";
      break;

    case NavigationPhase::Arrived:
      navigation_ui_.state = "ARRIVED";
      navigation_ui_.message =
        "We have arrived!";
      break;

    case NavigationPhase::Error:
      navigation_ui_.state = "ERROR";
      navigation_ui_.message =
        "I need a moment.";
      navigation_ui_.error_message =
        "Navigation is temporarily unavailable.";
      break;
  }
}

void UiNode::render_status_page()
{
  render_page_shell(status_shell_, "Status");

  const ColorRgb cyan{70U, 235U, 255U};
  const ColorRgb cyan_soft{145U, 210U, 230U};
  const ColorRgb white{235U, 248U, 255U};

  constexpr double status_tau =
    6.28318530717958647692;

  const double status_pulse =
    0.5 +
    0.5 *
    std::sin(
      status_animation_time_seconds_ *
      status_tau /
      1.60);

  const float status_live_alpha =
    static_cast<float>(
      0.68 +
      0.32 * status_pulse);

  const float status_tab_alpha =
    static_cast<float>(
      0.84 +
      0.12 * status_pulse);

  const float status_alert_alpha =
    status_ui_.alert_count > 0 ?
    static_cast<float>(
      0.58 +
      0.42 * status_pulse) :
    0.98F;
  const ColorRgb good{75U, 235U, 175U};
  const ColorRgb warning{255U, 190U, 70U};
  const ColorRgb danger{255U, 90U, 95U};
  const ColorRgb muted{135U, 165U, 185U};

  auto draw_status_text =
    [this](
      const int x,
      const int y,
      const std::string & value,
      const ColorRgb color,
      const bool medium,
      const float alpha = 1.0F)
    {
      if (medium) {
        Font::draw_atlas_text(
          canvas_,
          ui_font_medium_,
          20,
          24,
          16,
          2,
          x,
          y,
          value,
          color,
          alpha);
        return;
      }

      Font::draw_atlas_text(
        canvas_,
        ui_font_small_,
        18,
        22,
        16,
        2,
        x,
        y,
        value,
        color,
        alpha);
    };

  auto status_text_width =
    [this](
      const std::string & value,
      const bool medium) -> int
    {
      if (medium) {
        return Font::atlas_text_width(
          ui_font_medium_,
          20,
          24,
          16,
          value);
      }

      return Font::atlas_text_width(
        ui_font_small_,
        18,
        22,
        16,
        value);
    };

  auto status_severity =
    [](const std::string & raw_state) -> int
    {
      const std::string state = uppercase_copy(raw_state);

      if (
        state == "FAULT" ||
        state == "FAILED" ||
        state == "ERROR" ||
        state == "OFFLINE")
      {
        return 4;
      }

      if (
        state == "STOP" ||
        state == "STOPPED" ||
        state == "BLOCKED")
      {
        return 3;
      }

      if (
        state == "DEGRADED" ||
        state == "STALE" ||
        state == "SLOW" ||
        state == "RECOVERY" ||
        state == "WAITING" ||
        state == "WARNING")
      {
        return 2;
      }

      if (
        state == "MISSING" ||
        state == "UNKNOWN" ||
        state == "--")
      {
        return 1;
      }

      return 0;
    };

  auto status_color =
    [&](const std::string & state) -> ColorRgb
    {
      const int severity = status_severity(state);

      if (severity >= 3) {
        return danger;
      }

      if (severity == 2) {
        return warning;
      }

      if (severity == 1) {
        return muted;
      }

      return good;
    };

  auto combined_state =
    [&](const std::string & first,
      const std::string & second,
      const std::string & healthy_text) -> std::string
    {
      const int first_severity = status_severity(first);
      const int second_severity = status_severity(second);

      if (first_severity == 0 && second_severity == 0) {
        return healthy_text;
      }

      return first_severity >= second_severity ?
             uppercase_copy(first) :
             uppercase_copy(second);
    };

  // Remove the shell's duplicated internal heading.
  canvas_.blend_rect(
    132,
    82,
    618,
    356,
    ColorRgb{0U, 11U, 29U},
    1.0F);

  // ============================================================
  // Shared Status tabs
  // ============================================================
  struct TabVisual
  {
    int x;
    int width;
    StatusView view;
    const char * label;
  };

  const std::array<TabVisual, 4> tabs{{
    {138, 145, StatusView::Overview, "OVERVIEW"},
    {287, 145, StatusView::Sensors, "SENSORS"},
    {436, 145, StatusView::AiLink, "AI / LINK"},
    {585, 145, StatusView::AlertsSystem, "ALERTS"},
  }};

  for (const auto & tab : tabs) {
    const bool selected = status_view_ == tab.view;

    canvas_.blend_rect(
      tab.x,
      84,
      tab.width,
      28,
      selected ?
      ColorRgb{0U, 78U, 105U} :
      ColorRgb{0U, 20U, 43U},
      selected ? status_tab_alpha : 0.82F);

    if (selected) {
      canvas_.blend_rect(
        tab.x,
        110,
        tab.width,
        2,
        cyan,
        static_cast<float>(
          0.72 +
          0.26 * status_pulse));

      const int glow_width =
        34 +
        static_cast<int>(
        std::lround(
          34.0 * status_pulse));

      const int glow_x =
        tab.x +
        ((tab.width - glow_width) / 2);

      canvas_.blend_rect(
        glow_x,
        109,
        glow_width,
        3,
        white,
        static_cast<float>(
          0.10 +
          0.18 * status_pulse));
    }

    const std::string label{tab.label};
    const int width =
      status_text_width(label, false);

    draw_status_text(
      tab.x + ((tab.width - width) / 2),
      91,
      label,
      selected ? white : cyan_soft,
      false,
      selected ? 1.0F : 0.88F);
  }

  auto draw_alert_strip =
    [&]()
    {
      constexpr int alert_x = 132;
      constexpr int alert_y = 414;
      constexpr int alert_w = 618;
      constexpr int alert_h = 23;

      canvas_.blend_rect(
        alert_x,
        alert_y,
        alert_w,
        alert_h,
        ColorRgb{0U, 23U, 45U},
        0.94F);

      if (status_ui_.alert_count > 0) {
        canvas_.blend_rect(
          alert_x,
          alert_y,
          alert_w,
          2,
          warning,
          static_cast<float>(
            0.24 +
            0.50 * status_pulse));
      }

      const std::string label = "ACTIVE ALERT:";

      draw_status_text(
        alert_x + 12,
        alert_y + 2,
        label,
        cyan_soft,
        false,
        0.96F);

      const int value_x =
        alert_x + 12 +
        status_text_width(label, false) + 14;

      draw_status_text(
        value_x,
        alert_y + 2,
        status_ui_.active_alert,
        status_ui_.alert_count == 0 ? good : warning,
        false,
        status_alert_alpha);
    };

  // ============================================================
  // OVERVIEW
  // ============================================================
  if (status_view_ == StatusView::Overview) {
    constexpr int summary_x = 132;
    constexpr int summary_y = 122;
    constexpr int summary_w = 618;
    constexpr int summary_h = 30;

    canvas_.blend_rect(
      summary_x,
      summary_y,
      summary_w,
      summary_h,
      ColorRgb{0U, 25U, 52U},
      0.92F);

    draw_status_text(
      146,
      summary_y + 8,
      "ROBOT",
      cyan_soft,
      false);

    draw_status_text(
      204,
      summary_y + 8,
      status_ui_.robot_state,
      status_color(status_ui_.robot_state),
      false);

    draw_status_text(
      322,
      summary_y + 8,
      "MODE",
      cyan_soft,
      false);

    draw_status_text(
      378,
      summary_y + 8,
      status_ui_.operating_mode,
      white,
      false);

    draw_status_text(
      486,
      summary_y + 8,
      "ALERTS",
      cyan_soft,
      false);

    draw_status_text(
      558,
      summary_y + 8,
      std::to_string(status_ui_.alert_count),
      status_ui_.alert_count == 0 ? good : warning,
      false);

    const std::string live =
      status_ui_.live ? "LIVE" : "WAITING";

    draw_status_text(
      summary_x + summary_w -
      status_text_width(live, false) - 14,
      summary_y + 8,
      live,
      status_ui_.live ? cyan : warning,
      false,
      status_ui_.live ?
      status_live_alpha :
      0.95F);

    auto draw_domain_card =
      [&](const int x,
        const int y,
        const std::string & title,
        const std::string & state,
        const std::array<std::string, 5> & lines)
      {
        constexpr int width = 302;
        constexpr int height = 118;

        const ColorRgb card_color =
          status_color(state);

        canvas_.blend_rect(
          x,
          y,
          width,
          height,
          ColorRgb{0U, 18U, 40U},
          0.94F);

        canvas_.blend_rect(
          x,
          y,
          width,
          2,
          card_color,
          0.92F);

        draw_status_text(
          x + 11,
          y + 8,
          title,
          cyan_soft,
          false);

        const int state_width =
          status_text_width(state, false);

        draw_status_text(
          x + width - state_width - 11,
          y + 8,
          state,
          card_color,
          false);

        const std::array<int, 5> rows{
          y + 34,
          y + 51,
          y + 68,
          y + 85,
          y + 102
        };

        for (std::size_t index = 0; index < lines.size(); ++index) {
          draw_status_text(
            x + 11,
            rows[index],
            lines[index],
            index == 0 ? white : muted,
            false,
            0.94F);
        }
      };

    const std::string motion_state =
      combined_state(
      status_ui_.safety_state,
      status_ui_.control_state,
      "CLEAR");

    const std::string localization_state =
      combined_state(
      status_ui_.localization_state,
      status_ui_.perception_state,
      "READY");

    draw_domain_card(
      132,
      160,
      "MOTION + SAFETY",
      motion_state,
      {{
        "Safety Gate   " + status_ui_.safety_gate,
        "Base          " + status_ui_.base_state,
        "Control       " + status_ui_.control_mode,
        "Watchdog      " + status_ui_.watchdog_state,
        "Slowdown      " + status_ui_.slowdown,
      }});

    draw_domain_card(
      448,
      160,
      "LOCALIZATION + PERCEPTION",
      localization_state,
      {{
        "EKF           " + status_ui_.ekf_state,
        "Wheel         " + status_ui_.wheel_state,
        "IMU           " + status_ui_.imu_state,
        "VO            " + status_ui_.vo_state,
        "Perception    " + status_ui_.perception_state,
      }});

    draw_domain_card(
      132,
      286,
      "NAVIGATION + MAPPING",
      status_ui_.navigation_state,
      {{
        "Navigation    " + status_ui_.nav_state,
        "Mapping       " + status_ui_.mapping_state,
        "Recovery      " + status_ui_.recovery_state,
        "Goal          " + status_ui_.navigation_goal,
        "State         " + status_ui_.navigation_state,
      }});

    draw_domain_card(
      448,
      286,
      "AI + CONNECTIVITY",
      status_ui_.connectivity_state,
      {{
        "SavoMind      " + status_ui_.savomind_state,
        "Speech        " + status_ui_.speech_state,
        "Core-Edge     " + status_ui_.core_edge_state,
        "ROS Link      " + status_ui_.link_state,
        "Internet      " + status_ui_.internet_state,
      }});

    draw_alert_strip();
    return;
  }

  // ============================================================
  // SENSORS
  // ============================================================
  if (status_view_ == StatusView::Sensors) {
    bool nearest_seen = false;
    double nearest_distance = 0.0;
    std::string nearest_label{"None"};

    for (const auto & sensor : status_ui_.obstacle_distances) {
      if (
        !sensor.seen ||
        !sensor.valid ||
        sensor.clear_out_of_range)
      {
        continue;
      }

      if (
        !nearest_seen ||
        sensor.distance_m < nearest_distance)
      {
        nearest_seen = true;
        nearest_distance = sensor.distance_m;
        nearest_label = sensor.label;
      }
    }

    std::ostringstream nearest_value;

    if (nearest_seen) {
      nearest_value
        << nearest_label
        << "  "
        << std::fixed
        << std::setprecision(2)
        << nearest_distance
        << " m";
    } else {
      nearest_value << "No nearby obstacle";
    }

    canvas_.blend_rect(
      132,
      122,
      618,
      34,
      ColorRgb{0U, 25U, 52U},
      0.92F);

    draw_status_text(
      146,
      131,
      "NEAREST OBSTACLE",
      cyan_soft,
      false);

    draw_status_text(
      316,
      131,
      nearest_value.str(),
      nearest_seen ? white : good,
      false);

    const std::string live =
      status_ui_.live ? "LIVE" : "WAITING";

    draw_status_text(
      736 - status_text_width(live, false),
      131,
      live,
      status_ui_.live ? cyan : warning,
      false,
      status_ui_.live ?
      status_live_alpha :
      0.95F);

    constexpr int graph_x = 132;
    constexpr int graph_y = 164;
    constexpr int graph_w = 618;
    constexpr int graph_h = 240;
    constexpr double graph_max_m = 3.0;

    canvas_.blend_rect(
      graph_x,
      graph_y,
      graph_w,
      graph_h,
      ColorRgb{0U, 15U, 35U},
      0.94F);

    draw_status_text(
      graph_x + 14,
      graph_y + 10,
      "LIVE OBSTACLE DISTANCE",
      cyan,
      true);

    auto sensor_color =
      [&](const ObstacleDistanceUiState & sensor) -> ColorRgb
      {
        if (!sensor.seen || !sensor.valid) {
          return muted;
        }

        return status_color(sensor.state);
      };

    auto value_text =
      [](const ObstacleDistanceUiState & sensor) -> std::string
      {
        if (!sensor.seen) {
          return "--";
        }

        if (!sensor.valid) {
          return "INVALID";
        }

        if (sensor.clear_out_of_range) {
          return ">3.0 m";
        }

        std::ostringstream output;
        output
          << std::fixed
          << std::setprecision(2)
          << sensor.distance_m
          << " m";

        return output.str();
      };

    const std::array<int, 5> rows{
      211,
      243,
      275,
      307,
      339
    };

    for (
      std::size_t index = 0;
      index < status_ui_.obstacle_distances.size();
      ++index)
    {
      const auto & sensor =
        status_ui_.obstacle_distances[index];

      const ColorRgb row_color =
        sensor_color(sensor);

      draw_status_text(
        150,
        rows[index],
        sensor.label,
        cyan_soft,
        false);

      canvas_.blend_rect(
        292,
        rows[index] + 6,
        250,
        8,
        ColorRgb{20U, 55U, 72U},
        0.94F);

      if (sensor.seen && sensor.valid) {
        const double displayed =
          sensor.clear_out_of_range ?
          graph_max_m :
          std::clamp(
            sensor.distance_m,
            0.0,
            graph_max_m);

        const int width =
          static_cast<int>(
          std::lround(
            250.0 *
            displayed /
            graph_max_m));

        const int bounded_width =
          std::clamp(width, 0, 250);

        canvas_.blend_rect(
          292,
          rows[index] + 6,
          bounded_width,
          8,
          row_color,
          0.98F);

        if (
          status_ui_.live &&
          bounded_width > 14)
        {
          const double shimmer_phase =
            std::fmod(
            status_animation_time_seconds_ *
            0.42 +
            static_cast<double>(index) *
            0.17,
            1.0);

          constexpr int shimmer_width = 7;

          const int shimmer_x =
            292 +
            static_cast<int>(
            std::lround(
              shimmer_phase *
              static_cast<double>(
                bounded_width -
                shimmer_width)));

          canvas_.blend_rect(
            shimmer_x,
            rows[index] + 6,
            shimmer_width,
            8,
            white,
            static_cast<float>(
              0.10 +
              0.18 * status_pulse));
        }
      }

      draw_status_text(
        556,
        rows[index],
        value_text(sensor),
        sensor.seen && sensor.valid ?
        white :
        muted,
        false);

      const std::string state =
        sensor.seen ?
        uppercase_copy(sensor.state) :
        "MISSING";

      draw_status_text(
        736 - status_text_width(state, false),
        rows[index],
        state,
        row_color,
        false);
    }

    draw_status_text(
      292,
      378,
      "0.0 m",
      muted,
      false);

    const std::string max_label = "3.0 m";

    draw_status_text(
      542 - status_text_width(max_label, false),
      378,
      max_label,
      muted,
      false);

    draw_alert_strip();
    return;
  }

  // ============================================================
  // AI / LINK
  // ============================================================
  if (status_view_ == StatusView::AiLink) {
    auto draw_section =
      [&](const int x,
        const std::string & title,
        const std::array<std::string, 4> & labels,
        const std::array<std::string, 4> & values)
      {
        constexpr int y = 122;
        constexpr int width = 202;
        constexpr int height = 282;

        canvas_.blend_rect(
          x,
          y,
          width,
          height,
          ColorRgb{0U, 18U, 40U},
          0.94F);

        canvas_.blend_rect(
          x,
          y,
          width,
          2,
          cyan,
          0.90F);

        draw_status_text(
          x + 12,
          y + 11,
          title,
          cyan,
          true);

        const std::array<int, 4> label_rows{
          y + 45,
          y + 99,
          y + 153,
          y + 207
        };

        for (std::size_t index = 0; index < labels.size(); ++index) {
          draw_status_text(
            x + 12,
            label_rows[index],
            labels[index],
            muted,
            false);

          draw_status_text(
            x + 12,
            label_rows[index] + 18,
            values[index],
            status_color(values[index]),
            false);
        }
      };

    draw_section(
      132,
      "SAVOMIND",
      {{
        "BRAIN SERVICE",
        "INTENT ENGINE",
        "LLM PROVIDER",
        "MODEL",
      }},
      {{
        status_ui_.brain_service_state,
        status_ui_.intent_engine_state,
        status_ui_.llm_provider,
        status_ui_.llm_model,
      }});

    draw_section(
      340,
      "SPEECH",
      {{
        "MICROPHONE",
        "STT ROUTE",
        "TTS ROUTE",
        "PLAYBACK",
      }},
      {{
        status_ui_.microphone_state,
        status_ui_.stt_route_state,
        status_ui_.tts_route_state,
        status_ui_.playback_state,
      }});

    draw_section(
      548,
      "CONNECTIVITY",
      {{
        "CORE-EDGE",
        "ROS DISCOVERY",
        "INTERNET",
        "LINK STATE",
      }},
      {{
        status_ui_.core_edge_state,
        status_ui_.ros_discovery_state,
        status_ui_.internet_state,
        status_ui_.link_state,
      }});

    draw_alert_strip();
    return;
  }

  // ============================================================
  // ALERTS / SYSTEM
  // ============================================================
  canvas_.blend_rect(
    132,
    122,
    292,
    282,
    ColorRgb{0U, 18U, 40U},
    0.94F);

  canvas_.blend_rect(
    432,
    122,
    318,
    282,
    ColorRgb{0U, 18U, 40U},
    0.94F);

  canvas_.blend_rect(
    132,
    122,
    292,
    2,
    status_ui_.alert_count == 0 ? good : warning,
    0.94F);

  canvas_.blend_rect(
    432,
    122,
    318,
    2,
    cyan,
    0.94F);

  draw_status_text(
    145,
    133,
    "ACTIVE ALERTS",
    cyan,
    true);

  const std::string alert_count =
    std::to_string(status_ui_.alert_count);

  draw_status_text(
    407 - status_text_width(alert_count, true),
    133,
    alert_count,
    status_ui_.alert_count == 0 ? good : warning,
    true);

  if (status_ui_.alert_count == 0) {
    draw_status_text(
      158,
      196,
      "NO ACTIVE ALERTS",
      good,
      true);

    draw_status_text(
      158,
      232,
      "Robot systems are stable.",
      white,
      false);

    draw_status_text(
      158,
      256,
      "No immediate operator action.",
      muted,
      false);
  } else {
    draw_status_text(
      151,
      188,
      status_ui_.active_alert,
      warning,
      true);

    draw_status_text(
      151,
      224,
      "SEVERITY",
      muted,
      false);

    draw_status_text(
      151,
      244,
      "WARNING",
      warning,
      false);

    draw_status_text(
      151,
      282,
      "Operator review recommended.",
      white,
      false);
  }

  draw_status_text(
    445,
    133,
    "SYSTEM HEALTH",
    cyan,
    true);

  auto format_temperature =
    [](const double temperature) -> std::string
    {
      std::ostringstream output;
      output
        << std::fixed
        << std::setprecision(0)
        << temperature
        << " C";
      return output.str();
    };

  const std::array<std::string, 8> system_labels{{
    "Core Computer",
    "Edge Computer",
    "ROS Nodes",
    "Critical Nodes",
    "Stale Publishers",
    "Core / Edge CPU",
    "Core / Edge Memory",
    "Core / Edge Storage",
  }};

  const std::array<std::string, 8> system_values{{
    status_ui_.core_computer_state,
    status_ui_.edge_computer_state,
    std::to_string(status_ui_.ros_nodes_ready) +
      " / " +
      std::to_string(status_ui_.ros_nodes_total),
    status_ui_.critical_nodes_state,
    std::to_string(status_ui_.stale_publishers),
    format_temperature(status_ui_.core_cpu_temp_c) +
      " / " +
      format_temperature(status_ui_.edge_cpu_temp_c),
    std::to_string(status_ui_.core_memory_percent) +
      "% / " +
      std::to_string(status_ui_.edge_memory_percent) +
      "%",
    std::to_string(status_ui_.core_storage_percent) +
      "% / " +
      std::to_string(status_ui_.edge_storage_percent) +
      "%",
  }};

  const std::array<int, 8> system_rows{
    166,
    193,
    220,
    247,
    274,
    301,
    328,
    355
  };

  for (
    std::size_t index = 0;
    index < system_labels.size();
    ++index)
  {
    draw_status_text(
      447,
      system_rows[index],
      system_labels[index],
      muted,
      false);

    draw_status_text(
      625,
      system_rows[index],
      system_values[index],
      index < 5 ?
      status_color(system_values[index]) :
      white,
      false);
  }

  draw_alert_strip();
}


void UiNode::seed_status_preview_data()
{
  status_ui_ = StatusUiState{};

  status_ui_.robot_state = "READY";
  status_ui_.operating_mode = "IDLE";
  status_ui_.alert_count = 0;
  status_ui_.live = true;

  status_ui_.safety_state = "CLEAR";
  status_ui_.safety_gate = "Allowed";
  status_ui_.slowdown = "100%";

  status_ui_.control_state = "OK";
  status_ui_.base_state = "OK";
  status_ui_.control_mode = "Idle";
  status_ui_.watchdog_state = "OK";

  status_ui_.localization_state = "READY";
  status_ui_.ekf_state = "Ready";
  status_ui_.wheel_state = "OK";
  status_ui_.imu_state = "OK";
  status_ui_.vo_state = "OK";

  status_ui_.perception_state = "OK";
  status_ui_.lidar_state = "OK";
  status_ui_.tof_state = "OK";
  status_ui_.ultrasonic_state = "OK";
  status_ui_.depth_state = "OK";

  status_ui_.navigation_state = "STANDBY";
  status_ui_.nav_state = "Standby";
  status_ui_.mapping_state = "Idle";
  status_ui_.recovery_state = "Ready";
  status_ui_.navigation_goal = "None";

  status_ui_.connectivity_state = "ONLINE";
  status_ui_.savomind_state = "Online";
  status_ui_.speech_state = "Ready";
  status_ui_.link_state = "OK";

  status_ui_.brain_service_state = "READY";
  status_ui_.intent_engine_state = "READY";
  status_ui_.llm_provider = "CEREBRAS";
  status_ui_.llm_model = "GPT-OSS-120B";

  status_ui_.microphone_state = "READY";
  status_ui_.stt_route_state = "ONLINE";
  status_ui_.tts_route_state = "ONLINE";
  status_ui_.playback_state = "IDLE";

  status_ui_.core_edge_state = "ONLINE";
  status_ui_.ros_discovery_state = "OK";
  status_ui_.internet_state = "ONLINE";

  status_ui_.core_computer_state = "OK";
  status_ui_.edge_computer_state = "OK";
  status_ui_.critical_nodes_state = "OK";

  status_ui_.ros_nodes_ready = 24;
  status_ui_.ros_nodes_total = 24;
  status_ui_.stale_publishers = 0;

  status_ui_.core_cpu_temp_c = 51.0;
  status_ui_.edge_cpu_temp_c = 49.0;

  status_ui_.core_memory_percent = 38;
  status_ui_.edge_memory_percent = 31;
  status_ui_.core_storage_percent = 62;
  status_ui_.edge_storage_percent = 58;

  status_ui_.obstacle_distances = {{
    {"LiDAR Front", true, true, false, 2.42, "CLEAR"},
    {"ToF Left", true, true, false, 1.48, "CLEAR"},
    {"ToF Right", true, true, false, 1.82, "CLEAR"},
    {"Ultrasonic", true, true, false, 2.16, "CLEAR"},
    {"Depth Front", true, true, false, 0.68, "SLOW"},
  }};

  status_ui_.active_alert = "None";
}


void UiNode::render_power_page()
{
  render_page_shell(power_shell_, "Power");

  const ColorRgb cyan{70U, 235U, 255U};
  const ColorRgb cyan_soft{135U, 205U, 225U};
  const ColorRgb good{75U, 235U, 175U};
  const ColorRgb warning{255U, 190U, 70U};
  const ColorRgb danger{255U, 90U, 95U};
  const ColorRgb muted{145U, 175U, 195U};
  const ColorRgb white{235U, 248U, 255U};

  constexpr double power_tau =
    6.28318530717958647692;

  const double power_pulse =
    0.5 +
    0.5 *
    std::sin(
      power_animation_time_seconds_ *
      power_tau /
      1.55);

  const float power_live_alpha =
    static_cast<float>(
      0.68 +
      0.32 * power_pulse);

  auto draw_power_text =
    [this](
      const int x,
      const int y,
      const std::string & value,
      const ColorRgb color,
      const int scale,
      const float alpha)
    {
      if (scale >= 3) {
        Font::draw_atlas_text(
          canvas_,
          ui_font_large_bold_,
          32,
          36,
          16,
          2,
          x,
          y,
          value,
          color,
          alpha);
        return;
      }

      Font::draw_atlas_text(
        canvas_,
        ui_font_medium_,
        20,
        24,
        16,
        2,
        x,
        y,
        value,
        color,
        alpha);
    };

  auto power_text_width =
    [this](
      const std::string & value,
      const int scale) -> int
    {
      if (scale >= 3) {
        return Font::atlas_text_width(
          ui_font_large_bold_,
          32,
          36,
          16,
          value);
      }

      return Font::atlas_text_width(
        ui_font_medium_,
        20,
        24,
        16,
        value);
    };

  auto state_color =
    [&](const PowerUiSourceState & source) -> ColorRgb
    {
      if (!source.seen) {
        return muted;
      }

      if (power_source_is_stale(source)) {
        return warning;
      }

      const std::string state = lowercase_copy(source.state);

      if (
        state == "ok" ||
        state == "full" ||
        state == "charging")
      {
        return good;
      }

      if (state == "low") {
        return warning;
      }

      return danger;
    };

  auto source_state_text =
    [&](const PowerUiSourceState & source) -> std::string
    {
      if (!source.seen) {
        return "MISSING";
      }

      if (power_source_is_stale(source)) {
        return "STALE";
      }

      return uppercase_copy(source.state);
    };

  auto voltage_text =
    [](const PowerUiSourceState & source) -> std::string
    {
      if (!source.has_voltage) {
        return "--.-- V";
      }

      std::ostringstream output;
      output << std::fixed << std::setprecision(2)
             << source.voltage_v << " V";
      return output.str();
    };

  auto percent_text =
    [](const PowerUiSourceState & source) -> std::string
    {
      if (!source.has_percent) {
        return source.percent_label + " --.-%";
      }

      std::ostringstream output;
      output << source.percent_label << " "
             << std::fixed << std::setprecision(1)
             << source.percent << "%";
      return output.str();
    };

  // --------------------------------------------------------------
  // Overall status bar
  // --------------------------------------------------------------
  const int summary_x = 132;
  const int summary_y = 145;
  const int summary_w = 618;
  const int summary_h = 30;

  canvas_.blend_rect(
    summary_x,
    summary_y,
    summary_w,
    summary_h,
    ColorRgb{0U, 25U, 52U},
    0.88F);

  bool status_stale = false;

  if (power_status_seen_) {
    const double age_seconds =
      std::chrono::duration<double>(
      std::chrono::steady_clock::now() -
      power_status_last_update_).count();

    status_stale =
      age_seconds > config_.power_stale_timeout_s;
  }

  const std::string overall_text =
    !power_status_seen_ ?
    "WAITING" :
    status_stale ?
    "STALE" :
    uppercase_copy(power_overall_state_);

  const std::string health_text =
    !power_status_seen_ ?
    "--" :
    status_stale ?
    "STALE" :
    uppercase_copy(power_health_state_);

  ColorRgb overall_color = muted;

  if (!power_status_seen_ || status_stale) {
    overall_color = warning;
  } else if (
    power_overall_state_ == "ok" ||
    power_overall_state_ == "full" ||
    power_overall_state_ == "charging")
  {
    overall_color = good;
  } else if (power_overall_state_ == "low") {
    overall_color = warning;
  } else {
    overall_color = danger;
  }

  draw_power_text(
    summary_x + 14,
    summary_y + 6,
    "SYSTEM POWER",
    white,
    2,
    0.92F);

  draw_power_text(
    summary_x + 160,
    summary_y + 6,
    overall_text,
    overall_color,
    2,
    1.0F);

  draw_power_text(
    summary_x + 350,
    summary_y + 6,
    "HEALTH",
    cyan_soft,
    2,
    0.90F);

  draw_power_text(
    summary_x + 438,
    summary_y + 6,
    health_text,
    overall_color,
    2,
    1.0F);

  draw_power_text(
    summary_x + 548,
    summary_y + 6,
    status_stale ? "STALE" : "LIVE",
    status_stale ? warning : cyan,
    2,
    status_stale ?
    0.95F :
    power_live_alpha);

  // --------------------------------------------------------------
  // Three source cards
  // --------------------------------------------------------------
  auto draw_source_card =
    [&](const int x,
      const std::string & title,
      const PowerUiSourceState & source)
    {
      const int y = 184;
      const int w = 194;
      const int h = 99;

      const ColorRgb source_color = state_color(source);
      const std::string state_text = source_state_text(source);

      const bool source_live =
        source.seen &&
        !power_source_is_stale(source);

      const bool source_charging =
        source_live &&
        lowercase_copy(source.state) == "charging";

      const float source_activity_alpha =
        source_live ?
        static_cast<float>(
          0.62 +
          0.30 * power_pulse) :
        0.85F;

      canvas_.blend_rect(
        x,
        y,
        w,
        h,
        ColorRgb{0U, 19U, 42U},
        0.92F);

      canvas_.blend_rect(
        x,
        y,
        w,
        2,
        source_color,
        source_activity_alpha);

      if (source_live) {
        const int activity_width =
          36 +
          static_cast<int>(
          std::lround(
            42.0 * power_pulse));

        canvas_.blend_rect(
          x + ((w - activity_width) / 2),
          y,
          activity_width,
          2,
          white,
          static_cast<float>(
            0.08 +
            0.16 * power_pulse));
      }

      draw_power_text(
        x + 12,
        y + 11,
        title,
        cyan_soft,
        2,
        0.95F);

      const int state_width = power_text_width(state_text, 2);

      draw_power_text(
        x + w - state_width - 10,
        y + 11,
        state_text,
        source_color,
        2,
        1.0F);

      draw_power_text(
        x + 12,
        y + 36,
        voltage_text(source),
        white,
        3,
        1.0F);

      draw_power_text(
        x + 12,
        y + 68,
        percent_text(source),
        source.has_percent ? cyan : muted,
        2,
        0.95F);

      const int bar_x = x + 12;
      const int bar_y = y + 88;
      const int bar_w = w - 24;

      canvas_.blend_rect(
        bar_x,
        bar_y,
        bar_w,
        5,
        ColorRgb{20U, 55U, 72U},
        0.90F);

      if (source.has_percent) {
        const int fill_width =
          static_cast<int>(
          std::lround(
            static_cast<double>(bar_w) *
            source.percent / 100.0));

        const int bounded_fill_width =
          std::clamp(fill_width, 0, bar_w);

        canvas_.blend_rect(
          bar_x,
          bar_y,
          bounded_fill_width,
          5,
          source_color,
          0.95F);

        if (
          source_charging &&
          bounded_fill_width > 12)
        {
          const double charge_phase =
            std::fmod(
            power_animation_time_seconds_ *
            0.58,
            1.0);

          constexpr int charge_width = 8;

          const int charge_x =
            bar_x +
            static_cast<int>(
            std::lround(
              charge_phase *
              static_cast<double>(
                bounded_fill_width -
                charge_width)));

          canvas_.blend_rect(
            charge_x,
            bar_y,
            charge_width,
            5,
            white,
            static_cast<float>(
              0.18 +
              0.30 * power_pulse));
        }
      }
    };

  draw_source_card(132, "CORE UPS", core_power_);
  draw_source_card(344, "EDGE UPS", edge_power_);
  draw_source_card(556, "BASE", base_power_);

  // --------------------------------------------------------------
  // Voltage-history dashboard
  // --------------------------------------------------------------
  const int graph_panel_x = 132;
  const int graph_panel_y = 294;
  const int graph_panel_w = 618;
  const int graph_panel_h = 150;

  canvas_.blend_rect(
    graph_panel_x,
    graph_panel_y,
    graph_panel_w,
    graph_panel_h,
    ColorRgb{0U, 15U, 35U},
    0.90F);

  draw_power_text(
    graph_panel_x + 14,
    graph_panel_y + 11,
    "VOLTAGE HISTORY",
    cyan,
    2,
    0.95F);

  draw_power_text(
    graph_panel_x + 476,
    graph_panel_y + 11,
    "ROLLING 120 S",
    muted,
    2,
    0.85F);

  auto draw_line =
    [&](int x0,
      int y0,
      const int x1,
      const int y1,
      const ColorRgb color,
      const float alpha)
    {
      const int dx = std::abs(x1 - x0);
      const int sx = x0 < x1 ? 1 : -1;
      const int dy = -std::abs(y1 - y0);
      const int sy = y0 < y1 ? 1 : -1;

      int error = dx + dy;

      while (true) {
        canvas_.blend_pixel(x0, y0, color, alpha);

        if (x0 == x1 && y0 == y1) {
          break;
        }

        const int doubled_error = 2 * error;

        if (doubled_error >= dy) {
          error += dy;
          x0 += sx;
        }

        if (doubled_error <= dx) {
          error += dx;
          y0 += sy;
        }
      }
    };

  auto draw_voltage_series =
    [&](const PowerUiSourceState & source,
      const std::string & label,
      const double min_voltage,
      const double max_voltage,
      const double low_voltage,
      const int row_y,
      const ColorRgb line_color)
    {
      const int label_x = graph_panel_x + 14;
      const int value_x = graph_panel_x + 82;
      const int plot_x = graph_panel_x + 158;
      const int plot_y = row_y + 4;
      const int plot_w = graph_panel_w - 172;
      const int plot_h = 28;

      draw_power_text(
        label_x,
        row_y + 8,
        label,
        cyan_soft,
        2,
        0.90F);

      std::string current_value = "--.--V";

      if (source.has_voltage) {
        std::ostringstream output;
        output << std::fixed << std::setprecision(2)
               << source.voltage_v << "V";
        current_value = output.str();
      }

      draw_power_text(
        value_x,
        row_y + 8,
        current_value,
        state_color(source),
        2,
        0.95F);

      canvas_.blend_rect(
        plot_x,
        plot_y,
        plot_w,
        plot_h,
        ColorRgb{0U, 8U, 22U},
        0.70F);

      const double low_normalized =
        std::clamp(
          (low_voltage - min_voltage) /
          (max_voltage - min_voltage),
          0.0,
          1.0);

      const int low_y =
        plot_y + plot_h - 3 -
        static_cast<int>(
          std::lround(
            low_normalized *
            static_cast<double>(plot_h - 6)));

      draw_line(
        plot_x,
        low_y,
        plot_x + plot_w - 1,
        low_y,
        warning,
        0.28F);

      const auto & history = source.voltage_history;

      if (history.empty()) {
        draw_power_text(
          plot_x + plot_w / 2 - 42,
          plot_y + 8,
          "NO DATA",
          muted,
          2,
          0.65F);
        return;
      }

      int previous_x = plot_x;
      int previous_y = plot_y + plot_h / 2;

      for (std::size_t i = 0; i < history.size(); ++i) {
        const double normalized =
          std::clamp(
            (history[i] - min_voltage) /
            (max_voltage - min_voltage),
            0.0,
            1.0);

        const int x =
          history.size() <= 1U ?
          plot_x + plot_w - 2 :
          plot_x +
          static_cast<int>(
            (static_cast<double>(i) /
            static_cast<double>(history.size() - 1U)) *
            static_cast<double>(plot_w - 2));

        const int y =
          plot_y + plot_h - 3 -
          static_cast<int>(
            std::lround(
              normalized *
              static_cast<double>(plot_h - 6)));

        if (i == 0U) {
          previous_x = x;
          previous_y = y;
          canvas_.blend_rect(
            x - 1,
            y - 1,
            3,
            3,
            line_color,
            0.95F);
          continue;
        }

        draw_line(
          previous_x,
          previous_y,
          x,
          y,
          line_color,
          0.95F);

        previous_x = x;
        previous_y = y;
      }
    };

  draw_voltage_series(
    core_power_,
    "CORE",
    3.10,
    4.25,
    3.40,
    326,
    ColorRgb{60U, 225U, 255U});

  draw_voltage_series(
    edge_power_,
    "EDGE",
    3.10,
    4.25,
    3.40,
    363,
    ColorRgb{105U, 155U, 255U});

  draw_voltage_series(
    base_power_,
    "BASE",
    6.30,
    8.50,
    7.20,
    400,
    ColorRgb{75U, 235U, 175U});

  // Moving cursor is presentation-only. It appears only when at least
  // one fresh real voltage source exists.
  const bool voltage_graph_live =
    (
      core_power_.seen &&
      core_power_.has_voltage &&
      !power_source_is_stale(core_power_)
    ) ||
    (
      edge_power_.seen &&
      edge_power_.has_voltage &&
      !power_source_is_stale(edge_power_)
    ) ||
    (
      base_power_.seen &&
      base_power_.has_voltage &&
      !power_source_is_stale(base_power_)
    );

  if (voltage_graph_live) {
    const int sweep_plot_x =
      graph_panel_x + 158;

    const int sweep_plot_w =
      graph_panel_w - 172;

    const double sweep_phase =
      std::fmod(
      power_animation_time_seconds_ *
      0.30,
      1.0);

    const int sweep_x =
      sweep_plot_x +
      static_cast<int>(
      std::lround(
        sweep_phase *
        static_cast<double>(
          sweep_plot_w - 1)));

    const int sweep_y =
      graph_panel_y + 38;

    const int sweep_h =
      graph_panel_h - 50;

    canvas_.blend_rect(
      sweep_x - 3,
      sweep_y,
      1,
      sweep_h,
      cyan,
      0.08F);

    canvas_.blend_rect(
      sweep_x - 1,
      sweep_y,
      1,
      sweep_h,
      cyan,
      0.18F);

    canvas_.blend_rect(
      sweep_x,
      sweep_y,
      2,
      sweep_h,
      white,
      static_cast<float>(
        0.20 +
        0.26 * power_pulse));

    canvas_.blend_rect(
      sweep_x + 2,
      sweep_y,
      1,
      sweep_h,
      cyan,
      0.12F);
  }
}

void UiNode::seed_power_preview_data()
{
  core_power_ = PowerUiSourceState{};
  edge_power_ = PowerUiSourceState{};
  base_power_ = PowerUiSourceState{};

  const auto now = std::chrono::steady_clock::now();

  core_power_.seen = true;
  core_power_.has_voltage = true;
  core_power_.has_percent = true;
  core_power_.percent_label = "CAP";
  core_power_.percent = 99.2;
  core_power_.state = "full";
  core_power_.last_update = now;

  edge_power_.seen = true;
  edge_power_.has_voltage = true;
  edge_power_.has_percent = true;
  edge_power_.percent_label = "CAP";
  edge_power_.percent = 86.4;
  edge_power_.state = "ok";
  edge_power_.last_update = now;

  base_power_.seen = true;
  base_power_.has_voltage = true;
  base_power_.has_percent = true;
  base_power_.percent_label = "SOC";
  base_power_.percent = 83.8;
  base_power_.state = "ok";
  base_power_.last_update = now;

  for (int i = 0; i < 60; ++i) {
    core_power_.voltage_history.push_back(
      4.18 + 0.018 * std::sin(static_cast<double>(i) * 0.24));

    edge_power_.voltage_history.push_back(
      4.08 + 0.024 * std::sin(
        static_cast<double>(i) * 0.20 + 0.8));

    base_power_.voltage_history.push_back(
      8.07 + 0.045 * std::sin(
        static_cast<double>(i) * 0.16 + 1.4));
  }

  core_power_.voltage_v = core_power_.voltage_history.back();
  edge_power_.voltage_v = edge_power_.voltage_history.back();
  base_power_.voltage_v = base_power_.voltage_history.back();

  power_status_seen_ = true;
  power_overall_state_ = "ok";
  power_health_state_ = "OK";
  power_status_last_update_ = now;
}

void UiNode::maybe_export_power_live_preview()
{
  if (
    config_.enable_framebuffer ||
    !config_.export_preview_frames)
  {
    return;
  }

  const auto now = std::chrono::steady_clock::now();

  if (
    last_power_preview_export_.time_since_epoch().count() != 0 &&
    std::chrono::duration<double>(
      now - last_power_preview_export_).count() < 1.0)
  {
    return;
  }

  last_power_preview_export_ = now;

  render_power_page();

  const std::string path =
    config_.preview_output_dir + "/power_live.ppm";

  if (!canvas_.write_ppm(path)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "failed to write live power preview | %s",
      path.c_str());
  }

  render_current_screen();
}


void UiNode::render_placeholder_screen(
  const std::string & title,
  const std::string & subtitle)
{
  render_home();

  // Temporary touch-validation page overlay.
  // Later this will be replaced by real Voice/Nav/Status/Power screens.
  const int x = 132;
  const int y = 318;
  const int w = 360;
  const int h = 92;

  canvas_.blend_rect(x, y, w, h, ColorRgb{0U, 10U, 28U}, 0.82F);
  canvas_.blend_rect(x, y, w, 2, ColorRgb{0U, 210U, 255U}, 0.80F);

  Font::draw_text(canvas_, x + 24, y + 22, title, ColorRgb{90U, 230U, 255U}, 4, 0.95F);
  Font::draw_text(canvas_, x + 24, y + 62, subtitle, ColorRgb{230U, 245U, 255U}, 2, 0.85F);
}

void UiNode::present_if_enabled()
{
  if (!config_.enable_framebuffer) {
    return;
  }

  std::string error_message;
  if (!framebuffer_.present_rgb_canvas(canvas_, &error_message)) {
    RCLCPP_ERROR(
      get_logger(),
      "failed to present canvas | error=%s",
      error_message.c_str());
  }
}


void UiNode::request_status_view(
  const StatusView requested_view,
  const ScreenRequestSource source)
{
  const char * view_text = "overview";

  switch (requested_view) {
    case StatusView::Overview:
      view_text = "overview";
      break;
    case StatusView::Sensors:
      view_text = "sensors";
      break;
    case StatusView::AiLink:
      view_text = "ai_link";
      break;
    case StatusView::AlertsSystem:
      view_text = "alerts_system";
      break;
  }

  // Touch always changes the visible Status state immediately.
  if (source == ScreenRequestSource::Touch) {
    pending_screen_.reset();
    pending_status_view_.reset();

    status_view_ = requested_view;

    RCLCPP_INFO(
      get_logger(),
      "Status view accepted immediately | source=touch view=%s",
      view_text);

    transition_to(UiScreen::Status);
    return;
  }

  // SavoMind resolves the destination while Voice remains visible.
  if (
    source == ScreenRequestSource::SavoMind &&
    voice_phase_ == VoicePhase::Speaking)
  {
    queue_status_view_after_tts(requested_view);
    return;
  }

  pending_screen_.reset();
  pending_status_view_.reset();

  status_view_ = requested_view;

  RCLCPP_INFO(
    get_logger(),
    "Status view accepted | view=%s",
    view_text);

  transition_to(UiScreen::Status);
}


void UiNode::queue_status_view_after_tts(
  const StatusView requested_view)
{
  pending_screen_ = UiScreen::Status;
  pending_status_view_ = requested_view;

  RCLCPP_INFO(
    get_logger(),
    "Status view queued until TTS finishes");
}

void UiNode::request_screen(
  const UiScreen requested_screen,
  const ScreenRequestSource source)
{
  const char * source_text = "internal";

  switch (source) {
    case ScreenRequestSource::Touch:
      source_text = "touch";
      break;
    case ScreenRequestSource::Speech:
      source_text = "speech";
      break;
    case ScreenRequestSource::SavoMind:
      source_text = "savomind";
      break;
    case ScreenRequestSource::Internal:
      source_text = "internal";
      break;
  }

  // A SavoMind Navigation request begins in Preparing.
  // Real savo_nav feedback will later change this to Navigating,
  // Paused, Arrived or Error.
  if (
    requested_screen == UiScreen::Navigation &&
    source != ScreenRequestSource::Touch)
  {
    navigation_phase_ = NavigationPhase::Preparing;
    navigation_ui_.live = true;
    navigation_ui_.state = "PREPARING";
    navigation_ui_.message = "Getting ready...";
  }

  // A generic Status request means Status / Overview.
  if (requested_screen == UiScreen::Status) {
    if (
      source == ScreenRequestSource::SavoMind &&
      voice_phase_ == VoicePhase::Speaking)
    {
      queue_status_view_after_tts(StatusView::Overview);
      return;
    }

    request_status_view(
      StatusView::Overview,
      source);
    return;
  }

  // Touch commands always take effect immediately.
  if (source == ScreenRequestSource::Touch) {
    pending_screen_.reset();
    pending_status_view_.reset();

    RCLCPP_INFO(
      get_logger(),
      "screen request accepted immediately | source=%s screen=%s",
      source_text,
      to_string(requested_screen).c_str());

    transition_to(requested_screen);
    return;
  }

  if (
    source == ScreenRequestSource::SavoMind &&
    voice_phase_ == VoicePhase::Speaking)
  {
    queue_screen_after_tts(requested_screen);
    return;
  }

  pending_screen_.reset();
  pending_status_view_.reset();

  RCLCPP_INFO(
    get_logger(),
    "screen request accepted | source=%s screen=%s",
    source_text,
    to_string(requested_screen).c_str());

  transition_to(requested_screen);
}

void UiNode::queue_screen_after_tts(
  const UiScreen requested_screen)
{
  pending_screen_ = requested_screen;

  if (requested_screen == UiScreen::Status) {
    pending_status_view_ = StatusView::Overview;
  } else {
    pending_status_view_.reset();
  }

  RCLCPP_INFO(
    get_logger(),
    "screen queued until TTS finishes | pending=%s",
    to_string(requested_screen).c_str());
}

void UiNode::handle_tts_finished()
{
  voice_phase_ = VoicePhase::Idle;

  if (!pending_screen_.has_value()) {
    pending_status_view_.reset();

    RCLCPP_INFO(
      get_logger(),
      "TTS finished | returning to Voice Idle");

    transition_to(UiScreen::VoiceIdle);
    return;
  }

  const UiScreen requested_screen =
    *pending_screen_;

  const auto requested_status_view =
    pending_status_view_;

  pending_screen_.reset();
  pending_status_view_.reset();

  if (
    requested_screen == UiScreen::Status &&
    requested_status_view.has_value())
  {
    status_view_ = *requested_status_view;
  }

  RCLCPP_INFO(
    get_logger(),
    "TTS finished | opening pending screen=%s",
    to_string(requested_screen).c_str());

  transition_to(requested_screen);
}


void UiNode::transition_to(const UiScreen next_screen)
{
  if (active_screen_ == next_screen) {
    return;
  }

  const auto previous = active_screen_;
  active_screen_ = next_screen;

  RCLCPP_INFO(
    get_logger(),
    "screen transition | %s -> %s",
    to_string(previous).c_str(),
    to_string(active_screen_).c_str());
}

void UiNode::loop_callback()
{
  const auto now = std::chrono::steady_clock::now();
  const auto dt = std::chrono::duration<double>(now - last_loop_time_).count();
  last_loop_time_ = now;

  update_runtime(dt);
  render_current_screen();
  maybe_export_power_live_preview();
  present_if_enabled();

  if (!first_tick_logged_) {
    RCLCPP_INFO(
      get_logger(),
      "savo_ui runtime tick ok | active_screen=%s loaded_home_frames=%zu",
      to_string(active_screen_).c_str(),
      robot360_frames_.size());
    first_tick_logged_ = true;
  }
}

double UiNode::clamp_double(const double value, const double min_value, const double max_value)
{
  return std::clamp(value, min_value, max_value);
}

int UiNode::clamp_int(const int value, const int min_value, const int max_value)
{
  return std::clamp(value, min_value, max_value);
}

std::string UiNode::bool_text(const bool value)
{
  return value ? "true" : "false";
}

}  // namespace savo_ui
