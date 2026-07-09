#include "savo_ui/app/ui_node.hpp"
#include "savo_ui/render/preview_writer.hpp"
#include "savo_ui/render/font.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace savo_ui
{
namespace
{

std::string join_path(const std::string & left, const std::string & right)
{
  return (std::filesystem::path(left) / right).string();
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

void UiNode::declare_parameters()
{
  declare_parameter<std::string>("robot_name", config_.robot_name);
  declare_parameter<std::string>("framebuffer_device", config_.framebuffer_device);
  declare_parameter<std::string>("asset_root", config_.asset_root);
  declare_parameter<std::string>("preview_output_dir", config_.preview_output_dir);

  declare_parameter<int>("screen_width", config_.screen_width);
  declare_parameter<int>("screen_height", config_.screen_height);

  declare_parameter<double>("loop_hz", config_.loop_hz);
  declare_parameter<double>("intro_seconds", config_.intro_seconds);
  declare_parameter<double>(
    "robot360_seconds_per_frame",
    config_.robot360_seconds_per_frame);

  declare_parameter<int>("preview_animation_frames", config_.preview_animation_frames);

  declare_parameter<bool>("enable_framebuffer", config_.enable_framebuffer);
  declare_parameter<bool>("enable_touch", config_.enable_touch);
  declare_parameter<bool>("export_preview_frames", config_.export_preview_frames);
}

void UiNode::load_parameters()
{
  config_.robot_name = get_parameter("robot_name").as_string();
  config_.framebuffer_device = get_parameter("framebuffer_device").as_string();
  config_.asset_root = get_parameter("asset_root").as_string();
  config_.preview_output_dir = get_parameter("preview_output_dir").as_string();

  config_.screen_width = clamp_int(get_parameter("screen_width").as_int(), 320, 3840);
  config_.screen_height = clamp_int(get_parameter("screen_height").as_int(), 240, 2160);

  config_.loop_hz = clamp_double(get_parameter("loop_hz").as_double(), 1.0, 60.0);
  config_.intro_seconds = clamp_double(get_parameter("intro_seconds").as_double(), 1.0, 20.0);
  config_.robot360_seconds_per_frame =
    clamp_double(get_parameter("robot360_seconds_per_frame").as_double(), 0.1, 5.0);

  config_.preview_animation_frames =
    clamp_int(get_parameter("preview_animation_frames").as_int(), 4, 120);

  config_.enable_framebuffer = get_parameter("enable_framebuffer").as_bool();
  config_.enable_touch = get_parameter("enable_touch").as_bool();
  config_.export_preview_frames = get_parameter("export_preview_frames").as_bool();

  if (config_.framebuffer_device.empty()) {
    throw std::runtime_error("framebuffer_device parameter cannot be empty");
  }

  if (config_.asset_root.empty()) {
    throw std::runtime_error("asset_root parameter cannot be empty");
  }

  if (config_.preview_output_dir.empty()) {
    throw std::runtime_error("preview_output_dir parameter cannot be empty");
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
    RCLCPP_INFO(get_logger(), "preview frame export disabled by config");
    return;
  }

  std::string error_message;

  active_screen_ = UiScreen::Intro;

  const double saved_intro_elapsed = intro_elapsed_seconds_;

  for (int i = 0; i < 30; ++i) {
    const double phase = static_cast<double>(i) / 29.0;
    intro_elapsed_seconds_ = phase * config_.intro_seconds;
    render_intro();

    char filename[80];
    std::snprintf(filename, sizeof(filename), "/intro_anim_%03d.ppm", i);

    const auto intro_path = config_.preview_output_dir + filename;
    error_message.clear();

    if (PreviewWriter::write_canvas_ppm(canvas_, intro_path, &error_message)) {
      RCLCPP_INFO(get_logger(), "saved intro animation preview | %s", intro_path.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "failed to save intro animation preview | frame=%d error=%s",
        i,
        error_message.c_str());
    }
  }

  intro_elapsed_seconds_ = saved_intro_elapsed;
  render_intro();

  active_screen_ = UiScreen::Home;

  for (std::size_t i = 0; i < home_idle_sequence_.size(); ++i) {
    home_idle_sequence_index_ = i;
    render_home();

    char filename[64];
    std::snprintf(filename, sizeof(filename), "/home_idle_%03zu.ppm", i);

    const auto frame_path = config_.preview_output_dir + filename;
    error_message.clear();

    if (PreviewWriter::write_canvas_ppm(canvas_, frame_path, &error_message)) {
      RCLCPP_INFO(
        get_logger(),
        "saved preview frame | %s source_frame_index=%zu",
        frame_path.c_str(),
        home_idle_sequence_[i]);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "failed to save home idle preview | sequence_index=%zu error=%s",
        i,
        error_message.c_str());
    }
  }

  active_screen_ = UiScreen::Intro;
  home_idle_sequence_index_ = 0U;
  render_intro();

  export_home_animation_preview();

  RCLCPP_INFO(
    get_logger(),
    "preview animation export complete | dir=%s idle_frames=%zu smooth_frames=%d",
    config_.preview_output_dir.c_str(),
    home_idle_sequence_.size(),
    config_.preview_animation_frames);
}

void UiNode::export_home_animation_preview()
{
  if (home_idle_sequence_.empty()) {
    RCLCPP_WARN(get_logger(), "cannot export smooth home animation: idle sequence missing");
    return;
  }

  std::string error_message;

  const double saved_glow_time = home_glow_elapsed_seconds_;
  const std::size_t saved_sequence_index = home_idle_sequence_index_;
  const UiScreen saved_screen = active_screen_;

  active_screen_ = UiScreen::Home;

  const int frame_count = config_.preview_animation_frames;
  const double simulated_dt = 1.0 / 12.0;

  for (int i = 0; i < frame_count; ++i) {
    home_glow_elapsed_seconds_ = static_cast<double>(i) * simulated_dt;

    const double sway_phase =
      static_cast<double>(i) / static_cast<double>(std::max(1, frame_count));

    const auto sequence_size = home_idle_sequence_.size();
    const auto sequence_index =
      static_cast<std::size_t>(
        std::floor(sway_phase * static_cast<double>(sequence_size) * 2.0)) %
      sequence_size;

    home_idle_sequence_index_ = sequence_index;

    render_home();

    char filename[80];
    std::snprintf(filename, sizeof(filename), "/home_anim_%03d.ppm", i);

    const auto frame_path = config_.preview_output_dir + filename;
    error_message.clear();

    if (PreviewWriter::write_canvas_ppm(canvas_, frame_path, &error_message)) {
      RCLCPP_INFO(
        get_logger(),
        "saved smooth animation preview | %s sequence_index=%zu source_frame_index=%zu",
        frame_path.c_str(),
        home_idle_sequence_index_,
        home_idle_sequence_[home_idle_sequence_index_]);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "failed to save smooth animation preview | frame=%d error=%s",
        i,
        error_message.c_str());
    }
  }

  active_screen_ = saved_screen;
  home_idle_sequence_index_ = saved_sequence_index;
  home_glow_elapsed_seconds_ = saved_glow_time;
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
