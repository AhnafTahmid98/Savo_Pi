// Copyright 2026 Ahnaf Tahmid
#include <cmath>
#include <iostream>

#include "savo_ui/render/canvas.hpp"
#include "savo_ui/v2/navigation_renderer.hpp"
#include "savo_ui/v2/ui_v2_types.hpp"
#include "savo_ui/v2/voice_face_animation.hpp"
#include "savo_ui/v2/voice_face_renderer.hpp"

namespace
{

constexpr int kWidth = 800;
constexpr int kHeight = 480;
constexpr double kFps = 30.0;
constexpr double kDt = 1.0 / kFps;
constexpr double kTau = 6.28318530717958647692;

void emit_ppm(const savo_ui::Canvas & canvas)
{
  const auto & pixels = canvas.pixels_rgb();

  std::cout
    << "P6\n"
    << canvas.width() << ' ' << canvas.height()
    << "\n255\n";

  std::cout.write(
    reinterpret_cast<const char *>(pixels.data()),
    static_cast<std::streamsize>(pixels.size()));
}

void render_voice_segment(
  savo_ui::Canvas & canvas,
  savo_ui::v2::VoiceFaceRenderer & renderer,
  savo_ui::v2::VoiceFaceAnimation & animation,
  const savo_ui::v2::VoicePhase phase,
  const double seconds,
  const bool simulate_microphone)
{
  animation.reset(phase);

  savo_ui::v2::VoiceState state;
  state.phase = phase;

  const int frame_count =
    static_cast<int>(std::lround(seconds * kFps));

  for (int frame = 0; frame < frame_count; ++frame) {
    const double time_seconds =
      static_cast<double>(frame) * kDt;

    double input_level = 0.0;

    if (simulate_microphone) {
      const double speech_pulse =
        std::abs(std::sin(time_seconds * kTau * 1.65));

      const double speech_envelope =
        0.72 +
        0.22 * std::sin(time_seconds * kTau * 0.43);

      input_level =
        0.12 + 0.78 * speech_pulse * speech_envelope;
    }

    state.input_level = input_level;

    animation.update(
      kDt,
      phase,
      input_level);

    renderer.render(
      canvas,
      state,
      animation.frame());

    emit_ppm(canvas);
  }
}

void render_navigation_segment(
  savo_ui::Canvas & canvas,
  savo_ui::v2::NavigationRenderer & renderer,
  const savo_ui::v2::NavigationPhase phase,
  const double seconds)
{
  savo_ui::v2::NavigationState state;
  state.phase = phase;
  state.goal_id = "A201";
  state.distance_remaining_m = 12.4;

  const int frame_count =
    static_cast<int>(std::lround(seconds * kFps));

  for (int frame = 0; frame < frame_count; ++frame) {
    const double time_seconds =
      static_cast<double>(frame) * kDt;

    renderer.render(
      canvas,
      state,
      time_seconds);

    emit_ppm(canvas);
  }
}

}  // namespace

int main()
{
  savo_ui::Canvas canvas(kWidth, kHeight);

  savo_ui::v2::VoiceFaceRenderer voice_renderer(
    kWidth,
    kHeight);

  savo_ui::v2::VoiceFaceAnimation voice_animation;

  savo_ui::v2::NavigationRenderer navigation_renderer(
    kWidth,
    kHeight);

  std::cerr
    << "Savo UI V2 animation preview\n"
    << "800x480 @ 30 FPS\n"
    << "Sequence:\n"
    << "  Idle\n"
    << "  Listening\n"
    << "  Thinking\n"
    << "  Speaking\n"
    << "  Navigation preparing\n"
    << "  Navigating\n"
    << "  Navigation paused\n"
    << "  Arrived\n";

  render_voice_segment(
    canvas,
    voice_renderer,
    voice_animation,
    savo_ui::v2::VoicePhase::Idle,
    1.0,
    false);

  render_voice_segment(
    canvas,
    voice_renderer,
    voice_animation,
    savo_ui::v2::VoicePhase::Listening,
    3.0,
    true);

  render_voice_segment(
    canvas,
    voice_renderer,
    voice_animation,
    savo_ui::v2::VoicePhase::Thinking,
    2.5,
    false);

  render_voice_segment(
    canvas,
    voice_renderer,
    voice_animation,
    savo_ui::v2::VoicePhase::Speaking,
    3.0,
    false);

  render_navigation_segment(
    canvas,
    navigation_renderer,
    savo_ui::v2::NavigationPhase::Preparing,
    2.0);

  render_navigation_segment(
    canvas,
    navigation_renderer,
    savo_ui::v2::NavigationPhase::Navigating,
    3.0);

  render_navigation_segment(
    canvas,
    navigation_renderer,
    savo_ui::v2::NavigationPhase::Paused,
    1.5);

  render_navigation_segment(
    canvas,
    navigation_renderer,
    savo_ui::v2::NavigationPhase::Arrived,
    2.0);

  return 0;
}
