// Copyright 2026 Ahnaf Tahmid
#include <algorithm>
#include <cmath>

#include "savo_ui/v2/voice_face_animation.hpp"

namespace savo_ui::v2
{
namespace
{
constexpr double kTau = 6.28318530717958647692;
}

void VoiceFaceAnimation::reset(const VoicePhase phase)
{
  phase_ = phase;
  phase_seconds_ = 0.0;
  smoothed_input_level_ = 0.0;
}

void VoiceFaceAnimation::update(
  const double dt_seconds,
  const VoicePhase phase,
  const double input_level)
{
  const double dt = std::clamp(dt_seconds, 0.0, 0.1);
  time_seconds_ += dt;
  if (phase != phase_) {
    reset(phase);
  }
  phase_seconds_ += dt;

  const double target = std::clamp(input_level, 0.0, 1.0);
  const double response = 1.0 - std::exp(-dt * 12.0);
  smoothed_input_level_ += (target - smoothed_input_level_) * response;
}

VoiceFaceFrame VoiceFaceAnimation::frame() const
{
  VoiceFaceFrame out;

  const double breathe = 0.5 + 0.5 * std::sin(time_seconds_ * 1.8);
  out.face_breathe = breathe;
  out.eye_glow = 0.90 + 0.12 * breathe;

  // Deterministic natural blink: a short closure about every 4.7 seconds.
  const double blink_cycle = std::fmod(time_seconds_ + 0.55, 4.7);
  if (blink_cycle < 0.16) {
    const double x = blink_cycle / 0.16;
    out.eye_open = std::max(0.08, std::abs(2.0 * x - 1.0));
  }

  switch (phase_) {
    case VoicePhase::Idle:
      out.eye_width_scale = 0.98 + 0.02 * breathe;
      out.eye_height_scale = 0.98 + 0.02 * breathe;
      out.mouth_open = 0.0;
      break;

    case VoicePhase::Listening:
      out.eye_width_scale = 1.00 + 0.055 * smoothed_input_level_;
      out.eye_height_scale = 1.00 + 0.075 * smoothed_input_level_;
      out.eye_glow = 1.02 + 0.36 * smoothed_input_level_ + 0.08 * breathe;
      out.mouth_open = 0.0;
      break;

    case VoicePhase::Thinking:
      out.eye_height_scale = 0.88;
      out.eye_width_scale = 0.97;
      out.eye_shift_x = 13.0 * std::sin(time_seconds_ * 1.25);
      out.eye_shift_y = -5.0 + 3.0 * std::sin(time_seconds_ * 0.75);
      out.eye_glow = 0.92 + 0.12 * (0.5 + 0.5 * std::sin(time_seconds_ * 2.1));
      out.mouth_open = 0.0;
      break;

    case VoicePhase::Speaking:
      out.eye_width_scale = 1.0;
      out.eye_height_scale = 1.0;
      out.eye_glow = 1.05 + 0.08 * breathe;
      out.mouth_open = 0.18 + 0.82 * std::abs(std::sin(time_seconds_ * kTau * 2.15));
      out.mouth_width_scale = 0.92 + 0.12 * std::sin(time_seconds_ * kTau * 1.05);
      break;

    case VoicePhase::Error:
      out.eye_height_scale = 0.70;
      out.eye_width_scale = 1.03;
      out.eye_shift_x = phase_seconds_ < 0.9 ? 5.0 * std::sin(phase_seconds_ * 22.0) : 0.0;
      out.eye_glow = 0.70 + 0.08 * breathe;
      out.mouth_open = 0.0;
      break;
  }

  return out;
}

}  // namespace savo_ui::v2
