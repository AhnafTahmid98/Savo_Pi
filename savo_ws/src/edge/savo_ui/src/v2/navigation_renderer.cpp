// Copyright 2026 Ahnaf Tahmid
#include <algorithm>
#include <cmath>

#include "savo_ui/v2/navigation_renderer.hpp"

namespace savo_ui::v2
{

NavigationRenderer::NavigationRenderer(const int width, const int height)
: face_renderer_(width, height)
{
}

VoiceFaceFrame NavigationRenderer::navigation_frame(
  const NavigationState & state,
  const double animation_time_seconds) const
{
  VoiceFaceFrame frame;

  const double slow_breathe =
    0.5 + 0.5 * std::sin(animation_time_seconds * 1.35);

  switch (state.phase) {
    case NavigationPhase::Idle:
      // Same calm face as the normal idle Voice face.
      frame.eye_open = 1.0;
      frame.eye_width_scale = 1.0;
      frame.eye_height_scale = 1.0;
      frame.eye_glow = 0.82 + 0.10 * slow_breathe;
      frame.eye_shift_x = 0.0;
      frame.eye_shift_y = 0.0;
      frame.mouth_open = 0.0;
      frame.mouth_width_scale = 1.0;
      break;

    case NavigationPhase::Preparing:
      // Route planning should look like Savo is thinking.
      frame.eye_open = 0.88;
      frame.eye_width_scale = 1.0;
      frame.eye_height_scale = 0.96;
      frame.eye_glow =
        0.95 + 0.14 * slow_breathe;

      frame.eye_shift_x =
        7.0 * std::sin(animation_time_seconds * 1.55);

      frame.eye_shift_y =
        -3.0 + 2.0 * std::sin(animation_time_seconds * 0.9);

      frame.mouth_open = 0.0;
      frame.mouth_width_scale = 0.92;
      break;

    case NavigationPhase::Navigating:
      // Awake, confident and attentive while Savo is moving.
      frame.eye_open = 1.0;
      frame.eye_width_scale =
        1.02 + 0.025 * slow_breathe;

      frame.eye_height_scale =
        1.01 + 0.025 * slow_breathe;

      frame.eye_glow =
        1.08 + 0.16 * slow_breathe;

      // Very small synchronized motion gives a feeling of movement
      // without turning the face into a navigation dashboard.
      frame.eye_shift_x =
        2.5 * std::sin(animation_time_seconds * 1.1);

      frame.eye_shift_y =
        -2.0 * std::sin(animation_time_seconds * 2.1);

      frame.mouth_open = 0.0;
      frame.mouth_width_scale = 1.0;
      break;

    case NavigationPhase::Paused:
      // Calm waiting expression.
      frame.eye_open = 0.67;
      frame.eye_width_scale = 1.0;
      frame.eye_height_scale = 0.90;
      frame.eye_glow =
        0.66 + 0.08 * slow_breathe;

      frame.eye_shift_x = 0.0;
      frame.eye_shift_y = 3.0;
      frame.mouth_open = 0.0;
      frame.mouth_width_scale = 0.88;
      break;

    case NavigationPhase::Arrived:
      // Bright friendly arrival expression.
      frame.eye_open =
        0.90 + 0.06 * slow_breathe;

      frame.eye_width_scale =
        1.04 + 0.025 * slow_breathe;

      frame.eye_height_scale =
        0.95 + 0.03 * slow_breathe;

      frame.eye_glow =
        1.20 + 0.18 * slow_breathe;

      frame.eye_shift_x = 0.0;
      frame.eye_shift_y = -2.0;

      frame.mouth_open =
        0.08 + 0.04 * slow_breathe;

      frame.mouth_width_scale = 1.10;
      break;

    case NavigationPhase::Error:
      // Concerned but still recognizably the same Savo face.
      frame.eye_open = 0.50;
      frame.eye_width_scale = 1.02;
      frame.eye_height_scale = 0.82;
      frame.eye_glow =
        0.60 + 0.10 * slow_breathe;

      frame.eye_shift_x = 0.0;
      frame.eye_shift_y = 4.0;
      frame.mouth_open = 0.0;
      frame.mouth_width_scale = 0.72;
      break;
  }

  frame.eye_open = std::clamp(frame.eye_open, 0.05, 1.0);
  frame.eye_glow = std::clamp(frame.eye_glow, 0.0, 1.5);

  return frame;
}

void NavigationRenderer::render(
  Canvas & canvas,
  const NavigationState & state,
  const double animation_time_seconds)
{
  const VoiceFaceFrame frame =
    navigation_frame(state, animation_time_seconds);

  // VoiceFaceRenderer is intentionally reused here.
  // This guarantees Voice and Navigation have the same Savo face.
  const VoiceState face_state{};
  face_renderer_.render(canvas, face_state, frame);
}

}  // namespace savo_ui::v2
