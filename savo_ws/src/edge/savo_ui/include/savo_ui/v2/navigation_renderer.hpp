// Copyright 2026 Ahnaf Tahmid
#pragma once

#include "savo_ui/render/canvas.hpp"
#include "savo_ui/v2/ui_v2_types.hpp"
#include "savo_ui/v2/voice_face_animation.hpp"
#include "savo_ui/v2/voice_face_renderer.hpp"

namespace savo_ui::v2
{

class NavigationRenderer
{
public:
  NavigationRenderer(int width, int height);

  void render(
    Canvas & canvas,
    const NavigationState & state,
    double animation_time_seconds);

private:
  [[nodiscard]] VoiceFaceFrame navigation_frame(
    const NavigationState & state,
    double animation_time_seconds) const;

  VoiceFaceRenderer face_renderer_;
};

}  // namespace savo_ui::v2
