// Copyright 2026 Ahnaf Tahmid
#pragma once

#include "savo_ui/render/canvas.hpp"
#include "savo_ui/v2/ui_v2_types.hpp"
#include "savo_ui/v2/voice_face_animation.hpp"

namespace savo_ui::v2
{

class VoiceFaceRenderer
{
public:
  VoiceFaceRenderer(int width, int height);

  void render(
    Canvas & canvas,
    const VoiceState & state,
    const VoiceFaceFrame & frame);

private:
  void build_background();
  void draw_eye(
    Canvas & canvas,
    int center_x,
    int center_y,
    int radius_x,
    int radius_y,
    double glow,
    double openness) const;
  void draw_mouth(Canvas & canvas, const VoiceFaceFrame & frame) const;

  int width_{0};
  int height_{0};
  Canvas background_;
};

}  // namespace savo_ui::v2
