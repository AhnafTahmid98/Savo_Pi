// Copyright 2026 Ahnaf Tahmid
#pragma once

#include "savo_ui/v2/ui_v2_types.hpp"

namespace savo_ui::v2
{

struct VoiceFaceFrame
{
  double eye_open{1.0};
  double eye_width_scale{1.0};
  double eye_height_scale{1.0};
  double eye_glow{1.0};
  double eye_shift_x{0.0};
  double eye_shift_y{0.0};
  double mouth_open{0.0};
  double mouth_width_scale{1.0};
  double face_breathe{0.0};
};

class VoiceFaceAnimation
{
public:
  void reset(VoicePhase phase);
  void update(double dt_seconds, VoicePhase phase, double input_level);
  [[nodiscard]] VoiceFaceFrame frame() const;

private:
  VoicePhase phase_{VoicePhase::Idle};
  double time_seconds_{0.0};
  double phase_seconds_{0.0};
  double smoothed_input_level_{0.0};
};

}  // namespace savo_ui::v2
