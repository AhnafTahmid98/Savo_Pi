// Copyright 2026 Ahnaf Tahmid
#include <algorithm>
#include <cmath>

#include "savo_ui/v2/graphics.hpp"
#include "savo_ui/v2/voice_face_renderer.hpp"

namespace savo_ui::v2
{
namespace
{
constexpr ColorRgb kBlackGlass{7U, 5U, 5U};
constexpr ColorRgb kWarmGlass{28U, 12U, 6U};
constexpr ColorRgb kAmber{255U, 112U, 8U};
constexpr ColorRgb kAmberSoft{255U, 145U, 22U};
constexpr ColorRgb kEyeCore{255U, 248U, 205U};
}

VoiceFaceRenderer::VoiceFaceRenderer(const int width, const int height)
: width_(width), height_(height), background_(width, height)
{
  build_background();
}

void VoiceFaceRenderer::build_background()
{
  background_.clear(ColorRgb{9U, 4U, 2U});

  // Warm outer ambience, matching the orange edge illumination in the reference face.
  background_.blend_rect(0, 0, width_, height_, ColorRgb{56U, 15U, 2U}, 0.30F);
  background_.blend_rect(0, height_ - 80, width_, 80, ColorRgb{92U, 25U, 2U}, 0.16F);

  // Metallic rounded bezel. Each layer is static and cached.
  graphics::rounded_rect(background_, 12, 9, width_ - 24, height_ - 18, 62,
    ColorRgb{110U, 101U, 96U}, 1.0F);
  graphics::rounded_rect(background_, 17, 13, width_ - 34, height_ - 26, 58,
    ColorRgb{236U, 232U, 227U}, 1.0F);
  graphics::rounded_rect(background_, 22, 18, width_ - 44, height_ - 36, 54,
    ColorRgb{165U, 157U, 152U}, 1.0F);
  graphics::rounded_rect(background_, 29, 25, width_ - 58, height_ - 50, 49,
    ColorRgb{45U, 35U, 31U}, 1.0F);

  // Glossy black face glass.
  graphics::rounded_rect(background_, 34, 30, width_ - 68, height_ - 60, 45,
    kBlackGlass, 1.0F);
  graphics::rounded_rect(background_, 38, 34, width_ - 76, height_ - 68, 42,
    kWarmGlass, 0.34F);

  // Glass highlights. Kept subtle so the eyes remain the dominant feature.
  graphics::rounded_rect(background_, 47, 43, width_ - 94, 80, 34,
    ColorRgb{150U, 89U, 65U}, 0.055F);
  background_.blend_rect(62, 54, width_ - 124, 3, ColorRgb{245U, 218U, 198U}, 0.055F);
  background_.blend_rect(56, height_ - 80, width_ - 112, 30,
    ColorRgb{82U, 25U, 9U}, 0.06F);
}

void VoiceFaceRenderer::draw_eye(
  Canvas & canvas,
  const int center_x,
  const int center_y,
  const int radius_x,
  const int radius_y,
  const double glow,
  const double openness) const
{
  const int open_ry = std::max(7, static_cast<int>(std::lround(radius_y * openness)));
  const float glow_value = static_cast<float>(std::clamp(glow, 0.0, 1.5));

  graphics::ellipse_glow(canvas, center_x, center_y, radius_x + 6, open_ry + 7,
    kAmber, glow_value);
  graphics::ellipse(canvas, center_x, center_y, radius_x + 5, open_ry + 5,
    ColorRgb{196U, 53U, 2U}, 0.70F);
  graphics::ellipse(canvas, center_x, center_y, radius_x + 2, open_ry + 2,
    kAmber, 0.95F);
  graphics::ellipse(canvas, center_x, center_y, radius_x - 4, std::max(5, open_ry - 5),
    kAmberSoft, 0.98F);
  graphics::ellipse(canvas, center_x, center_y, radius_x - 9, std::max(4, open_ry - 10),
    kEyeCore, 1.0F);

  // Soft white-hot center, not a flat pure-white fill.
  graphics::ellipse(canvas, center_x - 3, center_y - 6,
    std::max(4, radius_x - 17), std::max(3, open_ry - 23),
    ColorRgb{255U, 254U, 238U}, 0.70F);
}

void VoiceFaceRenderer::draw_mouth(Canvas & canvas, const VoiceFaceFrame & frame) const
{
  const int center_x = width_ / 2;
  const int center_y = static_cast<int>(height_ * 0.735);
  const int base_width = static_cast<int>(width_ * 0.115);
  const int mouth_width = std::max(42,
    static_cast<int>(std::lround(base_width * frame.mouth_width_scale)));

  if (frame.mouth_open < 0.08) {
    graphics::rounded_rect(canvas, center_x - mouth_width / 2, center_y,
      mouth_width, 9, 5, ColorRgb{33U, 16U, 12U}, 0.96F);
    graphics::rounded_rect(canvas, center_x - mouth_width / 2 + 4, center_y + 1,
      mouth_width - 8, 2, 1, ColorRgb{115U, 57U, 31U}, 0.32F);
    return;
  }

  const int mouth_height = 8 + static_cast<int>(std::lround(frame.mouth_open * 29.0));
  graphics::rounded_rect(canvas, center_x - mouth_width / 2, center_y - mouth_height / 2,
    mouth_width, mouth_height, mouth_height / 2, ColorRgb{30U, 9U, 5U}, 1.0F);
  graphics::rounded_rect(canvas, center_x - mouth_width / 2 + 4,
    center_y - mouth_height / 2 + 3, mouth_width - 8, std::max(3, mouth_height - 7),
    std::max(2, mouth_height / 2 - 3), ColorRgb{118U, 41U, 10U}, 0.62F);
  graphics::rounded_rect(canvas, center_x - mouth_width / 2 + 8,
    center_y - mouth_height / 2 + 4, mouth_width - 16, std::max(2, mouth_height / 3),
    std::max(2, mouth_height / 5), ColorRgb{245U, 102U, 18U}, 0.30F);
}

void VoiceFaceRenderer::render(
  Canvas & canvas,
  const VoiceState & state,
  const VoiceFaceFrame & frame)
{
  (void)state;
  canvas = background_;

  const int base_eye_rx = static_cast<int>(width_ * 0.058);
  const int base_eye_ry = static_cast<int>(height_ * 0.205);
  const int eye_rx = std::max(18,
    static_cast<int>(std::lround(base_eye_rx * frame.eye_width_scale)));
  const int eye_ry = std::max(28,
    static_cast<int>(std::lround(base_eye_ry * frame.eye_height_scale)));

  const int center_y = static_cast<int>(height_ * 0.43 + frame.eye_shift_y);
  const int left_x = static_cast<int>(width_ * 0.35 + frame.eye_shift_x);
  const int right_x = static_cast<int>(width_ * 0.65 + frame.eye_shift_x);

  draw_eye(canvas, left_x, center_y, eye_rx, eye_ry, frame.eye_glow, frame.eye_open);
  draw_eye(canvas, right_x, center_y, eye_rx, eye_ry, frame.eye_glow, frame.eye_open);
  draw_mouth(canvas, frame);
}

}  // namespace savo_ui::v2
