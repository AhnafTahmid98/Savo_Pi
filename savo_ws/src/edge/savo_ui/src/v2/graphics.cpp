// Copyright 2026 Ahnaf Tahmid
#include <algorithm>
#include <cmath>

#include "savo_ui/v2/graphics.hpp"

namespace savo_ui::v2::graphics
{
namespace
{

bool inside_rounded_rect(
  const int px,
  const int py,
  const int x,
  const int y,
  const int width,
  const int height,
  const int radius)
{
  if (width <= 0 || height <= 0) {
    return false;
  }

  const int bounded_radius = std::max(0, std::min(radius, std::min(width, height) / 2));
  if (bounded_radius == 0) {
    return px >= x && px < x + width && py >= y && py < y + height;
  }

  const int left = x + bounded_radius;
  const int right = x + width - bounded_radius - 1;
  const int top = y + bounded_radius;
  const int bottom = y + height - bounded_radius - 1;

  if ((px >= left && px <= right) || (py >= top && py <= bottom)) {
    return px >= x && px < x + width && py >= y && py < y + height;
  }

  const int cx = px < left ? left : right;
  const int cy = py < top ? top : bottom;
  const int dx = px - cx;
  const int dy = py - cy;
  return dx * dx + dy * dy <= bounded_radius * bounded_radius;
}

}  // namespace

void rounded_rect(
  Canvas & canvas,
  const int x,
  const int y,
  const int width,
  const int height,
  const int radius,
  const ColorRgb color,
  const float alpha)
{
  if (!canvas.valid() || width <= 0 || height <= 0 || alpha <= 0.0F) {
    return;
  }

  const int start_x = std::max(0, x);
  const int start_y = std::max(0, y);
  const int end_x = std::min(canvas.width(), x + width);
  const int end_y = std::min(canvas.height(), y + height);

  for (int py = start_y; py < end_y; ++py) {
    for (int px = start_x; px < end_x; ++px) {
      if (inside_rounded_rect(px, py, x, y, width, height, radius)) {
        canvas.blend_pixel(px, py, color, alpha);
      }
    }
  }
}

void ellipse(
  Canvas & canvas,
  const int center_x,
  const int center_y,
  const int radius_x,
  const int radius_y,
  const ColorRgb color,
  const float alpha)
{
  if (!canvas.valid() || radius_x <= 0 || radius_y <= 0 || alpha <= 0.0F) {
    return;
  }

  const int start_x = std::max(0, center_x - radius_x);
  const int end_x = std::min(canvas.width() - 1, center_x + radius_x);
  const int start_y = std::max(0, center_y - radius_y);
  const int end_y = std::min(canvas.height() - 1, center_y + radius_y);
  const double inv_rx = 1.0 / static_cast<double>(radius_x);
  const double inv_ry = 1.0 / static_cast<double>(radius_y);

  const double edge_scale = static_cast<double>(std::min(radius_x, radius_y));
  for (int y = start_y; y <= end_y; ++y) {
    const double dy = static_cast<double>(y - center_y) * inv_ry;
    const double dy2 = dy * dy;
    for (int x = start_x; x <= end_x; ++x) {
      const double dx = static_cast<double>(x - center_x) * inv_rx;
      const double distance_squared = dx * dx + dy2;
      if (distance_squared <= 1.0) {
        const double distance = std::sqrt(distance_squared);
        const float edge_alpha = static_cast<float>(
          std::clamp((1.0 - distance) * edge_scale, 0.0, 1.0));
        canvas.blend_pixel(x, y, color, alpha * edge_alpha);
      }
    }
  }
}

void ellipse_glow(
  Canvas & canvas,
  const int center_x,
  const int center_y,
  const int radius_x,
  const int radius_y,
  const ColorRgb color,
  const float intensity)
{
  if (!canvas.valid() || radius_x <= 0 || radius_y <= 0 || intensity <= 0.0F) {
    return;
  }

  const float bounded = std::clamp(intensity, 0.0F, 1.5F);
  constexpr double outer_scale = 2.15;
  const int outer_rx = static_cast<int>(std::ceil(radius_x * outer_scale));
  const int outer_ry = static_cast<int>(std::ceil(radius_y * outer_scale));
  const int start_x = std::max(0, center_x - outer_rx);
  const int end_x = std::min(canvas.width() - 1, center_x + outer_rx);
  const int start_y = std::max(0, center_y - outer_ry);
  const int end_y = std::min(canvas.height() - 1, center_y + outer_ry);
  const double inv_rx = 1.0 / static_cast<double>(radius_x);
  const double inv_ry = 1.0 / static_cast<double>(radius_y);

  for (int y = start_y; y <= end_y; ++y) {
    const double dy = static_cast<double>(y - center_y) * inv_ry;
    const double dy2 = dy * dy;
    for (int x = start_x; x <= end_x; ++x) {
      const double dx = static_cast<double>(x - center_x) * inv_rx;
      const double distance_squared = dx * dx + dy2;
      if (distance_squared <= 1.0 || distance_squared >= outer_scale * outer_scale) {
        continue;
      }
      const double distance = std::sqrt(distance_squared);
      const double t = 1.0 - (distance - 1.0) / (outer_scale - 1.0);
      const float glow_alpha = static_cast<float>(0.24 * t * t) * bounded;
      canvas.blend_pixel(x, y, color, glow_alpha);
    }
  }
}

void line(
  Canvas & canvas,
  const int x0,
  const int y0,
  const int x1,
  const int y1,
  const int thickness,
  const ColorRgb color,
  const float alpha)
{
  if (!canvas.valid() || thickness <= 0 || alpha <= 0.0F) {
    return;
  }

  const int dx = x1 - x0;
  const int dy = y1 - y0;
  const int steps = std::max(std::abs(dx), std::abs(dy));
  if (steps == 0) {
    ellipse(canvas, x0, y0, thickness / 2 + 1, thickness / 2 + 1, color, alpha);
    return;
  }

  const double x_step = static_cast<double>(dx) / static_cast<double>(steps);
  const double y_step = static_cast<double>(dy) / static_cast<double>(steps);
  const int radius = std::max(1, thickness / 2);

  for (int i = 0; i <= steps; ++i) {
    const int x = static_cast<int>(std::lround(static_cast<double>(x0) + x_step * i));
    const int y = static_cast<int>(std::lround(static_cast<double>(y0) + y_step * i));
    ellipse(canvas, x, y, radius, radius, color, alpha);
  }
}

void line_glow(
  Canvas & canvas,
  const int x0,
  const int y0,
  const int x1,
  const int y1,
  const int thickness,
  const ColorRgb color,
  const float intensity)
{
  const float bounded = std::clamp(intensity, 0.0F, 1.5F);
  line(canvas, x0, y0, x1, y1, thickness + 18, color, 0.035F * bounded);
  line(canvas, x0, y0, x1, y1, thickness + 10, color, 0.065F * bounded);
  line(canvas, x0, y0, x1, y1, thickness + 4, color, 0.16F * bounded);
  line(canvas, x0, y0, x1, y1, thickness, color, std::min(1.0F, 0.88F * bounded));
}

}  // namespace savo_ui::v2::graphics
