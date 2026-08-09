// Copyright 2026 Ahnaf Tahmid
#pragma once

#include "savo_ui/render/canvas.hpp"
#include "savo_ui/render/color.hpp"

namespace savo_ui::v2::graphics
{

void rounded_rect(
  Canvas & canvas,
  int x,
  int y,
  int width,
  int height,
  int radius,
  ColorRgb color,
  float alpha = 1.0F);

void ellipse(
  Canvas & canvas,
  int center_x,
  int center_y,
  int radius_x,
  int radius_y,
  ColorRgb color,
  float alpha = 1.0F);

void ellipse_glow(
  Canvas & canvas,
  int center_x,
  int center_y,
  int radius_x,
  int radius_y,
  ColorRgb color,
  float intensity = 1.0F);

void line(
  Canvas & canvas,
  int x0,
  int y0,
  int x1,
  int y1,
  int thickness,
  ColorRgb color,
  float alpha = 1.0F);

void line_glow(
  Canvas & canvas,
  int x0,
  int y0,
  int x1,
  int y1,
  int thickness,
  ColorRgb color,
  float intensity = 1.0F);

}  // namespace savo_ui::v2::graphics
