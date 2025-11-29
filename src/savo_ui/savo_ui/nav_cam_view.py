#!/usr/bin/env python3
"""
Robot Savo — Navigation camera view renderer (NAVIGATE mode)

This module draws the NAVIGATE UI:

- Large camera area (placeholder rectangle for now).
- Status text overlay (goal / guidance).
- Subtitle text (what Robot Savo is saying).
- Simple "mouth bar" that reacts to mouth_level in a corner.

Actual camera frames will be wired later via nav_cam_view + Image messages.
"""

from __future__ import annotations

from typing import Dict, Tuple

import pygame

RGB = Tuple[int, int, int]


def _render_multiline_text(
    surface: pygame.Surface,
    text: str,
    font: pygame.font.Font,
    color: RGB,
    max_width: int,
    start_y: int,
    line_spacing: int = 4,
    center_x: bool = True,
) -> int:
    """
    Render text as multiple lines within max_width.
    Returns the y-coordinate after the last rendered line.
    """
    if not text:
        return start_y

    words = text.split()
    if not words:
        return start_y

    lines = []
    current: list[str] = []

    for w in words:
        trial = " ".join(current + [w]) if current else w
        width, _ = font.size(trial)
        if width <= max_width:
            current.append(w)
        else:
            if current:
                lines.append(" ".join(current))
            current = [w]

    if current:
        lines.append(" ".join(current))

    y = start_y
    for line in lines:
        surf = font.render(line, True, color)
        rect = surf.get_rect()
        if center_x:
            rect.centerx = surface.get_width() // 2
        else:
            rect.x = (surface.get_width() - max_width) // 2
        rect.y = y
        surface.blit(surf, rect)
        y = rect.bottom + line_spacing

    return y


def draw_navigation_view(
    surface: pygame.Surface,
    status_text: str,
    subtitle_text: str,
    mouth_level: float,
    fonts: Dict[str, pygame.font.Font],
    colors: Dict[str, RGB],
    screen_size: Tuple[int, int],
    camera_ready: bool,
) -> None:
    """
    Draw the NAVIGATE mode view.

    Parameters
    ----------
    surface:
        Pygame surface to draw onto (already cleared by caller).
    status_text:
        Navigation status ("Guiding to A201", "Please follow me", etc.).
    subtitle_text:
        TTS text (spoken sentence).
    mouth_level:
        0.0–1.0 mouth activity signal.
    fonts:
        Dict with keys "main", "status", "subtitle".
    colors:
        Dict with keys "bg", "text_main", "text_status", "text_subtitle".
    screen_size:
        (width, height) of the display.
    camera_ready:
        True if we have received at least one camera frame.
        For now this only changes placeholder text.
    """
    width, height = screen_size

    font_main = fonts.get("main")
    font_status = fonts.get("status", font_main)
    font_subtitle = fonts.get("subtitle", font_main)

    color_bg: RGB = colors.get("bg", (0, 0, 0))
    color_text_main: RGB = colors.get("text_main", (255, 255, 255))
    color_text_status: RGB = colors.get("text_status", (240, 240, 240))
    color_text_subtitle: RGB = colors.get("text_subtitle", (210, 210, 210))

    cam_border_color: RGB = (40, 120, 200)
    cam_inner_color: RGB = (10, 20, 40)

    # Background (caller already cleared with bg, but ensure it)
    surface.fill(color_bg)

    # ----------------------------------------------------------------------
    # Camera panel layout (top ~65% of the screen)
    # ----------------------------------------------------------------------
    margin = int(width * 0.04)
    cam_top = int(height * 0.06)
    cam_bottom = int(height * 0.70)
    cam_left = margin
    cam_right = width - margin

    cam_width = cam_right - cam_left
    cam_height = cam_bottom - cam_top

    cam_rect = pygame.Rect(cam_left, cam_top, cam_width, cam_height)

    # Draw camera placeholder
    pygame.draw.rect(surface, cam_inner_color, cam_rect)
    pygame.draw.rect(surface, cam_border_color, cam_rect, width=4)

    # Corner markers (simple "viewfinder" look)
    corner_len = min(cam_width, cam_height) // 10
    thickness = 3

    # top-left
    pygame.draw.line(surface, cam_border_color,
                     (cam_left, cam_top),
                     (cam_left + corner_len, cam_top),
                     thickness)
    pygame.draw.line(surface, cam_border_color,
                     (cam_left, cam_top),
                     (cam_left, cam_top + corner_len),
                     thickness)

    # top-right
    pygame.draw.line(surface, cam_border_color,
                     (cam_right, cam_top),
                     (cam_right - corner_len, cam_top),
                     thickness)
    pygame.draw.line(surface, cam_border_color,
                     (cam_right, cam_top),
                     (cam_right, cam_top + corner_len),
                     thickness)

    # bottom-left
    pygame.draw.line(surface, cam_border_color,
                     (cam_left, cam_bottom),
                     (cam_left + corner_len, cam_bottom),
                     thickness)
    pygame.draw.line(surface, cam_border_color,
                     (cam_left, cam_bottom),
                     (cam_left, cam_bottom - corner_len),
                     thickness)

    # bottom-right
    pygame.draw.line(surface, cam_border_color,
                     (cam_right, cam_bottom),
                     (cam_right - corner_len, cam_bottom),
                     thickness)
    pygame.draw.line(surface, cam_border_color,
                     (cam_right, cam_bottom),
                     (cam_right, cam_bottom - corner_len),
                     thickness)

    # Placeholder text inside camera area
    if font_status is not None:
        if camera_ready:
            cam_msg = "Camera active"
        else:
            cam_msg = "Starting camera…"  # or "No camera feed"

        cam_text = font_status.render(cam_msg, True, color_text_main)
        cam_text_rect = cam_text.get_rect(center=cam_rect.center)
        surface.blit(cam_text, cam_text_rect)

    # ----------------------------------------------------------------------
    # Navigation status overlay (just below camera)
    # ----------------------------------------------------------------------
    status_y = cam_bottom + int(height * 0.02)
    max_status_width = int(width * 0.92)

    _render_multiline_text(
        surface=surface,
        text=status_text or "Guiding…",
        font=font_status,
        color=color_text_status,
        max_width=max_status_width,
        start_y=status_y,
        line_spacing=2,
    )

    # ----------------------------------------------------------------------
    # Subtitle (TTS) at bottom
    # ----------------------------------------------------------------------
    subtitle_y = int(height * 0.80)
    max_subtitle_width = int(width * 0.92)

    _render_multiline_text(
        surface=surface,
        text=subtitle_text or "",
        font=font_subtitle,
        color=color_text_subtitle,
        max_width=max_subtitle_width,
        start_y=subtitle_y,
        line_spacing=3,
    )

    # ----------------------------------------------------------------------
    # Tiny "mouth bar" in bottom-left corner reacting to mouth_level
    # ----------------------------------------------------------------------
    m = max(0.0, min(1.0, float(mouth_level)))

    bar_width = int(width * 0.16)
    bar_height = int(height * 0.03)
    bar_margin_x = int(width * 0.04)
    bar_margin_y = int(height * 0.02)

    bar_x = bar_margin_x
    bar_y = height - bar_height - bar_margin_y

    # Border
    bar_rect = pygame.Rect(bar_x, bar_y, bar_width, bar_height)
    pygame.draw.rect(surface, (220, 220, 220), bar_rect, width=2)

    # Fill based on mouth_level
    inner_margin = 3
    inner_w = int((bar_width - 2 * inner_margin) * m)
    inner_h = bar_height - 2 * inner_margin

    level_color = (
        int(80 + 150 * m),
        int(120 + 80 * m),
        int(110),
    )
    inner_rect = pygame.Rect(
        bar_x + inner_margin,
        bar_y + inner_margin,
        inner_w,
        inner_h,
    )
    pygame.draw.rect(surface, level_color, inner_rect)
