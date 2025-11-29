#!/usr/bin/env python3
"""
Robot Savo — Face view renderer (INTERACT / MAP modes)

This module draws a simple friendly robot face:

- Two eyes (with pupils).
- A mouth that opens based on `mouth_level` (0.0–1.0).
- Status text (one–two lines).
- Subtitle text (current TTS text) near the bottom.

It is stateless: all information comes from the arguments.
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
) -> int:
    """
    Render `text` as multiple lines so that each line fits within `max_width`.
    Returns the y-coordinate after the last rendered line.
    """
    if not text:
        return start_y

    words = text.split()
    if not words:
        return start_y

    lines = []
    current_line: list[str] = []

    for word in words:
        trial = " ".join(current_line + [word]) if current_line else word
        w, _ = font.size(trial)
        if w <= max_width:
            current_line.append(word)
        else:
            if current_line:
                lines.append(" ".join(current_line))
            current_line = [word]

    if current_line:
        lines.append(" ".join(current_line))

    y = start_y
    for line in lines:
        surf = font.render(line, True, color)
        rect = surf.get_rect(centerx=surface.get_width() // 2, y=y)
        surface.blit(surf, rect)
        y = rect.bottom + line_spacing

    return y


def draw_face_view(
    surface: pygame.Surface,
    mouth_level: float,
    status_text: str,
    subtitle_text: str,
    fonts: Dict[str, pygame.font.Font],
    colors: Dict[str, RGB],
    screen_size: Tuple[int, int],
) -> None:
    """
    Draw Robot Savo's face view.

    Parameters
    ----------
    surface:
        Pygame surface to draw onto (already cleared by caller).
    mouth_level:
        0.0–1.0 value from /savo_speech/mouth_level controlling mouth opening.
    status_text:
        Human-readable status line (small text above subtitles).
    subtitle_text:
        TTS text (what Robot Savo is currently saying).
    fonts:
        Dict with keys "main", "status", "subtitle".
    colors:
        Dict with keys "bg", "text_main", "text_status", "text_subtitle".
    screen_size:
        (width, height) of the display surface.
    """
    width, height = screen_size

    # ----------------------------------------------------------------------
    # Unpack fonts & colors (with safe fallbacks)
    # ----------------------------------------------------------------------
    font_main = fonts.get("main")
    font_status = fonts.get("status", font_main)
    font_subtitle = fonts.get("subtitle", font_main)

    color_bg: RGB = colors.get("bg", (0, 0, 0))
    color_text_main: RGB = colors.get("text_main", (255, 255, 255))
    color_text_status: RGB = colors.get("text_status", (240, 240, 240))
    color_text_subtitle: RGB = colors.get("text_subtitle", (210, 210, 210))

    # Face colors
    eye_white: RGB = (245, 245, 245)
    eye_border: RGB = (30, 50, 80)
    pupil_color: RGB = (0, 0, 0)
    mouth_border: RGB = (255, 200, 120)
    mouth_fill: RGB = (255, 140, 100)

    # Caller already filled bg, but re-fill to be safe if needed
    surface.fill(color_bg)

    # ----------------------------------------------------------------------
    # Face layout (relative to screen size)
    # ----------------------------------------------------------------------
    cx = width // 2
    cy = height // 2 - int(0.08 * height)  # slightly above center

    # Eyes
    eye_spacing = int(width * 0.16)        # distance from center to each eye center
    eye_w = int(width * 0.14)
    eye_h = int(height * 0.20)
    eye_y = cy - int(0.25 * height)

    left_eye_rect = pygame.Rect(
        cx - eye_spacing - eye_w // 2, eye_y, eye_w, eye_h
    )
    right_eye_rect = pygame.Rect(
        cx + eye_spacing - eye_w // 2, eye_y, eye_w, eye_h
    )

    # Draw eye whites
    pygame.draw.ellipse(surface, eye_white, left_eye_rect)
    pygame.draw.ellipse(surface, eye_white, right_eye_rect)
    pygame.draw.ellipse(surface, eye_border, left_eye_rect, width=3)
    pygame.draw.ellipse(surface, eye_border, right_eye_rect, width=3)

    # Pupils: very small movement based on mouth_level (looks "alive")
    pupil_w = int(eye_w * 0.32)
    pupil_h = int(eye_h * 0.35)

    # Slight up/down offset with mouth_level (0 → relaxed, 1 → slightly up)
    pupil_offset_y = int(-0.06 * eye_h * mouth_level)

    # Left pupil
    left_pupil_rect = pygame.Rect(0, 0, pupil_w, pupil_h)
    left_pupil_rect.center = (
        left_eye_rect.centerx,
        left_eye_rect.centery + pupil_offset_y,
    )

    # Right pupil
    right_pupil_rect = pygame.Rect(0, 0, pupil_w, pupil_h)
    right_pupil_rect.center = (
        right_eye_rect.centerx,
        right_eye_rect.centery + pupil_offset_y,
    )

    pygame.draw.ellipse(surface, pupil_color, left_pupil_rect)
    pygame.draw.ellipse(surface, pupil_color, right_pupil_rect)

    # Optional small eye highlights
    highlight_r = max(2, pupil_w // 6)
    for rect in (left_pupil_rect, right_pupil_rect):
        highlight_center = (rect.centerx - highlight_r, rect.centery - highlight_r)
        pygame.draw.circle(surface, (255, 255, 255), highlight_center, highlight_r)

    # ----------------------------------------------------------------------
    # Mouth (responds to mouth_level)
    # ----------------------------------------------------------------------
    # Clamp mouth_level defensively
    m = max(0.0, min(1.0, float(mouth_level)))

    mouth_width = int(width * 0.32)
    mouth_height_max = int(height * 0.14)  # fully open
    mouth_height_min = int(height * 0.015)  # almost closed

    # Interpolate open height
    open_height = int(mouth_height_min + (mouth_height_max - mouth_height_min) * m)

    mouth_center_y = cy + int(0.10 * height)
    mouth_rect = pygame.Rect(
        cx - mouth_width // 2,
        mouth_center_y - open_height // 2,
        mouth_width,
        open_height,
    )

    # Filled mouth
    pygame.draw.ellipse(surface, mouth_fill, mouth_rect)
    pygame.draw.ellipse(surface, mouth_border, mouth_rect, width=4)

    # Optional "smile" accent line (slightly brighter when speaking)
    smile_strength = 1.0 if m > 0.2 else m * 0.8
    smile_color = (
        min(255, int(mouth_border[0] + 40 * smile_strength)),
        min(255, int(mouth_border[1] + 20 * smile_strength)),
        mouth_border[2],
    )
    smile_y = mouth_rect.centery + open_height // 4
    smile_left = mouth_rect.left + int(mouth_width * 0.08)
    smile_right = mouth_rect.right - int(mouth_width * 0.08)
    pygame.draw.arc(
        surface,
        smile_color,
        pygame.Rect(
            smile_left,
            smile_y - open_height // 3,
            smile_right - smile_left,
            open_height,
        ),
        0.1,
        3.0,
        width=3,
    )

    # ----------------------------------------------------------------------
    # Optional title ("Robot Savo") above the face (using main font)
    # ----------------------------------------------------------------------
    if font_main is not None:
        title_text = "Robot Savo"
        title_surf = font_main.render(title_text, True, color_text_main)
        title_rect = title_surf.get_rect(
            centerx=cx,
            y=int(height * 0.05),
        )
        surface.blit(title_surf, title_rect)

    # ----------------------------------------------------------------------
    # Status text (one–two lines) below the face
    # ----------------------------------------------------------------------
    status_top_y = int(height * 0.68)
    max_text_width = int(width * 0.90)

    _render_multiline_text(
        surface=surface,
        text=status_text or "",
        font=font_status,
        color=color_text_status,
        max_width=max_text_width,
        start_y=status_top_y,
        line_spacing=4,
    )

    # ----------------------------------------------------------------------
    # Subtitle text (TTS) near the bottom
    # ----------------------------------------------------------------------
    subtitle_top_y = int(height * 0.80)

    _render_multiline_text(
        surface=surface,
        text=subtitle_text or "",
        font=font_subtitle,
        color=color_text_subtitle,
        max_width=max_text_width,
        start_y=subtitle_top_y,
        line_spacing=3,
    )
