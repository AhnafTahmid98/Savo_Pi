#!/usr/bin/env python3
"""
Robot Savo — Face view renderer (INTERACT / MAP modes, v3 — text-less)

This module draws a friendly robot face:

- Two eyes with pupils + highlights.
- Eyelids / pupil position reacting to `face_state`:
    - "idle"      → neutral, soft expression
    - "listening" → eyes slightly raised and more open
    - "thinking"  → eyes slightly squinted, looking up
    - "speaking"  → eyes bright and open, mouth more animated
- A mouth that opens/closes based on `mouth_level` (0.0–1.0),
  with a small extra boost in "speaking" state.
- Date & time in the top-left corner.

IMPORTANT:
- We IGNORE status_text and subtitle_text on purpose.
- Only the small clock text is rendered; no bottom text is drawn.
"""

from __future__ import annotations

from typing import Dict, Tuple
import datetime

import pygame

RGB = Tuple[int, int, int]


def draw_face_view(
    surface: pygame.Surface,
    mouth_level: float,
    status_text: str,      # kept for API compatibility, but ignored
    subtitle_text: str,    # kept for API compatibility, but ignored
    face_state: str,
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
        0.0–1.0 value controlling mouth opening (from speech node).
    status_text:
        Human-readable status line (IGNORED in this version).
    subtitle_text:
        TTS text (spoken sentence, IGNORED in this version).
    face_state:
        High-level expression state:
            "idle" / "listening" / "thinking" / "speaking"
    fonts:
        Dict with keys "main", "status", "subtitle".
    colors:
        Dict with keys "bg", "text_main", "text_status", "text_subtitle".
    screen_size:
        (width, height) of the display.
    """
    width, height = screen_size

    # ------------------------------------------------------------------ #
    # Fonts & colors
    # ------------------------------------------------------------------ #
    font_main: pygame.font.Font | None = fonts.get("main")
    font_status: pygame.font.Font | None = fonts.get("status", font_main)
    font_subtitle: pygame.font.Font | None = fonts.get("subtitle", font_main)

    color_bg: RGB = colors.get("bg", (0, 0, 0))
    color_text_status: RGB = colors.get("text_status", (240, 240, 240))

    # Base face palette
    eye_white: RGB = (245, 245, 245)
    eye_border: RGB = (30, 50, 80)
    pupil_color: RGB = (0, 0, 0)
    mouth_border: RGB = (255, 200, 120)
    mouth_fill: RGB = (255, 140, 100)

    # ------------------------------------------------------------------ #
    # Face state configuration
    # ------------------------------------------------------------------ #
    state = (face_state or "idle").strip().lower()
    if state not in ("idle", "listening", "thinking", "speaking"):
        state = "idle"

    # Small style tweaks per state
    if state == "idle":
        eye_open_factor_base = 1.0
        pupil_vert_offset_factor = 0.0        # neutral
        mouth_boost = 0.05                    # a tiny, relaxed open
        iris_tint: RGB = (0, 190, 255)
    elif state == "listening":
        eye_open_factor_base = 1.05           # slightly more open
        pupil_vert_offset_factor = -0.10      # looking a bit up
        mouth_boost = 0.08
        iris_tint = (0, 255, 210)
    elif state == "thinking":
        eye_open_factor_base = 0.80           # slightly squinted
        pupil_vert_offset_factor = -0.18      # looking more up
        mouth_boost = 0.02
        iris_tint = (120, 210, 255)
    else:  # "speaking"
        eye_open_factor_base = 1.08           # a bit excited
        pupil_vert_offset_factor = -0.04
        mouth_boost = 0.20                    # talky mouth
        iris_tint = (0, 255, 180)

    # Effective mouth open (clamped)
    m_raw = float(mouth_level)
    m_clamped = max(0.0, min(1.0, m_raw + mouth_boost))

    # ------------------------------------------------------------------ #
    # Time (for clock + simple blinking)
    # ------------------------------------------------------------------ #
    now = datetime.datetime.now()
    dt_seconds = now.timestamp()

    # Example: "Sat 29 Nov 14:32"
    dt_str = now.strftime("%a %d %b %H:%M")

    # Simple blink: every ~4 seconds, blink for ~0.15 s
    blink_cycle_s = 4.0
    blink_duration_s = 0.15
    phase = dt_seconds % blink_cycle_s
    blink_active = phase < blink_duration_s

    # Combine base eye-open factor with blink
    eye_open_factor = eye_open_factor_base * (0.25 if blink_active else 1.0)
    eye_open_factor = max(0.35, min(1.15, eye_open_factor))

    surface.fill(color_bg)

    # ------------------------------------------------------------------ #
    # Layout (tuned for 800x480, with face slightly lower)
    # ------------------------------------------------------------------ #
    cx = width // 2

    eye_center_y = int(height * 0.40)   # eyes a bit below center
    mouth_center_y = int(height * 0.66) # mouth lower

    # ------------------------------------------------------------------ #
    # Subtle face "plate" behind the eyes/mouth (just a rounded rect)
    # ------------------------------------------------------------------ #
    plate_margin_x = int(width * 0.10)
    plate_margin_y_top = int(height * 0.17)
    plate_margin_y_bottom = int(height * 0.12)

    plate_rect = pygame.Rect(
        plate_margin_x,
        plate_margin_y_top,
        width - 2 * plate_margin_x,
        height - plate_margin_y_top - plate_margin_y_bottom,
    )
    plate_color = (
        min(255, color_bg[0] + 18),
        min(255, color_bg[1] + 24),
        min(255, color_bg[2] + 28),
    )
    pygame.draw.rect(surface, plate_color, plate_rect, border_radius=32)

    # ------------------------------------------------------------------ #
    # Date & time (top-left) — uses Pi local time
    # ------------------------------------------------------------------ #
    clock_font = font_subtitle or font_status or font_main
    if clock_font is not None:
        clock_surf = clock_font.render(dt_str, True, color_text_status)
        clock_rect = clock_surf.get_rect()
        clock_rect.topleft = (int(width * 0.03), int(height * 0.03))
        surface.blit(clock_surf, clock_rect)

    # ------------------------------------------------------------------ #
    # Eyes
    # ------------------------------------------------------------------ #
    eye_spacing = int(width * 0.16)
    eye_w = int(width * 0.14)
    eye_h = int(height * 0.22)

    # Apply eye-open factor by shrinking vertical eye size
    eye_h_eff = max(6, int(eye_h * eye_open_factor))

    left_eye_rect = pygame.Rect(
        cx - eye_spacing - eye_w // 2,
        eye_center_y - eye_h_eff // 2,
        eye_w,
        eye_h_eff,
    )
    right_eye_rect = pygame.Rect(
        cx + eye_spacing - eye_w // 2,
        eye_center_y - eye_h_eff // 2,
        eye_w,
        eye_h_eff,
    )

    # Eye whites + border
    pygame.draw.ellipse(surface, eye_white, left_eye_rect)
    pygame.draw.ellipse(surface, eye_white, right_eye_rect)
    pygame.draw.ellipse(surface, eye_border, left_eye_rect, width=3)
    pygame.draw.ellipse(surface, eye_border, right_eye_rect, width=3)

    # Pupil jitter from mouth + face_state
    pupil_w = int(eye_w * 0.32)
    pupil_h = int(eye_h_eff * 0.35)

    # base vertical offset from mouth_movement, then add face_state offset
    pupil_offset_y_from_mouth = int(-0.06 * eye_h_eff * m_clamped)
    pupil_offset_y_from_state = int(pupil_vert_offset_factor * eye_h_eff)
    pupil_offset_y = pupil_offset_y_from_mouth + pupil_offset_y_from_state

    left_pupil_rect = pygame.Rect(0, 0, pupil_w, pupil_h)
    left_pupil_rect.center = (
        left_eye_rect.centerx,
        left_eye_rect.centery + pupil_offset_y,
    )

    right_pupil_rect = pygame.Rect(0, 0, pupil_w, pupil_h)
    right_pupil_rect.center = (
        right_eye_rect.centerx,
        right_eye_rect.centery + pupil_offset_y,
    )

    pygame.draw.ellipse(surface, pupil_color, left_pupil_rect)
    pygame.draw.ellipse(surface, pupil_color, right_pupil_rect)

    # Iris tint (subtle colored ring around pupil)
    iris_margin = max(2, int(pupil_w * 0.10))
    iris_rect_left = left_pupil_rect.inflate(iris_margin * 2, iris_margin * 2)
    iris_rect_right = right_pupil_rect.inflate(iris_margin * 2, iris_margin * 2)
    pygame.draw.ellipse(surface, iris_tint, iris_rect_left, width=2)
    pygame.draw.ellipse(surface, iris_tint, iris_rect_right, width=2)

    # Highlights
    highlight_r = max(2, pupil_w // 6)
    for rect in (left_pupil_rect, right_pupil_rect):
        highlight_center = (rect.centerx - highlight_r, rect.centery - highlight_r)
        pygame.draw.circle(surface, (255, 255, 255), highlight_center, highlight_r)

    # ------------------------------------------------------------------ #
    # Eyebrows (react to face_state)
    # ------------------------------------------------------------------ #
    brow_color = eye_border
    brow_thickness = 3

    # Brow vertical offset per state (small tweaks)
    if state == "idle":
        brow_offset_y = -int(eye_h_eff * 0.55)
        brow_curve = 0.0
    elif state == "listening":
        brow_offset_y = -int(eye_h_eff * 0.70)
        brow_curve = -0.04
    elif state == "thinking":
        brow_offset_y = -int(eye_h_eff * 0.60)
        brow_curve = 0.08  # inward slight frown
    else:  # speaking
        brow_offset_y = -int(eye_h_eff * 0.72)
        brow_curve = -0.02

    def _draw_brow(eye_rect: pygame.Rect, sign: int) -> None:
        # sign = -1 for left, +1 for right
        ex = eye_rect.centerx
        ey = eye_rect.centery + brow_offset_y

        half_len = int(eye_w * 0.55)
        x1 = ex - half_len
        x2 = ex + half_len

        # Slight tilt based on brow_curve; left/right mirrored by sign
        y1 = ey + int(brow_curve * half_len * sign)
        y2 = ey - int(brow_curve * half_len * sign)

        pygame.draw.line(surface, brow_color, (x1, y1), (x2, y2), brow_thickness)

    _draw_brow(left_eye_rect, sign=-1)
    _draw_brow(right_eye_rect, sign=+1)

    # ------------------------------------------------------------------ #
    # Mouth
    # ------------------------------------------------------------------ #
    mouth_width = int(width * 0.34)
    mouth_height_max = int(height * 0.16)
    mouth_height_min = int(height * 0.02)

    open_height = int(
        mouth_height_min + (mouth_height_max - mouth_height_min) * m_clamped
    )

    mouth_rect = pygame.Rect(
        cx - mouth_width // 2,
        mouth_center_y - open_height // 2,
        mouth_width,
        open_height,
    )

    pygame.draw.ellipse(surface, mouth_fill, mouth_rect)
    pygame.draw.ellipse(surface, mouth_border, mouth_rect, width=4)

    # Smile strength: more smile when mouth is more open or in "speaking".
    smile_strength = 0.25 + 0.75 * m_clamped
    if state == "speaking":
        smile_strength = min(1.0, smile_strength + 0.15)

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

    # NOTE: We intentionally DO NOT draw status_text or subtitle_text here.
    # Any spoken/debug lines on the screen must come from somewhere else.
