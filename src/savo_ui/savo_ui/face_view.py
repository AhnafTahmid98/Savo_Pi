#!/usr/bin/env python3
"""
Robot Savo — Face view renderer (INTERACT / MAP modes)

This module draws a friendly robot face:

- Two eyes with pupils + small highlights.
- A mouth that opens/closes based on `mouth_level` (0.0–1.0).
- "Robot Savo" title at the top.
- Status text (one–two lines) above the bottom.
- Subtitle text (current TTS text) near the bottom edge.

It is completely stateless: all state is provided via function arguments.

Expected to be called from display_manager_node._draw_interact_mode() and
_display_map_mode() via:

    draw_face_view(
        surface=screen,
        mouth_level=self._mouth_level,
        status_text=self._status_text,
        subtitle_text=self._subtitle_text,
        fonts={...},
        colors={...},
        screen_size=(self.screen_width, self.screen_height),
    )
"""

from __future__ import annotations

from typing import Dict, Tuple

import pygame

RGB = Tuple[int, int, int]


# ============================================================================ #
# Internal helpers
# ============================================================================ #


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
    Render `text` as multiple lines constrained to `max_width`.

    Parameters
    ----------
    surface:
        Target surface.
    text:
        String to render. If empty, nothing is drawn.
    font:
        Loaded pygame font object.
    color:
        Text RGB color.
    max_width:
        Maximum width for each line.
    start_y:
        Initial Y coordinate to start drawing from (top of first line).
    line_spacing:
        Pixels between lines.

    Returns
    -------
    int
        The Y coordinate after the last rendered line (useful for stacking).
    """
    if not text or font is None:
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
    center_x = surface.get_width() // 2

    for line in lines:
        surf = font.render(line, True, color)
        rect = surf.get_rect()
        rect.centerx = center_x
        rect.y = y
        surface.blit(surf, rect)
        y = rect.bottom + line_spacing

    return y


# ============================================================================ #
# Public API
# ============================================================================ #


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
        Human-readable status line (short explanation of what robot does).
    subtitle_text:
        TTS text (exact sentence robot is currently speaking).
    fonts:
        Dict with keys "main", "status", "subtitle" (pygame.font.Font).
    colors:
        Dict with keys "bg", "text_main", "text_status", "text_subtitle".
    screen_size:
        (width, height) of the display surface.
    """
    width, height = screen_size

    # ---------------------------------------------------------------------- #
    # Fonts & colors (with robust fallbacks)
    # ---------------------------------------------------------------------- #
    font_main: pygame.font.Font | None = fonts.get("main")
    font_status: pygame.font.Font | None = fonts.get("status", font_main)
    font_subtitle: pygame.font.Font | None = fonts.get("subtitle", font_main)

    color_bg: RGB = colors.get("bg", (0, 0, 0))
    color_text_main: RGB = colors.get("text_main", (255, 255, 255))
    color_text_status: RGB = colors.get("text_status", (240, 240, 240))
    color_text_subtitle: RGB = colors.get("text_subtitle", (210, 210, 210))

    # Face palette (independent from parameter colors)
    eye_white: RGB = (245, 245, 245)
    eye_border: RGB = (30, 50, 80)
    pupil_color: RGB = (0, 0, 0)
    mouth_border: RGB = (255, 200, 120)
    mouth_fill: RGB = (255, 140, 100)

    surface.fill(color_bg)

    # ---------------------------------------------------------------------- #
    # Layout constants
    #
    # These are tuned for 800x480 but scale proportionally for other sizes.
    # ---------------------------------------------------------------------- #
    cx = width // 2

    # Title position: small margin from top
    title_y = int(height * 0.03)

    # Vertical anchor for the "face area".
    # On 800x480, 0.45 puts the face slightly above center, which looks natural.
    face_center_y = int(height * 0.45)

    # Eyes sit above the face center
    eye_center_y = int(height * 0.30)

    # Mouth sits between face center and lower half
    mouth_center_y = int(height * 0.56)

    # Status / subtitle vertical positions
    status_top_y = int(height * 0.68)
    subtitle_top_y = int(height * 0.80)

    max_text_width = int(width * 0.90)  # margin on left/right

    # ---------------------------------------------------------------------- #
    # Title ("Robot Savo")
    # ---------------------------------------------------------------------- #
    if font_main is not None:
        title_text = "Robot Savo"
        title_surf = font_main.render(title_text, True, color_text_main)
        title_rect = title_surf.get_rect(centerx=cx, y=title_y)
        surface.blit(title_surf, title_rect)

    # ---------------------------------------------------------------------- #
    # Eyes
    # ---------------------------------------------------------------------- #
    # Eye size scales with screen, slight bias for 16:9-ish shapes
    eye_spacing = int(width * 0.16)    # distance from center to each eye center
    eye_w = int(width * 0.14)
    eye_h = int(height * 0.22)

    left_eye_rect = pygame.Rect(
        cx - eye_spacing - eye_w // 2,
        eye_center_y - eye_h // 2,
        eye_w,
        eye_h,
    )
    right_eye_rect = pygame.Rect(
        cx + eye_spacing - eye_w // 2,
        eye_center_y - eye_h // 2,
        eye_w,
        eye_h,
    )

    # White of the eyes
    pygame.draw.ellipse(surface, eye_white, left_eye_rect)
    pygame.draw.ellipse(surface, eye_white, right_eye_rect)
    pygame.draw.ellipse(surface, eye_border, left_eye_rect, width=3)
    pygame.draw.ellipse(surface, eye_border, right_eye_rect, width=3)

    # Pupils with slight movement based on mouth_level (subtle "liveliness")
    m_clamped = max(0.0, min(1.0, float(mouth_level)))

    pupil_w = int(eye_w * 0.32)
    pupil_h = int(eye_h * 0.35)

    # Slight vertical offset (0 → neutral, 1 → slightly "smiling" eyes)
    pupil_offset_y = int(-0.06 * eye_h * m_clamped)

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

    # Small specular highlights for a more "alive" look
    highlight_r = max(2, pupil_w // 6)
    for rect in (left_pupil_rect, right_pupil_rect):
        highlight_center = (rect.centerx - highlight_r, rect.centery - highlight_r)
        pygame.draw.circle(surface, (255, 255, 255), highlight_center, highlight_r)

    # ---------------------------------------------------------------------- #
    # Mouth
    # ---------------------------------------------------------------------- #
    mouth_width = int(width * 0.34)
    mouth_height_max = int(height * 0.16)  # fully open
    mouth_height_min = int(height * 0.02)  # almost closed

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

    # Extra "smile" arc that brightens slightly when speaking
    smile_strength = 1.0 if m_clamped > 0.2 else m_clamped * 0.8
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
        0.1,   # radians
        3.0,   # radians
        width=3,
    )

    # ---------------------------------------------------------------------- #
    # Status & subtitle text (bottom area)
    # ---------------------------------------------------------------------- #
    _render_multiline_text(
        surface=surface,
        text=status_text or "",
        font=font_status,
        color=color_text_status,
        max_width=max_text_width,
        start_y=status_top_y,
        line_spacing=4,
    )

    _render_multiline_text(
        surface=surface,
        text=subtitle_text or "",
        font=font_subtitle,
        color=color_text_subtitle,
        max_width=max_text_width,
        start_y=subtitle_top_y,
        line_spacing=3,
    )
