#!/usr/bin/env python3
"""
Robot Savo — Face view renderer (INTERACT mode)

This module draws the main Robot Savo face, inspired by the small square
icon you liked:

- Rounded square with cyan border
- Dark inner face area
- Light bottom "chin" bar
- Two big eyes with cyan rings and highlights
- Simple smiling mouth whose height animates with mouth_level
- Status text and subtitle below the face

Used by display_manager_node._draw_interact_mode().
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
    """Render text as multiple lines within max_width, return final y."""
    if not text or font is None:
        return start_y

    words = text.split()
    if not words:
        return start_y

    lines: list[str] = []
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
    Draw the INTERACT face view.

    Parameters
    ----------
    surface:
        Pygame surface to draw onto.
    mouth_level:
        0.0–1.0 mouth activity (controls mouth height).
    status_text:
        One or two short lines like "Hi, I am Robot Savo!".
    subtitle_text:
        Spoken text / extra info.
    fonts:
        Dict with keys "main", "status", "subtitle".
    colors:
        Dict with keys "bg", "text_main", "text_status", "text_subtitle".
    screen_size:
        (width, height) of the display.
    """
    width, height = screen_size

    font_main = fonts.get("main")
    font_status = fonts.get("status", font_main)
    font_subtitle = fonts.get("subtitle", font_main)

    color_bg: RGB = colors.get("bg", (10, 25, 50))
    color_text_main: RGB = colors.get("text_main", (255, 255, 255))
    color_text_status: RGB = colors.get("text_status", (240, 240, 240))
    color_text_subtitle: RGB = colors.get("text_subtitle", (210, 210, 210))

    # Fill background
    surface.fill(color_bg)

    # ----------------------------------------------------------------------
    # Face panel layout (roughly square, centered near top)
    # ----------------------------------------------------------------------
    face_size = int(min(width, height) * 0.55)
    face_x = (width - face_size) // 2
    face_y = int(height * 0.08)
    face_rect = pygame.Rect(face_x, face_y, face_size, face_size)

    border_radius_outer = int(face_size * 0.18)
    border_radius_inner = int(face_size * 0.16)

    # Colors similar to the icon
    border_color: RGB = (90, 230, 255)    # cyan border
    inner_color: RGB = (20, 32, 64)       # dark navy face
    chin_color: RGB = (235, 244, 255)     # light chin strip

    # Outer rounded border
    pygame.draw.rect(surface, border_color, face_rect, border_radius=border_radius_outer)

    # Slight inner inset for dark face area
    inset = max(4, face_size // 40)
    inner_rect = face_rect.inflate(-2 * inset, -2 * inset)
    pygame.draw.rect(surface, inner_color, inner_rect, border_radius=border_radius_inner)

    # ----------------------------------------------------------------------
    # Chin strip (bottom light bar inside the face)
    # ----------------------------------------------------------------------
    chin_height = int(inner_rect.height * 0.32)
    chin_rect = pygame.Rect(
        inner_rect.x,
        inner_rect.bottom - chin_height,
        inner_rect.width,
        chin_height,
    )
    pygame.draw.rect(surface, chin_color, chin_rect, border_radius=int(border_radius_inner * 0.9))

    # ----------------------------------------------------------------------
    # Eyes (in dark upper part)
    # ----------------------------------------------------------------------
    eye_center_y = inner_rect.y + int(inner_rect.height * 0.33)
    eye_offset_x = int(inner_rect.width * 0.23)
    eye_radius_outer = int(face_size * 0.09)
    eye_radius_inner = int(face_size * 0.06)
    eye_radius_highlight = int(face_size * 0.025)

    eye_left_center = (inner_rect.centerx - eye_offset_x, eye_center_y)
    eye_right_center = (inner_rect.centerx + eye_offset_x, eye_center_y)

    iris_outer_color: RGB = (100, 240, 255)  # cyan ring
    iris_inner_color: RGB = (255, 255, 255)  # white inner
    pupil_color: RGB = (25, 45, 80)          # dark blue
    highlight_color: RGB = (255, 255, 255)

    def _draw_eye(center: Tuple[int, int]) -> None:
        # Outer glow ring
        pygame.draw.circle(surface, iris_outer_color, center, eye_radius_outer)
        # Inner white circle
        pygame.draw.circle(surface, iris_inner_color, center, eye_radius_inner)
        # Pupil
        pupil_radius = int(eye_radius_inner * 0.55)
        pygame.draw.circle(surface, pupil_color, center, pupil_radius)
        # Highlight (small circle up-left in pupil)
        highlight_center = (
            center[0] - int(pupil_radius * 0.35),
            center[1] - int(pupil_radius * 0.35),
        )
        pygame.draw.circle(surface, highlight_color, highlight_center, eye_radius_highlight)

    _draw_eye(eye_left_center)
    _draw_eye(eye_right_center)

    # Optional small eyebrows (simple rounded rectangles)
    brow_width = int(face_size * 0.16)
    brow_height = int(face_size * 0.02)
    brow_offset_y = int(face_size * 0.07)
    brow_color: RGB = (210, 240, 255)

    left_brow_rect = pygame.Rect(
        eye_left_center[0] - brow_width // 2,
        eye_left_center[1] - brow_offset_y,
        brow_width,
        brow_height,
    )
    right_brow_rect = pygame.Rect(
        eye_right_center[0] - brow_width // 2,
        eye_right_center[1] - brow_offset_y,
        brow_width,
        brow_height,
    )

    pygame.draw.rect(surface, brow_color, left_brow_rect, border_radius=brow_height // 2)
    pygame.draw.rect(surface, brow_color, right_brow_rect, border_radius=brow_height // 2)

    # ----------------------------------------------------------------------
    # Mouth (in the chin area, animated by mouth_level)
    # ----------------------------------------------------------------------
    m = max(0.0, min(1.0, float(mouth_level)))

    mouth_width = int(chin_rect.width * 0.26)
    mouth_max_height = int(chin_rect.height * 0.35)
    mouth_min_height = max(4, mouth_max_height // 3)
    mouth_height = int(mouth_min_height + (mouth_max_height - mouth_min_height) * m)

    mouth_center_x = chin_rect.centerx
    mouth_center_y = chin_rect.y + int(chin_rect.height * 0.55)

    mouth_rect = pygame.Rect(
        mouth_center_x - mouth_width // 2,
        mouth_center_y - mouth_height // 2,
        mouth_width,
        mouth_height,
    )

    mouth_color: RGB = (40, 50, 90)
    pygame.draw.ellipse(surface, mouth_color, mouth_rect)

    # ----------------------------------------------------------------------
    # Status + subtitle text under the face
    # ----------------------------------------------------------------------
    text_block_top = face_rect.bottom + int(height * 0.03)
    max_text_width = int(width * 0.90)

    # Main status (slightly larger, maybe bold)
    y = text_block_top
    if status_text and font_main is not None:
        y = _render_multiline_text(
            surface=surface,
            text=status_text,
            font=font_main,
            color=color_text_status,
            max_width=max_text_width,
            start_y=y,
            line_spacing=4,
        )

    # Subtitle (smaller, dimmer)
    if subtitle_text and font_subtitle is not None:
        y += int(height * 0.01)
        _render_multiline_text(
            surface=surface,
            text=subtitle_text,
            font=font_subtitle,
            color=color_text_subtitle,
            max_width=max_text_width,
            start_y=y,
            line_spacing=3,
        )
