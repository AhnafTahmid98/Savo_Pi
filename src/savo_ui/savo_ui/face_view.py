#!/usr/bin/env python3
"""
Robot Savo — Face view renderer (INTERACT mode, simple clean version)

This draws:
- One big rounded rectangle "head" in the middle
- Two round eyes
- A small smiling mouth that moves a bit with mouth_level
- Status text + subtitle under the face
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
    Draw the INTERACT face view (simple friendly face).
    """
    width, height = screen_size

    font_main = fonts.get("main")
    font_status = fonts.get("status", font_main)
    font_subtitle = fonts.get("subtitle", font_main)

    color_bg: RGB = colors.get("bg", (10, 25, 50))
    color_text_status: RGB = colors.get("text_status", (240, 240, 240))
    color_text_subtitle: RGB = colors.get("text_subtitle", (210, 210, 210))

    # Background
    surface.fill(color_bg)

    # ----------------------------------------------------------------------
    # Big head panel (rounded rectangle)
    # ----------------------------------------------------------------------
    head_width = int(width * 0.65)
    head_height = int(height * 0.55)
    head_x = (width - head_width) // 2
    head_y = int(height * 0.10)
    head_rect = pygame.Rect(head_x, head_y, head_width, head_height)

    border_radius = int(min(head_width, head_height) * 0.18)

    head_border_color: RGB = (80, 210, 255)   # cyan border
    head_inner_color: RGB = (15, 30, 60)      # dark navy

    # Border + inner fill
    pygame.draw.rect(surface, head_border_color, head_rect, border_radius=border_radius)
    inner_rect = head_rect.inflate(-8, -8)
    pygame.draw.rect(surface, head_inner_color, inner_rect, border_radius=border_radius - 4)

    # ----------------------------------------------------------------------
    # Eyes
    # ----------------------------------------------------------------------
    eye_center_y = inner_rect.y + int(inner_rect.height * 0.35)
    eye_offset_x = int(inner_rect.width * 0.23)
    eye_radius = int(min(head_width, head_height) * 0.09)

    left_eye_center = (inner_rect.centerx - eye_offset_x, eye_center_y)
    right_eye_center = (inner_rect.centerx + eye_offset_x, eye_center_y)

    eye_white: RGB = (240, 250, 255)
    eye_iris: RGB = (100, 230, 255)
    eye_pupil: RGB = (20, 40, 80)

    def _draw_eye(center):
        pygame.draw.circle(surface, eye_iris, center, eye_radius)
        pygame.draw.circle(surface, eye_white, center, int(eye_radius * 0.70))
        pygame.draw.circle(surface, eye_pupil, center, int(eye_radius * 0.40))
        # small highlight
        hl = int(eye_radius * 0.18)
        pygame.draw.circle(
            surface,
            (255, 255, 255),
            (center[0] - hl, center[1] - hl),
            hl,
        )

    _draw_eye(left_eye_center)
    _draw_eye(right_eye_center)

    # ----------------------------------------------------------------------
    # Mouth (simple smile, height animated with mouth_level)
    # ----------------------------------------------------------------------
    m = max(0.0, min(1.0, float(mouth_level)))
    mouth_width = int(inner_rect.width * 0.30)
    mouth_height_min = max(3, int(inner_rect.height * 0.03))
    mouth_height_max = int(inner_rect.height * 0.10)
    mouth_height = int(mouth_height_min + (mouth_height_max - mouth_height_min) * m)

    mouth_center_x = inner_rect.centerx
    mouth_center_y = inner_rect.y + int(inner_rect.height * 0.68)

    mouth_rect = pygame.Rect(
        mouth_center_x - mouth_width // 2,
        mouth_center_y - mouth_height // 2,
        mouth_width,
        mouth_height,
    )

    mouth_color: RGB = (30, 50, 90)
    pygame.draw.ellipse(surface, mouth_color, mouth_rect)

    # ----------------------------------------------------------------------
    # Status + subtitle text under the face
    # ----------------------------------------------------------------------
    text_top = head_rect.bottom + int(height * 0.04)
    max_text_width = int(width * 0.90)

    y = text_top
    if status_text and font_status is not None:
        y = _render_multiline_text(
            surface=surface,
            text=status_text,
            font=font_status,
            color=color_text_status,
            max_width=max_text_width,
            start_y=y,
            line_spacing=4,
        )

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
