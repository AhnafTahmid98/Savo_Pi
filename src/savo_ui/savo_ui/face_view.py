"""
Robot Savo — Face view helper

This module provides a single helper function:

    draw_face_view(surface, mouth_level, status_text, subtitle_text, fonts,
                   colors, screen_size)

It does NOT import rclpy or know anything about ROS. It only uses pygame and
the arguments passed in by display_manager_node.

Responsibilities:
- Draw a simple "face" on the given pygame surface:
    - Two eyes (white + pupil)
    - Mouth whose height is driven by mouth_level in [0.0, 1.0]
- Draw status_text and subtitle_text in reasonable positions.

The actual ROS node (savo_ui_display) is responsible for:
- Subscribing to topics
- Loading parameters from ui_params.yaml
- Creating the pygame surface and fonts
"""

from __future__ import annotations

from typing import Dict, Tuple

import pygame


RGB = Tuple[int, int, int]


def _lerp(a: float, b: float, t: float) -> float:
    """Linear interpolation between a and b with factor t in [0, 1]."""
    return (1.0 - t) * a + t * b


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
    Draw the Robot Savo face view on the given surface.

    Parameters
    ----------
    surface:
        Target pygame.Surface (usually the fullscreen DSI display surface).

    mouth_level:
        Float in [0.0, 1.0] controlling how "open" the mouth is.
        0.0 -> almost closed, 1.0 -> fully open.

    status_text:
        One- or two-line status text, e.g. "Guiding to A201" or
        "Mapping in progress, please keep distance."

    subtitle_text:
        Spoken text (from /savo_speech/tts_text). Typically shorter, may be
        empty when robot is silent.

    fonts:
        Dictionary of pygame.font.Font objects:
            fonts["main"]    -> main title (optional)
            fonts["status"]  -> status_text
            fonts["subtitle"]-> subtitle_text

    colors:
        Dictionary of RGB tuples:
            colors["bg"]            -> background color (already used by caller)
            colors["text_main"]     -> main/title text
            colors["text_status"]   -> status text
            colors["text_subtitle"] -> subtitle text

    screen_size:
        (width, height) of the target surface in pixels.
    """
    width, height = screen_size

    # --- Layout constants (tuned for 1024x600, scaled for other sizes) ----
    # These are chosen to look sensible on the 7" 1024x600 DSI, but they
    # scale proportionally with screen size.
    cx = width // 2
    cy = int(height * 0.45)  # face center a bit above vertical middle

    # Base scales
    base_w, base_h = 1024.0, 600.0
    scale_x = width / base_w
    scale_y = height / base_h
    # Use average scale for radii
    scale = 0.5 * (scale_x + scale_y)

    # Eyes
    eye_radius = int(40 * scale)
    pupil_radius = int(16 * scale)
    eye_offset_x = int(140 * scale_x)
    eye_offset_y = int(-40 * scale_y)

    # Mouth geometry
    mouth_offset_y = int(80 * scale_y)
    mouth_width = int(280 * scale_x)
    mouth_height_min = int(10 * scale_y)
    mouth_height_max = int(90 * scale_y)

    # --- Clamp inputs ------------------------------------------------------
    t = max(0.0, min(1.0, float(mouth_level)))
    status_text = (status_text or "").strip()
    subtitle_text = (subtitle_text or "").strip()

    # --- Draw eyes ---------------------------------------------------------
    eye_white_color: RGB = (235, 235, 235)
    eye_outline_color: RGB = (255, 255, 255)
    pupil_color: RGB = (30, 30, 30)

    # You can override these from colors dict in the future if needed
    eye_left_center = (cx - eye_offset_x, cy + eye_offset_y)
    eye_right_center = (cx + eye_offset_x, cy + eye_offset_y)

    # White part
    pygame.draw.circle(surface, eye_white_color, eye_left_center, eye_radius)
    pygame.draw.circle(surface, eye_white_color, eye_right_center, eye_radius)

    # Outline
    pygame.draw.circle(surface, eye_outline_color, eye_left_center, eye_radius, 2)
    pygame.draw.circle(surface, eye_outline_color, eye_right_center, eye_radius, 2)

    # Pupils: keep them centered for now (later you can animate gaze)
    pygame.draw.circle(surface, pupil_color, eye_left_center, pupil_radius)
    pygame.draw.circle(surface, pupil_color, eye_right_center, pupil_radius)

    # --- Draw mouth --------------------------------------------------------
    mouth_color: RGB = (230, 80, 80)
    mouth_outline_color: RGB = (255, 255, 255)

    # Compute mouth rect
    mouth_center_y = cy + mouth_offset_y
    current_height = int(_lerp(mouth_height_min, mouth_height_max, t))
    current_height = max(2, current_height)

    mouth_rect = pygame.Rect(
        cx - mouth_width // 2,
        mouth_center_y - current_height // 2,
        mouth_width,
        current_height,
    )

    # Slightly rounded effect by drawing two overlapping rects or by
    # using rounded rectangles if available in your pygame version.
    try:
        # pygame.draw.rect supports border_radius in recent versions
        pygame.draw.rect(
            surface,
            mouth_color,
            mouth_rect,
            border_radius=int(current_height * 0.4),
        )
        pygame.draw.rect(
            surface,
            mouth_outline_color,
            mouth_rect,
            width=2,
            border_radius=int(current_height * 0.4),
        )
    except TypeError:
        # Fallback if border_radius is not supported
        pygame.draw.rect(surface, mouth_color, mouth_rect)
        pygame.draw.rect(surface, mouth_outline_color, mouth_rect, width=2)

    # --- Draw texts --------------------------------------------------------
    # We assume fonts dict contains "status" and "subtitle".
    font_status = fonts.get("status") or fonts.get("main")
    font_subtitle = fonts.get("subtitle") or fonts.get("status") or fonts.get("main")

    if font_status is None or font_subtitle is None:
        # If fonts are missing, skip text drawing to avoid crashes.
        return

    # Status text near bottom-center
    if status_text:
        _draw_centered_text(
            surface=surface,
            text=status_text,
            font=font_status,
            color=colors.get("text_status", (255, 255, 255)),
            y=int(height * 0.80),
            max_width=int(width * 0.90),
        )

    # Subtitle below status text (if present)
    if subtitle_text:
        _draw_centered_text(
            surface=surface,
            text=subtitle_text,
            font=font_subtitle,
            color=colors.get("text_subtitle", (210, 210, 210)),
            y=int(height * 0.90),
            max_width=int(width * 0.90),
        )


def _draw_centered_text(
    surface: pygame.Surface,
    text: str,
    font: pygame.font.Font,
    color: RGB,
    y: int,
    max_width: int,
) -> None:
    """
    Draw a (possibly wrapped) text centered horizontally at a given y.

    This is a simple helper: it splits on spaces and builds lines that fit
    within max_width using font.size().
    """
    if not text:
        return

    words = text.split()
    if not words:
        return

    lines = []
    current = ""

    for word in words:
        candidate = f"{current} {word}".strip()
        width, _ = font.size(candidate)
        if width <= max_width or not current:
            current = candidate
        else:
            lines.append(current)
            current = word

    if current:
        lines.append(current)

    # Now render each line
    line_height = font.get_linesize()
    total_height = len(lines) * line_height
    start_y = y - total_height // 2

    surface_width = surface.get_width()

    for i, line in enumerate(lines):
        text_surface = font.render(line, True, color)
        text_rect = text_surface.get_rect()
        text_rect.centerx = surface_width // 2
        text_rect.y = start_y + i * line_height
        surface.blit(text_surface, text_rect)
