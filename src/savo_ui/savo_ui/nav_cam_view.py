"""
Robot Savo — Navigation camera view helper

This module provides a single helper function:

    draw_navigation_view(surface, status_text, subtitle_text, mouth_level,
                         fonts, colors, screen_size, camera_ready,
                         camera_surface=None)

It is responsible for drawing the NAVIGATE mode UI:

- Main area: camera view (or a placeholder if no frame is available yet).
- Status bar: "Guiding to A201" or similar text over the camera image.
- Optional subtitle: spoken text at the bottom.
- Optional small "face" overlay in a corner with a tiny mouth that moves
  based on mouth_level.

NOTE:
- This helper is intentionally independent of ROS. It only uses pygame and
  the arguments passed by display_manager_node.
- For now, camera_surface is optional and defaults to None. The
  display_manager_node currently does not pass frames; you can extend it
  later to pass a pygame.Surface here when you bridge sensor_msgs/Image.
"""

from __future__ import annotations

from typing import Dict, Tuple, Optional

import pygame


RGB = Tuple[int, int, int]


def draw_navigation_view(
    surface: pygame.Surface,
    status_text: str,
    subtitle_text: str,
    mouth_level: float,
    fonts: Dict[str, pygame.font.Font],
    colors: Dict[str, RGB],
    screen_size: Tuple[int, int],
    camera_ready: bool,
    camera_surface: Optional[pygame.Surface] = None,
) -> None:
    """
    Draw the Robot Savo navigation view on the given surface.

    Parameters
    ----------
    surface:
        Target pygame.Surface (usually the fullscreen DSI display surface).

    status_text:
        Status string, e.g. "Guiding to A201". Usually one or two lines.

    subtitle_text:
        Spoken text from /savo_speech/tts_text. May be empty.

    mouth_level:
        Float in [0.0, 1.0] controlling how "open" the tiny mouth is in
        the face overlay.

    fonts:
        Dictionary of pygame.font.Font objects:
            fonts["main"]     -> main / title (optional)
            fonts["status"]   -> status text (in the bar)
            fonts["subtitle"] -> subtitle at bottom

    colors:
        Dictionary of RGB tuples:
            colors["bg"]            -> default background (behind camera)
            colors["text_main"]     -> not used yet, reserved
            colors["text_status"]   -> status bar text
            colors["text_subtitle"] -> subtitle text

    screen_size:
        (width, height) of the target surface in pixels.

    camera_ready:
        True if we have started receiving frames. Even if camera_surface is
        None, you can use this to show "Camera active (placeholder)" instead
        of "Waiting for camera...".

    camera_surface:
        Optional pre-scaled camera frame surface. For now this will be None,
        but later you can pass a real frame from display_manager_node.
    """
    width, height = screen_size
    status_text = (status_text or "").strip()
    subtitle_text = (subtitle_text or "").strip()
    t = max(0.0, min(1.0, float(mouth_level)))

    font_status = fonts.get("status") or fonts.get("main")
    font_subtitle = fonts.get("subtitle") or fonts.get("status") or fonts.get("main")

    # ----------------------------------------------------------------------
    # 1) Clear background
    # ----------------------------------------------------------------------
    bg_color = colors.get("bg", (5, 35, 70))
    surface.fill(bg_color)

    # ----------------------------------------------------------------------
    # 2) Camera region
    #
    # For now:
    #   - If camera_surface is not provided, draw a placeholder rectangle.
    #   - Later you can blit a real camera image here and use letterboxing.
    # ----------------------------------------------------------------------
    cam_margin = int(width * 0.02)
    cam_top = int(height * 0.06)
    cam_bottom = int(height * 0.75)
    cam_height = max(20, cam_bottom - cam_top)
    cam_width = width - 2 * cam_margin

    cam_rect = pygame.Rect(cam_margin, cam_top, cam_width, cam_height)

    # Slightly darker camera background area
    camera_bg_color: RGB = (15, 15, 25)
    pygame.draw.rect(surface, camera_bg_color, cam_rect)

    # Thin border for the camera area
    pygame.draw.rect(surface, (220, 220, 220), cam_rect, width=2)

    # Inside camera area: either camera image or placeholder text
    if camera_surface is not None:
        # You can implement scaling/letterboxing here later.
        # For now we just center-fit the given surface inside cam_rect.
        _blit_centered(surface, camera_surface, cam_rect)
    else:
        placeholder_text = "Camera active (placeholder)" if camera_ready else "Waiting for camera..."
        if font_status is not None:
            _draw_centered_text_in_rect(
                surface=surface,
                text=placeholder_text,
                font=font_status,
                color=(200, 200, 200),
                rect=cam_rect,
            )

    # ----------------------------------------------------------------------
    # 3) Status bar over the camera view
    # ----------------------------------------------------------------------
    if font_status is not None:
        bar_height = int(height * 0.10)
        bar_rect = pygame.Rect(
            0,
            cam_bottom - bar_height // 2,
            width,
            bar_height,
        )

        bar_bg_color: RGB = (0, 0, 0)
        bar_line_color: RGB = (255, 255, 255)

        # Semi-transparent effect: draw smaller rect and border
        pygame.draw.rect(surface, bar_bg_color, bar_rect)
        pygame.draw.rect(surface, bar_line_color, bar_rect, width=1)

        if status_text:
            _draw_centered_text_in_rect(
                surface=surface,
                text=status_text,
                font=font_status,
                color=colors.get("text_status", (240, 240, 240)),
                rect=bar_rect,
                max_lines=2,
            )

    # ----------------------------------------------------------------------
    # 4) Subtitle (spoken text) near the bottom
    # ----------------------------------------------------------------------
    if subtitle_text and font_subtitle is not None:
        subtitle_rect = pygame.Rect(
            int(width * 0.05),
            int(height * 0.80),
            int(width * 0.90),
            int(height * 0.18),
        )
        _draw_centered_text_in_rect(
            surface=surface,
            text=subtitle_text,
            font=font_subtitle,
            color=colors.get("text_subtitle", (210, 210, 210)),
            rect=subtitle_rect,
            max_lines=2,
        )

    # ----------------------------------------------------------------------
    # 5) Small face overlay (bottom-right)
    # ----------------------------------------------------------------------
    # This is intentionally very simple: a small rounded box with a minimal
    # mouth rectangle that reacts to mouth_level.
    overlay_width = int(width * 0.26)
    overlay_height = int(height * 0.18)
    overlay_margin = int(width * 0.02)

    overlay_rect = pygame.Rect(
        width - overlay_width - overlay_margin,
        height - overlay_height - overlay_margin,
        overlay_width,
        overlay_height,
    )

    _draw_face_overlay(surface, overlay_rect, t)


# ==========================================================================#
# Helper drawing functions
# ==========================================================================#

def _blit_centered(
    surface: pygame.Surface,
    image: pygame.Surface,
    rect: pygame.Rect,
) -> None:
    """Blit image centered inside rect (no scaling)."""
    img_rect = image.get_rect()
    img_rect.center = rect.center
    surface.blit(image, img_rect)


def _draw_centered_text_in_rect(
    surface: pygame.Surface,
    text: str,
    font: pygame.font.Font,
    color: RGB,
    rect: pygame.Rect,
    max_lines: int = 3,
) -> None:
    """
    Draw (possibly wrapped) text centered within a rect.

    A simple word-wrapping is performed to avoid overflowing the rect width,
    and at most max_lines lines are drawn.
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
        if width <= rect.width or not current:
            current = candidate
        else:
            lines.append(current)
            current = word

    if current:
        lines.append(current)

    # Clamp to max_lines
    if len(lines) > max_lines:
        # Join extra lines into the last line and truncate
        extra = " ".join(lines[max_lines - 1 :])
        lines = lines[: max_lines - 1]
        lines.append(extra)

    line_height = font.get_linesize()
    total_height = len(lines) * line_height
    start_y = rect.centery - total_height // 2

    for i, line in enumerate(lines):
        text_surface = font.render(line, True, color)
        text_rect = text_surface.get_rect()
        text_rect.centerx = rect.centerx
        text_rect.y = start_y + i * line_height
        surface.blit(text_surface, text_rect)


def _draw_face_overlay(
    surface: pygame.Surface,
    rect: pygame.Rect,
    mouth_level: float,
) -> None:
    """
    Draw a minimal "face" overlay in a corner: simple eyes + mouth bar.

    mouth_level in [0.0, 1.0] controls the mouth height.
    """
    t = max(0.0, min(1.0, float(mouth_level)))

    # Background box
    bg_color: RGB = (15, 20, 35)
    outline_color: RGB = (230, 230, 230)
    mouth_color: RGB = (230, 80, 80)
    eye_color: RGB = (240, 240, 240)
    pupil_color: RGB = (30, 30, 30)

    try:
        pygame.draw.rect(
            surface,
            bg_color,
            rect,
            border_radius=int(min(rect.width, rect.height) * 0.15),
        )
        pygame.draw.rect(
            surface,
            outline_color,
            rect,
            width=1,
            border_radius=int(min(rect.width, rect.height) * 0.15),
        )
    except TypeError:
        pygame.draw.rect(surface, bg_color, rect)
        pygame.draw.rect(surface, outline_color, rect, width=1)

    # Simple eyes
    cx = rect.centerx
    cy = rect.top + int(rect.height * 0.35)
    eye_offset_x = int(rect.width * 0.18)
    eye_radius = max(3, int(rect.height * 0.10))
    pupil_radius = max(2, int(eye_radius * 0.45))

    left_eye_center = (cx - eye_offset_x, cy)
    right_eye_center = (cx + eye_offset_x, cy)

    pygame.draw.circle(surface, eye_color, left_eye_center, eye_radius)
    pygame.draw.circle(surface, eye_color, right_eye_center, eye_radius)
    pygame.draw.circle(surface, pupil_color, left_eye_center, pupil_radius)
    pygame.draw.circle(surface, pupil_color, right_eye_center, pupil_radius)

    # Mouth bar
    mouth_width = int(rect.width * 0.60)
    mouth_height_min = max(2, int(rect.height * 0.06))
    mouth_height_max = max(mouth_height_min + 1, int(rect.height * 0.30))
    current_height = int(mouth_height_min + t * (mouth_height_max - mouth_height_min))

    mouth_center_y = rect.bottom - int(rect.height * 0.25)

    mouth_rect = pygame.Rect(
        rect.centerx - mouth_width // 2,
        mouth_center_y - current_height // 2,
        mouth_width,
        current_height,
    )

    try:
        pygame.draw.rect(
            surface,
            mouth_color,
            mouth_rect,
            border_radius=int(current_height * 0.4),
        )
    except TypeError:
        pygame.draw.rect(surface, mouth_color, mouth_rect)
