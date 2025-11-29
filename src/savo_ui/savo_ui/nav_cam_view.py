#!/usr/bin/env python3
"""
Robot Savo — NAVIGATE camera view renderer

This module draws:

- Live camera view (full-screen, letter-boxed if needed).
- A top status bar with guidance text (status_text).
- A bottom bar with subtitle / speech text (subtitle_text).
- A simple "talk level" meter based on mouth_level.

It expects that the display manager passes in the latest camera frame
as a NumPy array of shape (H, W, 3) in RGB order.
"""

from __future__ import annotations

from typing import Dict, Tuple, Optional

import pygame

RGB = Tuple[int, int, int]


def _render_multiline_centered(
    surface: pygame.Surface,
    text: str,
    font: pygame.font.Font,
    color: RGB,
    max_width: int,
    center_y: int,
    line_spacing: int = 4,
) -> None:
    """Render multi-line text centered horizontally around center_y."""
    if not text or font is None:
        return

    words = text.split()
    if not words:
        return

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

    total_h = 0
    line_surfs = []
    for line in lines:
        surf = font.render(line, True, color)
        rect = surf.get_rect()
        line_surfs.append((surf, rect))
        total_h += rect.height + line_spacing
    total_h -= line_spacing  # remove last extra spacing

    width = surface.get_width()
    start_y = int(center_y - total_h / 2)

    y = start_y
    for surf, rect in line_surfs:
        rect.centerx = width // 2
        rect.y = y
        surface.blit(surf, rect)
        y = rect.bottom + line_spacing


def _blit_camera_frame(
    surface: pygame.Surface,
    frame_rgb: Optional[object],
    screen_size: Tuple[int, int],
) -> None:
    """
    Draw camera frame (H, W, 3 RGB uint8 NumPy array) centered & scaled.

    If frame_rgb is None, this function does nothing; caller should draw
    placeholders/overlays.
    """
    if frame_rgb is None:
        return

    try:
        h, w, _ = frame_rgb.shape  # type: ignore[attr-defined]
    except Exception:
        return

    screen_w, screen_h = screen_size

    # Compute scale factor to fit inside screen while preserving aspect ratio.
    scale = min(screen_w / float(w), screen_h / float(h))
    new_w = int(w * scale)
    new_h = int(h * scale)

    frame_bytes = frame_rgb.tobytes()  # type: ignore[attr-defined]
    frame_surf = pygame.image.frombuffer(frame_bytes, (w, h), "RGB")
    frame_surf = pygame.transform.smoothscale(frame_surf, (new_w, new_h))

    # Center on screen
    x = (screen_w - new_w) // 2
    y = (screen_h - new_h) // 2
    surface.blit(frame_surf, (x, y))


def draw_navigation_view(
    surface: pygame.Surface,
    status_text: str,
    subtitle_text: str,
    mouth_level: float,
    fonts: Dict[str, pygame.font.Font],
    colors: Dict[str, RGB],
    screen_size: Tuple[int, int],
    camera_ready: bool,
    camera_frame: Optional[object],
) -> None:
    """
    Draw the NAVIGATE view: live camera + overlays.

    Parameters
    ----------
    surface:
        Pygame surface to draw onto (already cleared by caller).
    status_text:
        High-level guidance text ("Guiding to Info Desk, please follow me.").
    subtitle_text:
        TTS text.
    mouth_level:
        0.0–1.0, current talking intensity.
    fonts:
        Dict with keys "main", "status", "subtitle".
    colors:
        Dict with keys "bg", "text_main", "text_status", "text_subtitle".
    screen_size:
        (width, height) of the display.
    camera_ready:
        True once at least one frame has been received.
    camera_frame:
        Latest RGB frame as NumPy array (H, W, 3), or None if not available.
    """
    width, height = screen_size

    font_main: pygame.font.Font | None = fonts.get("main")
    font_status: pygame.font.Font | None = fonts.get("status", font_main)
    font_subtitle: pygame.font.Font | None = fonts.get("subtitle", font_main)

    color_bg: RGB = colors.get("bg", (0, 0, 0))
    color_text_main: RGB = colors.get("text_main", (255, 255, 255))
    color_text_status: RGB = colors.get("text_status", (240, 240, 240))
    color_text_subtitle: RGB = colors.get("text_subtitle", (210, 210, 210))

    surface.fill(color_bg)

    # ------------------------------------------------------------------ #
    # 1) Camera frame or placeholder
    # ------------------------------------------------------------------ #
    if camera_ready and camera_frame is not None:
        _blit_camera_frame(surface, camera_frame, screen_size)
    else:
        # Simple placeholder if camera is not ready
        placeholder_rect = pygame.Rect(
            int(width * 0.1),
            int(height * 0.15),
            int(width * 0.8),
            int(height * 0.6),
        )
        pygame.draw.rect(surface, (15, 30, 60), placeholder_rect)
        pygame.draw.rect(surface, (200, 200, 200), placeholder_rect, width=2)

        msg = "Waiting for camera..." if not camera_ready else "No camera frame"
        if font_status is not None:
            text_surf = font_status.render(msg, True, color_text_status)
            text_rect = text_surf.get_rect(center=placeholder_rect.center)
            surface.blit(text_surf, text_rect)

    # ------------------------------------------------------------------ #
    # 2) Top status bar (destination / guidance)
    # ------------------------------------------------------------------ #
    bar_h = int(height * 0.16)
    status_bar = pygame.Surface((width, bar_h))
    status_bar.set_alpha(170)
    status_bar.fill((0, 0, 0))
    surface.blit(status_bar, (0, 0))

    _render_multiline_centered(
        surface=surface,
        text=status_text or "",
        font=font_status or font_main,
        color=color_text_status,
        max_width=int(width * 0.94),
        center_y=bar_h // 2,
        line_spacing=3,
    )

    # ------------------------------------------------------------------ #
    # 3) Bottom subtitle bar (speech + "talk level")
    # ------------------------------------------------------------------ #
    bottom_bar_h = int(height * 0.18)
    bottom_bar = pygame.Surface((width, bottom_bar_h))
    bottom_bar.set_alpha(185)
    bottom_bar.fill((0, 0, 0))
    surface.blit(bottom_bar, (0, height - bottom_bar_h))

    _render_multiline_centered(
        surface=surface,
        text=subtitle_text or "",
        font=font_subtitle or font_status or font_main,
        color=color_text_subtitle,
        max_width=int(width * 0.94),
        center_y=height - bottom_bar_h // 2,
        line_spacing=2,
    )

    # Simple "talk meter" on bottom-right based on mouth_level
    m = max(0.0, min(1.0, float(mouth_level)))
    meter_w = int(width * 0.18)
    meter_h = int(bottom_bar_h * 0.22)
    meter_x = width - meter_w - int(width * 0.03)
    meter_y = height - meter_h - int(bottom_bar_h * 0.25)

    # Background
    pygame.draw.rect(surface, (60, 60, 60), (meter_x, meter_y, meter_w, meter_h), border_radius=6)
    # Fill proportional to mouth_level
    fill_w = int(meter_w * m)
    if fill_w > 0:
        pygame.draw.rect(
            surface,
            (0, 220, 160),
            (meter_x, meter_y, fill_w, meter_h),
            border_radius=6,
        )

    # Optional label "voice"
    if font_subtitle is not None:
        label = font_subtitle.render("voice", True, color_text_subtitle)
        label_rect = label.get_rect()
        label_rect.midbottom = (meter_x + meter_w // 2, meter_y - 2)
        surface.blit(label, label_rect)
