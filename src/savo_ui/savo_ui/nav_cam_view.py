#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Navigation camera view renderer (NAVIGATE mode)
-------------------------------------------------------------

This module draws the NAVIGATE UI:

- Large camera area (centered panel for live video).
- Small Robot Savo face overlay (same style as face_view) in the top-left
  of the camera area. The mouth opens with mouth_level.
- Status text overlay (goal / guidance).
- Subtitle text (what Robot Savo is saying).
- Simple "mouth bar" that reacts to mouth_level in a corner.

Camera input:

- numpy.ndarray with shape (H, W, 3), dtype uint8, RGB
- sensor_msgs/Image (encoding rgb8/bgr8)

This file is pure pygame + numpy; no ROS node here. It is called from
display_manager_node.SavoUIDisplay via draw_navigation_view().
"""

from __future__ import annotations

from typing import Dict, Tuple, Any

import numpy as np
import pygame
from sensor_msgs.msg import Image as RosImage  # type: ignore[import-untyped]

RGB = Tuple[int, int, int]


# ======================================================================
# Text helpers
# ======================================================================


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

    Returns
    -------
    int
        The y-coordinate after the last rendered line.
    """
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


# ======================================================================
# Camera blit helper
# ======================================================================


def _blit_camera_frame(
    surface: pygame.Surface,
    cam_rect: pygame.Rect,
    camera_frame: Any,
) -> None:
    """
    Draw the camera frame inside cam_rect with letterboxing.

    Supports:
    - numpy.ndarray: shape (H, W, 3), dtype uint8, RGB
    - sensor_msgs/Image: encoding 'rgb8' or 'bgr8'
    """
    if camera_frame is None:
        return

    img_surface: pygame.Surface

    # -------------------------------------------------------------
    # Case 1: numpy array (H, W, 3)
    # -------------------------------------------------------------
    if isinstance(camera_frame, np.ndarray):
        frame = camera_frame

        if frame.ndim != 3 or frame.shape[2] < 3:
            return

        # Ensure uint8
        if frame.dtype != np.uint8:
            frame = np.clip(frame, 0, 255).astype(np.uint8)

        height, width = frame.shape[:2]

        # Only use first 3 channels (RGB)
        frame = frame[:, :, :3]

        frame_bytes = frame.tobytes()
        img_surface = pygame.image.frombuffer(frame_bytes, (width, height), "RGB")

    # -------------------------------------------------------------
    # Case 2: ROS Image (rgb8 / bgr8)
    # -------------------------------------------------------------
    else:
        if not hasattr(camera_frame, "encoding"):
            return

        msg: RosImage = camera_frame  # type: ignore[assignment]

        encoding = (msg.encoding or "").lower()
        if encoding not in ("rgb8", "bgr8"):
            return

        width = msg.width
        height = msg.height
        if width == 0 or height == 0:
            return

        step = msg.step  # bytes per row
        buf = np.frombuffer(msg.data, dtype=np.uint8)

        if step < width * 3:
            return

        try:
            # step is bytes/row; step//3 is pixels/row
            frame = buf.reshape((height, step // 3, 3))
            frame = frame[:, :width, :]
        except Exception:
            return

        if encoding == "bgr8":
            frame = frame[:, :, ::-1]  # BGR → RGB

        frame_bytes = frame.tobytes()
        img_surface = pygame.image.frombuffer(frame_bytes, (width, height), "RGB")

    # -------------------------------------------------------------
    # Letterbox into cam_rect while preserving aspect ratio
    # -------------------------------------------------------------
    src_w, src_h = img_surface.get_size()
    if src_w == 0 or src_h == 0 or cam_rect.width <= 0 or cam_rect.height <= 0:
        return

    src_aspect = src_w / float(src_h)
    dst_aspect = cam_rect.width / float(cam_rect.height)

    # Same aspect → scale exactly to panel
    if abs(src_aspect - dst_aspect) < 1e-3:
        disp = pygame.transform.smoothscale(img_surface, (cam_rect.width, cam_rect.height))
        surface.blit(disp, cam_rect)
        return

    if src_aspect > dst_aspect:
        # Source wider than panel → match width
        new_w = cam_rect.width
        new_h = max(1, int(new_w / src_aspect))
        disp = pygame.transform.smoothscale(img_surface, (new_w, new_h))
        y = cam_rect.y + (cam_rect.height - new_h) // 2
        surface.blit(disp, (cam_rect.x, y))
    else:
        # Source taller → match height
        new_h = cam_rect.height
        new_w = max(1, int(new_h * src_aspect))
        disp = pygame.transform.smoothscale(img_surface, (new_w, new_h))
        x = cam_rect.x + (cam_rect.width - new_w) // 2
        surface.blit(disp, (x, cam_rect.y))


# ======================================================================
# Small face overlay
# ======================================================================


def _draw_mini_face_same_style(
    surface: pygame.Surface,
    cam_rect: pygame.Rect,
    mouth_level: float,
) -> None:
    """
    Draw a small Robot Savo face in the top-left of the camera panel,
    using the same eye + mouth style as face_view (scaled to this rect).
    """
    m = max(0.0, min(1.0, float(mouth_level)))

    # Small face area inside the camera rect
    face_w = int(cam_rect.width * 0.24)
    face_h = int(cam_rect.height * 0.35)
    margin_x = int(cam_rect.width * 0.03)
    margin_y = int(cam_rect.height * 0.05)

    face_rect = pygame.Rect(
        cam_rect.x + margin_x,
        cam_rect.y + margin_y,
        face_w,
        face_h,
    )

    # Colors cloned from face_view
    color_bg: RGB = (10, 25, 50)
    eye_white: RGB = (245, 245, 245)
    eye_border: RGB = (30, 50, 80)
    pupil_color: RGB = (0, 0, 0)
    mouth_border: RGB = (255, 200, 120)
    mouth_fill: RGB = (255, 140, 100)

    # Background + rounded border
    border_radius = max(6, min(face_w, face_h) // 6)
    border_rect = face_rect.inflate(6, 6)
    pygame.draw.rect(surface, (0, 0, 0), border_rect, border_radius=border_radius + 2)
    pygame.draw.rect(surface, color_bg, face_rect, border_radius=border_radius)

    # Local layout within mini rect
    width = face_rect.width
    height = face_rect.height
    cx = face_rect.centerx

    eye_center_y = face_rect.y + int(height * 0.40)
    mouth_center_y = face_rect.y + int(height * 0.70)

    # Eyes
    eye_spacing = int(width * 0.16)
    eye_w = int(width * 0.24)
    eye_h = int(height * 0.34)

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

    pygame.draw.ellipse(surface, eye_white, left_eye_rect)
    pygame.draw.ellipse(surface, eye_white, right_eye_rect)
    pygame.draw.ellipse(surface, eye_border, left_eye_rect, width=2)
    pygame.draw.ellipse(surface, eye_border, right_eye_rect, width=2)

    # Pupils
    pupil_w = int(eye_w * 0.32)
    pupil_h = int(eye_h * 0.35)
    pupil_offset_y = int(-0.06 * eye_h * m)

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

    highlight_r = max(1, pupil_w // 6)
    for rect in (left_pupil_rect, right_pupil_rect):
        highlight_center = (rect.centerx - highlight_r, rect.centery - highlight_r)
        pygame.draw.circle(surface, (255, 255, 255), highlight_center, highlight_r)

    # Mouth
    mouth_width = int(width * 0.60)
    mouth_height_max = int(height * 0.40)
    mouth_height_min = max(2, int(height * 0.06))

    open_height = int(
        mouth_height_min + (mouth_height_max - mouth_height_min) * m
    )

    mouth_rect = pygame.Rect(
        cx - mouth_width // 2,
        mouth_center_y - open_height // 2,
        mouth_width,
        open_height,
    )

    pygame.draw.ellipse(surface, mouth_fill, mouth_rect)
    pygame.draw.ellipse(surface, mouth_border, mouth_rect, width=3)

    # Smile arc
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
        width=2,
    )


# ======================================================================
# Main public draw function
# ======================================================================


def draw_navigation_view(
    surface: pygame.Surface,
    status_text: str,
    subtitle_text: str,
    mouth_level: float,
    fonts: Dict[str, pygame.font.Font],
    colors: Dict[str, RGB],
    screen_size: Tuple[int, int],
    camera_ready: bool,
    camera_frame: Any = None,  # numpy array or RosImage
) -> None:
    """
    Draw the NAVIGATE mode view.

    This is called every frame from display_manager_node.SavoUIDisplay.
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

    # Background
    surface.fill(color_bg)

    # ----------------------------------------------------------------------
    # Camera panel layout — centered in upper ~60% of screen
    # ----------------------------------------------------------------------
    margin_x = int(width * 0.05)

    cam_width = width - 2 * margin_x
    cam_height = int(height * 0.60)

    cam_left = margin_x
    cam_top = int(height * 0.08)
    cam_rect = pygame.Rect(cam_left, cam_top, cam_width, cam_height)
    cam_bottom = cam_rect.bottom
    cam_right = cam_rect.right

    # Panel background
    pygame.draw.rect(surface, cam_inner_color, cam_rect)

    # Frame or placeholder
    if camera_ready and camera_frame is not None:
        _blit_camera_frame(surface, cam_rect, camera_frame)
    else:
        if font_status is not None:
            if not camera_ready:
                msg = "Starting camera…"
            else:
                # camera_ready True but no frame object; very rare
                msg = "Camera active, no frame."

            cam_text = font_status.render(msg, True, color_text_main)
            cam_text_rect = cam_text.get_rect(center=cam_rect.center)
            surface.blit(cam_text, cam_text_rect)

    # Border + corner markers
    pygame.draw.rect(surface, cam_border_color, cam_rect, width=4)

    corner_len = max(8, min(cam_width, cam_height) // 10)
    thickness = 3

    # top-left
    pygame.draw.line(
        surface, cam_border_color,
        (cam_left, cam_top),
        (cam_left + corner_len, cam_top),
        thickness,
    )
    pygame.draw.line(
        surface, cam_border_color,
        (cam_left, cam_top),
        (cam_left, cam_top + corner_len),
        thickness,
    )

    # top-right
    pygame.draw.line(
        surface, cam_border_color,
        (cam_right, cam_top),
        (cam_right - corner_len, cam_top),
        thickness,
    )
    pygame.draw.line(
        surface, cam_border_color,
        (cam_right, cam_top),
        (cam_right, cam_top + corner_len),
        thickness,
    )

    # bottom-left
    pygame.draw.line(
        surface, cam_border_color,
        (cam_left, cam_bottom),
        (cam_left + corner_len, cam_bottom),
        thickness,
    )
    pygame.draw.line(
        surface, cam_border_color,
        (cam_left, cam_bottom),
        (cam_left, cam_bottom - corner_len),
        thickness,
    )

    # bottom-right
    pygame.draw.line(
        surface, cam_border_color,
        (cam_right, cam_bottom),
        (cam_right - corner_len, cam_bottom),
        thickness,
    )
    pygame.draw.line(
        surface, cam_border_color,
        (cam_right, cam_bottom),
        (cam_right, cam_bottom - corner_len),
        thickness,
    )

    # ----------------------------------------------------------------------
    # Small face overlay (same style as INTERACT face)
    # ----------------------------------------------------------------------
    _draw_mini_face_same_style(surface, cam_rect, mouth_level)

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
    # Subtitle (TTS) near bottom center
    # ----------------------------------------------------------------------
    subtitle_y = int(height * 0.78)
    max_subtitle_width = int(width * 0.88)

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
    # "Mouth bar" in bottom-right corner
    # ----------------------------------------------------------------------
    m = max(0.0, min(1.0, float(mouth_level)))

    bar_width = int(width * 0.18)
    bar_height = int(height * 0.04)
    bar_margin_x = int(width * 0.04)
    bar_margin_y = int(height * 0.05)

    bar_x = width - bar_margin_x - bar_width
    bar_y = height - bar_height - bar_margin_y

    bar_rect = pygame.Rect(bar_x, bar_y, bar_width, bar_height)
    pygame.draw.rect(surface, (220, 220, 220), bar_rect, width=2)

    inner_margin = 3
    inner_w = int((bar_width - 2 * inner_margin) * m)
    inner_h = bar_height - 2 * inner_margin

    level_color = (
        int(80 + 150 * m),
        int(120 + 80 * m),
        110,
    )
    inner_rect = pygame.Rect(
        bar_x + inner_margin,
        bar_y + inner_margin,
        inner_w,
        inner_h,
    )
    pygame.draw.rect(surface, level_color, inner_rect)
