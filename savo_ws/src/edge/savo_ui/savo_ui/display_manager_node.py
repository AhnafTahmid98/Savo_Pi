#!/usr/bin/env python3
"""
Robot Savo — UI display manager node (TEXT-OFF build)

This build:
- Keeps face + mouth + camera + modes.
- Subscribes to /savo_ui/status_text and /savo_speech/tts_text
  but NEVER draws any text on the screen.

If you still see any text, then a different file is being used.
"""

from __future__ import annotations

import os
import sys
from typing import Dict, List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import Image

import pygame
import numpy as np

# Import helper modules
try:
    from .face_view import draw_face_view
    from .nav_cam_view import draw_navigation_view
except ImportError:
    draw_face_view = None
    draw_navigation_view = None


RGB = Tuple[int, int, int]


class SavoUIDisplay(Node):
    """Main display manager node for Robot Savo UI (text drawing OFF)."""

    def __init__(self) -> None:
        super().__init__("savo_ui_display")

        self.get_logger().info(
            "Initializing SavoUIDisplay node — TEXT_DRAWING_DISABLED_VERSION=1"
        )

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self._load_parameters()

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self._mode: str = "INTERACT"

        # We still store text, but WILL NOT draw it.
        self._status_text: str = ""
        self._subtitle_text: str = ""

        # Face / mouth state
        self._face_state: str = "idle"
        self._mouth_level: float = 0.0
        self._mouth_open: bool = False

        # Camera state
        self._have_camera: bool = False
        self._camera_ready: bool = False
        self._last_camera_frame: Optional[np.ndarray] = None
        self._warned_camera_encoding: bool = False

        # ------------------------------------------------------------------
        # Subscriptions
        # ------------------------------------------------------------------
        self.create_subscription(String, "/savo_ui/mode", self._on_mode, 10)
        self.create_subscription(String, "/savo_ui/status_text", self._on_status_text, 10)
        self.create_subscription(String, "/savo_ui/face_state", self._on_face_state, 10)

        self.create_subscription(Bool, "/savo_speech/mouth_open", self._on_mouth_open, 10)
        self.create_subscription(
            Float32, "/savo_speech/mouth_level", self._on_mouth_level, 10
        )
        self.create_subscription(
            String, "/savo_speech/tts_text", self._on_subtitle_text, 10
        )

        # Camera (optional)
        if self.camera_topic:
            qos_cam = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.create_subscription(
                Image, self.camera_topic, self._on_camera_image, qos_cam
            )
            self._have_camera = True
            self.get_logger().info(f"Subscribed to camera topic: {self.camera_topic}")
        else:
            self.get_logger().warn(
                "No navigation.camera_topic configured; "
                "NAVIGATE mode will show a placeholder camera view."
            )

        # ------------------------------------------------------------------
        # Pygame / display init
        # ------------------------------------------------------------------
        self._init_pygame()

        # ------------------------------------------------------------------
        # Render loop timer
        # ------------------------------------------------------------------
        period = 1.0 / float(self.screen_fps)
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"SavoUIDisplay started at {self.screen_width}x{self.screen_height} "
            f"@ {self.screen_fps} FPS (mode={self._mode}) — TEXT OFF"
        )

    # ======================================================================
    # Parameter loading
    # ======================================================================

    def _load_parameters(self) -> None:
        """Load UI parameters from YAML or use fallback defaults."""

        # Screen params
        self.declare_parameter("screen.width", 1024)
        self.declare_parameter("screen.height", 600)
        self.declare_parameter("screen.fps", 30)
        self.declare_parameter("screen.fullscreen", True)

        self.screen_width = int(
            self.get_parameter("screen.width").get_parameter_value().integer_value
        )
        self.screen_height = int(
            self.get_parameter("screen.height").get_parameter_value().integer_value
        )
        self.screen_fps = int(
            self.get_parameter("screen.fps").get_parameter_value().integer_value
        )
        self.screen_fullscreen = bool(
            self.get_parameter("screen.fullscreen").get_parameter_value().bool_value
        )

        # Camera topic
        self.declare_parameter("navigation.camera_topic", "/camera/image_rect")
        self.camera_topic = (
            self.get_parameter("navigation.camera_topic")
            .get_parameter_value()
            .string_value
        )

        # Background colors per mode
        self.colors_bg: Dict[str, RGB] = {
            "INTERACT": self._get_color_param(
                "colors.background.INTERACT", [10, 25, 50]
            ),
            "NAVIGATE": self._get_color_param(
                "colors.background.NAVIGATE", [5, 35, 70]
            ),
            "MAP": self._get_color_param("colors.background.MAP", [4, 50, 35]),
        }

        # Text colors (still required by face_view/nav_cam_view)
        self.color_text_main = self._get_color_param(
            "colors.text_main", [255, 255, 255]
        )
        self.color_text_status = self._get_color_param(
            "colors.text_status", [240, 240, 240]
        )
        self.color_text_subtitle = self._get_color_param(
            "colors.text_subtitle", [210, 210, 210]
        )

        # Text sizes
        self.declare_parameter("text.main.font_size", 32)
        self.declare_parameter("text.status.font_size", 28)
        self.declare_parameter("text.subtitle.font_size", 22)

        self.font_size_main = int(
            self.get_parameter("text.main.font_size")
            .get_parameter_value()
            .integer_value
        )
        self.font_size_status = int(
            self.get_parameter("text.status.font_size")
            .get_parameter_value()
            .integer_value
        )
        self.font_size_subtitle = int(
            self.get_parameter("text.subtitle.font_size")
            .get_parameter_value()
            .integer_value
        )

        # Debug
        self.declare_parameter("debug.show_fps", False)
        self.show_fps = bool(
            self.get_parameter("debug.show_fps").get_parameter_value().bool_value
        )

    def _get_color_param(self, name: str, default_rgb: List[int]) -> RGB:
        self.declare_parameter(name, default_rgb)

        try:
            param = self.get_parameter(name)
        except Exception:
            return int(default_rgb[0]), int(default_rgb[1]), int(default_rgb[2])

        value = param.value

        if isinstance(value, (list, tuple)) and len(value) >= 3:
            r, g, b = value[0], value[1], value[2]
            try:
                return int(r), int(g), int(b)
            except (ValueError, TypeError):
                self.get_logger().warn(
                    f"Parameter '{name}' has non-numeric values {value}, "
                    f"using default {default_rgb}"
                )
                return int(default_rgb[0]), int(default_rgb[1]), int(default_rgb[2])

        self.get_logger().warn(
            f"Parameter '{name}' not a valid RGB list, using default {default_rgb}"
        )
        return int(default_rgb[0]), int(default_rgb[1]), int(default_rgb[2])

    # ======================================================================
    # Subscriptions
    # ======================================================================

    def _on_mode(self, msg: String) -> None:
        mode = (msg.data or "").strip().upper()
        if mode not in ("INTERACT", "NAVIGATE", "MAP"):
            self.get_logger().warn(
                f"Unknown UI mode '{msg.data}', keeping previous mode '{self._mode}'"
            )
            return
        if mode != self._mode:
            self.get_logger().info(f"UI mode changed: {self._mode} -> {mode}")
        self._mode = mode

    def _on_status_text(self, msg: String) -> None:
        # We store it but DO NOT draw.
        self._status_text = msg.data or ""

    def _on_face_state(self, msg: String) -> None:
        state = (msg.data or "").strip().lower()
        if not state:
            return
        if state not in ("idle", "listening", "thinking", "speaking"):
            self.get_logger().debug(
                f"Unknown face_state '{state}', keeping '{self._face_state}'"
            )
            return
        if state != self._face_state:
            self.get_logger().debug(f"face_state changed: {self._face_state} -> {state}")
        self._face_state = state

    def _on_mouth_open(self, msg: Bool) -> None:
        self._mouth_open = bool(msg.data)
        self._mouth_level = 1.0 if self._mouth_open else 0.0

    def _on_mouth_level(self, msg: Float32) -> None:
        level = float(msg.data)
        self._mouth_level = max(0.0, min(1.0, level))

    def _on_subtitle_text(self, msg: String) -> None:
        # We store it but DO NOT draw.
        self._subtitle_text = msg.data or ""

    def _on_camera_image(self, msg: Image) -> None:
        if not self._camera_ready:
            self._camera_ready = True
            self.get_logger().info("Camera stream is active (first frame received).")

        try:
            enc = (msg.encoding or "").lower()
        except Exception:
            enc = ""

        try:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            expected = msg.height * msg.width * 3
            if buf.size != expected:
                if not self._warned_camera_encoding:
                    self._warned_camera_encoding = True
                    self.get_logger().warn(
                        f"Unexpected image size: got {buf.size}, "
                        f"expected {expected} (height={msg.height}, width={msg.width})"
                    )
                return

            frame = buf.reshape((msg.height, msg.width, 3))

            if enc in ("rgb8", "rgb_8", "rgb8c"):
                frame_rgb = frame
            elif enc in ("bgr8", "bgr_8", "bgr8c"):
                frame_rgb = frame[:, :, ::-1]
            else:
                if not self._warned_camera_encoding:
                    self._warned_camera_encoding = True
                    self.get_logger().warn(
                        f"Unsupported camera encoding '{msg.encoding}', "
                        "expected rgb8 or bgr8; NAVIGATE view will show placeholder."
                    )
                return

            self._last_camera_frame = frame_rgb

        except Exception as exc:
            if not self._warned_camera_encoding:
                self._warned_camera_encoding = True
                self.get_logger().warn(
                    f"Error converting camera image: {exc}. "
                    "NAVIGATE view will show placeholder."
                )

    # ======================================================================
    # Pygame init + render loop
    # ======================================================================

    def _init_pygame(self) -> None:
        os.environ.setdefault("SDL_VIDEO_WAYLAND_ALLOW_LIBDECOR", "0")
        os.environ.setdefault("SDL_RENDER_VSYNC", "1")

        pygame.init()
        pygame.font.init()

        drivers: List[str] = []
        env_drv = os.environ.get("SDL_VIDEODRIVER")
        if env_drv:
            drivers.append(env_drv)
        for drv in ("kmsdrm", "wayland", "x11"):
            if drv not in drivers:
                drivers.append(drv)

        tried: List[str] = []
        last_exc: Optional[BaseException] = None
        chosen: Optional[str] = None
        self._screen = None

        for drv in drivers:
            if not drv or drv in tried:
                continue
            tried.append(drv)
            os.environ["SDL_VIDEODRIVER"] = drv

            try:
                flags = 0
                if self.screen_fullscreen:
                    flags |= pygame.FULLSCREEN | pygame.HWSURFACE | pygame.DOUBLEBUF

                screen = pygame.display.set_mode(
                    (self.screen_width, self.screen_height),
                    flags,
                )
                self._screen = screen
                chosen = drv
                self.get_logger().info(
                    f"SDL driver '{drv}' selected, screen size {screen.get_size()}"
                )
                break
            except Exception as exc:
                last_exc = exc
                self.get_logger().warn(
                    f"SDL driver '{drv}' failed in set_mode: {exc}. Trying next..."
                )
                try:
                    pygame.display.quit()
                except Exception:
                    pass

        if self._screen is None or chosen is None:
            raise RuntimeError(
                "Unable to initialize any SDL video driver "
                f"(tried: {', '.join(tried)}). Last error: {last_exc}"
            )

        w, h = self._screen.get_size()
        pygame.display.set_caption(f"Robot Savo UI [{chosen}] {w}x{h}")
        pygame.mouse.set_visible(False)

        self._pg_clock = pygame.time.Clock()

        self._font_main = pygame.font.SysFont("DejaVu Sans", self.font_size_main)
        self._font_status = pygame.font.SysFont("DejaVu Sans", self.font_size_status)
        self._font_subtitle = pygame.font.SysFont(
            "DejaVu Sans", self.font_size_subtitle
        )

    def _on_timer(self) -> None:
        _dt_ms = self._pg_clock.tick(self.screen_fps)  # noqa: F841
        fps = self._pg_clock.get_fps() if self.show_fps else None

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.get_logger().info("Pygame QUIT event received, shutting down UI.")
                rclpy.shutdown()
                return
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                self.get_logger().info("ESC pressed, shutting down UI.")
                rclpy.shutdown()
                return

        if self._mode == "INTERACT":
            self._draw_interact_mode()
        elif self._mode == "NAVIGATE":
            self._draw_navigate_mode()
        elif self._mode == "MAP":
            self._draw_map_mode()
        else:
            self._draw_fallback()

        if fps is not None:
            fps_text = self._font_subtitle.render(
                f"{fps:5.1f} FPS", True, (255, 255, 0)
            )
            self._screen.blit(fps_text, (10, 10))

        pygame.display.flip()

    # ======================================================================
    # Per-mode drawing
    # ======================================================================

    def _clear_background(self, mode: str) -> None:
        color = self.colors_bg.get(mode, (0, 0, 0))
        self._screen.fill(color)

    def _draw_interact_mode(self) -> None:
        self._clear_background("INTERACT")

        # HARD OFF: no text
        status_text = ""
        subtitle_text = ""

        if draw_face_view is not None:
            draw_face_view(
                surface=self._screen,
                mouth_level=self._mouth_level,
                status_text=status_text,
                subtitle_text=subtitle_text,
                face_state=self._face_state,
                fonts={
                    "main": self._font_main,
                    "status": self._font_status,
                    "subtitle": self._font_subtitle,
                },
                colors={
                    "bg": self.colors_bg["INTERACT"],
                    "text_main": self.color_text_main,
                    "text_status": self.color_text_status,
                    "text_subtitle": self.color_text_subtitle,
                },
                screen_size=(self.screen_width, self.screen_height),
            )

    def _draw_navigate_mode(self) -> None:
        self._clear_background("NAVIGATE")

        status_text = ""
        subtitle_text = ""

        if draw_navigation_view is not None and self._have_camera:
            draw_navigation_view(
                surface=self._screen,
                status_text=status_text,
                subtitle_text=subtitle_text,
                mouth_level=self._mouth_level,
                fonts={
                    "main": self._font_main,
                    "status": self._font_status,
                    "subtitle": self._font_subtitle,
                },
                colors={
                    "bg": self.colors_bg["NAVIGATE"],
                    "text_main": self.color_text_main,
                    "text_status": self.color_text_status,
                    "text_subtitle": self.color_text_subtitle,
                },
                screen_size=(self.screen_width, self.screen_height),
                camera_ready=self._camera_ready,
                camera_frame=self._last_camera_frame,
            )

    def _draw_map_mode(self) -> None:
        self._clear_background("MAP")

        status_text = ""
        subtitle_text = ""

        if draw_face_view is not None:
            draw_face_view(
                surface=self._screen,
                mouth_level=0.0,
                status_text=status_text,
                subtitle_text=subtitle_text,
                face_state="idle",
                fonts={
                    "main": self._font_main,
                    "status": self._font_status,
                    "subtitle": self._font_subtitle,
                },
                colors={
                    "bg": self.colors_bg["MAP"],
                    "text_main": self.color_text_main,
                    "text_status": self.color_text_status,
                    "text_subtitle": self.color_text_subtitle,
                },
                screen_size=(self.screen_width, self.screen_height),
            )

    def _draw_fallback(self) -> None:
        self._screen.fill((0, 0, 0))
        # No text even here.


# ==========================================================================#
# main()
# ==========================================================================#

def main(argv: Optional[list] = None) -> None:
    rclpy.init(args=argv)
    node = SavoUIDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down SavoUIDisplay.")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        pygame.quit()


if __name__ == "__main__":
    main(sys.argv)
