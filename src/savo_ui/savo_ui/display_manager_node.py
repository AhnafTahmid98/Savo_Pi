#!/usr/bin/env python3
"""
Robot Savo — UI display manager node (v2)

This node drives the 7" DFRobot DSI display. It is responsible for:

High-level UI signals:
    - /savo_ui/mode          (INTERACT / NAVIGATE / MAP)
    - /savo_ui/status_text   (one- or two-line human-readable status)
    - /savo_ui/face_state    ("idle" / "listening" / "thinking" / "speaking")

Speech UI signals:
    - /savo_speech/mouth_open   (Bool: True while mouth should be open)
    - /savo_speech/mouth_level  (Float32 0.0–1.0; optional legacy support)
    - /savo_speech/tts_text     (subtitle of what Robot Savo is saying)

Rendering (via helper modules):
    - INTERACT:
        - Big friendly face (eyes + mouth)
        - Status + subtitle overlaid
        - Expression driven by /savo_ui/face_state
    - NAVIGATE:
        - Camera view as background (letterboxed)
        - Small face overlay + status + subtitle
    - MAP:
        - Mapping status screen, optionally reusing face layout

Camera handling:
    - Subscribes to sensor_msgs/Image on `navigation.camera_topic`
      (default: /camera/image_rect).
    - Supports `rgb8` and `bgr8` encodings.
    - Converts frames to NumPy RGB (H, W, 3) and passes them to nav_cam_view.

Pygame:
    - Fullscreen rendering on DSI display with driver fallback
      (kmsdrm → wayland → x11).
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
    # For early development, we allow these to be missing and use stubs.
    draw_face_view = None
    draw_navigation_view = None


RGB = Tuple[int, int, int]


class SavoUIDisplay(Node):
    """Main display manager node for Robot Savo UI."""

    def __init__(self) -> None:
        super().__init__("savo_ui_display")

        self.get_logger().info("Initializing SavoUIDisplay node")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self._load_parameters()

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self._mode: str = "INTERACT"
        self._status_text: str = ""
        self._subtitle_text: str = ""

        # Face / mouth state
        # - _face_state is a high-level semantic state:
        #       "idle" / "listening" / "thinking" / "speaking"
        # - _mouth_level is a numeric [0.0, 1.0] used by the renderer.
        #   It is derived from:
        #       /savo_speech/mouth_open (Bool, preferred) OR
        #       /savo_speech/mouth_level (Float32, legacy)
        self._face_state: str = "idle"
        self._mouth_level: float = 0.0
        self._mouth_open: bool = False  # last Bool from /savo_speech/mouth_open

        # Camera state
        self._have_camera: bool = False
        self._camera_ready: bool = False
        self._last_camera_frame: Optional[np.ndarray] = None  # (H, W, 3) RGB
        self._warned_camera_encoding: bool = False

        # ------------------------------------------------------------------
        # Subscriptions
        # ------------------------------------------------------------------
        # UI mode and status text
        self.create_subscription(String, "/savo_ui/mode", self._on_mode, 10)
        self.create_subscription(
            String,
            "/savo_ui/status_text",
            self._on_status_text,
            10,
        )

        # Face state from remote_speech_client_node
        self.create_subscription(
            String,
            "/savo_ui/face_state",
            self._on_face_state,
            10,
        )

        # Mouth animation:
        #  - Preferred: Bool mouth_open
        #  - Legacy: Float32 mouth_level (will still work if present)
        self.create_subscription(
            Bool,
            "/savo_speech/mouth_open",
            self._on_mouth_open,
            10,
        )
        self.create_subscription(
            Float32,
            "/savo_speech/mouth_level",
            self._on_mouth_level,
            10,
        )

        # Subtitle of what Robot Savo is saying
        self.create_subscription(
            String,
            "/savo_speech/tts_text",
            self._on_subtitle_text,
            10,
        )

        # Camera is optional; NAVIGATE mode will show a placeholder if no topic.
        if self.camera_topic:
            qos_cam = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.create_subscription(
                Image,
                self.camera_topic,
                self._on_camera_image,
                qos_cam,
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
            f"@ {self.screen_fps} FPS (mode={self._mode})"
        )

    # ======================================================================
    # Parameter loading
    # ======================================================================

    def _load_parameters(self) -> None:
        """Load UI parameters from YAML or use fallback defaults."""

        # Screen params (override in YAML to 800x480 on the Pi)
        self.declare_parameter("screen.width", 1024)
        self.declare_parameter("screen.height", 600)
        self.declare_parameter("screen.fps", 30)
        self.declare_parameter("screen.fullscreen", True)

        self.screen_width: int = (
            self.get_parameter("screen.width").get_parameter_value().integer_value
        )
        self.screen_height: int = (
            self.get_parameter("screen.height").get_parameter_value().integer_value
        )
        self.screen_fps: int = (
            self.get_parameter("screen.fps").get_parameter_value().integer_value
        )
        self.screen_fullscreen: bool = (
            self.get_parameter("screen.fullscreen").get_parameter_value().bool_value
        )

        # Camera topic (NAVIGATE mode)
        self.declare_parameter("navigation.camera_topic", "/camera/image_rect")
        self.camera_topic: str = (
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
            "MAP": self._get_color_param(
                "colors.background.MAP", [4, 50, 35]
            ),
        }

        # Text colors
        self.color_text_main: RGB = self._get_color_param(
            "colors.text_main", [255, 255, 255]
        )
        self.color_text_status: RGB = self._get_color_param(
            "colors.text_status", [240, 240, 240]
        )
        self.color_text_subtitle: RGB = self._get_color_param(
            "colors.text_subtitle", [210, 210, 210]
        )

        # Text sizes
        self.declare_parameter("text.main.font_size", 32)
        self.declare_parameter("text.status.font_size", 28)
        self.declare_parameter("text.subtitle.font_size", 22)

        self.font_size_main: int = (
            self.get_parameter("text.main.font_size")
            .get_parameter_value()
            .integer_value
        )
        self.font_size_status: int = (
            self.get_parameter("text.status.font_size")
            .get_parameter_value()
            .integer_value
        )
        self.font_size_subtitle: int = (
            self.get_parameter("text.subtitle.font_size")
            .get_parameter_value()
            .integer_value
        )

        # Debug
        self.declare_parameter("debug.show_fps", False)
        self.show_fps: bool = (
            self.get_parameter("debug.show_fps").get_parameter_value().bool_value
        )

    def _get_color_param(self, name: str, default_rgb: List[int]) -> RGB:
        """
        Helper to read an RGB color parameter.

        Accepts:
        - list/tuple of 3 numeric values (r, g, b)
        - list/tuple of 4 values (r, g, b, a) → a is ignored

        If missing or malformed, falls back to default_rgb.
        """
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
        self._status_text = msg.data or ""

    def _on_face_state(self, msg: String) -> None:
        """
        Track the high-level face state for expressions:
          "idle" / "listening" / "thinking" / "speaking"
        """
        state = (msg.data or "").strip().lower()
        if not state:
            return
        # Accept only known states to avoid weird typos
        if state not in ("idle", "listening", "thinking", "speaking"):
            # Do not spam logs; just keep last valid state
            self.get_logger().debug(
                f"Unknown face_state '{state}', keeping '{self._face_state}'"
            )
            return
        if state != self._face_state:
            self.get_logger().debug(f"face_state changed: {self._face_state} -> {state}")
        self._face_state = state

    def _on_mouth_open(self, msg: Bool) -> None:
        """
        Preferred mouth animation channel:
          - True  → mouth fully open
          - False → mouth closed

        We convert it to a [0.0, 1.0] mouth_level so face_view/nav_cam_view
        can stay numeric and compatible with older code.
        """
        self._mouth_open = bool(msg.data)
        self._mouth_level = 1.0 if self._mouth_open else 0.0

    def _on_mouth_level(self, msg: Float32) -> None:
        """
        Legacy Float32 channel (0.0–1.0). If present, we still honor it.

        NOTE:
          If both mouth_open and mouth_level are published, the *last*
          received message wins. This is fine in practice because we
          normally only use mouth_open now.
        """
        level = float(msg.data)
        # Clamp to [0.0, 1.0]
        self._mouth_level = max(0.0, min(1.0, level))

    def _on_subtitle_text(self, msg: String) -> None:
        self._subtitle_text = msg.data or ""

    def _on_camera_image(self, msg: Image) -> None:
        """
        Camera callback: convert ROS Image -> RGB NumPy frame for nav_cam_view.

        - Supports rgb8 and bgr8 encodings.
        - On first frame, marks camera as ready and logs once.
        - Stores only the latest frame; rendering happens in the pygame timer.
        """
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
                # Convert BGR -> RGB
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
    # Pygame initialization and render loop
    # ======================================================================

    def _init_pygame(self) -> None:
        """Initialize pygame display with robust driver fallback."""

        # Helpful defaults; harmless if backend doesn't support them
        os.environ.setdefault("SDL_VIDEO_WAYLAND_ALLOW_LIBDECOR", "0")
        os.environ.setdefault("SDL_RENDER_VSYNC", "1")

        pygame.init()
        pygame.font.init()

        # Build driver preference list.
        # We *prefer* kmsdrm on bare Pi console, then wayland/x11 if available.
        drivers: List[str] = []
        env_drv = os.environ.get("SDL_VIDEODRIVER")
        if env_drv:
            drivers.append(env_drv)
        # Add our preferred order, skipping duplicates
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

                # This actually opens the display; if it fails we try next driver.
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

        # Separate pygame clock for FPS limiting / measurement
        self._pg_clock = pygame.time.Clock()

        # Load fonts (if these fail, pygame will substitute defaults)
        self._font_main = pygame.font.SysFont("DejaVu Sans", self.font_size_main)
        self._font_status = pygame.font.SysFont("DejaVu Sans", self.font_size_status)
        self._font_subtitle = pygame.font.SysFont(
            "DejaVu Sans", self.font_size_subtitle
        )

    def _on_timer(self) -> None:
        """Render loop callback (called at ~FPS rate)."""

        # FPS timing (limit + compute FPS for overlay)
        _dt_ms = self._pg_clock.tick(self.screen_fps)  # noqa: F841
        fps = self._pg_clock.get_fps() if self.show_fps else None

        # Handle pygame events (quit, ESC, etc.)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.get_logger().info("Pygame QUIT event received, shutting down.")
                rclpy.shutdown()
                return
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                self.get_logger().info("ESC pressed, shutting down UI.")
                rclpy.shutdown()
                return

        # Draw current mode
        if self._mode == "INTERACT":
            self._draw_interact_mode()
        elif self._mode == "NAVIGATE":
            self._draw_navigate_mode()
        elif self._mode == "MAP":
            self._draw_map_mode()
        else:
            self._draw_fallback()

        # Optional FPS overlay (top-left)
        if fps is not None:
            fps_text = self._font_subtitle.render(
                f"{fps:5.1f} FPS", True, (255, 255, 0)
            )
            self._screen.blit(fps_text, (10, 10))

        # Flip buffer
        pygame.display.flip()

    # ======================================================================
    # Per-mode drawing
    # ======================================================================

    def _clear_background(self, mode: str) -> None:
        color = self.colors_bg.get(mode, (0, 0, 0))
        self._screen.fill(color)

    def _draw_interact_mode(self) -> None:
        """Draw the INTERACT face view."""
        self._clear_background("INTERACT")

        if draw_face_view is not None:
            # Prefer new signature with face_state; fall back to old one if needed.
            try:
                draw_face_view(
                    surface=self._screen,
                    mouth_level=self._mouth_level,
                    status_text=self._status_text,
                    subtitle_text=self._subtitle_text,
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
            except TypeError:
                # Older face_view without face_state parameter.
                draw_face_view(
                    surface=self._screen,
                    mouth_level=self._mouth_level,
                    status_text=self._status_text,
                    subtitle_text=self._subtitle_text,
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
        else:
            text = self._font_status.render(
                "INTERACT (face_view not implemented)", True, (255, 255, 255)
            )
            self._screen.blit(text, (40, self.screen_height // 2))

    def _draw_navigate_mode(self) -> None:
        """Draw the NAVIGATE camera view + overlay."""
        self._clear_background("NAVIGATE")

        if draw_navigation_view is not None and self._have_camera:
            # Prefer new signature with face_state; fall back to old one if needed.
            try:
                draw_navigation_view(
                    surface=self._screen,
                    status_text=self._status_text,
                    subtitle_text=self._subtitle_text,
                    mouth_level=self._mouth_level,
                    face_state=self._face_state,
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
            except TypeError:
                # Older nav_cam_view without face_state parameter.
                draw_navigation_view(
                    surface=self._screen,
                    status_text=self._status_text,
                    subtitle_text=self._subtitle_text,
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
        else:
            msg = "NAVIGATE mode (camera or nav_cam_view not implemented)"
            text = self._font_status.render(msg, True, (255, 255, 255))
            self._screen.blit(text, (40, self.screen_height // 2))

    def _draw_map_mode(self) -> None:
        """Draw the MAP status view (optionally reusing face layout)."""
        self._clear_background("MAP")

        if draw_face_view is not None:
            # During mapping we typically show a calm face; mouth_level = 0.0.
            try:
                draw_face_view(
                    surface=self._screen,
                    mouth_level=0.0,
                    status_text=(
                        self._status_text
                        or "Mapping in progress, please keep distance."
                    ),
                    subtitle_text=self._subtitle_text,
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
            except TypeError:
                # Older face_view without face_state parameter.
                draw_face_view(
                    surface=self._screen,
                    mouth_level=0.0,
                    status_text=(
                        self._status_text
                        or "Mapping in progress, please keep distance."
                    ),
                    subtitle_text=self._subtitle_text,
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
        else:
            msg = "MAP mode (face_view not implemented)"
            text = self._font_status.render(msg, True, (255, 255, 255))
            self._screen.blit(text, (40, self.screen_height // 2))

    def _draw_fallback(self) -> None:
        """Fallback drawing if mode is invalid."""
        self._screen.fill((0, 0, 0))
        msg = f"Unknown mode: {self._mode}"
        text = self._font_status.render(msg, True, (255, 255, 255))
        self._screen.blit(text, (40, self.screen_height // 2))


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
        rclpy.shutdown()
        pygame.quit()


if __name__ == "__main__":
    main(sys.argv)
