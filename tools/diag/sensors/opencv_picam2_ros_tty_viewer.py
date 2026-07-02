#!/usr/bin/env python3

import argparse
import signal
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


STOP_REQUESTED = False


def handle_signal(signum, frame) -> None:
    global STOP_REQUESTED
    STOP_REQUESTED = True
    print("\nShutdown requested. Closing TTY camera viewer...", flush=True)


def read_text(path: str) -> str:
    return Path(path).read_text(encoding="utf-8").strip()


def read_framebuffer_info(fb_device: str) -> tuple[int, int, int]:
    fb_name = Path(fb_device).name
    base = Path("/sys/class/graphics") / fb_name

    width_text, height_text = read_text(str(base / "virtual_size")).split(",")
    bpp = int(read_text(str(base / "bits_per_pixel")))

    return int(width_text), int(height_text), bpp


def resize_with_letterbox(frame: np.ndarray, screen_w: int, screen_h: int) -> np.ndarray:
    src_h, src_w = frame.shape[:2]
    scale = min(screen_w / src_w, screen_h / src_h)

    new_w = max(1, int(src_w * scale))
    new_h = max(1, int(src_h * scale))

    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((screen_h, screen_w, 3), dtype=np.uint8)

    x0 = (screen_w - new_w) // 2
    y0 = (screen_h - new_h) // 2

    canvas[y0:y0 + new_h, x0:x0 + new_w] = resized
    return canvas


def bgr_to_bgra_bytes(frame_bgr: np.ndarray) -> bytes:
    bgra = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2BGRA)
    return bgra.tobytes()


class TtyFramebufferViewer(Node):
    def __init__(self, topic: str, fb_device: str) -> None:
        super().__init__("opencv_picam2_ros_tty_viewer")

        self.topic = topic
        self.fb_device = fb_device
        self.bridge = CvBridge()
        self.closed = False

        self.screen_w, self.screen_h, self.bpp = read_framebuffer_info(fb_device)

        if self.bpp != 32:
            raise RuntimeError(f"Unsupported framebuffer bpp: {self.bpp}. Expected 32 bpp.")

        self.fb = open(fb_device, "r+b", buffering=0)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.subscription = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            qos,
        )

        self.frame_count = 0
        self.last_log_time = time.monotonic()

        print(
            f"TTY framebuffer viewer started: {fb_device}, "
            f"{self.screen_w}x{self.screen_h}, {self.bpp} bpp",
            flush=True,
        )
        print(f"Subscribing to {topic}", flush=True)

    def image_callback(self, msg: Image) -> None:
        if self.closed:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        screen_frame = resize_with_letterbox(frame, self.screen_w, self.screen_h)
        data = bgr_to_bgra_bytes(screen_frame)

        self.fb.seek(0)
        self.fb.write(data)

        self.frame_count += 1
        now = time.monotonic()

        if now - self.last_log_time >= 2.0:
            fps = self.frame_count / (now - self.last_log_time)
            print(f"Displaying stream: {fps:.1f} FPS", flush=True)
            self.frame_count = 0
            self.last_log_time = now

    def close(self) -> None:
        if self.closed:
            return

        self.closed = True

        if hasattr(self, "fb"):
            self.fb.close()

        print("TTY camera viewer stopped.", flush=True)

    def destroy_node(self) -> None:
        self.close()
        super().destroy_node()


def spin_until_stopped(node: Node) -> None:
    while rclpy.ok() and not STOP_REQUESTED:
        rclpy.spin_once(node, timeout_sec=0.1)


def main() -> int:
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/savo_head/picam/image_raw")
    parser.add_argument("--fb", default="/dev/fb0")
    args = parser.parse_args()

    rclpy.init(args=None)

    node: Optional[TtyFramebufferViewer] = None

    try:
        node = TtyFramebufferViewer(args.topic, args.fb)
        spin_until_stopped(node)

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        print("Shutdown complete.", flush=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
