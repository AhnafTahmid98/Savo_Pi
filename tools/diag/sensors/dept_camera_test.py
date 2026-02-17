#!/usr/bin/env python3
"""
Robot Savo — D435 Depth Diagnostic (ROS-based, reliable)

Why ROS-based:
- Your D435 depth stream is confirmed working in ROS2 at ~30 Hz.
- OpenCV/V4L2 often cannot reliably stream Z16 depth from /dev/video6.
- This tool subscribes to the ROS depth topic and computes a robust "front obstacle distance".

What it does:
- Subscribes: /camera/camera/depth/image_rect_raw   (sensor_msgs/Image, encoding usually 16UC1)
- Computes: robust near distance (percentile) inside ROI
- Prints: dist + status [OK/SLOW/STOP/NO_DATA]
- Optional: publish /depth/min_front_m (std_msgs/Float32)
- Optional: CSV logging

Run:
  # Terminal A: start RealSense driver
  ros2 launch realsense2_camera rs_launch.py

  # Terminal B: run this tool
  python3 tools/diag/sensors/dept_camera_test.py

Options:
  --depth-topic /camera/camera/depth/image_rect_raw
  --publish-topic /depth/min_front_m
  --no-publish
  --csv out.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from cv_bridge import CvBridge


@dataclass(frozen=True)
class ROIFrac:
    x0: float = 0.40
    x1: float = 0.60
    y0: float = 0.35
    y1: float = 0.80

    def to_pixels(self, w: int, h: int) -> Tuple[int, int, int, int]:
        x0 = int(self.x0 * w)
        x1 = int(self.x1 * w)
        y0 = int(self.y0 * h)
        y1 = int(self.y1 * h)

        x0 = max(0, min(w - 1, x0))
        x1 = max(x0 + 1, min(w, x1))
        y0 = max(0, min(h - 1, y0))
        y1 = max(y0 + 1, min(h, y1))
        return x0, x1, y0, y1


class DepthDiag(Node):
    def __init__(self, args):
        super().__init__("dept_camera_test")

        self.args = args
        self.bridge = CvBridge()
        self.roi = ROIFrac(args.roi_x0, args.roi_x1, args.roi_y0, args.roi_y1)

        self.pub: Optional[rclpy.publisher.Publisher] = None
        if not args.no_publish:
            self.pub = self.create_publisher(Float32, args.publish_topic, 10)
            self.get_logger().info(f"Publishing: {args.publish_topic}")

        self.csv_f = None
        self.csv_w = None
        if args.csv:
            self.csv_f = open(args.csv, "w", newline="")
            self.csv_w = csv.writer(self.csv_f)
            self.csv_w.writerow(["t_s", "dist_m", "valid_px", "roi_x0", "roi_x1", "roi_y0", "roi_y1"])

        self.last_print_t = 0.0
        self.frame_count = 0
        self.last_hz_t = time.time()
        self.hz_est = 0.0

        self.sub = self.create_subscription(Image, args.depth_topic, self.on_depth, 10)
        self.get_logger().info(f"Subscribing: {args.depth_topic}")

        self.get_logger().info(
            f"ROI frac: x[{args.roi_x0:.2f},{args.roi_x1:.2f}] y[{args.roi_y0:.2f},{args.roi_y1:.2f}] "
            f"percentile={args.percentile:.1f}"
        )

    def _compute_distance(self, depth: np.ndarray) -> Tuple[float, int, int, int, int, int]:
        # Depth image expected 16UC1 (often mm). We'll handle both mm and meters via flag.
        if depth is None:
            return float("nan"), 0, 0, 0, 0, 0

        if depth.ndim == 3 and depth.shape[2] == 1:
            depth = depth[:, :, 0]

        if depth.ndim != 2:
            return float("nan"), 0, 0, 0, 0, 0

        h, w = depth.shape[:2]
        x0, x1, y0, y1 = self.roi.to_pixels(w, h)

        roi_px = depth[y0:y1, x0:x1].astype(np.float32)

        if self.args.unit_mm:
            roi_px *= 0.001  # mm -> m

        roi_px = roi_px[np.isfinite(roi_px)]
        roi_px = roi_px[(roi_px >= self.args.min_valid_m) & (roi_px <= self.args.max_valid_m)]
        valid = int(roi_px.size)

        if valid < 50:
            return float("nan"), x0, x1, y0, y1, valid

        dist = float(np.percentile(roi_px, self.args.percentile))
        return dist, x0, x1, y0, y1, valid

    def on_depth(self, msg: Image):
        # Convert using cv_bridge
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        dist_m, x0, x1, y0, y1, valid_px = self._compute_distance(depth)

        # Publish Float32 if enabled
        if self.pub is not None:
            out = Float32()
            out.data = float("nan") if math.isnan(dist_m) else dist_m
            self.pub.publish(out)

        # CSV
        if self.csv_w:
            self.csv_w.writerow([time.time(), dist_m, valid_px, x0, x1, y0, y1])
            self.csv_f.flush()

        # Print status (throttled)
        label = "OK"
        if math.isnan(dist_m):
            label = "NO_DATA"
        elif dist_m <= self.args.stop_th_m:
            label = "STOP"
        elif dist_m <= self.args.warn_th_m:
            label = "SLOW"

        now = time.time()
        if now - self.last_print_t >= self.args.print_period_s:
            print(f"dist={dist_m:5.2f} m  valid_px={valid_px:6d}  [{label}]")
            self.last_print_t = now

        # Hz estimate
        if self.args.print_hz:
            dt = now - self.last_hz_t
            if dt > 0:
                hz = 1.0 / dt
                self.hz_est = hz if self.hz_est <= 0 else (0.9 * self.hz_est + 0.1 * hz)
            self.last_hz_t = now
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                print(f"depth_cb_hz≈{self.hz_est:.1f}")

    def close(self):
        if self.csv_f:
            self.csv_f.close()


def parse_args():
    ap = argparse.ArgumentParser(description="Robot Savo — D435 depth diagnostic (ROS subscriber)")
    ap.add_argument("--depth-topic", default="/camera/camera/depth/image_rect_raw")
    ap.add_argument("--publish-topic", default="/depth/min_front_m")
    ap.add_argument("--no-publish", action="store_true")

    ap.add_argument("--roi-x0", type=float, default=0.40)
    ap.add_argument("--roi-x1", type=float, default=0.60)
    ap.add_argument("--roi-y0", type=float, default=0.35)
    ap.add_argument("--roi-y1", type=float, default=0.80)

    ap.add_argument("--unit-mm", action="store_true", help="Interpret depth samples as millimeters (typical 16UC1).")
    ap.set_defaults(unit_mm=True)

    ap.add_argument("--min-valid-m", type=float, default=0.15)
    ap.add_argument("--max-valid-m", type=float, default=4.00)
    ap.add_argument("--percentile", type=float, default=5.0)

    ap.add_argument("--warn-th-m", type=float, default=0.45)
    ap.add_argument("--stop-th-m", type=float, default=0.28)

    ap.add_argument("--csv", default="", help="Optional CSV log path")
    ap.add_argument("--print-hz", action="store_true", help="Print callback Hz estimate")
    ap.add_argument("--print-period-s", type=float, default=0.25, help="Console print throttle (seconds)")
    return ap.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = DepthDiag(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
