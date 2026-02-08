#!/usr/bin/env python3
"""
Robot Savo — Intel RealSense D435 Depth Diagnostic (non-ROS)

Purpose (expert-level, practical):
- Verify D435 streams are accessible via V4L2 on Raspberry Pi 5 (Ubuntu Server 24.04).
- Compute a robust "front obstacle distance" from the depth stream using an ROI + percentile.
- Optionally preview depth (false color) + color (YUYV) for quick sanity checks.
- Optional CSV logging for later analysis.

Your current device mapping (from v4l2-ctl):
- Depth (Z16)  : /dev/video6
- IR (GREY)    : /dev/video10
- Color (YUYV) : /dev/video12

Important reality:
- Depth Z16 via OpenCV/V4L2 can be flaky on some platforms. If you see repeated
  NO_DATA or read failures BUT ROS2 realsense2_camera works at 30 Hz, then your camera
  is fine; it’s the non-ROS decode path. In that case, use a ROS-based diagnostic node.

Usage:
  # Depth only (recommended first)
  python3 dept_camera_test.py

  # With preview windows (requires display)
  python3 dept_camera_test.py --preview

  # Force devices / profiles
  python3 dept_camera_test.py --depth-dev /dev/video6 --depth-w 848 --depth-h 480 --fps 30

  # Log CSV
  python3 dept_camera_test.py --csv depth_log.csv

Keys:
  q  -> quit (when --preview enabled)
  Ctrl+C -> quit

Exit codes:
  0 success
  2 cannot open depth device
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

try:
    import cv2
except Exception:
    print("ERROR: OpenCV not available. Install: sudo apt install -y python3-opencv")
    raise


@dataclass(frozen=True)
class ROIFrac:
    """ROI expressed as fractions of width/height."""
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


def open_v4l2(
    dev: str,
    width: int,
    height: int,
    fps: int,
    fourcc: Optional[str],
) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        return cap

    # Hint desired pixel format FIRST (helps avoid "Invalid argument")
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
    cap.set(cv2.CAP_PROP_FPS, int(fps))

    # Reduce buffering latency if supported by backend
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def rate_estimate(prev_hz: float, dt: float) -> float:
    if dt <= 0:
        return prev_hz
    hz = 1.0 / dt
    return hz if prev_hz <= 0 else (0.9 * prev_hz + 0.1 * hz)


def compute_front_distance_m(
    depth_frame: np.ndarray,
    roi: ROIFrac,
    unit_is_mm: bool,
    min_valid_m: float,
    max_valid_m: float,
    percentile: float,
) -> Tuple[float, int, int, int, int, int]:
    """
    Returns:
      dist_m, x0, x1, y0, y1, valid_count

    dist_m is NaN if insufficient valid pixels.
    """
    if depth_frame is None:
        return float("nan"), 0, 0, 0, 0, 0

    # Normalize depth array shape
    if depth_frame.ndim == 3:
        if depth_frame.shape[2] == 1:
            depth_frame = depth_frame[:, :, 0]
        else:
            return float("nan"), 0, 0, 0, 0, 0

    if depth_frame.ndim != 2:
        return float("nan"), 0, 0, 0, 0, 0

    h, w = depth_frame.shape[:2]
    x0, x1, y0, y1 = roi.to_pixels(w, h)

    roi_px = depth_frame[y0:y1, x0:x1].astype(np.float32)

    if unit_is_mm:
        roi_px *= 0.001  # mm -> m

    # Filter valid depth
    roi_px = roi_px[np.isfinite(roi_px)]
    roi_px = roi_px[(roi_px >= min_valid_m) & (roi_px <= max_valid_m)]
    valid = int(roi_px.size)

    if valid < 50:
        return float("nan"), x0, x1, y0, y1, valid

    dist = float(np.percentile(roi_px, percentile))
    return dist, x0, x1, y0, y1, valid


def depth_false_color(depth_m: np.ndarray, near_m: float = 0.15, far_m: float = 2.0) -> np.ndarray:
    """
    Create a displayable false-color image from depth (meters).
    Maps [near_m..far_m] -> [0..255] then applies colormap.
    """
    d = np.clip(depth_m, near_m, far_m)
    u8 = ((d - near_m) / (far_m - near_m) * 255.0).astype(np.uint8)
    return cv2.applyColorMap(u8, cv2.COLORMAP_JET)


def main() -> int:
    ap = argparse.ArgumentParser(description="Robot Savo — D435 depth diagnostic (non-ROS)")
    ap.add_argument("--depth-dev", default="/dev/video6", help="Depth V4L2 device (Z16). Default: /dev/video6")
    ap.add_argument("--color-dev", default="/dev/video12", help="Color V4L2 device (YUYV). Default: /dev/video12")
    ap.add_argument("--no-color", action="store_true", help="Disable color capture")

    ap.add_argument("--depth-w", type=int, default=848)
    ap.add_argument("--depth-h", type=int, default=480)
    ap.add_argument("--color-w", type=int, default=640)
    ap.add_argument("--color-h", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)

    ap.add_argument("--roi-x0", type=float, default=0.40)
    ap.add_argument("--roi-x1", type=float, default=0.60)
    ap.add_argument("--roi-y0", type=float, default=0.35)
    ap.add_argument("--roi-y1", type=float, default=0.80)

    ap.add_argument("--unit-mm", action="store_true", help="Interpret depth samples as millimeters (Z16 typical).")
    ap.set_defaults(unit_mm=True)

    ap.add_argument("--min-valid-m", type=float, default=0.15)
    ap.add_argument("--max-valid-m", type=float, default=4.0)
    ap.add_argument("--percentile", type=float, default=5.0, help="Robust 'near obstacle' metric; 5th percentile recommended.")
    ap.add_argument("--warn-th-m", type=float, default=0.45, help="Warn threshold (m) for printing.")
    ap.add_argument("--stop-th-m", type=float, default=0.28, help="Stop threshold (m) for printing.")

    ap.add_argument("--preview", action="store_true", help="Show preview windows (needs display).")
    ap.add_argument("--csv", default="", help="CSV output path (optional).")
    ap.add_argument("--print-hz", action="store_true", help="Print estimated loop Hz occasionally.")
    args = ap.parse_args()

    roi = ROIFrac(args.roi_x0, args.roi_x1, args.roi_y0, args.roi_y1)

    # Depth: try to request Z16. Note: OpenCV may ignore FOURCC for some drivers.
    depth_cap = open_v4l2(args.depth_dev, args.depth_w, args.depth_h, args.fps, fourcc="Z16 ")
    if not depth_cap.isOpened():
        print(f"ERROR: Cannot open depth device: {args.depth_dev}")
        print("Checks:")
        print("  - v4l2-ctl --list-devices  (confirm mapping)")
        print("  - ensure no other process is using it (ROS node, ffplay, etc.)")
        print("  - try: sudo python3 dept_camera_test.py  (permissions)")
        return 2

    color_cap: Optional[cv2.VideoCapture] = None
    if not args.no_color and args.color_dev:
        color_cap = open_v4l2(args.color_dev, args.color_w, args.color_h, args.fps, fourcc="YUYV")
        if not color_cap.isOpened():
            print(f"WARNING: Cannot open color device: {args.color_dev} (continuing depth-only)")
            color_cap = None

    csv_f = None
    writer = None
    if args.csv:
        csv_f = open(args.csv, "w", newline="")
        writer = csv.writer(csv_f)
        writer.writerow(["t_s", "dist_m", "valid_px", "roi_x0", "roi_x1", "roi_y0", "roi_y1"])

    print("\nRobot Savo — D435 Depth Diagnostic (non-ROS)")
    print(f"Depth : {args.depth_dev}  profile={args.depth_w}x{args.depth_h}@{args.fps}  (expect Z16)")
    if color_cap:
        print(f"Color : {args.color_dev}  profile={args.color_w}x{args.color_h}@{args.fps}  (YUYV)")
    else:
        print("Color : disabled/unavailable")
    print(f"ROI   : x[{roi.x0:.2f},{roi.x1:.2f}] y[{roi.y0:.2f},{roi.y1:.2f}]  percentile={args.percentile:.1f}")
    print(f"Valid : [{args.min_valid_m:.2f}..{args.max_valid_m:.2f}] m  unit_mm={bool(args.unit_mm)}")
    print(f"Thresh: warn={args.warn_th_m:.2f} m  stop={args.stop_th_m:.2f} m")
    print("Ctrl+C to stop.\n")

    last_t = time.time()
    hz_est = 0.0
    frame_i = 0
    consecutive_read_fail = 0

    try:
        while True:
            t0 = time.time()

            ok_d, depth = depth_cap.read()
            if not ok_d or depth is None:
                consecutive_read_fail += 1
                if consecutive_read_fail % 30 == 0:
                    print("WARNING: Depth read failing repeatedly.")
                    print("  If ROS2 realsense2_camera works (it does for you), this is likely OpenCV/V4L2 Z16 handling.")
                    print("  In that case, keep using ROS for depth in production and convert this diagnostic to ROS subscriber.")
                time.sleep(0.01)
                continue
            consecutive_read_fail = 0

            # Compute distance
            dist_m, x0, x1, y0, y1, valid_px = compute_front_distance_m(
                depth_frame=depth,
                roi=roi,
                unit_is_mm=bool(args.unit_mm),
                min_valid_m=float(args.min_valid_m),
                max_valid_m=float(args.max_valid_m),
                percentile=float(args.percentile),
            )

            label = "OK"
            if math.isnan(dist_m):
                label = "NO_DATA"
            elif dist_m <= args.stop_th_m:
                label = "STOP"
            elif dist_m <= args.warn_th_m:
                label = "SLOW"

            print(f"dist={dist_m:5.2f} m  valid_px={valid_px:6d}  [{label}]")

            if writer:
                writer.writerow([t0, dist_m, valid_px, x0, x1, y0, y1])
                csv_f.flush()

            # Preview
            if args.preview:
                disp = depth
                if disp.ndim == 3 and disp.shape[2] == 1:
                    disp = disp[:, :, 0]

                disp_f = disp.astype(np.float32)
                if bool(args.unit_mm):
                    disp_f *= 0.001

                depth_img = depth_false_color(disp_f, near_m=0.15, far_m=2.0)
                cv2.rectangle(depth_img, (x0, y0), (x1 - 1, y1 - 1), (255, 255, 255), 1)
                cv2.putText(depth_img, f"{dist_m:.2f}m {label}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.imshow("D435 Depth (diag)", depth_img)

                if color_cap:
                    ok_c, color = color_cap.read()
                    if ok_c and color is not None:
                        # Many OpenCV builds already output BGR for V4L2 YUYV.
                        # If you see weird colors, uncomment conversion attempt.
                        # if color.ndim == 2:
                        #     color = cv2.cvtColor(color, cv2.COLOR_YUV2BGR_YUY2)
                        cv2.imshow("D435 Color (diag)", color)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break

            # Hz estimate
            if args.print_hz:
                dt = t0 - last_t
                hz_est = rate_estimate(hz_est, dt)
                last_t = t0
                frame_i += 1
                if frame_i % 30 == 0:
                    print(f"loop_hz≈{hz_est:.1f}")

    except KeyboardInterrupt:
        pass
    finally:
        try:
            depth_cap.release()
        except Exception:
            pass
        try:
            if color_cap:
                color_cap.release()
        except Exception:
            pass
        try:
            if writer and csv_f:
                csv_f.close()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

    print("\nDone.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
