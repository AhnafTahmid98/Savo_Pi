#!/usr/bin/env python3

import argparse
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


def build_picam_pipeline(width: int, height: int, fps: int) -> str:
    return (
        "libcamerasrc "
        f"! video/x-raw,format=I420,width={width},height={height},framerate={fps}/1 "
        "! videoconvert "
        "! video/x-raw,format=BGR "
        "! appsink drop=true max-buffers=1 sync=false"
    )


class PiCamPublisher(Node):
    def __init__(self, width: int, height: int, fps: int) -> None:
        super().__init__("opencv_picam2_stream_publisher")

        self.width = width
        self.height = height
        self.fps = fps
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(Image, "/savo_head/picam/image_raw", 10)

        pipeline = build_picam_pipeline(width, height, fps)
        self.get_logger().info(f"Opening Pi Camera pipeline: {pipeline}")

        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            raise RuntimeError("Could not open Pi Camera through OpenCV/GStreamer")

        self.frame_count = 0
        self.last_log_time = time.monotonic()

        timer_period = 1.0 / float(fps)
        self.timer = self.create_timer(timer_period, self.publish_frame)

    def publish_frame(self) -> None:
        ok, frame = self.cap.read()

        if not ok or frame is None:
            self.get_logger().warn("Failed to read camera frame")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "picam2_noir_optical_frame"

        self.publisher.publish(msg)

        self.frame_count += 1
        now = time.monotonic()

        if now - self.last_log_time >= 2.0:
            fps = self.frame_count / (now - self.last_log_time)
            self.get_logger().info(f"Publishing camera stream: {fps:.1f} FPS")
            self.frame_count = 0
            self.last_log_time = now

    def destroy_node(self) -> None:
        if hasattr(self, "cap"):
            self.cap.release()
        super().destroy_node()


class StreamViewer(Node):
    def __init__(self, fullscreen: bool) -> None:
        super().__init__("opencv_picam2_stream_viewer")

        self.bridge = CvBridge()
        self.fullscreen = fullscreen
        self.window_name = "Robot Savo Pi Camera 2 NoIR"

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        if fullscreen:
            cv2.setWindowProperty(
                self.window_name,
                cv2.WND_PROP_FULLSCREEN,
                cv2.WINDOW_FULLSCREEN,
            )

        self.subscription = self.create_subscription(
            Image,
            "/savo_head/picam/image_raw",
            self.image_callback,
            10,
        )

        self.frame_count = 0
        self.last_log_time = time.monotonic()

        self.get_logger().info("Waiting for /savo_head/picam/image_raw")

    def image_callback(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        cv2.imshow(self.window_name, frame)
        key = cv2.waitKey(1)

        if key == ord("q") or key == 27:
            self.get_logger().info("Viewer closed by user")
            rclpy.shutdown()
            return

        self.frame_count += 1
        now = time.monotonic()

        if now - self.last_log_time >= 2.0:
            fps = self.frame_count / (now - self.last_log_time)
            self.get_logger().info(f"Viewing camera stream: {fps:.1f} FPS")
            self.frame_count = 0
            self.last_log_time = now

    def destroy_node(self) -> None:
        cv2.destroyAllWindows()
        super().destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        choices=["publish", "view"],
        required=True,
        help="publish on savo-core, view on savo-edge",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--fullscreen", action="store_true")

    args = parser.parse_args()

    rclpy.init()

    node = None

    try:
        if args.mode == "publish":
            node = PiCamPublisher(args.width, args.height, args.fps)
        else:
            node = StreamViewer(args.fullscreen)

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
