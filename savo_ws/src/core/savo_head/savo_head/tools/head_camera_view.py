# -*- coding: utf-8 -*-

"""Pan-tilt + Pi Camera UDP stream diagnostic for Robot Savo head."""

from __future__ import annotations

import argparse
import os
import select
import shlex
import shutil
import signal
import subprocess
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import Optional

from savo_head.constants import (
    CAMERA_BACKEND_GSTREAMER_LIBCAMERASRC,
    CAMERA_BACKEND_OPENCV_UNVALIDATED,
    CAMERA_BITRATE_KBPS_DEFAULT,
    CAMERA_FORMAT_DEFAULT,
    CAMERA_FPS_DEFAULT,
    CAMERA_HEIGHT_DEFAULT,
    CAMERA_UDP_PORT_DEFAULT,
    CAMERA_WIDTH_DEFAULT,
    PAN_CENTER_DEG_DEFAULT,
    PAN_MAX_DEG_DEFAULT,
    PAN_MIN_DEG_DEFAULT,
    SCAN_STEP_DELAY_S_DEFAULT,
    TILT_CENTER_DEG_DEFAULT,
    TILT_MAX_DEG_DEFAULT,
    TILT_MIN_DEG_DEFAULT,
)
from savo_head.core.scan_pattern import make_scan_runtime
from savo_head.drivers.pantilt_driver import (
    BACKEND_DRYRUN,
    BACKEND_PCA9685,
    PanTiltDriver,
    PanTiltDriverConfig,
)
from savo_head.models.pantilt_command import (
    center_command,
    manual_step_command,
)
from savo_head.models.scan_status import ScanProfile


@dataclass(frozen=True)
class CameraStreamConfig:
    backend: str = CAMERA_BACKEND_GSTREAMER_LIBCAMERASRC
    host: str = ""
    port: int = CAMERA_UDP_PORT_DEFAULT
    width: int = CAMERA_WIDTH_DEFAULT
    height: int = CAMERA_HEIGHT_DEFAULT
    fps: int = CAMERA_FPS_DEFAULT
    fmt: str = CAMERA_FORMAT_DEFAULT
    bitrate_kbps: int = CAMERA_BITRATE_KBPS_DEFAULT
    gst_verbose: bool = False
    print_only: bool = False

    def normalized(self) -> "CameraStreamConfig":
        backend = str(self.backend).strip().lower()

        if backend not in (
            CAMERA_BACKEND_GSTREAMER_LIBCAMERASRC,
            CAMERA_BACKEND_OPENCV_UNVALIDATED,
        ):
            backend = CAMERA_BACKEND_GSTREAMER_LIBCAMERASRC

        return CameraStreamConfig(
            backend=backend,
            host=str(self.host).strip(),
            port=int(self.port),
            width=int(self.width),
            height=int(self.height),
            fps=int(self.fps),
            fmt=str(self.fmt).strip() or "I420",
            bitrate_kbps=int(self.bitrate_kbps),
            gst_verbose=bool(self.gst_verbose),
            print_only=bool(self.print_only),
        )


class CameraStreamProcess:
    def __init__(self, config: CameraStreamConfig):
        self.config = config.normalized()
        self._proc: Optional[subprocess.Popen] = None

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def build_gstreamer_command(self) -> list[str]:
        if self.config.backend != CAMERA_BACKEND_GSTREAMER_LIBCAMERASRC:
            raise NotImplementedError(
                "OpenCV VideoCapture backend is reserved for later validation."
            )

        cmd = ["gst-launch-1.0"]
        cmd.append("-v" if self.config.gst_verbose else "-q")

        cmd.extend(
            [
                "libcamerasrc",
                "!",
                (
                    f"video/x-raw,format={self.config.fmt},"
                    f"width={self.config.width},height={self.config.height},"
                    f"framerate={self.config.fps}/1"
                ),
                "!",
                "videoconvert",
                "!",
                "x264enc",
                "tune=zerolatency",
                f"bitrate={self.config.bitrate_kbps}",
                "speed-preset=superfast",
                "key-int-max=60",
                "!",
                "rtph264pay",
                "config-interval=1",
                "pt=96",
                "!",
                "udpsink",
                f"host={self.config.host}",
                f"port={self.config.port}",
                "sync=false",
                "async=false",
            ]
        )

        return cmd

    def receiver_command(self) -> str:
        return (
            f"gst-launch-1.0 -v "
            f"udpsrc port={self.config.port} "
            f'caps="application/x-rtp, media=video, encoding-name=H264, payload=96" '
            f"! rtph264depay ! h264parse ! avdec_h264 ! videoconvert "
            f"! autovideosink sync=false"
        )

    def print_summary(self) -> None:
        cmd = self.build_gstreamer_command()

        print("\nRobot Savo — Pi Camera 2 NoIR UDP Stream")
        print("-----------------------------------------")
        print(f"Backend    : {self.config.backend}")
        print(f"Host       : {self.config.host}")
        print(f"Port       : {self.config.port}")
        print(f"Resolution : {self.config.width}x{self.config.height}")
        print(f"FPS        : {self.config.fps}")
        print(f"Format     : {self.config.fmt}")
        print(f"Bitrate    : {self.config.bitrate_kbps} kbit/s")
        print("\nSender:")
        print("  " + " ".join(shlex.quote(item) for item in cmd))
        print("\nReceiver:")
        print("  " + self.receiver_command())
        print()

    def start(self) -> None:
        if self.running:
            return

        if self.config.backend == CAMERA_BACKEND_OPENCV_UNVALIDATED:
            raise NotImplementedError(
                "OpenCV VideoCapture is not validated yet for this head camera."
            )

        if not self.config.host:
            raise ValueError("--host is required unless --no-camera is used")

        if shutil.which("gst-launch-1.0") is None:
            raise RuntimeError("gst-launch-1.0 not found. Install GStreamer tools first.")

        cmd = self.build_gstreamer_command()
        self.print_summary()

        if self.config.print_only:
            return

        stdout_target = None if self.config.gst_verbose else subprocess.DEVNULL
        stderr_target = None if self.config.gst_verbose else subprocess.DEVNULL

        self._proc = subprocess.Popen(
            cmd,
            stdout=stdout_target,
            stderr=stderr_target,
            start_new_session=True,
        )

        time.sleep(1.0)

        if self._proc.poll() is not None:
            self._proc = None
            raise RuntimeError("camera pipeline exited early; retry with --gst-verbose")

    def stop(self) -> None:
        proc = self._proc
        self._proc = None

        if proc is None or proc.poll() is not None:
            return

        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=3.0)
        except Exception:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except Exception:
                pass


class RawKeyReader:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = None

    def __enter__(self) -> "RawKeyReader":
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        if self.old_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def read_key(self, timeout_s: float = 0.0) -> str:
        ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if ready:
            return sys.stdin.read(1)
        return ""


def print_help() -> None:
    print("\nControls:")
    print("  W / S     : tilt up / tilt down")
    print("  A / D     : pan left / pan right")
    print("  O         : toggle auto/manual")
    print("  SPACE     : pause/resume auto")
    print("  C         : center")
    print("  H         : help")
    print("  Q or ESC  : quit\n")


def make_scan_profile(args: argparse.Namespace) -> ScanProfile:
    return ScanProfile(
        pan_min_deg=PAN_MIN_DEG_DEFAULT,
        pan_center_deg=int(args.pan_center),
        pan_max_deg=PAN_MAX_DEG_DEFAULT,
        tilt_min_deg=int(args.tilt_min),
        tilt_max_deg=int(args.tilt_max),
        pan_step_deg=int(args.auto_pan_step),
        tilt_step_deg=int(args.auto_tilt_step),
        step_delay_s=float(args.auto_delay),
        start_pan_deg=int(args.auto_pan_min),
        start_tilt_deg=int(args.auto_tilt_min),
        pan_targets_deg=(
            int(args.pan_center),
            int(args.auto_pan_max),
            int(args.pan_center),
            int(args.auto_pan_min),
        ),
        tilt_sweep_pan_targets_deg=(int(args.pan_center),),
    ).normalized()


def print_head_summary(driver: PanTiltDriver, profile: ScanProfile, mode: str, step: int) -> None:
    cal = driver.calibration

    print("\nRobot Savo — Pan-Tilt Camera View")
    print("---------------------------------")
    print(f"Driver backend : {driver.config.backend}")
    print(f"Pan channel    : logical {cal.pan.logical_channel} / PCA9685 {cal.pan.pca9685_channel}")
    print(f"Tilt channel   : logical {cal.tilt.logical_channel} / PCA9685 {cal.tilt.pca9685_channel}")
    print(f"Pan range      : {cal.pan.min_deg} .. {cal.pan.max_deg} deg")
    print(f"Tilt range     : {cal.tilt.min_deg} .. {cal.tilt.max_deg} deg")
    print(f"Center         : pan={cal.pan.center_deg} deg, tilt={cal.tilt.center_deg} deg")
    print(f"Manual step    : {step} deg")
    print(f"Auto targets   : {profile.pan_targets_deg}")
    print(f"Auto tilt sweep: {profile.tilt_min_deg} .. {profile.tilt_max_deg} deg")
    print(f"Auto delay     : {profile.step_delay_s:.3f} s")
    print(f"Start mode     : {mode}")
    print_help()


def run_pantilt(args: argparse.Namespace) -> None:
    driver = PanTiltDriver(
        PanTiltDriverConfig(
            backend=BACKEND_DRYRUN if args.dryrun else BACKEND_PCA9685,
            center_on_open=True,
            center_on_close=True,
        )
    )

    profile = make_scan_profile(args)
    mode = str(args.mode)
    auto_paused = False
    runtime = make_scan_runtime(profile).start(stamp_s=time.monotonic())

    driver.open()
    print_head_summary(driver, profile, mode, args.step)

    try:
        with RawKeyReader() as keys:
            while True:
                key_raw = keys.read_key(timeout_s=0.02)

                if key_raw:
                    key = key_raw.upper()
                    code = ord(key_raw)

                    if key == "Q" or code == 27:
                        print("[INFO] Quit requested.")
                        break

                    if key == "H":
                        print_help()
                        continue

                    if key == "O":
                        mode = "auto" if mode == "manual" else "manual"
                        auto_paused = False
                        print(f"[MODE] {mode.upper()}")
                        continue

                    if key_raw == " ":
                        auto_paused = not auto_paused
                        print(f"[AUTO] paused={auto_paused}")
                        continue

                    if key == "C":
                        state = driver.apply_command(
                            center_command(source="manual", stamp_s=time.monotonic())
                        )
                        print(f"[CENTER] pan={state.pan_deg}°, tilt={state.tilt_deg}°")
                        continue

                    if key in ("W", "A", "S", "D"):
                        mode = "manual"
                        command = manual_step_command(key, step_deg=args.step)
                        state = driver.apply_command(command)
                        print(f"[MANUAL] pan={state.pan_deg}°, tilt={state.tilt_deg}°")
                        continue

                if mode == "auto" and not auto_paused:
                    now = time.monotonic()
                    last = runtime.status.stamp_s

                    if last <= 0.0 or now - last >= profile.step_delay_s:
                        runtime, result = runtime.step(stamp_s=now)
                        state = driver.apply_command(result.command)

                        print(
                            f"[AUTO] phase={runtime.status.phase} "
                            f"target={runtime.status.current_pan_target_deg}° "
                            f"pan={state.pan_deg}°, tilt={state.tilt_deg}°"
                        )

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C caught, exiting...")

    finally:
        try:
            state = driver.apply_command(center_command(source="shutdown", stamp_s=time.monotonic()))
            print(f"[INFO] Recentered: pan={state.pan_deg}°, tilt={state.tilt_deg}°")
        except Exception as exc:
            print(f"[WARN] Could not recenter: {exc}")

        driver.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Robot Savo pan-tilt + Pi Camera 2 NoIR UDP view diagnostic"
    )

    parser.add_argument("--host", default="", help="Laptop/Mac receiver IP address.")
    parser.add_argument("--port", type=int, default=CAMERA_UDP_PORT_DEFAULT)
    parser.add_argument("--width", type=int, default=CAMERA_WIDTH_DEFAULT)
    parser.add_argument("--height", type=int, default=CAMERA_HEIGHT_DEFAULT)
    parser.add_argument("--fps", type=int, default=CAMERA_FPS_DEFAULT)
    parser.add_argument("--format", default=CAMERA_FORMAT_DEFAULT)
    parser.add_argument("--bitrate", type=int, default=CAMERA_BITRATE_KBPS_DEFAULT)
    parser.add_argument("--no-camera", action="store_true")
    parser.add_argument("--print-only", action="store_true")
    parser.add_argument("--gst-verbose", action="store_true")

    parser.add_argument(
        "--camera-backend",
        choices=[
            CAMERA_BACKEND_GSTREAMER_LIBCAMERASRC,
            CAMERA_BACKEND_OPENCV_UNVALIDATED,
        ],
        default=CAMERA_BACKEND_GSTREAMER_LIBCAMERASRC,
    )

    parser.add_argument("--dryrun", action="store_true", help="Do not access PCA9685 hardware.")
    parser.add_argument("--mode", choices=["manual", "auto"], default="manual")

    parser.add_argument("--pan-center", type=int, default=PAN_CENTER_DEG_DEFAULT)
    parser.add_argument("--tilt-center", type=int, default=TILT_CENTER_DEG_DEFAULT)
    parser.add_argument("--tilt-min", type=int, default=TILT_MIN_DEG_DEFAULT)
    parser.add_argument("--tilt-max", type=int, default=TILT_MAX_DEG_DEFAULT)
    parser.add_argument("--step", type=int, default=2)

    parser.add_argument("--auto-pan-min", type=int, default=PAN_MIN_DEG_DEFAULT)
    parser.add_argument("--auto-pan-max", type=int, default=PAN_MAX_DEG_DEFAULT)
    parser.add_argument("--auto-pan-step", type=int, default=2)

    parser.add_argument("--auto-tilt-min", type=int, default=TILT_MIN_DEG_DEFAULT)
    parser.add_argument("--auto-tilt-max", type=int, default=TILT_MAX_DEG_DEFAULT)
    parser.add_argument("--auto-tilt-step", type=int, default=2)
    parser.add_argument("--auto-delay", type=float, default=SCAN_STEP_DELAY_S_DEFAULT)

    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if not args.no_camera and not args.host:
        parser.error("--host is required unless --no-camera is used")

    camera = CameraStreamProcess(
        CameraStreamConfig(
            backend=args.camera_backend,
            host=args.host,
            port=args.port,
            width=args.width,
            height=args.height,
            fps=args.fps,
            fmt=args.format,
            bitrate_kbps=args.bitrate,
            gst_verbose=args.gst_verbose,
            print_only=args.print_only,
        )
    )

    try:
        if not args.no_camera:
            camera.start()

        if args.print_only:
            return 0

        run_pantilt(args)
        return 0

    finally:
        camera.stop()
        print("[Done] head_camera_view.py finished.")


if __name__ == "__main__":
    raise SystemExit(main())
