#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Robot Savo pan-tilt + Pi Camera 2 NoIR UDP stream diagnostic.

Camera:
  Pi Camera 2 NoIR -> GStreamer/libcamerasrc -> H.264/RTP/UDP -> MacBook

Pan-tilt:
  Manual: W/S/A/D
  Auto: staged scan:
    pan 0 -> 72
    tilt 45 -> 150 -> 45
    pan 72 -> 170
    pan 170 -> 72
    tilt 45 -> 150 -> 45
    pan 72 -> 0
    repeat
"""

import argparse
import math
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

import smbus


# -----------------------------
# Camera streaming
# -----------------------------

def require(bin_name: str, human: str) -> None:
    """Ensure a required binary exists in PATH."""
    if shutil.which(bin_name) is None:
        print(
            f"[Camera] ERROR: {human} not found in PATH: {bin_name}",
            file=sys.stderr,
        )
        print(
            "Install required GStreamer packages first.",
            file=sys.stderr,
        )
        sys.exit(1)


def build_gst_pipeline(
    host: str,
    port: int,
    width: int,
    height: int,
    fps: int,
    bitrate_kbps: int,
    verbose: bool,
) -> list[str]:
    """Build the H.264 RTP UDP sender pipeline."""
    cmd = ["gst-launch-1.0"]

    if verbose:
        cmd.append("-v")
    else:
        cmd.append("-q")

    cmd.extend(
        [
            "libcamerasrc",
            "!",
            f"video/x-raw,format=I420,width={width},height={height},framerate={fps}/1",
            "!",
            "videoconvert",
            "!",
            "x264enc",
            "tune=zerolatency",
            f"bitrate={int(bitrate_kbps)}",
            "speed-preset=superfast",
            "key-int-max=60",
            "!",
            "rtph264pay",
            "config-interval=1",
            "pt=96",
            "!",
            "udpsink",
            f"host={host}",
            f"port={port}",
            "sync=false",
            "async=false",
        ]
    )

    return cmd


def start_camera_stream(args: argparse.Namespace) -> subprocess.Popen | None:
    """Start camera stream as a background process."""
    if args.no_camera:
        print("[Camera] Disabled with --no-camera.")
        return None

    require("gst-launch-1.0", "GStreamer")

    cmd = build_gst_pipeline(
        host=args.host,
        port=args.port,
        width=args.width,
        height=args.height,
        fps=args.fps,
        bitrate_kbps=args.bitrate,
        verbose=args.gst_verbose,
    )

    print("\n[Camera] Robot Savo — Pi Camera UDP Stream")
    print("------------------------------------------")
    print(f"[Camera] Host       : {args.host}")
    print(f"[Camera] Port       : {args.port}")
    print(f"[Camera] Resolution : {args.width}x{args.height}")
    print(f"[Camera] FPS        : {args.fps}")
    print(f"[Camera] Bitrate    : {args.bitrate} kbit/s")
    print("[Camera] Sender pipeline:")
    print("  " + " ".join(shlex.quote(x) for x in cmd))

    print("\n[Camera] MacBook receiver command:")
    print("  gst-launch-1.0 -v \\")
    print(
        f"    udpsrc port={args.port} "
        'caps="application/x-rtp, media=video, encoding-name=H264, payload=96" ! \\'
    )
    print("    rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false\n")

    if args.print_only:
        print("[Camera] --print-only set. Camera stream not started.")
        return None

    stdout_target = None if args.gst_verbose else subprocess.DEVNULL
    stderr_target = None if args.gst_verbose else subprocess.DEVNULL

    print("[Camera] Starting camera stream in background...")
    proc = subprocess.Popen(
        cmd,
        stdout=stdout_target,
        stderr=stderr_target,
        start_new_session=True,
    )
    time.sleep(1.0)

    if proc.poll() is not None:
        print("[Camera] ERROR: camera pipeline exited early.")
        print("[Camera] Re-run with --gst-verbose to see details.")
        return None

    print("[Camera] Stream running.")
    return proc


def stop_camera_stream(proc: subprocess.Popen | None) -> None:
    """Stop camera stream process."""
    if proc is None:
        return

    if proc.poll() is not None:
        print("[Camera] Stream already stopped.")
        return

    print("[Camera] Stopping camera stream...")
    try:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=3.0)
    except Exception:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except Exception:
            pass


# -----------------------------
# PCA9685 / servo control
# -----------------------------

class PCA9685:
    __MODE1 = 0x00
    __PRESCALE = 0xFE
    __LED0_ON_L = 0x06
    __LED0_ON_H = 0x07
    __LED0_OFF_L = 0x08
    __LED0_OFF_H = 0x09

    def __init__(self, address: int = 0x40, debug: bool = False):
        self.bus = smbus.SMBus(1)
        self.address = address
        self.debug = debug
        self.write(self.__MODE1, 0x00)

    def write(self, reg: int, value: int) -> None:
        self.bus.write_byte_data(self.address, reg, value)

    def read(self, reg: int) -> int:
        return self.bus.read_byte_data(self.address, reg)

    def set_pwm_freq(self, freq: float) -> None:
        prescaleval = 25000000.0
        prescaleval /= 4096.0
        prescaleval /= float(freq)
        prescaleval -= 1.0
        prescale = math.floor(prescaleval + 0.5)

        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10

        self.write(self.__MODE1, newmode)
        self.write(self.__PRESCALE, int(prescale))
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)

    def set_pwm(self, channel: int, on: int, off: int) -> None:
        self.write(self.__LED0_ON_L + 4 * channel, on & 0xFF)
        self.write(self.__LED0_ON_H + 4 * channel, on >> 8)
        self.write(self.__LED0_OFF_L + 4 * channel, off & 0xFF)
        self.write(self.__LED0_OFF_H + 4 * channel, off >> 8)

    def set_servo_pulse(self, channel: int, pulse_us: float) -> None:
        pulse = pulse_us * 4096.0 / 20000.0
        self.set_pwm(channel, 0, int(pulse))

    def close(self) -> None:
        self.bus.close()


class Servo:
    def __init__(self, addr: int = 0x40, debug: bool = True):
        self.pwm_frequency = 50

        # Freenove logical servo ports -> real PCA9685 channels.
        # Robot Savo validated mapping:
        #   logical 7 = pan  / left-right
        #   logical 6 = tilt / up-down
        self.pwm_channel_map = {
            "0": 8,
            "1": 9,
            "2": 10,
            "3": 11,
            "4": 12,
            "5": 13,
            "6": 14,
            "7": 15,
        }

        self.pwm_servo = PCA9685(addr, debug=debug)
        self.pwm_servo.set_pwm_freq(self.pwm_frequency)

    def angle_to_pulse(self, channel: str, angle: int, error: int = 10) -> float:
        angle = int(angle)

        if channel == "0":
            pulse = 2500.0 - float((angle + error) / 0.09)
        else:
            pulse = 500.0 + float((angle + error) / 0.09)

        return max(500.0, min(2500.0, pulse))

    def set_servo_angle(self, channel: str, angle: int, error: int = 10) -> None:
        if channel not in self.pwm_channel_map:
            raise ValueError(
                f"Invalid channel: {channel}. Valid: {list(self.pwm_channel_map.keys())}"
            )

        angle = clamp(angle, 0, 180)
        pulse = self.angle_to_pulse(channel, angle, error)
        pca_channel = self.pwm_channel_map[channel]
        self.pwm_servo.set_servo_pulse(pca_channel, pulse)

    def close(self) -> None:
        self.pwm_servo.close()


class RawKeyReader:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = None

    def __enter__(self):
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.old_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def read_key(self, timeout_s: float = 0.0) -> str:
        ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if ready:
            return sys.stdin.read(1)
        return ""


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, int(value)))


def servo_safe_angle(angle: int) -> int:
    return clamp(angle, 0, 180)


def print_help() -> None:
    print("\nControls:")
    print("  W / S     : tilt up / tilt down")
    print("  A / D     : pan left / pan right")
    print("  O         : toggle auto/manual")
    print("  SPACE     : pause/resume auto")
    print("  C         : calibrated center")
    print("  H         : show help")
    print("  Q or ESC  : quit\n")


def run_pantilt(args: argparse.Namespace) -> None:
    servo = Servo(addr=0x40, debug=True)

    pan_min = 0
    pan_max = 170

    pan_center = clamp(args.pan_center, pan_min, pan_max)
    tilt_center = clamp(args.tilt_center, args.tilt_min, args.tilt_max)

    auto_pan_min = clamp(args.auto_pan_min, pan_min, pan_max)
    auto_pan_max = clamp(args.auto_pan_max, pan_min, pan_max)
    auto_tilt_min = clamp(args.auto_tilt_min, args.tilt_min, args.tilt_max)
    auto_tilt_max = clamp(args.auto_tilt_max, args.tilt_min, args.tilt_max)

    if auto_pan_min > auto_pan_max:
        auto_pan_min, auto_pan_max = auto_pan_max, auto_pan_min

    if auto_tilt_min > auto_tilt_max:
        auto_tilt_min, auto_tilt_max = auto_tilt_max, auto_tilt_min

    pan_angle = auto_pan_min if args.mode == "auto" else pan_center
    tilt_angle = auto_tilt_min if args.mode == "auto" else tilt_center

    # Staged scan:
    # 0 -> 72 -> tilt sweep -> 170 -> 72 -> tilt sweep -> 0 -> repeat.
    pan_targets = [
        pan_center,
        auto_pan_max,
        pan_center,
        auto_pan_min,
    ]
    pan_target_index = 0
    current_pan_target = pan_targets[pan_target_index]

    tilt_phase = "idle"
    tilt_direction = 1

    auto_paused = False
    mode = args.mode

    servo.set_servo_angle(args.pan_chan, servo_safe_angle(pan_angle))
    servo.set_servo_angle(args.tilt_chan, servo_safe_angle(tilt_angle))

    print("\nRobot Savo — Pan-Tilt Camera View Test")
    print("--------------------------------------")
    print(f"Pan channel : '{args.pan_chan}' (PCA9685 ch {servo.pwm_channel_map[args.pan_chan]})")
    print(f"Tilt channel: '{args.tilt_chan}' (PCA9685 ch {servo.pwm_channel_map[args.tilt_chan]})")
    print(f"Pan range   : {pan_min} .. {pan_max} deg")
    print(f"Tilt range  : {args.tilt_min} .. {args.tilt_max} deg")
    print(f"Pan center  : {pan_center} deg")
    print(f"Tilt center : {tilt_center} deg")
    print(f"Manual step : {args.step} deg")
    print(f"Auto pan    : {auto_pan_min} .. {auto_pan_max} deg, step={args.auto_pan_step}")
    print(f"Auto tilt   : {auto_tilt_min} .. {auto_tilt_max} deg, step={args.auto_tilt_step}")
    print(f"Auto delay  : {args.auto_delay:.3f} s")
    print(f"Start mode  : {mode}")

    print_help()
    print(f"[INFO] Starting at pan={pan_angle}°, tilt={tilt_angle}°")

    last_auto_time = time.monotonic()

    try:
        with RawKeyReader() as kr:
            while True:
                key_raw = kr.read_key(timeout_s=0.02)

                if key_raw:
                    code = ord(key_raw)
                    key = key_raw.upper()

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
                        pan_angle = pan_center
                        tilt_angle = tilt_center
                        servo.set_servo_angle(args.pan_chan, servo_safe_angle(pan_angle))
                        servo.set_servo_angle(args.tilt_chan, servo_safe_angle(tilt_angle))
                        print(f"[CENTER] pan={pan_angle}°, tilt={tilt_angle}°")
                        continue

                    if key == "A":
                        mode = "manual"
                        pan_angle = clamp(pan_angle - args.step, pan_min, pan_max)
                        servo.set_servo_angle(args.pan_chan, servo_safe_angle(pan_angle))
                        print(f"[PAN] Left  → pan={pan_angle}°, tilt={tilt_angle}°")

                    elif key == "D":
                        mode = "manual"
                        pan_angle = clamp(pan_angle + args.step, pan_min, pan_max)
                        servo.set_servo_angle(args.pan_chan, servo_safe_angle(pan_angle))
                        print(f"[PAN] Right → pan={pan_angle}°, tilt={tilt_angle}°")

                    elif key == "W":
                        mode = "manual"
                        tilt_angle = clamp(
                            tilt_angle + args.step,
                            args.tilt_min,
                            args.tilt_max,
                        )
                        servo.set_servo_angle(args.tilt_chan, servo_safe_angle(tilt_angle))
                        print(f"[TILT] Up   → pan={pan_angle}°, tilt={tilt_angle}°")

                    elif key == "S":
                        mode = "manual"
                        tilt_angle = clamp(
                            tilt_angle - args.step,
                            args.tilt_min,
                            args.tilt_max,
                        )
                        servo.set_servo_angle(args.tilt_chan, servo_safe_angle(tilt_angle))
                        print(f"[TILT] Down → pan={pan_angle}°, tilt={tilt_angle}°")

                now = time.monotonic()

                if mode == "auto" and not auto_paused:
                    if now - last_auto_time >= args.auto_delay:
                        last_auto_time = now

                        if tilt_phase == "idle":
                            # Move pan toward the current target.
                            if pan_angle < current_pan_target:
                                pan_angle = min(
                                    pan_angle + args.auto_pan_step,
                                    current_pan_target,
                                )
                            elif pan_angle > current_pan_target:
                                pan_angle = max(
                                    pan_angle - args.auto_pan_step,
                                    current_pan_target,
                                )

                            servo.set_servo_angle(
                                args.pan_chan,
                                servo_safe_angle(pan_angle),
                            )
                            servo.set_servo_angle(
                                args.tilt_chan,
                                servo_safe_angle(tilt_angle),
                            )

                            print(
                                f"[AUTO-PAN] pan={pan_angle}°, "
                                f"tilt={tilt_angle}°, target={current_pan_target}°"
                            )

                            if pan_angle == current_pan_target:
                                # Tilt sweep only at calibrated center pan.
                                if current_pan_target == pan_center:
                                    tilt_angle = auto_tilt_min
                                    tilt_direction = 1
                                    tilt_phase = "sweep"
                                    servo.set_servo_angle(
                                        args.tilt_chan,
                                        servo_safe_angle(tilt_angle),
                                    )
                                    print(
                                        f"[AUTO-TILT-START] pan={pan_angle}°, "
                                        f"tilt={tilt_angle}°"
                                    )
                                else:
                                    pan_target_index = (
                                        pan_target_index + 1
                                    ) % len(pan_targets)
                                    current_pan_target = pan_targets[pan_target_index]
                                    print(
                                        f"[AUTO-NEXT] next pan target="
                                        f"{current_pan_target}°"
                                    )

                        elif tilt_phase == "sweep":
                            # Tilt sweep: 45 -> 150 -> 45.
                            tilt_angle += tilt_direction * args.auto_tilt_step

                            if tilt_angle >= auto_tilt_max:
                                tilt_angle = auto_tilt_max
                                tilt_direction = -1

                            elif tilt_angle <= auto_tilt_min:
                                tilt_angle = auto_tilt_min
                                tilt_phase = "idle"

                                pan_target_index = (
                                    pan_target_index + 1
                                ) % len(pan_targets)
                                current_pan_target = pan_targets[pan_target_index]

                                print(
                                    f"[AUTO-TILT-DONE] next pan target="
                                    f"{current_pan_target}°"
                                )

                            servo.set_servo_angle(
                                args.tilt_chan,
                                servo_safe_angle(tilt_angle),
                            )
                            print(
                                f"[AUTO-TILT] pan={pan_angle}°, tilt={tilt_angle}°"
                            )

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C caught, exiting...")

    finally:
        try:
            servo.set_servo_angle(args.pan_chan, pan_center)
            servo.set_servo_angle(args.tilt_chan, tilt_center)
            print(f"[INFO] On exit: recentered to pan={pan_center}°, tilt={tilt_center}°")
        except Exception as e:
            print(f"[WARN] Could not recenter on exit: {e}")

        servo.close()


# -----------------------------
# CLI / main
# -----------------------------

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="Robot Savo — Pan-tilt camera view test with UDP stream"
    )

    # Camera options
    ap.add_argument(
        "--host",
        required=False,
        default=None,
        help="MacBook/laptop IP address. Example: 10.57.81.173",
    )
    ap.add_argument(
        "--port",
        type=int,
        default=5000,
        help="UDP port on receiver. Default: 5000.",
    )
    ap.add_argument(
        "--width",
        type=int,
        default=640,
        help="Video width. Default: 640.",
    )
    ap.add_argument(
        "--height",
        type=int,
        default=480,
        help="Video height. Default: 480.",
    )
    ap.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Frame rate. Default: 30.",
    )
    ap.add_argument(
        "--bitrate",
        type=int,
        default=2000,
        help="H.264 bitrate in kbit/s. Default: 2000.",
    )
    ap.add_argument(
        "--no-camera",
        action="store_true",
        help="Disable camera stream and test pan-tilt only.",
    )
    ap.add_argument(
        "--print-only",
        action="store_true",
        help="Print camera pipeline only; do not start stream.",
    )
    ap.add_argument(
        "--gst-verbose",
        action="store_true",
        help="Show GStreamer/libcamera logs.",
    )

    # Pan-tilt options
    ap.add_argument(
        "--mode",
        choices=["manual", "auto"],
        default="manual",
        help="Start pan-tilt mode. Default: manual.",
    )
    ap.add_argument(
        "--pan-chan",
        default="7",
        help="Logical pan servo channel. Default: 7.",
    )
    ap.add_argument(
        "--tilt-chan",
        default="6",
        help="Logical tilt servo channel. Default: 6.",
    )
    ap.add_argument(
        "--pan-center",
        type=int,
        default=72,
        help="Calibrated pan center. Default: 72.",
    )
    ap.add_argument(
        "--tilt-center",
        type=int,
        default=55,
        help="Calibrated tilt center. Default: 55.",
    )
    ap.add_argument(
        "--tilt-min",
        type=int,
        default=45,
        help="Manual tilt minimum. Default: 45.",
    )
    ap.add_argument(
        "--tilt-max",
        type=int,
        default=130,
        help="Manual tilt maximum. Default: 130.",
    )
    ap.add_argument(
        "--step",
        type=int,
        default=2,
        help="Manual step. Default: 2 degrees.",
    )
    ap.add_argument(
        "--auto-pan-min",
        type=int,
        default=0,
        help="Automatic pan minimum. Default: 0.",
    )
    ap.add_argument(
        "--auto-pan-max",
        type=int,
        default=170,
        help="Automatic pan maximum. Default: 170.",
    )
    ap.add_argument(
        "--auto-pan-step",
        type=int,
        default=2,
        help="Automatic pan step. Default: 2 degrees.",
    )
    ap.add_argument(
        "--auto-tilt-min",
        type=int,
        default=45,
        help="Automatic tilt minimum. Default: 45.",
    )
    ap.add_argument(
        "--auto-tilt-max",
        type=int,
        default=130,
        help="Automatic tilt maximum. Default: 130.",
    )
    ap.add_argument(
        "--auto-tilt-step",
        type=int,
        default=2,
        help="Automatic tilt step. Default: 2 degrees.",
    )
    ap.add_argument(
        "--auto-delay",
        type=float,
        default=0.12,
        help="Delay between auto steps. Default: 0.12 seconds.",
    )

    return ap


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    if not args.no_camera and not args.host:
        parser.error("--host is required unless --no-camera is used.")

    camera_proc = None

    try:
        camera_proc = start_camera_stream(args)

        if args.print_only:
            return

        run_pantilt(args)

    finally:
        stop_camera_stream(camera_proc)
        print("[Done] pantilt_camera_view.py finished.")


if __name__ == "__main__":
    main()