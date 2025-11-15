#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Expert Auto Drive (Non-ROS, Mecanum + Safety)
-----------------------------------------------------------
- Self-contained:
    * PCA9685 class
    * RobotSavo motor wrapper (same mapping as teleop)
    * Mecanum kinematics helpers (mix_mecanum, to_duties)
- Uses safety APIs from tools/diag/sensors/api/:
    * DualToF (FR/FL) for near-field
    * Ultrasonic for front-center
    * LiDAR front-sector (optional, but supported)
- Reactive behaviour:
    * Prefer FORWARD when front is clear
    * If front blocked:
         - Try STRAFE LEFT / RIGHT
         - If no side open, try ROTATE L/R
         - If everything tight for a while, BACKUP then STOPPED
    * LiDAR also used to SLOW forward speed as robot approaches obstacles.

Author: Robot Savo
"""

import sys
import os
import time
import math
import argparse
import subprocess
from typing import Optional

import smbus

# -------------------------------------------------------------------
# Ensure project root (Savo_Pi) is on sys.path so "import tools..." works
# -------------------------------------------------------------------
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from tools.diag.sensors.api.vl53_dual_api import DualToF
from tools.diag.sensors.api.ultrasonic_api import read_ultrasonic_cm
from tools.diag.sensors.api.lidar_api import LidarSafetyAPI, RPLidarException


# ===================================================================
# PCA9685 (same style as your teleop)
# ===================================================================
class PCA9685:
    __SUBADR1 = 0x02
    __SUBADR2 = 0x03
    __SUBADR3 = 0x04
    __MODE1   = 0x00
    __PRESCALE = 0xFE
    __LED0_ON_L  = 0x06
    __LED0_ON_H  = 0x07
    __LED0_OFF_L = 0x08
    __LED0_OFF_H = 0x09
    __ALLLED_ON_L  = 0xFA
    __ALLLED_ON_H  = 0xFB
    __ALLLED_OFF_L = 0xFC
    __ALLLED_OFF_H = 0xFD

    def __init__(self, bus: int = 1, address: int = 0x40, debug: bool = False):
        self.busno = bus
        self.bus = smbus.SMBus(bus)
        self.address = address
        self.debug = debug
        self.write(self.__MODE1, 0x00)

    def write(self, reg: int, value: int) -> None:
        self.bus.write_byte_data(self.address, reg, value & 0xFF)

    def read(self, reg: int) -> int:
        return self.bus.read_byte_data(self.address, reg)

    def set_pwm_freq(self, freq: float) -> None:
        """Set PWM frequency in Hz (motor-friendly 50–1000 Hz)."""
        prescaleval = 25_000_000.0 / 4096.0 / float(freq) - 1.0
        prescale = int(math.floor(prescaleval + 0.5))
        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10          # sleep
        self.write(self.__MODE1, newmode)
        self.write(self.__PRESCALE, prescale)
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)   # restart

    def set_pwm(self, channel: int, on: int, off: int) -> None:
        base = self.__LED0_ON_L + 4 * channel
        self.write(base + 0, on & 0xFF)
        self.write(base + 1, (on >> 8) & 0x0F)
        self.write(base + 2, off & 0xFF)
        self.write(base + 3, (off >> 8) & 0x0F)

    def set_motor_pwm(self, channel: int, duty: int) -> None:
        """Set duty 0..4095 on a channel."""
        if duty < 0:
            duty = 0
        if duty > 4095:
            duty = 4095
        self.set_pwm(channel, 0, duty)

    def close(self) -> None:
        self.bus.close()


# ===================================================================
# Robot motor wrapper (same mapping as teleop)
# ===================================================================
class RobotSavo:
    """
    Channel map (LOCKED):
      FL (front-left)  : (0,1)
      RL (rear-left)   : (3,2)
      FR (front-right) : (6,7)
      RR (rear-right)  : (4,5)
    """
    def __init__(self, *, i2c_bus: int, addr: int, pwm_freq: float,
                 inv=(+1, +1, +1, +1), quench_ms: int = 18):
        self.pwm = PCA9685(bus=i2c_bus, address=addr, debug=False)
        self.pwm.set_pwm_freq(pwm_freq)
        self.FL_INV, self.RL_INV, self.FR_INV, self.RR_INV = inv
        self.quench_ms = max(0, int(quench_ms))
        self._last_sign = {'fl': 0, 'rl': 0, 'fr': 0, 'rr': 0}

    # low-level wheel writers (positive = IN_hi on 2nd channel of each pair)
    def _wheel_fl(self, d):
        if   d > 0:  self.pwm.set_motor_pwm(0, 0);   self.pwm.set_motor_pwm(1, d)
        elif d < 0:  self.pwm.set_motor_pwm(1, 0);   self.pwm.set_motor_pwm(0, -d)
        else:        self.pwm.set_motor_pwm(0, 4095); self.pwm.set_motor_pwm(1, 4095)

    def _wheel_rl(self, d):
        if   d > 0:  self.pwm.set_motor_pwm(3, 0);   self.pwm.set_motor_pwm(2, d)
        elif d < 0:  self.pwm.set_motor_pwm(2, 0);   self.pwm.set_motor_pwm(3, -d)
        else:        self.pwm.set_motor_pwm(2, 4095); self.pwm.set_motor_pwm(3, 4095)

    def _wheel_fr(self, d):
        if   d > 0:  self.pwm.set_motor_pwm(6, 0);   self.pwm.set_motor_pwm(7, d)
        elif d < 0:  self.pwm.set_motor_pwm(7, 0);   self.pwm.set_motor_pwm(6, -d)
        else:        self.pwm.set_motor_pwm(6, 4095); self.pwm.set_motor_pwm(7, 4095)

    def _wheel_rr(self, d):
        if   d > 0:  self.pwm.set_motor_pwm(4, 0);   self.pwm.set_motor_pwm(5, d)
        elif d < 0:  self.pwm.set_motor_pwm(5, 0);   self.pwm.set_motor_pwm(4, -d)
        else:        self.pwm.set_motor_pwm(4, 4095); self.pwm.set_motor_pwm(5, 4095)

    @staticmethod
    def _clamp4(d1, d2, d3, d4):
        def c(v): return 4095 if v > 4095 else (-4095 if v < -4095 else int(v))
        return c(d1), c(d2), c(d3), c(d4)

    def _apply_quench(self, name: str, prev_sign: int, new_val: int, fn) -> int:
        new_sign = 0 if new_val == 0 else (1 if new_val > 0 else -1)
        if self.quench_ms and prev_sign and new_sign and (prev_sign != new_sign):
            fn(0)
            time.sleep(self.quench_ms / 1000.0)
        fn(new_val)
        self._last_sign[name] = new_sign
        return new_sign

    def set_motor_model(self, d_fl, d_rl, d_fr, d_rr):
        d_fl *= self.FL_INV
        d_rl *= self.RL_INV
        d_fr *= self.FR_INV
        d_rr *= self.RR_INV
        d_fl, d_rl, d_fr, d_rr = self._clamp4(d_fl, d_rl, d_fr, d_rr)
        self._apply_quench('fl', self._last_sign['fl'], d_fl, self._wheel_fl)
        self._apply_quench('rl', self._last_sign['rl'], d_rl, self._wheel_rl)
        self._apply_quench('fr', self._last_sign['fr'], d_fr, self._wheel_fr)
        self._apply_quench('rr', self._last_sign['rr'], d_rr, self._wheel_rr)

    def stop(self):
        self.set_motor_model(0, 0, 0, 0)

    def close(self):
        try:
            self.stop()
        finally:
            self.pwm.close()


# ===================================================================
# Kinematics helpers (same formulas as teleop)
# ===================================================================
def mix_mecanum(vx, vy, wz, *, forward_sign, strafe_sign, rotate_sign, turn_gain):
    """
    Return normalized wheel commands (fl, rl, fr, rr) in [-1..1].
      fl =  vx - vy - w
      rl =  vx + vy - w
      fr =  vx + vy + w
      rr =  vx - vy + w
    """
    vx *= forward_sign
    vy *= strafe_sign
    w  = rotate_sign * turn_gain * wz
    fl = vx - vy - w
    rl = vx + vy - w
    fr = vx + vy + w
    rr = vx - vy + w
    m  = max(1.0, abs(fl), abs(rl), abs(fr), abs(rr))
    return fl / m, rl / m, fr / m, rr / m


def to_duties(nfl, nrl, nfr, nrr, max_duty):
    return int(nfl * max_duty), int(nrl * max_duty), int(nfr * max_duty), int(nrr * max_duty)


# ===================================================================
# Reactive planner helpers
# ===================================================================
def safe_cm(v: float) -> float:
    """Normalize distances: -1.0 means 'no valid reading' -> treat as far away."""
    return v if v >= 0.0 else 9999.0


def choose_mode(
    fr_cm: float,
    fl_cm: float,
    us_cm: Optional[float],
    lidar_near: bool,
    front_th: float,
    side_th: float,
    mode_prev: str,
    blocked_counter: int,
) -> (str, int):
    """
    Decide high-level motion mode based on near-field sensors.

    Modes:
      FORWARD    : move forward
      STRAFE_L   : strafe left
      STRAFE_R   : strafe right
      ROTATE_L   : rotate CCW
      ROTATE_R   : rotate CW
      BACKUP     : used only internally before STOPPED
      STOPPED    : fully stopped when all blocked for a while
    """
    fr = safe_cm(fr_cm)
    fl = safe_cm(fl_cm)
    us = us_cm if us_cm is not None else 9999.0

    front_blocked = (
        (us < front_th) or
        (fr < front_th and fl < front_th) or
        lidar_near
    )
    left_blocked  = (fl < side_th)
    right_blocked = (fr < side_th)

    if front_blocked and left_blocked and right_blocked:
        blocked_counter += 1
    else:
        blocked_counter = 0

    # If fully blocked for long enough, declare STOPPED.
    if blocked_counter > 40:  # e.g. ~2s at 20 Hz
        return "STOPPED", blocked_counter

    # If front clear, go forward
    if not front_blocked:
        return "FORWARD", blocked_counter

    # Front blocked: try side escapes
    if not left_blocked and right_blocked:
        return "STRAFE_L", blocked_counter
    if not right_blocked and left_blocked:
        return "STRAFE_R", blocked_counter
    if not left_blocked and not right_blocked:
        # both sides available: choose better one
        if fl > fr:
            return "STRAFE_L", blocked_counter
        else:
            return "STRAFE_R", blocked_counter

    # Sides both blocked, front blocked too -> try rotation
    if mode_prev != "ROTATE_L":
        return "ROTATE_L", blocked_counter
    else:
        return "ROTATE_R", blocked_counter


def mode_to_cmd(
    mode: str,
    v_forward: float,
    v_side: float,
    v_back: float,
    w_rotate: float,
) -> (float, float, float):
    """
    Convert mode -> (vx, vy, wz) in [-1..1].
    vx: +forward, vy: +left, wz: +CCW
    """
    if mode == "FORWARD":
        return (v_forward, 0.0, 0.0)
    if mode == "STRAFE_L":
        return (0.0, +v_side, 0.0)
    if mode == "STRAFE_R":
        return (0.0, -v_side, 0.0)
    if mode == "BACKUP":
        return (-v_back, 0.0, 0.0)
    if mode == "ROTATE_L":
        return (0.0, 0.0, +w_rotate)
    if mode == "ROTATE_R":
        return (0.0, 0.0, -w_rotate)
    return (0.0, 0.0, 0.0)  # STOPPED / unknown


# ===================================================================
# Optional: camera + face helpers (default OFF)
# ===================================================================
def start_camera_stream(host: str, port: int) -> Optional[subprocess.Popen]:
    """
    Optional camera stream to laptop (off by default).
    Enable with --enable-camera if you want.
    """
    pipeline = [
        "gst-launch-1.0",
        "libcamerasrc", "!",
        "video/x-raw,format=I420,width=1280,height=720,framerate=30/1", "!",
        "videoconvert", "!",
        "x264enc", "tune=zerolatency", "speed-preset=ultrafast", "bitrate=2000", "!",
        "rtph264pay", "config-interval=1", "pt=96", "!",
        "udpsink", f"host={host}", f"port={port}", "sync=false"
    ]
    try:
        print(f"[AutoDrive] Starting camera stream to {host}:{port} ...")
        return subprocess.Popen(
            pipeline,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception as e:
        print(f"[AutoDrive] WARNING: camera start failed: {e}", file=sys.stderr)
        return None


def start_face(face_cmd: str) -> Optional[subprocess.Popen]:
    if not face_cmd:
        return None
    try:
        print(f"[AutoDrive] Starting face UI: {face_cmd}")
        return subprocess.Popen(
            face_cmd.split(),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception as e:
        print(f"[AutoDrive] WARNING: face start failed: {e}", file=sys.stderr)
        return None


def stop_proc(proc: Optional[subprocess.Popen], name: str) -> None:
    if proc is None:
        return
    if proc.poll() is not None:
        return
    print(f"[AutoDrive] Stopping {name} ...")
    try:
        proc.terminate()
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            proc.kill()
    except Exception:
        pass


# ===================================================================
# Main auto-drive loop
# ===================================================================
def main():
    ap = argparse.ArgumentParser(description="Robot Savo — Expert Auto Drive (Non-ROS Mecanum)")
    # Motor / board
    ap.add_argument("--i2c-bus", type=int, default=1)
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=0x40)
    ap.add_argument("--pwm-freq", type=float, default=50.0)
    ap.add_argument("--max-duty", type=int, default=2500)

    ap.add_argument("--invert-fl", action="store_true")
    ap.add_argument("--invert-rl", action="store_true")
    ap.add_argument("--invert-fr", action="store_true")
    ap.add_argument("--invert-rr", action="store_true")
    ap.add_argument("--quench-ms", type=int, default=18)

    # Kinematic signs (must match your teleop)
    ap.add_argument("--forward-sign", type=int, choices=[-1, 1], default=-1)
    ap.add_argument("--strafe-sign", type=int, choices=[-1, 1], default=+1)
    ap.add_argument("--rotate-sign", type=int, choices=[-1, 1], default=+1)
    ap.add_argument("--turn-gain", type=float, default=1.0)

    # Motion magnitudes
    ap.add_argument("--v-forward", type=float, default=0.5)
    ap.add_argument("--v-side",    type=float, default=0.5)
    ap.add_argument("--v-back",    type=float, default=0.3)
    ap.add_argument("--w-rotate",  type=float, default=0.6)

    # Safety thresholds
    ap.add_argument("--front-th", type=float, default=28.0)
    ap.add_argument("--side-th",  type=float, default=22.0)
    ap.add_argument("--us-th",    type=float, default=28.0)
    ap.add_argument("--loop-hz",  type=float, default=20.0)

    # LiDAR
    ap.add_argument("--no-lidar", action="store_true")
    ap.add_argument("--lidar-th", type=float, default=0.28,
                    help="LiDAR obstacle threshold (m) for front_blocked")

    # LiDAR-based speed reduction
    ap.add_argument("--lidar-slow-dist", type=float, default=0.80,
                    help="Distance (m) where we start to slow forward motion")
    ap.add_argument("--min-speed-scale", type=float, default=0.25,
                    help="Minimum forward speed fraction near obstacle")

    # Camera & face (optional)
    ap.add_argument("--enable-camera", action="store_true")
    ap.add_argument("--cam-host", type=str, default="192.168.1.100")
    ap.add_argument("--cam-port", type=int, default=5000)
    ap.add_argument("--face-cmd", type=str, default="")

    args = ap.parse_args()

    # Build inversion vector
    inv = (
        -1 if args.invert_fl else +1,
        -1 if args.invert_rl else +1,
        -1 if args.invert_fr else +1,
        -1 if args.invert_rr else +1,
    )
    max_duty = max(0, min(4095, args.max_duty))

    # Init robot
    bot = RobotSavo(
        i2c_bus=args.i2c_bus,
        addr=args.addr,
        pwm_freq=args.pwm_freq,
        inv=inv,
        quench_ms=args.quench_ms,
    )

    # Sensors
    print("[AutoDrive] Initializing DualToF ...")
    tof = DualToF(rate_hz=args.loop_hz, median=3, threshold_cm=args.front_th)

    us_trig, us_echo = 27, 22
    print(f"[AutoDrive] Ultrasonic TRIG={us_trig} ECHO={us_echo}")

    lidar = None
    if not args.no_lidar:
        try:
            print("[AutoDrive] Initializing LiDAR Safety API ...")
            lidar = LidarSafetyAPI(
                port="/dev/ttyUSB0",
                baudrate=115200,
                timeout=2.0,
                motor_pwm=None,
                angle_min=-35.0,
                angle_max=35.0,
                range_min=0.05,
                range_max=2.0,
                obst_threshold=args.lidar_th,
                debounce=3,
                hyst=0.03,
                min_len=120,
                verbose=False,
            )
            lidar.start()
            print("[AutoDrive] LiDAR ready.")
        except Exception as e:
            print(f"[AutoDrive] WARNING: LiDAR init failed: {e}", file=sys.stderr)
            lidar = None

    # Camera & face (optional)
    cam_proc = start_camera_stream(args.cam_host, args.cam_port) if args.enable_camera else None
    face_proc = start_face(args.face_cmd) if args.face_cmd else None

    # Reactive loop state
    mode = "STOPPED"
    blocked_counter = 0
    loop_period = 1.0 / max(5.0, float(args.loop_hz))

    print("[AutoDrive] Entering main loop. Ctrl+C to stop.")

    try:
        while True:
            t0 = time.time()

            # ToF
            fr_raw, fr_f, fl_raw, fl_f, alert_str, tof_near = tof.read()

            # Ultrasonic
            us_cm = read_ultrasonic_cm(
                trig_pin=us_trig,
                echo_pin=us_echo,
                factory="lgpio",
                samples=5,
            )
            us_near = (us_cm is not None and us_cm < args.us_th)

            # LiDAR
            lidar_near = False
            lidar_min_m: Optional[float] = None
            if lidar is not None:
                try:
                    r = lidar.poll()
                    lidar_near = r.obstacle
                    lidar_min_m = r.min_dist_m
                except RPLidarException as e:
                    print(f"[AutoDrive] LiDAR error: {e}", file=sys.stderr)
                except Exception:
                    pass

            # Decide mode
            prev_mode = mode
            mode, blocked_counter = choose_mode(
                fr_cm=fr_f,
                fl_cm=fl_f,
                us_cm=us_cm,
                lidar_near=lidar_near,
                front_th=args.front_th,
                side_th=args.side_th,
                mode_prev=prev_mode,
                blocked_counter=blocked_counter,
            )

            # If fully blocked for long time, do small backup then stop.
            if mode == "STOPPED" and blocked_counter > 40:
                vx_b, vy_b, wz_b = mode_to_cmd(
                    "BACKUP",
                    args.v_forward,
                    args.v_side,
                    args.v_back,
                    args.w_rotate,
                )
                nfl, nrl, nfr, nrr = mix_mecanum(
                    vx_b, vy_b, wz_b,
                    forward_sign=args.forward_sign,
                    strafe_sign=args.strafe_sign,
                    rotate_sign=args.rotate_sign,
                    turn_gain=args.turn_gain,
                )
                dfl, drl, dfr, drr = to_duties(nfl, nrl, nfr, nrr, max_duty)
                bot.set_motor_model(dfl, drl, dfr, drr)
                time.sleep(0.3)
                mode = "STOPPED"

            # Convert mode -> (vx,vy,wz)
            vx, vy, wz = mode_to_cmd(
                mode,
                args.v_forward,
                args.v_side,
                args.v_back,
                args.w_rotate,
            )

            # LiDAR-based forward speed reduction
            if mode == "FORWARD" and lidar_min_m is not None:
                stop_d = args.lidar_th          # hard safety threshold
                slow_d = args.lidar_slow_dist   # start slowing distance
                if lidar_min_m <= stop_d:
                    vx = 0.0
                elif lidar_min_m < slow_d:
                    t = (lidar_min_m - stop_d) / max(1e-3, (slow_d - stop_d))  # 0..1
                    scale = max(args.min_speed_scale, min(1.0, t))
                    vx *= scale

            nfl, nrl, nfr, nrr = mix_mecanum(
                vx, vy, wz,
                forward_sign=args.forward_sign,
                strafe_sign=args.strafe_sign,
                rotate_sign=args.rotate_sign,
                turn_gain=args.turn_gain,
            )
            dfl, drl, dfr, drr = to_duties(nfl, nrl, nfr, nrr, max_duty)
            bot.set_motor_model(dfl, drl, dfr, drr)

            # Debug
            print(
                f"[AutoDrive] mode={mode:9s}  "
                f"FR_f={fr_f:5.1f}cm  FL_f={fl_f:5.1f}cm  "
                f"US={('%.1f cm' % us_cm) if us_cm is not None else 'None':>8s}  "
                f"ToF_near={tof_near}  US_near={us_near}  LiDAR_near={lidar_near}  "
                f"LiDAR_min={('%4.2f m' % lidar_min_m) if lidar_min_m is not None else 'None':>8s}  "
                f"blocked_cnt={blocked_counter}"
            )

            # Timing
            dt = loop_period - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)

    except KeyboardInterrupt:
        print("\n[AutoDrive] Ctrl+C — shutting down.")
    finally:
        print("[AutoDrive] Stopping motors & cleaning up ...")
        try:
            bot.stop()
            bot.close()
        except Exception:
            pass
        if lidar is not None:
            try:
                lidar.stop()
            except Exception:
                pass
        stop_proc(cam_proc, "camera")
        stop_proc(face_proc, "face")
        print("[AutoDrive] Done.")


if __name__ == "__main__":
    main()
