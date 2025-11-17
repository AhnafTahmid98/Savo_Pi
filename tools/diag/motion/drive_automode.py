#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Expert Auto Drive (Mecanum + Full 360° Safety)
--------------------------------------------------------------------
- Self-contained:
    * PCA9685 class
    * RobotSavo motor wrapper 
    * Mecanum kinematics helpers (mix_mecanum, to_duties)
- Uses safety sensors:
    * DualToF (FR/FL) for near-field front
    * Ultrasonic (front-center) via gpiozero DistanceSensor
    * LiDAR 360° bubble via LidarSafetyAPI
- Behaviour:
    * Prefer FORWARD when front is clear.
    * If front blocked:
         - Try STRAFE LEFT / RIGHT.
         - If both sides blocked → rely on LiDAR to see if BACK is safe.
             - If back free → short BACKUP.
             - If back also blocked → ROTATE in place.
    * LiDAR roles:
         - PRIMARY front window (−35..+35°) for early front slow-down.
         - 360° sectors FRONT / RIGHT / BACK / LEFT as safety bubble:
             * Avoid backing into close BACK obstacle.
             * Avoid strafing into close LEFT/RIGHT obstacles.
             * Emergency STOP only if something VERY close in FRONT
               (global min < emerg_lidar_m and |angle| <= 60°).
         - Speed reduction in ALL directions:
             * vx>0  → slow using FRONT sector
             * vx<0  → slow using BACK sector
             * vy>0  → slow using LEFT sector
             * vy<0  → slow using RIGHT sector
             * wz≠0 → slow using global LiDAR min
    * EMERGENCY STOP if something VERY close in front (LiDAR/ToF/US),
      followed by clear escape motion:
         - reverse + turn away from closer ToF side.

Face UI:
    - Starts tools/diag/ui/face.py by default on the DSI.
    - Disable with:  --no-face

Camera streaming (optional):
    - Enable with:  --enable-camera --cam-host <laptop-ip> --cam-port 5000
"""

import sys
import os
import time
import math
import argparse
import subprocess
from typing import Optional, Tuple

import smbus
from gpiozero import DistanceSensor
from gpiozero.pins.lgpio import LGPIOFactory

# -------------------------------------------------------------------
# Project root (Savo_Pi)
# -------------------------------------------------------------------
PROJECT_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..")
)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from tools.diag.sensors.api.vl53_dual_api import DualToF
from tools.diag.sensors.api.lidar_api import LidarSafetyAPI, RPLidarException


# ===================================================================
# PCA9685 driver
# ===================================================================
class PCA9685:
    __SUBADR1 = 0x02
    __SUBADR2 = 0x03
    __SUBADR3 = 0x04
    __MODE1 = 0x00
    __PRESCALE = 0xFE
    __LED0_ON_L = 0x06
    __LED0_ON_H = 0x07
    __LED0_OFF_L = 0x08
    __LED0_OFF_H = 0x09
    __ALLLED_ON_L = 0xFA
    __ALLLED_ON_H = 0xFB
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
# Robot motor wrapper
# ===================================================================
class RobotSavo:
    """
    Channel map (LOCKED):
      FL (front-left)  : (0,1)
      RL (rear-left)   : (3,2)
      FR (front-right) : (6,7)
      RR (rear-right)  : (4,5)

    Positive duty => "forward" according to your wiring & invert flags.
    """

    def __init__(
        self,
        *,
        i2c_bus: int,
        addr: int,
        pwm_freq: float,
        inv: Tuple[int, int, int, int] = (+1, +1, +1, +1),
        quench_ms: int = 18,
    ):
        self.pwm = PCA9685(bus=i2c_bus, address=addr, debug=False)
        self.pwm.set_pwm_freq(pwm_freq)
        self.FL_INV, self.RL_INV, self.FR_INV, self.RR_INV = inv
        self.quench_ms = max(0, int(quench_ms))
        self._last_sign = {"fl": 0, "rl": 0, "fr": 0, "rr": 0}

    # low-level wheel writers (positive = IN_hi on 2nd channel of each pair)
    def _wheel_fl(self, d: int) -> None:
        if d > 0:
            self.pwm.set_motor_pwm(0, 0)
            self.pwm.set_motor_pwm(1, d)
        elif d < 0:
            self.pwm.set_motor_pwm(1, 0)
            self.pwm.set_motor_pwm(0, -d)
        else:
            self.pwm.set_motor_pwm(0, 4095)
            self.pwm.set_motor_pwm(1, 4095)

    def _wheel_rl(self, d: int) -> None:
        if d > 0:
            self.pwm.set_motor_pwm(3, 0)
            self.pwm.set_motor_pwm(2, d)
        elif d < 0:
            self.pwm.set_motor_pwm(2, 0)
            self.pwm.set_motor_pwm(3, -d)
        else:
            self.pwm.set_motor_pwm(2, 4095)
            self.pwm.set_motor_pwm(3, 4095)

    def _wheel_fr(self, d: int) -> None:
        if d > 0:
            self.pwm.set_motor_pwm(6, 0)
            self.pwm.set_motor_pwm(7, d)
        elif d < 0:
            self.pwm.set_motor_pwm(7, 0)
            self.pwm.set_motor_pwm(6, -d)
        else:
            self.pwm.set_motor_pwm(6, 4095)
            self.pwm.set_motor_pwm(7, 4095)

    def _wheel_rr(self, d: int) -> None:
        if d > 0:
            self.pwm.set_motor_pwm(4, 0)
            self.pwm.set_motor_pwm(5, d)
        elif d < 0:
            self.pwm.set_motor_pwm(5, 0)
            self.pwm.set_motor_pwm(4, -d)
        else:
            self.pwm.set_motor_pwm(4, 4095)
            self.pwm.set_motor_pwm(5, 4095)

    @staticmethod
    def _clamp4(d1: int, d2: int, d3: int, d4: int) -> Tuple[int, int, int, int]:
        def c(v: int) -> int:
            return 4095 if v > 4095 else (-4095 if v < -4095 else int(v))
        return c(d1), c(d2), c(d3), c(d4)

    def _apply_quench(self, name: str, prev_sign: int, new_val: int, fn) -> int:
        new_sign = 0 if new_val == 0 else (1 if new_val > 0 else -1)
        if (
            self.quench_ms
            and prev_sign
            and new_sign
            and (prev_sign != new_sign)
        ):
            fn(0)
            time.sleep(self.quench_ms / 1000.0)
        fn(new_val)
        self._last_sign[name] = new_sign
        return new_sign

    def set_motor_model(self, d_fl: int, d_rl: int, d_fr: int, d_rr: int) -> None:
        d_fl *= self.FL_INV
        d_rl *= self.RL_INV
        d_fr *= self.FR_INV
        d_rr *= self.RR_INV
        d_fl, d_rl, d_fr, d_rr = self._clamp4(d_fl, d_rl, d_fr, d_rr)
        self._apply_quench("fl", self._last_sign["fl"], d_fl, self._wheel_fl)
        self._apply_quench("rl", self._last_sign["rl"], d_rl, self._wheel_rl)
        self._apply_quench("fr", self._last_sign["fr"], d_fr, self._wheel_fr)
        self._apply_quench("rr", self._last_sign["rr"], d_rr, self._wheel_rr)

    def stop(self) -> None:
        self.set_motor_model(0, 0, 0, 0)

    def close(self) -> None:
        try:
            self.stop()
        finally:
            self.pwm.close()


# ===================================================================
# Kinematics helpers
# ===================================================================
def mix_mecanum(
    vx: float,
    vy: float,
    wz: float,
    *,
    forward_sign: int,
    strafe_sign: int,
    rotate_sign: int,
    turn_gain: float,
) -> Tuple[float, float, float, float]:
    """
    Return normalized wheel commands (fl, rl, fr, rr) in [-1..1].
      fl =  vx - vy - w
      rl =  vx + vy - w
      fr =  vx + vy + w
      rr =  vx - vy + w
    """
    vx *= forward_sign
    vy *= strafe_sign
    w = rotate_sign * turn_gain * wz
    fl = vx - vy - w
    rl = vx + vy - w
    fr = vx + vy + w
    rr = vx - vy + w
    m = max(1.0, abs(fl), abs(rl), abs(fr), abs(rr))
    return fl / m, rl / m, fr / m, rr / m


def to_duties(
    nfl: float, nrl: float, nfr: float, nrr: float, max_duty: int
) -> Tuple[int, int, int, int]:
    return (
        int(nfl * max_duty),
        int(nrl * max_duty),
        int(nfr * max_duty),
        int(nrr * max_duty),
    )


# ===================================================================
# Reactive planner helpers
# ===================================================================
def safe_cm(v: Optional[float]) -> float:
    """Normalize distances: >=0.0 is valid cm; None / negative => treat as very far."""
    if v is None or v < 0.0:
        return 9999.0
    return v


def choose_mode(
    front_blocked: bool,
    left_blocked: bool,
    right_blocked: bool,
    back_blocked: bool,
    mode_prev: str,
    blocked_counter: int,
) -> Tuple[str, int]:
    """
    Decide high-level motion mode based on near-field + LiDAR bubbles.

    Modes:
      FORWARD    : move forward
      STRAFE_L   : strafe left
      STRAFE_R   : strafe right
      ROTATE_L   : rotate CCW
      ROTATE_R   : rotate CW
      BACKUP     : short reverse (only if back not blocked)
      STOPPED    : fully stopped when boxed for a while
    """
    # Boxed: front+back+both sides
    if front_blocked and back_blocked and left_blocked and right_blocked:
        blocked_counter += 1
    else:
        blocked_counter = 0

    if blocked_counter > 40:  # ~2s at 20 Hz
        return "STOPPED", blocked_counter

    # If front clear → always go forward
    if not front_blocked:
        return "FORWARD", blocked_counter

    # Front blocked: try lateral motion
    if not left_blocked and right_blocked:
        return "STRAFE_L", blocked_counter
    if not right_blocked and left_blocked:
        return "STRAFE_R", blocked_counter
    if not left_blocked and not right_blocked:
        # both sides available → prefer left (could be improved with LiDAR distance)
        return "STRAFE_L", blocked_counter

    # Front + both sides blocked, back free → consider BACKUP
    if front_blocked and left_blocked and right_blocked and not back_blocked:
        return "BACKUP", blocked_counter

    # Everything nasty → rotate in place to search
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
) -> Tuple[float, float, float]:
    """
    Convert mode -> (vx, vy, wz) in [-1..1].
    vx: +forward, vy: +left, wz: +CCW
    """
    if mode == "FORWARD":
        return v_forward, 0.0, 0.0
    if mode == "STRAFE_L":
        return 0.0, +v_side, 0.0
    if mode == "STRAFE_R":
        return 0.0, -v_side, 0.0
    if mode == "BACKUP":
        return -v_back, 0.0, 0.0
    if mode == "ROTATE_L":
        return 0.0, 0.0, +w_rotate
    if mode == "ROTATE_R":
        return 0.0, 0.0, -w_rotate
    return 0.0, 0.0, 0.0  # STOPPED / unknown


def lidar_slow_scale(
    dist_m: Optional[float],
    stop_d: float,
    slow_d: float,
    min_scale: float,
) -> float:
    """
    Compute a scale factor in [0..1] for a given direction:
      - dist <= stop_d       → 0.0  (stop)
      - dist >= slow_d       → 1.0  (no slowdown)
      - between              → linear scale, clamped to [min_scale, 1.0]
    """
    if dist_m is None:
        return 1.0
    if dist_m <= stop_d:
        return 0.0
    if dist_m >= slow_d:
        return 1.0
    t = (dist_m - stop_d) / max(1e-3, (slow_d - stop_d))
    return max(min_scale, min(1.0, t))


# ===================================================================
# Ultrasonic helper — SINGLE sensor, reused
# ===================================================================
class UltrasonicReader:
    """
    Front-center ultrasonic helper:
    - Uses LGPIOFactory + DistanceSensor once.
    - Provides read_cm() method, safe for use in tight loops.
    """

    def __init__(self, trig_pin: int, echo_pin: int, max_distance_m: float = 4.0):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.max_distance_m = max_distance_m
        self.sensor: Optional[DistanceSensor] = None
        self._init_sensor()

    def _init_sensor(self) -> None:
        try:
            factory = LGPIOFactory()
            self.sensor = DistanceSensor(
                echo=self.echo_pin,
                trigger=self.trig_pin,
                max_distance=self.max_distance_m,
                queue_len=3,
                pin_factory=factory,
            )
            print(
                f"[AutoDrive] Ultrasonic sensor ready "
                f"(TRIG={self.trig_pin}, ECHO={self.echo_pin}, max={self.max_distance_m} m)"
            )
        except Exception as e:
            print(
                f"[AutoDrive] WARNING: ultrasonic init failed: {e}",
                file=sys.stderr,
            )
            self.sensor = None

    def read_cm(self) -> Optional[float]:
        if self.sensor is None:
            return None
        try:
            d = self.sensor.distance  # 0.0..1.0
            if d is None:
                return None
            dist_cm = d * 100.0
            if dist_cm <= 0.0 or dist_cm > 400.0:
                return None
            return dist_cm
        except Exception as e:
            print(
                f"[AutoDrive] WARNING: ultrasonic read error: {e}",
                file=sys.stderr,
            )
            return None

    def close(self) -> None:
        if self.sensor is not None:
            try:
                self.sensor.close()
            except Exception:
                pass
            self.sensor = None


# ===================================================================
# Camera & face helpers
# ===================================================================

def start_camera_stream(host: str, port: int) -> Optional[subprocess.Popen]:
    pipeline = [
        "gst-launch-1.0",
        "libcamerasrc",
        "!",
        "video/x-raw,format=I420,width=1280,height=720,framerate=30/1",
        "!",
        "videoconvert",
        "!",
        "x264enc",
        "tune=zerolatency",
        "speed-preset=superfast",   # was ultrafast
        "bitrate=6000",             # was 2000 → more bits = better quality
        "key-int-max=30",
        "!",
        "rtph264pay",
        "config-interval=1",
        "pt=96",
        "!",
        "udpsink",
        f"host={host}",
        f"port={port}",
        "sync=false",
    ]
    try:
        print(f"[AutoDrive] Starting camera stream to {host}:{port} ...")
        env = os.environ.copy()
        env["XDG_RUNTIME_DIR"] = "/run/user/1000"
        env["WAYLAND_DISPLAY"] = "wayland-1"
        env["GST_PLUGIN_PATH"] = "/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0"
        env["LD_LIBRARY_PATH"] = "/usr/local/lib/aarch64-linux-gnu:/usr/local/lib"
        return subprocess.Popen(
            pipeline,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=env,
        )
    except Exception as e:
        print(
            f"[AutoDrive] WARNING: camera start failed: {e}",
            file=sys.stderr,
        )
        return None

def start_face() -> Optional[subprocess.Popen]:
    """
    Start the emotion-based face UI on the DSI display.

    Expected script:
        PROJECT_ROOT/tools/diag/ui/face.py
    """
    face_script = os.path.join(PROJECT_ROOT, "tools", "diag", "ui", "face.py")
    if not os.path.exists(face_script):
        print(
            f"[AutoDrive] WARNING: face script not found: {face_script}",
            file=sys.stderr,
        )
        return None

    try:
        print(f"[AutoDrive] Starting face UI: {face_script}")
        return subprocess.Popen(
            ["python3", face_script],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception as e:
        print(
            f"[AutoDrive] WARNING: face start failed: {e}",
            file=sys.stderr,
        )
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
def main() -> None:
    ap = argparse.ArgumentParser(
        description="Robot Savo — Expert Auto Drive (Non-ROS Mecanum + 360° LiDAR)"
    )
    # Motor / board
    ap.add_argument("--i2c-bus", type=int, default=1)
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=0x40)
    ap.add_argument("--pwm-freq", type=float, default=50.0)
    ap.add_argument("--max-duty", type=int, default=3200)

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
    ap.add_argument("--v-forward", type=float, default=0.4)
    ap.add_argument("--v-side", type=float, default=0.4)
    ap.add_argument("--v-back", type=float, default=0.3)
    ap.add_argument("--w-rotate", type=float, default=0.5)

    # Safety thresholds (near-field ToF + Ultrasonic)
    ap.add_argument(
        "--front-th-fr",
        type=float,
        default=40.0,
        help="Front block threshold for FRONT-RIGHT ToF (cm)",
    )
    ap.add_argument(
        "--front-th-fl",
        type=float,
        default=40.0,
        help="Front block threshold for FRONT-LEFT ToF (cm)",
    )
    ap.add_argument(
        "--side-th-fr",
        type=float,
        default=40.0,
        help="Side block threshold for FRONT-RIGHT ToF (cm)",
    )
    ap.add_argument(
        "--side-th-fl",
        type=float,
        default=40.0,
        help="Side block threshold for FRONT-LEFT ToF (cm)",
    )
    ap.add_argument(
        "--us-th",
        type=float,
        default=40.0,
        help="Near-field threshold for ultrasonic (cm)",
    )
    ap.add_argument("--loop-hz", type=float, default=20.0)

    # EMERGENCY thresholds (hard stop — FRONT only)
    ap.add_argument(
        "--emerg-front-cm",
        type=float,
        default=20.0,
        help="Emergency stop if ToF/US < this (cm) in front",
    )
    ap.add_argument(
        "--emerg-lidar-m",
        type=float,
        default=0.25,
        help="Emergency stop if LiDAR < this (m) AND |angle| <= 60°",
    )

    # LiDAR
    ap.add_argument("--no-lidar", action="store_true")
    ap.add_argument(
        "--lidar-th",
        type=float,
        default=0.40,   # was 0.50 → earlier stop & stronger slowdown zone
        help="LiDAR obstacle threshold (m) for ALL sectors (front/right/back/left)",
    )

    # LiDAR-based speed reduction (all directions)
    ap.add_argument(
        "--lidar-slow-dist",
        type=float,
        default=1.20,   # was 1.00 → start slowing earlier
        help="Distance (m) where we start to slow motion in that direction",
    )
    ap.add_argument(
        "--min-speed-scale",
        type=float,
        default=0.30,   # was 0.40 → allow slower crawl near obstacles
        help="Minimum speed fraction near obstacle (0.40 → 40%)",
    )

    # Camera & face (face ON by default)
    ap.add_argument("--enable-camera", action="store_true")
    ap.add_argument("--cam-host", type=str, default="192.168.1.100")
    ap.add_argument("--cam-port", type=int, default=5000)
    ap.add_argument(
        "--no-face",
        action="store_true",
        help="Disable starting the face UI on the DSI (defaults to ON)",
    )

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
    # Use the smaller front-threshold as the internal DualToF alert threshold
    tof_threshold = min(args.front_th_fr, args.front_th_fl)
    tof = DualToF(rate_hz=args.loop_hz, median=3, threshold_cm=tof_threshold)

    us_trig, us_echo = 27, 22  # locked mapping (TRIG=27, ECHO=22)
    print(f"[AutoDrive] Ultrasonic TRIG={us_trig} ECHO={us_echo}")
    us_reader = UltrasonicReader(us_trig, us_echo)

    lidar: Optional[LidarSafetyAPI] = None
    if not args.no_lidar:
        try:
            print("[AutoDrive] Initializing LiDAR Safety API (front PRIMARY + 360° sectors) ...")
            lidar = LidarSafetyAPI(
                port="/dev/ttyUSB0",
                baudrate=115200,
                timeout=2.0,
                motor_pwm=None,
                # PRIMARY sector: narrow front window for early detection
                angle_min=-35.0,
                angle_max=35.0,
                range_min=0.05,
                range_max=2.0,
                obst_threshold=args.lidar_th,  # threshold for PRIMARY sector
                debounce=3,
                hyst=0.03,
                min_len=120,
                verbose=False,
            )
            lidar.start()
            print("[AutoDrive] LiDAR ready.")
        except Exception as e:
            print(
                f"[AutoDrive] WARNING: LiDAR init failed: {e}",
                file=sys.stderr,
            )
            lidar = None

    # Camera & face
    cam_proc = (
        start_camera_stream(args.cam_host, args.cam_port)
        if args.enable_camera
        else None
    )
    face_proc = None if args.no_face else start_face()

    # Reactive loop state
    mode = "STOPPED"
    blocked_counter = 0
    loop_period = 1.0 / max(5.0, float(args.loop_hz))

    lidar_th_cm = args.lidar_th * 100.0

    print("[AutoDrive] Entering main loop. Ctrl+C to stop.")

    try:
        while True:
            t0 = time.time()

            # ---------- ToF ----------
            fr_raw, fr_f, fl_raw, fl_f, alert_str, tof_near = tof.read()

            # ---------- Ultrasonic ----------
            us_cm = us_reader.read_cm()
            us_near = us_cm is not None and us_cm < args.us_th

            # ---------- LiDAR (360° bubble) ----------
            lidar_near_primary = False
            lidar_min_m: Optional[float] = None
            lidar_min_ang_deg: Optional[float] = None

            front_lidar_cm = right_lidar_cm = back_lidar_cm = left_lidar_cm = None
            front_lidar_m = right_lidar_m = back_lidar_m = left_lidar_m = None

            if lidar is not None:
                try:
                    r = lidar.poll()
                    # PRIMARY sector (front window)
                    lidar_near_primary = r.obstacle
                    # PRIMARY min
                    prim_min_m = r.min_dist_m
                    prim_min_ang = r.min_angle_deg

                    # 360° sectors
                    if r.front_min_m is not None:
                        front_lidar_m = r.front_min_m
                        front_lidar_cm = r.front_min_m * 100.0
                    if r.right_min_m is not None:
                        right_lidar_m = r.right_min_m
                        right_lidar_cm = r.right_min_m * 100.0
                    if r.back_min_m is not None:
                        back_lidar_m = r.back_min_m
                        back_lidar_cm = r.back_min_m * 100.0
                    if r.left_min_m is not None:
                        left_lidar_m = r.left_min_m
                        left_lidar_cm = r.left_min_m * 100.0

                    # Global min: anywhere in 360°
                    if r.global_min_m is not None:
                        lidar_min_m = r.global_min_m
                        lidar_min_ang_deg = r.global_min_angle_deg
                    else:
                        lidar_min_m = prim_min_m
                        lidar_min_ang_deg = prim_min_ang

                except RPLidarException as e:
                    print(f"[AutoDrive] LiDAR error: {e}", file=sys.stderr)
                except Exception:
                    pass

            # ---------- Combined near-field (ToF + US) ----------
            fr_cm = safe_cm(fr_f)
            fl_cm = safe_cm(fl_f)
            us_cm_safe = us_cm if us_cm is not None else 9999.0

            # Front near-field: any of US, FR, FL under their own thresholds
            front_near_nf = (
                (us_cm_safe < args.us_th)
                or (fr_cm < args.front_th_fr)
                or (fl_cm < args.front_th_fl)
            )
            # Side near-field per sensor
            left_near_nf = fl_cm < args.side_th_fl
            right_near_nf = fr_cm < args.side_th_fr

            # ---------- LiDAR sector-based blocked flags ----------
            front_block_lidar = front_lidar_cm is not None and front_lidar_cm < lidar_th_cm
            right_block_lidar = right_lidar_cm is not None and right_lidar_cm < lidar_th_cm
            back_block_lidar = back_lidar_cm is not None and back_lidar_cm < lidar_th_cm
            left_block_lidar = left_lidar_cm is not None and left_lidar_cm < lidar_th_cm

            # ------------------------------------------------------------------
            # Combine ToF/US + LiDAR for each direction
            # ------------------------------------------------------------------
            front_blocked = front_near_nf or front_block_lidar
            left_blocked = left_near_nf or left_block_lidar
            right_blocked = right_near_nf or right_block_lidar
            back_blocked = back_block_lidar  # only LiDAR watches back

            # ---------- EMERGENCY STOP + ESCAPE (FRONT ONLY) ----------
            emerg = False
            emerg_reason = ""
            front_emerg = False

            # ToF + Ultrasonic in front
            if us_cm is not None and us_cm < args.emerg_front_cm:
                emerg = True
                front_emerg = True
                emerg_reason = f"US {us_cm:.1f}cm"

            if fr_f >= 0.0 and fr_f < args.emerg_front_cm:
                emerg = True
                front_emerg = True
                emerg_reason = (
                    (emerg_reason + " + ") if emerg_reason else ""
                ) + f"FR {fr_f:.1f}cm"

            if fl_f >= 0.0 and fl_f < args.emerg_front_cm:
                emerg = True
                front_emerg = True
                emerg_reason = (
                    (emerg_reason + " + ") if emerg_reason else ""
                ) + f"FL {fl_f:.1f}cm"

            # LiDAR emergency ONLY if obstacle is in front sector (|angle| <= 60°)
            if (
                lidar_min_m is not None
                and lidar_min_ang_deg is not None
                and abs(lidar_min_ang_deg) <= 60.0
                and lidar_min_m < args.emerg_lidar_m
            ):
                emerg = True
                front_emerg = True
                emerg_reason = (
                    (emerg_reason + " + ") if emerg_reason else ""
                ) + f"LiDAR {lidar_min_m:.2f}m @ {lidar_min_ang_deg:.1f}°"

            if emerg:
                # Hard stop first
                bot.set_motor_model(0, 0, 0, 0)
                print(
                    f"[AutoDrive][EMERG] HARD STOP  reason={emerg_reason}  "
                    f"FR_f={(fr_f if fr_f >= 0.0 else -1.0):.1f}cm "
                    f"FL_f={(fl_f if fl_f >= 0.0 else -1.0):.1f}cm "
                    f"US={('%.1f cm' % us_cm) if us_cm is not None else 'None'} "
                    f"LiDAR_min={('%4.2f m' % lidar_min_m) if lidar_min_m is not None else 'None'} "
                    f"LiDAR_ang={('%5.1f deg' % lidar_min_ang_deg) if lidar_min_ang_deg is not None else 'None'}"
                )
                time.sleep(0.1)

                if front_emerg:
                    # FRONT-type escape: reverse + turn away from closer side
                    vx_b = -max(0.4, args.v_back)
                    vy_b = 0.0

                    fr_close = fr_f if fr_f >= 0.0 else 9999.0
                    fl_close = fl_f if fl_f >= 0.0 else 9999.0

                    if fr_close + 1.0 < fl_close:
                        wz_b = +max(0.6, args.w_rotate)
                        turn_txt = "CCW (left)"
                    elif fl_close + 1.0 < fr_close:
                        wz_b = -max(0.6, args.w_rotate)
                        turn_txt = "CW (right)"
                    else:
                        wz_b = 0.0
                        turn_txt = "straight back"

                    print(
                        f"[AutoDrive][EMERG] ESCAPE: vx={vx_b:.2f}, wz={wz_b:.2f}  "
                        f"({turn_txt}), dur=0.8s"
                    )

                    nfl, nrl, nfr, nrr = mix_mecanum(
                        vx_b,
                        vy_b,
                        wz_b,
                        forward_sign=args.forward_sign,
                        strafe_sign=args.strafe_sign,
                        rotate_sign=args.rotate_sign,
                        turn_gain=args.turn_gain,
                    )
                    dfl, drl, dfr, drr = to_duties(nfl, nrl, nfr, nrr, max_duty)

                    escape_dur = 0.8  # seconds
                    t_escape = time.time()
                    while time.time() - t_escape < escape_dur:
                        bot.set_motor_model(dfl, drl, dfr, drr)
                        time.sleep(loop_period / 2.0)

                    bot.set_motor_model(0, 0, 0, 0)
                else:
                    # Should almost never happen now, but keep the message
                    print(
                        "[AutoDrive][EMERG] Obstacle not clearly in front sector -> "
                        "staying stopped for this cycle."
                    )
                    bot.set_motor_model(0, 0, 0, 0)

                mode = "STOPPED"
                blocked_counter = 0

                time.sleep(loop_period)
                continue
            # --------------------------------------------------------

            # Decide mode (planner layer; uses full 360° blocked flags)
            prev_mode = mode
            mode, blocked_counter = choose_mode(
                front_blocked=front_blocked,
                left_blocked=left_blocked,
                right_blocked=right_blocked,
                back_blocked=back_blocked,
                mode_prev=prev_mode,
                blocked_counter=blocked_counter,
            )

            # Convert mode -> (vx,vy,wz)
            vx, vy, wz = mode_to_cmd(
                mode,
                args.v_forward,
                args.v_side,
                args.v_back,
                args.w_rotate,
            )

            # ---------- LiDAR-based speed reduction in ALL directions ----------
            if lidar is not None:
                stop_d = args.lidar_th
                slow_d = args.lidar_slow_dist
                min_sc = args.min_speed_scale

                # Forward / backward component
                if vx > 0.0 and front_lidar_m is not None:
                    scale_vx = lidar_slow_scale(front_lidar_m, stop_d, slow_d, min_sc)
                    vx *= scale_vx
                elif vx < 0.0 and back_lidar_m is not None:
                    scale_vx = lidar_slow_scale(back_lidar_m, stop_d, slow_d, min_sc)
                    vx *= scale_vx

                # Left / right component
                if vy > 0.0 and left_lidar_m is not None:
                    scale_vy = lidar_slow_scale(left_lidar_m, stop_d, slow_d, min_sc)
                    vy *= scale_vy
                elif vy < 0.0 and right_lidar_m is not None:
                    scale_vy = lidar_slow_scale(right_lidar_m, stop_d, slow_d, min_sc)
                    vy *= scale_vy

                # Rotation component (any close obstacle around)
                if wz != 0.0 and lidar_min_m is not None:
                    scale_wz = lidar_slow_scale(lidar_min_m, stop_d, slow_d, min_sc)
                    wz *= scale_wz

            # Mix mecanum and send commands
            nfl, nrl, nfr, nrr = mix_mecanum(
                vx,
                vy,
                wz,
                forward_sign=args.forward_sign,
                strafe_sign=args.strafe_sign,
                rotate_sign=args.rotate_sign,
                turn_gain=args.turn_gain,
            )
            dfl, drl, dfr, drr = to_duties(nfl, nrl, nfr, nrr, max_duty)
            bot.set_motor_model(dfl, drl, dfr, drr)

            # ---------- Debug print ----------
            # Build blocked-sides string with distances
            block_tags = []
            # FRONT: pick the "best" distance we have
            if front_blocked:
                front_d = 9999.0
                if front_lidar_cm is not None:
                    front_d = front_lidar_cm
                else:
                    front_d = min(fr_cm, fl_cm, us_cm_safe)
                block_tags.append(f"F({front_d:4.0f}cm)")
            if right_blocked:
                if right_lidar_cm is not None:
                    block_tags.append(f"R({right_lidar_cm:4.0f}cm)")
                else:
                    block_tags.append("R(?)")
            if back_blocked:
                if back_lidar_cm is not None:
                    block_tags.append(f"B({back_lidar_cm:4.0f}cm)")
                else:
                    block_tags.append("B(?)")
            if left_blocked:
                if left_lidar_cm is not None:
                    block_tags.append(f"L({left_lidar_cm:4.0f}cm)")
                else:
                    block_tags.append("L(?)")

            block_str = ",".join(block_tags) if block_tags else "none"

            def fmt_lidar_cm(v: Optional[float]) -> str:
                return f"{v:5.1f}cm" if v is not None else "  N/A "

            def fmt_tof_cm(v: float) -> str:
                return f"{v:5.1f}cm" if v >= 0.0 else "  N/A "

            print(
                f"[AutoDrive] mode={mode:9s}  "
                f"FR_f={fmt_tof_cm(fr_f)}  FL_f={fmt_tof_cm(fl_f)}  "
                f"US={('%.1f cm' % us_cm) if us_cm is not None else 'None':>8s}  "
                f"ToF_near={tof_near!s:<5s}  US_near={us_near!s:<5s}  "
                f"BLOCKS={block_str:<24s}  "
                f"Lfront={fmt_lidar_cm(front_lidar_cm)}  "
                f"Lright={fmt_lidar_cm(right_lidar_cm)}  "
                f"Lback={fmt_lidar_cm(back_lidar_cm)}  "
                f"Lleft={fmt_lidar_cm(left_lidar_cm)}  "
                f"blocked_cnt={blocked_counter}"
            )

            # Extra debug line to see WHY it's blocked
            if front_blocked or left_blocked or right_blocked or back_blocked:
                print(
                    f"[AutoDrive][DBG] front_near_nf={front_near_nf} "
                    f"front_Li={front_block_lidar} "
                    f"Li_primary={lidar_near_primary}  "
                    f"left_near_nf={left_near_nf} right_near_nf={right_near_nf}  "
                    f"Lfb={front_block_lidar} Lrb={right_block_lidar} "
                    f"Lbb={back_block_lidar} Llb={left_block_lidar}"
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
        us_reader.close()
        stop_proc(cam_proc, "camera")
        stop_proc(face_proc, "face")
        print("[AutoDrive] Done.")


if __name__ == "__main__":
    main()
