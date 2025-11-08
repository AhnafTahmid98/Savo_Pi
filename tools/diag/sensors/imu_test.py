#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
imu_test.py
IMU (BNO055) diagnostic streamer + health checker for Robot Savo
Author: Robot Savo

FEATURES
---------------------------------------------------------------
- IMU-only (acc+gyro) or NDOF (fusion with magnetometer).
- Live stream at a fixed sample rate (e.g. 25 Hz).
- Motion-aware health grading (grades when stationary).
- Stats summary (mean/std) over the streamed window.
- CSV logging for offline analysis.
- Clear PASS/CAUTION/FAIL reasons.


"""

import argparse
import math
import sys
import time
import shutil
import subprocess
from dataclasses import dataclass
from typing import Optional, Tuple, List

try:
    from smbus2 import SMBus
except Exception:
    print("ERROR: smbus2 is required. Install with: pip3 install smbus2", file=sys.stderr)
    raise

# ---------------- BNO055 minimal map ----------------
BNO055_ADDRESS_A         = 0x28
BNO055_ID                = 0xA0
BNO055_PAGE_ID_ADDR      = 0x07
BNO055_CHIP_ID_ADDR      = 0x00
BNO055_ACCEL_DATA_X_LSB  = 0x08
BNO055_MAG_DATA_X_LSB    = 0x0E
BNO055_GYRO_DATA_X_LSB   = 0x14
BNO055_EULER_H_LSB       = 0x1A
BNO055_TEMP_ADDR         = 0x34
BNO055_CALIB_STAT_ADDR   = 0x35
BNO055_SYS_STATUS_ADDR   = 0x39
BNO055_SYS_ERR_ADDR      = 0x3A
BNO055_UNIT_SEL_ADDR     = 0x3B
BNO055_OPR_MODE_ADDR     = 0x3D
BNO055_PWR_MODE_ADDR     = 0x3E
BNO055_SYS_TRIGGER_ADDR  = 0x3F

POWER_MODE_NORMAL        = 0x00
OPERATION_MODE_CONFIG    = 0x00
OPERATION_MODE_IMU       = 0x08
OPERATION_MODE_NDOF      = 0x0C

# Units: metric + deg + dps + °C
UNIT_SEL_METRIC_DEG_DPS_C = 0b00000000

# --- Correct scales (datasheet with above units) ---
# Accel: 1 LSB = 0.01 m/s² → scale = 1/100
# Gyro : 1 LSB = 1/16 dps  → scale = 1/16
# Euler: 1 LSB = 1/16 deg  → already deg scale
# Mag  : 1 LSB = 1/16 µT   → already µT scale
ACC_SCALE = 1.0/100.0      # m/s² per LSB
GYR_SCALE = 1.0/16.0       # dps per LSB
EUL_SCALE = 1.0/16.0       # deg per LSB
MAG_SCALE = 1.0/16.0       # µT per LSB

G = 9.80665

@dataclass
class Sample:
    t: float
    yaw: Optional[float]
    roll: Optional[float]
    pitch: Optional[float]
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    b_mag: Optional[float]
    temp_c: float
    moving: int
    vibe_score: float

class BNO055:
    def __init__(self, bus: int = 1, addr: int = BNO055_ADDRESS_A):
        self.addr = addr
        self.bus = SMBus(bus)

    def write8(self, reg: int, val: int):
        self.bus.write_byte_data(self.addr, reg, val & 0xFF)

    def read8(self, reg: int) -> int:
        return self.bus.read_byte_data(self.addr, reg)

    def read16s(self, reg_lsb: int) -> int:
        lsb = self.bus.read_byte_data(self.addr, reg_lsb)
        msb = self.bus.read_byte_data(self.addr, reg_lsb + 1)
        v = (msb << 8) | lsb
        if v & 0x8000:
            v -= 0x10000
        return v

    def read_vec3(self, reg_lsb: int, scale: float) -> Tuple[float, float, float]:
        x = self.read16s(reg_lsb) * scale
        y = self.read16s(reg_lsb + 2) * scale
        z = self.read16s(reg_lsb + 4) * scale
        return x, y, z

    def set_mode(self, mode: int):
        self.write8(BNO055_OPR_MODE_ADDR, mode)
        time.sleep(0.03)

    def set_page(self, page: int):
        self.write8(BNO055_PAGE_ID_ADDR, page & 0xFF)
        time.sleep(0.001)

    def reset(self):
        self.write8(BNO055_SYS_TRIGGER_ADDR, 0x20)  # RST_SYS
        time.sleep(0.65)
        for _ in range(60):
            try:
                if self.read8(BNO055_CHIP_ID_ADDR) == BNO055_ID:
                    break
            except Exception:
                pass
            time.sleep(0.02)

    def initialize(self, imu_only: bool):
        self.set_mode(OPERATION_MODE_CONFIG)
        self.write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL)
        time.sleep(0.01)
        self.set_page(0)
        self.write8(BNO055_UNIT_SEL_ADDR, UNIT_SEL_METRIC_DEG_DPS_C)
        time.sleep(0.01)
        self.write8(BNO055_SYS_TRIGGER_ADDR, 0x00)
        time.sleep(0.01)
        self.set_mode(OPERATION_MODE_IMU if imu_only else OPERATION_MODE_NDOF)

    # Reads
    def chip_id(self) -> int:      return self.read8(BNO055_CHIP_ID_ADDR)
    def sys_status(self) -> int:   return self.read8(BNO055_SYS_STATUS_ADDR)
    def sys_err(self) -> int:      return self.read8(BNO055_SYS_ERR_ADDR)
    def calib_tuple(self) -> Tuple[int,int,int,int]:
        v = self.read8(BNO055_CALIB_STAT_ADDR)
        return ((v>>6)&3, (v>>4)&3, (v>>2)&3, v&3)
    def temperature_c(self) -> float:
        t = self.read8(BNO055_TEMP_ADDR)
        if t > 127:
            t -= 256
        return float(t)
    def read_euler_deg(self) -> Tuple[float,float,float]:
        h = self.read16s(BNO055_EULER_H_LSB) * EUL_SCALE
        r = self.read16s(BNO055_EULER_H_LSB + 2) * EUL_SCALE
        p = self.read16s(BNO055_EULER_H_LSB + 4) * EUL_SCALE
        return h, r, p
    def read_accel(self): return self.read_vec3(BNO055_ACCEL_DATA_X_LSB, ACC_SCALE)
    def read_gyro(self):  return self.read_vec3(BNO055_GYRO_DATA_X_LSB, GYR_SCALE)
    def read_mag(self):   return self.read_vec3(BNO055_MAG_DATA_X_LSB, MAG_SCALE)

# ---------------- helpers ----------------
def mag3(x,y,z):
    return math.sqrt(x*x + y*y + z*z)

def auto_detect_lidar() -> bool:
    """Heuristic: returns True if LiDAR likely running."""
    if shutil.which("ros2"):
        try:
            out = subprocess.check_output(["ros2","topic","list"], timeout=1.5).decode()
            if "/scan" in out:
                return True
        except Exception:
            pass
    try:
        out = subprocess.check_output(["ps","-A"], timeout=1.0).decode().lower()
        for key in ("rplidar","ydlidar","slamtec"):
            if key in out:
                return True
    except Exception:
        pass
    return False

def compute_rms(vals: List[float]) -> float:
    if not vals:
        return float("nan")
    return math.sqrt(sum(v*v for v in vals) / len(vals))

def grade_health(reasons: List[str]) -> str:
    return "C" if any(r.startswith("MAJOR:") for r in reasons) else ("B" if reasons else "A")

# ---------------- main ----------------
def main():
    ap = argparse.ArgumentParser(description="Robot Savo BNO055 IMU diagnostic (LiDAR+Motion aware)")
    ap.add_argument("--bus", type=int, default=1)
    ap.add_argument("--addr", type=lambda s: int(s,0), default=BNO055_ADDRESS_A)
    ap.add_argument("--rate", type=float, default=25.0)
    ap.add_argument("--samples", type=int, default=200)
    ap.add_argument("--imu_only", action="store_true", help="IMU mode (no mag/fusion)")
    ap.add_argument("--stats", action="store_true", help="Summary statistics")
    ap.add_argument("--health", action="store_true", help="Health grade A/B/C with notes")
    ap.add_argument("--csv", action="store_true", help="CSV output instead of table")
    ap.add_argument("--out", type=str, default="", help="Save CSV/table to file path")
    ap.add_argument("--tag", type=str, default="", help="Freeform test tag, e.g., LIDAR_ON, MOVING")
    ap.add_argument("--auto_lidar", action="store_true", help="Heuristically tag if LiDAR is ON")
    ap.add_argument("--detect_motion", action="store_true", help="Per-sample MOVING detection + vibe score")
    # Motion thresholds (tuned for desk testing)
    ap.add_argument("--gyro_rms_th", type=float, default=2.0, help="dps threshold for MOVING")
    ap.add_argument("--acc_std_th", type=float, default=0.15, help="m/s² threshold for MOVING")
    ap.add_argument("--window", type=int, default=20, help="window (samples) for motion/vibe calc")
    args = ap.parse_args()

    # Output sink
    out = None
    if args.out:
        out = open(args.out, "w", buffering=1)

    def _print(s: str):
        print(s)
        if out:
            print(s, file=out)

    # Connect sensor
    bno = BNO055(bus=args.bus, addr=args.addr)
    chip = bno.chip_id()
    if chip != BNO055_ID:
        _print(f"ERROR: CHIP_ID=0x{chip:02X}, expected 0xA0 at addr {hex(args.addr)} on bus {args.bus}")
        if out:
            out.close()
        sys.exit(2)

    bno.reset()
    bno.initialize(imu_only=args.imu_only)

    # Status banner
    sys_stat = bno.sys_status()
    sys_err  = bno.sys_err()
    calib = bno.calib_tuple()
    mode = "IMU (acc+gyro)" if args.imu_only else "NDOF (fusion+mag)"
    tag = args.tag.strip()
    if args.auto_lidar and auto_detect_lidar():
        tag = (tag + " ").strip() + "LIDAR_ON"

    _print(f"[{time.strftime('%H:%M:%S')}] BNO055 on i2c-{args.bus} addr {hex(args.addr)}")
    _print(f"CHIP_ID=0x{chip:02X}  SYS_STATUS={sys_stat}  SYS_ERR={sys_err}  CALIB(sys,gyro,acc,mag)={calib}")
    _print(f"Mode: {mode}  Rate: {args.rate:.1f} Hz  Samples: {args.samples}")
    if tag:
        _print(f"TAG: {tag}")

    # Headers
    if args.csv:
        cols = [
            "t_s","sys_stat","sys_err","c_sys","c_gyr","c_acc","c_mag",
            "yaw_deg","roll_deg","pitch_deg","ax","ay","az","gx","gy","gz","mag_uT","temp_C",
            "moving","vibe_score"
        ]
        _print(",".join(cols))
    else:
        _print("\n t(s) | stat err calib |   yaw     roll    pitch |    ax     ay     az  |   gx     gy     gz  |  |B|(uT) | T(°C) | M | Vibe")
        _print("-"*118)

    # Buffers for stats + motion windows
    ax_all: List[float] = []
    ay_all: List[float] = []
    az_all: List[float] = []
    gz_all: List[float] = []
    gnorm_all: List[float] = []

    win_ax: List[float] = []
    win_ay: List[float] = []
    win_az: List[float] = []
    win_gx: List[float] = []
    win_gy: List[float] = []
    win_gz: List[float] = []

    period = 1.0 / max(1e-3, args.rate)
    t0 = time.monotonic()

    for _ in range(args.samples):
        t_loop = time.monotonic()
        sys_stat = bno.sys_status()
        sys_err  = bno.sys_err()
        c_sys, c_gyr, c_acc, c_mag = bno.calib_tuple()

        if args.imu_only:
            yaw = roll = pitch = None
        else:
            try:
                yaw, roll, pitch = bno.read_euler_deg()
            except Exception:
                yaw = roll = pitch = None

        ax, ay, az = bno.read_accel()
        gx, gy, gz = bno.read_gyro()
        if args.imu_only:
            b_mag = None
        else:
            try:
                mx, my, mz = bno.read_mag()
                b_mag = mag3(mx,my,mz)
            except Exception:
                b_mag = None
        temp_c = bno.temperature_c()
        t = t_loop - t0

        # Motion/vibration window update
        if args.detect_motion:
            for buf,val in ((win_ax,ax),(win_ay,ay),(win_az,az),(win_gx,gx),(win_gy,gy),(win_gz,gz)):
                buf.append(val)
                if len(buf) > args.window:
                    buf.pop(0)

            if len(win_ax) >= max(5, args.window//2):
                # accel std and gyro RMS
                def _std(ws: List[float]) -> float:
                    n=len(ws)
                    if n<2:
                        return 0.0
                    m=sum(ws)/n
                    return math.sqrt(sum((v-m)**2 for v in ws)/max(1,n-1))

                ax_std = _std(win_ax)
                ay_std = _std(win_ay)
                az_std = _std(win_az)
                acc_std = (ax_std+ay_std+az_std)/3.0

                def _rms(ws: List[float]) -> float:
                    return math.sqrt(sum(v*v for v in ws)/len(ws)) if ws else 0.0
                gyro_rms = math.sqrt((_rms(win_gx)**2 + _rms(win_gy)**2 + _rms(win_gz)**2)/3.0)

                moving = int(gyro_rms > args.gyro_rms_th or acc_std > args.acc_std_th)
                vibe_score = gyro_rms + abs(acc_std)
            else:
                moving = 0
                vibe_score = 0.0
        else:
            moving = 0
            vibe_score = 0.0

        # Record stat buffers
        ax_all.append(ax); ay_all.append(ay); az_all.append(az)
        gz_all.append(gz)
        gnorm_all.append(mag3(ax,ay,az))

        # Emit row
        if args.csv:
            row = [
                f"{t:.2f}", str(sys_stat), str(sys_err), str(c_sys), str(c_gyr), str(c_acc), str(c_mag),
                "" if yaw is None else f"{yaw:.2f}",
                "" if roll is None else f"{roll:.2f}",
                "" if pitch is None else f"{pitch:.2f}",
                f"{ax:.3f}", f"{ay:.3f}", f"{az:.3f}",
                f"{gx:.3f}", f"{gy:.3f}", f"{gz:.3f}",
                "" if b_mag is None else f"{b_mag:.2f}",
                f"{temp_c:.1f}",
                str(moving),
                f"{vibe_score:.3f}",
            ]
            _print(",".join(row))
        else:
            yaw_s  = f"{yaw:6.2f}"   if yaw is not None else "      "
            roll_s = f"{roll:6.2f}"  if roll is not None else "      "
            pitch_s= f"{pitch:6.2f}" if pitch is not None else "      "
            bmag_s = f"{b_mag:7.2f}" if b_mag is not None else "       "
            _print(f"{t:5.2f} | {sys_stat:3d} {sys_err:3d} {c_sys}{c_gyr}{c_acc}{c_mag} |"
                   f"{yaw_s}  {roll_s}  {pitch_s} |"
                   f"{ax:7.3f} {ay:7.3f} {az:7.3f} |"
                   f"{gx:7.3f} {gy:7.3f} {gz:7.3f} |"
                   f"{bmag_s} | {temp_c:5.1f} | {moving:d} | {vibe_score:5.3f}")

        # pacing
        dt = time.monotonic() - t_loop
        slp = period - dt
        if slp > 0:
            time.sleep(slp)

    # ------- Summary -------
    if args.stats or args.health:
        _print("\n--- Summary ---")

    if args.stats:
        def stats(xs: List[float]):
            n = len(xs)
            mean = sum(xs)/n
            var = sum((x-mean)**2 for x in xs)/max(1,n-1)
            return mean, math.sqrt(var), min(xs), max(xs)

        ax_m, ax_s, ax_min, ax_max = stats(ax_all)
        ay_m, ay_s, ay_min, ay_max = stats(ay_all)
        az_m, az_s, az_min, az_max = stats(az_all)
        gz_m, gz_s, _, _           = stats(gz_all)
        gnorm_m, gnorm_s, _, _     = stats(gnorm_all)

        _print(f"Accel X  (m/s^2): mean={ax_m:.3f}  std={ax_s:.3f}  min={ax_min:.3f}  max={ax_max:.3f}")
        _print(f"Accel Y  (m/s^2): mean={ay_m:.3f}  std={ay_s:.3f}  min={ay_min:.3f}  max={ay_max:.3f}")
        _print(f"Accel Z  (m/s^2): mean={az_m:.3f}  std={az_s:.3f}  min={az_min:.3f}  max={az_max:.3f}")
        _print(f"|g| norm (m/s^2): mean={gnorm_m:.3f}  std={gnorm_s:.3f}  (target ≈ 9.81 at rest)")
        _print(f"Gyro Z  (dps)   : mean={gz_m:.3f}  std={gz_s:.3f}  (bias near 0 if at rest)")

    exit_code = 0  # default: success

    if args.health:
        reasons: List[str] = []
        sys_stat = bno.sys_status()
        sys_err  = bno.sys_err()
        c_sys, c_gyr, c_acc, c_mag = bno.calib_tuple()

        if sys_stat not in (5,0):
            reasons.append(f"MINOR: Unexpected SYS_STATUS={sys_stat}")
        if sys_err != 0:
            reasons.append(f"MAJOR: SYS_ERR={sys_err}")

        temp_c = bno.temperature_c()
        if temp_c < 0.0 or temp_c > 60.0:
            reasons.append(f"MAJOR: Temperature out of range: {temp_c:.1f}C")
        elif temp_c < 5.0 or temp_c > 55.0:
            reasons.append(f"MINOR: Temperature near limits: {temp_c:.1f}C")

        if gnorm_all:
            gmean = sum(gnorm_all)/len(gnorm_all)
            if abs(gmean - G) > 0.8:
                reasons.append(f"MAJOR: |g| mean {gmean:.2f} far from 9.81")
            elif abs(gmean - G) > 0.4:
                reasons.append(f"MINOR: |g| mean {gmean:.2f} slightly off 9.81")

        if gz_all:
            gz_mean = sum(gz_all)/len(gz_all)
            if abs(gz_mean) > 1.5:
                reasons.append(f"MAJOR: Gyro Z bias {gz_mean:.2f} dps high (resting?)")
            elif abs(gz_mean) > 0.8:
                reasons.append(f"MINOR: Gyro Z bias {gz_mean:.2f} dps elevated")

        if c_gyr < 2:
            reasons.append(f"MINOR: Gyro calib low ({c_gyr}/3)")
        if c_acc < 2:
            reasons.append(f"MINOR: Accel calib low ({c_acc}/3)")
        if not args.imu_only and c_mag < 2:
            reasons.append(f"MINOR: Mag calib low ({c_mag}/3)")

        grade = grade_health(reasons)
        _print(f"Health grade: {grade}")
        if reasons:
            _print("Notes:")
            for r in reasons:
                _print(f" - {r}")

        if any(r.startswith("MAJOR:") for r in reasons):
            exit_code = 1

    if out:
        out.close()

    # Exit with pass/fail when --health was requested
    if args.health:
        sys.exit(exit_code)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted by user.", file=sys.stderr)
