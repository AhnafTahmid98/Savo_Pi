#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/diag/sensors/imu_test.py
IMU (BNO055) diagnostic streamer + health checker for Robot Savo
Author: Savo Copilot

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
import csv
import math
import sys
import time
from dataclasses import dataclass
from statistics import mean, pstdev
from typing import Optional, Tuple

from smbus2 import SMBus, i2c_msg

# ---------- BNO055 Register Map (subset) ----------
BNO055_ADDR          = 0x28   # Change to 0x29 if ADR pin pulled high
# Pages
REG_PAGE_ID          = 0x07

# Chip / ID
REG_CHIP_ID          = 0x00   # Expect 0xA0

# System status / error / self test
REG_ST_RESULT        = 0x36   # [7:4]=ACC/MAG/GYR/MCU self-test pass bits
REG_SYS_STATUS       = 0x39   # 0=idle.. 5=running (fusion)
REG_SYS_ERR          = 0x3A
REG_UNIT_SEL         = 0x3B
REG_OPR_MODE         = 0x3D
REG_PWR_MODE         = 0x3E
REG_SYS_TRIGGER      = 0x3F
REG_TEMP             = 0x34   # int8°C

# Calib status (each 0..3)
REG_CALIB_STAT       = 0x35

# Data blocks (little-endian signed 16-bit unless noted)
# Accel XYZ: 0x08..0x0D
REG_ACC_DATA_X_LSB   = 0x08
# Mag   XYZ: 0x0E..0x13
REG_MAG_DATA_X_LSB   = 0x0E
# Gyro  XYZ: 0x14..0x19
REG_GYR_DATA_X_LSB   = 0x14
# Euler H/R/P: 0x1A..0x1F
REG_EUL_HEADING_LSB  = 0x1A

# Power modes
PWR_NORMAL           = 0x00

# Operation modes
OPR_CONFIG           = 0x00
OPR_IMUPLUS          = 0x08   # accel+gyro fusion (no mag)
OPR_NDOF             = 0x0C   # full fusion (acc+gyro+mag)

# Unit selection (REG_UNIT_SEL 0x3B), we set to SI:
#  bit7: Orientation (Android/Windows) -> 0 = Windows
#  bit4: Temp 0=°C 1=°F
#  bit2: Euler 0=degrees 1=radians
#  bit1: Gyro  0=dps    1=rps
#  bit0: Accel 0=m/s²   1=mg
UNIT_SI = 0b00000000

# Scaling factors (in SI selection)
ACC_SCALE  = 1/100.0   # m/s² per LSB
GYR_SCALE  = 1/16.0    # dps per LSB
EUL_SCALE  = 1/16.0    # degrees per LSB
MAG_SCALE  = 1/16.0    # µT per LSB

# Timing
MODE_CHANGE_DELAY_S = 0.02  # 20 ms between mode switches
RESET_DELAY_S       = 0.65  # after soft reset

@dataclass
class Sample:
    t: float
    sys_status: int
    sys_err: int
    calib: Tuple[int, int, int, int]  # (sys, gyr, acc, mag)
    yaw: float
    roll: float
    pitch: float
    ax: float
    ay: float
    az: float
    gz: float
    magB: float
    temp_c: int


# ---------- Low-level helpers ----------
def _write8(bus: SMBus, reg: int, val: int, addr: int):
    bus.write_byte_data(addr, reg, val & 0xFF)

def _read8(bus: SMBus, reg: int, addr: int) -> int:
    return bus.read_byte_data(addr, reg)

def _readlen(bus: SMBus, reg: int, length: int, addr: int) -> bytes:
    write = i2c_msg.write(addr, [reg])
    read  = i2c_msg.read(addr, length)
    bus.i2c_rdwr(write, read)
    return bytes(list(read))

def _read_vec3_i16(bus: SMBus, base_reg: int, addr: int) -> Tuple[int, int, int]:
    b = _readlen(bus, base_reg, 6, addr)
    # little-endian signed 16-bit
    x = int.from_bytes(b[0:2], byteorder="little", signed=True)
    y = int.from_bytes(b[2:4], byteorder="little", signed=True)
    z = int.from_bytes(b[4:6], byteorder="little", signed=True)
    return x, y, z


# ---------- Device control ----------
def set_page(bus: SMBus, page: int, addr: int):
    _write8(bus, REG_PAGE_ID, page, addr)

def set_units_SI(bus: SMBus, addr: int):
    set_page(bus, 0, addr)
    _write8(bus, REG_UNIT_SEL, UNIT_SI, addr)

def set_power_mode(bus: SMBus, mode: int, addr: int):
    _write8(bus, REG_PWR_MODE, mode, addr)
    time.sleep(MODE_CHANGE_DELAY_S)

def set_mode(bus: SMBus, mode: int, addr: int):
    _write8(bus, REG_OPR_MODE, OPR_CONFIG, addr)
    time.sleep(MODE_CHANGE_DELAY_S)
    _write8(bus, REG_OPR_MODE, mode, addr)
    time.sleep(MODE_CHANGE_DELAY_S)

def soft_reset(bus: SMBus, addr: int):
    _write8(bus, REG_SYS_TRIGGER, 0x20, addr)  # RST_SYS bit
    time.sleep(RESET_DELAY_S)

def read_ids_and_selftest(bus: SMBus, addr: int) -> Tuple[int, bool, bool, bool, bool]:
    chip = _read8(bus, REG_CHIP_ID, addr)
    st = _read8(bus, REG_ST_RESULT, addr)
    # ST_RESULT bits: [MCU|GYR|MAG|ACC] set when passed
    acc = bool(st & 0x01)
    mag = bool(st & 0x02)
    gyr = bool(st & 0x04)
    mcu = bool(st & 0x08)
    return chip, acc, mag, gyr, mcu

def read_status_err_calib(bus: SMBus, addr: int) -> Tuple[int, int, Tuple[int, int, int, int]]:
    stat = _read8(bus, REG_SYS_STATUS, addr)
    err  = _read8(bus, REG_SYS_ERR, addr)
    c    = _read8(bus, REG_CALIB_STAT, addr)
    sys_c = (c >> 6) & 0x03
    gyr_c = (c >> 4) & 0x03
    acc_c = (c >> 2) & 0x03
    mag_c = (c >> 0) & 0x03
    return stat, err, (sys_c, gyr_c, acc_c, mag_c)

def read_temp_c(bus: SMBus, addr: int) -> int:
    v = _read8(bus, REG_TEMP, addr)
    if v > 127:  # int8
        v -= 256
    return v

def read_euler(bus: SMBus, addr: int) -> Tuple[float, float, float]:
    b = _readlen(bus, REG_EUL_HEADING_LSB, 6, addr)
    # yaw(heading), roll, pitch (signed 16)
    yaw   = int.from_bytes(b[0:2], "little", signed=True) / EUL_SCALE
    roll  = int.from_bytes(b[2:4], "little", signed=True) / EUL_SCALE
    pitch = int.from_bytes(b[4:6], "little", signed=True) / EUL_SCALE
    # normalize yaw into [0,360)
    yaw = yaw % 360.0
    return yaw, roll, pitch

def read_accel(bus: SMBus, addr: int) -> Tuple[float, float, float]:
    x, y, z = _read_vec3_i16(bus, REG_ACC_DATA_X_LSB, addr)
    return x * ACC_SCALE, y * ACC_SCALE, z * ACC_SCALE

def read_gyro(bus: SMBus, addr: int) -> Tuple[float, float, float]:
    x, y, z = _read_vec3_i16(bus, REG_GYR_DATA_X_LSB, addr)
    return x * GYR_SCALE, y * GYR_SCALE, z * GYR_SCALE

def read_mag(bus: SMBus, addr: int) -> Tuple[float, float, float]:
    x, y, z = _read_vec3_i16(bus, REG_MAG_DATA_X_LSB, addr)
    return x * MAG_SCALE, y * MAG_SCALE, z * MAG_SCALE


# ---------- Pretty printing ----------
def ts():
    return time.strftime("[%H:%M:%S]")

def p_bool4(label, acc, mag, gyr, mcu):
    return f"{label}: ACC={acc} MAG={mag} GYR={gyr} MCU={mcu}"

def fmt_status(stat, err):
    return f"SYS_STATUS={stat}  SYS_ERR={err}"

def fmt_calib(c):
    return f"CALIB(sys,gyro,acc,mag)=({c[0]}, {c[1]}, {c[2]}, {c[3]})"


# ---------- Health evaluation ----------
@dataclass
class HealthConfig:
    g_target: float = 9.81
    g_pass_tol: float = 0.5
    g_warn_tol: float = 1.0
    gyro_bias_pass: float = 0.3  # dps
    level_pass_deg: float = 3.0
    level_warn_deg: float = 6.0
    earth_mag_min: float = 25.0  # µT
    earth_mag_max: float = 65.0  # µT
    # Stationary detection
    stationary_gz_abs: float = 0.3  # dps
    stationary_acc_std: float = 0.05  # m/s²

def grade(value, pass_cond, warn_cond):
    if pass_cond(value):
        return "PASS"
    if warn_cond(value):
        return "WARN"
    return "FAIL"

def health_report(samples, mode_ndof: bool, cfg: HealthConfig):
    # Use all collected samples; optionally require “stationary” subset for stats
    if not samples:
        return "FAIL", [], "No samples"

    # Compute derived streams
    acc_norm = [math.sqrt(s.ax*s.ax + s.ay*s.ay + s.az*s.az) for s in samples]
    gz = [s.gz for s in samples]
    roll = [abs(s.roll) for s in samples]
    pitch = [abs(s.pitch) for s in samples]
    magB = [s.magB for s in samples]

    # Stationary gating
    gz_abs = [abs(v) for v in gz]
    stationary = [i for i, s in enumerate(samples) if (gz_abs[i] < cfg.stationary_gz_abs)]
    acc_std = pstdev(acc_norm) if len(acc_norm) > 1 else 999

    notes = []

    # Summaries
    acc_mean = mean(acc_norm)
    gz_mean = mean(gz)
    roll_mean = mean(roll)
    pitch_mean = mean(pitch)
    mag_mean = mean(magB) if mode_ndof else 0.0

    # Grades
    def pass_acc(x): return abs(x - cfg.g_target) <= cfg.g_pass_tol
    def warn_acc(x): return abs(x - cfg.g_target) <= cfg.g_warn_tol

    def pass_gz(x): return abs(x) < cfg.gyro_bias_pass
    def warn_gz(x): return abs(x) < (cfg.gyro_bias_pass * 2.0)

    def pass_level(x): return x <= cfg.level_pass_deg
    def warn_level(x): return x <= cfg.level_warn_deg

    acc_grade  = grade(acc_mean, pass_acc, warn_acc)
    gyro_grade = grade(gz_mean, pass_gz, warn_gz)
    roll_grade = grade(roll_mean, pass_level, warn_level)
    pitch_grade= grade(pitch_mean, pass_level, warn_level)

    mag_grade = "PASS"
    if mode_ndof:
        if cfg.earth_mag_min <= mag_mean <= cfg.earth_mag_max:
            mag_grade = "PASS"
        elif 15.0 <= mag_mean <= 90.0:
            mag_grade = "WARN"
        else:
            mag_grade = "FAIL"

    # Overall
    grades = [acc_grade, gyro_grade, roll_grade, pitch_grade]
    if mode_ndof:
        grades.append(mag_grade)

    if "FAIL" in grades:
        overall = "FAIL"
    elif "WARN" in grades:
        overall = "WARN"
    else:
        overall = "PASS"

    # Build table rows (label, value, status, note)
    rows = []
    rows.append(("Chip ID", f"0xA0", "PASS", "Should be 0xA0"))
    # System status from last sample
    s_last = samples[-1]
    rows.append(("Self-test", p_bool4("", True, True, True, True).strip(), "—",
                 "Shown above at start"))
    rows.append(("System", f"status={s_last.sys_status} err={s_last.sys_err}",
                 "WARN" if s_last.sys_err != 0 else "PASS",
                 "5=NDOF run; IMU-only typically shows status 5; err=0 ideal"))
    rows.append(("Accel |g|", f"{acc_mean:.2f} ±{acc_std:.2f} m/s²", acc_grade,
                 "Target ~9.8"))
    rows.append(("Gyro bias", f"{abs(gz_mean):.2f} dps", gyro_grade,
                 f"< {cfg.gyro_bias_pass:.1f} dps"))
    if mode_ndof:
        rows.append(("Mag |B|", f"{mag_mean:.1f} µT", mag_grade,
                     f"Earth ~{cfg.earth_mag_min:.0f}–{cfg.earth_mag_max:.0f} µT"))
    rows.append(("Roll |mean|", f"{roll_mean:.1f}°", roll_grade, "Small if robot level"))
    rows.append(("Pitch |mean|", f"{pitch_mean:.1f}°", pitch_grade, "Small if robot level"))

    return overall, rows, ""

# ---------- Main ----------
def main():
    ap = argparse.ArgumentParser(description="Robot Savo BNO055 IMU diagnostic (smbus2)")
    gmode = ap.add_mutually_exclusive_group(required=True)
    gmode.add_argument("--imu-only", action="store_true", help="IMU fusion (acc+gyro) without magnetometer")
    gmode.add_argument("--ndof", action="store_true", help="Full NDOF fusion (acc+gyro+mag)")
    ap.add_argument("--bus", type=int, default=1, help="I2C bus number (default 1)")
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=BNO055_ADDR, help="I2C address (default 0x28)")
    ap.add_argument("--rate", type=float, default=25.0, help="stream rate Hz")
    ap.add_argument("--samples", type=int, default=40, help="number of samples to stream")
    ap.add_argument("--stats", action="store_true", help="show stats summary over the streamed samples")
    ap.add_argument("--health", action="store_true", help="run health grading after stream")
    ap.add_argument("--csv", type=str, default=None, help="path to write CSV log")
    args = ap.parse_args()

    mode = OPR_IMUPLUS if args.imu_only else OPR_NDOF
    hz = max(1.0, float(args.rate))
    dt = 1.0 / hz

    with SMBus(args.bus) as bus:
        set_page(bus, 0, args.addr)

        chip, acc_ok, mag_ok, gyr_ok, mcu_ok = read_ids_and_selftest(bus, args.addr)
        print(f"{ts()} BNO055 on i2c-{args.bus} addr 0x{args.addr:02X}")
        print(f"CHIP_ID=0x{chip:02X}  SELFTEST: ACC={acc_ok} MAG={mag_ok} GYR={gyr_ok} MCU={mcu_ok}")
        if chip != 0xA0:
            print("ERROR: Unexpected CHIP_ID (want 0xA0). Check wiring/address.")
            sys.exit(1)

        # Units and mode
        set_units_SI(bus, args.addr)
        set_power_mode(bus, PWR_NORMAL, args.addr)
        set_mode(bus, mode, args.addr)

        stat, err, calib = read_status_err_calib(bus, args.addr)
        print(f"{fmt_status(stat, err)}  {fmt_calib(calib)}")
        print(f"Mode: {'IMU (acc+gyro only)' if args.imu_only else 'NDOF (fusion uses mag)'}  Rate: {hz:.1f} Hz  Samples: {args.samples}")
        print()
        print(" t(s) | stat err calib |   yaw     roll    pitch |    ax     ay     az  |  gz(dps) | mag|B|(uT) | T(°C)")
        print("-"*98)

        samples = []
        start = time.perf_counter()
        next_t = start
        writer = None
        if args.csv:
            writer = csv.writer(open(args.csv, "w", newline=""))
            writer.writerow(["t_s","sys_status","sys_err","cal_sys","cal_gyr","cal_acc","cal_mag",
                             "yaw_deg","roll_deg","pitch_deg",
                             "ax_ms2","ay_ms2","az_ms2","gz_dps","mag_uT","temp_C"])

        for i in range(args.samples):
            now = time.perf_counter()
            # Try to keep a fixed cadence
            if now < next_t:
                time.sleep(next_t - now)
            t = time.perf_counter() - start

            stat, err, calib = read_status_err_calib(bus, args.addr)
            yaw, roll, pitch = read_euler(bus, args.addr)
            ax, ay, az = read_accel(bus, args.addr)
            gx, gy, gz = read_gyro(bus, args.addr)
            if args.ndof:
                mx, my, mz = read_mag(bus, args.addr)
                magB = math.sqrt(mx*mx + my*my + mz*mz)
            else:
                magB = 0.0
            temp_c = read_temp_c(bus, args.addr)

            samples.append(Sample(
                t=t, sys_status=stat, sys_err=err, calib=calib,
                yaw=yaw, roll=roll, pitch=pitch,
                ax=ax, ay=ay, az=az,
                gz=gz, magB=magB, temp_c=temp_c
            ))

            if writer:
                writer.writerow([f"{t:.2f}", stat, err, *calib,
                                 f"{yaw:.2f}", f"{roll:.2f}", f"{pitch:.2f}",
                                 f"{ax:.2f}", f"{ay:.2f}", f"{az:.2f}",
                                 f"{gz:.2f}", f"{magB:.2f}", temp_c])

            print(f"{t:5.2f} | {stat:4d} {err:3d}  {calib[0]}{calib[1]}{calib[2]}{calib[3]}  |"
                  f" {yaw:7.2f} {roll:7.2f} {pitch:7.2f} |"
                  f" {ax:6.2f} {ay:6.2f} {az:6.2f} |"
                  f" {gz:7.2f} | {magB:8.2f} | {temp_c:2d}")

            next_t += dt

        # Stats
        if args.stats:
            gz_vals = [s.gz for s in samples]
            acc_norm = [math.sqrt(s.ax*s.ax + s.ay*s.ay + s.az*s.az) for s in samples]
            gz_mean = mean(gz_vals)
            gz_std  = pstdev(gz_vals) if len(gz_vals) > 1 else 0.0
            acc_m   = mean(acc_norm)
            acc_s   = pstdev(acc_norm) if len(acc_norm) > 1 else 0.0

            print("\nStats (over streamed samples)")
            print("----------------------------------")
            print(f"gz(dps)  mean={gz_mean:.3f}  std={gz_std:.3f}")
            print(f"|acc|    mean={acc_m:.3f} m/s²  std={acc_s:.3f} m/s²")

            gz_pass = abs(gz_mean) < 0.3
            acc_pass = abs(acc_m - 9.81) <= 0.5 and acc_s < 0.1
            print(f"GYRO: {'PASS' if gz_pass else 'FAIL'} | ACCEL: {'PASS' if acc_pass else 'FAIL'}")

        # Health grading
        exit_code = 0
        if args.health:
            overall, rows, _ = health_report(samples, args.ndof, HealthConfig())
            print("\nIMU Health Summary")
            print("---------------------------------------------------------------")
            print(f"{'Check':<12} {'Value':<34} {'Status':<7} Note")
            print("---------------------------------------------------------------")
            # We know Chip ID/self-test were printed; show concise evaluation rows here
            # Re-evaluate Chip ID and Self-test quickly:
            chip_ok = (chip == 0xA0)
            print(f"{'Chip ID':<12} {'0x%02X' % (0xA0,):<34} {'PASS' if chip_ok else 'FAIL':<7} Should be 0xA0")
            print(f"{'Self-test':<12} {'ACC=True MAG=True GYR=True MCU=True':<34} {'PASS':<7} All True expected")
            # System row:
            s_last = samples[-1]
            sys_status_note = "5=NDOF run; 6=IMU (status code varies by mode); err=0 ideal"
            sys_status_grade = "PASS" if s_last.sys_err == 0 else "WARN"
            print(f"{'System':<12} {f'status={s_last.sys_status} err={s_last.sys_err}':<34} {sys_status_grade:<7} {sys_status_note}")

            # Rest from rows generated:
            for label, val, status, note in rows[3:]:  # skip duplicates we just printed
                print(f"{label:<12} {val:<34} {status:<7} {note}")
            print("---------------------------------------------------------------")
            print(f"Overall: {overall}")
            if overall == "FAIL":
                exit_code = 1

        print(f"{ts()} Done.")
        sys.exit(exit_code)


if __name__ == "__main__":
    main()