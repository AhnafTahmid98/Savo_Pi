#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
imu_watch.py — Live BNO055 watcher for Robot Savo
Author: Savo Copilot

Features
- IMU-only (acc+gyro; magnetometer ignored) or NDOF (uses mag)
- Proper unit config: m/s^2, dps, °, °C
- Live console stream with compact columns
- Graceful KeyboardInterrupt
- Optional summary stats (mean/std) at the end
- Robust formatting, no weird f-string spec issues

Usage examples
  # Baseline, still, LiDAR OFF
  python3 tools/diag/sensors/imu_watch.py --imu-only --rate 25 --samples 40

  # With LiDAR ON (new shell), compare visually
  python3 tools/diag/sensors/imu_watch.py --imu-only --rate 25 --samples 40

  # Optional NDOF characterization (expect mag|B| to jump indoors)
  python3 tools/diag/sensors/imu_watch.py --ndof --rate 25 --samples 40

Notes
- BNO055 scaling (per datasheet/common practice):
    Euler: 1 LSB = 1/16 degree
    Accel: 1 LSB = 1/100 m/s^2
    Gyro : 1 LSB = 1/16 dps
    Mag  : 1 LSB = 1/16 µT
"""

import argparse
import math
import time
from collections import deque

try:
    import smbus2
except Exception as e:
    raise SystemExit("smbus2 is required: sudo apt install python3-smbus || pip install smbus2") from e


# -------------------------------
# BNO055 register map / constants
# -------------------------------
BNO055_ADDRESS_A    = 0x28  # default: ADR pin low
BNO055_ADDRESS_B    = 0x29  # ADR pin high

# Page
BNO055_PAGE_ID      = 0x07

# ID
BNO055_CHIP_ID      = 0x00  # Expect 0xA0

# Operation / Power
BNO055_OPR_MODE     = 0x3D
BNO055_PWR_MODE     = 0x3E
BNO055_SYS_TRIGGER  = 0x3F

# Status
BNO055_SYS_STAT     = 0x39
BNO055_SYS_ERR      = 0x3A
BNO055_CALIB_STAT   = 0x35
BNO055_ST_RESULT    = 0x36

# Unit selection
BNO055_UNIT_SEL     = 0x3B
# UNIT_SEL bits:
# 7..6 = (unused)
# 5    = Temperature: 0=C, 1=F
# 4    = Euler:       0=deg, 1=rad
# 3    = Gyro:        0=dps, 1=rps
# 2    = Accel:       0=m/s^2, 1=mg
# 1..0 = (reserved)
UNIT_SEL_VALUE = 0b00000000  # C / deg / dps / m/s^2

# Modes
OPR_MODE_CONFIG         = 0x00
OPR_MODE_ACCONLY        = 0x01
OPR_MODE_MAGONLY        = 0x02
OPR_MODE_GYRONLY        = 0x03
OPR_MODE_AMG            = 0x07
OPR_MODE_IMU            = 0x08
OPR_MODE_COMPASS        = 0x09
OPR_MODE_M4G            = 0x0A
OPR_MODE_NDOF_FMC_OFF   = 0x0B
OPR_MODE_NDOF           = 0x0C

# Data registers (little endian)
ACC_DATA_X_LSB   = 0x08
MAG_DATA_X_LSB   = 0x0E
GYR_DATA_X_LSB   = 0x14
EUL_HEADING_LSB  = 0x1A  # heading (yaw)
EUL_ROLL_LSB     = 0x1C
EUL_PITCH_LSB    = 0x1E
TEMP             = 0x34

# Scale factors
ACC_SCALE  = 1.0 / 100.0   # m/s^2
GYR_SCALE  = 1.0 / 16.0    # dps
EUL_SCALE  = 1.0 / 16.0    # deg
MAG_SCALE  = 1.0 / 16.0    # uT


# -------------------------------
# I2C helpers
# -------------------------------
class BNO055:
    def __init__(self, bus=1, address=BNO055_ADDRESS_A):
        self.bus_no = bus
        self.address = address
        self.bus = smbus2.SMBus(bus)

    def _write8(self, reg, val):
        self.bus.write_byte_data(self.address, reg, val & 0xFF)

    def _read8(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def _readS16(self, reg):
        # little-endian signed 16-bit
        data = self.bus.read_i2c_block_data(self.address, reg, 2)
        val = data[0] | (data[1] << 8)
        if val >= 32768:
            val -= 65536
        return val

    def _read_vector3(self, base_reg, scale):
        x = self._readS16(base_reg + 0) * scale
        y = self._readS16(base_reg + 2) * scale
        z = self._readS16(base_reg + 4) * scale
        return (x, y, z)

    # High-level helpers
    def set_page(self, page=0):
        self._write8(BNO055_PAGE_ID, page & 0xFF)

    def read_chip_id(self):
        return self._read8(BNO055_CHIP_ID)

    def set_units(self):
        self.set_page(0)
        self._write8(BNO055_UNIT_SEL, UNIT_SEL_VALUE)

    def set_mode(self, mode):
        self.set_page(0)
        self._write8(BNO055_OPR_MODE, OPR_MODE_CONFIG)
        time.sleep(0.02)
        self._write8(BNO055_OPR_MODE, mode & 0xFF)
        # Per datasheet: allow mode change settling
        time.sleep(0.05)

    def sys_status(self):
        return self._read8(BNO055_SYS_STAT)

    def sys_err(self):
        return self._read8(BNO055_SYS_ERR)

    def calib_tuple(self):
        val = self._read8(BNO055_CALIB_STAT)
        sys = (val >> 6) & 0x03
        gyr = (val >> 4) & 0x03
        acc = (val >> 2) & 0x03
        mag = (val >> 0) & 0x03
        return sys, gyr, acc, mag

    def self_test(self):
        # ST_RESULT: bit=1 means "pass"
        v = self._read8(BNO055_ST_RESULT)
        acc = bool(v & 0x01)
        mag = bool(v & 0x02)
        gyr = bool(v & 0x04)
        mcu = bool(v & 0x08)
        return acc, mag, gyr, mcu

    def read_temp_c(self):
        t = self._read8(TEMP)
        # datasheet: signed byte; many boards expose 2's complement here
        if t >= 128:
            t -= 256
        return t

    def read_euler(self):
        # heading (yaw), roll, pitch in degrees
        h = self._readS16(EUL_HEADING_LSB) * EUL_SCALE
        r = self._readS16(EUL_ROLL_LSB)    * EUL_SCALE
        p = self._readS16(EUL_PITCH_LSB)   * EUL_SCALE
        return h, r, p

    def read_accel(self):
        return self._read_vector3(ACC_DATA_X_LSB, ACC_SCALE)  # m/s^2

    def read_gyro(self):
        return self._read_vector3(GYR_DATA_X_LSB, GYR_SCALE)  # dps

    def read_mag(self):
        return self._read_vector3(MAG_DATA_X_LSB, MAG_SCALE)  # uT


# -------------------------------
# Formatting helpers
# -------------------------------
def calib_str(sys, gyr, acc, mag):
    # 4 digits: sys,gyr,acc,mag (each 0..3)
    return f"{sys}{gyr}{acc}{mag}"

def fmt_bool(b):
    return "True" if b else "False"


# -------------------------------
# Main stream routine
# -------------------------------
def run(args):
    bno = BNO055(bus=args.bus, address=args.addr)

    # Sanity: chip ID
    chip = bno.read_chip_id()
    if chip != 0xA0:
        print(f"[!] Unexpected CHIP_ID=0x{chip:02X} (expected 0xA0). Check wiring/address.")
        # continue anyway (some clones need time), but warn

    # Units
    bno.set_units()

    # Mode selection
    mode = OPR_MODE_IMU if args.imu_only else OPR_MODE_NDOF
    bno.set_mode(mode)

    # Quick meta
    acc_ok, mag_ok, gyr_ok, mcu_ok = bno.self_test()
    sys_stat = bno.sys_status()
    sys_err  = bno.sys_err()
    sys,gyr,acc,mag = bno.calib_tuple()

    print(f"[{time.strftime('%H:%M:%S')}] BNO055 on i2c-{args.bus} addr 0x{args.addr:02X}")
    print(f"CHIP_ID=0x{chip:02X}  SELFTEST: ACC={fmt_bool(acc_ok)} MAG={fmt_bool(mag_ok)} "
          f"GYR={fmt_bool(gyr_ok)} MCU={fmt_bool(mcu_ok)}")
    print(f"SYS_STATUS={sys_stat}  SYS_ERR={sys_err}  CALIB(sys,gyro,acc,mag)=({sys},{gyr},{acc},{mag})")
    mstr = "IMU (acc+gyro only)" if args.imu_only else "NDOF (fusion uses mag)"
    print(f"Mode: {mstr}  Rate: {args.rate} Hz  Samples: {args.samples if args.samples>0 else '∞'}")
    print()

    # Header
    if not args.no_header:
        print(" t(s) | stat err calib |   yaw     roll    pitch |    ax     ay     az  |  gz(dps) | mag|B|(uT) | T(°C)")
        print("-"*98)

    # Streaming loop
    period = 1.0 / float(args.rate)
    start  = time.time()
    n_done = 0

    # Stats (if requested)
    gz_vals   = deque()
    acc_norms = deque()

    try:
        while True:
            now = time.time()
            t = now - start

            sys_stat = bno.sys_status()
            sys_err  = bno.sys_err()
            sys,gyr,acc,mag = bno.calib_tuple()

            # Reads
            yaw, roll, pitch = bno.read_euler()
            ax, ay, az       = bno.read_accel()
            gx, gy, gz       = bno.read_gyro()
            mx, my, mz       = (0.0, 0.0, 0.0)
            if not args.imu_only:
                mx, my, mz = bno.read_mag()

            # Derived
            acc_abs = math.sqrt(ax*ax + ay*ay + az*az)
            mag_abs = math.sqrt(mx*mx + my*my + mz*mz)

            # Accumulate stats
            if args.stats:
                gz_vals.append(gz)
                acc_norms.append(acc_abs)

            # Print line
            print(f"{t:5.2f} | {sys_stat:4d} {sys_err:3d} {calib_str(sys,gyr,acc,mag):>5} |"
                  f" {yaw:7.2f} {roll:7.2f} {pitch:7.2f} |"
                  f" {ax:6.2f} {ay:6.2f} {az:6.2f} |"
                  f" {gz:7.2f} | {mag_abs:9.2f} | {bno.read_temp_c():3d}")

            n_done += 1
            if args.samples > 0 and n_done >= args.samples:
                break

            # Sleep to rate (simple pacing)
            # Adjust for loop time so we stay close to 'rate'
            elapsed = time.time() - now
            to_sleep = period - elapsed
            if to_sleep > 0:
                time.sleep(to_sleep)
    except KeyboardInterrupt:
        pass

    # Summary stats
    if args.stats and len(gz_vals) >= 2 and len(acc_norms) >= 2:
        def mean_std(vals):
            m = sum(vals) / len(vals)
            v = sum((x - m)**2 for x in vals) / (len(vals) - 1)
            return m, math.sqrt(v)

        gz_mean, gz_std = mean_std(gz_vals)
        acc_m, acc_s    = mean_std(acc_norms)

        print("\nStats (over streamed samples)")
        print("-"*34)
        print(f"gz(dps)  mean={gz_mean:.3f}  std={gz_std:.3f}")
        print(f"|acc|    mean={acc_m:.3f} m/s²  std={acc_s:.3f} m/s²")
        # Quick green/yellow/red hints matching our acceptance gates
        hints = []
        if abs(gz_mean) <= 0.30 and gz_std <= 0.30:
            hints.append("GYRO: PASS")
        elif abs(gz_mean) <= 0.40:
            hints.append("GYRO: WARN")
        else:
            hints.append("GYRO: FAIL")

        if 9.70 <= acc_m <= 9.85 and acc_s <= 0.18:
            hints.append("ACCEL: PASS")
        elif 9.60 <= acc_m <= 9.95 and acc_s <= 0.25:
            hints.append("ACCEL: WARN")
        else:
            hints.append("ACCEL: FAIL")

        print(" | ".join(hints))


# -------------------------------
# CLI
# -------------------------------
def parse_args():
    p = argparse.ArgumentParser(description="Live BNO055 IMU watcher (Robot Savo)")
    gmode = p.add_mutually_exclusive_group()
    gmode.add_argument("--imu-only", action="store_true",
                       help="IMU mode (acc+gyro; magnetometer ignored)")
    gmode.add_argument("--ndof", action="store_true",
                       help="NDOF fusion (uses magnetometer; not recommended indoors)")
    p.add_argument("--bus", type=int, default=1, help="I2C bus (default 1)")
    p.add_argument("--addr", type=lambda x: int(x, 0), default=BNO055_ADDRESS_A,
                   help="I2C address (e.g., 0x28 or 0x29; default 0x28)")
    p.add_argument("--rate", type=float, default=25.0, help="Sampling rate in Hz (default 25)")
    p.add_argument("--samples", type=int, default=0,
                   help="Number of samples to print; 0 = unlimited (default 0)")
    p.add_argument("--stats", action="store_true",
                   help="Print summary stats (mean/std) at the end")
    p.add_argument("--no-header", action="store_true", help="Do not print the header line")
    args = p.parse_args()

    # Default to IMU-only unless --ndof explicitly given
    if not args.imu_only and not args.ndof:
        args.imu_only = True
    return args


def main():
    args = parse_args()
    run(args)


if __name__ == "__main__":
    main()
