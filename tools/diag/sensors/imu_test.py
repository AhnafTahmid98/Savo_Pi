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
import signal
import sys
import time
from collections import deque

# -------- Dependencies check --------
try:
    from smbus2 import SMBus
except Exception:
    print(
        "ERROR: smbus2 missing. Install with:\n"
        "  sudo apt-get install -y python3-smbus\n"
        "  pip3 install smbus2"
    )
    sys.exit(1)

# ===============================================================
# BNO055 REGISTER MAP (Page 0 unless noted)
# ===============================================================
BNO055_ADDRESS_A      = 0x28

REG_PAGE_ID           = 0x07
REG_CHIP_ID           = 0x00  # expects 0xA0

# Unit selection (set to m/s², dps, degrees, °C)
# bit0=ACC (0 m/s², 1 mg)
# bit1=GYR (0 dps, 1 rps)
# bit2=EUL (0 deg, 1 rad)
# bit4=TEMP(0 °C,  1 °F)
REG_UNIT_SEL          = 0x3B

# Modes
REG_OPR_MODE          = 0x3D
REG_PWR_MODE          = 0x3E
MODE_CONFIG           = 0x00
MODE_IMU              = 0x08   # IMU Plus (ACC + GYR)
MODE_NDOF             = 0x0C   # Fusion (ACC + GYR + MAG)

REG_SYS_TRIGGER       = 0x3F
REG_SYS_STATUS        = 0x39
REG_SYS_ERR           = 0x3A
REG_TEMP              = 0x34
REG_CALIB_STAT        = 0x35
REG_ST_RESULT         = 0x36

# Sensor data blocks (little-endian)
REG_ACC_DATA          = 0x08  # 6 bytes
REG_MAG_DATA          = 0x0E  # 6 bytes
REG_GYR_DATA          = 0x14  # 6 bytes
REG_EUL_DATA          = 0x1A  # 6 bytes (yaw/roll/pitch) in 1/16 deg units
REG_LIA_DATA          = 0x28  # 6 bytes (linear accel) in 1/100 m/s² units
REG_GRV_DATA          = 0x2E  # 6 bytes (gravity)

UNIT_SEL_DEFAULT      = 0x00  # m/s², dps, degrees, °C

# ===============================================================
# DIAG/GRADING THRESHOLDS
# ===============================================================
# Stationary detection gates (within a sliding 1s window)
GYRO_GATE_DPS_RMS     = 0.5
ACC_GATE_STD          = 0.05
STATIONARY_WINDOW_S   = 1.0

# Health thresholds
GYRO_BIAS_PASS        = 0.30
GYRO_BIAS_CAUTION     = 0.60
GYRO_STD_PASS         = 0.15
GYRO_STD_CAUTION      = 0.30

ACC_MEAN_MIN_PASS     = 9.60
ACC_MEAN_MAX_PASS     = 9.95
ACC_MEAN_MIN_CAUT     = 9.40
ACC_MEAN_MAX_CAUT     = 10.10
ACC_STD_PASS          = 0.05
ACC_STD_CAUTION       = 0.10

TILT_CAUTION_DEG      = 4.0

DEFAULT_I2C_BUS       = 1
DEFAULT_ADDR          = BNO055_ADDRESS_A


# ===============================================================
# Utility helpers
# ===============================================================
class GracefulExit(Exception):
    pass


def _install_sigint_handler():
    def handler(sig, frame):
        raise GracefulExit()
    signal.signal(signal.SIGINT, handler)


def norm3(x, y, z):
    return math.sqrt((x or 0.0) ** 2 + (y or 0.0) ** 2 + (z or 0.0) ** 2)


def mean_std(seq):
    n = len(seq)
    if n == 0:
        return 0.0, 0.0
    m = sum(seq) / n
    v = sum((x - m) ** 2 for x in seq) / n
    return m, math.sqrt(v)


# ===============================================================
# Low-level I²C helpers
# ===============================================================
def rd8(bus, addr, reg):
    return bus.read_byte_data(addr, reg)


def wr8(bus, addr, reg, val):
    bus.write_byte_data(addr, reg, val & 0xFF)


def rdlen(bus, addr, reg, n):
    return bus.read_i2c_block_data(addr, reg, n)


def rds16_le(buf, i):
    """Little-endian signed 16-bit"""
    v = buf[i] | (buf[i + 1] << 8)
    return v - 65536 if v > 32767 else v


# ===============================================================
# BNO055 setup & status
# ===============================================================
def select_page0(bus, addr):
    wr8(bus, addr, REG_PAGE_ID, 0x00)
    time.sleep(0.002)


def set_mode(bus, addr, mode):
    # Must go through CONFIG first per datasheet timing
    wr8(bus, addr, REG_OPR_MODE, MODE_CONFIG)
    time.sleep(0.02)
    wr8(bus, addr, REG_OPR_MODE, mode)
    time.sleep(0.02)


def set_units(bus, addr):
    # m/s², dps, degrees, °C
    wr8(bus, addr, REG_UNIT_SEL, UNIT_SEL_DEFAULT)
    time.sleep(0.002)


def pwr_normal(bus, addr):
    wr8(bus, addr, REG_PWR_MODE, 0x00)
    time.sleep(0.002)


def init_bno(bus, addr):
    select_page0(bus, addr)
    chip = rd8(bus, addr, REG_CHIP_ID)
    if chip != 0xA0:
        print(f"ERROR: CHIP_ID=0x{chip:02X}, expected 0xA0")
        sys.exit(2)
    pwr_normal(bus, addr)
    set_units(bus, addr)


def get_status(bus, addr):
    st = rd8(bus, addr, REG_SYS_STATUS)
    er = rd8(bus, addr, REG_SYS_ERR)
    cal = rd8(bus, addr, REG_CALIB_STAT)
    # calib bits: [7:6 sys][5:4 gyro][3:2 accel][1:0 mag]
    cal_sys = (cal >> 6) & 0x03
    cal_gy  = (cal >> 4) & 0x03
    cal_ac  = (cal >> 2) & 0x03
    cal_mg  = (cal >> 0) & 0x03

    st_res = rd8(bus, addr, REG_ST_RESULT)
    acc_ok = bool(st_res & 0x01)
    mag_ok = bool(st_res & 0x02)
    gyr_ok = bool(st_res & 0x04)
    mcu_ok = bool(st_res & 0x08)
    return st, er, (cal_sys, cal_gy, cal_ac, cal_mg), (acc_ok, mag_ok, gyr_ok, mcu_ok)


# ===============================================================
# Sensor reads 
# ===============================================================
def read_euler_deg(bus, addr):
    buf = rdlen(bus, addr, REG_EUL_DATA, 6)
    yaw   = rds16_le(buf, 0) / 16.0
    roll  = rds16_le(buf, 2) / 16.0
    pitch = rds16_le(buf, 4) / 16.0
    return yaw, roll, pitch


def read_gyro_dps(bus, addr):
    buf = rdlen(bus, addr, REG_GYR_DATA, 6)
    gx = rds16_le(buf, 0) / 16.0
    gy = rds16_le(buf, 2) / 16.0
    gz = rds16_le(buf, 4) / 16.0
    return gx, gy, gz


def read_accel_ms2(bus, addr):
    """
    Prefer linear acceleration (gravity-compensated). If all zeros (not valid),
    fall back to raw accelerometer (includes gravity).
    """
    lia = rdlen(bus, addr, REG_LIA_DATA, 6)
    ax, ay, az = (rds16_le(lia, 0) / 100.0, rds16_le(lia, 2) / 100.0, rds16_le(lia, 4) / 100.0)
    if ax == 0.0 and ay == 0.0 and az == 0.0:
        acc = rdlen(bus, addr, REG_ACC_DATA, 6)
        ax, ay, az = (rds16_le(acc, 0) / 100.0, rds16_le(acc, 2) / 100.0, rds16_le(acc, 4) / 100.0)
    return ax, ay, az


def read_mag_uT(bus, addr):
    buf = rdlen(bus, addr, REG_MAG_DATA, 6)
    mx = rds16_le(buf, 0) / 16.0
    my = rds16_le(buf, 2) / 16.0
    mz = rds16_le(buf, 4) / 16.0
    return mx, my, mz


# ===============================================================
# Grading helpers
# ===============================================================
def grade_value(name, value, pass_max=None, pass_min=None, caution_max=None, caution_min=None):
    """
    Generic grader: returns (grade, note).
    Grade ∈ {'PASS','CAUTION','FAIL'}.
    """
    if pass_min is not None and pass_max is not None:
        if pass_min <= value <= pass_max:
            return "PASS", ""
        if caution_min is not None and caution_max is not None and caution_min <= value <= caution_max:
            return "CAUTION", f"{name} marginal ({value:.3f})"
        return "FAIL", f"{name} out of range ({value:.3f})"

    if pass_max is not None:
        if value <= pass_max:
            return "PASS", ""
        if caution_max is not None and value <= caution_max:
            return "CAUTION", f"{name} marginal ({value:.3f})"
        return "FAIL", f"{name} high ({value:.3f})"

    if pass_min is not None:
        if value >= pass_min:
            return "PASS", ""
        if caution_min is not None and value >= caution_min:
            return "CAUTION", f"{name} marginal ({value:.3f})"
        return "FAIL", f"{name} low ({value:.3f})"

    return "PASS", ""


def combine_grades(*grades):
    s = set(grades)
    if "FAIL" in s:
        return "FAIL"
    if "CAUTION" in s:
        return "CAUTION"
    return "PASS"


def stationary_buffers(rate_hz):
    maxlen = max(2, int(STATIONARY_WINDOW_S * rate_hz))
    return deque(maxlen=maxlen), deque(maxlen=maxlen)


def stationary_update(gbuf, abuf, gz, accnorm):
    gbuf.append(abs(gz or 0.0))
    abuf.append(accnorm or 0.0)
    if len(gbuf) < gbuf.maxlen:
        return False, 0.0, 0.0
    _, gstd = mean_std(list(gbuf))
    _, astd = mean_std(list(abuf))
    return (gstd <= GYRO_GATE_DPS_RMS and astd <= ACC_GATE_STD), gstd, astd


# ===============================================================
# Main
# ===============================================================
def main():
    parser = argparse.ArgumentParser(description="BNO055 (smbus2) diagnostic streamer + health (Robot Savo)")
    m = parser.add_mutually_exclusive_group()
    m.add_argument("--imu-only", action="store_true", help="IMU mode (ACC+GYR) — ignores magnetometer")
    m.add_argument("--ndof", action="store_true", help="Fusion mode (ACC+GYR+MAG), uses magnetometer")

    parser.add_argument("--rate", type=float, default=25.0, help="Sample rate in Hz (default: 25)")
    parser.add_argument("--samples", type=int, default=40, help="Number of lines to stream (default: 40)")
    parser.add_argument("--stats", action="store_true", help="Print mean/std stats over streamed samples")
    parser.add_argument("--health", action="store_true", help="Print motion-aware health summary")
    parser.add_argument("--csv", type=str, default=None, help="Save streamed samples to CSV path")

    parser.add_argument("--i2c-bus", type=int, default=DEFAULT_I2C_BUS, help="I²C bus number (default: 1)")
    parser.add_argument("--addr", type=lambda x: int(x, 0), default=DEFAULT_ADDR, help="I²C address (default: 0x28)")

    args = parser.parse_args()

    # Default to IMU-only unless explicitly asked for NDOF
    use_ndof = True if args.ndof else False

    _install_sigint_handler()

    with SMBus(args.i2c_bus) as bus:
        # Init sequence
        select_page0(bus, args.addr)
        init_bno(bus, args.addr)
        set_mode(bus, args.addr, MODE_NDOF if use_ndof else MODE_IMU)
        time.sleep(0.05)

        chip = rd8(bus, args.addr, REG_CHIP_ID)
        st, er, cal, st_ok = get_status(bus, args.addr)
        acc_ok, mag_ok, gyr_ok, mcu_ok = st_ok

        print(f"[{time.strftime('%H:%M:%S')}] BNO055 on i2c-{args.i2c_bus} addr 0x{args.addr:02X}")
        print(f"CHIP_ID=0x{chip:02X}  SELFTEST: ACC={acc_ok} MAG={mag_ok} GYR={gyr_ok} MCU={mcu_ok}")
        print(f"SYS_STATUS={st}  SYS_ERR={er}  CALIB(sys,gyro,acc,mag)={cal}")
        print(f"Mode: {'NDOF (fusion uses mag)' if use_ndof else 'IMU (acc+gyro only)'}  "
              f"Rate: {args.rate:.1f} Hz  Samples: {args.samples}")

        # Header
        print("\n t(s) | stat err calib |   yaw     roll    pitch |    ax     ay     az  |  gz(dps) | mag|B|(uT) | T(°C)")
        print("-" * 98)

        # Optional CSV
        csvw = None
        if args.csv:
            f = open(args.csv, "w", newline="")
            csvw = csv.writer(f)
            csvw.writerow([
                "t_s", "sys_status", "sys_err", "cal_sys", "cal_gyro", "cal_acc", "cal_mag",
                "yaw_deg", "roll_deg", "pitch_deg", "ax_ms2", "ay_ms2", "az_ms2",
                "gz_dps", "mag_uT", "temp_C"
            ])

        # Streaming state
        gyro_buf, acc_buf = stationary_buffers(args.rate)
        gz_series = []
        acc_series = []

        t0 = time.time()
        period = 1.0 / max(1e-3, args.rate)

        try:
            for _ in range(args.samples):
                t = time.time() - t0

                yaw, roll, pitch = read_euler_deg(bus, args.addr)
                ax, ay, az = read_accel_ms2(bus, args.addr)
                gx, gy, gz = read_gyro_dps(bus, args.addr)

                if use_ndof:
                    mx, my, mz = read_mag_uT(bus, args.addr)
                    mag_norm = norm3(mx, my, mz)
                else:
                    mag_norm = 0.0

                temp_c = rd8(bus, args.addr, REG_TEMP)
                st, er, cal, _ = get_status(bus, args.addr)

                acc_norm = norm3(ax, ay, az)
                is_stat, gstd, astd = stationary_update(gyro_buf, acc_buf, gz, acc_norm)

                print(f"{t:5.2f} | {st:4d} {er:3d}  {cal[0]}{cal[1]}{cal[2]}{cal[3]}  |"
                      f" {yaw:7.2f}  {roll:7.2f}  {pitch:7.2f} |"
                      f" {ax:6.2f} {ay:6.2f} {az:6.2f} | {gz:7.2f} | {mag_norm:8.2f} | {int(temp_c):2d}")

                gz_series.append(gz)
                acc_series.append(acc_norm)

                if csvw:
                    csvw.writerow([
                        f"{t:.3f}", st, er, cal[0], cal[1], cal[2], cal[3],
                        f"{yaw:.3f}", f"{roll:.3f}", f"{pitch:.3f}",
                        f"{ax:.4f}", f"{ay:.4f}", f"{az:.4f}",
                        f"{gz:.4f}", f"{mag_norm:.4f}", int(temp_c)
                    ])

                # Timing control
                dt = period - ((time.time() - t0) - t)
                if dt > 0:
                    time.sleep(dt)

        except GracefulExit:
            print("\n[CTRL-C] Stopping...")

        finally:
            if csvw:
                f.close()

        # -------- Stats summary (over streamed samples) --------
        if args.stats:
            gz_mean, gz_std = mean_std(gz_series)
            acc_m, acc_s = mean_std(acc_series)
            print("\nStats (over streamed samples)")
            print("----------------------------------")
            print(f"gz(dps)  mean={gz_mean:.3f}  std={gz_std:.3f}")
            print(f"|acc|    mean={acc_m:.3f} m/s²  std={acc_s:.3f} m/s²")

            g1, _ = grade_value("GYRO mean|gz|", abs(gz_mean),
                                pass_max=GYRO_BIAS_PASS, caution_max=GYRO_BIAS_CAUTION)
            g2, _ = grade_value("GYRO std", gz_std,
                                pass_max=GYRO_STD_PASS, caution_max=GYRO_STD_CAUTION)
            g3, _ = grade_value("|ACC| mean", acc_m,
                                pass_min=ACC_MEAN_MIN_PASS, pass_max=ACC_MEAN_MAX_PASS,
                                caution_min=ACC_MEAN_MIN_CAUT, caution_max=ACC_MEAN_MAX_CAUT)
            g4, _ = grade_value("|ACC| std", acc_s,
                                pass_max=ACC_STD_PASS, caution_max=ACC_STD_CAUTION)
            overall = combine_grades(g1, g2, g3, g4)
            print(f"GYRO: {g1} | ACCEL: {combine_grades(g3, g4)}")
            print(f"Overall (stats-only): {overall}")

        # -------- Motion-aware health summary --------
        if args.health:
            print("\nIMU Health Summary")
            print("---------------------------------------------------------------")
            print("Check         Value                                Status   Note")
            print("---------------------------------------------------------------")

            chip = rd8(bus, args.addr, REG_CHIP_ID)
            print(f"Chip ID       0x{chip:02X}                               {'PASS' if chip == 0xA0 else 'FAIL'}    Should be 0xA0")

            st, er, cal, st_ok = get_status(bus, args.addr)
            acc_ok, mag_ok, gyr_ok, mcu_ok = st_ok
            all_ok = acc_ok and mag_ok and gyr_ok and mcu_ok
            print(f"Self-test     ACC={acc_ok} MAG={mag_ok} GYR={gyr_ok} MCU={mcu_ok}    "
                  f"{'PASS' if all_ok else 'FAIL'}    All True expected")
            # Note: SYS_STATUS typically 5 in NDOF, 6 in IMU; SYS_ERR should be 0 ideally.
            print(f"System        status={st} err={er}                      {'WARN' if er != 0 else 'PASS'}    5=NDOF, 6=IMU; err=0 ideal")

            gz_mean, gz_std = mean_std(gz_series)
            acc_m, acc_s = mean_std(acc_series)

            g_bias, _ = grade_value("Gyro bias", abs(gz_mean),
                                    pass_max=GYRO_BIAS_PASS, caution_max=GYRO_BIAS_CAUTION)
            g_gstd, _ = grade_value("Gyro std", gz_std,
                                    pass_max=GYRO_STD_PASS, caution_max=GYRO_STD_CAUTION)
            g_am, _ = grade_value("Accel |g|", acc_m,
                                  pass_min=ACC_MEAN_MIN_PASS, pass_max=ACC_MEAN_MAX_PASS,
                                  caution_min=ACC_MEAN_MIN_CAUT, caution_max=ACC_MEAN_MAX_CAUT)
            g_as, _ = grade_value("Accel |g| std", acc_s,
                                  pass_max=ACC_STD_PASS, caution_max=ACC_STD_CAUTION)

            # Snapshot of current orientation (for "is robot level?" hint)
            yaw, roll, pitch = read_euler_deg(bus, args.addr)
            print(f"Accel |g|     {acc_m:4.2f} ±{acc_s:.2f} m/s²                {g_am:<6}  Target ~9.8")
            print(f"Gyro bias     {abs(gz_mean):.2f} dps                         {g_bias:<6}  < 0.3 dps")

            if use_ndof:
                mx, my, mz = read_mag_uT(bus, args.addr)
                print(f"Mag |B|       {norm3(mx, my, mz):.1f} µT                       WARN    Indoors can vary (25–65 µT ideal)")
            else:
                print(f"Mag |B|       0.0 µT                       PASS    IMU-only (ignored)")

            print(f"Roll |mean|   {abs(roll):.1f}°                            "
                  f"{'WARN' if abs(roll) > TILT_CAUTION_DEG else 'PASS'}    Level robot for baseline")
            print(f"Pitch |mean|  {abs(pitch):.1f}°                            "
                  f"{'WARN' if abs(pitch) > TILT_CAUTION_DEG else 'PASS'}    Level robot for baseline")

            overall = combine_grades(g_bias, g_gstd, g_am, g_as)
            print("---------------------------------------------------------------")
            print(f"Overall: {overall}")
            print(f"[{time.strftime('%H:%M:%S')}] Done.")


if __name__ == "__main__":
    main()
