#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BNO055 IMU tester for Robot Savo
- Health summary: --health (quick checks, thresholds tuned for a desk test)
- IMU-only fusion: --imu-only (BNO055 IMU mode: accel+gyro only)
- Live stream with interference watch on magnetometer field magnitude
- Defaults: I2C bus 1, address 0x28

Requires: smbus2 (sudo apt-get install python3-smbus) or pip install smbus2
"""

import argparse
import math
import sys
import time
from collections import deque
from statistics import mean, pstdev

try:
    from smbus2 import SMBus
except Exception as e:
    print("ERROR: smbus2 not available. Install with: sudo apt-get install python3-smbus or pip3 install smbus2", file=sys.stderr)
    raise

# BNO055 registers (subset)
CHIP_ID         = 0x00
ACC_ID          = 0x01
MAG_ID          = 0x02
GYR_ID          = 0x03
SW_REV_ID_LSB   = 0x04
SW_REV_ID_MSB   = 0x05
BL_REV_ID       = 0x06

PAGE_ID         = 0x07
ACC_DATA_X_LSB  = 0x08  # 6 regs
MAG_DATA_X_LSB  = 0x0E  # 6 regs
GYR_DATA_X_LSB  = 0x14  # 6 regs
EUL_HEADING_LSB = 0x1A  # 6 regs (heading, roll, pitch)
QUA_DATA_W_LSB  = 0x20  # 8 regs
LIA_DATA_X_LSB  = 0x28  # 6 regs
GRV_DATA_X_LSB  = 0x2E  # 6 regs

TEMP            = 0x34
CALIB_STAT      = 0x35
ST_RESULT       = 0x36
INT_STATUS      = 0x37
SYS_CLK_STATUS  = 0x38
SYS_STATUS      = 0x39
SYS_ERR         = 0x3A
UNIT_SEL        = 0x3B
OPR_MODE        = 0x3D
PWR_MODE        = 0x3E
SYS_TRIGGER     = 0x3F

# OPR_MODE values
MODE_CONFIG     = 0x00
MODE_ACCONLY    = 0x01
MODE_MAGONLY    = 0x02
MODE_GYROONLY   = 0x03
MODE_ACCMAG     = 0x04
MODE_ACCGYRO    = 0x05
MODE_MAGGYRO    = 0x06
MODE_AMG        = 0x07
MODE_IMU        = 0x08   # Acc + Gyro fusion (no mag)
MODE_COMPASS    = 0x09
MODE_M4G        = 0x0A
MODE_NDOF_FMC   = 0x0B
MODE_NDOF       = 0x0C   # Full fusion (Acc + Gyro + Mag)

# PWR_MODE values
PWR_NORMAL      = 0x00
PWR_LOW         = 0x01
PWR_SUSPEND     = 0x02

# UNIT_SEL bits
#  bit0: Accel (0 = m/s^2, 1 = mg)
#  bit1: Angular rate (0 = dps, 1 = rps)
#  bit2: Euler (0 = degrees, 1 = radians)
#  bit4: Temperature (0 = Celsius, 1 = Fahrenheit)
UNITS_DEFAULT = 0b00000000  # m/s^2, dps, degrees, °C

# Scale factors (see datasheet)
ACC_SCALE   = 1/100.0     # m/s^2 per LSB when unit set to m/s^2
GYR_SCALE   = 1/16.0      # dps per LSB
EUL_SCALE   = 1/16.0      # deg per LSB
MAG_SCALE   = 1/16.0      # microTesla per LSB

def _read_len(bus, addr, reg, length):
    return bus.read_i2c_block_data(addr, reg, length)

def _read_u8(bus, addr, reg):
    return bus.read_byte_data(addr, reg)

def _write_u8(bus, addr, reg, val):
    bus.write_byte_data(addr, reg, val & 0xFF)

def _to_int16(lsb, msb):
    v = (msb << 8) | lsb
    return v - 65536 if v & 0x8000 else v

def _set_mode(bus, addr, mode):
    _write_u8(bus, addr, OPR_MODE, MODE_CONFIG)
    time.sleep(0.02)
    _write_u8(bus, addr, OPR_MODE, mode)
    time.sleep(0.02)

def _set_units(bus, addr):
    _write_u8(bus, addr, UNIT_SEL, UNITS_DEFAULT)
    time.sleep(0.01)

def _bringup(bus, addr, imu_only=False):
    # to page 0
    _write_u8(bus, addr, PAGE_ID, 0x00)
    time.sleep(0.002)
    # power mode normal
    _write_u8(bus, addr, PWR_MODE, PWR_NORMAL)
    time.sleep(0.01)
    # units
    _set_units(bus, addr)
    # select fusion mode
    _set_mode(bus, addr, MODE_IMU if imu_only else MODE_NDOF)

def _read_calib_bits(bus, addr):
    try:
        b = _read_u8(bus, addr, CALIB_STAT)
        sys_ = (b >> 6) & 0x3
        gyr  = (b >> 4) & 0x3
        acc  = (b >> 2) & 0x3
        mag  = (b >> 0) & 0x3
        return sys_, gyr, acc, mag
    except OSError:
        return 0,0,0,0

def _read_status(bus, addr):
    status = _read_u8(bus, addr, SYS_STATUS)
    err    = _read_u8(bus, addr, SYS_ERR)
    return status, err

def _read_selftest(bus, addr):
    val = _read_u8(bus, addr, ST_RESULT)
    acc = bool(val & 0x01)
    mag = bool(val & 0x02)
    gyr = bool(val & 0x04)
    mcu = bool(val & 0x08)
    return acc, mag, gyr, mcu

def _read_chip_id(bus, addr):
    return _read_u8(bus, addr, CHIP_ID)

def _read_temp(bus, addr):
    try:
        return _read_u8(bus, addr, TEMP)
    except OSError:
        return None

def _read_vec3(bus, addr, base, scale):
    data = _read_len(bus, addr, base, 6)
    x = _to_int16(data[0], data[1]) * scale
    y = _to_int16(data[2], data[3]) * scale
    z = _to_int16(data[4], data[5]) * scale
    return (x, y, z)

def read_all(bus, addr):
    acc = _read_vec3(bus, addr, ACC_DATA_X_LSB, ACC_SCALE)
    mag = _read_vec3(bus, addr, MAG_DATA_X_LSB, MAG_SCALE)
    gyr = _read_vec3(bus, addr, GYR_DATA_X_LSB, GYR_SCALE)
    eul = _read_vec3(bus, addr, EUL_HEADING_LSB, EUL_SCALE)  # heading, roll, pitch
    t   = _read_temp(bus, addr)
    sys_status, sys_err = _read_status(bus, addr)
    c_sys, c_gyr, c_acc, c_mag = _read_calib_bits(bus, addr)
    return acc, mag, gyr, eul, t, (sys_status, sys_err), (c_sys, c_gyr, c_acc, c_mag)

def fmt_bool(b): return "True" if b else "False"

def print_header_open(bus, addr):
    print(f"[{time.strftime('%H:%M:%S')}] Opening BNO055 on i2c-{args.bus} addr 0x{addr:02X}")
    chip = _read_chip_id(bus, addr)
    acc, mag, gyr, mcu = _read_selftest(bus, addr)
    print(f"[{time.strftime('%H:%M:%S')}] CHIP_ID=0x{chip:02X}  SELFTEST: ACC={fmt_bool(acc)} MAG={fmt_bool(mag)} GYR={fmt_bool(gyr)} MCU={fmt_bool(mcu)}")
    sys_status, sys_err = _read_status(bus, addr)
    print(f"[{time.strftime('%H:%M:%S')}] SYS_STATUS={sys_status}  SYS_ERR={sys_err}  CALIB(sys,gyro,acc,mag)={_calib_str(bus, addr)}\n")

def _calib_str(bus, addr):
    c_sys, c_gyr, c_acc, c_mag = _read_calib_bits(bus, addr)
    return f"({c_sys},{c_gyr},{c_acc},{c_mag})"

def do_health(bus, addr, seconds=3.0, imu_only=False):
    # Collect a short window
    N = max(1, int(seconds * 20))  # ~20 Hz sampling
    acc_norms, gyr_norms, mag_norms = [], [], []
    rolls, pitchs = [], []
    for _ in range(N):
        acc, mag, gyr, eul, t, st, calib = read_all(bus, addr)
        ax, ay, az = acc
        gx, gy, gz = gyr
        mx, my, mz = mag
        _, roll, pitch = eul
        acc_norms.append(math.sqrt(ax*ax + ay*ay + az*az))
        gyr_norms.append(abs(gx) + abs(gy) + abs(gz))  # simple bias proxy
        mag_norms.append(math.sqrt(mx*mx + my*my + mz*mz))
        rolls.append(roll)
        pitchs.append(pitch)
        time.sleep(0.05)

    # Compute stats
    g_mean = mean(acc_norms)
    g_std  = pstdev(acc_norms) if len(acc_norms) > 1 else 0.0
    gyro_bias = mean(gyr_norms)
    mag_field = mean(mag_norms)
    roll_mean = mean([abs(r) for r in rolls])
    pitch_mean= mean([abs(p) for p in pitchs])

    # Thresholds (desk test)
    accel_ok   = (9.5 <= g_mean <= 9.9) and (g_std <= 0.08)
    gyro_ok    = (gyro_bias <= 0.30)
    # Earth field ~25–65 µT: allow up to 120 µT to tolerate indoor noise if IMU-only
    mag_limit  = 120.0 if imu_only else 80.0
    mag_ok     = (mag_field <= mag_limit)
    level_warn = (roll_mean <= 3.0) and (pitch_mean <= 3.0)  # warn if not

    chip = _read_chip_id(bus, addr)
    st_acc, st_mag, st_gyr, st_mcu = _read_selftest(bus, addr)
    sys_status, sys_err = _read_status(bus, addr)
    c_sys, c_gyr, c_acc, c_mag = _read_calib_bits(bus, addr)

    # Table
    print("IMU Health Summary")
    print("---------------------------------------------------------------")
    print("Check         Value                                Status   Note")
    print("---------------------------------------------------------------")
    print(f"Chip ID       0x{chip:02X:<34} {'PASS' if chip==0xA0 else 'FAIL'}    Should be 0xA0")
    print(f"Self-test     ACC={fmt_bool(st_acc)} MAG={fmt_bool(st_mag)} "
          f"GYR={fmt_bool(st_gyr)} MCU={fmt_bool(st_mcu)}  "
          f"{'PASS' if (st_acc and st_mag and st_gyr and st_mcu) else 'FAIL'}    All True expected")
    print(f"System        status={sys_status} err={sys_err:<22} "
          f"{'PASS' if (sys_status in (5, 6) and sys_err==0) else 'WARN'}    5=NDOF, 6=IMU; err=0")
    print(f"Accel |g|     {g_mean:0.2f} ±{g_std:0.02f} m/s²                "
          f"{'PASS' if accel_ok else 'WARN'}    Target ~9.8")
    print(f"Gyro bias     {gyro_bias:0.2f} dps                         "
          f"{'PASS' if gyro_ok else 'WARN'}    < 0.3 dps")
    print(f"Mag |B|       {mag_field:0.1f} µT{' '*23}"
          f"{'PASS' if mag_ok else 'FAIL'}    Earth 25–65 µT"
          f"{' (IMU-only relaxed)' if imu_only else ''}")
    print(f"Roll |mean|   {roll_mean:0.1f}°{' '*28}"
          f"{'PASS' if level_warn else 'WARN'}    Small if robot level")
    print(f"Pitch |mean|  {pitch_mean:0.1f}°{' '*28}"
          f"{'PASS' if level_warn else 'WARN'}    Small if robot level")
    print(f"Calib bits    {c_sys}{c_gyr}{c_acc}{c_mag:<31} "
          f"{'PASS' if (c_sys==3 or imu_only) else 'WARN'}    Target 3333 (IMU-only: sys may be <3)")
    print("---------------------------------------------------------------")

    overall = "PASS" if (accel_ok and gyro_ok and (mag_ok or imu_only)) else "FAIL"
    print(f"Overall: {overall}")

    return overall == "PASS"

def stream(bus, addr, rate_hz=20.0, imu_only=False, watch_threshold=120.0, watch_secs=5.0):
    dt = max(0.01, 1.0/float(rate_hz))
    window = deque(maxlen=int(watch_secs / dt))
    print(f"[{time.strftime('%H:%M:%S')}] Streaming… Press Ctrl+C to stop.")
    last_warn = 0.0
    while True:
        acc, mag, gyr, eul, t, st, calib = read_all(bus, addr)
        (ax,ay,az) = acc
        (mx,my,mz) = mag
        (gx,gy,gz) = gyr
        (heading, roll, pitch) = eul
        field = math.sqrt(mx*mx + my*my + mz*mz)
        window.append(field)

        c_sys, c_gyr, c_acc, c_mag = calib
        calib_str = f"{c_sys}{c_gyr}{c_acc}{c_mag}"

        print(f"Eul(deg)=({heading:7.2f},{roll:7.2f},{pitch:7.2f})  "
              f"Acc(m/s^2)=({ax:6.2f},{ay:6.2f},{az:6.2f})  "
              f"Gyr(dps)=({gx:7.2f},{gy:7.2f},{gz:7.2f})  "
              f"Mag(uT)=({mx:6.2f},{my:6.2f},{mz:7.2f})  "
              f"T={t:3.0f}°C  Calib[{calib_str}]")

        # Interference watch (skip if IMU-only? We still report raw mag.)
        if len(window) == window.maxlen and (not imu_only):
            avg_field = mean(window)
            now = time.time()
            if avg_field > watch_threshold and (now - last_warn) > 2.0:
                print(f"!!! WARNING: Average |B| over last {watch_secs:.0f}s "
                      f"is {avg_field:.1f} µT (> {watch_threshold:.0f} µT). "
                      f"Likely magnetic interference near the IMU.")
                last_warn = now

        time.sleep(dt)

def parse_args():
    p = argparse.ArgumentParser(description="Robot Savo BNO055 IMU test")
    p.add_argument("--bus", type=int, default=1, help="I2C bus number (default 1)")
    p.add_argument("--addr", type=lambda x:int(x,0), default=0x28, help="I2C address (default 0x28)")
    p.add_argument("--health", action="store_true", help="Run health summary then exit")
    p.add_argument("--imu-only", action="store_true", help="Use IMU fusion (no magnetometer)")
    p.add_argument("--rate", type=float, default=20.0, help="Stream rate Hz (default 20)")
    p.add_argument("--watch-threshold", type=float, default=120.0, help="µT threshold for interference warning")
    p.add_argument("--watch-seconds", type=float, default=5.0, help="Seconds of high |B| before warning")
    return p.parse_args()

if __name__ == "__main__":
    args = parse_args()
    try:
        with SMBus(args.bus) as bus:
            print_header_open(bus, args.addr)
            # Bring-up in requested mode
            _bringup(bus, args.addr, imu_only=args.imu_only)

            # Reprint status after mode switch
            sys_status, sys_err = _read_status(bus, args.addr)
            mode = _read_u8(bus, args.addr, OPR_MODE)
            print(f"[{time.strftime('%H:%M:%S')}] Mode set to 0x{mode:02X} "
                  f"({'IMU' if args.imu_only else 'NDOF'})  SYS_STATUS={sys_status} SYS_ERR={sys_err}\n")

            if args.health:
                ok = do_health(bus, args.addr, seconds=3.0, imu_only=args.imu_only)
                print(f"[{time.strftime('%H:%M:%S')}] Done.")
                sys.exit(0 if ok else 1)

            # Otherwise stream
            stream(bus, args.addr, rate_hz=args.rate, imu_only=args.imu_only,
                   watch_threshold=args.watch_threshold, watch_secs=args.watch_seconds)

    except KeyboardInterrupt:
        print(f"\n[{time.strftime('%H:%M:%S')}] Done.")
    except OSError as e:
        print(f"ERROR: I2C communication failed: {e}", file=sys.stderr)
        sys.exit(2)
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(3)
