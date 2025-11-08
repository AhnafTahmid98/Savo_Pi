#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BNO055 IMU tester for Robot Savo
- Health summary: --health (quick checks, thresholds tuned for a desk test)
- IMU-only fusion: --imu-only (BNO055 IMU mode: accel+gyro only)
- Live stream with interference watch on magnetometer field magnitude
- Defaults: I2C-1 @ 0x28

Requires: smbus2 (sudo apt-get install python3-smbus) or pip install smbus2
"""

import argparse, math, sys, time
from collections import deque
from statistics import mean, pstdev

try:
    from smbus2 import SMBus
except Exception:
    print("ERROR: smbus2 not available. Install: sudo apt-get install python3-smbus", file=sys.stderr)
    raise

# ----- BNO055 registers (subset)
CHIP_ID         = 0x00
PAGE_ID         = 0x07
ACC_DATA_X_LSB  = 0x08
MAG_DATA_X_LSB  = 0x0E
GYR_DATA_X_LSB  = 0x14
EUL_HEADING_LSB = 0x1A
TEMP            = 0x34
CALIB_STAT      = 0x35
ST_RESULT       = 0x36
SYS_STATUS      = 0x39
SYS_ERR         = 0x3A
UNIT_SEL        = 0x3B
OPR_MODE        = 0x3D
PWR_MODE        = 0x3E

# ----- Modes
MODE_CONFIG     = 0x00
MODE_IMU        = 0x08   # fusion (acc+gyro only)
MODE_NDOF       = 0x0C   # fusion (acc+gyro+mag)
PWR_NORMAL      = 0x00

# ----- Units: m/s², dps, degrees, °C
UNITS_DEFAULT   = 0b00000000

# ----- Scales (datasheet)
ACC_SCALE   = 1/100.0     # m/s² / LSB
GYR_SCALE   = 1/16.0      # dps / LSB
EUL_SCALE   = 1/16.0      # deg / LSB
MAG_SCALE   = 1/16.0      # µT / LSB

def _read_len(bus, addr, reg, length): return bus.read_i2c_block_data(addr, reg, length)
def _read_u8(bus, addr, reg): return bus.read_byte_data(addr, reg)
def _write_u8(bus, addr, reg, val): bus.write_byte_data(addr, reg, val & 0xFF)
def _to_int16(lsb, msb):
    v = (msb << 8) | lsb
    return v - 65536 if v & 0x8000 else v

def _set_mode(bus, addr, mode):
    _write_u8(bus, addr, OPR_MODE, MODE_CONFIG); time.sleep(0.02)
    _write_u8(bus, addr, OPR_MODE, mode);        time.sleep(0.02)

def _set_units(bus, addr):
    _write_u8(bus, addr, UNIT_SEL, UNITS_DEFAULT); time.sleep(0.01)

def _bringup(bus, addr, imu_only=False):
    _write_u8(bus, addr, PAGE_ID, 0x00); time.sleep(0.002)
    _write_u8(bus, addr, PWR_MODE, PWR_NORMAL); time.sleep(0.01)
    _set_units(bus, addr)
    _set_mode(bus, addr, MODE_IMU if imu_only else MODE_NDOF)

def _read_calib_bits(bus, addr):
    try:
        b = _read_u8(bus, addr, CALIB_STAT)
        return ( (b>>6)&3, (b>>4)&3, (b>>2)&3, b&3 )  # sys,gyr,acc,mag
    except OSError:
        return 0,0,0,0

def _read_status(bus, addr):
    return _read_u8(bus, addr, SYS_STATUS), _read_u8(bus, addr, SYS_ERR)

def _read_selftest(bus, addr):
    v = _read_u8(bus, addr, ST_RESULT)
    return bool(v&1), bool(v&2), bool(v&4), bool(v&8)  # acc,mag,gyr,mcu

def _read_chip_id(bus, addr): return _read_u8(bus, addr, CHIP_ID)
def _read_temp(bus, addr):
    try: return _read_u8(bus, addr, TEMP)
    except OSError: return None

def _read_vec3(bus, addr, base, scale):
    d = _read_len(bus, addr, base, 6)
    return (_to_int16(d[0],d[1])*scale, _to_int16(d[2],d[3])*scale, _to_int16(d[4],d[5])*scale)

def read_all(bus, addr):
    acc = _read_vec3(bus, addr, ACC_DATA_X_LSB, ACC_SCALE)
    mag = _read_vec3(bus, addr, MAG_DATA_X_LSB, MAG_SCALE)
    gyr = _read_vec3(bus, addr, GYR_DATA_X_LSB, GYR_SCALE)
    eul = _read_vec3(bus, addr, EUL_HEADING_LSB, EUL_SCALE) # heading, roll, pitch
    t   = _read_temp(bus, addr)
    st  = _read_status(bus, addr)
    cb  = _read_calib_bits(bus, addr)
    return acc, mag, gyr, eul, t, st, cb

def fmt_bool(b): return "True" if b else "False"

def print_header_open(bus, addr, busnum):
    print(f"[{time.strftime('%H:%M:%S')}] Opening BNO055 on i2c-{busnum} addr 0x{addr:02X}")
    chip = _read_chip_id(bus, addr)
    acc, mag, gyr, mcu = _read_selftest(bus, addr)
    print(f"[{time.strftime('%H:%M:%S')}] CHIP_ID=0x{chip:02X}  SELFTEST: ACC={fmt_bool(acc)} MAG={fmt_bool(mag)} GYR={fmt_bool(gyr)} MCU={fmt_bool(mcu)}")
    sys_status, sys_err = _read_status(bus, addr)
    c_sys, c_gyr, c_acc, c_mag = _read_calib_bits(bus, addr)
    print(f"[{time.strftime('%H:%M:%S')}] SYS_STATUS={sys_status}  SYS_ERR={sys_err}  CALIB(sys,gyro,acc,mag)=({c_sys},{c_gyr},{c_acc},{c_mag})\n")

def do_health(bus, addr, seconds=3.0, imu_only=False):
    N = max(1, int(seconds*20))
    acc_norms, gyr_norms, mag_norms, rolls, pitchs = [], [], [], [], []
    for _ in range(N):
        acc, mag, gyr, eul, t, st, calib = read_all(bus, addr)
        ax, ay, az = acc
        gx, gy, gz = gyr
        mx, my, mz = mag
        _, roll, pitch = eul
        acc_norms.append(math.sqrt(ax*ax + ay*ay + az*az))
        gyr_norms.append(abs(gx)+abs(gy)+abs(gz))
        mag_norms.append(math.sqrt(mx*mx + my*my + mz*mz))
        rolls.append(roll); pitchs.append(pitch)
        time.sleep(0.05)

    g_mean = mean(acc_norms); g_std = pstdev(acc_norms) if len(acc_norms)>1 else 0.0
    gyro_bias = mean(gyr_norms)
    mag_field = mean(mag_norms)
    roll_mean = mean([abs(r) for r in rolls]); pitch_mean = mean([abs(p) for p in pitchs])

    accel_ok = (9.5 <= g_mean <= 9.9) and (g_std <= 0.08)
    gyro_ok  = (gyro_bias <= 0.30)
    mag_limit = 120.0 if imu_only else 80.0
    mag_ok   = (mag_field <= mag_limit)
    level_ok = (roll_mean <= 3.0) and (pitch_mean <= 3.0)

    chip = _read_chip_id(bus, addr)
    st_acc, st_mag, st_gyr, st_mcu = _read_selftest(bus, addr)
    sys_status, sys_err = _read_status(bus, addr)
    c_sys, c_gyr, c_acc, c_mag = _read_calib_bits(bus, addr)

    print("IMU Health Summary")
    print("---------------------------------------------------------------")
    print("Check         Value                                Status   Note")
    print("---------------------------------------------------------------")
    # FIXED: format safely and align with ljust
    chip_str = f"0x{chip:02X}".ljust(34)
    print(f"Chip ID       {chip_str} {'PASS' if chip==0xA0 else 'FAIL'}    Should be 0xA0")
    st_str = f"ACC={fmt_bool(st_acc)} MAG={fmt_bool(st_mag)} GYR={fmt_bool(st_gyr)} MCU={fmt_bool(st_mcu)}"
    print(f"Self-test     {st_str:<38} {'PASS' if (st_acc and st_mag and st_gyr and st_mcu) else 'FAIL'}    All True expected")
    print(f"System        status={sys_status} err={sys_err:<22} {'PASS' if (sys_status in (5,6) and sys_err==0) else 'WARN'}    5=NDOF, 6=IMU; err=0")
    print(f"Accel |g|     {g_mean:0.2f} ±{g_std:0.02f} m/s²                {'PASS' if accel_ok else 'WARN'}    Target ~9.8")
    print(f"Gyro bias     {gyro_bias:0.2f} dps                         {'PASS' if gyro_ok else 'WARN'}    < 0.3 dps")
    print(f"Mag |B|       {mag_field:0.1f} µT{' '*23}{'PASS' if mag_ok else 'FAIL'}    Earth 25–65 µT{' (IMU-only relaxed)' if imu_only else ''}")
    print(f"Roll |mean|   {roll_mean:0.1f}°{' '*28}{'PASS' if level_ok else 'WARN'}    Small if robot level")
    print(f"Pitch |mean|  {pitch_mean:0.1f}°{' '*28}{'PASS' if level_ok else 'WARN'}    Small if robot level")
    calib_bits = f"{c_sys}{c_gyr}{c_acc}{c_mag}"
    note = "Target 3333 (IMU-only: sys may be <3)" if imu_only else "Target 3333"
    print(f"Calib bits    {calib_bits:<34} {'PASS' if (c_sys==3 or imu_only) else 'WARN'}    {note}")
    print("---------------------------------------------------------------")

    overall = "PASS" if (accel_ok and gyro_ok and (mag_ok or imu_only)) else "FAIL"
    print(f"Overall: {overall}")
    return overall == "PASS"

def stream(bus, addr, rate_hz=20.0, imu_only=False, watch_threshold=120.0, watch_secs=5.0):
    dt = max(0.01, 1.0/float(rate_hz))
    window = deque(maxlen=int(watch_secs/dt))
    print(f"[{time.strftime('%H:%M:%S')}] Streaming… Press Ctrl+C to stop.")
    last_warn = 0.0
    while True:
        acc, mag, gyr, eul, t, st, calib = read_all(bus, addr)
        (ax,ay,az) = acc; (mx,my,mz) = mag; (gx,gy,gz) = gyr
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

        if len(window)==window.maxlen and (not imu_only):
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
            print_header_open(bus, args.addr, args.bus)
            _bringup(bus, args.addr, imu_only=args.imu_only)

            sys_status, sys_err = _read_status(bus, args.addr)
            mode = _read_u8(bus, args.addr, OPR_MODE)
            print(f"[{time.strftime('%H:%M:%S')}] Mode set to 0x{mode:02X} "
                  f"({'IMU' if args.imu_only else 'NDOF'})  SYS_STATUS={sys_status} SYS_ERR={sys_err}\n")

            if args.health:
                ok = do_health(bus, args.addr, seconds=3.0, imu_only=args.imu_only)
                print(f"[{time.strftime('%H:%M:%S')}] Done.")
                sys.exit(0 if ok else 1)

            stream(bus, args.addr, rate_hz=args.rate, imu_only=args.imu_only,
                   watch_threshold=args.watch_threshold, watch_secs=args.watch_seconds)

    except KeyboardInterrupt:
        print(f"\n[{time.strftime('%H:%M:%S')}] Done.")
    except OSError as e:
        print(f"ERROR: I2C communication failed: {e}", file=sys.stderr); sys.exit(2)
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr); sys.exit(3)
