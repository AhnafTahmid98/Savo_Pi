#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU (BNO055) diagnostic for Robot Savo
- Initializes sensor in NDOF mode
- Prints Euler (deg), Quaternion, Accel (m/s^2), Gyro (deg/s), Mag (uT), Temp (°C)
- Shows calibration and self-test status
- Optional CSV logging

Usage:
  python3 tools/diag/sensors/imu_test.py
  python3 tools/diag/sensors/imu_test.py --hz 20 --csv logs/imu_$(date +%F_%H%M%S).csv
  python3 tools/diag/sensors/imu_test.py --bus 1 --addr 0x28 --duration 30
  python3 tools/diag/sensors/imu_test.py --ext-crystal

Requires:
  pip install smbus2
"""

import argparse
import csv
import os
import sys
import time
from datetime import datetime

try:
    from smbus2 import SMBus, i2c_msg
except Exception as e:
    print("ERROR: smbus2 not available. Install with: pip install smbus2", file=sys.stderr)
    raise

# -------- BNO055 registers / constants (datasheet) ----------
BNO055_ADDRESS_A       = 0x28
REG_CHIP_ID            = 0x00  # expected 0xA0
REG_PAGE_ID            = 0x07

REG_ACCEL_DATA_LSB     = 0x08
REG_MAG_DATA_LSB       = 0x0E
REG_GYRO_DATA_LSB      = 0x14
REG_EUL_DATA_LSB       = 0x1A
REG_QUA_DATA_LSB       = 0x20

REG_TEMP               = 0x34
REG_CALIB_STAT         = 0x35
REG_ST_RESULT          = 0x36
REG_SYS_STATUS         = 0x39
REG_SYS_ERR            = 0x3A
REG_UNIT_SEL           = 0x3B
REG_OPR_MODE           = 0x3D
REG_PWR_MODE           = 0x3E
REG_SYS_TRIGGER        = 0x3F

# Operation modes
MODE_CONFIG            = 0x00
MODE_NDOF              = 0x0C  # Fusion mode (absolute orientation)

# Power modes
PWR_NORMAL             = 0x00

# Unit selection bits (we want: m/s^2, dps, degrees, Celsius)
# bit0 ACC: 0=m/s^2, 1=mg
# bit1 GYR: 0=dps,  1=rps
# bit2 EUL: 0=deg,  1=rad
# bit4 TMP: 0=C,    1=F
UNIT_SEL_VALUE         = 0x00

# Scale factors per datasheet
ACC_LSB_MS2            = 1.0 / 100.0   # 1 LSB = 1 mg = 0.01 m/s^2 (when ACC in m/s^2 units)
GYR_LSB_DPS            = 1.0 / 16.0    # 1 LSB = 1/16 dps
EUL_LSB_DEG            = 1.0 / 16.0    # 1 LSB = 1/16 degree
MAG_LSB_UT             = 1.0 / 16.0    # 1 LSB = 1/16 uT
QUAT_LSB               = 1.0 / 16384.0 # per datasheet

def log(msg: str):
    print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}", flush=True)

class CsvLogger:
    def __init__(self, path: str, header: list[str]):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(header); self.f.flush()
    def row(self, *vals):
        self.w.writerow(vals); self.f.flush()
    def close(self):
        try:
            self.f.close()
        except Exception:
            pass

class BNO055:
    def __init__(self, bus: int = 1, addr: int = BNO055_ADDRESS_A):
        self.busno = bus
        self.addr = addr
        self.bus = SMBus(bus)

    # --- low-level helpers ---
    def write8(self, reg: int, val: int):
        self.bus.write_byte_data(self.addr, reg & 0xFF, val & 0xFF)

    def read8(self, reg: int) -> int:
        return self.bus.read_byte_data(self.addr, reg & 0xFF)

    def readlen(self, reg: int, length: int) -> bytes:
        write = i2c_msg.write(self.addr, [reg & 0xFF])
        read = i2c_msg.read(self.addr, length)
        self.bus.i2c_rdwr(write, read)
        return bytes(read)

    # --- device control ---
    def set_mode(self, mode: int):
        self.write8(REG_OPR_MODE, MODE_CONFIG)
        time.sleep(0.02)
        self.write8(REG_OPR_MODE, mode & 0xFF)
        # According to datasheet, after mode change, wait:
        time.sleep(0.02 if mode == MODE_CONFIG else 0.07)

    def init(self, use_ext_crystal=False):
        # Confirm chip
        chip = self.read8(REG_CHIP_ID)
        if chip != 0xA0:
            raise RuntimeError(f"BNO055 not found or wrong CHIP_ID (got 0x{chip:02X}, expected 0xA0) at 0x{self.addr:02X} on i2c-{self.busno}")

        # Go to config to set units
        self.set_mode(MODE_CONFIG)

        # Normal power
        self.write8(REG_PWR_MODE, PWR_NORMAL)
        time.sleep(0.01)

        # Units: m/s^2, dps, degrees, Celsius
        self.write8(REG_UNIT_SEL, UNIT_SEL_VALUE)
        time.sleep(0.01)

        # Optional: external crystal
        if use_ext_crystal:
            # Set EXT_CRYSTAL_EN in SYS_TRIGGER (bit 7)
            self.write8(REG_SYS_TRIGGER, 0x80)
            time.sleep(0.01)
        else:
            # Clear SYS_TRIGGER
            self.write8(REG_SYS_TRIGGER, 0x00)
            time.sleep(0.01)

        # Switch to NDOF (fusion)
        self.set_mode(MODE_NDOF)
        # Allow fusion to stabilize
        time.sleep(0.2)

    # --- data reads ---
    def _read_vector3(self, base_reg: int, scale: float):
        data = self.readlen(base_reg, 6)
        x = int.from_bytes(data[0:2], byteorder="little", signed=True) * scale
        y = int.from_bytes(data[2:4], byteorder="little", signed=True) * scale
        z = int.from_bytes(data[4:6], byteorder="little", signed=True) * scale
        return (x, y, z)

    def read_accel_ms2(self):
        return self._read_vector3(REG_ACCEL_DATA_LSB, ACC_LSB_MS2)

    def read_gyro_dps(self):
        return self._read_vector3(REG_GYRO_DATA_LSB, GYR_LSB_DPS)

    def read_mag_uT(self):
        return self._read_vector3(REG_MAG_DATA_LSB, MAG_LSB_UT)

    def read_euler_deg(self):
        heading, roll, pitch = (
            int.from_bytes(self.readlen(REG_EUL_DATA_LSB + i, 2), "little", signed=True) * EUL_LSB_DEG
            for i in (0, 2, 4)
        )
        return (heading, roll, pitch)

    def read_quaternion(self):
        data = self.readlen(REG_QUA_DATA_LSB, 8)
        w = int.from_bytes(data[0:2], "little", signed=True) * QUAT_LSB
        x = int.from_bytes(data[2:4], "little", signed=True) * QUAT_LSB
        y = int.from_bytes(data[4:6], "little", signed=True) * QUAT_LSB
        z = int.from_bytes(data[6:8], "little", signed=True) * QUAT_LSB
        return (w, x, y, z)

    def read_temp_c(self):
        # signed int8, °C
        t = self.read8(REG_TEMP)
        if t > 127:
            t -= 256
        return float(t)

    def read_calib_status(self):
        stat = self.read8(REG_CALIB_STAT)
        sys = (stat >> 6) & 0x03
        gyro = (stat >> 4) & 0x03
        accel = (stat >> 2) & 0x03
        mag = (stat >> 0) & 0x03
        return sys, gyro, accel, mag

    def read_sys_status(self):
        return self.read8(REG_SYS_STATUS), self.read8(REG_SYS_ERR)

    def read_selftest(self):
        res = self.read8(REG_ST_RESULT)
        # Bits: bit0:ACC, bit1:MAG, bit2:GYR, bit3:MCU (1=pass)
        return {
            "ACC": bool(res & 0x01),
            "MAG": bool(res & 0x02),
            "GYR": bool(res & 0x04),
            "MCU": bool(res & 0x08),
        }

def main():
    ap = argparse.ArgumentParser(description="BNO055 IMU diagnostic")
    ap.add_argument("--bus", type=int, default=1, help="I2C bus number (default: 1)")
    ap.add_argument("--addr", type=lambda x:int(x,0), default=BNO055_ADDRESS_A, help="I2C address (default: 0x28)")
    ap.add_argument("--hz", type=float, default=10.0, help="Sampling rate Hz (default: 10)")
    ap.add_argument("--duration", type=float, default=0.0, help="Stop after N seconds (0 = run until Ctrl+C)")
    ap.add_argument("--csv", type=str, default="", help="Optional CSV path to log samples")
    ap.add_argument("--ext-crystal", action="store_true", help="Enable external crystal (if fitted)")
    args = ap.parse_args()

    period = 1.0 / max(args.hz, 0.1)
    csv_logger = None

    log(f"Opening BNO055 on i2c-{args.bus} addr 0x{args.addr:02X}")
    imu = BNO055(bus=args.bus, addr=args.addr)
    imu.init(use_ext_crystal=args.ext_crystal)

    # First status dump
    chip = imu.read8(REG_CHIP_ID)
    st = imu.read_selftest()
    sys_status, sys_err = imu.read_sys_status()
    sys_c, gyr_c, acc_c, mag_c = imu.read_calib_status()

    log(f"CHIP_ID=0x{chip:02X}  SELFTEST: ACC={st['ACC']} MAG={st['MAG']} GYR={st['GYR']} MCU={st['MCU']}")
    log(f"SYS_STATUS={sys_status}  SYS_ERR={sys_err}  CALIB(sys,gyro,acc,mag)=({sys_c},{gyr_c},{acc_c},{mag_c})")

    if args.csv:
        header = [
            "ts","eul_heading_deg","eul_roll_deg","eul_pitch_deg",
            "quat_w","quat_x","quat_y","quat_z",
            "acc_x_ms2","acc_y_ms2","acc_z_ms2",
            "gyr_x_dps","gyr_y_dps","gyr_z_dps",
            "mag_x_uT","mag_y_uT","mag_z_uT",
            "temp_c","calib_sys","calib_gyro","calib_acc","calib_mag"
        ]
        csv_logger = CsvLogger(args.csv, header)
        log(f"Logging to CSV: {args.csv}")

    log("Streaming… Press Ctrl+C to stop.")
    start = time.time()
    n = 0
    try:
        while True:
            t0 = time.time()

            e_h, e_r, e_p = imu.read_euler_deg()
            q_w, q_x, q_y, q_z = imu.read_quaternion()
            a_x, a_y, a_z = imu.read_accel_ms2()
            g_x, g_y, g_z = imu.read_gyro_dps()
            m_x, m_y, m_z = imu.read_mag_uT()
            temp_c = imu.read_temp_c()
            sys_c, gyr_c, acc_c, mag_c = imu.read_calib_status()

            # Console output (succinct, one line)
            print(
                f"Eul(deg)=({e_h:7.2f},{e_r:7.2f},{e_p:7.2f})  "
                f"Acc(m/s^2)=({a_x:6.2f},{a_y:6.2f},{a_z:6.2f})  "
                f"Gyr(dps)=({g_x:7.2f},{g_y:7.2f},{g_z:7.2f})  "
                f"Mag(uT)=({m_x:6.2f},{m_y:6.2f},{m_z:6.2f})  "
                f"T={temp_c:5.1f}°C  Calib[{sys_c}{gyr_c}{acc_c}{mag_c}]",
                flush=True
            )

            if csv_logger:
                ts = datetime.now().isoformat(timespec="milliseconds")
                csv_logger.row(
                    ts, e_h, e_r, e_p, q_w, q_x, q_y, q_z,
                    a_x, a_y, a_z, g_x, g_y, g_z, m_x, m_y, m_z,
                    temp_c, sys_c, gyr_c, acc_c, mag_c
                )

            n += 1
            if args.duration and (time.time() - start) >= args.duration:
                break

            # Rate control
            dt = time.time() - t0
            sleep = period - dt
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        pass
    finally:
        if csv_logger:
            csv_logger.close()
        log(f"Done. Samples: {n}")

if __name__ == "__main__":
    main()
