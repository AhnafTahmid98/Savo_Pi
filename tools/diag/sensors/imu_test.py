#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU (BNO055) diagnostic for Robot Savo
- NDOF init + unit config (m/s^2, dps, deg, °C)
- Streaming readout (Euler, quat, accel, gyro, mag, temp, calib)
- CSV logging optional
- NEW: Health Summary (--health / --health-only)

Usage:
  # normal stream
  python3 tools/diag/sensors/imu_test.py
  # faster + CSV
  python3 tools/diag/sensors/imu_test.py --hz 20 --csv logs/imu_$(date +%F_%H%M%S).csv
  # health check only (quick PASS/WARN/FAIL and exit)
  python3 tools/diag/sensors/imu_test.py --health-only
"""

import argparse, csv, os, sys, time, statistics
from datetime import datetime

try:
    from smbus2 import SMBus, i2c_msg
except Exception:
    print("ERROR: smbus2 not available. Install with: pip install smbus2", file=sys.stderr)
    raise

# -------- BNO055 registers / constants ----------
BNO055_ADDRESS_A       = 0x28
REG_CHIP_ID            = 0x00  # expect 0xA0
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

MODE_CONFIG            = 0x00
MODE_NDOF              = 0x0C
PWR_NORMAL             = 0x00
UNIT_SEL_VALUE         = 0x00  # m/s^2, dps, deg, °C

ACC_LSB_MS2            = 1.0 / 100.0
GYR_LSB_DPS            = 1.0 / 16.0
EUL_LSB_DEG            = 1.0 / 16.0
MAG_LSB_UT             = 1.0 / 16.0
QUAT_LSB               = 1.0 / 16384.0

def log(msg): print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}", flush=True)

class CsvLogger:
    def __init__(self, path, header):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f); self.w.writerow(header); self.f.flush()
    def row(self, *vals): self.w.writerow(vals); self.f.flush()
    def close(self): 
        try: self.f.close()
        except Exception: pass

class BNO055:
    def __init__(self, bus=1, addr=BNO055_ADDRESS_A):
        self.busno = bus; self.addr = addr; self.bus = SMBus(bus)
    def write8(self, reg, val): self.bus.write_byte_data(self.addr, reg & 0xFF, val & 0xFF)
    def read8(self, reg): return self.bus.read_byte_data(self.addr, reg & 0xFF)
    def readlen(self, reg, length):
        w = i2c_msg.write(self.addr, [reg & 0xFF]); r = i2c_msg.read(self.addr, length)
        self.bus.i2c_rdwr(w, r); return bytes(r)
    def set_mode(self, mode):
        self.write8(REG_OPR_MODE, MODE_CONFIG); time.sleep(0.02)
        self.write8(REG_OPR_MODE, mode & 0xFF); time.sleep(0.02 if mode == MODE_CONFIG else 0.07)
    def init(self, use_ext_crystal=False):
        chip = self.read8(REG_CHIP_ID)
        if chip != 0xA0:
            raise RuntimeError(f"BNO055 wrong CHIP_ID 0x{chip:02X} @0x{self.addr:02X} on i2c-{self.busno} (expect 0xA0)")
        self.set_mode(MODE_CONFIG)
        self.write8(REG_PWR_MODE, PWR_NORMAL); time.sleep(0.01)
        self.write8(REG_UNIT_SEL, UNIT_SEL_VALUE); time.sleep(0.01)
        self.write8(REG_SYS_TRIGGER, 0x80 if use_ext_crystal else 0x00); time.sleep(0.01)
        self.set_mode(MODE_NDOF); time.sleep(0.2)
    def _vec3(self, base, scale):
        d = self.readlen(base, 6)
        x = int.from_bytes(d[0:2], "little", signed=True) * scale
        y = int.from_bytes(d[2:4], "little", signed=True) * scale
        z = int.from_bytes(d[4:6], "little", signed=True) * scale
        return x, y, z
    def read_accel_ms2(self): return self._vec3(REG_ACCEL_DATA_LSB, ACC_LSB_MS2)
    def read_gyro_dps(self):  return self._vec3(REG_GYRO_DATA_LSB,  GYR_LSB_DPS)
    def read_mag_uT(self):    return self._vec3(REG_MAG_DATA_LSB,  MAG_LSB_UT)
    def read_euler_deg(self):
        h = int.from_bytes(self.readlen(REG_EUL_DATA_LSB+0, 2), "little", signed=True)*EUL_LSB_DEG
        r = int.from_bytes(self.readlen(REG_EUL_DATA_LSB+2, 2), "little", signed=True)*EUL_LSB_DEG
        p = int.from_bytes(self.readlen(REG_EUL_DATA_LSB+4, 2), "little", signed=True)*EUL_LSB_DEG
        return h, r, p
    def read_quaternion(self):
        d = self.readlen(REG_QUA_DATA_LSB, 8)
        return (
            int.from_bytes(d[0:2], "little", signed=True)*QUAT_LSB,
            int.from_bytes(d[2:4], "little", signed=True)*QUAT_LSB,
            int.from_bytes(d[4:6], "little", signed=True)*QUAT_LSB,
            int.from_bytes(d[6:8], "little", signed=True)*QUAT_LSB,
        )
    def read_temp_c(self):
        t = self.read8(REG_TEMP)
        return float(t-256 if t>127 else t)
    def read_calib_status(self):
        s = self.read8(REG_CALIB_STAT)
        return (s>>6)&3, (s>>4)&3, (s>>2)&3, s&3   # sys, gyro, accel, mag
    def read_sys_status(self): return self.read8(REG_SYS_STATUS), self.read8(REG_SYS_ERR)
    def read_selftest(self):
        r = self.read8(REG_ST_RESULT)
        return {"ACC": bool(r&1), "MAG": bool(r&2), "GYR": bool(r&4), "MCU": bool(r&8)}

# ---------------- Health check helpers ----------------
def classify(val, ok_range, warn_range):
    lo_ok, hi_ok = ok_range; lo_w, hi_w = warn_range
    if lo_ok <= val <= hi_ok: return "PASS"
    if lo_w  <= val <= hi_w:  return "WARN"
    return "FAIL"

def print_health_table(rows):
    # rows: list of (name, value_str, status, note)
    name_w = max(len(r[0]) for r in rows)+2
    val_w  = max(len(r[1]) for r in rows)+2
    print("\nIMU Health Summary")
    print("-"* (name_w+val_w+12))
    print(f"{'Check'.ljust(name_w)}{'Value'.ljust(val_w)}Status   Note")
    print("-"* (name_w+val_w+12))
    for n, v, s, note in rows:
        print(f"{n.ljust(name_w)}{v.ljust(val_w)}{s:<7} {note}")
    print("-"* (name_w+val_w+12))

def health_check(imu, samples=60, hz=20.0):
    # quick ramp: allow fusion to settle
    time.sleep(0.2)
    dt = 1.0/max(hz, 1.0)
    acc_mag, gyr_abs = [], []
    mag_mag = []
    eul_roll, eul_pitch = [], []
    for _ in range(samples):
        ax, ay, az = imu.read_accel_ms2()
        gx, gy, gz = imu.read_gyro_dps()
        mx, my, mz = imu.read_mag_uT()
        _, r, p = imu.read_euler_deg()
        acc_mag.append((ax*ax + ay*ay + az*az) ** 0.5)
        gyr_abs.append((abs(gx)+abs(gy)+abs(gz))/3.0)
        mag_mag.append((mx*mx + my*my + mz*mz) ** 0.5)
        eul_roll.append(abs(r)); eul_pitch.append(abs(p))
        time.sleep(dt)

    # stats
    acc_mean = statistics.fmean(acc_mag)
    acc_std  = statistics.pstdev(acc_mag)
    gyr_mean = statistics.fmean(gyr_abs)
    mag_mean = statistics.fmean(mag_mag)
    roll_mean = statistics.fmean(eul_roll)
    pitch_mean= statistics.fmean(eul_pitch)

    chip = imu.read8(REG_CHIP_ID)
    st = imu.read_selftest()
    sys_status, sys_err = imu.read_sys_status()
    sys_c, gyr_c, acc_c, mag_c = imu.read_calib_status()

    rows = []

    rows.append(("Chip ID", f"0x{chip:02X}", "PASS" if chip==0xA0 else "FAIL", "Should be 0xA0"))
    rows.append(("Self-test", f"ACC={st['ACC']} MAG={st['MAG']} GYR={st['GYR']} MCU={st['MCU']}",
                 "PASS" if all(st.values()) else "FAIL", "All True expected"))
    rows.append(("System", f"status={sys_status} err={sys_err}",
                 "PASS" if (sys_status==5 and sys_err==0) else ("WARN" if sys_err==0 else "FAIL"),
                 "5=NDOF; err=0"))

    # Accel magnitude near gravity
    acc_status = classify(acc_mean, ok_range=(9.5, 10.1), warn_range=(9.0, 10.6))
    rows.append(("Accel |g|", f"{acc_mean:.2f} ±{acc_std:.2f} m/s²", acc_status, "Target 9.8"))
    # Gyro mean absolute (still)
    gyr_status = classify(gyr_mean, ok_range=(0.0, 0.3), warn_range=(0.3, 0.7))
    rows.append(("Gyro bias", f"{gyr_mean:.2f} dps", gyr_status, "Should be near 0 when still"))
    # Mag magnitude (Earth field ~45–70 µT; indoors can vary)
    mag_status = classify(mag_mean, ok_range=(35, 80), warn_range=(25, 100))
    rows.append(("Mag |B|", f"{mag_mean:.1f} µT", mag_status, "Earth field range"))

    # Orientation offsets (board roughly level)
    rows.append(("Roll |mean|", f"{roll_mean:.1f}°",
                 "PASS" if roll_mean <= 5 else "WARN", "Small if robot is level"))
    rows.append(("Pitch |mean|", f"{pitch_mean:.1f}°",
                 "PASS" if pitch_mean <= 5 else "WARN", "Small if robot is level"))

    # Calibration bits
    cal_note = "目标 3333 (sys,gyro,acc,mag)"  # target 3333
    cal_status = "PASS" if (sys_c==3 and gyr_c==3 and acc_c==3 and mag_c==3) else ("WARN" if gyr_c==3 else "WARN")
    rows.append(("Calib bits", f"{sys_c}{gyr_c}{acc_c}{mag_c}", cal_status, cal_note))

    print_health_table(rows)

    # Overall decision: FAIL if any FAIL; else WARN if any WARN; else PASS
    statuses = [r[2] for r in rows]
    overall = "PASS"
    if "FAIL" in statuses: overall = "FAIL"
    elif "WARN" in statuses: overall = "WARN"

    print(f"Overall: {overall}")
    return overall

# ---------------- Main ----------------
def main():
    ap = argparse.ArgumentParser(description="BNO055 IMU diagnostic")
    ap.add_argument("--bus", type=int, default=1, help="I2C bus (default 1)")
    ap.add_argument("--addr", type=lambda x:int(x,0), default=BNO055_ADDRESS_A, help="I2C addr (default 0x28)")
    ap.add_argument("--hz", type=float, default=10.0, help="Stream Hz (default 10)")
    ap.add_argument("--duration", type=float, default=0.0, help="Stop after seconds (0=until Ctrl+C)")
    ap.add_argument("--csv", type=str, default="", help="CSV log path")
    ap.add_argument("--ext-crystal", action="store_true", help="Enable external crystal")
    ap.add_argument("--health", action="store_true", help="Print health summary before streaming")
    ap.add_argument("--health-only", action="store_true", help="Only run health summary and exit")
    ap.add_argument("--health-samples", type=int, default=60, help="Samples for health check (default 60)")
    ap.add_argument("--health-hz", type=float, default=20.0, help="Sampling Hz for health check (default 20)")
    args = ap.parse_args()

    log(f"Opening BNO055 on i2c-{args.bus} addr 0x{args.addr:02X}")
    imu = BNO055(bus=args.bus, addr=args.addr)
    imu.init(use_ext_crystal=args.ext_crystal)

    # Always print initial low-level status
    chip = imu.read8(REG_CHIP_ID)
    st = imu.read_selftest()
    sys_status, sys_err = imu.read_sys_status()
    sys_c, gyr_c, acc_c, mag_c = imu.read_calib_status()
    log(f"CHIP_ID=0x{chip:02X}  SELFTEST: ACC={st['ACC']} MAG={st['MAG']} GYR={st['GYR']} MCU={st['MCU']}")
    log(f"SYS_STATUS={sys_status}  SYS_ERR={sys_err}  CALIB(sys,gyro,acc,mag)=({sys_c},{gyr_c},{acc_c},{mag_c})")

    # Health summary mode
    if args.health or args.health_only:
        overall = health_check(imu, samples=args.health_samples, hz=args.health_hz)
        if args.health_only:
            sys.exit(0 if overall == "PASS" else 1)

    # CSV logging setup
    csv_logger = None
    if args.csv:
        header = ["ts","eul_heading_deg","eul_roll_deg","eul_pitch_deg",
                  "quat_w","quat_x","quat_y","quat_z",
                  "acc_x_ms2","acc_y_ms2","acc_z_ms2",
                  "gyr_x_dps","gyr_y_dps","gyr_z_dps",
                  "mag_x_uT","mag_y_uT","mag_z_uT",
                  "temp_c","calib_sys","calib_gyro","calib_acc","calib_mag"]
        csv_logger = CsvLogger(args.csv, header)
        log(f"Logging to CSV: {args.csv}")

    # Stream loop
    period = 1.0 / max(args.hz, 0.1)
    log("Streaming… Press Ctrl+C to stop.")
    start = time.time(); n = 0
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
                csv_logger.row(ts, e_h, e_r, e_p, q_w, q_x, q_y, q_z,
                               a_x, a_y, a_z, g_x, g_y, g_z, m_x, m_y, m_z,
                               temp_c, sys_c, gyr_c, acc_c, mag_c)

            n += 1
            if args.duration and (time.time() - start) >= args.duration:
                break

            sleep = period - (time.time() - t0)
            if sleep > 0: time.sleep(sleep)
    except KeyboardInterrupt:
        pass
    finally:
        if csv_logger: csv_logger.close()
        log(f"Done. Samples: {n}")

if __name__ == "__main__":
    main()
