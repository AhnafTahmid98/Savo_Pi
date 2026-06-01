#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot SAVO — imu_api.py (BNO055)
--------------------------------
Reusable IMU API extracted from tools/diag/sensors/imu_test.py and refactored for ROS2 nodes.

Recommended for Robot SAVO localization:
- IMU-only mode (acc + gyro) WITHOUT magnetometer (indoor + metal standoffs distort mag)

Provides:
- Accel in m/s^2
- Gyro in rad/s
- Optional (NDOF): fused Euler (deg) + quaternion + mag norm (NOT recommended unless mag environment is clean)
- Health/status: chip id, sys status/err, calibration tuple, temperature

Dependencies:
- smbus2
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple, Callable

try:
    from smbus2 import SMBus
except Exception as e:
    raise RuntimeError("smbus2 is required. Install with: pip3 install smbus2") from e


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
OPERATION_MODE_IMU       = 0x08   # acc + gyro
OPERATION_MODE_NDOF      = 0x0C   # fusion + mag (indoor often noisy)

# Units: metric + deg + dps + °C
UNIT_SEL_METRIC_DEG_DPS_C = 0b00000000

# --- Correct scales (datasheet with above units) ---
ACC_SCALE = 1.0 / 100.0   # m/s² per LSB
GYR_SCALE = 1.0 / 16.0    # dps per LSB
EUL_SCALE = 1.0 / 16.0    # deg per LSB
MAG_SCALE = 1.0 / 16.0    # µT per LSB

DEG2RAD = math.pi / 180.0


@dataclass
class ImuStatus:
    chip_ok: bool
    chip_id: int
    sys_status: int
    sys_err: int
    calib_sys: int
    calib_gyr: int
    calib_acc: int
    calib_mag: int
    temp_c: float


@dataclass
class ImuSample:
    t_monotonic: float

    ax: float
    ay: float
    az: float

    gx: float
    gy: float
    gz: float

    qx: Optional[float] = None
    qy: Optional[float] = None
    qz: Optional[float] = None
    qw: Optional[float] = None

    yaw_deg: Optional[float] = None
    roll_deg: Optional[float] = None
    pitch_deg: Optional[float] = None

    mag_uT: Optional[float] = None

    status: Optional[ImuStatus] = None


def _mag3(x: float, y: float, z: float) -> float:
    return math.sqrt(x*x + y*y + z*z)


def _euler_deg_to_quat(yaw_deg: float, roll_deg: float, pitch_deg: float) -> Tuple[float, float, float, float]:
    """
    Convert Euler angles (degrees) to quaternion (x,y,z,w).
    Compose: q = qz(yaw) * qx(roll) * qy(pitch)
    """
    yaw = yaw_deg * DEG2RAD
    roll = roll_deg * DEG2RAD
    pitch = pitch_deg * DEG2RAD

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    qw = cy*cr*cp + sy*sr*sp
    qx = cy*sr*cp + sy*cr*sp
    qy = cy*cr*sp - sy*sr*cp
    qz = sy*cr*cp - cy*sr*sp
    return (qx, qy, qz, qw)


class BNO055:
    """Minimal BNO055 driver for production node usage (I2C)."""

    def __init__(self, bus: int = 1, addr: int = BNO055_ADDRESS_A):
        self.addr = addr
        self.bus_num = bus
        self.bus = SMBus(bus)

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass

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

    def initialize(self, imu_only: bool = True):
        self.set_mode(OPERATION_MODE_CONFIG)
        self.write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL)
        time.sleep(0.01)
        self.set_page(0)
        self.write8(BNO055_UNIT_SEL_ADDR, UNIT_SEL_METRIC_DEG_DPS_C)
        time.sleep(0.01)
        self.write8(BNO055_SYS_TRIGGER_ADDR, 0x00)
        time.sleep(0.01)
        self.set_mode(OPERATION_MODE_IMU if imu_only else OPERATION_MODE_NDOF)

    # status
    def chip_id(self) -> int:      return self.read8(BNO055_CHIP_ID_ADDR)
    def sys_status(self) -> int:   return self.read8(BNO055_SYS_STATUS_ADDR)
    def sys_err(self) -> int:      return self.read8(BNO055_SYS_ERR_ADDR)

    def calib_tuple(self) -> Tuple[int, int, int, int]:
        v = self.read8(BNO055_CALIB_STAT_ADDR)
        return ((v >> 6) & 3, (v >> 4) & 3, (v >> 2) & 3, v & 3)

    def temperature_c(self) -> float:
        t = self.read8(BNO055_TEMP_ADDR)
        if t > 127:
            t -= 256
        return float(t)

    # reads
    def read_accel_m_s2(self) -> Tuple[float, float, float]:
        return self.read_vec3(BNO055_ACCEL_DATA_X_LSB, ACC_SCALE)

    def read_gyro_rad_s(self) -> Tuple[float, float, float]:
        gx_dps, gy_dps, gz_dps = self.read_vec3(BNO055_GYRO_DATA_X_LSB, GYR_SCALE)
        return (gx_dps * DEG2RAD, gy_dps * DEG2RAD, gz_dps * DEG2RAD)

    def read_mag_uT(self) -> Tuple[float, float, float]:
        return self.read_vec3(BNO055_MAG_DATA_X_LSB, MAG_SCALE)

    def read_euler_deg(self) -> Tuple[float, float, float]:
        h = self.read16s(BNO055_EULER_H_LSB) * EUL_SCALE
        r = self.read16s(BNO055_EULER_H_LSB + 2) * EUL_SCALE
        p = self.read16s(BNO055_EULER_H_LSB + 4) * EUL_SCALE
        return (h, r, p)


class ImuApi:
    """
    High-level API wrapper used by ROS node.

    Professional defaults for Robot SAVO:
      imu_only=True (no magnetometer)
      with_status=False in fast loop (status read is extra I2C traffic)

    Optional axis remap:
      remap_fn(ax,ay,az,gx,gy,gz) -> remapped tuple
    """

    def __init__(
        self,
        bus: int = 1,
        addr: int = BNO055_ADDRESS_A,
        imu_only: bool = True,
        read_retries: int = 2,
        retry_delay_s: float = 0.002,
        remap_fn: Optional[Callable[[float, float, float, float, float, float],
                                    Tuple[float, float, float, float, float, float]]] = None,
    ):
        self.bus = bus
        self.addr = addr
        self.imu_only = imu_only
        self.read_retries = max(0, int(read_retries))
        self.retry_delay_s = max(0.0, float(retry_delay_s))
        self.remap_fn = remap_fn

        self._dev: Optional[BNO055] = None
        self._started = False

    def __enter__(self) -> "ImuApi":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()

    def is_started(self) -> bool:
        return bool(self._started and self._dev is not None)

    def start(self, do_reset: bool = True):
        self._dev = BNO055(bus=self.bus, addr=self.addr)
        chip = self._dev.chip_id()
        if chip != BNO055_ID:
            self._dev.close()
            self._dev = None
            raise RuntimeError(
                f"BNO055 CHIP_ID mismatch: got 0x{chip:02X}, expected 0x{BNO055_ID:02X} "
                f"(bus={self.bus}, addr={hex(self.addr)})"
            )

        if do_reset:
            self._dev.reset()

        self._dev.initialize(imu_only=self.imu_only)
        self._started = True

    def stop(self):
        if self._dev:
            self._dev.close()
        self._dev = None
        self._started = False

    def read_status(self) -> ImuStatus:
        if not self._dev:
            raise RuntimeError("ImuApi not started")

        chip = self._dev.chip_id()
        sys_stat = self._dev.sys_status()
        sys_err = self._dev.sys_err()
        c_sys, c_gyr, c_acc, c_mag = self._dev.calib_tuple()
        temp_c = self._dev.temperature_c()

        return ImuStatus(
            chip_ok=(chip == BNO055_ID),
            chip_id=chip,
            sys_status=sys_stat,
            sys_err=sys_err,
            calib_sys=c_sys,
            calib_gyr=c_gyr,
            calib_acc=c_acc,
            calib_mag=c_mag,
            temp_c=temp_c,
        )

    def _read_once(self) -> ImuSample:
        assert self._dev is not None

        t = time.monotonic()
        ax, ay, az = self._dev.read_accel_m_s2()
        gx, gy, gz = self._dev.read_gyro_rad_s()

        if self.remap_fn is not None:
            ax, ay, az, gx, gy, gz = self.remap_fn(ax, ay, az, gx, gy, gz)

        yaw = roll = pitch = None
        qx = qy = qz = qw = None
        mag_norm = None

        if not self.imu_only:
            # NDOF fields are best-effort (do not crash the caller)
            try:
                yaw, roll, pitch = self._dev.read_euler_deg()
                qx, qy, qz, qw = _euler_deg_to_quat(yaw, roll, pitch)
            except Exception:
                pass

            try:
                mx, my, mz = self._dev.read_mag_uT()
                mag_norm = _mag3(mx, my, mz)
            except Exception:
                pass

        return ImuSample(
            t_monotonic=t,
            ax=ax, ay=ay, az=az,
            gx=gx, gy=gy, gz=gz,
            qx=qx, qy=qy, qz=qz, qw=qw,
            yaw_deg=yaw, roll_deg=roll, pitch_deg=pitch,
            mag_uT=mag_norm,
            status=None,
        )

    def read_sample(self, with_status: bool = False) -> ImuSample:
        if not self._dev or not self._started:
            raise RuntimeError("ImuApi not started")

        last_exc: Optional[Exception] = None
        for attempt in range(self.read_retries + 1):
            try:
                s = self._read_once()
                if with_status:
                    s.status = self.read_status()
                return s
            except Exception as e:
                last_exc = e
                if attempt < self.read_retries and self.retry_delay_s > 0.0:
                    time.sleep(self.retry_delay_s)

        raise RuntimeError(f"IMU read failed after {self.read_retries + 1} attempts: {last_exc}") from last_exc