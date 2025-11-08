#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Motor Direction Test (PCA9685-only + Encoder Polling)
------------------------------------------------------------------

Purpose
  Verify our sign convention: when a side spins FORWARD, its encoder delta is POSITIVE.
  This locks polarity before higher-level teleop/ROS2 work.

What this script assumes
  • Your motor HAT exposes each motor’s PWM, IN1, IN2 as **PCA9685 channels** (I²C @ 0x40).
  • Encoders are wired (LOCKED) to BCM pins:
        Left  : L_A=21, L_B=20
        Right : R_A=12, R_B=26
    with **external 10 kΩ pull-ups** (so use --no-pullup).
  • Robot is on blocks (wheels free).

How it works
  • FORWARD  = IN1=1, IN2=0, PWM=duty
  • REVERSE  = IN1=0, IN2=1, PWM=duty
  • STOP     = IN1=0, IN2=0, PWM=0
  • Encoders are sampled at ~1 kHz (software debounce).

Examples (EDIT CHANNELS to your HAT mapping)
  # Manual prompts (you power motors externally)
  python3 tools/diag/motion/motor_direction_test.py --chip 4 --no-pullup

  # PCA9685 auto (all control over I²C; no GPIO dir pins)
  python3 tools/diag/motion/motor_direction_test.py \
    --chip 4 --no-pullup \
    --driver pca --i2c-bus 1 --pwm-addr 0x40 --pwm-freq 1000 \
    --m1-pwm-ch 8  --m1-in1-ch 9  --m1-in2-ch 10 \
    --m2-pwm-ch 11 --m2-in1-ch 12 --m2-in2-ch 13 \
    --m3-pwm-ch 2  --m3-in1-ch 3  --m3-in2-ch 4  \
    --m4-pwm-ch 5  --m4-in1-ch 6  --m4-in2-ch 7  \
    --left-group 0,2 --right-group 1,3 --duty 0.45 --seconds 2.0
"""

import sys
import time
import argparse
from dataclasses import dataclass
from typing import Tuple, List, Optional

# ======================== Encoder polling via lgpio ===========================
try:
    import lgpio
except Exception as e:
    print("ERROR: lgpio is required. Install: sudo apt install -y python3-lgpio", file=sys.stderr)
    raise

# LOCKED encoder pins (BCM)
L_A, L_B = 21, 20
R_A, R_B = 12, 26

GRAY_NEXT = {
    (0, 1): +1, (1, 3): +1, (3, 2): +1, (2, 0): +1,
    (1, 0): -1, (3, 1): -1, (2, 3): -1, (0, 2): -1,
}

@dataclass
class QuadPoll:
    chip: int
    a: int
    b: int
    debounce_s: float
    count: int = 0
    last_state: int = 0
    illegal: int = 0
    _last_change_a: float = 0.0
    _last_change_b: float = 0.0
    _state_a: int = 0
    _state_b: int = 0

    def init_state(self):
        # claim as inputs
        lgpio.gpio_claim_input(self.chip, self.a)
        lgpio.gpio_claim_input(self.chip, self.b)
        self._state_a = lgpio.gpio_read(self.chip, self.a)
        self._state_b = lgpio.gpio_read(self.chip, self.b)
        self.last_state = (self._state_a << 1) | self._state_b
        now = time.monotonic()
        self._last_change_a = now
        self._last_change_b = now

    def step(self, now: float):
        a_val = lgpio.gpio_read(self.chip, self.a)
        b_val = lgpio.gpio_read(self.chip, self.b)

        # software debounce
        if self.debounce_s > 0:
            if a_val != self._state_a:
                if (now - self._last_change_a) >= self.debounce_s:
                    self._state_a = a_val
                    self._last_change_a = now
                else:
                    a_val = self._state_a
            else:
                self._last_change_a = now

            if b_val != self._state_b:
                if (now - self._last_change_b) >= self.debounce_s:
                    self._state_b = b_val
                    self._last_change_b = now
                else:
                    b_val = self._state_b
            else:
                self._last_change_b = now

        s_new = (a_val << 1) | b_val
        if s_new != self.last_state:
            d = GRAY_NEXT.get((self.last_state, s_new))
            if d is None:
                self.illegal += 1
            else:
                self.count += d
            self.last_state = s_new

@dataclass
class Encoders:
    chip: int
    L: QuadPoll
    R: QuadPoll

def encoders_open_poll(chip_idx: Optional[int], pullup: bool, poll_debounce_s: float=0.0003) -> Encoders:
    # open a gpiochip
    chip = None
    if chip_idx is not None:
        chip = lgpio.gpiochip_open(chip_idx)
    else:
        for n in range(8):
            try:
                chip = lgpio.gpiochip_open(n)
                break
            except Exception:
                pass
    if chip is None:
        raise RuntimeError("No gpiochip available")

    # internal bias (skip if you have 10k externals with --no-pullup)
    if pullup:
        for pin in (L_A, L_B, R_A, R_B):
            try:
                lgpio.gpio_set_flags(chip, pin, lgpio.BIAS_PULL_UP)
            except Exception:
                pass

    L = QuadPoll(chip, L_A, L_B, debounce_s=poll_debounce_s)
    R = QuadPoll(chip, R_A, R_B, debounce_s=poll_debounce_s)
    L.init_state(); R.init_state()
    return Encoders(chip, L, R)

def encoders_close(E: Encoders):
    try:
        lgpio.gpiochip_close(E.chip)
    except Exception:
        pass

# ========================= PCA9685 low-level (I²C only) =======================
class PCA9685:
    MODE1      = 0x00
    MODE2      = 0x01
    LED0_ON_L  = 0x06
    PRE_SCALE  = 0xFE

    RESTART    = 0x80
    SLEEP      = 0x10
    ALLCALL    = 0x01
    OUTDRV     = 0x04
    AI         = 0x20     # Auto-Increment

    FULL_ON_H  = 0x10
    FULL_OFF_H = 0x10

    def __init__(self, bus, addr: int):
        self.bus  = bus
        self.addr = addr
        # MODE1: ALLCALL + Auto-Increment, wake
        self._write(self.MODE1, self.ALLCALL | self.AI)
        time.sleep(0.005)
        # MODE2: OUTDRV (totem pole)
        self._write(self.MODE2, self.OUTDRV)

    def set_freq(self, hz: int):
        hz = max(40, min(1500, int(hz)))
        prescale = int(round(25_000_000.0 / (4096 * hz)) - 1)
        oldmode = self._read(self.MODE1)
        sleep   = (oldmode & ~self.RESTART) | self.SLEEP
        self._write(self.MODE1, sleep)
        self._write(self.PRE_SCALE, prescale)
        self._write(self.MODE1, oldmode)
        time.sleep(0.005)
        self._write(self.MODE1, oldmode | self.RESTART | self.AI)

    def set_pwm_duty(self, ch: int, duty_12bit: int):
        """0..4095; 0 = full off, 4095 = full on, in-between = PWM."""
        duty = max(0, min(4095, int(duty_12bit)))
        reg  = self.LED0_ON_L + 4 * ch
        if duty == 0:
            # full off
            payload = [0x00, 0x00, 0x00, self.FULL_OFF_H]
        elif duty >= 4095:
            # full on
            payload = [0x00, self.FULL_ON_H, 0x00, 0x00]
        else:
            on_l, on_h = 0x00, 0x00
            off_l      = duty & 0xFF
            off_h      = (duty >> 8) & 0x0F
            payload    = [on_l, on_h, off_l, off_h]
        self.bus.write_i2c_block_data(self.addr, reg, payload)

    def _write(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val)

    def _read(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

# ============================= Driver (PCA-only) ==============================
class BaseDriver:
    def enable(self): pass
    def disable(self): pass
    def stop(self): pass

class PCAMotorDriver(BaseDriver):
    """
    Pure-PCA9685 H-bridge control:
      Each motor uses 3 PCA9685 channels: PWM, IN1, IN2.
      Direction via forcing IN1/IN2 full-on/full-off; speed on PWM.
    """
    def __init__(self,
                 i2c_bus: int, addr: int, freq: int,
                 m_pwm_ch: List[int], m_in1_ch: List[int], m_in2_ch: List[int],
                 duty_limit: float = 1.0):
        try:
            from smbus2 import SMBus  # noqa: F401
        except Exception as e:
            raise RuntimeError("smbus2 is required. Install: sudo apt install -y python3-smbus2") from e
        from smbus2 import SMBus

        assert len(m_pwm_ch) == len(m_in1_ch) == len(m_in2_ch) == 4, "Need 4 motors (FL, FR, RL, RR)"

        self.bus = SMBus(i2c_bus)
        self.pca = PCA9685(self.bus, addr)
        self.pca.set_freq(freq)

        self.pwm = m_pwm_ch
        self.in1 = m_in1_ch
        self.in2 = m_in2_ch
        self.duty_limit = max(0.0, min(1.0, duty_limit))
        self.stop()

    def _motor_cmd(self, idx: int, val_norm: float):
        val = max(-1.0, min(1.0, float(val_norm))) * self.duty_limit
        duty = int(abs(val) * 4095)

        if val > 0:
            # Forward: IN1=1, IN2=0
            self.pca.set_pwm_duty(self.in1[idx], 4095)
            self.pca.set_pwm_duty(self.in2[idx], 0)
            self.pca.set_pwm_duty(self.pwm[idx], duty)
        elif val < 0:
            # Reverse: IN1=0, IN2=1
            self.pca.set_pwm_duty(self.in1[idx], 0)
            self.pca.set_pwm_duty(self.in2[idx], 4095)
            self.pca.set_pwm_duty(self.pwm[idx], duty)
        else:
            # Stop
            self.pca.set_pwm_duty(self.pwm[idx], 0)
            self.pca.set_pwm_duty(self.in1[idx], 0)
            self.pca.set_pwm_duty(self.in2[idx], 0)

    def drive_side(self, indices: Tuple[int, ...], val_norm: float):
        for i in indices:
            self._motor_cmd(i, val_norm)

    def stop(self):
        for ch in (self.pwm + self.in1 + self.in2):
            self.pca.set_pwm_duty(ch, 0)

    def disable(self):
        self.stop()

# ================================ Test Logic ==================================
def measure_poll(E: Encoders, seconds: float) -> Tuple[int, int, int, int]:
    L0, R0  = E.L.count, E.R.count
    iL0, iR0 = E.L.illegal, E.R.illegal
    t0 = time.monotonic()
    # poll at ~1 kHz
    while (time.monotonic() - t0) < seconds:
        now = time.monotonic()
        E.L.step(now); E.R.step(now)
        time.sleep(0.001)
    dL, dR = E.L.count - L0, E.R.count - R0
    iL, iR = E.L.illegal - iL0, E.R.illegal - iR0
    return dL, dR, iL, iR

def print_step(name: str, dL: int, dR: int, iL: int, iR: int, expect_side: str):
    active = dL if expect_side == 'L' else dR
    sign = '+' if active > 0 else '-' if active < 0 else '0'
    print(f"{name:<16} | ΔL={dL:>6} ΔR={dR:>6}  sign({expect_side})={sign}  illegal(L/R)={iL}/{iR}")

def suggest(forwL: int, forwR: int) -> str:
    invL = forwL < 0
    invR = forwR < 0
    if invL and invR: return "Suggestion: set --invert-left --invert-right (in your control stack)"
    if invL:          return "Suggestion: set --invert-left (in your control stack)"
    if invR:          return "Suggestion: set --invert-right (in your control stack)"
    return "Suggestion: no inversion needed (forward is positive on both sides)"

# ================================== Main ======================================
def main():
    ap = argparse.ArgumentParser(description="Motor direction test via PCA9685 (no GPIO dirs) + encoder polling")
    # Mode & timings
    ap.add_argument('--driver', choices=['manual','pca'], default='manual',
                    help="manual: you power motors; pca: use PCA9685 channels for PWM+IN1+IN2")
    ap.add_argument('--seconds', type=float, default=2.0, help='Observation window per step')
    ap.add_argument('--settle',  type=float, default=0.6, help='Idle settle time between steps')
    ap.add_argument('--duty',    type=float, default=0.35, help='Normalized duty in auto mode (0..1)')

    # PCA9685 addressing
    ap.add_argument('--i2c-bus',  type=int, default=1)
    ap.add_argument('--pwm-addr', type=lambda x:int(x,0), default=0x40)
    ap.add_argument('--pwm-freq', type=int, default=1000)

    # Per-motor PCA9685 channels (PWM, IN1, IN2) — EDIT to your HAT mapping
    ap.add_argument('--m1-pwm-ch',  type=int, default=0)
    ap.add_argument('--m1-in1-ch',  type=int, default=1)
    ap.add_argument('--m1-in2-ch',  type=int, default=2)
    ap.add_argument('--m2-pwm-ch',  type=int, default=3)
    ap.add_argument('--m2-in1-ch',  type=int, default=4)
    ap.add_argument('--m2-in2-ch',  type=int, default=5)
    ap.add_argument('--m3-pwm-ch',  type=int, default=6)
    ap.add_argument('--m3-in1-ch',  type=int, default=7)
    ap.add_argument('--m3-in2-ch',  type=int, default=8)
    ap.add_argument('--m4-pwm-ch',  type=int, default=9)
    ap.add_argument('--m4-in1-ch',  type=int, default=10)
    ap.add_argument('--m4-in2-ch',  type=int, default=11)

    # Grouping by side (indices 0..3 = M1..M4)
    ap.add_argument('--left-group',  type=str, default='0,2', help='Motor indices for LEFT side (e.g., 0,2)')
    ap.add_argument('--right-group', type=str, default='1,3', help='Motor indices for RIGHT side (e.g., 1,3)')

    # Encoders / gpiochip options
    ap.add_argument('--chip', type=int, default=None, help='gpiochip index (use the one where 12/20/21/26 are unused, e.g., 4)')
    ap.add_argument('--no-pullup', action='store_true', help='Skip internal pull-ups (you have 10k external)')
    ap.add_argument('--poll-debounce-s', type=float, default=0.0003, help='Polling debounce per line')

    args = ap.parse_args()

    # Encoders
    E = encoders_open_poll(chip_idx=args.chip, pullup=(not args.no_pullup), poll_debounce_s=args.poll_debounce_s)
    print("[Motor Direction Test] Robot on blocks. "
          f"Observation per step: {args.seconds:.2f}s (settle {args.settle:.2f}s)")
    print("Pins (BCM): Left(21,20) Right(12,26) — counts should be POSITIVE when the side spins FORWARD.\n")

    # Driver
    drv = None
    if args.driver == 'pca':
        try:
            from smbus2 import SMBus  # noqa: F401
        except Exception as e:
            print("ERROR: smbus2 is required. Install: sudo apt install -y python3-smbus2", file=sys.stderr)
            sys.exit(1)
        drv = PCAMotorDriver(
            i2c_bus=args.i2c_bus, addr=args.pwm_addr, freq=args.pwm_freq,
            m_pwm_ch=[args.m1_pwm_ch, args.m2_pwm_ch, args.m3_pwm_ch, args.m4_pwm_ch],
            m_in1_ch=[args.m1_in1_ch, args.m2_in1_ch, args.m3_in1_ch, args.m4_in1_ch],
            m_in2_ch=[args.m1_in2_ch, args.m2_in2_ch, args.m3_in2_ch, args.m4_in2_ch],
            duty_limit=1.0
        )

    left_idxs  = tuple(int(x) for x in args.left_group.split(','))
    right_idxs = tuple(int(x) for x in args.right_group.split(','))

    try:
        # --- LEFT FORWARD ---
        if args.driver == 'manual':
            input(f">>> Step 1: LEFT FORWARD. Press Enter, then power ONLY LEFT side forward for {args.seconds:.1f}s and release…")
        else:
            drv.drive_side(left_idxs, +args.duty)
        dL1, dR1, iL1, iR1 = measure_poll(E, args.seconds)
        if drv: drv.stop()
        print_step('LEFT forward', dL1, dR1, iL1, iR1, 'L')
        time.sleep(args.settle)

        # --- LEFT REVERSE ---
        if args.driver == 'manual':
            input(f">>> Step 2: LEFT REVERSE. Press Enter, then power ONLY LEFT side reverse for {args.seconds:.1f}s and release…")
        else:
            drv.drive_side(left_idxs, -args.duty)
        dL2, dR2, iL2, iR2 = measure_poll(E, args.seconds)
        if drv: drv.stop()
        print_step('LEFT reverse', dL2, dR2, iL2, iR2, 'L')
        time.sleep(args.settle)

        # --- RIGHT FORWARD ---
        if args.driver == 'manual':
            input(f">>> Step 3: RIGHT FORWARD. Press Enter, then power ONLY RIGHT side forward for {args.seconds:.1f}s and release…")
        else:
            drv.drive_side(right_idxs, +args.duty)
        dL3, dR3, iL3, iR3 = measure_poll(E, args.seconds)
        if drv: drv.stop()
        print_step('RIGHT forward', dL3, dR3, iL3, iR3, 'R')
        time.sleep(args.settle)

        # --- RIGHT REVERSE ---
        if args.driver == 'manual':
            input(f">>> Step 4: RIGHT REVERSE. Press Enter, then power ONLY RIGHT side reverse for {args.seconds:.1f}s and release…")
        else:
            drv.drive_side(right_idxs, -args.duty)
        dL4, dR4, iL4, iR4 = measure_poll(E, args.seconds)
        if drv: drv.stop()
        print_step('RIGHT reverse', dL4, dR4, iL4, iR4, 'R')

        # Summary
        print("\n=== SUMMARY ===")
        print(f"LEFT forward  ΔL={dL1}  illegal={iL1}")
        print(f"RIGHT forward ΔR={dR3}  illegal={iR3}")
        print(suggest(dL1, dR3))
        print("Target: forward motion ⇒ ΔL>0 and ΔR>0")

    finally:
        try:
            if drv: drv.disable()
        except Exception:
            pass
        encoders_close(E)

if __name__ == '__main__':
    main()
