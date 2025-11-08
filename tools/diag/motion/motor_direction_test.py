#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Motor Direction Test (Mecanum, Encoder-Grounded, Vendor-Neutral)
-----------------------------------------------------------------------------

Goal
  Lock the sign convention so that "forward" motion yields **positive encoder
  counts on BOTH sides**. The script observes encoders while each side is driven
  forward and reverse (manually or automatically).

Modes
  1) Manual (default): you power each side when prompted; script measures Δenc.
  2) Auto (--driver pwmgpio): script drives sides using PCA9685 (I²C) PWM +
     two GPIO direction pins per motor (board-agnostic).

Safety
  • Put the robot on blocks (wheels free) for ALL tests.
  • Keep duty modest (default 0.35) and duration short (2.0 s).

Encoders (LOCKED for Robot Savo)
  Left:  L_A=21, L_B=20   | Right: R_A=12, R_B=26   (BCM)
  Gray-code quadrature with illegal transition counting.

Examples
  # Manual prompts
  python3 tools/diag/motion/motor_direction_test.py

  # Auto (PCA9685 + GPIO dir), edit channels/pins to match wiring
  python3 tools/diag/motion/motor_direction_test.py \
    --driver pwmgpio --pwm-addr 0x40 --pwm-freq 1000 \
    --m1-pwm 0 --m1-in1 5  --m1-in2 6  \
    --m2-pwm 1 --m2-in1 13 --m2-in2 19 \
    --m3-pwm 2 --m3-in1 20 --m3-in2 21 \
    --m4-pwm 3 --m4-in1 16 --m4-in2 12 \
    --left-group 0,2 --right-group 1,3 --duty 0.35 --seconds 2.0
"""

import sys
import time
import argparse
from dataclasses import dataclass
from typing import Tuple, List, Optional

# ------------------------ Encoder (lgpio, Gray decode) ------------------------
try:
    import lgpio
except Exception as e:
    print("ERROR: lgpio is required. Install: sudo apt install -y python3-lgpio", file=sys.stderr)
    raise

# Locked pins (BCM)
L_A, L_B = 21, 20
R_A, R_B = 12, 26

# Gray-code transitions (00,01,11,10) => 0,1,3,2
GRAY_NEXT = {
    (0, 1): +1, (1, 3): +1, (3, 2): +1, (2, 0): +1,
    (1, 0): -1, (3, 1): -1, (2, 3): -1, (0, 2): -1,
}

@dataclass
class Quad:
    chip: int
    a: int
    b: int
    count: int = 0
    last_state: int = 0
    illegal: int = 0
    pullup: bool = True

    def setup(self):
        for pin in (self.a, self.b):
            lgpio.gpio_claim_input(self.chip, pin)
        if self.pullup:
            # Optional internal pull-ups (ignored if driver doesn't support)
            try:
                lgpio.gpio_set_flags(self.chip, self.a, lgpio.BIAS_PULL_UP)
                lgpio.gpio_set_flags(self.chip, self.b, lgpio.BIAS_PULL_UP)
            except Exception:
                pass
        a = lgpio.gpio_read(self.chip, self.a)
        b = lgpio.gpio_read(self.chip, self.b)
        self.last_state = (a << 1) | b

    def cb(self, _chip, _gpio, _level, _ts):
        a = lgpio.gpio_read(self.chip, self.a)
        b = lgpio.gpio_read(self.chip, self.b)
        s = (a << 1) | b
        d = GRAY_NEXT.get((self.last_state, s))
        if d is None:
            self.illegal += 1
        else:
            self.count += d
        self.last_state = s

@dataclass
class Encoders:
    chip: int
    L: Quad
    R: Quad
    alerts: List[int]

def encoders_open(chip_idx: Optional[int] = None, pullup: bool = True) -> Encoders:
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

    qL, qR = Quad(chip, L_A, L_B, pullup=pullup), Quad(chip, R_A, R_B, pullup=pullup)
    qL.setup(); qR.setup()

    alerts = [
        lgpio.gpio_claim_alert(chip, L_A, lgpio.BOTH_EDGES),
        lgpio.gpio_claim_alert(chip, L_B, lgpio.BOTH_EDGES),
        lgpio.gpio_claim_alert(chip, R_A, lgpio.BOTH_EDGES),
        lgpio.gpio_claim_alert(chip, R_B, lgpio.BOTH_EDGES),
    ]
    lgpio.callback(chip, L_A, lgpio.BOTH_EDGES, qL.cb)
    lgpio.callback(chip, L_B, lgpio.BOTH_EDGES, qL.cb)
    lgpio.callback(chip, R_A, lgpio.BOTH_EDGES, qR.cb)
    lgpio.callback(chip, R_B, lgpio.BOTH_EDGES, qR.cb)
    return Encoders(chip, qL, qR, alerts)

def encoders_close(E: Encoders):
    try:
        lgpio.gpiochip_close(E.chip)
    except Exception:
        pass

# ------------------------ Drivers ------------------------
class BaseDriver:
    def enable(self): pass
    def disable(self): pass
    def stop(self): pass
    def drive_side(self, side: str, val: float, group: Tuple[int, ...]):
        """Drive a logical side ('L' or 'R') at normalized command [-1..+1]."""
        raise NotImplementedError

class ManualDriver(BaseDriver):
    """No automatic motor control; user powers motors externally when prompted."""
    pass

class PwmGpioPca9685Driver(BaseDriver):
    """
    Vendor-neutral: PCA9685 PWM (I²C) + two GPIO direction pins per motor (BCM).
    Pure smbus2 implementation (no third-party PCA9685 library).
    """
    class _PCA9685:
        MODE1      = 0x00
        MODE2      = 0x01
        LED0_ON_L  = 0x06
        PRE_SCALE  = 0xFE

        RESTART    = 0x80
        SLEEP      = 0x10
        ALLCALL    = 0x01
        OUTDRV     = 0x04
        AI         = 0x20    # Auto-Increment

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

        def set_pwm(self, ch: int, duty_12bit: int):
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

    def __init__(self,
                 addr: int, freq: int,
                 m_pwm: List[int], m_in1: List[int], m_in2: List[int],
                 duty_limit: float = 1.0):
        try:
            from smbus2 import SMBus  # noqa
        except Exception as e:
            raise RuntimeError("smbus2 is required. Install: sudo apt install -y python3-smbus2") from e

        # I²C
        from smbus2 import SMBus
        self.bus = SMBus(1)
        self.pca = self._PCA9685(self.bus, addr)
        self.pca.set_freq(freq)

        # Channels & pins
        assert len(m_pwm) == len(m_in1) == len(m_in2) == 4, "Need 4 motors: FL, FR, RL, RR"
        self.m_pwm = m_pwm
        self.m_in1 = m_in1
        self.m_in2 = m_in2
        self.duty_limit = max(0.0, min(1.0, duty_limit))

        # GPIO chip for direction pins
        self.chip = None
        for n in range(8):
            try:
                self.chip = lgpio.gpiochip_open(n)
                break
            except Exception:
                pass
        if self.chip is None:
            raise RuntimeError("No gpiochip for direction pins (lgpio)")

        for pin in (self.m_in1 + self.m_in2):
            lgpio.gpio_claim_output(self.chip, pin)
            lgpio.gpio_write(self.chip, pin, 0)

    def enable(self): pass

    def stop(self):
        for ch in self.m_pwm:
            self.pca.set_pwm(ch, 0)
        for pin in (self.m_in1 + self.m_in2):
            lgpio.gpio_write(self.chip, pin, 0)

    def disable(self):
        self.stop()

    def _set_motor(self, idx: int, val: float):
        val = max(-1.0, min(1.0, val)) * self.duty_limit
        duty12 = int(abs(val) * 4095)
        # Direction
        if val >= 0:
            lgpio.gpio_write(self.chip, self.m_in1[idx], 1)
            lgpio.gpio_write(self.chip, self.m_in2[idx], 0)
        else:
            lgpio.gpio_write(self.chip, self.m_in1[idx], 0)
            lgpio.gpio_write(self.chip, self.m_in2[idx], 1)
        # PWM
        self.pca.set_pwm(self.m_pwm[idx], duty12)

    def drive_side(self, side: str, val: float, group: Tuple[int, ...]):
        for idx in group:
            self._set_motor(idx, val)

# ------------------------ Test Logic ------------------------
def measure(E: Encoders, seconds: float) -> Tuple[int, int, int, int]:
    L0, R0  = E.L.count, E.R.count
    iL0, iR0 = E.L.illegal, E.R.illegal
    time.sleep(seconds)
    dL, dR  = E.L.count - L0, E.R.count - R0
    iL, iR  = E.L.illegal - iL0, E.R.illegal - iR0
    return dL, dR, iL, iR

def print_step(name: str, dL: int, dR: int, iL: int, iR: int, expect: str):
    active = dL if expect == 'L' else dR
    sign = '+' if active > 0 else '-' if active < 0 else '0'
    print(f"{name:<16} | ΔL={dL:>6} ΔR={dR:>6}  sign({expect})={sign}  illegal(L/R)={iL}/{iR}")

def suggest(forwL: int, forwR: int) -> str:
    invL = forwL < 0
    invR = forwR < 0
    if invL and invR: return "Suggestion: set --invert-left --invert-right"
    if invL:          return "Suggestion: set --invert-left"
    if invR:          return "Suggestion: set --invert-right"
    return "Suggestion: no inversion needed (forward is positive on both sides)"

# ------------------------ Main ------------------------
def main():
    ap = argparse.ArgumentParser(description="Motor direction test (manual or PWM+GPIO auto)")
    # Mode & timings
    ap.add_argument('--driver', choices=['manual','pwmgpio'], default='manual',
                    help="manual: user powers motors; pwmgpio: PCA9685+GPIO auto mode")
    ap.add_argument('--seconds', type=float, default=2.0, help='Observation window per step')
    ap.add_argument('--settle',  type=float, default=0.6, help='Idle settle time between steps')
    ap.add_argument('--duty',    type=float, default=0.35, help='Normalized duty in auto mode (0..1)')
    # PCA9685 + GPIO mapping
    ap.add_argument('--pwm-addr', type=lambda x:int(x,0), default=0x40)
    ap.add_argument('--pwm-freq', type=int, default=1000)
    ap.add_argument('--m1-pwm', type=int, default=0)
    ap.add_argument('--m2-pwm', type=int, default=1)
    ap.add_argument('--m3-pwm', type=int, default=2)
    ap.add_argument('--m4-pwm', type=int, default=3)
    ap.add_argument('--m1-in1', type=int, default=5)
    ap.add_argument('--m1-in2', type=int, default=6)
    ap.add_argument('--m2-in1', type=int, default=13)
    ap.add_argument('--m2-in2', type=int, default=19)
    ap.add_argument('--m3-in1', type=int, default=20)
    ap.add_argument('--m3-in2', type=int, default=21)
    ap.add_argument('--m4-in1', type=int, default=16)
    ap.add_argument('--m4-in2', type=int, default=12)
    ap.add_argument('--left-group',  type=str, default='0,2', help='Motor indices for LEFT side (e.g., 0,2)')
    ap.add_argument('--right-group', type=str, default='1,3', help='Motor indices for RIGHT side (e.g., 1,3)')
    # Encoders / gpiochip options
    ap.add_argument('--chip', type=int, default=None, help='gpiochip index (auto if omitted)')
    ap.add_argument('--no-pullup', action='store_true', help='Do not request internal pull-ups on encoder lines')
    args = ap.parse_args()

    # Open encoders
    E = encoders_open(chip_idx=args.chip, pullup=(not args.no_pullup))
    print("[Motor Direction Test] Robot on blocks. "
          f"Observation per step: {args.seconds:.2f}s (settle {args.settle:.2f}s)")
    print("Pins (BCM): Left(21,20) Right(12,26) — counts should be POSITIVE when the side spins FORWARD.\n")

    # Driver
    drv: BaseDriver
    if args.driver == 'manual':
        drv = ManualDriver()
    else:
        drv = PwmGpioPca9685Driver(
            addr=args.pwm_addr, freq=args.pwm_freq,
            m_pwm=[args.m1_pwm, args.m2_pwm, args.m3_pwm, args.m4_pwm],
            m_in1=[args.m1_in1, args.m2_in1, args.m3_in1, args.m4_in1],
            m_in2=[args.m1_in2, args.m2_in2, args.m3_in2, args.m4_in2],
            duty_limit=1.0
        )

    left_idxs  = tuple(int(x) for x in args.left_group.split(','))
    right_idxs = tuple(int(x) for x in args.right_group.split(','))

    try:
        # --- LEFT FORWARD ---
        if args.driver == 'manual':
            input(f">>> Step 1: LEFT FORWARD. Press Enter, then power ONLY LEFT side forward for {args.seconds:.1f}s and release…")
        else:
            drv.drive_side('L', +args.duty, left_idxs)
        dL1, dR1, iL1, iR1 = measure(E, args.seconds)
        if args.driver != 'manual': drv.stop()
        print_step('LEFT forward', dL1, dR1, iL1, iR1, 'L')
        time.sleep(args.settle)

        # --- LEFT REVERSE ---
        if args.driver == 'manual':
            input(f">>> Step 2: LEFT REVERSE. Press Enter, then power ONLY LEFT side reverse for {args.seconds:.1f}s and release…")
        else:
            drv.drive_side('L', -args.duty, left_idxs)
        dL2, dR2, iL2, iR2 = measure(E, args.seconds)
        if args.driver != 'manual': drv.stop()
        print_step('LEFT reverse', dL2, dR2, iL2, iR2, 'L')
        time.sleep(args.settle)

        # --- RIGHT FORWARD ---
        if args.driver == 'manual':
            input(f">>> Step 3: RIGHT FORWARD. Press Enter, then power ONLY RIGHT side forward for {args.seconds:.1f}s and release…")
        else:
            drv.drive_side('R', +args.duty, right_idxs)
        dL3, dR3, iL3, iR3 = measure(E, args.seconds)
        if args.driver != 'manual': drv.stop()
        print_step('RIGHT forward', dL3, dR3, iL3, iR3, 'R')
        time.sleep(args.settle)

        # --- RIGHT REVERSE ---
        if args.driver == 'manual':
            input(f">>> Step 4: RIGHT REVERSE. Press Enter, then power ONLY RIGHT side reverse for {args.seconds:.1f}s and release…")
        else:
            drv.drive_side('R', -args.duty, right_idxs)
        dL4, dR4, iL4, iR4 = measure(E, args.seconds)
        if args.driver != 'manual': drv.stop()
        print_step('RIGHT reverse', dL4, dR4, iL4, iR4, 'R')

        # Summary
        print("\n=== SUMMARY ===")
        print(f"LEFT forward  ΔL={dL1}  illegal={iL1}")
        print(f"RIGHT forward ΔR={dR3}  illegal={iR3}")
        print(suggest(dL1, dR3))
        print("Target convention: forward motion ⇒ ΔL>0 and ΔR>0")

    finally:
        try:
            drv.disable()
        except Exception:
            pass
        encoders_close(E)

if __name__ == '__main__':
    main()
