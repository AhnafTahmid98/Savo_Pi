#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot SAVO — encoders_api.py (lgpio, quadrature, 2 rear encoders)
-----------------------------------------------------------------
Reusable encoder API extracted from tools/diag/motion/encoders_test.py,
refactored for ROS2 nodes or other runtime code.

Hardware assumption:
- Only TWO encoders: Rear Left and Rear Right
- Each encoder provides A/B quadrature signals.

Provides:
- Signed tick counts (with invert flags)
- Delta ticks since last read
- Wheel linear speeds (m/s)
- Robot v and omega (rad/s) diagnostic (diff model approximation)

Dependencies:
- python3-lgpio (sudo apt install -y python3-lgpio)
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple, List

try:
    import lgpio
except Exception as e:
    raise RuntimeError(
        "python3-lgpio is required. Install with: sudo apt install -y python3-lgpio"
    ) from e


# Valid Gray-code transitions (A:bit1, B:bit0)
_DELTA = {
    (0b00, 0b01): +1, (0b01, 0b11): +1, (0b11, 0b10): +1, (0b10, 0b00): +1,
    (0b01, 0b00): -1, (0b11, 0b01): -1, (0b10, 0b11): -1, (0b00, 0b10): -1,
}


def autodetect_chip(pins: List[int], start: int = 0, end: int = 7, pullup: bool = True) -> Tuple[int, int]:
    """
    Try gpiochip[start..end] and return (chip_index, handle) that can claim all pins.
    """
    last_error: Optional[Exception] = None
    for chip in range(start, end + 1):
        try:
            h = lgpio.gpiochip_open(chip)
            claimed: List[int] = []
            try:
                for p in pins:
                    lgpio.gpio_claim_input(h, p)
                    claimed.append(p)
                    if pullup:
                        try:
                            lgpio.gpio_set_flags(h, p, lgpio.BIAS_PULL_UP)
                        except Exception:
                            pass

                for p in claimed:
                    lgpio.gpio_free(h, p)
                lgpio.gpiochip_close(h)
                return chip, lgpio.gpiochip_open(chip)

            except Exception as e:
                last_error = e
                for p in claimed:
                    try:
                        lgpio.gpio_free(h, p)
                    except Exception:
                        pass
                lgpio.gpiochip_close(h)

        except Exception as e:
            last_error = e

    raise RuntimeError(
        f"Could not find a gpiochip that can claim pins {pins}. Last error: {last_error}"
    )


class _QuadSide:
    """One quadrature encoder (A,B) with Gray-code state machine."""

    def __init__(
        self,
        h: int,
        a: int,
        b: int,
        *,
        debounce_s: float,
        invert: int = +1,
        use_hw_debounce: bool = True,
        pullup: bool = True,
    ):
        self.h, self.a, self.b = h, a, b
        self.debounce_s = max(0.0, float(debounce_s))
        self.pullup = bool(pullup)
        self.invert = +1 if invert >= 0 else -1

        # Claim inputs
        lgpio.gpio_claim_input(h, a)
        lgpio.gpio_claim_input(h, b)

        # Pull-ups (harmless if ignored by driver)
        if self.pullup:
            try:
                lgpio.gpio_set_flags(h, a, lgpio.BIAS_PULL_UP)
                lgpio.gpio_set_flags(h, b, lgpio.BIAS_PULL_UP)
            except Exception:
                pass

        # Hardware debounce (best effort)
        self.hw_debounce = False
        if use_hw_debounce and self.debounce_s > 0.0:
            try:
                usec = int(self.debounce_s * 1e6)
                lgpio.gpio_set_debounce_micros(h, a, usec)
                lgpio.gpio_set_debounce_micros(h, b, usec)
                self.hw_debounce = True
            except Exception:
                self.hw_debounce = False

        # Software debounce bookkeeping
        now = time.monotonic()
        self.last_change = {a: now, b: now}
        self.state_bits = {a: lgpio.gpio_read(h, a), b: lgpio.gpio_read(h, b)}
        self.prev_state = ((self.state_bits[a] & 1) << 1) | (self.state_bits[b] & 1)

        self.count = 0     # signed count (includes invert)
        self.dir = 0       # last step dir: -1, 0, +1
        self.illegal = 0   # illegal Gray-code jumps

    def close(self):
        for p in (self.a, self.b):
            try:
                lgpio.gpio_free(self.h, p)
            except Exception:
                pass

    def sample_once(self, now: float) -> None:
        a_val = lgpio.gpio_read(self.h, self.a)
        b_val = lgpio.gpio_read(self.h, self.b)

        # Software debounce if HW debounce not active
        if not self.hw_debounce and self.debounce_s > 0.0:
            if a_val != self.state_bits[self.a]:
                if (now - self.last_change[self.a]) >= self.debounce_s:
                    self.state_bits[self.a] = a_val
                    self.last_change[self.a] = now
                else:
                    a_val = self.state_bits[self.a]
            else:
                self.last_change[self.a] = now

            if b_val != self.state_bits[self.b]:
                if (now - self.last_change[self.b]) >= self.debounce_s:
                    self.state_bits[self.b] = b_val
                    self.last_change[self.b] = now
                else:
                    b_val = self.state_bits[self.b]
            else:
                self.last_change[self.b] = now

        s_new = ((a_val & 1) << 1) | (b_val & 1)
        if s_new != self.prev_state:
            d = _DELTA.get((self.prev_state, s_new), None)
            if d is None:
                self.illegal += 1
            else:
                d *= self.invert
                self.count += d
                self.dir = 1 if d > 0 else -1
            self.prev_state = s_new


@dataclass
class EncoderKinematics:
    wheel_dia_m: float = 0.065
    cpr: int = 20
    decoding: int = 4
    gear: float = 1.0
    track_m: float = 0.165

    def counts_per_wheel_rev(self) -> int:
        edges_per_rev = int(self.cpr) * int(self.decoding)
        return max(1, int(edges_per_rev * float(self.gear)))

    def wheel_m_per_rev(self) -> float:
        return math.pi * float(self.wheel_dia_m)


# ---------- Public dataclasses expected by sensors_api/__init__.py ----------

@dataclass
class EncodersStatus:
    """
    Static-ish info about the encoder interface (useful for diagnostics/logging).
    """
    started: bool
    chip_used: Optional[int]

    left_a: int
    left_b: int
    right_a: int
    right_b: int

    invert_left: bool
    invert_right: bool

    pullup: bool
    use_hw_debounce: bool
    debounce_s: float

    kin: EncoderKinematics


@dataclass
class EncodersSample:
    """
    One read() output sample (call read() at your publish rate, e.g. 20–50 Hz).
    """
    t_monotonic: float
    dt_s: float

    left_count: int
    right_count: int
    left_delta: int
    right_delta: int

    left_dir: int
    right_dir: int

    left_v_mps: float
    right_v_mps: float

    v_mps: float
    omega_rad_s: float

    left_illegal: int
    right_illegal: int


# Backward-compat alias (if any old code used EncoderReadout)
EncoderReadout = EncodersSample


class EncodersApi:
    """
    High-level API for two encoders (Left + Right).

    Pattern:
      api.start()
      loop:
        api.poll_once() repeatedly (tight loop)
        at publish tick: sample = api.read()
    """

    def __init__(
        self,
        *,
        l_a: int = 21,
        l_b: int = 20,
        r_a: int = 12,
        r_b: int = 26,
        chip: Optional[int] = None,
        pullup: bool = True,
        use_hw_debounce: bool = True,
        debounce_s: float = 0.0003,
        invert_left: bool = False,
        invert_right: bool = False,
        kin: Optional[EncoderKinematics] = None,
    ):
        self.l_a, self.l_b, self.r_a, self.r_b = int(l_a), int(l_b), int(r_a), int(r_b)
        self.chip = chip
        self.pullup = bool(pullup)
        self.use_hw_debounce = bool(use_hw_debounce)
        self.debounce_s = float(debounce_s)
        self.invert_left = bool(invert_left)
        self.invert_right = bool(invert_right)
        self.kin = kin if kin is not None else EncoderKinematics()

        self._h: Optional[int] = None
        self._chip_used: Optional[int] = None
        self._L: Optional[_QuadSide] = None
        self._R: Optional[_QuadSide] = None

        self._started = False
        self._last_read_t: Optional[float] = None
        self._lastL: int = 0
        self._lastR: int = 0

    def __enter__(self) -> "EncodersApi":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()

    @property
    def chip_used(self) -> Optional[int]:
        return self._chip_used

    def get_status(self) -> EncodersStatus:
        return EncodersStatus(
            started=self._started,
            chip_used=self._chip_used,
            left_a=self.l_a,
            left_b=self.l_b,
            right_a=self.r_a,
            right_b=self.r_b,
            invert_left=self.invert_left,
            invert_right=self.invert_right,
            pullup=self.pullup,
            use_hw_debounce=self.use_hw_debounce,
            debounce_s=self.debounce_s,
            kin=self.kin,
        )

    def start(self) -> None:
        pins = [self.l_a, self.l_b, self.r_a, self.r_b]

        if self.chip is not None:
            self._h = lgpio.gpiochip_open(int(self.chip))
            self._chip_used = int(self.chip)
        else:
            self._chip_used, self._h = autodetect_chip(pins, pullup=self.pullup)

        assert self._h is not None

        self._L = _QuadSide(
            self._h, self.l_a, self.l_b,
            debounce_s=self.debounce_s,
            invert=(-1 if self.invert_left else +1),
            use_hw_debounce=self.use_hw_debounce,
            pullup=self.pullup,
        )
        self._R = _QuadSide(
            self._h, self.r_a, self.r_b,
            debounce_s=self.debounce_s,
            invert=(-1 if self.invert_right else +1),
            use_hw_debounce=self.use_hw_debounce,
            pullup=self.pullup,
        )

        now = time.monotonic()
        self._last_read_t = now
        self._lastL = self._L.count
        self._lastR = self._R.count
        self._started = True

    def stop(self) -> None:
        try:
            if self._L:
                self._L.close()
            if self._R:
                self._R.close()
        finally:
            self._L = None
            self._R = None

            if self._h is not None:
                try:
                    lgpio.gpiochip_close(self._h)
                except Exception:
                    pass
            self._h = None
            self._chip_used = None

            self._started = False
            self._last_read_t = None
            self._lastL = 0
            self._lastR = 0

    def reset_counts(self) -> None:
        """Reset software counts (diagnostics / test convenience)."""
        if not self._started or self._L is None or self._R is None:
            raise RuntimeError("EncodersApi not started")
        self._L.count = 0
        self._R.count = 0
        self._L.illegal = 0
        self._R.illegal = 0
        now = time.monotonic()
        self._last_read_t = now
        self._lastL = 0
        self._lastR = 0

    def poll_once(self) -> None:
        """One high-rate sample step (call this in a tight loop)."""
        if not self._started or self._L is None or self._R is None:
            raise RuntimeError("EncodersApi not started")
        now = time.monotonic()
        self._L.sample_once(now)
        self._R.sample_once(now)

    def poll_for(self, poll_s: float) -> None:
        """Poll encoder pins for approximately poll_s seconds."""
        t_end = time.monotonic() + max(0.0, float(poll_s))
        while time.monotonic() < t_end:
            self.poll_once()

    def read(self) -> EncodersSample:
        """
        Return one sample since last read() call.
        Call this at your publishing rate (e.g., 20–50 Hz).
        """
        if not self._started or self._L is None or self._R is None:
            raise RuntimeError("EncodersApi not started")

        now = time.monotonic()
        last_t = self._last_read_t if self._last_read_t is not None else now
        dt = max(1e-6, now - last_t)

        Lc = self._L.count
        Rc = self._R.count
        dL = Lc - self._lastL
        dR = Rc - self._lastR

        self._lastL, self._lastR = Lc, Rc
        self._last_read_t = now

        counts_per_wrev = self.kin.counts_per_wheel_rev()
        wheel_m_per_rev = self.kin.wheel_m_per_rev()

        # counts/sec -> rev/sec -> m/s
        L_v = (dL / dt) / counts_per_wrev * wheel_m_per_rev
        R_v = (dR / dt) / counts_per_wrev * wheel_m_per_rev

        # diagnostic diff model
        v = 0.5 * (L_v + R_v)
        omega = (R_v - L_v) / max(1e-9, float(self.kin.track_m))

        return EncodersSample(
            t_monotonic=now,
            dt_s=dt,
            left_count=Lc,
            right_count=Rc,
            left_delta=dL,
            right_delta=dR,
            left_dir=self._L.dir,
            right_dir=self._R.dir,
            left_v_mps=L_v,
            right_v_mps=R_v,
            v_mps=v,
            omega_rad_s=omega,
            left_illegal=self._L.illegal,
            right_illegal=self._R.illegal,
        )