#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Manual Teleop (NO ROS2) via PCA9685 + H-Bridge
Pattern Mode + Paired Board + Dual-PWM Reverse Support
---------------------------------------------------------------------------
WHY THIS VERSION:
- Some boards (like your FNK0043 + PCA9685 wiring) need a DIFFERENT PWM channel
  for REVERSE vs FORWARD. This driver lets each wheel (or side) use:
     * pwm_fwd  (for forward)
     * pwm_rev  (for reverse)
  and optionally swap IN1/IN2 only during reverse.

- Supports "paired board" (FL=RL and FR=RR) with --paired-mode.
- Motor-safe channels are 0..7; servos 8..15 are reserved OFF by default.

Key patterns (order = FL, FR, RL, RR):
  F  = (+1,+1,+1,+1)
  B  = (-1,-1,-1,-1)
  SL = (-1,+1,+1,-1)  (mapped to TL in paired mode)
  SR = (+1,-1,-1,+1)  (mapped to TR in paired mode)
  TL = (-1,-1,+1,+1)
  TR = (+1,+1,-1,-1)

Keys: W/S/A/D/Q/E, Space stop, G gentle, 1..8 pulses, M status, R reset scale, ESC/Ctrl+C quit.
"""

import argparse, os, select, sys, termios, time, tty, signal
from dataclasses import dataclass

# ---------------- I2C ----------------
try:
    from smbus2 import SMBus
except Exception:
    print("ERROR: smbus2 missing. Install: sudo apt install -y python3-smbus  OR  pip3 install smbus2", file=sys.stderr)
    raise

# --------------- PCA9685 low-level ---------------
MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, OUTDRV, ALLCALL = 0x80, 0x10, 0x20, 0x04, 0x01

class PCA9685:
    def __init__(self, bus: int = 1, addr: int = 0x40, freq_hz: float = 800.0, set_freq: bool = True):
        self.addr = int(addr)
        self.bus = SMBus(int(bus))
        self._w8(MODE2, OUTDRV)
        self._w8(MODE1, AI)
        time.sleep(0.003)
        if set_freq:
            self.set_pwm_freq(freq_hz)
        m1 = self._r8(MODE1)
        self._w8(MODE1, (m1 | RESTART | AI) & ~ALLCALL)

    def close(self):
        try: self.bus.close()
        except Exception: pass

    def _w8(self, reg, val): self.bus.write_byte_data(self.addr, reg & 0xFF, val & 0xFF)
    def _r8(self, reg):      return self.bus.read_byte_data(self.addr, reg & 0xFF)

    def set_pwm_freq(self, f_hz: float):
        f = float(max(40.0, min(1500.0, f_hz)))
        prescale = int(max(3, min(255, round(25_000_000.0/(4096.0*f)-1.0))))
        old = self._r8(MODE1)
        self._w8(MODE1, (old & ~RESTART) | SLEEP)
        self._w8(PRESCALE, prescale)
        self._w8(MODE1, old & ~SLEEP)
        time.sleep(0.003)
        self._w8(MODE1, (old | RESTART | AI) & ~ALLCALL)
        print(f"[PCA] freq={f:.1f}Hz prescale={prescale}")

    def _raw(self, ch, on, off):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0, on & 0xFF); self._w8(base+1, (on>>8)&0x0F)
        self._w8(base+2, off&0xFF);  self._w8(base+3, (off>>8)&0x0F)

    def set_duty(self, ch: int, duty: float):
        duty = max(0.0, min(1.0, float(duty)))
        off = int(round(duty * 4095))
        if off <= 0:
            self.full_off(ch)
        elif off >= 4095:
            self.full_on(ch)
        else:
            self._raw(ch, 0, off)

    def full_off(self, ch: int):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0,0); self._w8(base+1,0); self._w8(base+2,0); self._w8(base+3,0x10)

    def full_on(self, ch: int):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0,0); self._w8(base+1,0x10); self._w8(base+2,0); self._w8(base+3,0)

# --------------- H-bridge models ----------------
@dataclass
class MotorCH:
    pwm_fwd: int        # PWM channel used for forward (>0 duty)
    in1: int
    in2: int
    pwm_rev: int = None # OPTIONAL alternate PWM channel used for reverse
    name: str = "M"
    swap_fwd: bool = False   # swap IN1/IN2 when driving forward
    swap_rev: bool = False   # swap IN1/IN2 when driving reverse

class HBridgeDualPWM:
    """
    Supports two *different* PWM channels for FWD/REV and independent IN swaps.
    This matches boards where reverse only works when PWM is moved to another channel.
    """
    def __init__(self, pca: PCA9685, ch: MotorCH, invert=False, in_active_low=False):
        self.pca = pca
        self.ch = ch
        self.invert = bool(invert)
        self.in_active_low = bool(in_active_low)
        # idle
        self.stop()

    def _digital(self, channel: int, level: int):
        if self.in_active_low:
            level ^= 1
        (self.pca.full_on if level else self.pca.full_off)(channel)

    def _set_dir(self, forward: bool):
        # choose swap per direction
        swap = self.ch.swap_fwd if forward else self.ch.swap_rev
        a, b = (self.ch.in1, self.ch.in2) if not swap else (self.ch.in2, self.ch.in1)
        # H-bridge truth table: A=1,B=0 forward; A=0,B=1 reverse
        if forward:
            self._digital(a, 1); self._digital(b, 0)
        else:
            self._digital(a, 0); self._digital(b, 1)

    def _set_pwm(self, forward: bool, duty: float):
        ch = self.ch.pwm_fwd if forward else (self.ch.pwm_rev if self.ch.pwm_rev is not None else self.ch.pwm_fwd)
        self.pca.set_duty(ch, duty)

    def drive(self, duty_signed: float):
        d = max(-1.0, min(1.0, float(duty_signed)))
        if self.invert: d = -d
        if abs(d) < 1e-3:
            self.stop(); return
        forward = (d > 0)
        self._set_dir(forward)
        self._set_pwm(forward, abs(d))

    def stop(self):
        # idle both INs low and both PWMs off (if rev pwm exists)
        self._digital(self.ch.in1, 0)
        self._digital(self.ch.in2, 0)
        self.pca.set_duty(self.ch.pwm_fwd, 0.0)
        if self.ch.pwm_rev is not None:
            self.pca.set_duty(self.ch.pwm_rev, 0.0)

# --------------- keyboard ----------------
class Keyboard:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)

    def _read1(self, t=0.0):
        r,_,_ = select.select([sys.stdin], [], [], t)
        return os.read(self.fd,1) if r else None

    def read(self):
        out=[]; ch=self._read1(0.0)
        while ch is not None:
            if ch==b'\x1b':
                a=self._read1(0.02)
                if a in (b'[',b'O'):
                    b=self._read1(0.04)
                    arrows={b'A':b'UP',b'B':b'DOWN',b'C':b'RIGHT',b'D':b'LEFT'}
                    if b in arrows: out.append(arrows[b])
                else:
                    out.append(b'\x1b'); 
                    if a: out.append(a)
            else:
                out.append(ch)
            ch=self._read1(0.0)
        return out

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

# --------------- helpers ----------------
def clamp(x, lo, hi): return max(lo, min(hi, x))

# --------------- teleop ----------------
class DirectTeleop:
    def __init__(self, args):
        self.args = args
        self.hz = float(args.hz)
        self.deadman = float(args.deadman)
        self.idle_exit = float(args.idle_exit)
        self.max_runtime = float(args.max_runtime)
        self.scale_low = float(args.scale_low)
        self.scale_high = float(args.scale_high)
        self.scale = 1.0
        self._running = True

        # PCA + reserve channels (servos 8..15 OFF by default)
        self.pca = PCA9685(bus=args.i2c_bus, addr=args.pca_addr, freq_hz=args.pwm_freq, set_freq=args.set_freq)
        try:
            reserved = [int(x) for x in str(args.reserve).split(',') if x.strip()!='']
            for ch in reserved: self.pca.full_off(ch)
            if reserved: print(f"[Teleop] Reserved OFF: {reserved}")
        except Exception as e:
            print(f"[Teleop] Warning: reserve failed: {e}")

        # Motors (dual-PWM capable)
        self.FL = HBridgeDualPWM(self.pca, MotorCH(
            pwm_fwd=args.fl_pwm, pwm_rev=args.fl_pwm_rev, in1=args.fl_in1, in2=args.fl_in2,
            name="FL", swap_fwd=args.swap_in12_fl, swap_rev=args.swap_in12_fl_rev
        ), invert=args.inv_fl, in_active_low=args.in_active_low)

        self.FR = HBridgeDualPWM(self.pca, MotorCH(
            pwm_fwd=args.fr_pwm, pwm_rev=args.fr_pwm_rev, in1=args.fr_in1, in2=args.fr_in2,
            name="FR", swap_fwd=args.swap_in12_fr, swap_rev=args.swap_in12_fr_rev
        ), invert=args.inv_fr, in_active_low=args.in_active_low)

        self.RL = HBridgeDualPWM(self.pca, MotorCH(
            pwm_fwd=args.rl_pwm, pwm_rev=args.rl_pwm_rev, in1=args.rl_in1, in2=args.rl_in2,
            name="RL", swap_fwd=args.swap_in12_rl, swap_rev=args.swap_in12_rl_rev
        ), invert=args.inv_rl, in_active_low=args.in_active_low)

        self.RR = HBridgeDualPWM(self.pca, MotorCH(
            pwm_fwd=args.rr_pwm, pwm_rev=args.rr_pwm_rev, in1=args.rr_in1, in2=args.rr_in2,
            name="RR", swap_fwd=args.swap_in12_rr, swap_rev=args.swap_in12_rr_rev
        ), invert=args.inv_rr, in_active_low=args.in_active_low)

        self._check_channels(args)

        # Current signed duties per wheel
        self.duties = [0.0, 0.0, 0.0, 0.0]
        self.last_input = time.monotonic()
        self.start_time = self.last_input

        # Signals
        def _sig(_s,_f): self._running=False
        signal.signal(signal.SIGINT, _sig)
        signal.signal(signal.SIGTERM, _sig)

        print(
            "[Teleop] READY. Esc/Ctrl+C to quit.\n"
            f" rate={self.hz}Hz deadman={self.deadman}s idle-exit={self.idle_exit}s max-runtime={self.max_runtime}s\n"
            f" PCA=0x{args.pca_addr:02X} bus={args.i2c_bus} pwm={args.pwm_freq}Hz  paired_mode={args.paired_mode}\n"
            " Keys: W/S/A/D/Q/E, Space stop, G gentle, 1..8 pulses, M print, R reset scale\n"
        )

    # -------- safety checks --------
    def _check_channels(self, args):
        reserved = set(int(x) for x in str(args.reserve).split(',') if x.strip()!='')
        def check_name_pwm(name, pwm):
            if pwm is None: return
            if pwm < 0 or pwm > 7:
                raise ValueError(f"[ERROR] {name} PWM={pwm} out of motor-safe range 0..7 (8..15 are servos).")
            if pwm in reserved:
                raise ValueError(f"[ERROR] {name} PWM={pwm} is reserved ({sorted(reserved)}).")
        def check_name_in(name, ch):
            if ch < 0 or ch > 7:
                raise ValueError(f"[ERROR] {name} IN={ch} out of motor-safe range 0..7 (8..15 are servos).")
            if ch in reserved:
                raise ValueError(f"[ERROR] {name} IN={ch} is reserved ({sorted(reserved)}).")

        # per wheel
        wheel_defs = [
            ("FL", args.fl_pwm, args.fl_pwm_rev, args.fl_in1, args.fl_in2),
            ("FR", args.fr_pwm, args.fr_pwm_rev, args.fr_in1, args.fr_in2),
            ("RL", args.rl_pwm, args.rl_pwm_rev, args.rl_in1, args.rl_in2),
            ("RR", args.rr_pwm, args.rr_pwm_rev, args.rr_in1, args.rr_in2),
        ]
        for name, pwm_fwd, pwm_rev, in1, in2 in wheel_defs:
            check_name_pwm(f"{name}.pwm_fwd", pwm_fwd)
            if pwm_rev is not None:
                check_name_pwm(f"{name}.pwm_rev", pwm_rev)
            check_name_in(f"{name}.in1", in1)
            check_name_in(f"{name}.in2", in2)

        # avoid PWM overlap with IN pins
        all_pwms = [p for p in (args.fl_pwm, args.fr_pwm, args.rl_pwm, args.rr_pwm,
                                args.fl_pwm_rev, args.fr_pwm_rev, args.rl_pwm_rev, args.rr_pwm_rev) if p is not None]
        ins = [args.fl_in1, args.fl_in2, args.fr_in1, args.fr_in2, args.rl_in1, args.rl_in2, args.rr_in1, args.rr_in2]
        bad = sorted(set(all_pwms) & set(ins))
        if bad:
            raise ValueError(f"[ERROR] PWM channels {bad} also used as IN pins — not allowed.")

    # -------- lifecycle --------
    def close(self):
        print("[Teleop] Shutting down safely...")
        try:
            for m in (self.FL, self.FR, self.RL, self.RR):
                try: m.stop()
                except: pass
            if hasattr(self, 'kb'): self.kb.restore()
            self.pca.close()
        except Exception as e:
            print(f"[Teleop] Close warning: {e}")

    def loop(self):
        self.kb = Keyboard()
        period = 1.0 / max(1.0, self.hz)
        try:
            while self._running:
                now = time.monotonic()

                if self.max_runtime > 0 and (now - self.start_time) >= self.max_runtime:
                    print("[Time] Max runtime reached → exiting"); break

                got = False
                for ch in self.kb.read():
                    got = True
                    if self._handle_key(ch):
                        self._running = False; break
                    self.last_input = now
                if not self._running: break

                # deadman = stop motors (not process)
                if not got and (now - self.last_input) > self.deadman:
                    if any(abs(x) > 1e-6 for x in self.duties):
                        print("[Deadman] STOP")
                    self.apply_pattern((0,0,0,0), magnitude=0.0)

                # idle exit (quit process)
                if self.idle_exit > 0 and (now - self.last_input) > self.idle_exit:
                    print("[Idle] No input → exiting"); break

                self._apply_duties()

                sleep_t = period - (time.monotonic() - now)
                if sleep_t > 0: time.sleep(sleep_t)
        finally:
            self.close()

    # -------- input handling --------
    def _handle_key(self, ch) -> bool:
        if ch in (b"\x1b", b"\x03"):
            print("[Teleop] Quit"); return True
        if ch == b' ':
            self.apply_pattern((0,0,0,0), magnitude=0.0, announce="[Cmd] STOP"); return False
        if ch in (b'r', b'R', b"\x12"):
            self.scale = 1.0; print("[Scale] reset → 1.0"); return False
        if ch in (b'g', b'G'):
            self._gentle_forward(); return False
        if ch in (b'm', b'M'):
            self._print_status(); return False

        slow = (isinstance(ch,bytes) and len(ch)==1 and 1 <= ch[0] <= 26)
        fast = (isinstance(ch,bytes) and ch.isalpha() and ch.isupper())
        self.scale = clamp(self.scale_low if slow else (self.scale_high if fast else 1.0), 0.05, 3.0)

        # patterns
        F  = (+1,+1,+1,+1)
        B  = (-1,-1,-1,-1)
        SL = (-1,+1,+1,-1)
        SR = (+1,-1,-1,+1)
        TL = (-1,-1,+1,+1)
        TR = (+1,+1,-1,-1)

        if ch in (b'w', b'W', b'UP'):      self.apply_pattern(F,  announce='[Cmd] F (forward)')
        elif ch in (b's', b'S', b'DOWN'):   self.apply_pattern(B,  announce='[Cmd] B (back)')
        elif ch in (b'a', b'A', b'LEFT'):   self.apply_pattern(SL, announce='[Cmd] SL (strafe L)')
        elif ch in (b'd', b'D', b'RIGHT'):  self.apply_pattern(SR, announce='[Cmd] SR (strafe R)')
        elif ch in (b'q', b'Q'):            self.apply_pattern(TL, announce='[Cmd] TL (turn CCW)')
        elif ch in (b'e', b'E'):            self.apply_pattern(TR, announce='[Cmd] TR (turn CW)')
        elif ch == b'1': self._pulse_wheels([1,0,0,0]);        return False
        elif ch == b'2': self._pulse_wheels([0,1,0,0]);        return False
        elif ch == b'3': self._pulse_wheels([0,0,1,0]);        return False
        elif ch == b'4': self._pulse_wheels([0,0,0,1]);        return False
        elif ch == b'5': self._pulse_wheels([1,0,0,0], -0.12); return False
        elif ch == b'6': self._pulse_wheels([0,1,0,0], -0.12); return False
        elif ch == b'7': self._pulse_wheels([0,0,1,0], -0.12); return False
        elif ch == b'8': self._pulse_wheels([0,0,0,1], -0.12); return False

        return False

    # -------- patterns to duties --------
    def apply_pattern(self, signs, magnitude=1.0, announce=None):
        """
        signs: (FL, FR, RL, RR) each in {-1,0,+1}
        magnitude: scales duty (0..1) then multiplied by 'scale'
        """
        mag = clamp(float(magnitude) * self.scale, 0.0, 1.0)
        s = list(signs)

        if self.args.paired_mode:
            # Map impossible strafes and enforce side pairing
            if tuple(signs) == (-1,+1,+1,-1):   # SL
                print("[Paired] SL not possible → using TL")
                s = [-1,-1,+1,+1]
            elif tuple(signs) == (+1,-1,-1,+1): # SR
                print("[Paired] SR not possible → using TR")
                s = [+1,+1,-1,-1]
            s[2] = s[0]  # RL = FL
            s[3] = s[1]  # RR = FR

        self.duties = [clamp(si * mag, -1.0, 1.0) for si in s]

        if announce:
            print(f"{announce}  signs={tuple(s)}  mag={mag:.2f}")

    def _apply_duties(self):
        d = self.duties
        try:
            self.FL.drive(d[0])
            self.FR.drive(d[1])
            self.RL.drive(d[2])
            self.RR.drive(d[3])
        except Exception as e:
            print(f"[Teleop] Motor drive error: {e}")

    # -------- utils --------
    def _pulse_wheels(self, mask, duty=0.12, t=0.30):
        try:
            motors = [self.FL, self.FR, self.RL, self.RR]
            for i, m in enumerate(motors):
                if mask[i]: m.drive(duty)
            time.sleep(t)
            for m in motors: m.stop()
        except Exception as e:
            print(f"[Teleop] Wheel pulse error: {e}")

    def _gentle_forward(self, duty=0.10, t=0.50):
        print("[Test] Gentle forward pulse")
        try:
            for m in (self.FL, self.FR, self.RL, self.RR): m.drive(duty)
            time.sleep(t)
            for m in (self.FL, self.FR, self.RL, self.RR): m.stop()
        except Exception as e:
            print(f"[Teleop] Gentle forward error: {e}")

    def _print_status(self):
        d = self.duties
        print(f"[duties] FL {d[0]:+0.2f}  FR {d[1]:+0.2f}  RL {d[2]:+0.2f}  RR {d[3]:+0.2f}")

# --------------- CLI ---------------
def build_argparser():
    p = argparse.ArgumentParser(description='Robot Savo — Manual Teleop (pattern mode, dual-PWM reverse)')

    # timing / exit / scaling
    p.add_argument('--hz', type=float, default=30.0)
    p.add_argument('--deadman', type=float, default=0.40)
    p.add_argument('--idle-exit', type=float, default=0.0, help='Auto-exit after SEC idle (0=never)')
    p.add_argument('--max-runtime', type=float, default=0.0, help='Hard exit after SEC (0=never)')
    p.add_argument('--scale-low', type=float, default=0.25)
    p.add_argument('--scale-high', type=float, default=1.35)

    # I2C/PCA
    p.add_argument('--i2c-bus', type=int, default=1)
    p.add_argument('--pca-addr', type=lambda x:int(x,0), default=0x40)
    p.add_argument('--pwm-freq', type=float, default=800.0)
    p.add_argument('--set-freq', dest='set_freq', action='store_true', default=True)
    p.add_argument('--no-set-freq', dest='set_freq', action='store_false')
    p.add_argument('--reserve', type=str, default='8,9,10,11,12,13,14,15')
    p.add_argument('--in-active-low', action='store_true')

    # Channels (REQUIRED, 0..7). Dual-PWM: *_pwm (forward) + optional *_pwm-rev (reverse)
    # Left Front
    p.add_argument('--fl-pwm', type=int, required=True)
    p.add_argument('--fl-pwm-rev', type=int, default=None)
    p.add_argument('--fl-in1', type=int, required=True)
    p.add_argument('--fl-in2', type=int, required=True)
    # Right Front
    p.add_argument('--fr-pwm', type=int, required=True)
    p.add_argument('--fr-pwm-rev', type=int, default=None)
    p.add_argument('--fr-in1', type=int, required=True)
    p.add_argument('--fr-in2', type=int, required=True)
    # Left Rear
    p.add_argument('--rl-pwm', type=int, required=True)
    p.add_argument('--rl-pwm-rev', type=int, default=None)
    p.add_argument('--rl-in1', type=int, required=True)
    p.add_argument('--rl-in2', type=int, required=True)
    # Right Rear
    p.add_argument('--rr-pwm', type=int, required=True)
    p.add_argument('--rr-pwm-rev', type=int, default=None)
    p.add_argument('--rr-in1', type=int, required=True)
    p.add_argument('--rr-in2', type=int, required=True)

    # Polarity / IN swaps
    p.add_argument('--inv-fl', action='store_true'); p.add_argument('--inv-fr', action='store_true')
    p.add_argument('--inv-rl', action='store_true'); p.add_argument('--inv-rr', action='store_true')
    # Swap IN1/IN2 only for FORWARD on that wheel
    p.add_argument('--swap-in12-fl', action='store_true'); p.add_argument('--swap-in12-fr', action='store_true')
    p.add_argument('--swap-in12-rl', action='store_true'); p.add_argument('--swap-in12-rr', action='store_true')
    # Swap IN1/IN2 only for REVERSE on that wheel (matches your reverse combos)
    p.add_argument('--swap-in12-fl-rev', dest='swap_in12_fl_rev', action='store_true')
    p.add_argument('--swap-in12-fr-rev', dest='swap_in12_fr_rev', action='store_true')
    p.add_argument('--swap-in12-rl-rev', dest='swap_in12_rl_rev', action='store_true')
    p.add_argument('--swap-in12-rr-rev', dest='swap_in12_rr_rev', action='store_true')

    # Paired board
    p.add_argument('--paired-mode', action='store_true', help='Force FL=RL and FR=RR; map SL/SR to TL/TR')

    return p

def main(argv=None):
    args = build_argparser().parse_args(argv)
    t = None
    try:
        t = DirectTeleop(args)
        t.loop()
    except KeyboardInterrupt:
        print("\n[Main] KeyboardInterrupt")
    finally:
        if t: t.close()
    print("[Main] Exit complete")

if __name__ == '__main__':
    main()
