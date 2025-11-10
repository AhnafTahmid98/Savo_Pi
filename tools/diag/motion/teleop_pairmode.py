#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Manual Teleop (NO ROS2) via PCA9685 + H-Bridge
Direction-Specific Wiring (FWD/REV triplets) + Paired-Board Mode
---------------------------------------------------------------------------

Adds:
- Quench + small settle delay when changing direction (fixes stale highs).
- Per-wheel reverse logic mode: std (IN1=0,IN2=1) or swap (IN1=1,IN2=0).
- --debug-pins to print exact pins/levels/duty on transitions.
- Robust exits: ESC/Ctrl+C/x/X, --max-runtime, --idle-exit, --headless,
  and auto-headless if no TTY (SSH without -t / VSCode tasks).
- Headless drive pattern: --headless-pattern {F,B,SL,SR,TL,TR,STOP} with --headless-mag.

Safety:
- Motor channels must be 0..7. Servo 8..15 default OFF (reserved).
"""

import argparse, os, select, sys, termios, time, tty, signal
from dataclasses import dataclass

# ---------------------- I2C ----------------------
try:
    from smbus2 import SMBus
except Exception:
    print("ERROR: smbus2 missing. Install: sudo apt install -y python3-smbus  OR  pip3 install smbus2", file=sys.stderr)
    raise

# ---------------------- PCA9685 low-level ----------------------
MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, OUTDRV, ALLCALL = 0x80, 0x10, 0x20, 0x04, 0x01

class PCA9685:
    def __init__(self, bus: int = 1, addr: int = 0x40, freq_hz: float = 800.0, set_freq: bool = True):
        self.addr = int(addr); self.bus = SMBus(int(bus))
        self._w8(MODE2, OUTDRV); self._w8(MODE1, AI); time.sleep(0.003)
        if set_freq: self.set_pwm_freq(freq_hz)
        m1 = self._r8(MODE1); self._w8(MODE1, (m1 | RESTART | AI) & ~ALLCALL)

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
        if off <= 0: self.full_off(ch)
        elif off >= 4095: self.full_on(ch)
        else: self._raw(ch, 0, off)

    def full_off(self, ch: int):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0,0); self._w8(base+1,0); self._w8(base+2,0); self._w8(base+3,0x10)

    def full_on(self, ch: int):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0,0); self._w8(base+1,0x10); self._w8(base+2,0); self._w8(base+3,0)

# ---------------------- Direction-specific motor model ----------------------
@dataclass
class DirTriplet:
    pwm: int
    in1: int
    in2: int

@dataclass
class MotorDirConfig:
    fwd: DirTriplet
    rev: DirTriplet
    name: str = "M"

class HBridgeDir:
    """
    Uses EXACT (pwm, in1, in2) triplets per direction.
    Quenches on direction flip; supports reverse mode swap; optional debug.
    """
    def __init__(self, pca: PCA9685, cfg: MotorDirConfig, in_active_low=False, invert_direction=False,
                 rev_mode: str = 'std', settle_s: float = 0.003, debug=False):
        self.pca = pca
        self.cfg = cfg
        self.in_active_low = bool(in_active_low)
        self.invert_direction = bool(invert_direction)
        self.rev_mode = rev_mode  # 'std' | 'swap'
        self.settle_s = float(settle_s)
        self.debug = bool(debug)
        self._last_dir = 0  # -1,0,+1
        self.stop()

    def _digital(self, ch: int, level: int):
        if self.in_active_low: level ^= 1
        (self.pca.full_on if level else self.pca.full_off)(ch)

    def _quench_all(self, why=""):
        for t in (self.cfg.fwd, self.cfg.rev):
            self._digital(t.in1, 0)
            self._digital(t.in2, 0)
            self.pca.set_duty(t.pwm, 0.0)
        if self.debug and why:
            print(f"[{self.cfg.name}] quench: {why}")

    def _apply(self, name, t: DirTriplet, in1_hi: int, in2_hi: int, duty: float, phase: str):
        self._digital(t.in1, in1_hi)
        self._digital(t.in2, in2_hi)
        self.pca.set_duty(t.pwm, duty)
        if self.debug:
            print(f"[{name}] {phase}: PWM ch{t.pwm} duty={duty:.2f}  IN1 ch{t.in1}={in1_hi} IN2 ch{t.in2}={in2_hi}")

    def drive(self, duty_signed: float):
        d = max(-1.0, min(1.0, float(duty_signed)))
        if self.invert_direction: d = -d

        if abs(d) < 1e-3:
            self.stop()
            return

        new_dir = 1 if d > 0 else -1
        if self._last_dir != 0 and new_dir != self._last_dir:
            self._quench_all("dir flip")
            if self.settle_s > 0: time.sleep(self.settle_s)

        if new_dir > 0:
            t = self.cfg.fwd
            self._apply(self.cfg.name, t, 1, 0, abs(d), "FWD")
        else:
            t = self.cfg.rev
            if self.rev_mode == 'swap':
                self._apply(self.cfg.name, t, 1, 0, abs(d), "REV.swap")
            else:
                self._apply(self.cfg.name, t, 0, 1, abs(d), "REV.std")

        self._last_dir = new_dir

    def stop(self):
        self._quench_all("stop")
        self._last_dir = 0

# ---------------------- keyboard ----------------------
class Keyboard:
    def __init__(self):
        if not sys.stdin.isatty():
            raise RuntimeError("stdin is not a TTY")
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
                    out.append(b'\x1b')
                    if a: out.append(a)
            else:
                out.append(ch)
            ch=self._read1(0.0)
        return out

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

# ---------------------- helpers ----------------------
def clamp(x, lo, hi): return max(lo, min(hi, x))

# pattern mapping
PATTERNS = {
    'F':  (+1,+1,+1,+1),
    'B':  (-1,-1,-1,-1),
    'SL': (-1,+1,+1,-1),
    'SR': (+1,-1,-1,+1),
    'TL': (-1,-1,+1,+1),
    'TR': (+1,+1,-1,-1),
    'STOP': (0,0,0,0),
}

# ---------------------- teleop core ----------------------
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
        self._headless = bool(args.headless)
        self._kb = None

        # PCA + reserve channels (servos 8..15 OFF by default)
        self.pca = PCA9685(bus=args.i2c_bus, addr=args.pca_addr, freq_hz=args.pwm_freq, set_freq=args.set_freq)
        try:
            reserved = [int(x) for x in str(args.reserve).split(',') if x.strip()!='']
            for ch in reserved: self.pca.full_off(ch)
            if reserved: print(f"[Teleop] Reserved OFF: {reserved}")
        except Exception as e:
            print(f"[Teleop] Warning: reserve failed: {e}")

        # Motors
        mk = lambda name, fwd_pwm,fwd_in1,fwd_in2, rev_pwm,rev_in1,rev_in2, inv: HBridgeDir(
            self.pca,
            MotorDirConfig(DirTriplet(fwd_pwm,fwd_in1,fwd_in2), DirTriplet(rev_pwm,rev_in1,rev_in2), name=name),
            in_active_low=args.in_active_low,
            invert_direction=inv,
            rev_mode=getattr(args, f"{name.lower()}_rev_mode") or args.rev_mode,
            settle_s=args.settle_ms/1000.0,
            debug=args.debug_pins
        )
        self.FL = mk("FL", args.fl_fwd_pwm,args.fl_fwd_in1,args.fl_fwd_in2, args.fl_rev_pwm,args.fl_rev_in1,args.fl_rev_in2, args.invert_fl)
        self.FR = mk("FR", args.fr_fwd_pwm,args.fr_fwd_in1,args.fr_fwd_in2, args.fr_rev_pwm,args.fr_rev_in1,args.fr_rev_in2, args.invert_fr)
        self.RL = mk("RL", args.rl_fwd_pwm,args.rl_fwd_in1,args.rl_fwd_in2, args.rl_rev_pwm,args.rl_rev_in1,args.rl_rev_in2, args.invert_rl)
        self.RR = mk("RR", args.rr_fwd_pwm,args.rr_fwd_in1,args.rr_fwd_in2, args.rr_rev_pwm,args.rr_rev_in1,args.rr_rev_in2, args.invert_rr)

        self._soft_check_channels(args)

        self.duties = [0.0, 0.0, 0.0, 0.0]
        self.last_input = time.monotonic()
        self.start_time = self.last_input

        # Keyboard init (fallback to headless if no TTY)
        if not self._headless:
            try:
                self._kb = Keyboard()
            except Exception as e:
                print(f"[Teleop] No TTY for keyboard ({e}) → headless mode")
                self._headless = True

        # Safety: prevent infinite run in headless without timers
        if self._headless and self.max_runtime <= 0 and self.idle_exit <= 0:
            self.idle_exit = 5.0
            print("[Teleop] Headless with no timers → idle-exit set to 5.0s")

        # Prepare headless pattern
        self._hp_signs = PATTERNS.get(args.headless_pattern, PATTERNS['STOP'])
        self._hp_mag = float(args.headless_mag)
        self._headless_applied = False

        # Signals
        def _sig(_s,_f): self._running=False
        signal.signal(signal.SIGINT, _sig)
        signal.signal(signal.SIGTERM, _sig)

        print(
            "[Teleop] READY (dir-triplet mode). Esc/Ctrl+C/x/X to quit.\n"
            f" rate={self.hz}Hz deadman={self.deadman}s idle-exit={self.idle_exit}s max-runtime={self.max_runtime}s\n"
            f" PCA=0x{args.pca_addr:02X} bus={args.i2c_bus} pwm={args.pwm_freq}Hz  paired_mode={args.paired_mode}\n"
            f" RevModes: FL={args.fl_rev_mode or args.rev_mode} FR={args.fr_rev_mode or args.rev_mode} "
            f"RL={args.rl_rev_mode or args.rev_mode} RR={args.rr_rev_mode or args.rev_mode}\n"
            f" DebugPins={args.debug_pins}  Settle={args.settle_ms:.1f} ms  Headless={self._headless} "
            f"HeadlessPattern={args.headless_pattern} mag={self._hp_mag:.2f}\n"
        )

    # ---------------- soft safety check ----------------
    def _soft_check_channels(self, args):
        reserved = set(int(x) for x in str(args.reserve).split(',') if x.strip()!='')
        def check_triplet(name, t: DirTriplet):
            warn = []
            for label, ch in (("PWM",t.pwm),("IN1",t.in1),("IN2",t.in2)):
                if ch < 0 or ch > 7:
                    warn.append(f"{label}={ch} out of motor-safe 0..7")
                if ch in reserved:
                    warn.append(f"{label}={ch} in reserved {sorted(reserved)}")
            if len({t.pwm, t.in1, t.in2}) < 3:
                warn.append("PWM/IN overlap inside triplet (verify wiring)")
            if warn: print(f"[WARN] {name}: " + "; ".join(warn))
        check_triplet("FL.fwd", DirTriplet(self.args.fl_fwd_pwm, self.args.fl_fwd_in1, self.args.fl_fwd_in2))
        check_triplet("FL.rev", DirTriplet(self.args.fl_rev_pwm, self.args.fl_rev_in1, self.args.fl_rev_in2))
        check_triplet("FR.fwd", DirTriplet(self.args.fr_fwd_pwm, self.args.fr_fwd_in1, self.args.fr_fwd_in2))
        check_triplet("FR.rev", DirTriplet(self.args.fr_rev_pwm, self.args.fr_rev_in1, self.args.fr_rev_in2))
        check_triplet("RL.fwd", DirTriplet(self.args.rl_fwd_pwm, self.args.rl_fwd_in1, self.args.rl_fwd_in2))
        check_triplet("RL.rev", DirTriplet(self.args.rl_rev_pwm, self.args.rl_rev_in1, self.args.rl_rev_in2))
        check_triplet("RR.fwd", DirTriplet(self.args.rr_fwd_pwm, self.args.rr_fwd_in1, self.args.rr_fwd_in2))
        check_triplet("RR.rev", DirTriplet(self.args.rr_rev_pwm, self.args.rr_rev_in1, self.args.rr_rev_in2))

    # ---------------- lifecycle ----------------
    def close(self):
        print("[Teleop] Shutting down safely...")
        try:
            for m in (self.FL, self.FR, self.RL, self.RR):
                try: m.stop()
                except: pass
            if self._kb: self._kb.restore()
            self.pca.close()
        except Exception as e:
            print(f"[Teleop] Close warning: {e}")

    def loop(self):
        period = 1.0 / max(1.0, self.hz)
        try:
            while self._running:
                now = time.monotonic()
                # Hard cap
                if self.max_runtime > 0 and (now - self.start_time) >= self.max_runtime:
                    print("[Time] Max runtime reached → exiting"); break

                got = False
                if self._kb:
                    for ch in self._kb.read():
                        got = True
                        if self._handle_key(ch):
                            self._running = False; break
                        self.last_input = now
                if not self._running: break

                # Headless pattern: apply once and keep it running
                if self._headless and not self._headless_applied:
                    self.apply_pattern(self._hp_signs, self._hp_mag, "[Headless] Apply pattern")
                    self._headless_applied = True
                    self.last_input = now  # keep timers happy

                # Deadman: skip in headless (no keyboard to refresh)
                if (not self._headless) and (not got) and (now - self.last_input) > self.deadman:
                    if any(abs(x) > 1e-6 for x in self.duties):
                        print("[Deadman] STOP")
                    self.apply_pattern((0,0,0,0), 0.0)

                # Idle auto-exit
                if self.idle_exit > 0 and (now - self.last_input) > self.idle_exit:
                    print("[Idle] No input → exiting"); break

                self._apply_duties()

                sleep_t = period - (time.monotonic() - now)
                if sleep_t > 0: time.sleep(sleep_t)
        finally:
            self.close()

    # ---------------- input handling ----------------
    def _handle_key(self, ch) -> bool:
        # Quit keys: ESC, Ctrl+C, x/X
        if ch in (b"\x1b", b"\x03", b'x', b'X'):
            print("[Teleop] Quit"); return True
        if ch == b' ': self.apply_pattern((0,0,0,0), 0.0, "[Cmd] STOP"); return False
        if ch in (b'r', b'R', b"\x12"): self.scale=1.0; print("[Scale] reset → 1.0"); return False
        if ch in (b'g', b'G'): self._gentle_forward(); return False
        if ch in (b'm', b'M'): self._print_status(); return False

        slow = (isinstance(ch,bytes) and len(ch)==1 and 1 <= ch[0] <= 26)
        fast = (isinstance(ch,bytes) and ch.isalpha() and ch.isupper())
        self.scale = clamp(self.scale_low if slow else (self.scale_high if fast else 1.0), 0.05, 3.0)

        F, B, SL, SR, TL, TR = PATTERNS['F'], PATTERNS['B'], PATTERNS['SL'], PATTERNS['SR'], PATTERNS['TL'], PATTERNS['TR']
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

    # ---------------- patterns to duties ----------------
    def apply_pattern(self, signs, magnitude=1.0, announce=None):
        mag = clamp(float(magnitude) * self.scale, 0.0, 1.0)
        s = list(signs)

        if self.args.paired_mode:
            if tuple(signs) == (-1,+1,+1,-1):
                print("[Paired] SL not possible → using TL"); s = [-1,-1,+1,+1]
            elif tuple(signs) == (+1,-1,-1,+1):
                print("[Paired] SR not possible → using TR"); s = [+1,+1,-1,-1]
            s[2] = s[0]  # RL = FL
            s[3] = s[1]  # RR = FR

        self.duties = [clamp(si * mag, -1.0, 1.0) for si in s]
        if announce: print(f"{announce}  signs={tuple(s)}  mag={mag:.2f}")

    def _apply_duties(self):
        d = self.duties
        try:
            self.FL.drive(d[0]); self.FR.drive(d[1]); self.RL.drive(d[2]); self.RR.drive(d[3])
        except Exception as e:
            print(f"[Teleop] Motor drive error: {e}")

    # ---------------- utilities ----------------
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

# ---------------------- CLI ----------------------
def build_argparser():
    p = argparse.ArgumentParser(description='Robot Savo — Manual Teleop (direction-specific wiring)')

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

    # Debug / settling
    p.add_argument('--debug-pins', action='store_true', help='Print pin levels/duty on transitions')
    p.add_argument('--settle-ms', type=float, default=3.0, help='Delay after quench on dir flip')

    # Headless / exits
    p.add_argument('--headless', action='store_true', help='Run without keyboard; use with --max-runtime or --idle-exit')
    p.add_argument('--headless-pattern', choices=['F','B','SL','SR','TL','TR','STOP'], default='STOP')
    p.add_argument('--headless-mag', type=float, default=0.20)

    # Direction inversion flags
    p.add_argument('--invert-fl', action='store_true')
    p.add_argument('--invert-fr', action='store_true')
    p.add_argument('--invert-rl', action='store_true')
    p.add_argument('--invert-rr', action='store_true')

    # Global + per-wheel reverse mode
    p.add_argument('--rev-mode', choices=['std','swap'], default='std')
    p.add_argument('--fl-rev-mode', choices=['std','swap'])
    p.add_argument('--fr-rev-mode', choices=['std','swap'])
    p.add_argument('--rl-rev-mode', choices=['std','swap'])
    p.add_argument('--rr-rev-mode', choices=['std','swap'])

    # Direction-specific triplets (0..7)
    # FL
    p.add_argument('--fl-fwd-pwm', type=int, required=True)
    p.add_argument('--fl-fwd-in1', type=int, required=True)
    p.add_argument('--fl-fwd-in2', type=int, required=True)
    p.add_argument('--fl-rev-pwm', type=int, required=True)
    p.add_argument('--fl-rev-in1', type=int, required=True)
    p.add_argument('--fl-rev-in2', type=int, required=True)
    # FR
    p.add_argument('--fr-fwd-pwm', type=int, required=True)
    p.add_argument('--fr-fwd-in1', type=int, required=True)
    p.add_argument('--fr-fwd-in2', type=int, required=True)
    p.add_argument('--fr-rev-pwm', type=int, required=True)
    p.add_argument('--fr-rev-in1', type=int, required=True)
    p.add_argument('--fr-rev-in2', type=int, required=True)
    # RL
    p.add_argument('--rl-fwd-pwm', type=int, required=True)
    p.add_argument('--rl-fwd-in1', type=int, required=True)
    p.add_argument('--rl-fwd-in2', type=int, required=True)
    p.add_argument('--rl-rev-pwm', type=int, required=True)
    p.add_argument('--rl-rev-in1', type=int, required=True)
    p.add_argument('--rl-rev-in2', type=int, required=True)
    # RR
    p.add_argument('--rr-fwd-pwm', type=int, required=True)
    p.add_argument('--rr-fwd-in1', type=int, required=True)
    p.add_argument('--rr-fwd-in2', type=int, required=True)
    p.add_argument('--rr-rev-pwm', type=int, required=True)
    p.add_argument('--rr-rev-in1', type=int, required=True)
    p.add_argument('--rr-rev-in2', type=int, required=True)

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
