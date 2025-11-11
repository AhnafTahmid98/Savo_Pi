#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Manual Teleop (NO ROS2) via PCA9685 + H-Bridge
Direction-Specific Triplets + Paired-Board Mode + Safe Quench + *Shorthand triplets with 'x'*
--------------------------------------------------------------------------------------------
WHAT'S NEW (per your request)
- **Shorthand flags**: you can now pass triplets as a single flag, e.g.
    --fl-fwd 1,0,x   --fl-rev 0,1,3
  Same for FR/RL/RR and FWD/REV. The third value may be **x** (= don't-touch).
- **'x' (ignore) support**: if an IN pin is 'x' (or -1), we won't drive it.
  Useful for your board where the second IN seems irrelevant per direction.
- **Backwards-compatible**: the long form (--fl-fwd-pwm/--fl-fwd-in1/--fl-fwd-in2...) still works.
- **Safe quench** retained: on direction flip, both triplets are idled then we wait --settle-ms before energizing the new set.

Keys
  W/S/A/D/Q/E = drive (mecanum patterns) • Space = STOP • G = gentle pulse
  1..4 / 5..8 = pulse FL/FR/RL/RR forward / reverse • M = show duties • R = reset scale

Notes
- Motor channels should be 0..7. Servos 8..15 are reserved OFF by default.
- If you use 'x', wiring must ensure that ignored line is already safe.
"""

import argparse, os, select, sys, termios, time, tty, signal
from dataclasses import dataclass
from typing import Optional, Tuple

# ---------------------- I2C ----------------------
try:
    from smbus2 import SMBus
except Exception:
    print("ERROR: smbus2 missing. Install: sudo apt install -y python3-smbus OR pip3 install smbus2", file=sys.stderr)
    raise

# ---------------------- PCA9685 low-level ----------------------
MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, OUTDRV, ALLCALL = 0x80, 0x10, 0x20, 0x04, 0x01

class PCA9685:
    def __init__(self, bus: int = 1, addr: int = 0x40, freq_hz: float = 800.0, set_freq: bool = True):
        self.addr = int(addr); self.bus = SMBus(int(bus))
        self._w8(MODE2, OUTDRV); self._w8(MODE1, AI); time.sleep(0.003)
        if set_freq:
            self.set_pwm_freq(freq_hz)
        m1 = self._r8(MODE1); self._w8(MODE1, (m1 | RESTART | AI) & ~ALLCALL)

    def close(self):
        try: self.bus.close()
        except Exception: pass

    def _w8(self, reg, val):
        self.bus.write_byte_data(self.addr, reg & 0xFF, val & 0xFF)

    def _r8(self, reg):
        return self.bus.read_byte_data(self.addr, reg & 0xFF)

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

# ---------------------- Direction-specific motor model ----------------------
@dataclass
class DirTriplet:
    pwm: Optional[int]
    in1: Optional[int]
    in2: Optional[int]

@dataclass
class MotorDirConfig:
    fwd: DirTriplet
    rev: DirTriplet
    name: str = "M"

class HBridgeDir:
    """
    Uses EXACT (pwm, in1, in2) triplets per direction (forward vs reverse).
    Safe quench: when switching direction, idle *both* triplets and wait.
    'x' (None) pins are ignored (not driven). Use only if your wiring is safe.
    """
    def __init__(self, pca: PCA9685, cfg: MotorDirConfig, in_active_low=False, settle_ms: float = 12.0):
        self.pca = pca
        self.cfg = cfg
        self.in_active_low = bool(in_active_low)
        self.settle_ms = float(settle_ms)
        self._last_sign = 0  # -1, 0, +1
        self.stop()

    # --- low level ---
    def _digital(self, ch: Optional[int], level: int):
        if ch is None: return
        if self.in_active_low:
            level ^= 1
        (self.pca.full_on if level else self.pca.full_off)(ch)

    def _idle_triplet(self, dcfg: DirTriplet):
        # Order: drop PWM first (if any), then INs.
        try:
            if dcfg.pwm is not None:
                self.pca.set_duty(dcfg.pwm, 0.0)
        except Exception:
            pass
        self._digital(dcfg.in1, 0)
        self._digital(dcfg.in2, 0)

    def _quench_both(self):
        self._idle_triplet(self.cfg.fwd)
        self._idle_triplet(self.cfg.rev)

    def _apply_dir(self, dcfg: DirTriplet, duty: float, forward_logic: bool):
        # Logic: A=1,B=0 for forward; A=0,B=1 for reverse on chosen triplet
        if forward_logic:
            self._digital(dcfg.in1, 1); self._digital(dcfg.in2, 0)
        else:
            self._digital(dcfg.in1, 0); self._digital(dcfg.in2, 1)
        if dcfg.pwm is not None:
            self.pca.set_duty(dcfg.pwm, duty)

    # --- public ---
    def drive(self, duty_signed: float):
        d = max(-1.0, min(1.0, float(duty_signed)))
        sign = 0 if abs(d) < 1e-3 else (1 if d > 0 else -1)
        if sign == 0:
            self.stop(); self._last_sign = 0; return
        if self._last_sign != 0 and sign != self._last_sign:
            self._quench_both()
            time.sleep(self.settle_ms / 1000.0)
        if sign > 0:
            self._apply_dir(self.cfg.fwd, +d, forward_logic=True)
        else:
            self._apply_dir(self.cfg.rev, -d, forward_logic=False)
        self._last_sign = sign

    def stop(self):
        self._quench_both()

# ---------------------- keyboard ----------------------
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
            if ch==b'':
                a=self._read1(0.02)
                if a in (b'[',b'O'):
                    b=self._read1(0.04)
                    arrows={b'A':b'UP',b'B':b'DOWN',b'C':b'RIGHT',b'D':b'LEFT'}
                    if b in arrows: out.append(arrows[b])
                else:
                    out.append(b'');
                    if a: out.append(a)
            else:
                out.append(ch)
            ch=self._read1(0.0)
        return out

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

# ---------------------- helpers ----------------------
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def parse_triplet(s: Optional[str]) -> Optional[Tuple[Optional[int],Optional[int],Optional[int]]]:
    if not s: return None
    parts = [p.strip().lower() for p in str(s).split(',')]
    if len(parts) != 3: raise ValueError(f"Triplet must be 'a,b,c' got: {s}")
    def tok(t):
        if t in ('x','-1','none',''): return None
        return int(t, 0)
    return (tok(parts[0]), tok(parts[1]), tok(parts[2]))

# ---------------------- teleop core ----------------------
class DirectTeleop:
    def __init__(self, args):
        self.args = args
        self._expand_shorthand(args)
        self._validate_args(args)

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

        # Build direction-specific configs from parsed values
        self.FL = HBridgeDir(self.pca, MotorDirConfig(
            fwd=DirTriplet(args.fl_fwd_pwm, args.fl_fwd_in1, args.fl_fwd_in2),
            rev=DirTriplet(args.fl_rev_pwm, args.fl_rev_in1, args.fl_rev_in2),
            name="FL"
        ), in_active_low=args.in_active_low, settle_ms=args.settle_ms)
        self.FR = HBridgeDir(self.pca, MotorDirConfig(
            fwd=DirTriplet(args.fr_fwd_pwm, args.fr_fwd_in1, args.fr_fwd_in2),
            rev=DirTriplet(args.fr_rev_pwm, args.fr_rev_in1, args.fr_rev_in2),
            name="FR"
        ), in_active_low=args.in_active_low, settle_ms=args.settle_ms)
        self.RL = HBridgeDir(self.pca, MotorDirConfig(
            fwd=DirTriplet(args.rl_fwd_pwm, args.rl_fwd_in1, args.rl_fwd_in2),
            rev=DirTriplet(args.rl_rev_pwm, args.rl_rev_in1, args.rl_rev_in2),
            name="RL"
        ), in_active_low=args.in_active_low, settle_ms=args.settle_ms)
        self.RR = HBridgeDir(self.pca, MotorDirConfig(
            fwd=DirTriplet(args.rr_fwd_pwm, args.rr_fwd_in1, args.rr_fwd_in2),
            rev=DirTriplet(args.rr_rev_pwm, args.rr_rev_in1, args.rr_rev_in2),
            name="RR"
        ), in_active_low=args.in_active_low, settle_ms=args.settle_ms)

        self._soft_check_channels(args)
        self.duties = [0.0, 0.0, 0.0, 0.0]
        self.last_input = time.monotonic()
        self.start_time = self.last_input

        # Signals
        def _sig(_s,_f): self._running=False
        signal.signal(signal.SIGINT, _sig)
        signal.signal(signal.SIGTERM, _sig)

        print(
            "[Teleop] READY (dir-triplet mode, safe-quench, shorthand x). Esc/Ctrl+C to quit."
            f" rate={self.hz}Hz deadman={self.deadman}s idle-exit={self.idle_exit}s max-runtime={self.max_runtime}s"
            f" PCA=0x{args.pca_addr:02X} bus={args.i2c_bus} pwm={args.pwm_freq}Hz paired_mode={args.paired_mode} settle={args.settle_ms}ms in_active_low={args.in_active_low}"
        )

    # ---------- shorthand expansion & validation ----------
    def _apply_triplet(self, prefix: str, t: Tuple[Optional[int],Optional[int],Optional[int]]):
        setattr(self.args, f"{prefix}_pwm", t[0])
        setattr(self.args, f"{prefix}_in1", t[1])
        setattr(self.args, f"{prefix}_in2", t[2])

    def _expand_shorthand(self, args):
        for side in ("fl","fr","rl","rr"):
            for pol in ("fwd","rev"):
                key = f"{side}_{pol}"
                t = parse_triplet(getattr(args, key))
                if t is not None:
                    self._apply_triplet(f"{side}_{pol}", t)

    def _validate_args(self, args):
        # Ensure every required field exists after expansion; allow None for 'x'.
        missing = []
        for side in ("fl","fr","rl","rr"):
            for pol in ("fwd","rev"):
                for field in ("pwm","in1","in2"):
                    attr = f"{side}_{pol}_{field}"
                    if not hasattr(args, attr):
                        missing.append(attr)
        if missing:
            raise SystemExit("Missing required triplet values (use long form or shorthand): " + ", ".join(missing))

    # ---------------- soft safety check ----------------
    def _soft_check_channels(self, args):
        reserved = set(int(x) for x in str(args.reserve).split(',') if x.strip()!='')

        def check_triplet(name, t: DirTriplet):
            warn = []
            for label, ch in (("PWM",t.pwm),("IN1",t.in1),("IN2",t.in2)):
                if ch is None: continue
                if ch < 0 or ch > 7: warn.append(f"{label}={ch} out of motor-safe 0..7")
                if ch in reserved: warn.append(f"{label}={ch} in reserved {sorted(reserved)}")
            alive = [c for c in (t.in1, t.in2) if c is not None]
            if len(alive) == 0:
                warn.append("no IN lines active (x,x) → nothing to drive")
            if t.pwm is None:
                warn.append("PWM is 'x' → no speed control on this direction")
            if warn: print(f"[WARN] {name}: " + "; ".join(warn))

        # Per-triplet
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

                if not got and (now - self.last_input) > self.deadman:
                    if any(abs(x) > 1e-6 for x in self.duties):
                        print("[Deadman] STOP")
                        self.apply_pattern((0,0,0,0), 0.0)
                    if self.idle_exit > 0 and (now - self.last_input) > self.idle_exit:
                        print("[Idle] No input → exiting"); break

                self._apply_duties()
                sleep_t = period - (time.monotonic() - now)
                if sleep_t > 0: time.sleep(sleep_t)
        finally:
            self.close()

    # ---------------- input handling ----------------
    def _handle_key(self, ch) -> bool:
        if ch in (b"", b""):
            print("[Teleop] Quit"); return True
        if ch == b' ':
            self.apply_pattern((0,0,0,0), 0.0, "[Cmd] STOP"); return False
        if ch in (b'r', b'R', b""):
            self.scale=1.0; print("[Scale] reset → 1.0"); return False
        if ch in (b'g', b'G'):
            self._gentle_forward(); return False
        if ch in (b'm', b'M'):
            self._print_status(); return False

        slow = (isinstance(ch,bytes) and len(ch)==1 and 1 <= ch[0] <= 26)
        fast = (isinstance(ch,bytes) and ch.isalpha() and ch.isupper())
        self.scale = clamp(self.scale_low if slow else (self.scale_high if fast else 1.0), 0.05, 3.0)

        # patterns
        F = (+1,+1,+1,+1)
        B = (-1,-1,-1,-1)
        SL = (-1,+1,+1,-1)
        SR = (+1,-1,-1,+1)
        TL = (-1,-1,+1,+1)
        TR = (+1,+1,-1,-1)

        if ch in (b'w', b'W', b'UP'):
            self.apply_pattern(F, announce='[Cmd] F (forward)')
        elif ch in (b's', b'S', b'DOWN'):
            self.apply_pattern(B, announce='[Cmd] B (back)')
        elif ch in (b'a', b'A', b'LEFT'):
            self.apply_pattern(SL, announce='[Cmd] SL (strafe L)')
        elif ch in (b'd', b'D', b'RIGHT'):
            self.apply_pattern(SR, announce='[Cmd] SR (strafe R)')
        elif ch in (b'q', b'Q'):
            self.apply_pattern(TL, announce='[Cmd] TL (turn CCW)')
        elif ch in (b'e', b'E'):
            self.apply_pattern(TR, announce='[Cmd] TR (turn CW)')
        elif ch == b'1':
            self._pulse_wheels([1,0,0,0]); return False
        elif ch == b'2':
            self._pulse_wheels([0,1,0,0]); return False
        elif ch == b'3':
            self._pulse_wheels([0,0,1,0]); return False
        elif ch == b'4':
            self._pulse_wheels([0,0,0,1]); return False
        elif ch == b'5':
            self._pulse_wheels([1,0,0,0], -0.12); return False
        elif ch == b'6':
            self._pulse_wheels([0,1,0,0], -0.12); return False
        elif ch == b'7':
            self._pulse_wheels([0,0,1,0], -0.12); return False
        elif ch == b'8':
            self._pulse_wheels([0,0,0,1], -0.12); return False
        return False

    # ---------------- patterns to duties ----------------
    def apply_pattern(self, signs, magnitude=1.0, announce=None):
        """ signs: (FL, FR, RL, RR) each ∈ {-1,0,+1} """
        mag = clamp(float(magnitude) * self.scale, 0.0, 1.0)
        s = list(signs)
        if self.args.paired_mode:
            if tuple(signs) == (-1,+1,+1,-1):  # SL → TL
                print("[Paired] SL not possible → using TL"); s = [-1,-1,+1,+1]
            elif tuple(signs) == (+1,-1,-1,+1):  # SR → TR
                print("[Paired] SR not possible → using TR"); s = [+1,+1,-1,-1]
            s[2] = s[0]  # RL = FL
            s[3] = s[1]  # RR = FR
        self.duties = [clamp(si * mag, -1.0, 1.0) for si in s]
        if announce:
            print(f"{announce} signs={tuple(s)} mag={mag:.2f}")

    def _apply_duties(self):
        d = self.duties
        try:
            self.FL.drive(d[0])
            self.FR.drive(d[1])
            self.RL.drive(d[2])
            self.RR.drive(d[3])
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
        print(f"[duties] FL {d[0]:+0.2f} FR {d[1]:+0.2f} RL {d[2]:+0.2f} RR {d[3]:+0.2f}")

# ---------------------- CLI ----------------------
def build_argparser():
    p = argparse.ArgumentParser(description='Robot Savo — Manual Teleop (direction-specific wiring + safe quench + shorthand)')
    # timing / exit / scaling
    p.add_argument('--hz', type=float, default=30.0)
    p.add_argument('--deadman', type=float, default=0.40)
    p.add_argument('--idle-exit', type=float, default=0.0, help='Auto-exit after SEC idle (0=never)')
    p.add_argument('--max-runtime', type=float, default=0.0, help='Hard exit after SEC (0=never)')
    p.add_argument('--scale-low', type=float, default=0.25)
    p.add_argument('--scale-high', type=float, default=1.35)
    p.add_argument('--settle-ms', type=float, default=12.0, help='Delay after quench before energizing new direction')

    # I2C/PCA
    p.add_argument('--i2c-bus', type=int, default=1)
    p.add_argument('--pca-addr', type=lambda x:int(x,0), default=0x40)
    p.add_argument('--pwm-freq', type=float, default=800.0)
    p.add_argument('--set-freq', dest='set_freq', action='store_true', default=True)
    p.add_argument('--no-set-freq', dest='set_freq', action='store_false')
    p.add_argument('--reserve', type=str, default='8,9,10,11,12,13,14,15')
    p.add_argument('--in-active-low', action='store_true')

    # ---------- NEW shorthand triplets ----------
    for side in ("fl","fr","rl","rr"):
        p.add_argument(f'--{side}-fwd', type=str, help="shorthand 'pwm,in1,in2' (use x to ignore)")
        p.add_argument(f'--{side}-rev', type=str, help="shorthand 'pwm,in1,in2' (use x to ignore)")

    # ---------- Long form (optional now; filled by shorthand if given) ----------
    for side in ("fl","fr","rl","rr"):
        for pol in ("fwd","rev"):
            p.add_argument(f'--{side}-{pol}-pwm', type=lambda x: None if str(x).lower() in ('x','-1','none') else int(x), required=False)
            p.add_argument(f'--{side}-{pol}-in1', type=lambda x: None if str(x).lower() in ('x','-1','none') else int(x), required=False)
            p.add_argument(f'--{side}-{pol}-in2', type=lambda x: None if str(x).lower() in ('x','-1','none') else int(x), required=False)

    # Force side pairing
    p.add_argument('--paired-mode', action='store_true', help='Force FL=RL and FR=RR; map SL/SR to TL/TR')
    return p


def main(argv=None):
    args = build_argparser().parse_args(argv)
    t = None
    try:
        t = DirectTeleop(args)
        t.loop()
    except KeyboardInterrupt:
        print("[Main] KeyboardInterrupt")
    finally:
        if t: t.close()
        print("[Main] Exit complete")

if __name__ == '__main__':
    main()