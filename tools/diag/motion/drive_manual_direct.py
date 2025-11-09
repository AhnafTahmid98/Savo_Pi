#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Manual Teleop (NO ROS2) for DC Motors via PCA9685 + H-Bridge
---------------------------------------------------------------------------
MOTOR-ONLY (no servo control). Safe on FNK0043 that shares PCA9685 with servos.

What this script guarantees
- Uses **physical PCA9685 channels only** (no Freenove 'servo' logical mapping),
  so 0..7 are PCA 0..7, and 8..15 are PCA 8..15.
- **Refuses** to drive any channel ≥ 8. You must map all motor channels to 0..7.
- Does **not** change PWM frequency unless you pass `--set-freq`.
- FULL-OFF on `--reserve` channels at start (default 12,13,14,15) so servos stay idle.
- H-bridge style control: IN1/IN2 = full-on/full-off, PWM = duty (0..1).

Keys
  W / ↑ : Forward        S / ↓ : Backward
  A / ← : Strafe left    D / → : Strafe right   (set --max-vy 0.0 to disable)
  Q     : Turn CCW       E     : Turn CW
  SPACE : Stop           M     : Print duties
  1..4  : Pulse FL/FR/RL/RR forward    5..8 : reverse
  Shift : fast scale     Ctrl  : slow scale
  R     : reset speed scale to 1.0
  Esc / Ctrl+C : Quit (safe stop)

Tip for FNK0043 paired wiring
- If your board is hard-wired so **FL & RL share PWM and INs** (and similarly FR & RR),
  just pass the same numbers for those motors (e.g., `--fl-*` == `--rl-*`, `--fr-*` == `--rr-*`).
  The script will still **never** touch 8..15.
"""

import argparse, os, select, sys, termios, time, tty, signal
from dataclasses import dataclass

# ---------------- deps ----------------
try:
    from smbus2 import SMBus
except Exception:
    print("ERROR: smbus2 is required. Install: sudo apt install -y python3-smbus || pip3 install smbus2", file=sys.stderr)
    raise

# ---------------- PCA9685 (physical channels) ----------------
MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, OUTDRV, ALLCALL = 0x80, 0x10, 0x20, 0x04, 0x01

class PCA9685:
    def __init__(self, bus=1, addr=0x40, freq_hz=800.0, set_freq=False):
        self.addr = int(addr)
        self.bus  = SMBus(int(bus))
        # push-pull; auto-increment; DO NOT use ALLCALL
        self._w8(MODE2, OUTDRV)
        self._w8(MODE1, AI)
        time.sleep(0.003)
        if set_freq:
            self.set_pwm_freq(freq_hz)
        # ensure restart + AI, and ALLCALL cleared
        m1 = self._r8(MODE1)
        self._w8(MODE1, (m1 | RESTART | AI) & ~ALLCALL)

    def close(self):
        try: self.bus.close()
        except Exception: pass

    def _w8(self, reg, val): self.bus.write_byte_data(self.addr, reg & 0xFF, val & 0xFF)
    def _r8(self, reg):      return self.bus.read_byte_data(self.addr, reg & 0xFF)

    def set_pwm_freq(self, freq_hz: float):
        f = float(max(40.0, min(1500.0, freq_hz)))
        prescale = int(max(3, min(255, round(25_000_000.0 / (4096.0 * f) - 1.0))))
        old = self._r8(MODE1)
        self._w8(MODE1, (old & ~RESTART) | SLEEP)
        self._w8(PRESCALE, prescale)
        self._w8(MODE1, old & ~SLEEP)
        time.sleep(0.003)
        self._w8(MODE1, (old | RESTART | AI) & ~ALLCALL)

    def _raw(self, ch: int, on: int, off: int):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0, on & 0xFF)
        self._w8(base+1, (on >> 8) & 0x0F)
        self._w8(base+2, off & 0xFF)
        self._w8(base+3, (off >> 8) & 0x0F)

    def full_off(self, ch: int):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0, 0x00); self._w8(base+1, 0x00)
        self._w8(base+2, 0x00); self._w8(base+3, 0x10)  # OFF_H full-off bit

    def full_on(self, ch: int):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0, 0x00); self._w8(base+1, 0x10)  # ON_H full-on bit
        self._w8(base+2, 0x00); self._w8(base+3, 0x00)

    def set_duty(self, ch: int, duty: float):
        duty = max(0.0, min(1.0, float(duty)))
        off = int(round(duty * 4095))
        if off <= 0: self.full_off(ch)
        elif off >= 4095: self.full_on(ch)
        else: self._raw(ch, 0, off)

# ---------------- H-bridge primitive (PCA channels only) ----------------
@dataclass
class MotorCH:
    pwm: int
    in1: int
    in2: int

class HBridge:
    def __init__(self, pca: PCA9685, ch: MotorCH, invert=False, in_active_low=False, swap_in12=False):
        self.pca, self.ch = pca, ch
        self.invert = bool(invert)
        self.in_active_low = bool(in_active_low)
        self.swap = bool(swap_in12)
        self.stop()

    def _digital(self, channel: int, level: int):
        # Active-high unless in_active_low
        if self.in_active_low: level ^= 1
        (self.pca.full_on if level else self.pca.full_off)(channel)

    def _dir(self, a_on: int, b_on: int):
        a, b = (self.ch.in1, self.ch.in2) if not self.swap else (self.ch.in2, self.ch.in1)
        self._digital(a, a_on); self._digital(b, b_on)

    def drive(self, signed: float):
        d = max(-1.0, min(1.0, float(signed)))
        if self.invert: d = -d
        if abs(d) < 1e-3: self.stop(); return
        if d > 0:  # forward
            self._dir(1, 0); self.pca.set_duty(self.ch.pwm, d)
        else:      # reverse
            self._dir(0, 1); self.pca.set_duty(self.ch.pwm, -d)

    def stop(self):
        self._dir(0, 0); self.pca.set_duty(self.ch.pwm, 0.0)

# ---------------- keyboard ----------------
class Keyboard:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
    def _read1(self, timeout=0.0):
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return os.read(self.fd, 1) if r else None
    def read(self):
        out = []
        ch = self._read1(0.0)
        while ch is not None:
            if ch == b"\x1b":
                a = self._read1(0.02)
                if a in (b'[', b'O'):
                    b = self._read1(0.04)
                    arrows = {b'A': b'UP', b'B': b'DOWN', b'C': b'RIGHT', b'D': b'LEFT'}
                    if b in arrows: out.append(arrows[b])
                else:
                    out.append(b"\x1b")
                    if a: out.append(a)
            else:
                out.append(ch)
            ch = self._read1(0.0)
        return out
    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

# ---------------- teleop core ----------------
def clamp(x, lo, hi): return max(lo, min(hi, x))

class Teleop:
    # Freenove table (M1,M2,M3,M4 = FL,FR,RL,RR)
    V_F  = (+1, +1, +1, +1)
    V_B  = (-1, -1, -1, -1)
    V_L  = (-1, +1, +1, -1)   # strafe left
    V_R  = (+1, -1, -1, +1)   # strafe right
    V_TL = (-1, -1, +1, +1)   # turn CCW
    V_TR = (+1, +1, -1, -1)   # turn CW

    def __init__(self, a):
        self.hz        = float(a.hz)
        self.deadman   = float(a.deadman)
        self.scale_low = float(a.scale_low)
        self.scale_high= float(a.scale_high)
        self.scale     = 1.0

        # refuse any PCA channel 8..15
        mot_ch = [a.fl_pwm, a.fl_in1, a.fl_in2, a.fr_pwm, a.fr_in1, a.fr_in2,
                  a.rl_pwm, a.rl_in1, a.rl_in2, a.rr_pwm, a.rr_in1, a.rr_in2]
        bad = sorted({ch for ch in mot_ch if ch >= 8})
        if bad:
            raise SystemExit(f"ERROR: motor channels {bad} are ≥ 8 (servo header). "
                             f"Map all motor PWM/IN to PCA channels 0..7 only.")

        self.ws = [int(x) for x in a.wheel_signs.split(',')]
        if len(self.ws) != 4 or any(s not in (-1, 1) for s in self.ws):
            raise SystemExit("--wheel-signs must be CSV of four values in {-1,1}")

        self.pca = PCA9685(bus=a.i2c_bus, addr=a.pca_addr, freq_hz=a.pwm_freq, set_freq=a.set_freq)

        # reserve (FULL-OFF) any channels you want untouched (default 12,13,14,15)
        try:
            reserved = [int(x) for x in str(a.reserve).split(',') if x.strip()]
            for ch in reserved: self.pca.full_off(ch)
            if reserved: print(f"[Teleop] Reserved OFF: {reserved}")
        except Exception:
            pass

        self.M = [
            HBridge(self.pca, MotorCH(a.fl_pwm, a.fl_in1, a.fl_in2), a.inv_fl, a.in_active_low, a.swap_in12_fl),
            HBridge(self.pca, MotorCH(a.fr_pwm, a.fr_in1, a.fr_in2), a.inv_fr, a.in_active_low, a.swap_in12_fr),
            HBridge(self.pca, MotorCH(a.rl_pwm, a.rl_in1, a.rl_in2), a.inv_rl, a.in_active_low, a.swap_in12_rl),
            HBridge(self.pca, MotorCH(a.rr_pwm, a.rr_in1, a.rr_in2), a.inv_rr, a.in_active_low, a.swap_in12_rr),
        ]

        # flips: fix W/S on FNK0043 (default True for vx)
        self.flip_vx = bool(a.flip_vx)
        self.flip_vy = bool(a.flip_vy)
        self.flip_om = bool(a.flip_omega)

        self.kb = Keyboard()
        self.last_in = time.monotonic()
        self.t_vx = self.t_vy = self.t_wz = 0.0
        self.c_vx = self.c_vy = self.c_wz = 0.0
        self._duties = [0.0, 0.0, 0.0, 0.0]
        print("[Teleop] READY. W/S forward/back, A/D strafe, Q/E turn. Esc=quit")

    def close(self):
        try: self.kb.restore()
        except Exception: pass
        for m in self.M:
            try: m.stop()
            except Exception: pass
        try: self.pca.close()
        except Exception: pass

    def _apply_vec(self, vec, mag):
        d = [self.ws[i] * clamp(vec[i]*mag, -1.0, 1.0) for i in range(4)]
        for i, m in enumerate(self.M): m.drive(d[i])
        self._duties = d

    def _mix_and_apply(self, vx, vy, wz):
        # optional axis flips (default flip_vx=True so W=forward)
        if self.flip_vx: vx = -vx
        if self.flip_vy: vy = -vy
        if self.flip_om: wz = -wz

        sx = abs(vx); sy = abs(vy); so = abs(wz)
        acc = [0.0, 0.0, 0.0, 0.0]
        def add(vec, s):
            for i in range(4): acc[i] += vec[i]*s
        if   vx > 1e-6: add(self.V_F,  sx)
        elif vx < -1e-6:add(self.V_B,  sx)
        if   vy > 1e-6: add(self.V_L,  sy)
        elif vy < -1e-6:add(self.V_R,  sy)
        if   wz > 1e-6: add(self.V_TL, so)
        elif wz < -1e-6:add(self.V_TR, so)
        # clamp + apply signs
        for i in range(4):
            acc[i] = self.ws[i] * clamp(acc[i], -1.0, 1.0)
            self.M[i].drive(acc[i])
        self._duties = acc

    def _print_status(self, label=None):
        d = self._duties
        print(f"{('['+label+'] ') if label else ''}"
              f"M1 {d[0]:+0.2f}  M2 {d[1]:+0.2f}  M3 {d[2]:+0.2f}  M4 {d[3]:+0.2f}")

    def _pulse(self, mask, duty, t=0.35):
        for i, m in enumerate(self.M): m.drive(duty if mask[i] else 0.0)
        time.sleep(t)
        for m in self.M: m.stop()

    def _handle_key(self, ch):
        # quit
        if ch in (b"\x1b", b"\x03"): print("[Teleop] Quit"); return True
        # stop
        if ch == b' ':
            self.t_vx = self.t_vy = self.t_wz = 0.0
            for m in self.M: m.stop()
            print("[Cmd] STOP"); return False
        # print
        if ch in (b'm', b'M'): self._print_status("duties"); return False
        # scale (Ctrl slow / Shift fast)
        if isinstance(ch, bytes) and len(ch) == 1:
            slow = 1 <= ch[0] <= 26
            fast = ch.isalpha() and ch.isupper()
            self.scale = self.scale_low if slow else (self.scale_high if fast else 1.0)
        # motions
        if ch in (b'w', b'W', b'UP'):
            self.t_vx = +1.0; print("[Move] Forward")
        elif ch in (b's', b'S', b'DOWN'):
            self.t_vx = -1.0; print("[Move] Backward")
        elif ch in (b'a', b'A', b'LEFT'):
            self.t_vy = +1.0; print("[Move] Left (strafe)")
        elif ch in (b'd', b'D', b'RIGHT'):
            self.t_vy = -1.0; print("[Move] Right (strafe)")
        elif ch in (b'q', b'Q'):
            self.t_wz = +1.0; print("[Turn] CCW")
        elif ch in (b'e', b'E'):
            self.t_wz = -1.0; print("[Turn] CW")
        elif ch in (b'x', b'X'):
            self.t_vx = 0.0; self.t_vy = 0.0; print("[Cmd] Zero VX/VY")
        elif ch in (b'z', b'Z'):
            self.t_wz = 0.0; print("[Cmd] Zero WZ")
        # wheel pulses
        elif ch == b'1': self._pulse([1,0,0,0], +0.12)
        elif ch == b'2': self._pulse([0,1,0,0], +0.12)
        elif ch == b'3': self._pulse([0,0,1,0], +0.12)
        elif ch == b'4': self._pulse([0,0,0,1], +0.12)
        elif ch == b'5': self._pulse([1,0,0,0], -0.12)
        elif ch == b'6': self._pulse([0,1,0,0], -0.12)
        elif ch == b'7': self._pulse([0,0,1,0], -0.12)
        elif ch == b'8': self._pulse([0,0,0,1], -0.12)
        return False

    def loop(self, hz):
        period = 1.0 / hz
        last = time.monotonic()
        try:
            while True:
                t0 = time.monotonic()
                got = False
                for ch in self.kb.read():
                    got = True
                    if self._handle_key(ch): return
                    self.last_in = t0
                if not got and (t0 - self.last_in) > self.deadman:
                    self.t_vx = self.t_vy = self.t_wz = 0.0
                # apply current command with scaling
                vx = self.t_vx * self.scale
                vy = self.t_vy * self.scale
                wz = self.t_wz * self.scale
                self._mix_and_apply(vx, vy, wz)
                # tick
                dt = time.monotonic() - t0
                time.sleep(max(0.0, period - dt))
        finally:
            self.close()

# ---------------- CLI ----------------
def build_ap():
    p = argparse.ArgumentParser(description="Robot Savo — Manual Teleop (PCA9685 + H-bridge, servo-safe)")
    # rate & safety
    p.add_argument('--hz', type=float, default=30.0)
    p.add_argument('--deadman', type=float, default=0.25)
    p.add_argument('--scale-low', type=float, default=0.20)
    p.add_argument('--scale-high', type=float, default=1.5)
    # I2C / PCA
    p.add_argument('--i2c-bus', type=int, default=1)
    p.add_argument('--pca-addr', type=lambda x:int(x,0), default=0x40)
    p.add_argument('--pwm-freq', type=float, default=800.0)
    p.add_argument('--set-freq', action='store_true', help="Actually set PCA9685 PWM frequency (default: leave as-is)")
    p.add_argument('--reserve', type=str, default='12,13,14,15', help='Channels to FULL-OFF at startup (comma list)')
    # channel mapping (PHYSICAL PCA channels; MUST be 0..7)
    p.add_argument('--fl-pwm', type=int, required=True); p.add_argument('--fl-in1', type=int, required=True); p.add_argument('--fl-in2', type=int, required=True)
    p.add_argument('--fr-pwm', type=int, required=True); p.add_argument('--fr-in1', type=int, required=True); p.add_argument('--fr-in2', type=int, required=True)
    p.add_argument('--rl-pwm', type=int, required=True); p.add_argument('--rl-in1', type=int, required=True); p.add_argument('--rl-in2', type=int, required=True)
    p.add_argument('--rr-pwm', type=int, required=True); p.add_argument('--rr-in1', type=int, required=True); p.add_argument('--rr-in2', type=int, required=True)
    # polarity helpers
    p.add_argument('--inv-fl', action='store_true'); p.add_argument('--inv-fr', action='store_true')
    p.add_argument('--inv-rl', action='store_true'); p.add_argument('--inv-rr', action='store_true')
    p.add_argument('--in-active-low', action='store_true')
    p.add_argument('--swap-in12-fl', action='store_true'); p.add_argument('--swap-in12-fr', action='store_true')
    p.add_argument('--swap-in12-rl', action='store_true'); p.add_argument('--swap-in12-rr', action='store_true')
    # wheel sign fixes (if wiring flips a motor mechanically/electrically)
    p.add_argument('--wheel-signs', type=str, default='+1,+1,+1,+1', help='CSV of -1/1 for (M1,M2,M3,M4)')
    # axis flips (default flip-vx True so W=forward on FNK0043)
    p.add_argument('--flip-vx', action='store_true', default=True)
    p.add_argument('--flip-vy', action='store_true', default=False)
    p.add_argument('--flip-omega', action='store_true', default=False)
    return p

def main(argv=None):
    args = build_ap().parse_args(argv)
    t = Teleop(args)
    try:
        t.loop(args.hz)
    except KeyboardInterrupt:
        pass
    finally:
        t.close()

if __name__ == '__main__':
    main()
