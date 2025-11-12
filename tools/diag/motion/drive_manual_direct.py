#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Expert PCA9685 Mecanum Teleop (NO ROS2, letters only)
------------------------------------------------------------------
Controls
  W / S : forward / backward
  A / D : strafe left / right
  Q / E : rotate CCW / CW
  X or SPACE : stop (zero all)
  Z / C : speed scale down / up
  R : reset velocities & scale
  ESC : quit

Highlights
  • Embedded minimal PCA9685 driver (I²C @ 0x40 by default).
  • Robust CLI: bus/address/frequency, scale & kinematics, per-wheel invert, quench ms.
  • “Quench” neutral on direction sign flip to protect H-bridge latching.
  • Optional --poke mode to test each wheel in +/− without keyboard.
  • Smooth decay, duty clamping, safe shutdown on Ctrl+C.

Examples
  # default run (WASD only)
  python3 teleop_mecanum.py

  # gentler power and stronger rotation, with longer quench
  python3 teleop_mecanum.py --max-duty 2400 --turn-gain 1.2 --quench-ms 22

  # if forward is backwards for your wiring (flip global forward sign)
  python3 teleop_mecanum.py --forward-sign -1

  # quick wheel poke test instead of teleop
  python3 teleop_mecanum.py --poke
"""

import sys, time, tty, termios, select, math, argparse
import smbus

# ================== PCA9685 (embedded) ==================
class PCA9685:
    __SUBADR1=0x02; __SUBADR2=0x03; __SUBADR3=0x04
    __MODE1=0x00;   __PRESCALE=0xFE
    __LED0_ON_L=0x06; __LED0_ON_H=0x07
    __LED0_OFF_L=0x08; __LED0_OFF_H=0x09
    __ALLLED_ON_L=0xFA; __ALLLED_ON_H=0xFB
    __ALLLED_OFF_L=0xFC; __ALLLED_OFF_H=0xFD

    def __init__(self, bus: int = 1, address: int = 0x40, debug: bool = False):
        self.busno = bus
        self.bus = smbus.SMBus(bus)
        self.address = address
        self.debug = debug
        self.write(self.__MODE1, 0x00)

    def write(self, reg: int, value: int) -> None:
        self.bus.write_byte_data(self.address, reg, value & 0xFF)

    def read(self, reg: int) -> int:
        return self.bus.read_byte_data(self.address, reg)

    def set_pwm_freq(self, freq: float) -> None:
        """Set PWM frequency in Hz (motor-friendly carriers often 50–1000 Hz)."""
        prescaleval = 25_000_000.0 / 4096.0 / float(freq) - 1.0
        prescale = int(math.floor(prescaleval + 0.5))
        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10          # sleep
        self.write(self.__MODE1, newmode)
        self.write(self.__PRESCALE, prescale)
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)   # restart

    def set_pwm(self, channel: int, on: int, off: int) -> None:
        base = self.__LED0_ON_L + 4 * channel
        self.write(base + 0, on & 0xFF)
        self.write(base + 1, (on >> 8) & 0x0F)
        self.write(base + 2, off & 0xFF)
        self.write(base + 3, (off >> 8) & 0x0F)

    def set_motor_pwm(self, channel: int, duty: int) -> None:
        """Set duty 0..4095 on a channel."""
        if duty < 0: duty = 0
        if duty > 4095: duty = 4095
        self.set_pwm(channel, 0, duty)

    def close(self) -> None:
        self.bus.close()

# ================== Robot motor wrapper ==================
class RobotSavo:
    """
    Channel map (locked to your earlier code):
      FL (front-left)  : (0,1)
      RL (rear-left)   : (3,2)
      FR (front-right) : (6,7)
      RR (rear-right)  : (4,5)
    """
    def __init__(self, *, i2c_bus: int, addr: int, pwm_freq: float,
                 inv=(+1,+1,+1,+1), quench_ms: int = 18):
        self.pwm = PCA9685(bus=i2c_bus, address=addr, debug=True)
        self.pwm.set_pwm_freq(pwm_freq)
        self.FL_INV, self.RL_INV, self.FR_INV, self.RR_INV = inv
        self.quench_ms = max(0, int(quench_ms))
        self._last_sign = {'fl':0,'rl':0,'fr':0,'rr':0}

    # low-level wheel writers (positive = IN_hi on 2nd channel of each pair)
    def _wheel_fl(self, d):
        if   d > 0:  self.pwm.set_motor_pwm(0,0);   self.pwm.set_motor_pwm(1,d)
        elif d < 0:  self.pwm.set_motor_pwm(1,0);   self.pwm.set_motor_pwm(0,-d)
        else:        self.pwm.set_motor_pwm(0,4095);self.pwm.set_motor_pwm(1,4095)

    def _wheel_rl(self, d):
        if   d > 0:  self.pwm.set_motor_pwm(3,0);   self.pwm.set_motor_pwm(2,d)
        elif d < 0:  self.pwm.set_motor_pwm(2,0);   self.pwm.set_motor_pwm(3,-d)
        else:        self.pwm.set_motor_pwm(2,4095);self.pwm.set_motor_pwm(3,4095)

    def _wheel_fr(self, d):
        if   d > 0:  self.pwm.set_motor_pwm(6,0);   self.pwm.set_motor_pwm(7,d)
        elif d < 0:  self.pwm.set_motor_pwm(7,0);   self.pwm.set_motor_pwm(6,-d)
        else:        self.pwm.set_motor_pwm(6,4095);self.pwm.set_motor_pwm(7,4095)

    def _wheel_rr(self, d):
        if   d > 0:  self.pwm.set_motor_pwm(4,0);   self.pwm.set_motor_pwm(5,d)
        elif d < 0:  self.pwm.set_motor_pwm(5,0);   self.pwm.set_motor_pwm(4,-d)
        else:        self.pwm.set_motor_pwm(4,4095);self.pwm.set_motor_pwm(5,4095)

    @staticmethod
    def _clamp4(d1,d2,d3,d4):
        def c(v): return 4095 if v>4095 else (-4095 if v<-4095 else int(v))
        return c(d1), c(d2), c(d3), c(d4)

    def _apply_quench(self, name: str, prev_sign: int, new_val: int, fn) -> int:
        new_sign = 0 if new_val == 0 else (1 if new_val > 0 else -1)
        if self.quench_ms and prev_sign and new_sign and (prev_sign != new_sign):
            fn(0); time.sleep(self.quench_ms/1000.0)
        fn(new_val)
        self._last_sign[name] = new_sign
        return new_sign

    def set_motor_model(self, d_fl, d_rl, d_fr, d_rr):
        d_fl *= self.FL_INV; d_rl *= self.RL_INV
        d_fr *= self.FR_INV; d_rr *= self.RR_INV
        d_fl, d_rl, d_fr, d_rr = self._clamp4(d_fl, d_rl, d_fr, d_rr)
        self._apply_quench('fl', self._last_sign['fl'], d_fl, self._wheel_fl)
        self._apply_quench('rl', self._last_sign['rl'], d_rl, self._wheel_rl)
        self._apply_quench('fr', self._last_sign['fr'], d_fr, self._wheel_fr)
        self._apply_quench('rr', self._last_sign['rr'], d_rr, self._wheel_rr)

    def stop(self): self.set_motor_model(0,0,0,0)
    def close(self): 
        try: self.stop()
        finally: self.pwm.close()

# ================== Terminal helpers (letters only) ==================
class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)  # cbreak (we only read single-byte letters/space/ESC)
        return self
    def __exit__(self, *args):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

def read_key():
    """
    Non-blocking single-char reader (letters/space/ESC only).
    Returns: 'w','a','s','d','q','e','x',' ','z','c','r','esc',''
    """
    r,_,_ = select.select([sys.stdin], [], [], 0)
    if not r: return ''
    ch = sys.stdin.read(1)
    if ch == '\x1b':   # ESC
        return 'esc'
    return ch.lower()

# ================== Kinematics ==================
def mix_mecanum(vx, vy, wz, *, forward_sign, strafe_sign, rotate_sign, turn_gain):
    """
    Return normalized wheel commands (fl, rl, fr, rr) in [-1..1].
      fl =  vx - vy - w
      rl =  vx + vy - w
      fr =  vx + vy + w
      rr =  vx - vy + w
    """
    vx *= forward_sign
    vy *= strafe_sign
    w  = rotate_sign * turn_gain * wz
    fl =  vx - vy - w
    rl =  vx + vy - w
    fr =  vx + vy + w
    rr =  vx - vy + w
    m = max(1.0, abs(fl), abs(rl), abs(fr), abs(rr))
    return fl/m, rl/m, fr/m, rr/m

def to_duties(nfl, nrl, nfr, nrr, max_duty):
    return int(nfl*max_duty), int(nrl*max_duty), int(nfr*max_duty), int(nrr*max_duty)

# ================== Poke (diagnostic) ==================
def run_poke(bot: RobotSavo, duty: int = 1200, hold_s: float = 0.6, gap_s: float = 0.4):
    seq = [
        ("FL +", ( duty,    0,    0,    0)), ("FL -", (-duty,    0,    0,    0)),
        ("RL +", (   0,  duty,    0,    0)), ("RL -", (   0, -duty,    0,    0)),
        ("FR +", (   0,    0,  duty,    0)), ("FR -", (   0,    0, -duty,    0)),
        ("RR +", (   0,    0,    0,  duty)), ("RR -", (   0,    0,    0, -duty)),
    ]
    try:
        for name, d in seq:
            print(f"[poke] {name}")
            bot.set_motor_model(*d); time.sleep(hold_s)
            bot.stop(); time.sleep(gap_s)
    finally:
        bot.stop()

# ================== Main (teleop) ==================
def main():
    ap = argparse.ArgumentParser(description="Robot Savo — Expert PCA9685 Mecanum Teleop (letters only)")
    # hardware
    ap.add_argument("--i2c-bus", type=int, default=1, help="I²C bus number (default: 1)")
    ap.add_argument("--addr", type=lambda x:int(x,0), default=0x40, help="PCA9685 I²C addr (default: 0x40)")
    ap.add_argument("--pwm-freq", type=float, default=50.0, help="PWM frequency Hz (default: 50.0)")

    # control loop & scale
    ap.add_argument("--hz", type=float, default=30.0, help="Control loop Hz (default: 30)")
    ap.add_argument("--max-duty", type=int, default=3000, help="Max duty (0..4095) (default: 3000)")
    ap.add_argument("--step", type=float, default=0.15, help="Increment per keypress (default: 0.15)")
    ap.add_argument("--decay", type=float, default=0.85, help="Decay factor (0..1, higher = coast) (default: 0.85)")

    # kinematics signs & turn gain
    ap.add_argument("--forward-sign", type=int, choices=[-1,1], default=-1, help="Flip forward axis (+1/-1) (default: -1)")
    ap.add_argument("--strafe-sign", type=int, choices=[-1,1], default=+1, help="Flip strafe axis (+1/-1) (default: +1)")
    ap.add_argument("--rotate-sign", type=int, choices=[-1,1], default=+1, help="Flip rotate axis (+1/-1) (default: +1)")
    ap.add_argument("--turn-gain", type=float, default=1.0, help="Rotation gain (default: 1.0)")

    # per-wheel invert & quench
    ap.add_argument("--invert-fl", action="store_true", help="Invert FL wheel")
    ap.add_argument("--invert-rl", action="store_true", help="Invert RL wheel")
    ap.add_argument("--invert-fr", action="store_true", help="Invert FR wheel")
    ap.add_argument("--invert-rr", action="store_true", help="Invert RR wheel")
    ap.add_argument("--quench-ms", type=int, default=18, help="Neutral ms on sign flip (0 to disable) (default: 18)")

    # modes
    ap.add_argument("--poke", action="store_true", help="Run wheel poke test and exit")
    ap.add_argument("--timeout", type=float, default=0.0, help="Auto-exit after N seconds (0=off)")

    args = ap.parse_args()

    if not sys.stdin.isatty() and not args.poke:
        print("⚠ Requires a TTY for keyboard input. Run in a terminal or use --poke.")
        return

    inv = (
        -1 if args.invert_fl else +1,
        -1 if args.invert_rl else +1,
        -1 if args.invert_fr else +1,
        -1 if args.invert_rr else +1,
    )

    bot = RobotSavo(i2c_bus=args.i2c_bus, addr=args.addr, pwm_freq=args.pwm_freq,
                    inv=inv, quench_ms=args.quench_ms)

    if args.poke:
        print(f"[info] POKE mode  duty=1200  quench={args.quench_ms}ms  i2c-bus={args.i2c_bus} addr=0x{args.addr:02X}")
        try:
            run_poke(bot)
        finally:
            bot.close()
        return

    # Teleop
    vx = vy = wz = 0.0
    max_duty  = max(0, min(4095, args.max_duty))
    step      = args.step
    decay     = max(0.0, min(1.0, args.decay))
    hz        = max(5.0, float(args.hz))
    turn_gain = args.turn_gain

    print("\nRobot Savo teleop ready (letters only).")
    print("W/S/A/D; Q/E rotate; X/Space stop; Z (down) / C (up) scale; R reset; ESC quit.")
    print(f"scale={max_duty}/4095  quench={args.quench_ms}ms  i2c-bus={args.i2c_bus} addr=0x{args.addr:02X}  pwm-freq={args.pwm_freq}Hz\n")

    try:
        with RawTerminal():
            last = time.time()
            t0 = last
            while True:
                # optional timeout
                if args.timeout and (time.time() - t0) >= args.timeout:
                    print("[info] timeout reached, exiting.")
                    break

                key = read_key()
                if key:
                    if key == 'esc': break
                    elif key == 'w':           vx = min(1.0,  vx + step)
                    elif key == 's':           vx = max(-1.0, vx - step)
                    elif key == 'a':           vy = max(-1.0, vy - step)
                    elif key == 'd':           vy = min(1.0,  vy + step)
                    elif key == 'q':           wz = min(1.0,  wz + step)   # CCW
                    elif key == 'e':           wz = max(-1.0, wz - step)   # CW
                    elif key in ('x',' '):     vx = vy = wz = 0.0
                    elif key == 'z':           # scale DOWN
                        max_duty = max(600, int(max_duty*0.85))
                        print(f"[scale↓] max_duty={max_duty}")
                    elif key == 'c':           # scale UP
                        max_duty = min(4095, int(max_duty*1.15))
                        print(f"[scale↑] max_duty={max_duty}")
                    elif key == 'r':
                        vx = vy = wz = 0.0
                        max_duty  = max(0, min(4095, args.max_duty))
                        turn_gain = args.turn_gain
                        print("[reset] zeroed; scale/turn_gain reset.")

                # smooth decay toward zero
                now = time.time(); dt = now - last; last = now
                d = decay ** (dt * hz)
                vx *= d; vy *= d; wz *= d

                # mix and send
                fl, rl, fr, rr = mix_mecanum(vx, vy, wz,
                    forward_sign=args.forward_sign,
                    strafe_sign=args.strafe_sign,
                    rotate_sign=args.rotate_sign,
                    turn_gain=turn_gain)
                dfl, drl, dfr, drr = to_duties(fl, rl, fr, rr, max_duty)
                bot.set_motor_model(dfl, drl, dfr, drr)

                # ~hz loop
                sleep_left = (1.0/hz) - (time.time()-now)
                if sleep_left > 0: time.sleep(sleep_left)

    except KeyboardInterrupt:
        pass
    finally:
        bot.close()
        print("\nStopped. Bye.")

if __name__ == "__main__":
    main()
