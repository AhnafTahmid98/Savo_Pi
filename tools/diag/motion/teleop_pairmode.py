#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Systematic Wheel Debugging
Let's figure out the correct wheel mappings step by step
"""
import argparse, os, select, sys, termios, time, tty, signal
from dataclasses import dataclass

# ---- I2C ----
try:
    from smbus2 import SMBus
except Exception:
    print("Install smbus2: sudo apt install -y python3-smbus  OR  pip3 install smbus2", file=sys.stderr)
    raise

# ---- PCA9685 minimal ----
MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, OUTDRV, ALLCALL = 0x80, 0x10, 0x20, 0x04, 0x01

class PCA9685:
    def __init__(self, bus=1, addr=0x40, freq_hz=200.0, set_freq=True, verbose=False):
        self.addr = int(addr); self.bus = SMBus(int(bus)); self.verbose=verbose
        self._w8(MODE2, OUTDRV); self._w8(MODE1, AI); time.sleep(0.003)
        if set_freq: self.set_pwm_freq(freq_hz)
        m1 = self._r8(MODE1); self._w8(MODE1, (m1 | RESTART | AI) & ~ALLCALL)
    def close(self):
        try: self.bus.close()
        except: pass
    def _w8(self, reg, val): self.bus.write_byte_data(self.addr, reg & 0xFF, val & 0xFF)
    def _r8(self, reg):      return self.bus.read_byte_data(self.addr, reg & 0xFF)
    def set_pwm_freq(self, f):
        f = float(max(40.0, min(1500.0, f)))
        prescale = int(max(3, min(255, round(25_000_000.0/(4096.0*f)-1.0))))
        old = self._r8(MODE1); self._w8(MODE1, (old & ~RESTART)|SLEEP)
        self._w8(PRESCALE, prescale); self._w8(MODE1, old & ~SLEEP)
        time.sleep(0.003); self._w8(MODE1, (old | RESTART | AI) & ~ALLCALL)
        if self.verbose: print(f"[PCA] freq={f:.1f}Hz prescale={prescale}")
    def _raw(self, ch, on, off):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0, on & 0xFF); self._w8(base+1, (on>>8)&0x0F)
        self._w8(base+2, off&0xFF);  self._w8(base+3, (off>>8)&0x0F)
    def full_off(self, ch):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0,0); self._w8(base+1,0); self._w8(base+2,0); self._w8(base+3,0x10)
    def full_on(self, ch):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0,0); self._w8(base+1,0x10); self._w8(base+2,0); self._w8(base+3,0)
    def set_duty(self, ch, duty):
        duty = max(0.0, min(1.0, float(duty)))
        off = int(round(duty*4095))
        if off<=0: self.full_off(ch)
        elif off>=4095: self.full_on(ch)
        else: self._raw(ch,0,off)

# ---- H-bridge + wheels ----
@dataclass
class Triplet:
    pwm:int; in1:int; in2:int
    invert:bool=False; in_active_low:bool=False; swap_in12:bool=False

class Wheel:
    def __init__(self, p: PCA9685, t: Triplet, name:str, verbose=False):
        self.p, self.t, self.name, self.verbose = p, t, name, verbose
        self.stop()
    def _digital(self, ch, lvl):
        if self.t.in_active_low: lvl ^= 1
        (self.p.full_on if lvl else self.p.full_off)(ch)
        if self.verbose: print(f"[{self.name}] CH{ch}: {'HIGH' if lvl else 'LOW'} (active_low={self.t.in_active_low})")
    def drive(self, signed, duty_max):
        d = max(-1.0, min(1.0, float(signed)))
        if self.t.invert: d = -d
        a,b = (self.t.in1, self.t.in2) if not self.t.swap_in12 else (self.t.in2, self.t.in1)
        if abs(d) < 1e-6:
            self._digital(a,0); self._digital(b,0); self.p.set_duty(self.t.pwm,0.0)
            if self.verbose: print(f"[{self.name}] STOP")
            return
        if d>0:
            self._digital(a,1); self._digital(b,0); self.p.set_duty(self.t.pwm,abs(d)*duty_max)
            if self.verbose: print(f"[{self.name}] FWD duty={abs(d)*duty_max:.2f}")
        else:
            self._digital(a,0); self._digital(b,1); self.p.set_duty(self.t.pwm,abs(d)*duty_max)
            if self.verbose: print(f"[{self.name}] REV duty={abs(d)*duty_max:.2f}")
    def stop(self):
        a,b = (self.t.in1, self.t.in2) if not self.t.swap_in12 else (self.t.in2, self.t.in1)
        self._digital(a,0); self._digital(b,0); self.p.set_duty(self.t.pwm,0.0)

class Motor4:
    def __init__(self, p: PCA9685, FL:Triplet, BL:Triplet, FR:Triplet, BR:Triplet, max_duty=1.0, verbose=False):
        self.FL, self.BL, self.FR, self.BR = Wheel(p,FL,"FL",verbose), Wheel(p,BL,"BL",verbose), Wheel(p,FR,"FR",verbose), Wheel(p,BR,"BR",verbose)
        self.max = float(max(0.05, min(1.0, max_duty)))
    def _norm(self, v):
        v = float(v)
        return max(-1.0, min(1.0, v/4095.0)) if abs(v)>1.01 else max(-1.0, min(1.0, v))
    def set_motor_model(self, FL, BL, FR, BR):
        fL,bL,fR,bR = map(self._norm, (FL,BL,FR,BR))
        self.FL.drive(fL, self.max); self.BL.drive(bL, self.max); self.FR.drive(fR, self.max); self.BR.drive(bR, self.max)
    def stop(self):
        for w in (self.FL,self.BL,self.FR,self.BR): w.stop()

# ---- keyboard ----
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
            else: out.append(ch)
            ch=self._read1(0.0)
        return out
    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

# ---- Patterns - WE WILL DISCOVER THESE ----
@dataclass
class P:
    F=(+1,+1,+1,+1)   # Forward
    B=(-1,-1,-1,-1)   # Backward
    SL=(-1,+1,+1,-1)  # Strafe Left
    SR=(+1,-1,-1,+1)  # Strafe Right
    TL=(-1,-1,+1,+1)  # Turn CCW
    TR=(+1,+1,-1,-1)  # Turn CW
    STOP=(0,0,0,0)

# ---- main controller ----
class App:
    def __init__(self, a):
        self.a=a
        self.pca = PCA9685(a.i2c_bus, a.pca_addr, a.pwm_freq, set_freq=a.set_freq, verbose=a.verbose)
        
        if a.reserve:
            r=[int(x) for x in a.reserve.split(',') if x.strip()!='']; [self.pca.full_off(ch) for ch in r]
            
        ial=a.in_active_low
        def T(pwm,in1,in2,inv,sw): return Triplet(pwm,in1,in2,inv,ial,sw)
        
        self.mot = Motor4(
            self.pca,
            FL=T(a.fl_pwm,a.fl_in1,a.fl_in2, a.inv_fl, a.swap_in12_fl),
            BL=T(a.bl_pwm,a.bl_in1,a.bl_in2, a.inv_bl, a.swap_in12_bl),
            FR=T(a.fr_pwm,a.fr_in1,a.fr_in2, a.inv_fr, a.swap_in12_fr),
            BR=T(a.br_pwm,a.br_in1,a.br_in2, a.inv_br, a.swap_in12_br),
            max_duty=max(0.05, min(1.0, a.max)),
            verbose=a.verbose
        )
        
        print("\n" + "="*70)
        print("ROBOT SAVO - SYSTEMATIC DEBUGGING")
        print("="*70)
        print("We will discover the correct wheel mappings step by step")
        print(f"\n[CONFIG] Speed: {a.mag}  Max: {a.max}")

    def setv(self, vec, mag): 
        self.mot.set_motor_model(vec[0]*mag, vec[1]*mag, vec[2]*mag, vec[3]*mag)
    def stop(self): self.mot.stop()

    def test_wheel_direction(self):
        """Step 1: Test each wheel's forward direction"""
        print("\n" + "="*70)
        print("STEP 1: TEST INDIVIDUAL WHEEL DIRECTIONS")
        print("="*70)
        print("We need to verify which way each wheel spins for 'forward'")
        print("Look at each wheel from the TOP - clockwise or counter-clockwise?")
        
        wheels = [
            ("Front Left", "FL", (1, 0, 0, 0)),
            ("Back Left", "BL", (0, 1, 0, 0)),
            ("Front Right", "FR", (0, 0, 1, 0)),
            ("Back Right", "BR", (0, 0, 0, 1))
        ]
        
        corrections = {}
        
        for wheel_name, wheel_code, mask in wheels:
            print(f"\n>>> Testing {wheel_name} ({wheel_code})")
            print(f"   Wheel should spin FORWARD (move robot forward)")
            input("   Press Enter to test...")
            
            self.mot.set_motor_model(*(m*0.2 for m in mask))
            time.sleep(1.0)
            self.stop()
            
            response = input("   Which way did it spin? [f]orward (correct) / [r]everse (wrong) / [s]kip: ").lower()
            if response == 'r':
                corrections[wheel_code] = True
                print(f"   ✓ Will INVERT {wheel_code}")
            else:
                corrections[wheel_code] = False
                print(f"   ✓ {wheel_code} direction is OK")
            
            time.sleep(0.5)
        
        return corrections

    def test_forward_movement(self, corrections):
        """Step 2: Test forward movement with corrections"""
        print("\n" + "="*70)
        print("STEP 2: TEST FORWARD MOVEMENT")
        print("="*70)
        
        # Apply corrections
        fl_inv, bl_inv, fr_inv, br_inv = corrections.get('FL', False), corrections.get('BL', False), corrections.get('FR', False), corrections.get('BR', False)
        
        print(f"\nCorrections applied:")
        print(f"  FL invert: {fl_inv}, BL invert: {bl_inv}")
        print(f"  FR invert: {fr_inv}, BR invert: {br_inv}")
        
        # Test forward
        print("\n>>> Testing FORWARD movement")
        print("   Robot should move STRAIGHT FORWARD")
        input("   Press Enter to test...")
        
        self.setv(P.F, 0.3)
        time.sleep(2.0)
        self.stop()
        
        response = input("   Did robot move [f]orward / [b]ackward / [l]eft / [r]ight / [s]pin? : ").lower()
        return response

    def discover_patterns(self):
        """Step 3: Discover correct patterns through testing"""
        print("\n" + "="*70)
        print("STEP 3: DISCOVER MOVEMENT PATTERNS")
        print("="*70)
        print("We'll test different wheel combinations to find what works")
        
        patterns_to_test = [
            ("Forward", (+1, +1, +1, +1)),
            ("Backward", (-1, -1, -1, -1)),
            ("Strafe Left TEST 1", (-1, +1, +1, -1)),
            ("Strafe Left TEST 2", (+1, -1, -1, +1)),
            ("Strafe Right TEST 1", (+1, -1, -1, +1)),
            ("Strafe Right TEST 2", (-1, +1, +1, -1)),
            ("Turn Left TEST 1", (-1, -1, +1, +1)),
            ("Turn Left TEST 2", (+1, +1, -1, -1)),
            ("Turn Right TEST 1", (+1, +1, -1, -1)),
            ("Turn Right TEST 2", (-1, -1, +1, +1)),
        ]
        
        working_patterns = {}
        
        for name, pattern in patterns_to_test:
            print(f"\n>>> Testing: {name}")
            print(f"   Pattern: {pattern}")
            input("   Press Enter to test...")
            
            self.mot.set_motor_model(*[p*0.3 for p in pattern])
            time.sleep(2.0)
            self.stop()
            
            response = input("   What happened? [f]orward / [b]ackward / [l]eft / [r]ight / turn_[l] / turn_[r] / [n]one: ").lower()
            if response in ['f', 'b', 'l', 'r', 'turn_l', 'turn_r']:
                working_patterns[response] = (name, pattern)
                print(f"   ✓ Mapped '{response}' to {name}")
            else:
                print(f"   ✗ No clear movement")
            
            time.sleep(0.5)
        
        return working_patterns

    def run_systematic_debug(self):
        """Main debugging routine"""
        print("ROBOT SAVO - SYSTEMATIC DEBUGGING MODE")
        print("We will discover the correct wheel configuration step by step")
        
        # Step 1: Test individual wheels
        corrections = self.test_wheel_direction()
        
        # Step 2: Test forward movement
        forward_result = self.test_forward_movement(corrections)
        
        # Step 3: Discover patterns
        working_patterns = self.discover_patterns()
        
        # Generate final configuration
        self.generate_final_config(corrections, working_patterns)

    def generate_final_config(self, corrections, working_patterns):
        """Generate the final working configuration"""
        print("\n" + "="*70)
        print("FINAL CONFIGURATION")
        print("="*70)
        
        print("\n=== WHEEL INVERSION SETTINGS ===")
        print("Add these flags to your command:")
        inv_flags = []
        if corrections.get('FL'): inv_flags.append("--inv-fl")
        if corrections.get('BL'): inv_flags.append("--inv-bl") 
        if corrections.get('FR'): inv_flags.append("--inv-fr")
        if corrections.get('BR'): inv_flags.append("--inv-br")
        
        if inv_flags:
            print(" ".join(inv_flags))
        else:
            print("# No inversion needed")
        
        print("\n=== WORKING PATTERNS ===")
        for movement, (name, pattern) in working_patterns.items():
            print(f"{movement.upper():10} -> {name:20} {pattern}")
        
        print("\n=== FINAL COMMAND ===")
        base_cmd = f"python3 {sys.argv[0]} --mode teleop"
        base_cmd += f" --fl-pwm {self.a.fl_pwm} --fl-in1 {self.a.fl_in1} --fl-in2 {self.a.fl_in2}"
        base_cmd += f" --bl-pwm {self.a.bl_pwm} --bl-in1 {self.a.bl_in1} --bl-in2 {self.a.bl_in2}"
        base_cmd += f" --fr-pwm {self.a.fr_pwm} --fr-in1 {self.a.fr_in1} --fr-in2 {self.a.fr_in2}"
        base_cmd += f" --br-pwm {self.a.br_pwm} --br-in1 {self.a.br_in1} --br-in2 {self.a.br_in2}"
        
        if inv_flags:
            base_cmd += " " + " ".join(inv_flags)
            
        print(base_cmd)

    def run_teleop(self):
        """Simple teleop for testing"""
        kb=Keyboard()
        pattern=P.STOP
        scale=1.0
        
        print("\n[TELEOP] Testing mode - Use WASD, Space=stop, ESC=quit")
        
        try:
            while True:
                for ch in kb.read():
                    if ch in (b'\x1b', b'\x03'):
                        return
                    if ch in (b' ', b'0'):
                        pattern=P.STOP; print("STOP")
                    elif ch in (b'w',): pattern=P.F; print("FWD")
                    elif ch in (b's',): pattern=P.B; print("BWD") 
                    elif ch in (b'a',): pattern=P.SL; print("LEFT")
                    elif ch in (b'd',): pattern=P.SR; print("RIGHT")
                    elif ch in (b'q',): pattern=P.TL; print("TURN L")
                    elif ch in (b'e',): pattern=P.TR; print("TURN R")
                
                self.setv(pattern, self.a.mag*scale)
                time.sleep(0.05)
        finally:
            kb.restore()
            self.stop()

def build_ap():
    p=argparse.ArgumentParser(description="Robot Savo — Systematic Wheel Debugging")
    p.add_argument('--mode', choices=['debug','teleop','pulse'], default='debug')
    p.add_argument('--mag', type=float, default=0.3)
    p.add_argument('--max', type=float, default=0.6)
    p.add_argument('--i2c-bus', type=int, default=1)
    p.add_argument('--pca-addr', type=lambda x:int(x,0), default=0x40)
    p.add_argument('--pwm-freq', type=float, default=200.0)
    p.add_argument('--set-freq', action='store_true', default=True)
    p.add_argument('--reserve', type=str, default='12,13,14,15')
    p.add_argument('--in-active-low', action='store_true')
    p.add_argument('--verbose', action='store_true')
    # wheel channels
    p.add_argument('--fl-pwm', type=int, required=True); p.add_argument('--fl-in1', type=int, required=True); p.add_argument('--fl-in2', type=int, required=True)
    p.add_argument('--bl-pwm', type=int, required=True); p.add_argument('--bl-in1', type=int, required=True); p.add_argument('--bl-in2', type=int, required=True)
    p.add_argument('--fr-pwm', type=int, required=True); p.add_argument('--fr-in1', type=int, required=True); p.add_argument('--fr-in2', type=int, required=True)
    p.add_argument('--br-pwm', type=int, required=True); p.add_argument('--br-in1', type=int, required=True); p.add_argument('--br-in2', type=int, required=True)
    # polarity
    p.add_argument('--inv-fl', action='store_true'); p.add_argument('--inv-bl', action='store_true')
    p.add_argument('--inv-fr', action='store_true'); p.add_argument('--inv-br', action='store_true')
    p.add_argument('--swap-in12-fl', action='store_true'); p.add_argument('--swap-in12-bl', action='store_true')
    p.add_argument('--swap-in12-fr', action='store_true'); p.add_argument('--swap-in12-br', action='store_true')
    return p

def main(argv=None):
    a = build_ap().parse_args(argv)
    app = App(a)
    signal.signal(signal.SIGINT, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        if a.mode == 'debug':
            app.run_systematic_debug()
        elif a.mode == 'teleop':
            app.run_teleop()
        elif a.mode == 'pulse':
            # Simple pulse test
            tests = [("FL", (1,0,0,0)), ("BL", (0,1,0,0)), ("FR", (0,0,1,0)), ("BR", (0,0,0,1))]
            for name, mask in tests:
                print(f"Testing {name}...")
                app.mot.set_motor_model(*(m*0.2 for m in mask))
                time.sleep(1)
                app.stop()
                time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        app.stop()

if __name__=='__main__': 
    main()