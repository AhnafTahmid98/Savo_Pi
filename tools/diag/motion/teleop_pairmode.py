#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Freenove FN-K0043 4WD Systematic Discovery
Tests ALL PWM (0-15) and IN1/IN2 (0-15) combinations to find working mappings
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
    def drive(self, signed, duty_max):
        d = max(-1.0, min(1.0, float(signed)))
        if self.t.invert: d = -d
        a,b = (self.t.in1, self.t.in2) if not self.t.swap_in12 else (self.t.in2, self.t.in1)
        if abs(d) < 1e-6:
            self._digital(a,0); self._digital(b,0); self.p.set_duty(self.t.pwm,0.0)
            return
        if d>0:
            self._digital(a,1); self._digital(b,0); self.p.set_duty(self.t.pwm,abs(d)*duty_max)
        else:
            self._digital(a,0); self._digital(b,1); self.p.set_duty(self.t.pwm,abs(d)*duty_max)
    def stop(self):
        a,b = (self.t.in1, self.t.in2) if not self.t.swap_in12 else (self.t.in2, self.t.in1)
        self._digital(a,0); self._digital(b,0); self.p.set_duty(self.t.pwm,0.0)

class Motor4:
    def __init__(self, p: PCA9685, FL:Triplet, BL:Triplet, FR:Triplet, BR:Triplet, max_duty=1.0, verbose=False):
        self.FL, self.BL, self.FR, self.BR = Wheel(p,FL,"FL",verbose), Wheel(p,BL,"BL",verbose), Wheel(p,FR,"FR",verbose), Wheel(p,BR,"BR",verbose)
        self.max = float(max(0.05, min(1.0, max_duty)))
    def set_motor_model(self, FL, BL, FR, BR):
        self.FL.drive(FL, self.max); self.BL.drive(BL, self.max); self.FR.drive(FR, self.max); self.BR.drive(BR, self.max)
    def stop(self):
        for w in (self.FL,self.BL,self.FR,self.BR): w.stop()

# ---- Patterns ----
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
        self.mot = None
        self.current_mapping = None
        
        print("\n" + "="*80)
        print("FREENOVE FN-K0043 4WD - COMPLETE CHANNEL DISCOVERY")
        print("="*80)
        print("This will test ALL PWM and IN1/IN2 combinations to find working motor mappings")

    def create_motor_mapping(self, fl_pwm, fl_in1, fl_in2, bl_pwm, bl_in1, bl_in2, fr_pwm, fr_in1, fr_in2, br_pwm, br_in1, br_in2):
        """Create motor mapping with given channels"""
        ial = self.a.in_active_low
        def T(pwm,in1,in2,inv,sw): return Triplet(pwm,in1,in2,inv,ial,sw)
        
        self.mot = Motor4(
            self.pca,
            FL=T(fl_pwm, fl_in1, fl_in2, self.a.inv_fl, self.a.swap_in12_fl),
            BL=T(bl_pwm, bl_in1, bl_in2, self.a.inv_bl, self.a.swap_in12_bl),
            FR=T(fr_pwm, fr_in1, fr_in2, self.a.inv_fr, self.a.swap_in12_fr),
            BR=T(br_pwm, br_in1, br_in2, self.a.inv_br, self.a.swap_in12_br),
            max_duty=self.a.max,
            verbose=self.a.verbose
        )
        
        self.current_mapping = {
            'FL': (fl_pwm, fl_in1, fl_in2),
            'BL': (bl_pwm, bl_in1, bl_in2),
            'FR': (fr_pwm, fr_in1, fr_in2),
            'BR': (br_pwm, br_in1, br_in2)
        }

    def setv(self, vec, mag): 
        if self.mot:
            self.mot.set_motor_model(vec[0]*mag, vec[1]*mag, vec[2]*mag, vec[3]*mag)

    def stop(self): 
        if self.mot:
            self.mot.stop()

    def test_single_wheel(self, wheel_name, pwm_ch, in1_ch, in2_ch):
        """Test a single wheel configuration"""
        print(f"Testing {wheel_name}: PWM={pwm_ch}, IN1={in1_ch}, IN2={in2_ch}")
        
        # Create temporary mapping for this test
        test_mapping = {
            'FL': (0, 0, 0), 'BL': (0, 0, 0), 'FR': (0, 0, 0), 'BR': (0, 0, 0)
        }
        test_mapping[wheel_name] = (pwm_ch, in1_ch, in2_ch)
        
        self.create_motor_mapping(
            test_mapping['FL'][0], test_mapping['FL'][1], test_mapping['FL'][2],
            test_mapping['BL'][0], test_mapping['BL'][1], test_mapping['BL'][2],
            test_mapping['FR'][0], test_mapping['FR'][1], test_mapping['FR'][2],
            test_mapping['BR'][0], test_mapping['BR'][1], test_mapping['BR'][2]
        )
        
        # Test forward
        print("  Testing FORWARD...", end="")
        if wheel_name == 'FL': self.setv((0.3, 0, 0, 0), 1.0)
        elif wheel_name == 'BL': self.setv((0, 0.3, 0, 0), 1.0)
        elif wheel_name == 'FR': self.setv((0, 0, 0.3, 0), 1.0)
        elif wheel_name == 'BR': self.setv((0, 0, 0, 0.3), 1.0)
        
        time.sleep(1.0)
        self.stop()
        response = input(" Did wheel spin? [y/n]: ").lower()
        works = response == 'y'
        
        if works:
            # Test direction
            print("  Testing direction...", end="")
            if wheel_name == 'FL': self.setv((0.3, 0, 0, 0), 1.0)
            elif wheel_name == 'BL': self.setv((0, 0.3, 0, 0), 1.0)
            elif wheel_name == 'FR': self.setv((0, 0, 0.3, 0), 1.0)
            elif wheel_name == 'BR': self.setv((0, 0, 0, 0.3), 1.0)
            
            time.sleep(1.0)
            self.stop()
            direction = input(" Direction correct for FORWARD? [y/n]: ").lower()
            return works, direction == 'y'
        
        return works, False

    def discover_all_channels(self):
        """Discover ALL working PWM and IN1/IN2 combinations"""
        print("\n" + "="*80)
        print("DISCOVERING ALL WORKING CHANNEL COMBINATIONS")
        print("="*80)
        
        # Test range: PWM 0-15, IN1/IN2 0-15
        pwm_channels = list(range(0, 16))
        in_channels = list(range(0, 16))
        
        working_combinations = {
            'FL': [], 'BL': [], 'FR': [], 'BR': []
        }
        
        for wheel in ['FL', 'BL', 'FR', 'BR']:
            print(f"\n>>> Discovering {wheel} motor channels...")
            input("Make sure only ONE motor is connected. Press Enter when ready...")
            
            for pwm_ch in pwm_channels:
                for in1_ch in in_channels:
                    for in2_ch in in_channels:
                        if in1_ch == in2_ch:
                            continue  # Skip same IN channels
                        
                        print(f"\nTesting {wheel}: PWM={pwm_ch}, IN1={in1_ch}, IN2={in2_ch}")
                        works, correct_direction = self.test_single_wheel(wheel, pwm_ch, in1_ch, in2_ch)
                        
                        if works:
                            status = "CORRECT_DIR" if correct_direction else "REVERSE_DIR"
                            working_combinations[wheel].append((pwm_ch, in1_ch, in2_ch, status))
                            print(f"✓ FOUND: {wheel} -> PWM:{pwm_ch}, IN1:{in1_ch}, IN2:{in2_ch} [{status}]")
                        
                        time.sleep(0.1)
            
            print(f"\n*** {wheel} discovery complete. Found {len(working_combinations[wheel])} working combinations")
        
        return working_combinations

    def test_standard_freenove_mapping(self):
        """Test the standard Freenove mapping"""
        print("\n" + "="*80)
        print("TESTING STANDARD FREENOVE MAPPING")
        print("="*80)
        
        # Standard Freenove FN-K0043 mapping (common configuration)
        standard_mapping = {
            'FL': (4, 5, 6),   # PWM4, IN1=5, IN2=6
            'BL': (7, 8, 9),   # PWM7, IN1=8, IN2=9  
            'FR': (0, 1, 2),   # PWM0, IN1=1, IN2=2
            'BR': (3, 10, 11)  # PWM3, IN1=10, IN2=11
        }
        
        print("Testing standard Freenove mapping:")
        for wheel, (pwm, in1, in2) in standard_mapping.items():
            print(f"  {wheel}: PWM={pwm}, IN1={in1}, IN2={in2}")
        
        input("\nPress Enter to test this mapping...")
        
        self.create_motor_mapping(
            standard_mapping['FL'][0], standard_mapping['FL'][1], standard_mapping['FL'][2],
            standard_mapping['BL'][0], standard_mapping['BL'][1], standard_mapping['BL'][2],
            standard_mapping['FR'][0], standard_mapping['FR'][1], standard_mapping['FR'][2],
            standard_mapping['BR'][0], standard_mapping['BR'][1], standard_mapping['BR'][2]
        )
        
        self.test_movement_patterns("Standard Freenove Mapping")

    def test_movement_patterns(self, mapping_name):
        """Test all movement patterns with current mapping"""
        print(f"\nTesting movement patterns for: {mapping_name}")
        
        patterns = [
            ("FORWARD", P.F, "Robot should move FORWARD"),
            ("BACKWARD", P.B, "Robot should move BACKWARD"),
            ("STRAFE LEFT", P.SL, "Robot should move LEFT"),
            ("STRAFE RIGHT", P.SR, "Robot should move RIGHT"), 
            ("TURN LEFT", P.TL, "Robot should rotate COUNTER-CLOCKWISE"),
            ("TURN RIGHT", P.TR, "Robot should rotate CLOCKWISE")
        ]
        
        results = []
        
        for name, pattern, expected in patterns:
            print(f"\n>>> Testing {name}")
            print(f"   Expected: {expected}")
            input("   Press Enter to test...")
            
            # Ramp up slowly
            for speed in [0.1, 0.2, 0.3]:
                self.setv(pattern, speed)
                time.sleep(0.3)
            
            time.sleep(1.0)
            self.stop()
            
            response = input("   What happened? [f]orward/[b]ackward/[l]eft/[r]ight/[t]urn/[s]pin/[n]othing: ").lower()
            results.append((name, response, expected))
            
            time.sleep(0.5)
        
        return results

    def run_complete_discovery(self):
        """Run the complete discovery process"""
        print("FREENOVE FN-K0043 COMPLETE DISCOVERY")
        print("This will help us find the exact motor mappings for your kit")
        
        # Option 1: Test standard mapping first
        print("\n1. First, let's test the standard Freenove mapping...")
        self.test_standard_freenove_mapping()
        
        # Option 2: Full discovery if standard doesn't work
        response = input("\nDid the standard mapping work correctly? [y/n]: ").lower()
        if response != 'y':
            print("\n2. Standard mapping didn't work. Running complete channel discovery...")
            working_combinations = self.discover_all_channels()
            
            print("\n" + "="*80)
            print("DISCOVERY RESULTS")
            print("="*80)
            for wheel, combos in working_combinations.items():
                print(f"\n{wheel} working combinations:")
                for pwm, in1, in2, status in combos:
                    print(f"  PWM:{pwm}, IN1:{in1}, IN2:{in2} [{status}]")
        
        # Option 3: Manual testing
        print("\n3. Manual mapping test")
        self.manual_mapping_test()

    def manual_mapping_test(self):
        """Allow manual testing of specific mappings"""
        print("\n" + "="*80)
        print("MANUAL MAPPING TEST")
        print("="*80)
        
        while True:
            print("\nEnter motor mappings to test:")
            try:
                fl_pwm = int(input("FL PWM channel: "))
                fl_in1 = int(input("FL IN1 channel: "))
                fl_in2 = int(input("FL IN2 channel: "))
                
                bl_pwm = int(input("BL PWM channel: "))
                bl_in1 = int(input("BL IN1 channel: "))
                bl_in2 = int(input("BL IN2 channel: "))
                
                fr_pwm = int(input("FR PWM channel: "))
                fr_in1 = int(input("FR IN1 channel: "))
                fr_in2 = int(input("FR IN2 channel: "))
                
                br_pwm = int(input("BR PWM channel: "))
                br_in1 = int(input("BR IN1 channel: "))
                br_in2 = int(input("BR IN2 channel: "))
                
                print(f"\nTesting mapping:")
                print(f"FL: PWM={fl_pwm}, IN1={fl_in1}, IN2={fl_in2}")
                print(f"BL: PWM={bl_pwm}, IN1={bl_in1}, IN2={bl_in2}") 
                print(f"FR: PWM={fr_pwm}, IN1={fr_in1}, IN2={fr_in2}")
                print(f"BR: PWM={br_pwm}, IN1={br_in1}, IN2={br_in2}")
                
                input("Press Enter to test...")
                
                self.create_motor_mapping(fl_pwm, fl_in1, fl_in2, bl_pwm, bl_in1, bl_in2, fr_pwm, fr_in1, fr_in2, br_pwm, br_in1, br_in2)
                results = self.test_movement_patterns("Manual Test")
                
                print("\nTest results:")
                for name, actual, expected in results:
                    print(f"  {name}: got '{actual}', expected '{expected}'")
                
                cont = input("\nTest another mapping? [y/n]: ").lower()
                if cont != 'y':
                    break
                    
            except (ValueError, KeyboardInterrupt):
                print("\nInvalid input or cancelled")
                break

    def run_quick_test(self):
        """Quick test with current mapping"""
        if self.a.fl_pwm is not None:
            self.create_motor_mapping(
                self.a.fl_pwm, self.a.fl_in1, self.a.fl_in2,
                self.a.bl_pwm, self.a.bl_in1, self.a.bl_in2,
                self.a.fr_pwm, self.a.fr_in1, self.a.fr_in2,
                self.a.br_pwm, self.a.br_in1, self.a.br_in2
            )
            self.test_movement_patterns("Quick Test")

def build_ap():
    p=argparse.ArgumentParser(description="Freenove FN-K0043 4WD Complete Channel Discovery")
    p.add_argument('--mode', choices=['discover', 'standard-test', 'manual', 'quick'], default='discover')
    p.add_argument('--mag', type=float, default=0.3)
    p.add_argument('--max', type=float, default=0.6)
    p.add_argument('--i2c-bus', type=int, default=1)
    p.add_argument('--pca-addr', type=lambda x:int(x,0), default=0x40)
    p.add_argument('--pwm-freq', type=float, default=200.0)
    p.add_argument('--set-freq', action='store_true', default=True)
    p.add_argument('--in-active-low', action='store_true')
    p.add_argument('--verbose', action='store_true')
    # wheel channels (optional for discovery mode)
    p.add_argument('--fl-pwm', type=int, default=None); p.add_argument('--fl-in1', type=int, default=None); p.add_argument('--fl-in2', type=int, default=None)
    p.add_argument('--bl-pwm', type=int, default=None); p.add_argument('--bl-in1', type=int, default=None); p.add_argument('--bl-in2', type=int, default=None)
    p.add_argument('--fr-pwm', type=int, default=None); p.add_argument('--fr-in1', type=int, default=None); p.add_argument('--fr-in2', type=int, default=None)
    p.add_argument('--br-pwm', type=int, default=None); p.add_argument('--br-in1', type=int, default=None); p.add_argument('--br-in2', type=int, default=None)
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
        if a.mode == 'discover':
            app.run_complete_discovery()
        elif a.mode == 'standard-test':
            app.test_standard_freenove_mapping()
        elif a.mode == 'manual':
            app.manual_mapping_test()
        elif a.mode == 'quick' and all([a.fl_pwm, a.fl_in1, a.fl_in2, a.bl_pwm, a.bl_in1, a.bl_in2, a.fr_pwm, a.fr_in1, a.fr_in2, a.br_pwm, a.br_in1, a.br_in2]):
            app.run_quick_test()
        else:
            print("Please specify a valid mode and all channel parameters for quick test")
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        app.stop()

if __name__=='__main__': 
    main()