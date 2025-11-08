#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ultrasonic_test.py
HC-SR04-class ultrasonic tester for Robot Savo (Pi 5, Ubuntu 24.04)

Notes:
- Sensor powered at 5V on Freenove HAT; level-shifted to 3.3V for Pi.
- Pass the correct mapping discovered by ultra_diag.py with --trig/--echo.

Requires:
  sudo apt install -y python3-lgpio  (or python3-libgpiod)
  sudo usermod -aG gpio $USER && newgrp gpio
"""
import argparse, csv, sys, time
from statistics import median, mean, pstdev
from typing import Optional, List, Tuple

DEF_RATE=10.0; DEF_BURST=5; DEF_SAMPLES=50; DEF_TEMP=20.0
DEF_THRESH=0.28; DEF_TIMEOUT=0.03; DEF_TRIG_PULSE=10e-6; DEF_SETTLE=0.05

def sos(t): return 331.3 + 0.606*t
def now_ns(): return time.perf_counter_ns()

class GpioBackend:
    def setup_output(self, pin:int): ...
    def setup_input(self, pin:int): ...
    def write(self, pin:int, lvl:int): ...
    def read(self, pin:int)->int: ...
    def close(self): ...

class LgpioBackend(GpioBackend):
    def __init__(self, chip_index:int=0):
        import lgpio; self.l=lgpio; self.h=self.l.gpiochip_open(chip_index)
        self.out=set(); self._in=set()
    def setup_output(self,p): self.l.gpio_claim_output(self.h,p,0); self.out.add(p)
    def setup_input(self,p): self.l.gpio_claim_input(self.h,p); self._in.add(p)
    def write(self,p,v): self.l.gpio_write(self.h,p,1 if v else 0)
    def read(self,p): return 1 if self.l.gpio_read(self.h,p) else 0
    def close(self):
        try:
            for p in list(self.out): self.l.gpio_write(self.h,p,0)
        except Exception: pass
        try: self.l.gpiochip_close(self.h)
        except Exception: pass

class GpiodBackend(GpioBackend):
    def __init__(self, chip:str="gpiochip0"):
        import gpiod; self.g=gpiod; self.c=gpiod.Chip(chip); self.o={}; self.i={}
    def setup_output(self,p):
        ln=self.c.get_line(p); ln.request(consumer="ultra_out", type=self.g.LINE_REQ_DIR_OUT, default_vals=[0]); self.o[p]=ln
    def setup_input(self,p):
        ln=self.c.get_line(p); ln.request(consumer="ultra_in", type=self.g.LINE_REQ_DIR_IN); self.i[p]=ln
    def write(self,p,v): self.o[p].set_value(1 if v else 0)
    def read(self,p): return 1 if self.i[p].get_value() else 0
    def close(self):
        for p,ln in list(self.o.items()):
            try: ln.set_value(0)
            except Exception: pass
            try: ln.release()
            except Exception: pass
        for p,ln in list(self.i.items()):
            try: ln.release()
            except Exception: pass
        try: self.c.close()
        except Exception: pass

def pick_backend(name:str|None):
    if name=="lgpio":
        import importlib; 
        if importlib.util.find_spec("lgpio"): return LgpioBackend()
        raise RuntimeError("lgpio not available")
    if name=="gpiod":
        import importlib; 
        if importlib.util.find_spec("gpiod"): return GpiodBackend()
        raise RuntimeError("gpiod not available")
    try: return LgpioBackend()
    except Exception: ...
    try: return GpiodBackend()
    except Exception: ...
    raise RuntimeError("No GPIO backend (install python3-lgpio or python3-libgpiod)")

def wait_level(gpio: GpioBackend, pin:int, lvl:int, to_s:float)->bool:
    t0=now_ns(); lim=int(to_s*1e9)
    while now_ns()-t0<lim:
        if gpio.read(pin)==lvl: return True
    return False

def pulse_width(gpio: GpioBackend, trig:int, echo:int, tp_s:float, to_s:float):
    gpio.write(trig,0); time.sleep(2e-6)
    gpio.write(trig,1); t0=now_ns()
    while now_ns()-t0<int(tp_s*1e9): pass
    gpio.write(trig,0)
    if not wait_level(gpio, echo, 1, to_s): return None
    tr=now_ns()
    if not wait_level(gpio, echo, 0, to_s): return None
    tf=now_ns()
    w=(tf-tr)/1e9
    return w if w>0 else None

def measure(gpio, trig, echo, temp_c, tp_s, to_s):
    w=pulse_width(gpio,trig,echo,tp_s,to_s)
    if w is None: return None
    return 0.5*sos(temp_c)*w, w

def burst(gpio,trig,echo,temp_c, n,tp_s,to_s, gap=0.01):
    ds,ws=[],[]
    for _ in range(n):
        r=measure(gpio,trig,echo,temp_c,tp_s,to_s)
        if r: d,w=r; ds.append(d); ws.append(w)
        time.sleep(gap)
    if not ds: return None
    from statistics import median
    return median(ds), median(ws), ds, ws

def grade(distances:list,th:float):
    if not distances: return "FAIL","No valid readings."
    from statistics import mean,pstdev
    dmin,dmax=min(distances),max(distances)
    dmean=mean(distances); dj=pstdev(distances) if len(distances)>1 else 0.0
    valid=0.02<=dmean<=4.0; near=sum(1 for d in distances if d<=th)
    if valid and dj<=0.01: g="PASS"
    elif valid and dj<=0.03: g="CAUTION"
    else: g="FAIL"
    return g, f"range=[{dmin:.3f},{dmax:.3f}] m, mean={dmean:.3f} m, jitter={dj:.3f} m, near({th:.2f})={near}/{len(distances)}"

def main():
    ap=argparse.ArgumentParser(description="Robot Savo ultrasonic tester")
    ap.add_argument("--trig", type=int, required=True, help="TRIG BCM (e.g., 27)")
    ap.add_argument("--echo", type=int, required=True, help="ECHO BCM (e.g., 22)")
    ap.add_argument("--rate", type=float, default=DEF_RATE)
    ap.add_argument("--burst", type=int, default=DEF_BURST)
    ap.add_argument("--samples", type=int, default=DEF_SAMPLES)
    ap.add_argument("--temp", type=float, default=DEF_TEMP)
    ap.add_argument("--thresh", type=float, default=DEF_THRESH)
    ap.add_argument("--timeout", type=float, default=DEF_TIMEOUT)
    ap.add_argument("--csv", type=str, default=None)
    ap.add_argument("--tag", type=str, default=None)
    ap.add_argument("--backend", choices=["auto","lgpio","gpiod"], default="auto")
    ap.add_argument("--quiet", action="store_true")
    args=ap.parse_args()

    try:
        gpio=pick_backend(None if args.backend=="auto" else args.backend)
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr); sys.exit(2)

    try:
        gpio.setup_output(args.trig); gpio.setup_input(args.echo)
    except Exception as e:
        print(f"ERROR: line setup failed: {e}", file=sys.stderr); gpio.close(); sys.exit(3)

    writer=None; fcsv=None
    if args.csv:
        fcsv=open(args.csv,"w",newline=""); writer=csv.writer(fcsv)
        writer.writerow(["ts_iso","tag","median_m","mean_m","std_m","burst_valid","burst_samples_m","burst_pulses_s"])

    print(f"[Ultrasonic] backend={gpio.__class__.__name__} TRIG={args.trig} ECHO={args.echo} "
          f"rate={args.rate:.2f} Hz burst={args.burst} temp={args.temp:.1f}°C thresh={args.thresh:.2f} m\n")
    time.sleep(DEF_SETTLE)
    period=1.0/max(1e-3,args.rate)

    all_d=[]; n=0
    try:
        while True:
            t0=time.perf_counter()
            r=burst(gpio,args.trig,args.echo,args.temp,args.burst,DEF_TRIG_PULSE,args.timeout)
            ts=time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())
            if not r:
                if not args.quiet: print(f"{ts}  median: --- (timeouts)")
                if writer: writer.writerow([ts,args.tag or "","","","",0,"[]","[]"])
            else:
                dmed,wmed,ds,ws=r
                from statistics import mean,pstdev
                dmean=mean(ds); dstd=pstdev(ds) if len(ds)>1 else 0.0
                all_d.append(dmed)
                if not args.quiet:
                    print(f"{ts}  median={dmed*100:.1f} cm  (mean={dmean*100:.1f} σ={dstd*100:.1f})  burst={len(ds)}/{args.burst}")
                if writer:
                    writer.writerow([ts,args.tag or "",f"{dmed:.6f}",f"{dmean:.6f}",f"{dstd:.6f}",
                                     len(ds),"["+",".join(f"{x:.6f}" for x in ds)+"]",
                                     "["+",".join(f"{x:.6e}" for x in ws)+"]"])
                n+=1
            if args.samples>0 and n>=args.samples: break
            dt=time.perf_counter()-t0
            if (period-dt)>0: time.sleep(period-dt)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        g,msg=grade(all_d,args.thresh)
        if all_d:
            print(f"\nHealth: {g} | {msg}")
        else:
            print("\nHealth: FAIL | No valid readings collected.")
        try: gpio.write(args.trig,0)
        except Exception: pass
        gpio.close()
        if fcsv: fcsv.close()

if __name__=="__main__":
    main()
