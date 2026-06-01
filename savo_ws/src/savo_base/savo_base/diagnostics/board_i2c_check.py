#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/diagnostics/board_i2c_check.py
-----------------------------------------------------
Professional I2C diagnostic for Robot Savo base/board bringup.

Purpose
-------
Quickly verify that expected I2C devices are visible on the selected bus(es)
before running `savo_base` motor-control nodes.

This script supports multiple probe methods (in order):
1) Python SMBus via `smbus2` (preferred)
2) Python SMBus via legacy `smbus`
3) `i2cdetect` command fallback (if installed)

Why this matters
----------------
Robot Savo uses multiple I2C devices across buses (Pi + Freenove + UPS HAT).
Before base bringup, this script helps confirm:
- PCA9685 motor PWM board is visible (0x40)
- ADS7830 ADC is visible (0x48)
- PCA9685 all-call responds (0x70) [optional/diagnostic]
- UPS HAT fuel gauge is visible (0x36) [on I2C-1]
- IMU / ToF devices are visible if connected on same bus (0x28, 0x29)

Examples
--------
# Default Robot Savo check (bus 1 primary, bus 0 optional if present)
python3 board_i2c_check.py

# Check only bus 1 with verbose output
python3 board_i2c_check.py --bus 1 --verbose

# Custom expected addresses
python3 board_i2c_check.py --bus 1 --expect 0x40 0x48 0x70

# Strict mode (exit non-zero if required devices are missing)
python3 board_i2c_check.py --strict

# Include bus 0 left ToF check (VL53L1X-FL on I2C-0 in Robot Savo)
python3 board_i2c_check.py --include-bus0

Notes
-----
- Accessing I2C may require permissions (run as root or ensure i2c group access).
- Addresses like 0x70 (PCA9685 all-call) may or may not ACK depending on device state.
- This is a diagnostic tool; it does not configure devices.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple


# =============================================================================
# Robot Savo expected devices (informational + defaults)
# =============================================================================
# Locked Robot Savo I2C map context (relevant subset for diagnostics):
# I2C-1: 0x28 BNO055, 0x29 VL53L1X-FR, 0x36 UPS HAT fuel gauge, 0x40 PCA9685,
#        0x48 ADS7830, 0x70 PCA9685 all-call
# I2C-0: 0x29 VL53L1X-FL (left ToF)
ROBOT_SAVO_BUS1_DEFAULTS: Dict[int, str] = {
    0x28: "BNO055 IMU",
    0x29: "VL53L1X ToF (FR/right)",
    0x36: "DFRobot UPS HAT fuel gauge",
    0x40: "PCA9685 motor PWM (Freenove board)",
    0x48: "ADS7830 ADC (Freenove board)",
    0x70: "PCA9685 all-call (diagnostic/optional)",
}

ROBOT_SAVO_BUS0_DEFAULTS: Dict[int, str] = {
    0x29: "VL53L1X ToF (FL/left on I2C-0)",
}

# For strict checks, usually these are the base-board critical ones:
ROBOT_SAVO_BASE_REQUIRED_BUS1: Tuple[int, ...] = (0x40, 0x48)
ROBOT_SAVO_BASE_OPTIONAL_BUS1: Tuple[int, ...] = (0x70, 0x36, 0x28, 0x29)


# =============================================================================
# Data models
# =============================================================================
@dataclass(frozen=True)
class ProbeHit:
    address: int
    method: str  # "smbus2", "smbus", "i2cdetect"

@dataclass
class BusProbeResult:
    bus_num: int
    hits: Dict[int, ProbeHit] = field(default_factory=dict)
    errors: List[str] = field(default_factory=list)
    methods_attempted: List[str] = field(default_factory=list)

    @property
    def found_addrs(self) -> Set[int]:
        return set(self.hits.keys())

    def add_hit(self, addr: int, method: str) -> None:
        self.hits[int(addr)] = ProbeHit(address=int(addr), method=str(method))

    def add_error(self, msg: str) -> None:
        self.errors.append(str(msg))


# =============================================================================
# Helpers
# =============================================================================
def _parse_addr(text: str) -> int:
    try:
        v = int(str(text), 0)
    except Exception as e:
        raise argparse.ArgumentTypeError(f"Invalid address '{text}': {e}") from e
    if v < 0x03 or v > 0x77:
        raise argparse.ArgumentTypeError(f"Address out of 7-bit I2C range (0x03..0x77): {text}")
    return v


def _fmt_addr(addr: int) -> str:
    return f"0x{int(addr):02X}"


def _sorted_addrs(addrs: Iterable[int]) -> List[int]:
    return sorted({int(a) for a in addrs})


def _addr_name(addr: int, catalog: Dict[int, str]) -> str:
    return catalog.get(int(addr), "Unknown device")


# =============================================================================
# SMBus probe backend
# =============================================================================
class SMBusProber:
    """
    Probe I2C addresses using Python SMBus access.

    Tries:
    - smbus2.SMBus
    - smbus.SMBus
    """
    def __init__(self, bus_num: int, verbose: bool = False) -> None:
        self.bus_num = int(bus_num)
        self.verbose = bool(verbose)
        self._bus = None
        self._method: Optional[str] = None

    @property
    def method(self) -> str:
        return self._method or "unknown"

    def open(self) -> None:
        last_err = None

        # Try smbus2 first
        try:
            from smbus2 import SMBus  # type: ignore
            self._bus = SMBus(self.bus_num)
            self._method = "smbus2"
            return
        except Exception as e:
            last_err = e

        # Fallback to legacy smbus
        try:
            import smbus  # type: ignore
            self._bus = smbus.SMBus(self.bus_num)
            self._method = "smbus"
            return
        except Exception as e:
            last_err = e

        raise RuntimeError(f"SMBus import/open failed (tried smbus2, smbus): {last_err}")

    def close(self) -> None:
        try:
            if self._bus is not None and hasattr(self._bus, "close"):
                self._bus.close()
        except Exception:
            pass
        finally:
            self._bus = None

    def probe_address(self, addr: int) -> bool:
        """
        Best-effort ACK probe for a 7-bit I2C address.

        Strategy:
        - Try write_quick (if available)
        - Fall back to read_byte (some devices NACK reads but many will ACK)
        - Treat any non-address-related exception as not-found (diagnostic mode)
        """
        if self._bus is None:
            raise RuntimeError("SMBus not opened")

        a = int(addr)

        # Method 1: quick write (best pure ACK check, not always supported)
        if hasattr(self._bus, "write_quick"):
            try:
                self._bus.write_quick(a)
                return True
            except Exception:
                pass

        # Method 2: read byte
        try:
            # Some devices may return a byte or throw OSError on NACK
            _ = self._bus.read_byte(a)
            return True
        except Exception:
            return False

    def probe_many(self, addrs: Sequence[int]) -> Dict[int, ProbeHit]:
        out: Dict[int, ProbeHit] = {}
        for a in _sorted_addrs(addrs):
            ok = self.probe_address(a)
            if ok:
                out[a] = ProbeHit(address=a, method=self.method)
        return out


# =============================================================================
# i2cdetect fallback backend
# =============================================================================
def _parse_i2cdetect_output(text: str) -> Set[int]:
    """
    Parse `i2cdetect -y <bus>` grid output and return detected addresses.

    Typical cells:
    - "--" means empty
    - "40", "48", etc. means detected
    - "UU" means in use by kernel driver (still present)
    """
    found: Set[int] = set()
    lines = text.splitlines()

    for line in lines:
        line = line.rstrip()
        if not line:
            continue
        # Data rows usually start with "00:", "10:", ... "70:"
        if len(line) < 3 or line[2] != ":":
            continue
        row_prefix = line[:2]
        try:
            row_base = int(row_prefix, 16)
        except Exception:
            continue

        cells = line[3:].split()
        # i2cdetect prints up to 16 columns
        for col, cell in enumerate(cells):
            token = cell.strip()
            if token == "--":
                continue
            if token.upper() == "UU":
                found.add(row_base + col)
                continue
            # Hex token like "40"
            try:
                val = int(token, 16)
            except Exception:
                continue
            if 0x03 <= val <= 0x77:
                found.add(val)

    return found


def probe_with_i2cdetect(bus_num: int, verbose: bool = False) -> Tuple[Set[int], Optional[str]]:
    exe = shutil.which("i2cdetect")
    if exe is None:
        return set(), "i2cdetect not found in PATH"

    cmd = [exe, "-y", str(int(bus_num))]
    try:
        cp = subprocess.run(
            cmd,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except Exception as e:
        return set(), f"Failed to run i2cdetect: {e}"

    if cp.returncode != 0:
        err = cp.stderr.strip() or cp.stdout.strip() or f"i2cdetect exit code {cp.returncode}"
        return set(), f"i2cdetect failed: {err}"

    if verbose and cp.stderr.strip():
        print(f"[debug] i2cdetect stderr (bus {bus_num}): {cp.stderr.strip()}", file=sys.stderr)

    found = _parse_i2cdetect_output(cp.stdout)
    return found, None


# =============================================================================
# Probe orchestration
# =============================================================================
def probe_bus(bus_num: int, targets: Sequence[int], verbose: bool = False) -> BusProbeResult:
    result = BusProbeResult(bus_num=int(bus_num))
    target_list = _sorted_addrs(targets)

    # Try Python SMBus first
    prober = SMBusProber(bus_num=bus_num, verbose=verbose)
    try:
        result.methods_attempted.append("smbus2/smbus")
        prober.open()
        smb_hits = prober.probe_many(target_list)
        for a, hit in smb_hits.items():
            result.hits[a] = hit

        # If all targets found, great
        missing_after_smbus = set(target_list) - set(smb_hits.keys())
        if not missing_after_smbus:
            return result
    except Exception as e:
        result.add_error(f"SMBus probe unavailable on bus {bus_num}: {e}")
    finally:
        prober.close()

    # Fallback: i2cdetect
    result.methods_attempted.append("i2cdetect")
    found_all, err = probe_with_i2cdetect(bus_num=bus_num, verbose=verbose)
    if err:
        result.add_error(err)
        return result

    for a in target_list:
        if a in found_all and a not in result.hits:
            result.add_hit(a, "i2cdetect")

    return result


# =============================================================================
# Reporting
# =============================================================================
def print_bus_report(
    result: BusProbeResult,
    expected: Dict[int, str],
    required: Sequence[int],
    optional: Sequence[int],
    verbose: bool = False,
) -> None:
    required_set = set(required)
    optional_set = set(optional)
    target_set = set(expected.keys())

    found = result.found_addrs
    missing = target_set - found

    print(f"\n=== I2C Bus {result.bus_num} Report ===")
    print(f"Methods attempted: {', '.join(result.methods_attempted) if result.methods_attempted else 'none'}")

    if result.errors:
        for e in result.errors:
            print(f"[warn] {e}")

    print("\nDetected expected devices:")
    any_found = False
    for addr in _sorted_addrs(target_set):
        if addr not in found:
            continue
        any_found = True
        hit = result.hits.get(addr)
        method = hit.method if hit else "unknown"
        tag = "REQUIRED" if addr in required_set else ("OPTIONAL" if addr in optional_set else "INFO")
        print(f"  [OK] { _fmt_addr(addr) }  {expected.get(addr, 'Unknown')}  ({tag}, via {method})")
    if not any_found:
        print("  (none of the expected addresses were detected)")

    print("\nMissing expected devices:")
    if missing:
        for addr in _sorted_addrs(missing):
            tag = "REQUIRED" if addr in required_set else ("OPTIONAL" if addr in optional_set else "INFO")
            print(f"  [MISS] { _fmt_addr(addr) }  {expected.get(addr, 'Unknown')}  ({tag})")
    else:
        print("  none")

    req_missing = required_set - found
    print("\nSummary:")
    print(f"  Found:    {len(found & target_set)}/{len(target_set)} expected")
    print(f"  Required: {len(required_set - req_missing)}/{len(required_set)}")
    print(f"  Optional: {len(optional_set & found)}/{len(optional_set)}")

    if req_missing:
        print("  Status:   FAIL (required devices missing)")
    else:
        print("  Status:   PASS (all required devices detected)")

    if verbose:
        print("\nExpected address table:")
        for addr in _sorted_addrs(target_set):
            kind = "REQUIRED" if addr in required_set else ("OPTIONAL" if addr in optional_set else "INFO")
            print(f"  - {_fmt_addr(addr)} : {expected[addr]} [{kind}]")


# =============================================================================
# CLI
# =============================================================================
def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Robot Savo I2C board/device check (SMBus + i2cdetect fallback)"
    )

    # Bus selection
    p.add_argument(
        "--bus",
        type=int,
        default=1,
        help="Primary I2C bus to check (default: 1)",
    )
    p.add_argument(
        "--include-bus0",
        action="store_true",
        help="Also check I2C bus 0 using Robot Savo FL ToF defaults (0x29)",
    )

    # Expected addresses
    p.add_argument(
        "--expect",
        nargs="*",
        type=_parse_addr,
        default=None,
        help="Custom expected addresses for primary bus (e.g. --expect 0x40 0x48 0x70)",
    )
    p.add_argument(
        "--required",
        nargs="*",
        type=_parse_addr,
        default=None,
        help="Custom required addresses for primary bus (subset of --expect recommended)",
    )
    p.add_argument(
        "--skip-optional",
        action="store_true",
        help="Primary bus: only check required/default-required addresses",
    )

    # Behavior
    p.add_argument(
        "--strict",
        action="store_true",
        help="Exit non-zero if required devices are missing on any checked bus",
    )
    p.add_argument(
        "--verbose",
        action="store_true",
        help="Verbose diagnostic output",
    )

    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_argparser().parse_args(argv)

    primary_bus = int(args.bus)
    include_bus0 = bool(args.include_bus0)
    verbose = bool(args.verbose)

    # Build primary expected catalog
    if args.expect is None:
        # Robot Savo professional defaults for bus 1 or generic fallback
        if primary_bus == 1:
            expected_primary = dict(ROBOT_SAVO_BUS1_DEFAULTS)
            default_required = set(ROBOT_SAVO_BASE_REQUIRED_BUS1)
            default_optional = set(ROBOT_SAVO_BASE_OPTIONAL_BUS1)
        elif primary_bus == 0:
            expected_primary = dict(ROBOT_SAVO_BUS0_DEFAULTS)
            default_required = set()  # bus0 is optional unless user marks required
            default_optional = set(expected_primary.keys())
        else:
            expected_primary = {}
            default_required = set()
            default_optional = set()
    else:
        expected_primary = {a: "Custom expected device" for a in _sorted_addrs(args.expect)}
        default_required = set(_sorted_addrs(args.required or []))
        default_optional = set(expected_primary.keys()) - default_required

    # If using defaults and user supplied --required only, honor it
    if args.expect is None and args.required is not None:
        custom_req = set(_sorted_addrs(args.required))
        # Keep only those that are in expected set if possible
        default_required = custom_req & set(expected_primary.keys()) if expected_primary else custom_req
        default_optional = set(expected_primary.keys()) - default_required

    # Skip optional mode
    if args.skip_optional:
        expected_primary = {a: expected_primary[a] for a in expected_primary if a in default_required}
        default_optional = set()

    # If primary expected empty, provide a practical generic base default
    if not expected_primary:
        expected_primary = {
            0x40: "PCA9685 motor PWM",
            0x48: "ADS7830 ADC",
        }
        if args.required is None:
            default_required = {0x40, 0x48}
            default_optional = set()
        else:
            default_required = set(_sorted_addrs(args.required))
            default_optional = set(expected_primary.keys()) - default_required

    print("Robot SAVO — I2C Board Check")
    print("----------------------------")
    print(f"Primary bus: {primary_bus}")
    print(f"SMBus probe: smbus2 -> smbus -> i2cdetect fallback")
    if include_bus0:
        print("Bus 0 check: enabled (Robot Savo FL ToF default)")

    # Probe primary bus
    result_primary = probe_bus(primary_bus, list(expected_primary.keys()), verbose=verbose)
    print_bus_report(
        result=result_primary,
        expected=expected_primary,
        required=_sorted_addrs(default_required),
        optional=_sorted_addrs(default_optional),
        verbose=verbose,
    )

    overall_fail = False
    if set(default_required) - result_primary.found_addrs:
        overall_fail = True

    # Optional bus0 probe (Robot Savo FL ToF)
    if include_bus0:
        expected_bus0 = dict(ROBOT_SAVO_BUS0_DEFAULTS)
        required_bus0: Set[int] = set()  # informative by default
        optional_bus0: Set[int] = set(expected_bus0.keys())

        result_bus0 = probe_bus(0, list(expected_bus0.keys()), verbose=verbose)
        print_bus_report(
            result=result_bus0,
            expected=expected_bus0,
            required=_sorted_addrs(required_bus0),
            optional=_sorted_addrs(optional_bus0),
            verbose=verbose,
        )
        # not failing strict by default because bus0 check is optional/informational
        # If user wants bus0 strict, they can run --bus 0 --required 0x29 --strict.

    # Final exit status
    if args.strict and overall_fail:
        print("\n[RESULT] FAIL (strict mode): missing required I2C device(s) on primary bus.")
        return 2

    print("\n[RESULT] PASS" if not overall_fail else "\n[RESULT] WARN (required missing, non-strict mode)")
    return 0 if (not args.strict or not overall_fail) else 2


if __name__ == "__main__":
    raise SystemExit(main())