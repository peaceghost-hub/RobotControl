#!/usr/bin/env python3
"""QMC/HMC compass diagnostic helper.

Designed for cases where a device ACKs on I2C (e.g. 0x2C) but returns constant
or zero magnetometer data.

Usage examples:
  python3 -m raspberry_pi.sensors.compass_diag --addr 0x2c
  python3 -m raspberry_pi.sensors.compass_diag --scan

This does not require smbus2; it will use python-smbus if present.
"""

from __future__ import annotations

import argparse
import time

try:
    from smbus2 import SMBus
except Exception:
    from smbus import SMBus  # type: ignore


def parse_addr(s: str) -> int:
    s = s.strip().lower()
    return int(s, 16) if s.startswith("0x") else int(s)


def read_regs(bus: SMBus, addr: int, start: int, count: int) -> list[int]:
    vals: list[int] = []
    for off in range(count):
        reg = (start + off) & 0xFF
        try:
            vals.append(int(bus.read_byte_data(addr, reg)))
        except Exception:
            vals.append(-1)
    return vals


def fmt(vals: list[int]) -> str:
    out = []
    for v in vals:
        out.append("??" if v < 0 else f"{v:02X}")
    return " ".join(out)


def try_write(bus: SMBus, addr: int, reg: int, val: int) -> bool:
    try:
        bus.write_byte_data(addr, reg & 0xFF, val & 0xFF)
        return True
    except Exception:
        return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bus", type=int, default=1)
    ap.add_argument("--addr", type=parse_addr, default=None)
    ap.add_argument("--scan", action="store_true")
    args = ap.parse_args()

    bus = SMBus(args.bus)

    addrs: list[int]
    if args.scan:
        # Common compass/mag addresses + your observed 0x2C.
        candidates = [0x0C, 0x0D, 0x1C, 0x1E, 0x2C]
        addrs = []
        for a in candidates:
            try:
                bus.read_byte_data(a, 0x00)
                addrs.append(a)
            except Exception:
                pass
        if not addrs:
            print("No candidates responded on I2C")
            return 1
    else:
        if args.addr is None:
            ap.error("--addr is required unless --scan is used")
        addrs = [args.addr]

    for addr in addrs:
        print(f"\n=== Probe addr 0x{addr:02X} ===")

        regs_00_0d = read_regs(bus, addr, 0x00, 0x0E)
        print("regs 0x00..0x0D:")
        print(" ", fmt(regs_00_0d))

        # If it looks like QMC (control/status addresses exist), try a minimal QMC init.
        print("\nTry QMC init writes:")
        for (r, v, name) in [
            (0x09, 0x00, "CTRL standby"),
            (0x0B, 0x01, "SETRST"),
            (0x09, 0x1D, "CTRL continuous"),
        ]:
            ok = try_write(bus, addr, r, v)
            print(f"  write reg 0x{r:02X} = 0x{v:02X} ({name}): {'OK' if ok else 'FAIL'}")

        time.sleep(0.05)
        rb = read_regs(bus, addr, 0x06, 1)[0]
        ctrl = read_regs(bus, addr, 0x09, 1)[0]
        ctrl2 = read_regs(bus, addr, 0x0A, 1)[0]
        setrst = read_regs(bus, addr, 0x0B, 1)[0]
        print(f"\nReadback: STATUS(0x06)={rb if rb>=0 else '??'} CTRL(0x09)={ctrl if ctrl>=0 else '??'} CTRL2(0x0A)={ctrl2 if ctrl2>=0 else '??'} SETRST(0x0B)={setrst if setrst>=0 else '??'}")

        # Show a few samples of the first 6 registers as 'data'
        print("\nRead 6 bytes from 0x00 (as-if data):")
        for i in range(5):
            vals = read_regs(bus, addr, 0x00, 6)
            print(f"  {i}: {fmt(vals)}")
            time.sleep(0.05)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
