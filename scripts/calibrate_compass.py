#!/usr/bin/env python3
"""
Compass Calibration Tool
========================

Computes hard-iron offsets, soft-iron scales, and heading offset
for your robot's magnetometer. Saves results to config.json.

Usage:
    1. Place robot in an open area away from metal/concrete
    2. Run:  python3 scripts/calibrate_compass.py
    3. Slowly rotate the robot 360° (at least 2 full turns in 60 seconds)
    4. When prompted, point the robot NORTH and press Enter
    5. Calibration is saved to raspberry_pi/config.json

What this fixes:
    - Hard-iron:  Offsets from motors, battery, wires shift the zero point
    - Soft-iron:  Metal chassis squashes the circle into an ellipse
    - Heading offset: Corrects if 0° doesn't align with magnetic North
"""

import sys
import os
import json
import math
import time

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def main():
    print("=" * 60)
    print("  COMPASS CALIBRATION TOOL")
    print("=" * 60)
    print()

    # Try to import compass
    try:
        from raspberry_pi.sensors.compass import Compass
    except Exception as e:
        print(f"ERROR: Cannot import Compass: {e}")
        print("Run this on the Raspberry Pi with the compass connected.")
        sys.exit(1)

    # Load existing config
    config_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                               'raspberry_pi', 'config.json')
    try:
        with open(config_path, 'r') as f:
            config = json.load(f)
    except Exception:
        config = {}

    # Initialize compass (raw, no calibration applied)
    try:
        compass = Compass(bus=1, declination_deg=0)
        print(f"  Compass detected: {compass.chip} at 0x{compass.address:02X}")
    except Exception as e:
        print(f"ERROR: Cannot initialize compass: {e}")
        sys.exit(1)

    # ----------------------------------------------------------------
    # Phase 1: Collect raw data while user rotates robot
    # ----------------------------------------------------------------
    print()
    print("STEP 1: ROTATE THE ROBOT")
    print("-" * 40)
    print("  Slowly rotate the robot 360° (2+ full turns).")
    print("  Keep it LEVEL. You have 60 seconds.")
    print()
    input("  Press ENTER to start collecting data...")
    print()
    print("  Collecting... rotate now!")

    min_x = float('inf')
    max_x = float('-inf')
    min_y = float('inf')
    max_y = float('-inf')
    samples = 0

    t0 = time.time()
    duration = 60  # seconds

    while time.time() - t0 < duration:
        try:
            x, y, _ = compass.read_raw()
            if x == 0 and y == 0:
                continue

            min_x = min(min_x, x)
            max_x = max(max_x, x)
            min_y = min(min_y, y)
            max_y = max(max_y, y)
            samples += 1

            elapsed = time.time() - t0
            remaining = duration - elapsed
            # Live feedback every ~0.5s
            if samples % 5 == 0:
                print(f"\r  Samples: {samples}  |  "
                      f"X: [{min_x:6.0f} .. {max_x:6.0f}]  |  "
                      f"Y: [{min_y:6.0f} .. {max_y:6.0f}]  |  "
                      f"Time left: {remaining:.0f}s   ", end="", flush=True)

        except Exception:
            pass
        time.sleep(0.1)

    print()
    print()

    if samples < 50:
        print(f"ERROR: Only {samples} samples collected. Need at least 50.")
        print("Make sure compass is connected and you're rotating the robot.")
        sys.exit(1)

    # ----------------------------------------------------------------
    # Compute hard-iron offsets (center of the ellipse)
    # ----------------------------------------------------------------
    offset_x = (max_x + min_x) / 2.0
    offset_y = (max_y + min_y) / 2.0

    # Compute soft-iron scales (normalize ellipse to circle)
    range_x = max_x - min_x
    range_y = max_y - min_y

    if range_x == 0 or range_y == 0:
        print("ERROR: No variation in readings. Check compass wiring.")
        sys.exit(1)

    avg_range = (range_x + range_y) / 2.0
    scale_x = avg_range / range_x
    scale_y = avg_range / range_y

    print(f"  Hard-iron offsets:  X = {offset_x:.1f},  Y = {offset_y:.1f}")
    print(f"  Soft-iron scales:  X = {scale_x:.4f},  Y = {scale_y:.4f}")
    print(f"  Raw ranges:        X = {range_x:.0f},  Y = {range_y:.0f}")
    print()

    # ----------------------------------------------------------------
    # Phase 2: Determine heading offset (point robot North)
    # ----------------------------------------------------------------
    print("STEP 2: POINT THE ROBOT NORTH")
    print("-" * 40)
    print("  Point the FRONT of the robot toward magnetic NORTH")
    print("  (use a phone compass or known landmark).")
    print()
    input("  Press ENTER when the robot is pointing NORTH...")
    print()

    # Take 20 readings and average the angle
    angles = []
    for _ in range(20):
        try:
            x, y, _ = compass.read_raw()
            # Apply hard-iron and soft-iron correction
            cx = (x - offset_x) * scale_x
            cy = (y - offset_y) * scale_y

            if cx == 0 and cy == 0:
                continue

            angle = math.atan2(cy, cx) * 180.0 / math.pi
            if angle < 0:
                angle += 360.0
            angles.append(angle)
        except Exception:
            pass
        time.sleep(0.05)

    if len(angles) < 5:
        print("ERROR: Could not get stable readings. Check compass.")
        sys.exit(1)

    # Circular mean (handles wraparound at 0/360)
    sin_sum = sum(math.sin(math.radians(a)) for a in angles)
    cos_sum = sum(math.cos(math.radians(a)) for a in angles)
    raw_north_angle = math.degrees(math.atan2(sin_sum, cos_sum))
    if raw_north_angle < 0:
        raw_north_angle += 360.0

    # heading_offset: what to subtract so that "pointing North" reads 0°
    heading_offset = raw_north_angle

    print(f"  Raw heading when pointing North: {raw_north_angle:.1f}°")
    print(f"  Heading offset to apply: {heading_offset:.1f}°")
    print()

    # ----------------------------------------------------------------
    # Verify with a quick test
    # ----------------------------------------------------------------
    print("VERIFICATION")
    print("-" * 40)
    print("  Keep the robot pointing NORTH. Reading calibrated heading...")
    print()

    for i in range(10):
        try:
            x, y, _ = compass.read_raw()
            cx = (x - offset_x) * scale_x
            cy = (y - offset_y) * scale_y
            raw_angle = math.atan2(cy, cx) * 180.0 / math.pi
            if raw_angle < 0:
                raw_angle += 360.0
            calibrated = (raw_angle - heading_offset) % 360.0
            print(f"    Reading {i+1}/10:  raw={raw_angle:.1f}°  →  calibrated={calibrated:.1f}°")
        except Exception:
            pass
        time.sleep(0.3)

    print()
    print("  If calibrated reads ~0° (or ~355-5°), calibration is GOOD.")
    print("  Now rotate 90° RIGHT — it should read ~90°.")
    print()

    proceed = input("  Save calibration to config.json? (y/n): ").strip().lower()
    if proceed != 'y':
        print("  Calibration NOT saved.")
        sys.exit(0)

    # ----------------------------------------------------------------
    # Save to config.json
    # ----------------------------------------------------------------
    if 'compass' not in config:
        config['compass'] = {}

    config['compass']['offset_x'] = round(offset_x, 2)
    config['compass']['offset_y'] = round(offset_y, 2)
    config['compass']['scale_x'] = round(scale_x, 6)
    config['compass']['scale_y'] = round(scale_y, 6)
    config['compass']['heading_offset'] = round(heading_offset, 2)
    config['compass']['calibrated'] = True

    try:
        with open(config_path, 'w') as f:
            json.dump(config, f, indent=4)
        print()
        print(f"  ✅ Calibration saved to {config_path}")
        print()
        print("  Restart main.py to apply the new calibration.")
    except Exception as e:
        print(f"  ERROR saving config: {e}")
        print()
        print("  Manually add to config.json → 'compass' section:")
        print(f'    "offset_x": {offset_x:.2f},')
        print(f'    "offset_y": {offset_y:.2f},')
        print(f'    "scale_x": {scale_x:.6f},')
        print(f'    "scale_y": {scale_y:.6f},')
        print(f'    "heading_offset": {heading_offset:.2f}')


if __name__ == '__main__':
    main()
