#!/usr/bin/env python3
"""
Compass Diagnostic & Correction Table Generator
=================================================

Interactive tool to calibrate the compass correction table.
The user points the robot at known physical directions (using a
reference compass or landmarks), records the magnetic reading,
and this tool generates a correction mapping file.

Output:  raspberry_pi/calibration/compass_correction.json

The correction file is auto-loaded by compass.py at startup
and used by the dashboard to show a True North compass alongside
the raw Magnetic compass.

Usage (run on the Raspberry Pi):
    python3 -m raspberry_pi.tools.compass_diagnostic
    -- or --
    cd raspberry_pi && python3 tools/compass_diagnostic.py

Supports 4-point (N/E/S/W) or 8-point (N/NE/E/SE/S/SW/W/NW)
calibration.  If errors are nearly constant across all points,
a simple "constant" offset is used.  Otherwise a "lookup_table"
with linear interpolation is generated.

GOLDEN RULE:  This tool is run offline — it does NOT affect
the running robot.  It only writes a JSON file.
"""

import json
import math
import os
import sys
import time
import statistics
import logging

# Allow running from project root or from raspberry_pi/
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PI_DIR = os.path.dirname(_SCRIPT_DIR)
_PROJECT_ROOT = os.path.dirname(_PI_DIR)
sys.path.insert(0, _PROJECT_ROOT)
sys.path.insert(0, _PI_DIR)

logger = logging.getLogger("compass_diagnostic")
logging.basicConfig(level=logging.INFO, format="%(message)s")

# Output paths
CALIBRATION_DIR = os.path.join(_PI_DIR, "calibration")
CORRECTION_FILE = os.path.join(CALIBRATION_DIR, "compass_correction.json")

# Known direction sets
FOUR_POINT = [
    ("North", 0),
    ("East", 90),
    ("South", 180),
    ("West", 270),
]

EIGHT_POINT = [
    ("North", 0),
    ("North-East", 45),
    ("East", 90),
    ("South-East", 135),
    ("South", 180),
    ("South-West", 225),
    ("West", 270),
    ("North-West", 315),
]


def _load_config():
    """Load Pi config.json for compass calibration values."""
    config_path = os.path.join(_PI_DIR, "config.json")
    if os.path.exists(config_path):
        try:
            with open(config_path, "r") as f:
                return json.load(f)
        except Exception as e:
            logger.warning("Could not load config.json: %s", e)
    return {}


def _create_compass(config):
    """Create a Compass instance using the project driver."""
    try:
        from sensors.compass import Compass
        return Compass(config=config)
    except Exception as e:
        logger.error("Failed to initialise compass: %s", e)
        logger.error("Make sure you're running on the Raspberry Pi with I2C enabled.")
        sys.exit(1)


def _read_averaged_heading(compass, samples=10, delay=0.05):
    """Read multiple pre-correction magnetic headings and return circular mean."""
    sin_sum = 0.0
    cos_sum = 0.0
    for _ in range(samples):
        h = compass.read_heading_magnetic()
        rad = math.radians(h)
        sin_sum += math.sin(rad)
        cos_sum += math.cos(rad)
        time.sleep(delay)
    mean_rad = math.atan2(sin_sum / samples, cos_sum / samples)
    return mean_rad * 180.0 / math.pi % 360.0


def _normalize_error(error):
    """Normalize angle difference to -180..+180."""
    while error > 180:
        error -= 360
    while error < -180:
        error += 360
    return error


def run_calibration():
    """Interactive calibration session."""
    print("=" * 60)
    print("  COMPASS CORRECTION TABLE GENERATOR")
    print("=" * 60)
    print()
    print("This tool measures the error between the compass's magnetic")
    print("reading and the ACTUAL physical direction the robot faces.")
    print("Run it AFTER your normal compass calibration so the table")
    print("captures the remaining directional error only.")
    print()
    print("You will point the robot at known directions (use a phone")
    print("compass, landmarks, or a protractor) and press ENTER to")
    print("record each reading.")
    print()

    # Choose calibration density
    while True:
        choice = input("Calibration points?  [4] N/E/S/W  or  [8] + diagonals: ").strip()
        if choice in ("4", ""):
            directions = FOUR_POINT
            break
        elif choice == "8":
            directions = EIGHT_POINT
            break
        print("Please enter 4 or 8.")

    # Load compass
    config = _load_config()
    compass = _create_compass(config)
    print(f"\nCompass found: {compass.chip} at 0x{compass.address:02X}")
    print(f"Calibration: {'YES' if compass._calibrated else 'NO (using defaults)'}")
    correction_type = compass.get_correction_data().get("type", "none")
    print(f"Correction table: {correction_type.upper()} (ignored while measuring magnetic headings)")
    print()

    # Collect readings
    readings = []
    for name, true_bearing in directions:
        print(f"--- Point the robot EXACTLY towards {name} ({true_bearing}°) ---")
        input(f"    Press ENTER when ready... ")

        # Take averaged reading
        magnetic = _read_averaged_heading(compass, samples=20, delay=0.03)
        error = _normalize_error(magnetic - true_bearing)

        readings.append({
            "true_bearing": true_bearing,
            "magnetic_reading": round(magnetic, 1),
            "error": round(error, 1),
        })
        print(f"    Magnetic: {magnetic:.1f}°  |  Error: {error:+.1f}°")
        print()

    # Analyse results
    errors = [r["error"] for r in readings]
    mean_error = statistics.mean(errors)
    stdev_error = statistics.stdev(errors) if len(errors) > 1 else 0.0

    print("=" * 60)
    print("  RESULTS")
    print("=" * 60)
    print(f"  {'Direction':<14} {'True':>6} {'Magnetic':>10} {'Error':>8}")
    print(f"  {'-'*14} {'-'*6} {'-'*10} {'-'*8}")
    for r in readings:
        name = [d[0] for d in directions if d[1] == r["true_bearing"]][0]
        print(f"  {name:<14} {r['true_bearing']:>5.0f}° {r['magnetic_reading']:>9.1f}° {r['error']:>+7.1f}°")
    print()
    print(f"  Mean error:  {mean_error:+.1f}°")
    print(f"  Std dev:     {stdev_error:.1f}°")
    print()

    # Decide correction type
    # If stdev < 3°, a constant offset is sufficient
    # Otherwise use a lookup table for direction-dependent correction
    CONSTANT_THRESHOLD = 3.0

    if stdev_error < CONSTANT_THRESHOLD:
        correction_type = "constant"
        offset = round(mean_error, 1)
        print(f"  → Error is nearly uniform.  Using CONSTANT offset: {offset:+.1f}°")
        correction_data = {
            "type": "constant",
            "offset": offset,
            "description": f"Subtract {offset}° from magnetic heading to get true heading",
            "calibration_points": readings,
            "mean_error": round(mean_error, 1),
            "stdev_error": round(stdev_error, 1),
            "generated": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
    else:
        correction_type = "lookup_table"
        print(f"  → Error varies by direction.  Using LOOKUP TABLE with interpolation.")
        # Build table: magnetic_bearing → correction (amount to subtract)
        table = []
        for r in readings:
            table.append({
                "magnetic": r["magnetic_reading"],
                "true": r["true_bearing"],
                "correction": round(r["error"], 1),
            })
        # Sort by magnetic bearing for interpolation
        table.sort(key=lambda e: e["magnetic"])

        correction_data = {
            "type": "lookup_table",
            "table": table,
            "description": "For a given magnetic heading, interpolate between nearest table entries to find correction (subtract from magnetic to get true)",
            "calibration_points": readings,
            "mean_error": round(mean_error, 1),
            "stdev_error": round(stdev_error, 1),
            "generated": time.strftime("%Y-%m-%d %H:%M:%S"),
        }

    # Verification pass
    print()
    print("  VERIFICATION — re-checking each direction:")
    print(f"  {'Direction':<14} {'Magnetic':>10} {'Corrected':>10} {'True':>6} {'Residual':>9}")
    print(f"  {'-'*14} {'-'*10} {'-'*10} {'-'*6} {'-'*9}")

    for name, true_bearing in directions:
        magnetic = _read_averaged_heading(compass, samples=10, delay=0.03)
        if correction_type == "constant":
            corrected = (magnetic - correction_data["offset"]) % 360.0
        else:
            corrected = _apply_lookup_correction(magnetic, correction_data["table"])
        residual = _normalize_error(corrected - true_bearing)
        print(f"  {name:<14} {magnetic:>9.1f}° {corrected:>9.1f}° {true_bearing:>5.0f}° {residual:>+8.1f}°")

    # Save
    print()
    save = input("Save correction file? [Y/n]: ").strip().lower()
    if save in ("", "y", "yes"):
        os.makedirs(CALIBRATION_DIR, exist_ok=True)
        with open(CORRECTION_FILE, "w") as f:
            json.dump(correction_data, f, indent=2)
        print(f"\n  ✓ Saved to: {CORRECTION_FILE}")
        print("  The correction will be loaded automatically on next robot start.")
    else:
        print("\n  Correction NOT saved.")

    print()
    print("Done!")


def _apply_lookup_correction(magnetic_heading, table):
    """Apply lookup-table correction with linear interpolation.

    Table entries: [{magnetic, true, correction}, ...]
    sorted by magnetic ascending.  Wraps around 0°/360°.
    """
    if not table:
        return magnetic_heading

    n = len(table)
    mag = magnetic_heading % 360.0

    # Find bracketing entries
    lower_idx = None
    upper_idx = None
    for i in range(n):
        if table[i]["magnetic"] <= mag:
            lower_idx = i
        if table[i]["magnetic"] >= mag and upper_idx is None:
            upper_idx = i

    # Wrap-around cases
    if lower_idx is None:
        lower_idx = n - 1  # wrap from end
    if upper_idx is None:
        upper_idx = 0  # wrap to start

    if lower_idx == upper_idx:
        correction = table[lower_idx]["correction"]
    else:
        m1 = table[lower_idx]["magnetic"]
        m2 = table[upper_idx]["magnetic"]
        c1 = table[lower_idx]["correction"]
        c2 = table[upper_idx]["correction"]

        # Handle wrap-around
        if m2 < m1:
            m2 += 360.0
        mag_adj = mag if mag >= m1 else mag + 360.0

        span = m2 - m1
        if span == 0:
            correction = c1
        else:
            t = (mag_adj - m1) / span
            # Interpolate corrections (handling sign wrap)
            diff = _normalize_error(c2 - c1)
            correction = c1 + t * diff

    corrected = (magnetic_heading - correction) % 360.0
    return corrected


if __name__ == "__main__":
    run_calibration()
