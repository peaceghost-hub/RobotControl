#!/usr/bin/env python3
"""
Test PPM Calibration with Current Sensor Values
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from raspberry_pi.utils.gas_calibration import GasCalibration

# Your current sensor readings
current_values = {
    'MQ-2': 2048,
    'MQ-135': 8672,
    'MQ-7': 6096
}

print("=" * 60)
print("Gas Sensor PPM Calibration Test")
print("=" * 60)
print()

# Calculate all concentrations
gas_data = GasCalibration.get_all_concentrations(
    current_values['MQ-2'],
    current_values['MQ-135'],
    current_values['MQ-7']
)

print("Raw ADC Values:")
print("-" * 60)
print(f"  MQ-2 (Smoke):    {current_values['MQ-2']:>6} ADC  →  {gas_data['mq2_volts']:.3f}V")
print(f"  MQ-135 (CO2):    {current_values['MQ-135']:>6} ADC  →  {gas_data['mq135_volts']:.3f}V")
print(f"  MQ-7 (CO):       {current_values['MQ-7']:>6} ADC  →  {gas_data['mq7_volts']:.3f}V")
print()

print("Calculated Gas Concentrations (PPM):")
print("-" * 60)
print(f"  🔥 MQ-2 Smoke:   {gas_data['mq2_smoke_ppm']:>10.2f} ppm")
print(f"  🔥 MQ-2 LPG:     {gas_data['mq2_lpg_ppm']:>10.2f} ppm")
print(f"  🌫️  MQ-135 CO2:  {gas_data['mq135_co2_ppm']:>10.2f} ppm")
print(f"  🌫️  MQ-135 NH3:  {gas_data['mq135_nh3_ppm']:>10.2f} ppm")
print(f"  ⚠️  MQ-7 CO:     {gas_data['mq7_co_ppm']:>10.2f} ppm")
print()

print("Sensor Status:")
print("-" * 60)
print(f"  MQ-2:   {GasCalibration.get_sensor_status(current_values['MQ-2'])}")
print(f"  MQ-135: {GasCalibration.get_sensor_status(current_values['MQ-135'])}")
print(f"  MQ-7:   {GasCalibration.get_sensor_status(current_values['MQ-7'])}")
print()

print("Reference Levels:")
print("-" * 60)
print("  CO2 (Indoor Air):")
print("    • 400-1000 ppm:  Good")
print("    • 1000-2000 ppm: Moderate")
print("    • 2000+ ppm:     Poor ventilation")
print()
print("  CO (Carbon Monoxide):")
print("    • 0-9 ppm:       Safe")
print("    • 9-50 ppm:      Moderate (max 8hr exposure)")
print("    • 50+ ppm:       Dangerous")
print()
print("  Smoke/LPG:")
print("    • <300 ppm:      Clean air")
print("    • 300-1000 ppm:  Slight detection")
print("    • 1000+ ppm:     Significant presence")
print()

print("=" * 60)
print("✓ Dashboard now shows:")
print("  - Raw ADC values (for ThingSpeak cloud upload)")
print("  - Calculated PPM concentrations (for visualization)")
print("  - Both values preserved in data stream")
print("=" * 60)
