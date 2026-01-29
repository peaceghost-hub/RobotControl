#!/usr/bin/env python3
"""
Debug script to test MQ sensor readings directly
Run this on the Pi to verify sensors are being read correctly
"""

import sys
import time
sys.path.insert(0, '/home/raspberry/RobotControl/raspberry_pi')

from sensors.mq_sensors import MQSensors

# Match your config.json
sensor_config = {
    "mq2": {
        "channel": 0,
        "enabled": True,
        "calibration": 1.0
    },
    "mq135": {
        "channel": 1,
        "enabled": True,
        "calibration": 1.0
    },
    "mq7": {
        "channel": 2,
        "enabled": True,
        "calibration": 1.0
    }
}

adc_config = {
    "type": "ADS1115",
    "address": 0x48,
    "gain": 1
}

print("=" * 60)
print("MQ Sensors Debug Test")
print("=" * 60)
print("\nInitializing MQ sensors...")

try:
    mq = MQSensors(sensor_config, adc_config)
    print("✓ MQ sensors initialized")
    print(f"  ADC available: {mq.ads is not None}")
    print(f"  Sensors configured: {list(mq.sensors.keys())}")
    
    print("\nReading sensors 10 times (1 second intervals)...")
    print("\nTime     | MQ-2   | MQ-135 | MQ-7   | Raw A0 | Raw A1 | Raw A2")
    print("-" * 70)
    
    for i in range(10):
        # Read all sensors
        readings = mq.read_all()
        
        # Also read raw ADC values
        raw_a0 = mq._read_adc(0) if mq.ads else 0
        raw_a1 = mq._read_adc(1) if mq.ads else 0
        raw_a2 = mq._read_adc(2) if mq.ads else 0
        
        timestamp = time.strftime("%H:%M:%S")
        print(f"{timestamp} | {readings.get('mq2', 0):6.1f} | {readings.get('mq135', 0):6.1f} | {readings.get('mq7', 0):6.1f} | {raw_a0:6d} | {raw_a1:6d} | {raw_a2:6d}")
        
        time.sleep(1)
    
    print("\n" + "=" * 60)
    print("Analysis:")
    print("=" * 60)
    
    final_reading = mq.read_all()
    
    for sensor_name in ['mq2', 'mq135', 'mq7']:
        value = final_reading.get(sensor_name, 0)
        channel = mq.sensors[sensor_name]['channel']
        raw = mq._read_adc(channel)
        
        print(f"\n{sensor_name.upper()}:")
        print(f"  Channel: A{channel}")
        print(f"  Raw ADC: {raw}")
        print(f"  Calibrated: {value}")
        
        if value == 0 or value is None:
            print(f"  ⚠️  WARNING: Sensor returning 0 or None!")
            print(f"     Check:")
            print(f"     - Is sensor powered? (needs 5V)")
            print(f"     - Is AOUT connected to ADS1115 A{channel}?")
            print(f"     - Is sensor heating up? (MQ sensors need 24-48hr preheat)")
        elif raw < 100:
            print(f"  ⚠️  WARNING: Very low reading (raw < 100)")
            print(f"     Sensor may not be warmed up or not connected")
        else:
            print(f"  ✓ Sensor appears to be reading correctly")

except Exception as e:
    print(f"\n✗ Error: {e}")
    import traceback
    traceback.print_exc()
