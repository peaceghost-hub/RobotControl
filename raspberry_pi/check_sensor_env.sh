#!/bin/bash
# Check Pi environment and sensor readiness

echo "=========================================="
echo "Raspberry Pi Sensor Environment Check"
echo "=========================================="
echo ""

echo "1. Python packages installed:"
echo "-----------------------------"
python3 -c "import Adafruit_ADS1x15; print('✓ Adafruit_ADS1x15')" 2>/dev/null || echo "✗ Adafruit_ADS1x15 NOT installed"
python3 -c "import Adafruit_DHT; print('✓ Adafruit_DHT')" 2>/dev/null || echo "✗ Adafruit_DHT NOT installed"
python3 -c "import smbus2; print('✓ smbus2')" 2>/dev/null || echo "✓ smbus2"

echo ""
echo "2. Quick sensor test:"
echo "--------------------"
python3 << 'PYEOF'
import sys
sys.path.insert(0, '/home/raspberry/RobotControl/raspberry_pi')

try:
    from sensors.mq_sensors import MQSensors
    
    config = {
        "mq2": {"channel": 0, "enabled": True, "calibration": 1.0},
        "mq7": {"channel": 2, "enabled": True, "calibration": 1.0},
        "mq135": {"channel": 1, "enabled": True, "calibration": 1.0}
    }
    
    adc = {"type": "ADS1115", "address": 0x48, "gain": 1}
    
    mq = MQSensors(config, adc)
    print(f"  ADS1115 initialized: {mq.ads is not None}")
    
    if mq.ads:
        readings = mq.read_all()
        print(f"  MQ-2 (A0): {readings.get('mq2', 'N/A')}")
        print(f"  MQ-135 (A1): {readings.get('mq135', 'N/A')}")
        print(f"  MQ-7 (A2): {readings.get('mq7', 'N/A')}")
        
        # Check if values are changing
        import time
        time.sleep(0.5)
        readings2 = mq.read_all()
        
        if readings == readings2:
            print("\n  ⚠️  WARNING: Sensor values not changing!")
            print("     This could mean:")
            print("     - Sensors not connected/powered")
            print("     - Sensors need warm-up time (24-48 hours for MQ sensors)")
            print("     - ADC wiring issue")
        else:
            print("\n  ✓ Sensor values are changing (good!)")
    else:
        print("  ✗ ADS1115 not available - using mock values")
        
except Exception as e:
    print(f"  ✗ Error: {e}")
PYEOF

echo ""
echo "=========================================="
echo "If ADS1115 is NOT installed:"
echo "=========================================="
echo "Install with:"
echo "  pip3 install Adafruit-ADS1x15"
echo ""
echo "If DHT sensor library missing:"
echo "  pip3 install Adafruit-DHT"
